/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * nicogpu_vm_core.h -- pure-C core of the nicogpu memory manager.
 *
 * Everything with semantics lives here so that it compiles both inside the
 * kernel module and as plain host C: the BO argument
 * rules, a bitmap page pool (VRAM heap, page-table pool), the page-table
 * writer, the VA mapping table and the VM_BIND batch transaction.  The kernel
 * glue (nicogpu_mm.c) only supplies memory, MMIO and BO reference callbacks.
 *
 * Semantics are a C port of the functional model, which is the reference:
 *   - page-table writer      (PageTableBuilder of the model):
 *     page size choice 2M > 64K > 4K, split/demote/collapse, retired pages,
 *     begin/commit/rollback journal -- same algorithm, same table contents,
 *     same allocation order;
 *   - VM_BIND batch          (the model's sim backend, vm_bind()):
 *     validate everything first, apply in array order with replace semantics,
 *     whole-call atomic on any failure including OOM,
 *     invalidate the VMID on both paths, then
 *     reclaim unlinked table pages.
 * Entry encoding comes from the ptegen output gen/nicogpu_pte.h.
 *
 * Differences from the model (all stricter, none observable on success):
 *   - the journal and every host allocation can fail and are rolled back the
 *     same way (the model only catches std::bad_alloc around the batch);
 *   - per-call op limit and per-VM mapping limit.
 *
 * Page tables are kept as host "shadow" copies; every entry write is mirrored
 * to device memory with one 64-bit store (ops->pt_write) and a new table is
 * pushed in full (ops->pt_fill) before the PDE that links it is written.
 * Reads never touch the device.
 *
 * Not thread safe: the caller serialises all calls on one VM (and on one
 * pool).
 */
#ifndef NICOGPU_VM_CORE_H
#define NICOGPU_VM_CORE_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#else
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#endif
#include "nicogpu_errno.h"
#include "nicogpu_pte.h"	/* generated, vendored in gen/ */

#ifdef __cplusplus
extern "C" {
#endif

/* Linux errno values, returned negated (aliases of the shared table, nicogpu_errno.h). */
#define XVC_ENOENT	NGE_NOENT
#define XVC_ENOMEM	NGE_NOMEM
#define XVC_EINVAL	NGE_INVAL
#define XVC_ENOSPC	NGE_NOSPC

/* HAL v0 constants (frozen); nicogpu_uapi.h
 * exports the same values. */
#define XVC_OP_MAP		0u
#define XVC_OP_UNMAP		1u
#define XVC_OP_SPARSE_NULL	2u
#define XVC_VM_READ		(1u << 0)
#define XVC_VM_WRITE		(1u << 1)
#define XVC_VM_EXEC		(1u << 2)
#define XVC_VM_UNCACHED		(1u << 3)
#define XVC_VM_FLAGS_ALL	0xfu
#define XVC_DOMAIN_VRAM		1u
#define XVC_DOMAIN_GTT		2u
#define XVC_BO_HOST_VISIBLE	(1u << 0)

#define XVC_PAGE	UINT64_C(4096)
#define XVC_PAGE_64K	UINT64_C(0x10000)
#define XVC_PAGE_2M	UINT64_C(0x200000)
#define XVC_LEVELS	4u
#define XVC_ENTRIES	512u

/* Resource limits (PROVISIONAL). */
#define XVC_MAX_BIND_OPS	4096u		/* ops per VM_BIND call */
#define XVC_MAX_MAPPINGS	(1u << 20)	/* mapping records per VM */

/* Same layout as HAL struct nicogpu_vm_bind_op and uapi struct nicogpu_vm_bind_op. */
struct xvc_bind_op {
	uint32_t op;		/* XVC_OP_* */
	uint32_t flags;		/* XVC_VM_* (MAP only) */
	uint32_t bo_handle;	/* MAP only */
	uint32_t pad;		/* must be 0 */
	uint64_t bo_offset;	/* 4 KiB aligned */
	uint64_t va;		/* 4 KiB aligned */
	uint64_t range;		/* 4 KiB multiple, non-zero */
};

/* Resolved BO: device PA of byte 0 (contiguous) and size. */
struct xvc_bo_info {
	void *bo;
	uint64_t pa;
	uint64_t size;
};

/*
 * Callbacks.  `ctx` is the per-VM context given to xvc_vm_init().
 * Optional (may be NULL): pt_fill, pt_write, invalidate, bo_get, bo_put.
 */
struct xvc_ops {
	void *(*zalloc)(void *ctx, size_t size);	/* zeroed host memory or NULL */
	void (*free)(void *ctx, void *p);
	uint64_t (*pt_alloc)(void *ctx);		/* 4 KiB page in SYS, 0 = OOM */
	void (*pt_free)(void *ctx, uint64_t pa);
	/* mirror a new table (512 entries) to the device; called before linking */
	void (*pt_fill)(void *ctx, uint64_t pa, const uint64_t *ents);
	/* mirror one entry write: exactly one 64-bit store at `pa` */
	void (*pt_write)(void *ctx, uint64_t pa, uint64_t val);
	/* flush posted writes + invalidate the VMID + wait; false = timeout */
	bool (*invalidate)(void *ctx);
	int (*bo_lookup)(void *ctx, uint32_t handle, struct xvc_bo_info *out);
	void (*bo_get)(void *ctx, void *bo);
	void (*bo_put)(void *ctx, void *bo);
};

/* ---- BO arguments (HAL bo_create rules, same order as the sim backend) ---- */
int xvc_bo_args_check(uint64_t size, uint64_t alignment, uint32_t domain, uint32_t flags,
		      uint64_t *out_size, uint64_t *out_align);

/* ---- bitmap page pool: first fit, lowest address (== model RangeAllocator
 * for page-granular requests); free never fails ---- */
struct xvc_pool {
	uint64_t base;		/* byte address of page 0, 4 KiB aligned */
	uint64_t npages;
	uint64_t used;		/* pages */
	uint64_t *bits;		/* 1 = used; xvc_pool_words(npages) words */
};
size_t xvc_pool_words(uint64_t npages);
void xvc_pool_init(struct xvc_pool *p, uint64_t base, uint64_t npages, uint64_t *bits);
bool xvc_pool_alloc(struct xvc_pool *p, uint64_t size, uint64_t align, uint64_t *out);
void xvc_pool_free(struct xvc_pool *p, uint64_t addr, uint64_t size);

/* ---- entry decoding (rules D1..D15) ---- */
enum xvc_kind {			/* == NICOGPU_PTE_KIND_* of nicogpu_pte_vectors.h */
	XVC_NOT_PRESENT = 0,
	XVC_BAD_PTE = 1,
	XVC_SPARSE_NULL = 2,
	XVC_TABLE = 3,
	XVC_LEAF = 4,
};
struct xvc_decoded {
	enum xvc_kind kind;
	unsigned int rule;	/* 1..15 */
	uint64_t size;		/* leaf page size / sparse span */
	uint64_t base;		/* leaf PA / table PA */
};
struct xvc_decoded xvc_decode(uint64_t e, unsigned int level);

/* ---- one VM: page tables + mapping table ---- */
struct xvc_pt;			/* shadow page table, private */
struct xvc_jchunk;		/* journal chunk, private */

struct xvc_map {		/* one mapping record (model "Mapping") */
	uint64_t va, size;
	uint64_t bo_offset;
	void *bo;		/* NULL for sparse */
	uint64_t bo_pa;		/* PA of BO byte 0 */
	uint32_t flags;		/* XVC_VM_* */
	uint32_t kind;		/* XVC_OP_MAP or XVC_OP_SPARSE_NULL */
};

struct xvc_vm {
	const struct xvc_ops *ops;
	void *ctx;
	struct xvc_pt *root;
	bool large;			/* allow 64K/2M pages (default true) */
	size_t tables;			/* live tables incl. root */
	struct xvc_pt *retired;		/* unlinked, freed by reclaim */
	bool reclaim_blocked;		/* last invalidate timed out */
	/* transaction */
	bool txn;
	struct xvc_jchunk *journal;
	struct xvc_pt *txn_alloc;
	struct xvc_pt *txn_mark;	/* retired head at begin */
	/* mapping table, sorted by va, non-overlapping */
	struct xvc_map *maps;		/* room for maps_cap records */
	uint32_t nmaps;
	uint32_t maps_cap;
	uint32_t max_maps;		/* default XVC_MAX_MAPPINGS */
};

/* Allocates the (empty) root table.  -ENOMEM. */
int xvc_vm_init(struct xvc_vm *vm, const struct xvc_ops *ops, void *ctx);
/* Frees every table (live and retired) and drops every BO reference.  The
 * caller has disabled the context and invalidated the VMID first. */
void xvc_vm_fini(struct xvc_vm *vm);
uint64_t xvc_vm_root_pa(const struct xvc_vm *vm);

/* VM_BIND (HAL v0 section 3).  0, -EINVAL, -ENOENT (unknown BO handle),
 * -ENOMEM (any allocation failed: nothing changed), -ENOSPC (mapping limit:
 * nothing changed). */
int xvc_vm_bind(struct xvc_vm *vm, const struct xvc_bind_op *ops, uint32_t n);

/* Page-table writer (PageTableBuilder::set/begin/commit/rollback/reclaim).
 * vm_bind uses these; exported for builder-level tests. */
int xvc_pt_set(struct xvc_vm *vm, uint64_t va, uint64_t size, uint32_t kind, uint64_t pa,
	       uint64_t leaf_attrs);
void xvc_pt_begin(struct xvc_vm *vm);
void xvc_pt_commit(struct xvc_vm *vm);
void xvc_pt_rollback(struct xvc_vm *vm);
size_t xvc_pt_reclaim(struct xvc_vm *vm);
#ifndef __KERNEL__	/* host tests only: not part of the module */
size_t xvc_pt_retired(const struct xvc_vm *vm);
#endif
uint64_t xvc_leaf_attrs(uint32_t vm_flags);

/* Walk the shadow tables like the MMU walker. */
struct xvc_xlate {
	enum xvc_kind kind;	/* NOT_PRESENT, BAD_PTE, SPARSE_NULL or LEAF */
	unsigned int level;	/* level of the deciding entry */
	uint64_t entry;
	uint64_t pa;		/* LEAF: translated byte address */
	uint64_t page_size;	/* LEAF: page size; SPARSE_NULL: span */
};
void xvc_vm_lookup(const struct xvc_vm *vm, uint64_t va, struct xvc_xlate *out);

#ifndef __KERNEL__	/* host tests only: not part of the module */
/* Invariants of the live tables (PageTableBuilder::verify).  0 or a static
 * description of the first violation. */
const char *xvc_vm_verify(const struct xvc_vm *vm);

/* FNV-1a over (table PA, 512 entries) of every live table, depth first --
 * the same walk as the sim backend's state digest. */
uint64_t xvc_fnv_mix(uint64_t h, uint64_t v);
uint64_t xvc_vm_digest(const struct xvc_vm *vm, uint64_t h);

#define XVC_FNV_SEED UINT64_C(1469598103934665603)
#endif

#ifdef __cplusplus
}
#endif

#endif /* NICOGPU_VM_CORE_H */
