// SPDX-License-Identifier: GPL-2.0
/*
 * nicogpu_mm.c -- kernel glue of the nicogpu memory manager.
 *
 * All semantics (argument rules, page-table writer, mapping table, VM_BIND
 * transaction and its OOM rollback) live in nicogpu_vm_core.c, which is plain C
 * and is unit-tested on the host.  This file only supplies:
 *   - memory: kvzalloc, the page-table page pool (C3 [16 MiB, 256 MiB)), the
 *     VRAM heap (BAR2-visible VRAM) and GTT BOs (dma_alloc_coherent);
 *   - MMIO: page-table stores through the BAR2 C3 window,
 *     MMU context enable and TLB invalidation;
 *   - objects: BOs as GEM objects (own allocators, driver-private drm_gem_object: handle table, mmap offsets,
 *     reference counting and per-file access control come from the DRM core), per-file VMs (one VMID
 *     each), ioctl handlers and mmap of BOs.
 *
 * No drm_gpuvm: the VM core (nicogpu_vm_core.c) keeps its own mapping table (7295 differential test items against the
 * model's PageTableBuilder), the GEM objects are only what it maps.
 * Not compiled on the host: checked only against stub headers and, on a machine with kernel headers,
 * by a real kbuild.
 */
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/bitops.h>
#include <linux/xarray.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/capability.h>
#include <linux/property.h>
#include <linux/dma-mapping.h>
#include <drm/drm_prime.h>	/* drm_gem_prime_export */
#include "nicogpu_drv.h"

/* Layout of C3 and of BAR2 (PROVISIONAL; ring/firmware are). */
#define NICOGPU_BAR2_LEN		SZ_256M
#define NICOGPU_C3_WIN_LOG2	24			/* 16 MiB window: spec recommendation */
#define NICOGPU_C3_WIN_SIZE	(1ULL << NICOGPU_C3_WIN_LOG2)
#define NICOGPU_C3_PT_OFF		SZ_16M			/* [0, 16 MiB) reserved: ring, firmware */
#define NICOGPU_C3_PT_END		SZ_256M			/* page-table pool end */
#define NICOGPU_GTT_BO_MAX		SZ_1G			/* one contiguous IOVA range; needs a CMA pool this big (cma=2G) */
#define NICOGPU_GTT_TOTAL_MAX	(2ULL * SZ_1G)
#define NICOGPU_INVAL_TIMEOUT_MS	100

/* Every open file of every device; torn-down files are marked, never touch
 * their (possibly gone) device again. */
static DEFINE_MUTEX(nicogpu_files_lock);
static LIST_HEAD(nicogpu_files);

/* the UAPI and the core share one set of values (HAL v0) */
static_assert(NICOGPU_VM_BIND_MAP == XVC_OP_MAP && NICOGPU_VM_BIND_UNMAP == XVC_OP_UNMAP &&
	      NICOGPU_VM_BIND_MAP_SPARSE_NULL == XVC_OP_SPARSE_NULL, "VM_BIND op values");
static_assert(NICOGPU_VM_READ == XVC_VM_READ && NICOGPU_VM_WRITE == XVC_VM_WRITE && NICOGPU_VM_EXEC == XVC_VM_EXEC &&
	      NICOGPU_VM_UNCACHED == XVC_VM_UNCACHED, "VM flag values");
static_assert(NICOGPU_BO_DOMAIN_VRAM == XVC_DOMAIN_VRAM && NICOGPU_BO_DOMAIN_GTT == XVC_DOMAIN_GTT &&
	      NICOGPU_BO_HOST_VISIBLE == XVC_BO_HOST_VISIBLE, "BO domain/flag values");
static_assert(NICOGPU_VM_BIND_MAX_OPS == XVC_MAX_BIND_OPS, "op limit");

/* GEM object reference count: one per handle, one per VM mapping record, one per vma (drm_gem_vm_open/close),
 * transient ones while an ioctl works on the BO.  The last put runs nicogpu_gem_free(). */
struct nicogpu_bo {
	struct drm_gem_object base;
	bool stored;		/* the storage below was allocated (a half-built BO is freed by the same callback) */
	u64 size;		/* what the user asked for, 4 KiB rounded */
	u64 alloc;		/* really allocated: PAGE_ALIGN(size) */
	u64 pa;			/* device PA of byte 0 */
	u32 domain;
	u32 flags;
	void *cpu;		/* GTT */
	dma_addr_t dma;		/* GTT */
};

#define to_bo(o) container_of(o, struct nicogpu_bo, base)

struct nicogpu_vm {
	struct xvc_vm core;
	struct nicogpu_file *xf;
	struct xarray pin;	/* handle -> nicogpu_bo, referenced: the BOs of the VM_BIND in progress (xf->vm_lock) */
	u32 vmid;
	bool hw;		/* tables mirrored to C3, MMU context enabled */
};

/* ---- device state helpers ---------------------------------------------- */

/* RM kind from the shell's RM_INFO (valid after a PR load); 0 = no RM / decoupled / not identified */
u32 nicogpu_rm_kind(struct nicogpu_dev *x)
{
	u32 magic;

	if (!ng_rm_present(x))
		return 0;
	magic = ng_rd(x, NICOGPU_BAR_SHELL, RM_INFO0);
	if (magic != NGI_RMINFO_MAGIC && magic != NGI_RMINFO_MAGIC_LEGACY)
		return 0;
	return ng_rd(x, NICOGPU_BAR_SHELL, RM_INFO1) >> 16;
}

/* RM with the v0.2 BAR2 window (and therefore the MMU) behind BAR0? */
static bool rm_has_mmu(struct nicogpu_dev *x)
{
	u32 info;

	if (!x->mm.mmu_possible || !ng_rm_present(x))
		return false;
	if (!ng_rm_id_ok(ng_rd(x, NICOGPU_BAR_CTRL, RM_ID)))
		return false;
	info = ng_rd(x, NICOGPU_BAR_CTRL, RM_BAR2_INFO);
	if (!info || info == 0xffffffffu)
		return false;
	return (info & 0xff) == 28 && ((info >> 8) & 0xff) <= NICOGPU_C3_WIN_LOG2 &&
	       ((info >> 16) & 0xff) >= NICOGPU_C3_WIN_LOG2;
}

static int win_enable(struct nicogpu_dev *x)
{
	struct nicogpu_mm *m = &x->mm;
	u32 c;

	ng_wr(x, NICOGPU_BAR_CTRL, RM_C3_WIN_CTRL, C3_WIN_ENABLE | (NICOGPU_C3_WIN_LOG2 << C3_WIN_SIZE_SHIFT));
	ng_wr(x, NICOGPU_BAR_CTRL, RM_C3_WIN_BASE, 0);
	c = ng_rd(x, NICOGPU_BAR_CTRL, RM_C3_WIN_CTRL);	/* read-back: mapping is live */
	if ((c & C3_WIN_ERR) || !(c & C3_WIN_ENABLE)) {
		ng_wr(x, NICOGPU_BAR_CTRL, RM_C3_WIN_CTRL, 0);
		dev_err(&x->pdev->dev, "C3 window rejected (CTRL=%08x)\n", c);
		return -EIO;
	}
	m->win_base = 0;
	m->win_on = true;
	m->io_err = false;
	return 0;
}

/* Window still as we left it (a PR / reset of the RM clears it)? */
static bool win_alive(struct nicogpu_dev *x)
{
	u32 c, st;
	bool ok;

	st = ng_rd(x, NICOGPU_BAR_SHELL, RM_STATUS);
	if (st & RM_ST_DECOUPLED) {
		dev_warn_ratelimited(&x->pdev->dev, "win_alive: RM decoupled (RM_STATUS %08x)\n", st);
		return false;
	}
	c = ng_rd(x, NICOGPU_BAR_CTRL, RM_C3_WIN_CTRL);
	ok = (c & C3_WIN_ENABLE) && !(c & C3_WIN_ERR) && ((c >> C3_WIN_SIZE_SHIFT) & 0x1f) == NICOGPU_C3_WIN_LOG2;
	if (!ok)
		dev_warn_ratelimited(&x->pdev->dev, "win_alive: C3_WIN_CTRL read %08x (RM_STATUS %08x)\n", c, st);
	return ok;
}

/* Pointer into the window for C3 byte offset `off` (caller holds bind_lock).
 * Sliding works like this: drain earlier BAR2 writes with a
 * BAR0 read, change the base, read CTRL back before the next BAR2 access. */
static void __iomem *c3_ptr(struct nicogpu_dev *x, u64 off)
{
	struct nicogpu_mm *m = &x->mm;
	u64 base = off & ~(NICOGPU_C3_WIN_SIZE - 1);

	if (base != m->win_base) {
		u32 c;

		wmb();
		(void)ng_rd(x, NICOGPU_BAR_CTRL, RM_C3_WIN_CTRL);
		ng_wr(x, NICOGPU_BAR_CTRL, RM_C3_WIN_BASE, (u32)base);
		c = ng_rd(x, NICOGPU_BAR_CTRL, RM_C3_WIN_CTRL);
		if (c & C3_WIN_ERR)
			m->io_err = true;
		m->win_base = base;
	}
	return m->c3win + (off - base);
}

static bool mmu_inval_wait(struct nicogpu_dev *x, u32 mask)
{
	u32 v;

	ng_wr(x, NICOGPU_BAR_CTRL, NICOGPU_REG_MMU_INVALIDATE, mask);
	return !read_poll_timeout(ng_rd, v, v != 0xffffffffu && !(v & mask), 20,
				  NICOGPU_INVAL_TIMEOUT_MS * USEC_PER_MSEC, false, x, NICOGPU_BAR_CTRL,
				  NICOGPU_REG_MMU_INVALIDATE);
}

static u32 ctx_reg(u32 vmid, u32 reg)
{
	return NICOGPU_REG_MMU_CTX_BASE + NICOGPU_REG_MMU_CTX_STRIDE * vmid + reg;
}

static bool ctx_enable(struct nicogpu_dev *x, u32 vmid, u64 root)
{
	/* the root table was filled through the window: push it first */
	wmb();
	ng_wr(x, NICOGPU_BAR_CTRL, ctx_reg(vmid, NICOGPU_REG_MMU_CTX_ROOT_LO), lower_32_bits(root));
	ng_wr(x, NICOGPU_BAR_CTRL, ctx_reg(vmid, NICOGPU_REG_MMU_CTX_ROOT_HI), upper_32_bits(root));
	ng_wr(x, NICOGPU_BAR_CTRL, ctx_reg(vmid, NICOGPU_REG_MMU_CTX_CTRL), 1);
	return mmu_inval_wait(x, BIT(vmid));
}

static void ctx_disable(struct nicogpu_dev *x, u32 vmid)
{
	ng_wr(x, NICOGPU_BAR_CTRL, ctx_reg(vmid, NICOGPU_REG_MMU_CTX_CTRL), 0);
	(void)mmu_inval_wait(x, BIT(vmid));
}

/* ---- xvc_ops ------------------------------------------------------------ */

static struct nicogpu_dev *vm_dev(void *ctx)
{
	return ((struct nicogpu_vm *)ctx)->xf->x;
}

static void *op_zalloc(void *ctx, size_t size)
{
	return kvzalloc(size, GFP_KERNEL_ACCOUNT);
}

static void op_free(void *ctx, void *p)
{
	kvfree(p);
}

static u64 op_pt_alloc(void *ctx)
{
	struct nicogpu_mm *m = &vm_dev(ctx)->mm;
	u64 pa = 0;
	bool ok;

	mutex_lock(&m->lock);
	ok = xvc_pool_alloc(&m->pt, XVC_PAGE, XVC_PAGE, &pa);
	mutex_unlock(&m->lock);
	return ok ? pa : 0;
}

static void op_pt_free(void *ctx, u64 pa)
{
	struct nicogpu_mm *m = &vm_dev(ctx)->mm;

	mutex_lock(&m->lock);
	xvc_pool_free(&m->pt, pa, XVC_PAGE);
	mutex_unlock(&m->lock);
}

/* New table: full 512-entry image before the PDE that links it exists.  Entries
 * are little endian in device memory; the host (aarch64/x86) is too. */
static void op_pt_fill(void *ctx, u64 pa, const u64 *ents)
{
	struct nicogpu_vm *v = ctx;
	struct nicogpu_mm *m = &v->xf->x->mm;
	void __iomem *p;

	if (!v->hw)
		return;
	p = c3_ptr(v->xf->x, pa - NICOGPU_PA_SYS_BASE);
	memcpy_toio(p, ents, XVC_PAGE);
	m->last_wr = p;
	m->dirty = true;
}

/* exactly one 64-bit store */
static void op_pt_write(void *ctx, u64 pa, u64 val)
{
	struct nicogpu_vm *v = ctx;
	struct nicogpu_mm *m = &v->xf->x->mm;
	void __iomem *p;

	if (!v->hw)
		return;
	p = c3_ptr(v->xf->x, pa - NICOGPU_PA_SYS_BASE);
	writeq_relaxed(val, p);	/* writeq converts to little endian itself */
	m->last_wr = p;
	m->dirty = true;
}

/* Read back a location written through the same window
 * (non-posted, pushes the earlier posted stores), then invalidate the VMID. */
static bool op_invalidate(void *ctx)
{
	struct nicogpu_vm *v = ctx;
	struct nicogpu_dev *x = v->xf->x;
	struct nicogpu_mm *m = &x->mm;
	bool ok;

	if (!v->hw)
		return true;
	if (m->dirty) {
		wmb();
		(void)readq_relaxed(m->last_wr);
		m->dirty = false;
	}
	ok = mmu_inval_wait(x, BIT(v->vmid));
	if (!ok)
		dev_warn(&x->pdev->dev, "VMID %u TLB invalidate timed out\n", v->vmid);
	return ok;
}

/* The core resolves handles itself, one by one, while it validates; GEM handles can be closed by another thread
 * (DRM_IOCTL_GEM_CLOSE does not take xf->vm_lock) so nicogpu_ioc_vm_bind() looks the BOs of the call up first, with
 * references, into `pin` and the core only reads that table. */
static int op_bo_lookup(void *ctx, u32 handle, struct xvc_bo_info *out)
{
	struct nicogpu_bo *bo = xa_load(&((struct nicogpu_vm *)ctx)->pin, handle);

	if (!bo)
		return -ENOENT;
	out->bo = bo;
	out->pa = bo->pa;
	out->size = bo->size;
	return 0;
}

static void op_bo_get(void *ctx, void *bo)
{
	drm_gem_object_get(&((struct nicogpu_bo *)bo)->base);
}

static void op_bo_put(void *ctx, void *bo)
{
	drm_gem_object_put(&((struct nicogpu_bo *)bo)->base);
}

static const struct xvc_ops nicogpu_vm_ops = {
	.zalloc = op_zalloc,
	.free = op_free,
	.pt_alloc = op_pt_alloc,
	.pt_free = op_pt_free,
	.pt_fill = op_pt_fill,
	.pt_write = op_pt_write,
	.invalidate = op_invalidate,
	.bo_lookup = op_bo_lookup,
	.bo_get = op_bo_get,
	.bo_put = op_bo_put,
};

/* ---- BOs ---------------------------------------------------------------- */

/* drm_gem_object_funcs.free: last reference gone.  May run after the device was removed (an open file, a vma or a
 * GEM handle outlived it): nicogpu_mm_fini() then already dropped the pool bitmaps (NULL under m->lock), the CPU mappings
 * were zapped by remove(), and only the host-memory (GTT) storage is still returned. */
static void nicogpu_gem_free(struct drm_gem_object *obj)
{
	struct nicogpu_bo *bo = to_bo(obj);
	struct nicogpu_dev *x = to_nicogpu(obj->dev);
	struct nicogpu_mm *m = &x->mm;

	if (bo->stored && bo->domain == NICOGPU_BO_DOMAIN_GTT) {
		dma_free_coherent(&x->pdev->dev, bo->alloc, bo->cpu, bo->dma);
		mutex_lock(&m->lock);
		m->gtt_used -= bo->alloc;
		mutex_unlock(&m->lock);
	} else if (bo->stored) {
		mutex_lock(&m->lock);
		if (m->vram_bits)
			xvc_pool_free(&m->vram, bo->pa, bo->alloc);
		mutex_unlock(&m->lock);
	}
	drm_gem_object_release(obj);
	kfree(bo);
}

static int nicogpu_gem_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma);

/* PRIME export: host-memory (GTT) BOs only.  VRAM is BAR2 IO memory without struct page
 * (no scatterlist an importer could DMA-map); the display path blits to a GTT BO first.  The dma-buf is
 * the DRM core's (drm_gem_prime_export): it keeps a GEM reference and maps through get_sg_table() below.  No implicit
 * sync (explicit sync only, syncobj / sync_file).  Import: only the DRM core default (same-device loopback); a foreign
 * dma-buf is refused by the core.  Export runs under drm_dev_enter(): after unplug no new dma-buf is created. */
static struct dma_buf *nicogpu_gem_export(struct drm_gem_object *obj, int flags)
{
	struct nicogpu_bo *bo = to_bo(obj);
	struct dma_buf *buf;
	int idx;

	if (!drm_dev_enter(obj->dev, &idx))
		return ERR_PTR(-ENODEV);
	if (bo->domain != NICOGPU_BO_DOMAIN_GTT || !bo->stored)
		buf = ERR_PTR(-EOPNOTSUPP);
	else
		buf = drm_gem_prime_export(obj, flags);
	drm_dev_exit(idx);
	return buf;
}

/* Pages behind the dma_alloc_coherent() buffer, for the importer's DMA mapping (freed by the DRM core: sg_free_table +
 * kfree).  Cache attributes: the export itself is safe on non-coherent hosts (the sg table only names pages; the
 * importer's DMA is coherent with dma_alloc_coherent memory for the importing device as the DMA API defines).  The
 * alias risk (uncached coherent mapping vs the cacheable linear alias) exists only for an importer's CPU kmap of the
 * pages; CPU access through the dma-buf goes through vmap / mmap below, which use the coherent mapping itself. */
static struct sg_table *nicogpu_gem_get_sg_table(struct drm_gem_object *obj)
{
	struct nicogpu_bo *bo = to_bo(obj);
	struct nicogpu_dev *x = to_nicogpu(obj->dev);
	struct sg_table *sgt;
	int rc, idx;

	if (!drm_dev_enter(obj->dev, &idx))
		return ERR_PTR(-ENODEV);
	if (bo->domain != NICOGPU_BO_DOMAIN_GTT || !bo->stored) {
		sgt = ERR_PTR(-EINVAL);
		goto out;
	}
	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		sgt = ERR_PTR(-ENOMEM);
		goto out;
	}
	rc = dma_get_sgtable(&x->pdev->dev, sgt, bo->cpu, bo->dma, bo->alloc);
	if (rc) {
		kfree(sgt);
		sgt = ERR_PTR(rc);
	}
out:
	drm_dev_exit(idx);
	return sgt;
}

/* Kernel CPU mapping for dma-buf vmap (GTT BOs only): the coherent allocation's own mapping, valid until the BO is
 * freed (the dma-buf holds a GEM reference, so not before vunmap). */
static int nicogpu_gem_vmap(struct drm_gem_object *obj, struct iosys_map *map)
{
	struct nicogpu_bo *bo = to_bo(obj);
	int idx, rc = 0;

	if (!drm_dev_enter(obj->dev, &idx))
		return -ENODEV;
	if (bo->domain != NICOGPU_BO_DOMAIN_GTT || !bo->stored || !bo->cpu)
		rc = -EINVAL;
	else
		iosys_map_set_vaddr(map, bo->cpu);
	drm_dev_exit(idx);
	return rc;
}

static void nicogpu_gem_vunmap(struct drm_gem_object *obj, struct iosys_map *map)
{
	/* nothing to undo: the mapping belongs to the BO */
}

static const struct vm_operations_struct nicogpu_gem_vm_ops = {
	.open = drm_gem_vm_open,	/* get/put the GEM object per vma (fork, munmap splits) */
	.close = drm_gem_vm_close,
};

static const struct drm_gem_object_funcs nicogpu_gem_funcs = {
	.free = nicogpu_gem_free,
	.mmap = nicogpu_gem_mmap,
	.export = nicogpu_gem_export,
	.get_sg_table = nicogpu_gem_get_sg_table,
	.vmap = nicogpu_gem_vmap,
	.vunmap = nicogpu_gem_vunmap,
	.vm_ops = &nicogpu_gem_vm_ops,
};

static bool bo_mappable(const struct nicogpu_bo *bo)
{
	return bo->domain == NICOGPU_BO_DOMAIN_GTT || (bo->flags & NICOGPU_BO_HOST_VISIBLE);
}

/* New BOs must read as zero: VRAM is cleared through
 * the WC mapping of BAR2; GTT comes zeroed from dma_alloc_coherent. */
static int bo_alloc_vram(struct nicogpu_dev *x, struct nicogpu_bo *bo, u64 align)
{
	struct nicogpu_mm *m = &x->mm;
	u64 off, pos = 0;
	bool ok;

	if (!ng_rm_present(x))
		return -EAGAIN;		/* BAR2 is the RM's memory: nothing to clear through */
	mutex_lock(&m->lock);
	ok = xvc_pool_alloc(&m->vram, bo->alloc, align, &bo->pa);
	mutex_unlock(&m->lock);
	if (!ok)
		return -ENOMEM;
	bo->stored = true;
	off = bo->pa;
	while (pos < bo->alloc) {
		u64 n = min_t(u64, bo->alloc - pos, SZ_1M);

		memset_io(m->bar2wc + off + pos, 0, n);
		pos += n;
		cond_resched();
	}
	wmb();
	(void)ng_rd(x, NICOGPU_BAR_CTRL, RM_ID);	/* drain the posted clears */
	return 0;
}

/* true: the host is not dma-coherent, GTT memory is CPU-uncached (WC) */
static bool gtt_uncached(struct nicogpu_dev *x)
{
	return device_get_dma_attr(&x->pdev->dev) != DEV_DMA_COHERENT;
}

static int bo_alloc_gtt(struct nicogpu_dev *x, struct nicogpu_bo *bo, u64 align, bool explicit_align)
{
	const u64 must_align = explicit_align ? align : PAGE_SIZE;
	struct nicogpu_mm *m = &x->mm;
	int rc = 0;

	/* Coherent host: dma_alloc_coherent() is cached + snooped.  Non-coherent host: the same call returns CPU-uncached
	 * (write-combined) memory and dma_mmap_coherent() maps it that way, so the device's No-Snoop=0 accesses and the CPU
	 * agree without cache maintenance; GET_GPU_INFO says so with NICOGPU_GPU_INFO_F_GTT_UNCACHED. */
	if (bo->alloc > NICOGPU_GTT_BO_MAX)
		return -ENOMEM;
	mutex_lock(&m->lock);
	if (m->gtt_used + bo->alloc > NICOGPU_GTT_TOTAL_MAX)
		rc = -ENOMEM;
	else
		m->gtt_used += bo->alloc;
	mutex_unlock(&m->lock);
	if (rc)
		return rc;
	bo->cpu = dma_alloc_coherent(&x->pdev->dev, bo->alloc, &bo->dma, GFP_KERNEL | __GFP_NOWARN);
	/* IOVA must be below 2^47 and honour an explicitly requested alignment (best effort of the DMA API: checked, not
	 * forced).  The default 64K / 2M alignment is only a preference: the page-table writer checks the PA before using
	 * a large page, and CMA hands out blocks > 4 MiB with CONFIG_CMA_ALIGNMENT (1 MiB) alignment only. */
	if (bo->cpu && ((u64)bo->dma + bo->alloc > (1ULL << 47) || ((u64)bo->dma & (must_align - 1)))) {
		dma_free_coherent(&x->pdev->dev, bo->alloc, bo->cpu, bo->dma);
		bo->cpu = NULL;
	}
	if (!bo->cpu) {
		mutex_lock(&m->lock);
		m->gtt_used -= bo->alloc;
		mutex_unlock(&m->lock);
		return -ENOMEM;
	}
	bo->pa = NICOGPU_PA_HOST_BASE + bo->dma;
	bo->stored = true;
	return 0;
}

long nicogpu_ioc_bo_create(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_dev *x = xf->x;
	struct nicogpu_bo_create *a = data;
	struct nicogpu_bo *bo;
	u64 size, align;
	u32 handle = 0;
	bool mappable;
	int rc;

	if (a->pad)
		return -EINVAL;
	rc = xvc_bo_args_check(a->size, a->alignment, a->domain, a->flags, &size, &align);
	if (rc)
		return rc;
	if (a->domain == NICOGPU_BO_DOMAIN_VRAM && !ngi_rm_kind_has_vram(nicogpu_rm_kind(x)))
		return -EOPNOTSUPP;	/* GET_GPU_INFO.info_flags has NICOGPU_GPU_INFO_F_NO_VRAM: the loaded RM has no VRAM path */
	bo = kzalloc(sizeof(*bo), GFP_KERNEL_ACCOUNT);
	if (!bo)
		return -ENOMEM;
	bo->size = size;
	bo->alloc = PAGE_ALIGN(size);
	bo->domain = a->domain;
	bo->flags = a->flags;
	mappable = bo_mappable(bo);
	drm_gem_private_object_init(&x->drm, &bo->base, bo->alloc);	/* no shmem backing: our own allocators */
	bo->base.funcs = &nicogpu_gem_funcs;
	if (align < PAGE_SIZE)
		align = PAGE_SIZE;	/* no BO shares a host page with another */
	rc = a->domain == NICOGPU_BO_DOMAIN_VRAM ? bo_alloc_vram(x, bo, align) : bo_alloc_gtt(x, bo, align, a->alignment != 0);
	if (!rc && mappable)
		rc = drm_gem_create_mmap_offset(&bo->base);
	if (!rc)
		rc = drm_gem_handle_create(xf->file, &bo->base, &handle);	/* the handle holds its own reference */
	if (!rc) {
		a->size = size;
		a->handle = handle;
		a->mmap_offset = mappable ? drm_vma_node_offset_addr(&bo->base.vma_node) : 0;
	}
	drm_gem_object_put(&bo->base);	/* the creation reference; on failure this frees the BO */
	if (rc)
		return rc == -ENOSPC ? -ENOMEM : rc;	/* -ENOSPC is reserved for "no handle / mapping limit" in the UAPI */
	return 0;
}

long nicogpu_ioc_bo_free(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_bo_free *a = data;

	if (a->pad)
		return -EINVAL;
	/* VM mappings and vmas keep the storage alive until they go away; drm_gem_handle_delete() says -EINVAL for an
	 * unknown handle, the UAPI says -ENOENT */
	return drm_gem_handle_delete(xf->file, a->handle) ? -ENOENT : 0;
}

/* ---- VMs ---------------------------------------------------------------- */

/* caller holds xf->vm_lock; takes bind_lock, then xf->lock to publish the VM */
static int vm_create_locked(struct nicogpu_file *xf, struct nicogpu_vm_create *a)
{
	struct nicogpu_dev *x = xf->x;
	struct nicogpu_mm *m = &x->mm;
	struct nicogpu_vm *v;
	u32 vmid;
	int rc;

	v = kzalloc(sizeof(*v), GFP_KERNEL_ACCOUNT);
	if (!v)
		return -ENOMEM;
	v->xf = xf;
	xa_init(&v->pin);
	mutex_lock(&m->lock);
	if (m->pr_busy) {			/* a partial reconfiguration is about to replace the RM */
		mutex_unlock(&m->lock);
		kfree(v);
		return -EBUSY;
	}
	vmid = find_first_zero_bit(&m->vmid_map, NICOGPU_NUM_VMIDS);
	if (vmid >= NICOGPU_NUM_VMIDS) {
		mutex_unlock(&m->lock);
		kfree(v);
		return -ENOSPC;
	}
	set_bit(vmid, &m->vmid_map);
	mutex_unlock(&m->lock);
	v->vmid = vmid;

	mutex_lock(&m->bind_lock);
	v->hw = rm_has_mmu(x);
	if (v->hw && !(m->win_on && win_alive(x)))
		rc = win_enable(x);
	else
		rc = 0;
	if (!rc) {
		rc = xvc_vm_init(&v->core, &nicogpu_vm_ops, v);
		if (!rc && v->hw) {
			if (m->io_err || !ctx_enable(x, vmid, xvc_vm_root_pa(&v->core))) {
				rc = -EIO;
				xvc_vm_fini(&v->core);
			} else {
				m->ctx_root[vmid] = xvc_vm_root_pa(&v->core);	/* nicogpu_mm_rm_restore() re-programs it */
			}
		}
	}
	mutex_unlock(&m->bind_lock);
	if (rc) {
		mutex_lock(&m->lock);
		clear_bit(vmid, &m->vmid_map);
		mutex_unlock(&m->lock);
		kfree(v);
		return rc;
	}
	mutex_lock(&xf->lock);
	xf->vms[vmid] = v;
	mutex_unlock(&xf->lock);
	a->vm_id = vmid;
	a->status = v->hw ? 0 : NICOGPU_VM_STATUS_SHADOW_ONLY;
	return 0;
}

/* Take the VM out of xf->vms[] (caller holds xf->vm_lock): from here no ioctl of the file can find it.  The caller
 * has checked under the same xf->lock section that no queue is bound to it, if that matters. */
static void vm_unpublish(struct nicogpu_file *xf, struct nicogpu_vm *v)
{
	mutex_lock(&xf->lock);
	xf->vms[v->vmid] = NULL;
	mutex_unlock(&xf->lock);
}

/* caller holds xf->vm_lock; `v` is unpublished (vm_unpublish).  `mmio`: the device may still be programmed. */
static void vm_destroy_locked(struct nicogpu_file *xf, struct nicogpu_vm *v, bool mmio)
{
	struct nicogpu_mm *m = &xf->x->mm;

	mutex_lock(&m->bind_lock);
	if (v->hw && mmio && ng_rm_present(xf->x))
		ctx_disable(xf->x, v->vmid);
	m->ctx_root[v->vmid] = 0;
	xvc_vm_fini(&v->core);
	mutex_unlock(&m->bind_lock);
	mutex_lock(&m->lock);
	clear_bit(v->vmid, &m->vmid_map);
	mutex_unlock(&m->lock);
	xa_destroy(&v->pin);
	kfree(v);
}

static struct nicogpu_vm *vm_get(struct nicogpu_file *xf, u32 vmid)
{
	return vmid > 0 && vmid < NICOGPU_NUM_VMIDS ? xf->vms[vmid] : NULL;
}

long nicogpu_ioc_vm_create(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_vm_create *a = data;
	int rc;

	if (a->flags || a->pad)
		return -EINVAL;
	mutex_lock(&xf->vm_lock);
	rc = vm_create_locked(xf, a);
	mutex_unlock(&xf->vm_lock);
	return rc;
}

long nicogpu_ioc_vm_destroy(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_vm_destroy *a = data;
	struct nicogpu_vm *v;
	long rc = 0;

	if (a->pad)
		return -EINVAL;
	mutex_lock(&xf->vm_lock);
	v = vm_get(xf, a->vm_id);
	if (!v) {
		rc = -ENOENT;
	} else {
		/* the in-use check and the unpublish in one xf->lock section: QUEUE_CREATE (xf->lock) either sees the VM
		 * gone or made it busy */
		mutex_lock(&xf->lock);
		if (nicogpu_sched_vm_in_use(xf, a->vm_id))
			rc = -EBUSY;	/* no queue may reference a VM being destroyed */
		else
			xf->vms[v->vmid] = NULL;
		mutex_unlock(&xf->lock);
		if (!rc)
			vm_destroy_locked(xf, v, true);	/* slow (TLB wait): submits of the file are not blocked */
	}
	mutex_unlock(&xf->vm_lock);
	return rc;
}

/* Resolve the MAP BOs of one VM_BIND call into v->pin, holding a reference each (xf->vm_lock held).  Handles that do not
 * exist are simply absent: the core reports -ENOENT at the right place of its validation order. */
static void vm_bind_unpin(struct nicogpu_vm *v)
{
	struct nicogpu_bo *bo;
	unsigned long idx;

	xa_for_each(&v->pin, idx, bo) {
		xa_erase(&v->pin, idx);
		drm_gem_object_put(&bo->base);
	}
}

static int vm_bind_pin(struct nicogpu_vm *v, const struct xvc_bind_op *ops, u32 n)
{
	u32 i;

	for (i = 0; i < n; i++) {
		struct drm_gem_object *obj;

		if (ops[i].op != XVC_OP_MAP || xa_load(&v->pin, ops[i].bo_handle))
			continue;
		obj = drm_gem_object_lookup(v->xf->file, ops[i].bo_handle);
		if (!obj)
			continue;
		if (xa_err(xa_store(&v->pin, ops[i].bo_handle, to_bo(obj), GFP_KERNEL_ACCOUNT))) {
			drm_gem_object_put(obj);
			vm_bind_unpin(v);
			return -ENOMEM;
		}
	}
	return 0;
}

long nicogpu_ioc_vm_bind(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_vm_bind *a = data;
	struct xvc_bind_op *ops = NULL;
	struct nicogpu_vm *v;
	struct nicogpu_mm *m = &xf->x->mm;
	long rc;

	BUILD_BUG_ON(sizeof(struct nicogpu_vm_bind_op) != sizeof(struct xvc_bind_op));
	if (a->flags || a->pad || a->num_ops > NICOGPU_VM_BIND_MAX_OPS)
		return -EINVAL;
	if (a->num_ops) {
		ops = kvmalloc_array(a->num_ops, sizeof(*ops), GFP_KERNEL_ACCOUNT);
		if (!ops)
			return -ENOMEM;
		if (copy_from_user(ops, u64_to_user_ptr(a->ops), (size_t)a->num_ops * sizeof(*ops))) {
			kvfree(ops);
			return -EFAULT;
		}
	}
	mutex_lock(&xf->vm_lock);	/* not xf->lock: a bind must not stall the file's submits */
	v = vm_get(xf, a->vm_id);
	if (!v) {
		rc = -ENOENT;
		goto out;
	}
	rc = vm_bind_pin(v, ops, a->num_ops);
	if (rc)
		goto out;
	mutex_lock(&m->bind_lock);
	if (v->hw && !win_alive(xf->x)) {
		/* the RM was replaced or reset: this VM's tables are gone */
		rc = -EIO;
	} else {
		m->io_err = false;
		rc = xvc_vm_bind(&v->core, ops, a->num_ops);
		if (m->io_err) {
			/* a slide was rejected: some page-table stores went to the
			 * wrong place; atomicity cannot be promised any more */
			dev_err(&xf->x->pdev->dev, "VM_BIND: C3 window slide rejected, VM %u unreliable\n",
				v->vmid);
			if (!rc)
				rc = -EIO;
		}
	}
	mutex_unlock(&m->bind_lock);
	vm_bind_unpin(v);
out:
	mutex_unlock(&xf->vm_lock);
	kvfree(ops);
	return rc;
}

long nicogpu_ioc_vm_lookup(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_vm_lookup *a = data;
	struct xvc_xlate t;
	struct nicogpu_vm *v;
	long rc = 0;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	if (a->pad)
		return -EINVAL;
	mutex_lock(&xf->vm_lock);
	v = vm_get(xf, a->vm_id);
	if (v)
		xvc_vm_lookup(&v->core, a->va, &t);
	else
		rc = -ENOENT;
	mutex_unlock(&xf->vm_lock);
	if (rc)
		return rc;
	a->kind = t.kind;
	a->level = t.level;
	a->entry = t.entry;
	a->pa = t.pa;
	a->page_size = t.page_size;
	return 0;
}

/* ---- mmap of BOs -------------------------------------------------------- */

/* drm_gem_object_funcs.mmap: called by drm_gem_mmap() with the GEM reference for this vma already taken and
 * vm_ops (open/close = get/put) set; the whole object from byte 0 (the fake offset names the object, the length may
 * be shorter than the object). */
static int nicogpu_gem_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	struct nicogpu_bo *bo = to_bo(obj);
	struct nicogpu_dev *x = to_nicogpu(obj->dev);
	unsigned long start = drm_vma_node_start(&obj->vma_node);
	u64 len = vma->vm_end - vma->vm_start;
	int idx, rc;

	if (!drm_dev_enter(obj->dev, &idx))
		return -ENODEV;
	if (vma->vm_pgoff >= start)
		vma->vm_pgoff -= start;		/* fake offset -> offset into the object (drivers differ on who does it) */
	if (!(vma->vm_flags & VM_SHARED) || !bo_mappable(bo) || vma->vm_pgoff || len > bo->alloc) {
		rc = -EINVAL;
		goto out;
	}
	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);
	if (bo->domain == NICOGPU_BO_DOMAIN_VRAM)
		rc = io_remap_pfn_range(vma, vma->vm_start,
					(pci_resource_start(x->pdev, NICOGPU_BAR_MEM) + bo->pa) >> PAGE_SHIFT,
					len, pgprot_writecombine(vma->vm_page_prot));
	else
		rc = dma_mmap_coherent(&x->pdev->dev, vma, bo->cpu, bo->dma, bo->alloc);
out:
	drm_dev_exit(idx);
	return rc;
}

/* The C3 window hides the top of BAR2 from users once the driver enabled it.
 * Debug BAR2 mmaps must not reach it; the
 * reservation is unconditional so users never see the layout change. */
bool nicogpu_mm_bar2_user_ok(struct nicogpu_dev *x, u64 off, u64 len)
{
	if (!x->mm.mmu_possible)
		return true;
	return off + len >= off && off + len <= x->mm.vram_visible;
}

/* GET_GPU_INFO inputs that only the allocator knows */
void nicogpu_mm_fill_hw(struct nicogpu_dev *x, struct ngi_hw *hw)
{
	struct nicogpu_mm *m = &x->mm;

	mutex_lock(&m->lock);
	hw->vram_heap_bytes = m->vram.npages * XVC_PAGE;
	hw->vram_used = m->vram.used * XVC_PAGE;
	hw->gtt_used = m->gtt_used;
	mutex_unlock(&m->lock);
	hw->gtt_uncached = gtt_uncached(x);
	hw->gtt_total_max = NICOGPU_GTT_TOTAL_MAX;
	hw->gtt_bo_max = NICOGPU_GTT_BO_MAX;
	hw->max_vms_limit = NICOGPU_NUM_VMIDS - 1;
	hw->va_user_start = NICOGPU_VA_USER_START;
	hw->va_user_end = NICOGPU_VA_USER_END;
}

/* ---- file lifetime ------------------------------------------------------ */

int nicogpu_mm_open(struct nicogpu_file *xf)
{
	mutex_init(&xf->vm_lock);
	mutex_init(&xf->lock);
	nicogpu_sched_open(xf);		/* queue / syncobj tables */
	INIT_LIST_HEAD(&xf->node);
	mutex_lock(&nicogpu_files_lock);
	list_add(&xf->node, &nicogpu_files);
	mutex_unlock(&nicogpu_files_lock);
	return 0;
}

/* nicogpu_files_lock held; releases the queues and VMs (the VMs hold BO references); the GEM handles of the file are
 * dropped by the DRM core (drm_gem_release) */
static void file_teardown(struct nicogpu_file *xf, bool mmio)
{
	u32 i;

	/* queues first: their jobs run in the VMs and touch the BOs below */
	nicogpu_sched_file_fini(xf);
	mutex_lock(&xf->vm_lock);
	for (i = 1; i < NICOGPU_NUM_VMIDS; i++) {
		struct nicogpu_vm *v = xf->vms[i];

		if (v) {
			vm_unpublish(xf, v);
			vm_destroy_locked(xf, v, mmio);
		}
	}
	mutex_unlock(&xf->vm_lock);
	xf->torn = true;
}

/* Final close.  A file torn down by remove() no longer touches the device. */
void nicogpu_mm_release(struct nicogpu_file *xf)
{
	mutex_lock(&nicogpu_files_lock);
	if (!xf->torn)
		file_teardown(xf, true);
	list_del_init(&xf->node);
	mutex_unlock(&nicogpu_files_lock);
}

/* remove(): after drm_dev_unplug() (no ioctl is running or can start), device still programmable.
 * Tears down the queues and VMs the open files still own, so no VM outlives the pools; their BOs are dropped by the
 * DRM core when the files are closed (nicogpu_gem_free() copes with a dead device), their CPU mappings were zapped by
 * remove(). */
void nicogpu_mm_remove_files(struct nicogpu_dev *x)
{
	struct nicogpu_file *xf, *tmp;

	mutex_lock(&nicogpu_files_lock);
	list_for_each_entry_safe(xf, tmp, &nicogpu_files, node) {
		if (xf->x != x || xf->torn)
			continue;
		file_teardown(xf, true);
		list_del_init(&xf->node);
	}
	mutex_unlock(&nicogpu_files_lock);
}

/* ---- C3 front region (ring / firmware) ---------------------------- */

/* The kernel half of the C3 discipline: make sure the C3 window is open.  The
 * ring lives in C3 [0, 16 MiB), below the page-table pool. */
int nicogpu_mm_c3_open(struct nicogpu_dev *x)
{
	struct nicogpu_mm *m = &x->mm;
	int rc = 0;

	mutex_lock(&m->bind_lock);
	if (!rm_has_mmu(x))
		rc = -ENODEV;
	else if (!(m->win_on && win_alive(x)))
		rc = win_enable(x);
	mutex_unlock(&m->bind_lock);
	return rc;
}

/* After a GPU-zone reset the RM forgot the C3 window and every MMU context, but the page
 * tables themselves live in C3 (static DDR) and survive: put the window and the contexts back. */
int nicogpu_mm_rm_restore(struct nicogpu_dev *x)
{
	struct nicogpu_mm *m = &x->mm;
	u32 vmid;
	int rc;

	mutex_lock(&m->bind_lock);
	if (!rm_has_mmu(x)) {
		rc = -ENODEV;
	} else {
		rc = win_enable(x);
		for (vmid = 1; !rc && vmid < NICOGPU_NUM_VMIDS; vmid++)
			if (m->ctx_root[vmid] && !ctx_enable(x, vmid, m->ctx_root[vmid]))
				rc = -EIO;
	}
	mutex_unlock(&m->bind_lock);
	return rc;
}

int nicogpu_mm_pr_begin(struct nicogpu_dev *x)
{
	struct nicogpu_mm *m = &x->mm;
	int rc = 0;

	mutex_lock(&m->lock);
	if (m->vmid_map != 1UL)			/* bit 0 = the physical bypass: anything else is a live VM */
		rc = -EBUSY;
	else
		m->pr_busy = true;
	mutex_unlock(&m->lock);
	return rc;
}

void nicogpu_mm_pr_end(struct nicogpu_dev *x, bool rm_replaced)
{
	struct nicogpu_mm *m = &x->mm;

	if (rm_replaced) {
		mutex_lock(&m->bind_lock);
		m->win_on = false;		/* the new RM starts with the window closed */
		m->io_err = false;
		mutex_unlock(&m->bind_lock);
	}
	mutex_lock(&m->lock);
	m->pr_busy = false;
	mutex_unlock(&m->lock);
}

/* Little-endian dwords at C3 byte offset `off` (4-aligned), through the WC
 * window with the slide/read-back discipline of c3_ptr().  The caller rings
 * the doorbell afterwards: the DFX boundary holds it back until the BAR2
 * writes have landed. */
int nicogpu_mm_c3_write(struct nicogpu_dev *x, u64 off, const u32 *dw, u32 n)
{
	struct nicogpu_mm *m = &x->mm;
	void __iomem *p;
	int rc = 0;

	if ((off & 3) || off + 4ULL * n > NICOGPU_C3_PT_OFF)
		return -EINVAL;
	mutex_lock(&m->bind_lock);
	if (!m->win_on || !win_alive(x)) {
		dev_err(&x->pdev->dev, "C3 ring write failed: win_on %d, C3_WIN_CTRL %08x, C3_WIN_BASE %08x, RM_STATUS %08x\n",
			m->win_on, ng_rd(x, NICOGPU_BAR_CTRL, RM_C3_WIN_CTRL), ng_rd(x, NICOGPU_BAR_CTRL, RM_C3_WIN_BASE),
			ng_rd(x, NICOGPU_BAR_SHELL, RM_STATUS));
		rc = -EIO;		/* RM replaced / reset: the ring is gone */
	} else {
		m->io_err = false;
		p = c3_ptr(x, off);
		memcpy_toio(p, dw, 4ULL * n);
		wmb();
		if (m->io_err)
			rc = -EIO;
	}
	mutex_unlock(&m->bind_lock);
	return rc;
}

/* ---- device init / fini ------------------------------------------------- */

int nicogpu_mm_init(struct nicogpu_dev *x)
{
	struct nicogpu_mm *m = &x->mm;
	resource_size_t bar2 = pci_resource_len(x->pdev, NICOGPU_BAR_MEM);
	u64 pt_pages = (NICOGPU_C3_PT_END - NICOGPU_C3_PT_OFF) / XVC_PAGE;
	u64 vram_pages;

	mutex_init(&m->lock);
	mutex_init(&m->bind_lock);
	m->mmu_possible = bar2 == NICOGPU_BAR2_LEN;
	m->vram_visible = bar2 > NICOGPU_C3_WIN_SIZE ? bar2 - NICOGPU_C3_WIN_SIZE : 0;
	m->vmid_map = 1;		/* VMID 0 is the physical bypass */
	vram_pages = m->vram_visible / XVC_PAGE;

	m->bar2wc = ioremap_wc(pci_resource_start(x->pdev, NICOGPU_BAR_MEM), bar2);
	if (!m->bar2wc)
		return -ENOMEM;
	m->c3win = m->bar2wc + (m->mmu_possible ? NICOGPU_BAR2_LEN - NICOGPU_C3_WIN_SIZE : 0);
	m->vram_bits = kvcalloc(xvc_pool_words(vram_pages), sizeof(u64), GFP_KERNEL);
	m->pt_bits = kvcalloc(xvc_pool_words(pt_pages), sizeof(u64), GFP_KERNEL);
	if (!m->vram_bits || !m->pt_bits) {
		nicogpu_mm_fini(x);
		return -ENOMEM;
	}
	xvc_pool_init(&m->vram, NICOGPU_PA_VRAM_BASE, vram_pages, m->vram_bits);
	xvc_pool_init(&m->pt, NICOGPU_PA_SYS_BASE + NICOGPU_C3_PT_OFF, pt_pages, m->pt_bits);
	dev_info(&x->pdev->dev, "mm: VRAM heap %llu MiB, page-table pool %llu MiB, MMU/C3 window %s\n",
		 m->vram_visible >> 20, (u64)(NICOGPU_C3_PT_END - NICOGPU_C3_PT_OFF) >> 20,
		 m->mmu_possible ? "layout ok" : "unavailable (BAR2 != 256 MiB)");
	return 0;
}

/* After nicogpu_mm_remove_files(): no VM or BO remains. */
void nicogpu_mm_fini(struct nicogpu_dev *x)
{
	struct nicogpu_mm *m = &x->mm;

	if (m->win_on && ng_rm_present(x))
		ng_wr(x, NICOGPU_BAR_CTRL, RM_C3_WIN_CTRL, 0);	/* BAR2 back to identity */
	m->win_on = false;
	mutex_lock(&m->lock);			/* nicogpu_gem_free() of a BO that outlived the device looks at vram_bits */
	kvfree(m->vram_bits);
	kvfree(m->pt_bits);
	m->vram_bits = m->pt_bits = NULL;
	mutex_unlock(&m->lock);
	if (m->bar2wc)
		iounmap(m->bar2wc);
	m->bar2wc = NULL;
}
