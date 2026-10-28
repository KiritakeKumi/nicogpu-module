/* SPDX-License-Identifier: GPL-2.0 */
#ifndef NICOGPU_DRV_H
#define NICOGPU_DRV_H

#include <linux/pci.h>
#include <linux/delay.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/xarray.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/dma-fence.h>
#include <linux/dma-fence-chain.h>
#include <drm/drm_syncobj.h>
#include "nicogpu_regs.h"
#include "nicogpu_uapi.h"
#include "nicogpu_vm_core.h"
#include "nicogpu_sched_core.h"
#include "nicogpu_dfx_core.h"
#include "nicogpu_dump_core.h"
#include "nicogpu_ginfo_core.h"
#include "nicogpu_fence.h"

/* Scheduler / synchronisation state (nicogpu_sched.c).  Initialised at
 * probe, brought up (CP handshake, ring) lazily by the first queue/syncobj
 * ioctl.  Lock order: file->lock -> sched.lock (spinlock, IRQ-safe, leaf);
 * mm.bind_lock is taken (inside the sleeping ring_write op, by the worker) with
 * no other lock held. */
struct nicogpu_sched {
	struct xsc_core core;
	struct nicogpu_dev *x;
	spinlock_t lock;		/* the core's lock: ops->lock/unlock */
	unsigned long irqflags;		/* valid while `lock` is held */
	wait_queue_head_t wq;		/* every waiter: woken by ops->notify */
	struct mutex init_lock;		/* serialises nicogpu_sched_ensure */
	bool ready;			/* CP up, ring configured, work items live */
	struct workqueue_struct *wq_run;	/* ordered: run / timeout / CP error never overlap */
	struct work_struct work;	/* push jobs, handle CP error events */
	struct delayed_work dwork;	/* watchdog */
	atomic_t events;		/* IRQ_STATUS bits for the worker */
	void *page_cpu;			/* host page: fence word at +0, RPTR write-back at +0x40 */
	dma_addr_t page_dma;
	struct xsc_c3 c3;		/* C3 [0, 16 MiB) front-region layout */
	u64 ring_off, fw_off;		/* C3 byte offsets (fw_off: the firmware image) */
	u64 fw_pa;			/* staged firmware image: CP_FW_BASE (0 = not staged yet) */
	u32 fw_size;
	struct mutex reset_lock;	/* serialises nicogpu_sched_reset() callers */
	struct completion reset_done;
	u32 reset_level;		/* request for the worker (NICOGPU_RESET_*), 0 = none */
	unsigned long io_next;		/* jiffies: earliest next I/O-recovery attempt */
	int reset_rc;
	struct xdd_limiter dump_lim;	/* devcoredump trigger limiter */
	struct nicogpu_fences fences;	/* dma_fence adapter (valid while `ready`) */
};

/* DFX boundary state (nicogpu_dfx.c): GPU-zone reset and PR share one core and one lock. */
struct nicogpu_dfx {
	struct xdf core;
	struct mutex lock;		/* serialises xdf_gpu_reset() / xdf_pr_load */
	struct mutex pr_mutex;		/* one PR / debugfs load at a time */
	bool inited;			/* xdf_init() done (reads SHELL_UID once) */
	u8 *good;			/* known-good NGPR container (vmalloc copy), the last one that loaded */
	size_t good_len;
	const struct firmware *fallback;	/* the blank-RM container while a rollback may need it */
	bool dump_pending;		/* a flow hit something worth a devcoredump (taken after the flow) */
	enum xdd_reason dump_reason;
};

/* Device-wide memory manager state (nicogpu_mm.c).
 * Lock order: file->vm_lock -> mm.bind_lock -> mm.lock (leaf; never held across
 * MMIO or allocation of anything but pool bits). */
struct nicogpu_mm {
	struct mutex lock;		/* pools, vmid_map, gtt_used */
	struct mutex bind_lock;		/* serialises every access through the C3 window */
	bool mmu_possible;		/* BAR2 is 256 MiB: the layout below is valid */
	bool win_on;			/* C3 window enabled by this driver (sticky until fini) */
	bool io_err;			/* sticky: a window slide was rejected by the RM */
	bool dirty;			/* page-table stores not yet pushed by a read-back */
	bool pr_busy;			/* a PR is in progress or about to start: no new VM; under `lock` */
	u64 ctx_root[NICOGPU_NUM_VMIDS];	/* MMU context roots of the VMs that are programmed into the RM (0 = none); bind_lock */
	void __iomem *bar2wc;		/* ioremap_wc of BAR2: VRAM heap + C3 window */
	void __iomem *c3win;		/* bar2wc + (256 MiB - window size) */
	void __iomem *last_wr;		/* last page-table location written (read-back target) */
	u64 win_base;			/* C3 byte offset the window currently shows */
	u64 vram_visible;		/* bytes of VRAM the heap may use: 256 MiB - window */
	struct xvc_pool vram;		/* VRAM heap: [0, vram_visible) (PROVISIONAL) */
	u64 *vram_bits;
	struct xvc_pool pt;		/* page-table pages: C3 [PT_OFF, PT_END) */
	u64 *pt_bits;
	u64 gtt_used;			/* bytes of live GTT BOs */
	unsigned long vmid_map;		/* bit n set: VMID n in use (bit 0 reserved) */
};

/* Record of the most recent device loss (GET_DEV_STATUS).  Written by the scheduler worker (reset ladder
 * exhausted) and by the DFX flow (PR wedged), read by the ioctl; `lock` is a leaf. */
struct nicogpu_lostrec {
	spinlock_t lock;
	u32 reason;			/* NICOGPU_LOST_* of the latest loss, 0 = never */
	s32 err;			/* negative errno that ended the flow */
	u64 ns;				/* ktime_get_ns() at that moment */
	u32 count;			/* losses so far */
};

struct nicogpu_dev {
	struct drm_device drm;		/* first: devm_drm_dev_alloc(); the render node */
	struct pci_dev *pdev;
	void __iomem *bar[NICOGPU_NUM_BARS];
	bool bar_lock;			/* module param bar_lock (default 0 since shell 20261014): take bar0_lock around BAR0/BAR1 accesses */
	spinlock_t bar0_lock;		/* BAR0 (RM registers) accesses are serialised, see ng_rd */
	int nvec;
	struct dentry *dbg;

	/* remove() vs file ops: drm_dev_enter()/drm_dev_exit() around every ioctl / mmap, drm_dev_unplug() in remove()
	 * waits for the ones in flight and fails new ones with -ENODEV (replaces the misc-era alive_sem). */
	struct address_space *mapping;	/* of the device node (drm anon inode): revoked before a PR and at remove() */

	/* interrupt bookkeeping (merged vector: handler reads both status regs) */
	atomic64_t irq_count;
	atomic_t irq_shell_latched, irq_rm_latched;

	/* one DMA buffer at a time */
	struct mutex dbuf_lock;
	void *dbuf_cpu;
	dma_addr_t dbuf_iova;
	size_t dbuf_size;
	struct drm_file *dbuf_owner;

	struct nicogpu_mm mm;
	struct nicogpu_sched sched;
	struct nicogpu_dfx dfx;
	struct nicogpu_lostrec lost;
	struct mutex ts_lock;		/* TIMESTAMP_LO/HI is a latch pair, one reader at a time */
};

/* Per open file (drm_file::driver_priv): BO handles (GEM handles of the drm_file) and VMs are
 * file-local. */
struct nicogpu_vm;

/* A queue as the file sees it: the core queue plus its dma_fence timeline. */
struct nicogpu_queue {
	struct xsc_queue *q;
	struct nicogpu_fctx *fctx;
};

/*
 * Per-file locking.  VM state and queue state have separate locks so that a VM_BIND, which writes page tables through
 * the C3 window and waits for the TLB invalidation (up to NICOGPU_INVAL_TIMEOUT_MS), never stalls a SUBMIT of the same
 * file:
 *   vm_lock  the VMs of the file: VM_CREATE / VM_DESTROY / VM_BIND (incl. nicogpu_vm.pin) / VM_LOOKUP and the VM part
 *            of the file teardown.  Sleeps across MMIO, takes mm.bind_lock (-> mm.lock) inside.
 *   lock     queues, syncobjs, SUBMIT (incl. the per-queue dma_fence seqno), the draining-queue counts, and the
 *            publication of vms[]: an entry is only written with BOTH locks held, so either one is enough to read it
 *            (QUEUE_CREATE checks its VM under `lock` alone).  Never held across page-table MMIO or a TLB wait.
 * Lock order: vm_lock -> lock -> sched.lock / fence locks;  vm_lock -> mm.bind_lock -> mm.lock.
 * Nothing takes vm_lock while holding `lock`.  The paths that hold both, always in that order and only briefly:
 * VM_CREATE (publish), VM_DESTROY (in-use check + unpublish) and file_teardown (unpublish); the slow part of a VM
 * destroy (context disable, TLB wait, table free) runs after `lock` is dropped, on a VM nobody can find any more.
 * A VM cannot go away under a queue: VM_DESTROY fails with -EBUSY while a queue of the file is bound to it, including
 * a queue that QUEUE_DESTROY already unlinked but is still draining (vm_draining[]); SUBMIT only uses the queue's VMID.
 */
struct nicogpu_file {
	struct nicogpu_dev *x;
	struct drm_file *file;		/* the owning DRM file: GEM handle table */
	struct list_head node;		/* global open-file list (nicogpu_mm.c) */
	bool torn;			/* resources already released by remove */
	struct mutex vm_lock;		/* VMs and every VM_BIND of this file (see above) */
	struct mutex lock;		/* queues, syncobjs, submits (see above) */
	struct nicogpu_vm *vms[NICOGPU_NUM_VMIDS];	/* indexed by VMID; written under vm_lock + lock */
	u32 vm_draining[NICOGPU_NUM_VMIDS];	/* queues of the VM unlinked by QUEUE_DESTROY, still draining; lock */
	struct xarray queues;		/* queue_id -> struct nicogpu_queue */
	struct xarray syncobjs;		/* handle -> struct xsc_syncobj (handle reference) */
};

#define to_nicogpu(d) container_of(d, struct nicogpu_dev, drm)

/* One BAR0/BAR1 register access at a time.  Board run 2026-10-13: a BAR0 read (C3_WIN_CTRL) intermittently returned
 * the data of a BAR1 read (RM_STATUS 0x172, once 0x170, once 0) when another CPU (ISR / scheduler worker) read a
 * BAR1 register at the same time: the two reads' completions get paired the wrong way round on the shell side (the
 * router passes BAR1 reads straight through while BAR0 reads cross the clock-domain converter; the XDMA bridge
 * matches m_axib responses to requests by order).  Serialising every register read/write in the driver removes the
 * overlap.  PROVISIONAL: the shell (ng_bar_router, static) should keep responses in request order itself. */
static inline bool ng_rm_id_ok(u32 v)
{
	return v == RM_ID_MAGIC || v == RM_ID_MAGIC_LEGACY;	/* "NGPU", or the pre-b20 "XGPU" */
}

static inline u32 ng_rd(struct nicogpu_dev *x, int bar, u32 off)
{
	unsigned long fl;
	u32 v;

	if (!x->bar_lock || (bar != NICOGPU_BAR_CTRL && bar != NICOGPU_BAR_SHELL))
		return ioread32(x->bar[bar] + off);
	spin_lock_irqsave(&x->bar0_lock, fl);
	v = ioread32(x->bar[bar] + off);
	spin_unlock_irqrestore(&x->bar0_lock, fl);
	return v;
}
static inline void ng_wr(struct nicogpu_dev *x, int bar, u32 off, u32 v)
{
	unsigned long fl;

	if (!x->bar_lock || (bar != NICOGPU_BAR_CTRL && bar != NICOGPU_BAR_SHELL)) {
		iowrite32(v, x->bar[bar] + off);
		return;
	}
	spin_lock_irqsave(&x->bar0_lock, fl);
	iowrite32(v, x->bar[bar] + off);
	spin_unlock_irqrestore(&x->bar0_lock, fl);
}

/* the shell has the RM coupled (not isolated / being reconfigured): RM registers and BAR2 are reachable */
static inline bool ng_rm_present(struct nicogpu_dev *x)
{
	return !(ng_rd(x, NICOGPU_BAR_SHELL, RM_STATUS) & RM_ST_DECOUPLED);
}

/* Callbacks shared by the core ops tables (struct xdf_ops, xdd_io, xsc_ops): ctx is the struct nicogpu_dev
 * (msleep ignores it) */
static inline u32 ng_op_shell_rd(void *ctx, u32 off)
{
	return ng_rd(ctx, NICOGPU_BAR_SHELL, off);
}

static inline u32 ng_op_rm_rd(void *ctx, u32 off)
{
	return ng_rd(ctx, NICOGPU_BAR_CTRL, off);
}

static inline void ng_op_msleep(void *ctx, u32 ms)
{
	msleep(ms);
}

void nicogpu_fill_info(struct nicogpu_dev *x, struct nicogpu_info *i);
void nicogpu_debugfs_init(struct nicogpu_dev *x);
void nicogpu_debugfs_fini(struct nicogpu_dev *x);
extern const struct drm_driver nicogpu_drm_driver;	/* nicogpu_drm.c: render node, ioctl table, fops, debug ioctls / mmaps */
void nicogpu_dma_free(struct nicogpu_dev *x);

/* nicogpu_mm.c */
int nicogpu_mm_init(struct nicogpu_dev *x);
void nicogpu_mm_fini(struct nicogpu_dev *x);
int nicogpu_mm_open(struct nicogpu_file *xf);
void nicogpu_mm_release(struct nicogpu_file *xf);
/* ioctl handlers (nicogpu_drm.c table): `data` is the kernel copy of the argument struct; drm_ioctl() copies it in
 * before and out after the handler, also when the handler fails */
long nicogpu_ioc_bo_create(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_bo_free(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_vm_create(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_vm_destroy(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_vm_bind(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_vm_lookup(struct nicogpu_file *xf, void *data);
bool nicogpu_mm_bar2_user_ok(struct nicogpu_dev *x, u64 off, u64 len);
/* Allocator numbers for GET_GPU_INFO: fills vram_heap_bytes, gtt_total_max, gtt_bo_max, vram_used, gtt_used
 * (live BO bytes, page rounded) and the VM limit of struct ngi_hw */
void nicogpu_mm_fill_hw(struct nicogpu_dev *x, struct ngi_hw *hw);
u32 nicogpu_rm_kind(struct nicogpu_dev *x);	/* RM_INFO1[31:16] of the loaded RM, 0 = unknown */
void nicogpu_mm_remove_files(struct nicogpu_dev *x);
/* C3 front region [0, 16 MiB) for the ring / firmware: enable the
 * window if needed (-ENODEV: no RM with the v0.2 window), write dwords at a
 * C3 byte offset (-EIO: window lost / slide rejected / RM decoupled). */
int nicogpu_mm_c3_open(struct nicogpu_dev *x);
int nicogpu_mm_c3_write(struct nicogpu_dev *x, u64 off, const u32 *dw, u32 n);
/* After the RM was reset behind our back (GPU-zone reset): re-open the C3 window and re-program the MMU
 * context of every live VM.  Sleeping; takes only mm.bind_lock. */
int nicogpu_mm_rm_restore(struct nicogpu_dev *x);
/* PR gate.  begin: -EBUSY while any VM exists, else no new VM can be created until end (VM_CREATE
 * returns -EBUSY).  end(rm_replaced): the RM may have changed: forget the C3 window state. */
int nicogpu_mm_pr_begin(struct nicogpu_dev *x);
void nicogpu_mm_pr_end(struct nicogpu_dev *x, bool rm_replaced);

/* nicogpu_sched.c */
void nicogpu_sched_init(struct nicogpu_dev *x);	/* probe: software state only */
void nicogpu_sched_fini(struct nicogpu_dev *x);	/* remove: stop the CP, free everything */
void nicogpu_sched_irq(struct nicogpu_dev *x, u32 rm_irq);	/* hard IRQ: RM IRQ_STATUS bits */
void nicogpu_sched_open(struct nicogpu_file *xf);
void nicogpu_sched_file_fini(struct nicogpu_file *xf);	/* queues drained, syncobjs dropped */
bool nicogpu_sched_vm_in_use(struct nicogpu_file *xf, u32 vmid);	/* a queue is bound to it, maybe draining (xf->lock held) */
long nicogpu_ioc_queue_create(struct nicogpu_file *xf, void *data);	/* ioctl handlers, same convention as the nicogpu_mm.c ones */
long nicogpu_ioc_queue_destroy(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_queue_status(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_syncobj_create(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_syncobj_destroy(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_syncobj_reset(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_syncobj_signal(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_syncobj_wait(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_submit(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_fence_wait(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_queue_fault(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_reset(struct nicogpu_file *xf, void *data);
long nicogpu_ioc_dev_status(struct nicogpu_file *xf, void *data);
void nicogpu_note_lost(struct nicogpu_dev *x, u32 reason, int err);	/* record a device loss (any context) */
struct seq_file;
void nicogpu_sched_debugfs_show(struct nicogpu_dev *x, struct seq_file *m);
/* Reset request executed by the scheduler worker (debugfs / NICOGPU_IOC_RESET): NICOGPU_RESET_* level;
 * 0 / -EIO (device lost) / -ENODEV (no CP) / -EBUSY (PR in progress). */
int nicogpu_sched_reset(struct nicogpu_dev *x, u32 level);
/* devcoredump (process context, sleeping): reason, optional DFX state */
void nicogpu_dump(struct nicogpu_dev *x, enum xdd_reason reason);

/* nicogpu_dfx.c */
void nicogpu_dfx_init(struct nicogpu_dev *x);
void nicogpu_dfx_fini(struct nicogpu_dev *x);
int nicogpu_dfx_gpu_reset(struct nicogpu_dev *x);	/* reset ladder level 2 (sched worker) */
long nicogpu_ioc_pr_load(struct nicogpu_file *xf, void *data);	/* NICOGPU_IOC_PR_LOAD */
int nicogpu_dfx_pr_load_name(struct nicogpu_dev *x, const char *name);	/* debugfs: request_firmware(name) */
void nicogpu_dfx_fill_dump(struct nicogpu_dev *x, struct xdd_dfx *out);
void nicogpu_dfx_icap_show(struct nicogpu_dev *x, struct seq_file *m);	/* debugfs icap */

#endif
