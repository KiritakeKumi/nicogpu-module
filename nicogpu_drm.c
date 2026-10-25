// SPDX-License-Identifier: GPL-2.0
// nicogpu_drm.c -- the DRM render node of the nicogpu driver (was the misc device /dev/nicogpuN, nicogpu_fops.c).
//
//   * struct drm_driver: DRIVER_GEM | DRIVER_RENDER | DRIVER_SYNCOBJ | DRIVER_SYNCOBJ_TIMELINE (the DRM core
//     syncobj ioctls incl. timeline and sync_file import / export; SUBMIT takes drm_syncobj handles, nicogpu_fence.c is
//     the dma_fence adapter), per-open nicogpu_file in drm_file::driver_priv;
//   * the ioctl table: every ioctl of ABI v1..v3 keeps its number as DRM_COMMAND_BASE + DRM_NICOGPU_* (nicogpu_uapi.h);
//     the handlers live next to their state (nicogpu_mm.c, nicogpu_sched.c, nicogpu_dfx.c);
//   * the debug facilities of the first skeleton that must not be reachable by every user of a render node: BAR mmaps,
//     the DMA buffer, PTEATTR, KTEST -- DRM_ROOT_ONLY ioctls / CAP_SYS_ADMIN mmaps;
//   * file operations: drm_gem_mmap() for BO mappings (DRM fake offsets >= 4 GiB), the debug BAR/DMA mappings below.
// ABI in nicogpu_uapi.h.
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/capability.h>
#include <drm/drm_vma_manager.h>
#include "nicogpu_drv.h"

#define DMABUF_MAX (4UL << 20)

static struct nicogpu_file *file_xf(struct drm_file *f)
{
	return f->driver_priv;
}

static int nicogpu_drm_open(struct drm_device *dev, struct drm_file *file)
{
	struct nicogpu_dev *x = to_nicogpu(dev);
	struct nicogpu_file *xf;
	int rc;

	xf = kzalloc(sizeof(*xf), GFP_KERNEL);
	if (!xf)
		return -ENOMEM;
	xf->x = x;
	xf->file = file;
	rc = nicogpu_mm_open(xf);
	if (rc) {
		kfree(xf);
		return rc;
	}
	file->driver_priv = xf;
	return 0;
}

/* ---- DMA buffer (debug): one at a time, owned by the allocating file ---- */
static void dbuf_free_locked(struct nicogpu_dev *x)
{
	if (x->dbuf_cpu)
		dma_free_coherent(&x->pdev->dev, x->dbuf_size, x->dbuf_cpu, x->dbuf_iova);
	x->dbuf_cpu = NULL;
	x->dbuf_size = 0;
	x->dbuf_iova = 0;
	x->dbuf_owner = NULL;
}

void nicogpu_dma_free(struct nicogpu_dev *x)
{
	mutex_lock(&x->dbuf_lock);
	dbuf_free_locked(x);
	mutex_unlock(&x->dbuf_lock);
}

static long ioc_dmabuf(struct nicogpu_file *xf, void *up)
{
	struct nicogpu_dev *x = xf->x;
	struct nicogpu_dmabuf a;
	long rc = 0;

	if (nicogpu_arg_in(&a, up, sizeof(a)))
		return -EFAULT;
	if (!a.size || a.size > DMABUF_MAX)
		return -EINVAL;
	mutex_lock(&x->dbuf_lock);
	if (x->dbuf_cpu) {
		rc = -EBUSY;
		goto out;
	}
	x->dbuf_size = PAGE_ALIGN(a.size);
	x->dbuf_cpu = dma_alloc_coherent(&x->pdev->dev, x->dbuf_size, &x->dbuf_iova, GFP_KERNEL);
	if (!x->dbuf_cpu) {
		x->dbuf_size = 0;
		rc = -ENOMEM;
		goto out;
	}
	x->dbuf_owner = xf->file;
	a.size = x->dbuf_size;
	a.iova = x->dbuf_iova;
	dev_info(&x->pdev->dev, "DMABUF size=%zu iova=0x%llx\n", x->dbuf_size, (u64)x->dbuf_iova);
	nicogpu_arg_out(up, &a, sizeof(a));
out:
	mutex_unlock(&x->dbuf_lock);
	return rc;
}

static long ioc_dmafree(struct nicogpu_file *xf, void *up)
{
	struct nicogpu_dev *x = xf->x;
	long rc = 0;

	mutex_lock(&x->dbuf_lock);
	if (!x->dbuf_cpu)
		rc = -ENOENT;
	else if (x->dbuf_owner != xf->file)
		rc = -EPERM;
	else
		dbuf_free_locked(x);
	mutex_unlock(&x->dbuf_lock);
	return rc;
}

/* Last close of a file (drm_file_free(), after drm_gem_release() dropped its GEM handles).  BOs and VMs first (a file
 * torn down by remove() skips the device); the DMA buffer of a removed device is freed by remove() itself. */
static void nicogpu_drm_postclose(struct drm_device *dev, struct drm_file *file)
{
	struct nicogpu_file *xf = file_xf(file);
	struct nicogpu_dev *x = xf->x;
	bool gone = READ_ONCE(xf->torn);	/* only remove() sets it before the close */

	nicogpu_mm_release(xf);
	if (!gone) {
		mutex_lock(&x->dbuf_lock);
		if (x->dbuf_cpu && x->dbuf_owner == file)
			dbuf_free_locked(x);
		mutex_unlock(&x->dbuf_lock);
	}
	kfree(xf);
}

/* ---- mmap ---- */
static int map_dma(struct nicogpu_dev *x, struct drm_file *df, struct vm_area_struct *vma)
{
	unsigned long len = vma->vm_end - vma->vm_start;
	int rc;

	mutex_lock(&x->dbuf_lock);
	if (!x->dbuf_cpu || x->dbuf_owner != df || len > x->dbuf_size) {
		rc = -EINVAL;
	} else {
		vma->vm_pgoff = 0;	/* dma_mmap_coherent: vm_pgoff is an offset into the buffer */
		rc = dma_mmap_coherent(&x->pdev->dev, vma, x->dbuf_cpu, x->dbuf_iova, x->dbuf_size);
	}
	mutex_unlock(&x->dbuf_lock);
	return rc;
}

/* Debug mappings of the device node: raw BAR0/BAR1/BAR2 windows (with the BAR2 top reserved for the C3 window) and the
 * NICOGPU_IOC_DMABUF buffer.  Offsets below 4 GiB; anything at or above is a GEM fake offset (drm_gem_mmap). */
static int nicogpu_debug_mmap(struct file *f, struct vm_area_struct *vma)
{
	struct drm_file *df = f->private_data;
	struct nicogpu_dev *x = file_xf(df)->x;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long len = vma->vm_end - vma->vm_start;
	bool wc = off & NICOGPU_MMAP_WC;
	int b, rc, idx;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;		/* raw register / DMA windows: not for a render-node client */
	if (!drm_dev_enter(&x->drm, &idx))
		return -ENODEV;
	if (off & NICOGPU_MMAP_DMA) {
		rc = map_dma(x, df, vma);
		goto out;
	}
	off &= ~NICOGPU_MMAP_WC;
	if (off >= NICOGPU_MMAP_BAR2)
		b = NICOGPU_BAR_MEM, off -= NICOGPU_MMAP_BAR2;
	else if (off >= NICOGPU_MMAP_BAR1)
		b = NICOGPU_BAR_SHELL, off -= NICOGPU_MMAP_BAR1;
	else
		b = NICOGPU_BAR_CTRL;
	if (off + len > pci_resource_len(x->pdev, b) ||
	    (b == NICOGPU_BAR_MEM && !nicogpu_mm_bar2_user_ok(x, off, len))) {
		rc = -EINVAL;
		goto out;
	}
	rc = io_remap_pfn_range(vma, vma->vm_start,
				(pci_resource_start(x->pdev, b) + off) >> PAGE_SHIFT, len,
				wc ? pgprot_writecombine(vma->vm_page_prot)
				   : pgprot_noncached(vma->vm_page_prot));
out:
	drm_dev_exit(idx);
	return rc;
}

static int nicogpu_drm_mmap(struct file *f, struct vm_area_struct *vma)
{
	if (vma->vm_pgoff >= DRM_FILE_PAGE_OFFSET_START)
		return drm_gem_mmap(f, vma);	/* a BO: access check per file, funcs->mmap = nicogpu_gem_mmap() */
	return nicogpu_debug_mmap(f, vma);
}

/* ---- debug ioctls (DRM_ROOT_ONLY) ---- */
static long ioc_pteattr(struct nicogpu_file *xf, void *up)
{
	struct nicogpu_pteattr a;
	struct follow_pfnmap_args args = { };
	struct mm_struct *mm = current->mm;
	int rc;

	if (nicogpu_arg_in(&a, up, sizeof(a)))
		return -EFAULT;
	mmap_read_lock(mm);
	args.vma = vma_lookup(mm, a.va);
	args.address = a.va;
	rc = args.vma ? follow_pfnmap_start(&args) : -EFAULT;
	if (!rc) {
		a.pfn = args.pfn;
		a.prot = pgprot_val(args.pgprot);
		follow_pfnmap_end(&args);
	}
	mmap_read_unlock(mm);
	if (rc)
		return rc;
#ifdef CONFIG_ARM64
	a.attrindx = (a.prot & PTE_ATTRINDX_MASK) >> 2;
	a.mair = (read_sysreg(mair_el1) >> (8 * a.attrindx)) & 0xFF;
#else
	a.attrindx = 0xFF;
	a.mair = 0xFF;
#endif
	return nicogpu_arg_out(up, &a, sizeof(a));
}

static u32 lane_expect(u32 off, u32 len, const unsigned char *pat)
{
	u32 v = NICOGPU_SCRATCH_INIT, b;

	for (b = 0; b < 4; b++) {
		u32 a = RM_SCRATCH + b;

		if (a >= off && a < off + len)
			v = (v & ~(0xFFu << (8 * b))) | ((u32)pat[a - off] << (8 * b));
	}
	return v;
}

// Driver rule: ioremap_wc + memcpy_toio/memset_io.
static long ioc_ktest(struct nicogpu_file *xf, void *up)
{
	static const unsigned int lens[] = { 1, 2, 3, 4, 5, 7, 8, 15, 64, 4095 };
	struct nicogpu_dev *x = xf->x;
	struct nicogpu_ktest t = { };
	void __iomem *wc2, *wc0;
	unsigned char *src;
	unsigned int off, i, len;

	if (xg_rd(x, NICOGPU_BAR_SHELL, RM_STATUS) & RM_ST_DECOUPLED)
		return -EAGAIN;
	src = kmalloc(8192, GFP_KERNEL);
	wc2 = ioremap_wc(pci_resource_start(x->pdev, NICOGPU_BAR_MEM), 16384);
	wc0 = ioremap_wc(pci_resource_start(x->pdev, NICOGPU_BAR_CTRL), PAGE_SIZE);
	if (!src || !wc2 || !wc0) {
		if (wc0)
			iounmap(wc0);
		if (wc2)
			iounmap(wc2);
		kfree(src);
		return -ENOMEM;
	}
	for (i = 0; i < 8192; i++)
		src[i] = (unsigned char)(i * 37 + 11) | 0x80;

	for (off = 0; off < 16; off++)
		for (i = 0; i < ARRAY_SIZE(lens); i++) {
			memcpy_toio(wc2 + off, src + ((off * 5) & 15), lens[i]);
			memset_io(wc2 + 4096 + off, 0, lens[i]);
			t.bar2_ops += 2;
		}
	wmb();

	for (off = NICOGPU_LANE_LO; off < NICOGPU_LANE_HI; off++)
		for (len = 1; off + len <= NICOGPU_LANE_HI; len++) {
			const unsigned char *pat = src + t.lane_cases % 4096;
			u32 got, want = lane_expect(off, len, pat);

			xg_wr(x, NICOGPU_BAR_CTRL, RM_SCRATCH, NICOGPU_SCRATCH_INIT);
			memcpy_toio(wc0 + off, pat, len);
			wmb();
			got = xg_rd(x, NICOGPU_BAR_CTRL, RM_SCRATCH);
			if (got != want && t.lane_fail++ == 0) {
				t.first_off = off;
				t.first_len = len;
				t.first_got = got;
				t.first_want = want;
			}
			t.lane_cases++;
		}

	iounmap(wc0);
	iounmap(wc2);
	kfree(src);
	dev_info(&x->pdev->dev, "KTEST bar2_ops=%u lane_cases=%u lane_fail=%u\n",
		 t.bar2_ops, t.lane_cases, t.lane_fail);
	return nicogpu_arg_out(up, &t, sizeof(t));
}

static long ioc_get_info(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_dev *x = xf->x;
	const struct nicogpu_info *in = data;
	struct nicogpu_info i = { };
	int k;

	if (in->struct_size != sizeof(i) || (in->flags & ~NICOGPU_INFO_CLEAR_LATCHED))
		return -EINVAL;
	/* rsvd1 must be zero in */
	for (k = 0; k < 4; k++)
		if (in->rsvd1[k])
			return -EINVAL;
	i.struct_size = in->struct_size;
	i.flags = in->flags;
	nicogpu_fill_info(x, &i);
	if (i.flags & NICOGPU_INFO_CLEAR_LATCHED) {
		atomic_andnot(i.irq_shell_latched, &x->irq_shell_latched);
		atomic_andnot(i.irq_rm_latched, &x->irq_rm_latched);
	}
	return nicogpu_arg_out(data, &i, sizeof(i));
}

/* ---- ABI v5: GET_GPU_INFO / GET_TIMESTAMP ---- */

static_assert(NICOGPU_GPU_INFO_F_RM_CAPS == XGI_F_RM_CAPS && NICOGPU_GPU_INFO_F_RM_REV == XGI_F_RM_REV &&
	      NICOGPU_GPU_INFO_F_TS_RM == XGI_F_TS_RM && NICOGPU_GPU_INFO_F_TS_SHELL == XGI_F_TS_SHELL && NICOGPU_GPU_INFO_F_GTT_UNCACHED == XGI_F_GTT_UNCACHED &&
	      NICOGPU_GPU_INFO_F_NO_VRAM == XGI_F_NO_VRAM && NICOGPU_GPU_INFO_F_SP_UNITS == XGI_F_SP_UNITS &&
	      NICOGPU_SP_UNIT_SFU == XGI_SPU_SFU && NICOGPU_SP_UNIT_XLANE == XGI_SPU_XLANE && NICOGPU_SP_UNIT_FP64 == XGI_SPU_FP64 &&
	      NICOGPU_SP_UNIT_F16CVT == XGI_SPU_F16CVT,
	      "GPU info flags");

#define NICOGPU_FEATURES_V6 (NICOGPU_FEAT_GPU_INFO | NICOGPU_FEAT_TIMESTAMP | NICOGPU_FEAT_SYNCOBJ_WAIT_AVAILABLE | \
			  NICOGPU_FEAT_SYNCOBJ_SIGNAL | NICOGPU_FEAT_DEV_STATUS | NICOGPU_FEAT_QUEUE_FAULT | \
			  NICOGPU_FEAT_PR_LOAD | NICOGPU_FEAT_RESET | NICOGPU_FEAT_SYNCOBJ_TIMELINE | NICOGPU_FEAT_PRIME | NICOGPU_FEAT_DRM_SYNCOBJ | \
			  NICOGPU_FEAT_SYNC_FILE)

/* The RM's own register map (CAPS / REV / TIMESTAMP_*) is only there when the RM is coupled and identifies as the GPU
 * RM: the blank / test RMs of the board have other registers at these offsets (the register map is PROVISIONAL). */
static bool rm_regs_ok(struct nicogpu_dev *x)
{
	if (xg_rd(x, NICOGPU_BAR_SHELL, RM_STATUS) & RM_ST_DECOUPLED)
		return false;
	return xg_rd(x, NICOGPU_BAR_CTRL, RM_ID) == RM_ID_MAGIC;
}

static u32 shell_ts_khz(struct nicogpu_dev *x)
{
	return (xg_rd(x, NICOGPU_BAR_SHELL, SH_VERSION) & 0xFFFF) >= 2 ? xg_rd(x, NICOGPU_BAR_SHELL, CLK_FREQ_TS) : 0;
}

static long ioc_get_gpu_info(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_dev *x = xf->x;
	const struct nicogpu_gpu_info *in = data;
	struct nicogpu_gpu_info i = { };
	struct xgi_hw hw = { };
	struct xgi_info o;
	int k;

	if (in->struct_size != sizeof(i) || in->flags)
		return -EINVAL;
	if (in->sp_units || in->pad)
		return -EINVAL;
	for (k = 0; k < 3; k++)
		if (in->rsvd[k])
			return -EINVAL;
	hw.rm_regs = rm_regs_ok(x);
	if (hw.rm_regs) {
		hw.rm_rev = xg_rd(x, NICOGPU_BAR_CTRL, RM_REV);
		hw.rm_caps = xg_rd(x, NICOGPU_BAR_CTRL, RM_CAPS);
		hw.rm_ts_freq_hz = xg_rd(x, NICOGPU_BAR_CTRL, RM_TIMESTAMP_FREQ);
	}
	hw.rm_kind = nicogpu_rm_kind(x);
	hw.shell_ts_khz = shell_ts_khz(x);
	nicogpu_mm_fill_hw(x, &hw);
	xgi_build(&hw, &o);

	i.struct_size = sizeof(i);
	i.abi_version = NICOGPU_ABI_VERSION;
	i.info_flags = o.info_flags;
	i.sp_units = o.sp_units;
	i.features = NICOGPU_FEATURES_V6;
	i.vendor_id = x->pdev->vendor;
	i.device_id = x->pdev->device;
	i.revision = o.revision;
	i.num_cores = o.num_cores;
	i.sp_per_core = o.sp_per_core;
	i.warp_size = o.warp_size;
	i.warp_slots_per_sp = o.warp_slots_per_sp;
	i.max_vregs = o.max_vregs;
	i.num_uregs = o.num_uregs;
	i.max_workgroup_threads = o.max_workgroup_threads;
	i.max_shared_bytes = o.max_shared_bytes;
	i.max_user_uregs = o.max_user_uregs;
	i.va_bits = o.va_bits;
	i.page_sizes = o.page_sizes;	/* XGI and NICOGPU_GPU_PAGE_* use the HAL bit values */
	i.max_vms = o.max_vms;
	i.max_submit_cmdbufs = NICOGPU_SUBMIT_MAX_CMDBUFS;
	i.max_submit_syncs = NICOGPU_SUBMIT_MAX_SYNCS;
	i.max_bind_ops = NICOGPU_VM_BIND_MAX_OPS;
	i.va_user_start = o.va_user_start;
	i.va_user_end = o.va_user_end;
	i.vram_size = o.vram_size;
	i.vram_visible_size = o.vram_visible_size;
	i.gtt_size = o.gtt_size;
	i.gtt_bo_max = o.gtt_bo_max;
	i.vram_used = o.vram_used;
	i.gtt_used = o.gtt_used;
	i.timestamp_freq_hz = o.timestamp_freq_hz;
	return nicogpu_arg_out(data, &i, sizeof(i));
}

static long ioc_get_timestamp(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_dev *x = xf->x;
	const struct nicogpu_timestamp *in = data;
	struct nicogpu_timestamp t = { };
	struct xgi_hw hw = { };
	u32 lo, hi, src;
	bool raw;
	long rc = 0;

	if (in->flags || in->rsvd[0] || in->rsvd[1] || (in->clock_id != 0 && in->clock_id != 1 && in->clock_id != 4))
		return -EINVAL;
	raw = in->clock_id == 4;
	mutex_lock(&x->ts_lock);
	if (!rm_regs_ok(x)) {
		rc = -ENODEV;
		goto out;
	}
	t.cpu_ns_before = raw ? ktime_get_raw_ns() : ktime_get_ns();
	lo = xg_rd(x, NICOGPU_BAR_CTRL, RM_TIMESTAMP_LO);	/* latches HI */
	hi = xg_rd(x, NICOGPU_BAR_CTRL, RM_TIMESTAMP_HI);
	hw.rm_regs = true;
	hw.rm_ts_freq_hz = xg_rd(x, NICOGPU_BAR_CTRL, RM_TIMESTAMP_FREQ);
	t.cpu_ns_after = raw ? ktime_get_raw_ns() : ktime_get_ns();
	/* the RM was isolated / replaced while we read (all-ones, or DECOUPLED now): the pair is not a timestamp */
	if ((lo == 0xFFFFFFFFu && hi == 0xFFFFFFFFu) || (xg_rd(x, NICOGPU_BAR_SHELL, RM_STATUS) & RM_ST_DECOUPLED)) {
		rc = -EAGAIN;
		goto out;
	}
	hw.shell_ts_khz = shell_ts_khz(x);
	t.ts = xgi_ts_join(lo, hi);
	t.freq_hz = xgi_ts_freq(&hw, &src);
out:
	mutex_unlock(&x->ts_lock);
	if (rc)
		return rc;
	return nicogpu_arg_out(data, &t, sizeof(t));
}

/* ---- the ioctl table ---- */

/* Every handler runs between drm_dev_enter() and drm_dev_exit(): remove() calls drm_dev_unplug(), which waits for the
 * ones in flight and makes later calls fail with -ENODEV before the device state is torn down. */
#define XG_IOCTL(name, handler)								\
static int nicogpu_drm_##name(struct drm_device *dev, void *data, struct drm_file *file)	\
{											\
	int idx;									\
	long rc;									\
											\
	if (!drm_dev_enter(dev, &idx))							\
		return -ENODEV;								\
	rc = handler(file_xf(file), data);						\
	drm_dev_exit(idx);								\
	return rc;									\
}

XG_IOCTL(PTEATTR, ioc_pteattr)
XG_IOCTL(KTEST, ioc_ktest)
XG_IOCTL(DMABUF, ioc_dmabuf)
XG_IOCTL(DMAFREE, ioc_dmafree)
XG_IOCTL(GET_INFO, ioc_get_info)
XG_IOCTL(GET_GPU_INFO, ioc_get_gpu_info)
XG_IOCTL(GET_TIMESTAMP, ioc_get_timestamp)
XG_IOCTL(GET_DEV_STATUS, nicogpu_ioc_dev_status)
XG_IOCTL(BO_CREATE, nicogpu_ioc_bo_create)
XG_IOCTL(BO_FREE, nicogpu_ioc_bo_free)
XG_IOCTL(VM_CREATE, nicogpu_ioc_vm_create)
XG_IOCTL(VM_DESTROY, nicogpu_ioc_vm_destroy)
XG_IOCTL(VM_BIND, nicogpu_ioc_vm_bind)
XG_IOCTL(VM_LOOKUP, nicogpu_ioc_vm_lookup)
XG_IOCTL(QUEUE_CREATE, nicogpu_ioc_queue_create)
XG_IOCTL(QUEUE_DESTROY, nicogpu_ioc_queue_destroy)
XG_IOCTL(QUEUE_STATUS, nicogpu_ioc_queue_status)
XG_IOCTL(SYNCOBJ_CREATE, nicogpu_ioc_syncobj_create)
XG_IOCTL(SYNCOBJ_DESTROY, nicogpu_ioc_syncobj_destroy)
XG_IOCTL(SYNCOBJ_RESET, nicogpu_ioc_syncobj_reset)
XG_IOCTL(SYNCOBJ_SIGNAL, nicogpu_ioc_syncobj_signal)
XG_IOCTL(SYNCOBJ_WAIT, nicogpu_ioc_syncobj_wait)
XG_IOCTL(SUBMIT, nicogpu_ioc_submit)
XG_IOCTL(FENCE_WAIT, nicogpu_ioc_fence_wait)
XG_IOCTL(QUEUE_FAULT, nicogpu_ioc_queue_fault)
XG_IOCTL(PR_LOAD, nicogpu_ioc_pr_load)
XG_IOCTL(RESET, nicogpu_ioc_reset)

/* Render nodes only run ioctls marked DRM_RENDER_ALLOW.  Debug ones add DRM_ROOT_ONLY (CAP_SYS_ADMIN, -EACCES);
 * VM_LOOKUP / PR_LOAD / RESET check CAP_SYS_ADMIN themselves (-EPERM, the ABI v3 behaviour). */
#define XG_DEF(name, flags) DRM_IOCTL_DEF_DRV(NICOGPU_##name, nicogpu_drm_##name, DRM_RENDER_ALLOW | (flags))

static const struct drm_ioctl_desc nicogpu_ioctls[] = {
	XG_DEF(PTEATTR, DRM_ROOT_ONLY),
	XG_DEF(KTEST, DRM_ROOT_ONLY),
	XG_DEF(DMABUF, DRM_ROOT_ONLY),
	XG_DEF(DMAFREE, DRM_ROOT_ONLY),
	XG_DEF(GET_INFO, 0),
	XG_DEF(GET_GPU_INFO, 0),
	XG_DEF(GET_TIMESTAMP, 0),
	XG_DEF(GET_DEV_STATUS, 0),
	XG_DEF(BO_CREATE, 0),
	XG_DEF(BO_FREE, 0),
	XG_DEF(VM_CREATE, 0),
	XG_DEF(VM_DESTROY, 0),
	XG_DEF(VM_BIND, 0),
	XG_DEF(VM_LOOKUP, 0),
	XG_DEF(QUEUE_CREATE, 0),
	XG_DEF(QUEUE_DESTROY, 0),
	XG_DEF(QUEUE_STATUS, 0),
	XG_DEF(SYNCOBJ_CREATE, 0),
	XG_DEF(SYNCOBJ_DESTROY, 0),
	XG_DEF(SYNCOBJ_RESET, 0),
	XG_DEF(SYNCOBJ_SIGNAL, 0),
	XG_DEF(SYNCOBJ_WAIT, 0),
	XG_DEF(SUBMIT, 0),
	XG_DEF(FENCE_WAIT, 0),
	XG_DEF(QUEUE_FAULT, 0),
	XG_DEF(PR_LOAD, 0),
	XG_DEF(RESET, 0),
};

static const struct file_operations nicogpu_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.compat_ioctl = drm_compat_ioctl,
	.poll = drm_poll,
	.read = drm_read,
	.llseek = noop_llseek,
	.mmap = nicogpu_drm_mmap,
#ifdef FOP_UNSIGNED_OFFSET
	.fop_flags = FOP_UNSIGNED_OFFSET,	/* as DEFINE_DRM_GEM_FOPS: offsets above 2^63 are legal GEM offsets */
#endif
};

const struct drm_driver nicogpu_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_RENDER | DRIVER_SYNCOBJ | DRIVER_SYNCOBJ_TIMELINE,
	.open = nicogpu_drm_open,
	.postclose = nicogpu_drm_postclose,
	.ioctls = nicogpu_ioctls,
	.num_ioctls = ARRAY_SIZE(nicogpu_ioctls),
	.fops = &nicogpu_fops,
	.name = "nicogpu",
	.desc = "NicoWorks Heathcliff GPU (render node)",
};
