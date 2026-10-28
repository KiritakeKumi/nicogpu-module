/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* nicogpu_uapi.h -- PROVISIONAL ioctl/mmap ABI of the nicogpu DRM render node (/dev/dri/renderD*, ABI v6).
 * Linux ioctl numbering is left to the driver; since ABI v4 the driver is a DRM render node and every ioctl
 * is a DRM driver ioctl: request nr = DRM_COMMAND_BASE (0x40) + the DRM_NICOGPU_* index below, magic 'd' (the
 * DRM_IOCTL_BASE), direction/size encoded by DRM_IO/IOR/IOW/IOWR.  The DRM_NICOGPU_* indices are the old 'X' numbers of
 * ABI v1..v3 (0x01..0x10 skeleton / debug, 0x20.. memory management, queues, faults), so the
 * argument structs and their semantics did not change -- only the ioctl request codes and the mmap rules did (v4). */
#ifndef NICOGPU_UAPI_H
#define NICOGPU_UAPI_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <drm/drm.h>

#define NICOGPU_ABI_VERSION 6	/* 1: BO/VM ioctls; 2: queue/syncobj/submit/fence ioctls; 3: fault info / PR / reset
			 * (all additive); 4: DRM render node (BREAKING: ioctl request codes are DRM_COMMAND_BASE based, BO handles
			 * are GEM handles, BO mmap offsets are opaque DRM fake offsets, debug ioctls / BAR mmaps need CAP_SYS_ADMIN);
			 * 5: GET_GPU_INFO / GET_TIMESTAMP / GET_DEV_STATUS + SYNCOBJ_WAIT_AVAILABLE (pure additions: no existing
			 * ioctl number, struct or flag changed);
			 * 6: DRIVER_SYNCOBJ(+TIMELINE): the DRM core syncobj ioctls work on this node, SUBMIT waits / signals may name
			 * drm_syncobj handles (NICOGPU_SYNC_F_DRM, binary or timeline point); sync_file import / export come from the DRM core.
			 * Pure addition again: one new bit in nicogpu_sync.flags, new feature bits; the private SYNCOBJ_* ioctls are DEPRECATED
			 * but unchanged */

/* Debug mmap offsets of the device node (CAP_SYS_ADMIN only since ABI v4: they expose raw BARs / the debug DMA
 * buffer).  Byte offset passed to mmap; OR in _WC for write-combining.  All lie below 4 GiB; the DRM fake offsets of BOs
 * (nicogpu_bo_create.mmap_offset) start at 4 GiB. */
#define NICOGPU_MMAP_BAR0	0x0000000UL
#define NICOGPU_MMAP_BAR1	0x1000000UL
#define NICOGPU_MMAP_BAR2	0x2000000UL
#define NICOGPU_MMAP_DMA	0x20000000UL	/* buffer from NICOGPU_IOC_DMABUF (no _WC) */
#define NICOGPU_MMAP_WC	0x40000000UL

struct nicogpu_pteattr {		/* from p0test, unchanged layout */
	__u64 va;		/* in */
	__u64 pfn;		/* out */
	__u64 prot;		/* out */
	__u32 attrindx;		/* out: arm64 AttrIndx, 0xff elsewhere */
	__u32 mair;		/* out */
};

struct nicogpu_ktest {		/* from p0test, unchanged layout */
	__u32 bar2_ops, lane_cases, lane_fail;
	__u32 first_off, first_len, first_got, first_want;
	__u32 rsvd;
};

struct nicogpu_dmabuf {		/* from p0test, unchanged layout */
	__u64 size;		/* in: 1..4 MiB; out: page-rounded */
	__u64 iova;		/* out: PCIe bus address */
};

#define NICOGPU_INFO_CLEAR_LATCHED	(1u << 0)	/* in.flags: clear irq_*_latched after reading */

struct nicogpu_info {
	__u32 struct_size;	/* in: sizeof(struct nicogpu_info) as built by the caller */
	__u32 flags;		/* in: NICOGPU_INFO_* */
	__u32 abi_version;	/* out */
	__u32 msi_vectors;	/* out: vectors the kernel granted (1 or 2; shell merges 0|1) */
	__u32 shell_id, shell_version, shell_caps, shell_status;
	__u32 shell_uid_lo, shell_uid_hi;
	__u32 rm_status, rm_info[4];
	__u32 clk_freq_khz[4];	/* core, l2, ts, core2x */
	__u32 clk_status;
	__u32 rsvd0;
	__u64 bar_len[3];
	__u64 irq_count;	/* merged-vector interrupts handled */
	__u32 irq_shell_latched; /* OR of SHELL_IRQ_STATUS seen since last clear */
	__u32 irq_rm_latched;	/* OR of RM IRQ_STATUS seen since last clear */
	__u64 rsvd1[4];		/* must be zero in, zero out */
};

/* Driver ioctl indices (request nr = DRM_COMMAND_BASE + index; the old 'X' numbers of ABI v1..v3) */
#define DRM_NICOGPU_PTEATTR	0x01	/* debug, CAP_SYS_ADMIN */
#define DRM_NICOGPU_KTEST		0x02	/* debug, CAP_SYS_ADMIN */
#define DRM_NICOGPU_DMABUF		0x03	/* debug, CAP_SYS_ADMIN */
#define DRM_NICOGPU_DMAFREE	0x04	/* debug, CAP_SYS_ADMIN */
#define DRM_NICOGPU_GET_INFO	0x10
#define DRM_NICOGPU_GET_GPU_INFO	0x11	/* ABI v5 */
#define DRM_NICOGPU_GET_TIMESTAMP	0x12	/* ABI v5 */
#define DRM_NICOGPU_GET_DEV_STATUS	0x13	/* ABI v5 */
#define DRM_NICOGPU_BO_CREATE	0x20
#define DRM_NICOGPU_BO_FREE	0x21
#define DRM_NICOGPU_VM_CREATE	0x22
#define DRM_NICOGPU_VM_DESTROY	0x23
#define DRM_NICOGPU_VM_BIND	0x24
#define DRM_NICOGPU_VM_LOOKUP	0x25	/* debug, CAP_SYS_ADMIN */
#define DRM_NICOGPU_QUEUE_CREATE	0x26
#define DRM_NICOGPU_QUEUE_DESTROY	0x27
#define DRM_NICOGPU_QUEUE_STATUS	0x28
#define DRM_NICOGPU_SYNCOBJ_CREATE	0x29
#define DRM_NICOGPU_SYNCOBJ_DESTROY 0x2a
#define DRM_NICOGPU_SYNCOBJ_RESET	0x2b
#define DRM_NICOGPU_SYNCOBJ_SIGNAL	0x2c
#define DRM_NICOGPU_SYNCOBJ_WAIT	0x2d
#define DRM_NICOGPU_SUBMIT		0x2e
#define DRM_NICOGPU_FENCE_WAIT	0x2f
#define DRM_NICOGPU_QUEUE_FAULT	0x30
#define DRM_NICOGPU_PR_LOAD	0x31	/* CAP_SYS_ADMIN */
#define DRM_NICOGPU_RESET		0x32	/* CAP_SYS_ADMIN */

#define DRM_IOCTL_NICOGPU_PTEATTR	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_PTEATTR, struct nicogpu_pteattr)
#define NICOGPU_IOC_PTEATTR	DRM_IOCTL_NICOGPU_PTEATTR
#define DRM_IOCTL_NICOGPU_KTEST	DRM_IOR(DRM_COMMAND_BASE + DRM_NICOGPU_KTEST, struct nicogpu_ktest)
#define NICOGPU_IOC_KTEST	DRM_IOCTL_NICOGPU_KTEST
#define DRM_IOCTL_NICOGPU_DMABUF	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_DMABUF, struct nicogpu_dmabuf)
#define NICOGPU_IOC_DMABUF	DRM_IOCTL_NICOGPU_DMABUF
#define DRM_IOCTL_NICOGPU_DMAFREE	DRM_IO(DRM_COMMAND_BASE + DRM_NICOGPU_DMAFREE)
#define NICOGPU_IOC_DMAFREE	DRM_IOCTL_NICOGPU_DMAFREE
#define DRM_IOCTL_NICOGPU_GET_INFO	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_GET_INFO, struct nicogpu_info)
#define NICOGPU_IOC_GET_INFO	DRM_IOCTL_NICOGPU_GET_INFO

/* ---- memory management (PROVISIONAL) ----------
 * Object model and every check follow HAL v0:
 * BO handles and VM ids are per open file;
 * all pad/rsvd fields and unknown flags must be zero (-EINVAL).  Errors:
 * -EINVAL bad argument, -ENOENT unknown handle/VM, -ENOMEM out of memory
 * (VM_BIND: nothing changed), -ENOSPC no free VMID / mapping limit,
 * -EFAULT bad user pointer. */

/* domains / flags: identical values to NICOGPU_BO_DOMAIN_* / NICOGPU_BO_* of HAL v0 */
#define NICOGPU_BO_DOMAIN_VRAM	1u	/* device-local */
#define NICOGPU_BO_DOMAIN_GTT	2u	/* host memory, snooped */
#define NICOGPU_BO_HOST_VISIBLE	(1u << 0)	/* VRAM only: CPU-visible BAR window */

/* BO mmap (ABI v4): BO handles are GEM handles of the DRM file (the generic DRM_IOCTL_GEM_CLOSE closes them too);
 * nicogpu_bo_create.mmap_offset is the DRM fake offset of the GEM object -- opaque, valid only for the file that created
 * the BO.  mmap(offset, length <= BO size, MAP_SHARED) maps the BO from byte 0 (no partial offsets any more).
 * VRAM: write-combining; GTT: cached/coherent; VRAM without HOST_VISIBLE is not mappable (mmap_offset 0). */

struct nicogpu_bo_create {
	__u64 size;		/* in: bytes; out: rounded up to 4 KiB */
	__u64 alignment;	/* in: 0 (default) or a power of two >= 4096 */
	__u32 domain;		/* in: NICOGPU_BO_DOMAIN_* */
	__u32 flags;		/* in: NICOGPU_BO_* */
	__u32 handle;		/* out: non-zero (GEM handle) */
	__u32 pad;		/* must be 0 */
	__u64 mmap_offset;	/* out: mmap() offset of byte 0 (DRM fake offset, opaque), 0 = not mappable */
};

struct nicogpu_bo_free {
	__u32 handle;		/* in; mappings keep the storage alive */
	__u32 pad;
};

#define NICOGPU_VM_STATUS_SHADOW_ONLY (1u << 0)	/* no MMU/C3 window: tables kept in host memory only */

struct nicogpu_vm_create {
	__u32 flags;		/* in: must be 0 */
	__u32 vm_id;		/* out: VMID 1..15 */
	__u32 status;		/* out: NICOGPU_VM_STATUS_* */
	__u32 pad;
};

struct nicogpu_vm_destroy {
	__u32 vm_id;
	__u32 pad;
};

/* op / flags: identical values to HAL v0 NICOGPU_VM_BIND_* / NICOGPU_VM_* */
#define NICOGPU_VM_BIND_MAP		0u	/* map bo[bo_offset, +range) at va */
#define NICOGPU_VM_BIND_UNMAP		1u	/* unmap [va, +range) */
#define NICOGPU_VM_BIND_MAP_SPARSE_NULL	2u	/* reads 0, writes dropped, no fault */
#define NICOGPU_VM_READ		(1u << 0)
#define NICOGPU_VM_WRITE		(1u << 1)
#define NICOGPU_VM_EXEC		(1u << 2)
#define NICOGPU_VM_UNCACHED	(1u << 3)
#define NICOGPU_VM_BIND_MAX_OPS	4096u

struct nicogpu_vm_bind_op {	/* same layout as HAL struct nicogpu_vm_bind_op */
	__u32 op;
	__u32 flags;		/* MAP only */
	__u32 bo_handle;	/* MAP only */
	__u32 pad;
	__u64 bo_offset;	/* 4 KiB aligned */
	__u64 va;		/* 4 KiB aligned, [4 GiB, 2^47) */
	__u64 range;		/* 4 KiB multiple, non-zero */
};

/* Ops apply in array order with replace semantics; the call is atomic: on
 * any error nothing has changed.  Synchronous: page tables written and the
 * VMID's TLB invalidated before return. */
struct nicogpu_vm_bind {
	__u32 vm_id;
	__u32 num_ops;		/* 0..NICOGPU_VM_BIND_MAX_OPS */
	__u64 ops;		/* user pointer to struct nicogpu_vm_bind_op[num_ops] */
	__u32 flags;		/* must be 0 */
	__u32 pad;
};

/* Debug (CAP_SYS_ADMIN): translate one VA through the VM's page tables. */
#define NICOGPU_VM_LOOKUP_NOT_PRESENT	0u
#define NICOGPU_VM_LOOKUP_BAD_PTE		1u
#define NICOGPU_VM_LOOKUP_SPARSE_NULL	2u
#define NICOGPU_VM_LOOKUP_LEAF		4u
struct nicogpu_vm_lookup {
	__u32 vm_id;		/* in */
	__u32 pad;
	__u64 va;		/* in */
	__u32 kind;		/* out: NICOGPU_VM_LOOKUP_* */
	__u32 level;		/* out: page-table level of the deciding entry */
	__u64 entry;		/* out: raw entry */
	__u64 pa;		/* out: LEAF: device PA of va */
	__u64 page_size;	/* out: LEAF page size / sparse span */
};

#define DRM_IOCTL_NICOGPU_BO_CREATE	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_BO_CREATE, struct nicogpu_bo_create)
#define NICOGPU_IOC_BO_CREATE	DRM_IOCTL_NICOGPU_BO_CREATE
#define DRM_IOCTL_NICOGPU_BO_FREE	DRM_IOW(DRM_COMMAND_BASE + DRM_NICOGPU_BO_FREE, struct nicogpu_bo_free)
#define NICOGPU_IOC_BO_FREE	DRM_IOCTL_NICOGPU_BO_FREE
#define DRM_IOCTL_NICOGPU_VM_CREATE	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_VM_CREATE, struct nicogpu_vm_create)
#define NICOGPU_IOC_VM_CREATE	DRM_IOCTL_NICOGPU_VM_CREATE
#define DRM_IOCTL_NICOGPU_VM_DESTROY	DRM_IOW(DRM_COMMAND_BASE + DRM_NICOGPU_VM_DESTROY, struct nicogpu_vm_destroy)
#define NICOGPU_IOC_VM_DESTROY	DRM_IOCTL_NICOGPU_VM_DESTROY
#define DRM_IOCTL_NICOGPU_VM_BIND	DRM_IOW(DRM_COMMAND_BASE + DRM_NICOGPU_VM_BIND, struct nicogpu_vm_bind)
#define NICOGPU_IOC_VM_BIND	DRM_IOCTL_NICOGPU_VM_BIND
#define DRM_IOCTL_NICOGPU_VM_LOOKUP	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_VM_LOOKUP, struct nicogpu_vm_lookup)
#define NICOGPU_IOC_VM_LOOKUP	DRM_IOCTL_NICOGPU_VM_LOOKUP

/* ---- scheduling and synchronisation (PROVISIONAL) --
 * Object model and every check follow HAL v0 (its sim backend is the
 * behavioural reference): queue and syncobj handles are per open file; all
 * pad/rsvd fields and unknown flags must be zero (-EINVAL); timeouts are
 * relative nanoseconds (0 = poll, NICOGPU_TIMEOUT_INFINITE = no limit).
 * Errors: -EINVAL bad argument, -ENOENT unknown handle, -ECANCELED the queue is
 * GUILTY (VK_ERROR_DEVICE_LOST), -ETIME wait timed out, -EBUSY (VM_DESTROY
 * with queues bound), -ENODEV no CP (firmware not running), -EIO device
 * access failed, -EFAULT bad user pointer, -ERESTARTSYS signal.
 * The private syncobjs below are binary only (`point` 0, no export/import); since ABI v6 they are DEPRECATED in
 * favour of the DRM core syncobjs (NICOGPU_SYNC_F_DRM), kept unchanged for v5 user space. */

#define NICOGPU_QUEUE_PRIORITY_NORMAL	0u
#define NICOGPU_QUEUE_STATUS_OK		0u
#define NICOGPU_QUEUE_STATUS_GUILTY	1u	/* a job of this queue faulted or timed out */

struct nicogpu_queue_create {
	__u32 vm_id;		/* in: a VM of this file (VM_CREATE); it cannot be destroyed while queues use it */
	__u32 priority;		/* in: NICOGPU_QUEUE_PRIORITY_NORMAL */
	__u32 flags;		/* in: must be 0 */
	__u32 queue_id;		/* out: non-zero */
	__u64 rsvd[2];		/* must be 0 */
};

struct nicogpu_queue_destroy {
	__u32 queue_id;		/* waits for the queue's jobs (bounded by the job timeout) */
	__u32 pad;
};

struct nicogpu_queue_status {
	__u32 queue_id;		/* in */
	__u32 status;		/* out: NICOGPU_QUEUE_STATUS_* */
	__u64 rsvd;		/* must be 0 */
};

#define NICOGPU_SYNCOBJ_CREATE_SIGNALED	(1u << 0)
#define NICOGPU_SYNCOBJ_WAIT_ALL		(1u << 0)	/* HAL v0 */
#define NICOGPU_SYNCOBJ_WAIT_FOR_SUBMIT	(1u << 1)	/* extension: a syncobj without a fence is "not yet signalled", not -EINVAL */
#define NICOGPU_SYNCOBJ_WAIT_AVAILABLE	(1u << 2)	/* ABI v5: wait until a fence is ATTACHED (submitted / host-signalled), not until it
							 * signals; a fence-less syncobj is "not available yet" (as WAIT_FOR_SUBMIT, which
							 * it implies).  Differs from FOR_SUBMIT, which still waits for completion. */
#define NICOGPU_TIMEOUT_INFINITE		0x7fffffffffffffffLL

struct nicogpu_syncobj_create {
	__u32 flags;		/* in: NICOGPU_SYNCOBJ_CREATE_* */
	__u32 handle;		/* out: non-zero */
	__u64 rsvd;		/* must be 0 */
};

struct nicogpu_syncobj_destroy {
	__u32 handle;
	__u32 pad;
};

/* RESET (drop the fence: "unsignaled") and SIGNAL (host signal): all handles
 * must exist, otherwise nothing changes (-ENOENT).  count 0 is a no-op. */
struct nicogpu_syncobj_array {
	__u64 handles;		/* user pointer to __u32[count] */
	__u32 count;		/* 0..NICOGPU_SUBMIT_MAX_SYNCS */
	__u32 flags;		/* must be 0 */
};

/* Wait for any (default) or all (WAIT_ALL) of the handles.  A handle without a
 * fence is -EINVAL unless WAIT_FOR_SUBMIT or WAIT_AVAILABLE (ABI v5: then the wait is for "has a fence", first_signaled
 * is the index of the first handle that has one).  A job that failed still signals its
 * fence: the wait succeeds, the error is visible in the queue status and in
 * NICOGPU_IOC_FENCE_WAIT. */
struct nicogpu_syncobj_wait {
	__u64 handles;		/* user pointer to __u32[count] */
	__s64 timeout_ns;	/* in: relative, >= 0 */
	__u32 count;		/* 1..NICOGPU_SUBMIT_MAX_SYNCS */
	__u32 flags;		/* in: NICOGPU_SYNCOBJ_WAIT_* */
	__u32 first_signaled;	/* out: index of the first signalled handle (0 for WAIT_ALL) */
	__u32 pad;
	__u64 rsvd;		/* must be 0 */
};

#define NICOGPU_SUBMIT_MAX_CMDBUFS	128u
#define NICOGPU_SUBMIT_MAX_SYNCS	64u

struct nicogpu_cmdbuf {		/* == HAL struct nicogpu_cmdbuf */
	__u64 va;		/* GPU VA in the queue's VM: 4-byte aligned, < 2^48 */
	__u32 size_dw;		/* 1 .. 2^20 */
	__u32 flags;		/* must be 0 */
};

/* nicogpu_sync.flags (ABI v6).  Without NICOGPU_SYNC_F_DRM the entry names a private syncobj (ioctls 0x29..0x2d, DEPRECATED
 * since ABI v6) exactly as in v5.  With it, `handle` is a drm_syncobj handle of this DRM file (DRM_IOCTL_SYNCOBJ_CREATE,
 * FD_TO_HANDLE, ...) and `point` is a timeline point:
 *   waits:   point 0 = the syncobj's current fence; point N on a timeline = that point (a point not submitted yet, or a
 *            binary syncobj without a fence, is -EINVAL as in v5: the fence must exist at call time);
 *   signals: point 0 replaces the syncobj's fence (binary semantics); point N > 0 adds the job's fence as timeline point N
 *            (use a timeline syncobj, points must increase).
 * Other flag bits stay reserved (-EINVAL).  The same drm_syncobj may not appear twice in `signals`. */
#define NICOGPU_SYNC_F_DRM		(1u << 0)

struct nicogpu_sync {		/* == HAL struct nicogpu_sync */
	__u32 handle;
	__u32 flags;		/* NICOGPU_SYNC_F_* (ABI v6); 0 = private syncobj */
	__u64 point;		/* must be 0 for private syncobjs; the timeline point with NICOGPU_SYNC_F_DRM */
};

/* One job: the kernel builds INDIRECT(cmdbuf[i], VMID of the queue) ... + a
 * FENCE(IRQ) with a new 64-bit sequence number in the ring.
 * Jobs run in submission order; `waits` must all have a fence at call time
 * (dependencies on earlier jobs are satisfied by the in-order ring), `signals`
 * receive this job's fence.  Argument checks (HAL order): queue (-ENOENT),
 * queue GUILTY (-ECANCELED), cmdbufs, waits, signals (-EINVAL / -ENOENT; the
 * same syncobj twice in `signals` is -EINVAL).  On success nothing else can
 * fail synchronously; CP errors, MMU faults and timeouts make the queue GUILTY
 * and signal the job's fence with an error. */
struct nicogpu_submit {
	__u32 queue_id;
	__u32 flags;		/* must be 0 */
	__u64 cmdbufs;		/* user pointer to struct nicogpu_cmdbuf[num_cmdbufs] */
	__u32 num_cmdbufs;	/* 0..NICOGPU_SUBMIT_MAX_CMDBUFS (0: a bare fence) */
	__u32 num_waits;	/* 0..NICOGPU_SUBMIT_MAX_SYNCS */
	__u64 waits;		/* user pointer to struct nicogpu_sync[num_waits] */
	__u32 num_signals;	/* 0..NICOGPU_SUBMIT_MAX_SYNCS */
	__u32 pad;
	__u64 signals;		/* user pointer to struct nicogpu_sync[num_signals] */
	__u64 fence_seq;	/* out: sequence number of the job (per device, starts at 1) */
	__u64 rsvd[2];		/* must be 0 */
};

/* Wait until job `seq` (returned by SUBMIT) has retired.  Returns 0 once it
 * has (status = 0 or a negative errno: -EIO job failed, -ECANCELED dropped
 * because its queue became GUILTY), -ETIME on timeout, -EINVAL if `seq` was
 * never issued.  status is remembered for the last 64 failed jobs only. */
struct nicogpu_fence_wait {
	__u64 seq;
	__s64 timeout_ns;	/* in: relative, >= 0 */
	__u32 flags;		/* must be 0 */
	__s32 status;		/* out */
	__u64 rsvd[2];		/* must be 0 */
};

#define DRM_IOCTL_NICOGPU_QUEUE_CREATE	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_QUEUE_CREATE, struct nicogpu_queue_create)
#define NICOGPU_IOC_QUEUE_CREATE	DRM_IOCTL_NICOGPU_QUEUE_CREATE
#define DRM_IOCTL_NICOGPU_QUEUE_DESTROY	DRM_IOW(DRM_COMMAND_BASE + DRM_NICOGPU_QUEUE_DESTROY, struct nicogpu_queue_destroy)
#define NICOGPU_IOC_QUEUE_DESTROY	DRM_IOCTL_NICOGPU_QUEUE_DESTROY
#define DRM_IOCTL_NICOGPU_QUEUE_STATUS	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_QUEUE_STATUS, struct nicogpu_queue_status)
#define NICOGPU_IOC_QUEUE_STATUS	DRM_IOCTL_NICOGPU_QUEUE_STATUS
#define DRM_IOCTL_NICOGPU_SYNCOBJ_CREATE	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_SYNCOBJ_CREATE, struct nicogpu_syncobj_create)
#define NICOGPU_IOC_SYNCOBJ_CREATE	DRM_IOCTL_NICOGPU_SYNCOBJ_CREATE
#define DRM_IOCTL_NICOGPU_SYNCOBJ_DESTROY	DRM_IOW(DRM_COMMAND_BASE + DRM_NICOGPU_SYNCOBJ_DESTROY, struct nicogpu_syncobj_destroy)
#define NICOGPU_IOC_SYNCOBJ_DESTROY	DRM_IOCTL_NICOGPU_SYNCOBJ_DESTROY
#define DRM_IOCTL_NICOGPU_SYNCOBJ_RESET	DRM_IOW(DRM_COMMAND_BASE + DRM_NICOGPU_SYNCOBJ_RESET, struct nicogpu_syncobj_array)
#define NICOGPU_IOC_SYNCOBJ_RESET	DRM_IOCTL_NICOGPU_SYNCOBJ_RESET
#define DRM_IOCTL_NICOGPU_SYNCOBJ_SIGNAL	DRM_IOW(DRM_COMMAND_BASE + DRM_NICOGPU_SYNCOBJ_SIGNAL, struct nicogpu_syncobj_array)
#define NICOGPU_IOC_SYNCOBJ_SIGNAL	DRM_IOCTL_NICOGPU_SYNCOBJ_SIGNAL
#define DRM_IOCTL_NICOGPU_SYNCOBJ_WAIT	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_SYNCOBJ_WAIT, struct nicogpu_syncobj_wait)
#define NICOGPU_IOC_SYNCOBJ_WAIT	DRM_IOCTL_NICOGPU_SYNCOBJ_WAIT
#define DRM_IOCTL_NICOGPU_SUBMIT	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_SUBMIT, struct nicogpu_submit)
#define NICOGPU_IOC_SUBMIT	DRM_IOCTL_NICOGPU_SUBMIT
#define DRM_IOCTL_NICOGPU_FENCE_WAIT	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_FENCE_WAIT, struct nicogpu_fence_wait)
#define NICOGPU_IOC_FENCE_WAIT	DRM_IOCTL_NICOGPU_FENCE_WAIT

/* ---- faults, recovery, partial reconfiguration (PROVISIONAL) ----
 * What a GUILTY queue knows about why (VMID, VA, access type and page-table level of
 * the fault), plus the two operator calls: load a partial bitstream (NGPR container) and force a
 * reset.  Both operator calls need CAP_SYS_ADMIN. */

#define NICOGPU_FAULT_CAUSE_NONE		0u	/* the queue is not guilty */
#define NICOGPU_FAULT_CAUSE_TIMEOUT	1u	/* a job did not finish in time (hang) */
#define NICOGPU_FAULT_CAUSE_CP_ERROR	2u	/* the CP halted on one of its jobs: cp_error says why */
#define NICOGPU_FAULT_CAUSE_MMU_FAULT	3u	/* ... together with an MMU fault of the queue's VMID: mmu_* are valid */
#define NICOGPU_FAULT_CAUSE_FW_PANIC	4u	/* the CP firmware died while the job was at the head of the ring */
#define NICOGPU_FAULT_CAUSE_DEV_LOST	5u	/* innocent: the context died with a GPU-zone reset */

#define NICOGPU_FAULT_F_MMU_VALID		(1u << 0)	/* mmu_* describe a latched MMU fault */
#define NICOGPU_FAULT_F_MMU_MULTI		(1u << 1)	/* the MMU dropped later faults while this one was latched */
#define NICOGPU_FAULT_F_DEVICE_LOST	(1u << 2)	/* every reset level failed: the whole device is lost (submits -ENODEV) */

#define NICOGPU_FAULT_ACCESS_READ		0u
#define NICOGPU_FAULT_ACCESS_WRITE		1u
#define NICOGPU_FAULT_ACCESS_EXEC		2u

struct nicogpu_queue_fault {
	__u32 queue_id;		/* in */
	__u32 status;		/* out: NICOGPU_QUEUE_STATUS_* (same as QUEUE_STATUS) */
	__u32 cause;		/* out: NICOGPU_FAULT_CAUSE_* */
	__u32 cp_error;		/* out: CP_ERROR code of the CP halt, 0 if none */
	__u32 flags;		/* out: NICOGPU_FAULT_F_* */
	__s32 err;		/* out: error code of the failing job (-EIO), 0 if not guilty */
	__u64 fence_seq;	/* out: sequence number of the failing job (0: none, e.g. DEV_LOST) */
	__u32 mmu_code;		/* out: fault code (1 NOT_PRESENT, 2 NON_CANONICAL, 3 PERMISSION, ...) */
	__u32 mmu_access;	/* out: NICOGPU_FAULT_ACCESS_* */
	__u32 mmu_level;	/* out: page-table level of the deciding entry */
	__u32 mmu_vmid;		/* out */
	__u64 mmu_va;		/* out: faulting byte VA */
	__u64 mmu_pte;		/* out: the entry read at `mmu_level` */
	__u32 resets;		/* out: CP / firmware / GPU-zone resets since the CP came up (device wide) */
	__u32 gpu_resets;	/* out: ... of which GPU-zone resets */
	__u64 rsvd[2];		/* in: must be 0 */
};

#define NICOGPU_PR_LOADED		1u	/* the new RM runs */
#define NICOGPU_PR_REJECTED	2u	/* refused before the hardware was touched (container check, queues alive) */
#define NICOGPU_PR_RESTORED	3u	/* the load failed twice, the previous / the blank RM runs again (ioctl returns -EIO) */
#define NICOGPU_PR_WEDGED		4u	/* nothing could be loaded: reboot / IPROG (ioctl returns -ENODEV, later calls too) */

#define NICOGPU_PR_MAX_BYTES	(256ull << 20)	/* container size limit */

/* Load a partial bitstream.  The container is checked first (magic, SHELL_UID -- a bitstream of another static region
 * never reaches ICAP --, interface version, SHA-256, length); every queue / VM of every file must be gone (-EBUSY).
 * Errors: -EINVAL malformed, -ENXIO another shell, -EPROTO interface incompatible, -EBADMSG digest, -E2BIG, -EBUSY,
 * -EPERM (CAP_SYS_ADMIN), -EIO (see result), -ENODEV (wedged), -EFAULT. */
struct nicogpu_pr_load {
	__u64 data;		/* in: user pointer to the NGPR container */
	__u64 size;		/* in: bytes: 128-byte header + payload */
	__u32 flags;		/* in: must be 0 */
	__u32 result;		/* out: NICOGPU_PR_* (0 when the container was rejected before a load started) */
	__s32 stage;		/* out: stage of the last failed attempt (1 isolate, 2 pr, 3 identify, 4 release, 5 init), 0 none */
	__s32 error;		/* out: its errno (negative), 0 none */
	__u64 rsvd[2];		/* in: must be 0 */
};

#define NICOGPU_RESET_CP		1u	/* CP_CTRL.RESET, escalating if the CP does not idle */
#define NICOGPU_RESET_FW		2u	/* firmware reload: CORE_RESET + the 7.4 handshake */
#define NICOGPU_RESET_GPU		3u	/* GPU-zone soft reset: every context is lost */

struct nicogpu_reset {
	__u32 level;		/* in: NICOGPU_RESET_* */
	__u32 flags;		/* in: must be 0 */
	__s32 result;		/* out: 0 or the negative errno of the reset (-EIO: the device is lost) */
	__u32 pad;		/* must be 0 */
	__u64 rsvd[2];		/* in: must be 0 */
};

#define DRM_IOCTL_NICOGPU_QUEUE_FAULT	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_QUEUE_FAULT, struct nicogpu_queue_fault)
#define NICOGPU_IOC_QUEUE_FAULT	DRM_IOCTL_NICOGPU_QUEUE_FAULT
#define DRM_IOCTL_NICOGPU_PR_LOAD	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_PR_LOAD, struct nicogpu_pr_load)
#define NICOGPU_IOC_PR_LOAD	DRM_IOCTL_NICOGPU_PR_LOAD
#define DRM_IOCTL_NICOGPU_RESET	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_RESET, struct nicogpu_reset)
#define NICOGPU_IOC_RESET	DRM_IOCTL_NICOGPU_RESET

/* ---- ABI v5: capabilities, timestamp, device status (PROVISIONAL) ----
 * Pure additions for the Mesa winsys (HAL v2): three new ioctls (0x11..0x13, the gap behind GET_INFO) and the
 * NICOGPU_SYNCOBJ_WAIT_AVAILABLE flag.  All pad/rsvd fields and unknown flags must be zero (-EINVAL).  Host-signalling a
 * syncobj needs no new ioctl: DRM_NICOGPU_SYNCOBJ_SIGNAL (0x2c) has existed since ABI v2. */

/* GET_GPU_INFO.info_flags: where the numbers came from.  A clear bit means "driver default, PROVISIONAL": user space
 * must not treat it as measured hardware truth (the RM may be the blank/test RM, or the regmap is not implemented). */
#define NICOGPU_GPU_INFO_F_RM_CAPS		(1u << 0)	/* num_cores/sp_per_core/warp_size read from the RM CAPS register */
#define NICOGPU_GPU_INFO_F_RM_REV		(1u << 1)	/* revision read from the RM REV register */
#define NICOGPU_GPU_INFO_F_TS_RM		(1u << 2)	/* timestamp_freq_hz = RM TIMESTAMP_FREQ */
#define NICOGPU_GPU_INFO_F_TS_SHELL	(1u << 3)	/* timestamp_freq_hz = shell CLK_FREQ_TS (kHz * 1000); mutually exclusive with TS_RM */

/* Pure addition (no ABI bump): host memory (GTT BOs) is CPU-uncached / write-combined because the host is not dma-coherent.
 * The device still reads/writes it with No-Snoop=0 TLPs, and the CPU mapping is uncached, so no cache maintenance is
 * needed -- but the GTT is then NOT HOST_CACHED and NOT HOST_COHERENT-as-cached (slow CPU reads, WC writes).  A clear
 * bit means the host is dma-coherent: GTT is cached + coherent (HAL NICOGPU_BO_DOMAIN_GTT contract). */
#define NICOGPU_GPU_INFO_F_GTT_UNCACHED	(1u << 4)

/* Pure addition (ABI v6 stays): the loaded RM has no device path to VRAM (rm_cp kind 0x8002, rm_sp kind 0x8003, read from
 * RM_INFO).  Then vram_size == vram_visible_size == vram_used == 0, BO_CREATE with NICOGPU_BO_DOMAIN_VRAM fails with
 * -EOPNOTSUPP, and user space must expose host-visible (GTT) memory only; shader code and buffers have to live in GTT.
 * A clear bit means VRAM is usable (blank/test RM, future full RM, unidentified RM). */
#define NICOGPU_GPU_INFO_F_NO_VRAM		(1u << 5)

/* Pure addition (ABI v6 stays): nicogpu_gpu_info.sp_units was read from the RM CAPS register
 * (CAPS[26:24] with CAPS[27] UNITS_VALID set).  A clear bit means "unknown", NOT "no optional units": an RM built before the
 * unit bits (or without the RM regmap) leaves them 0, and user space must then use its own default. */
#define NICOGPU_GPU_INFO_F_SP_UNITS	(1u << 6)

/* nicogpu_gpu_info.sp_units: the optional SP execution units this RM was built with (same bits as CAPS[26:24] and HAL
 * NICOGPU_SP_UNIT_*).  valu / salu / lsu / branch / sync are always there. */
#define NICOGPU_SP_UNIT_SFU		(1u << 0)	/* rcp rsq sqrt exp2 log2 sin cos, integer divide */
#define NICOGPU_SP_UNIT_XLANE		(1u << 1)	/* readlane writelane ballot shuffle ... */
#define NICOGPU_SP_UNIT_FP64		(1u << 2)
/* Pure addition: the VALU executes the binary16 <-> binary32 conversions
 * fcvt.f32.f16.lo/hi, fcvt.f16.f32.lo/hi(.m), pkcvt.f16.f32(.m) (RM CAPS[29]); clear on RMs that predate it. */
#define NICOGPU_SP_UNIT_F16CVT		(1u << 3)

/* GET_GPU_INFO.features: which optional kernel facilities this driver implements (bit set = present) */
#define NICOGPU_FEAT_GPU_INFO		(1ull << 0)
#define NICOGPU_FEAT_TIMESTAMP		(1ull << 1)	/* GET_TIMESTAMP */
#define NICOGPU_FEAT_SYNCOBJ_WAIT_AVAILABLE (1ull << 2)
#define NICOGPU_FEAT_SYNCOBJ_SIGNAL	(1ull << 3)	/* host SYNCOBJ_SIGNAL (since ABI v2) */
#define NICOGPU_FEAT_DEV_STATUS		(1ull << 4)	/* GET_DEV_STATUS */
#define NICOGPU_FEAT_QUEUE_FAULT		(1ull << 5)
#define NICOGPU_FEAT_PR_LOAD		(1ull << 6)
#define NICOGPU_FEAT_RESET			(1ull << 7)
#define NICOGPU_FEAT_SYNCOBJ_TIMELINE	(1ull << 8)	/* ABI v6: timeline points in SUBMIT (NICOGPU_SYNC_F_DRM) and DRIVER_SYNCOBJ_TIMELINE */
#define NICOGPU_FEAT_PRIME			(1ull << 9)	/* GTT BO export + same-device loopback import (DRM_IOCTL_PRIME_*) */
#define NICOGPU_FEAT_DRM_SYNCOBJ		(1ull << 10)	/* ABI v6: DRIVER_SYNCOBJ, SUBMIT accepts drm_syncobj handles */
#define NICOGPU_FEAT_SYNC_FILE		(1ull << 11)	/* ABI v6: sync_file import / export via DRM_IOCTL_SYNCOBJ_{HANDLE_TO_FD,FD_TO_HANDLE} */

#define NICOGPU_GPU_PAGE_4K	(1u << 0)	/* page_sizes: same values as HAL NICOGPU_PAGE_* */
#define NICOGPU_GPU_PAGE_64K	(1u << 1)
#define NICOGPU_GPU_PAGE_2M	(1u << 2)

/* Field names and units follow HAL struct nicogpu_device_info so a winsys copies field by field.  vram_size is what the
 * allocator can serve (the BAR2 VRAM heap; the C3 window is already subtracted): in ABI v5 every VRAM BO lives in it,
 * so vram_size == vram_visible_size.  *_used are live BO bytes (page rounded), a budget hint only. */
struct nicogpu_gpu_info {
	__u32 struct_size;	/* in: sizeof(struct nicogpu_gpu_info) */
	__u32 flags;		/* in: must be 0 */
	__u32 abi_version;	/* out */
	__u32 info_flags;	/* out: NICOGPU_GPU_INFO_F_* */
	__u64 features;		/* out: NICOGPU_FEAT_* */
	__u32 vendor_id, device_id;
	__u32 revision;
	__u32 num_cores;
	__u32 sp_per_core;
	__u32 warp_size;
	__u32 warp_slots_per_sp;
	__u32 max_vregs;
	__u32 num_uregs;
	__u32 max_workgroup_threads;
	__u32 max_shared_bytes;
	__u32 max_user_uregs;
	__u32 va_bits;
	__u32 page_sizes;	/* NICOGPU_GPU_PAGE_* */
	__u32 max_vms;		/* VM_CREATE limit per device (VMIDs 1..15) */
	__u32 max_submit_cmdbufs;	/* NICOGPU_SUBMIT_MAX_CMDBUFS */
	__u32 max_submit_syncs;	/* NICOGPU_SUBMIT_MAX_SYNCS (also the SYNCOBJ_* array limit) */
	__u32 max_bind_ops;	/* NICOGPU_VM_BIND_MAX_OPS */
	__u64 va_user_start;	/* VM_BIND VA window [start, end) */
	__u64 va_user_end;
	__u64 vram_size;
	__u64 vram_visible_size;
	__u64 gtt_size;		/* total GTT BO bytes the driver allows */
	__u64 gtt_bo_max;	/* largest single GTT BO */
	__u64 vram_used;
	__u64 gtt_used;
	__u64 timestamp_freq_hz;	/* ticks per second of GET_TIMESTAMP / the TIMESTAMP packet */
	__u32 sp_units;		/* in: must be 0; out: NICOGPU_SP_UNIT_* (valid iff info_flags has NICOGPU_GPU_INFO_F_SP_UNITS, else 0) */
	__u32 pad;		/* in: must be 0; out: 0 */
	__u64 rsvd[3];		/* in: must be 0; out: 0 */
};

/* GET_TIMESTAMP: the device timestamp counter (BAR0 TIMESTAMP_LO latches _HI; same counter the TIMESTAMP packet
 * writes) together with two host clock samples taken around the register read, for VK_EXT_calibrated_timestamps.
 * clock_id: 0 or 1 = CLOCK_MONOTONIC, 4 = CLOCK_MONOTONIC_RAW.  Errors: -EINVAL, -ENODEV (no RM with the regmap: the RM
 * is isolated / replaced / the blank RM), -EAGAIN (the RM went away during the read; retry). */
struct nicogpu_timestamp {
	__u32 flags;		/* in: must be 0 */
	__u32 clock_id;		/* in */
	__u64 ts;		/* out: device ticks */
	__u64 freq_hz;		/* out: ticks per second (same as nicogpu_gpu_info.timestamp_freq_hz) */
	__u64 cpu_ns_before;	/* out: host clock before the register read, ns */
	__u64 cpu_ns_after;	/* out: host clock after it (the read happened in [before, after]) */
	__u64 rsvd[2];		/* must be 0 */
};

/* GET_DEV_STATUS: whole-device state, independent of any queue (QUEUE_FAULT is per queue).  `status` is the state now;
 * lost_reason / lost_err / lost_time_ns describe the MOST RECENT loss and are kept after a recovery (lost_count tells
 * how often it happened).  Clock: CLOCK_MONOTONIC, so now_ns - lost_time_ns is the age.  Never fails for a live
 * device; an unplugged device fails every ioctl with -ENODEV. */
#define NICOGPU_DEVSTAT_LOST	(1u << 0)	/* every reset level failed (submits -ENODEV) or the PR flow wedged the device */
#define NICOGPU_DEVSTAT_PR_WEDGED	(1u << 1)	/* no RM could be loaded: reboot / IPROG (implies LOST) */
#define NICOGPU_DEVSTAT_IO_ERROR	(1u << 2)	/* ring / doorbell access failed, recovery pending (informational) */
#define NICOGPU_DEVSTAT_CP_READY	(1u << 3)	/* the CP is up (it is brought up lazily by the first queue / syncobj ioctl) */

#define NICOGPU_LOST_NONE		0u
#define NICOGPU_LOST_RESET_FAILED	1u	/* the reset ladder (CP reset, firmware reload, GPU zone) was exhausted */
#define NICOGPU_LOST_PR_WEDGED	2u	/* partial reconfiguration failed twice and the rollback failed too */

struct nicogpu_dev_status {
	__u32 flags;		/* in: must be 0 */
	__u32 status;		/* out: NICOGPU_DEVSTAT_* now */
	__u32 lost_reason;	/* out: NICOGPU_LOST_* of the most recent loss, 0 if never lost */
	__s32 lost_err;		/* out: negative errno that ended that flow, 0 if none */
	__u64 lost_time_ns;	/* out: CLOCK_MONOTONIC at the most recent loss, 0 = never */
	__u32 lost_count;	/* out: times the device was declared lost / wedged */
	__u32 resets;		/* out: CP / firmware / GPU-zone resets since the CP came up (device wide, as QUEUE_FAULT) */
	__u32 gpu_resets;	/* out: ... of which GPU-zone resets */
	__u32 pad;		/* out: 0 */
	__u64 now_ns;		/* out: CLOCK_MONOTONIC now */
	__u64 rsvd[2];		/* must be 0 */
};

#define DRM_IOCTL_NICOGPU_GET_GPU_INFO	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_GET_GPU_INFO, struct nicogpu_gpu_info)
#define NICOGPU_IOC_GET_GPU_INFO	DRM_IOCTL_NICOGPU_GET_GPU_INFO
#define DRM_IOCTL_NICOGPU_GET_TIMESTAMP	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_GET_TIMESTAMP, struct nicogpu_timestamp)
#define NICOGPU_IOC_GET_TIMESTAMP	DRM_IOCTL_NICOGPU_GET_TIMESTAMP
#define DRM_IOCTL_NICOGPU_GET_DEV_STATUS	DRM_IOWR(DRM_COMMAND_BASE + DRM_NICOGPU_GET_DEV_STATUS, struct nicogpu_dev_status)
#define NICOGPU_IOC_GET_DEV_STATUS	DRM_IOCTL_NICOGPU_GET_DEV_STATUS

/* BAR0 scratch byte-lane window used by KTEST (same as p0test) */
#define NICOGPU_LANE_LO	0x0C
#define NICOGPU_LANE_HI	0x18
#define NICOGPU_SCRATCH_INIT 0x11223344u

#endif
