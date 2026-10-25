/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * nicogpu_sched_core.h -- pure-C core of the nicogpu scheduler / synchronisation
 * layer.
 *
 * Everything with semantics lives here so it compiles both in the kernel
 * module and as plain host C: the C3 front-region
 * layout, the command ring (WPTR/RPTR arithmetic, wrap, full/empty, doorbell),
 * ring-entry encoding (INDIRECT.. + FENCE), 64-bit fence sequence numbers and
 * completion, jobs, queues, binary syncobjs, the in-order scheduler with
 * dependency tracking, the timeout / CP-error handlers (guilty marking, job
// ABI in nicogpu_uapi.h.
 * The kernel glue (nicogpu_sched.c) supplies memory, locking, MMIO, time and
 * wake-ups through `struct xsc_ops`, and the ioctl layer.
 *
 * The mapping to drm_sched is
 *   drm_sched_job_arm/push      -> xsc_submit()  (arm: seq + syncobj fences)
 *   drm_sched .run_job          -> the ring push in xsc_run()
 *   drm_sched dependencies      -> xsc_job.deps[] (xsc_run only pushes a job
 *                                  whose dependencies are signalled or are
 *                                  jobs ahead of it in the same ring)
 *   drm_sched .timedout_job     -> xsc_check_timeout() / xsc_cp_error()
 *                                  -> the reset ladder (recover_from())
 *   drm_sched .free_job         -> retire_locked()
 *
 * Locking: the core takes the caller's lock (ops->lock/unlock, a spinlock
 * with IRQs disabled in the kernel) around its own state; every function
 * below states whether it may be called from IRQ context.  The *sleeping*
 * ops (ring_write, doorbell, read_rptr, cp_reset, reg_*, fw_load, msleep,
 * gpu_reset, rm_restore) are only ever called with the lock released, from the
 * single worker that serialises xsc_run / xsc_check_timeout / xsc_cp_error /
 * xsc_cp_fw_event / xsc_force_reset / xsc_recover_io / xsc_cp_bringup /
 * xsc_abort_all.  The reset ladder sleeps through them, so while it
 * runs other threads can still submit, take fence interrupts and wait.  ops->zalloc may sleep and is only called by the allocation
 * helpers, which the caller invokes without the lock.  Nothing inside a
 * locked region allocates; ops->free may run under the lock (kfree).
 *
 * Time is milliseconds from ops->now_ms (monotonic).
 */
#ifndef NICOGPU_SCHED_CORE_H
#define NICOGPU_SCHED_CORE_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#else
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#endif
#include "nicogpu_cmd.h"	/* generated, vendored in gen/ */
#include "nicogpu_pte.h"	/* generated: MMU_FAULT_* field macros */

#ifdef __cplusplus
extern "C" {
#endif

/* Linux errno values, returned negated. */
#define XSC_ENOENT	2
#define XSC_EIO		5
#define XSC_ENOMEM	12
#define XSC_EBUSY	16
#define XSC_ENODEV	19
#define XSC_EINVAL	22
#define XSC_ENOSPC	28
#define XSC_ETIME	62
#define XSC_EPROTO	71
#define XSC_ETIMEDOUT	110
#define XSC_ECANCELED	125

/* ---- limits (PROVISIONAL) ---------------------------------------------- */
#define XSC_MAX_IBS		128u	/* cmdbufs per submit */
#define XSC_MAX_SYNCS		64u	/* waits / signals / handles per call */
#define XSC_IB_MAX_DW		NICOGPU_CMD_LIMIT_MAX_IB_DWORDS	/* 2^20 */
#define XSC_VA_BITS		48u	/* INDIRECT.IB_VA_HI carries VA[47:32] */
#define XSC_INDIRECT_DW		5u	/* header + 4 payload */
#define XSC_FENCE_DW		6u	/* header + 5 payload */
#define XSC_JOB_MAX_DW		(XSC_MAX_IBS * XSC_INDIRECT_DW + XSC_FENCE_DW)
#define XSC_RING_LOG2_MIN	10u	/* CP_RING_SIZE_LOG2 range */
#define XSC_RING_LOG2_MAX	20u
#define XSC_ERR_HIST		64u	/* remembered failed seqs for xsc_seq_status() */
#define XSC_NUM_VMIDS		16u
#define XSC_FAULT_LOG		8u	/* MMU fault records kept */
#define XSC_FAULT_MATCH_MS	1000u	/* a fault record older than this is not attached to a failing job (PROVISIONAL) */
#define XSC_HB_STALL_MS		100u	/* CP_HEARTBEAT unchanged for this long = firmware dead */
#define XSC_ENGINE_IDLE_MS	20u	/* ops->engine_busy must drop within this long after a reset level (SP drain is ~0.2 ms) */
#define XSC_RESET_IDLE_MS	10u	/* CP_STATUS.IDLE within this long after CP_CTRL.RESET */

/* C3 front region reserved for the kernel. */
#define XSC_C3_FRONT_SIZE	(16ULL << 20)

/* BAR0 IRQ_STATUS / IRQ_ENABLE bits (generated: NICOGPU_REG_IRQ_*; same bit positions in both) */
#define XSC_IRQ_FENCE		NICOGPU_REG_IRQ_STATUS_FENCE_MASK
#define XSC_IRQ_CP_ERROR	NICOGPU_REG_IRQ_STATUS_CP_ERROR_MASK
#define XSC_IRQ_MMU_FAULT	NICOGPU_REG_IRQ_STATUS_MMU_FAULT_MASK
#define XSC_IRQ_CP_FW		NICOGPU_REG_IRQ_STATUS_CP_FW_MASK

/* minimum command-stream minor the driver needs: C3 window + FENCE.L2_WB (v0.2) */
#define XSC_CMD_MAJOR		0u
#define XSC_CMD_MINOR_MIN	2u
#define XSC_FW_READY_TIMEOUT_MS	100u
#define XSC_FW_SKIP		1	/* ops->fw_load: nothing to load */

/* syncobj wait flags (== HAL NICOGPU_SYNCOBJ_WAIT_ALL; WAIT_FOR_SUBMIT is an extension) */
#define XSC_WAIT_ALL		(1u << 0)
#define XSC_WAIT_FOR_SUBMIT	(1u << 1)
#define XSC_WAIT_FLAGS_ALL	(XSC_WAIT_ALL | XSC_WAIT_FOR_SUBMIT)

/* ---- C3 front-region layout --------------------------------------------- */

struct xsc_c3 {
	uint64_t limit;		/* bytes: [0, limit) */
	uint64_t next;		/* bump pointer */
};
void xsc_c3_init(struct xsc_c3 *l, uint64_t limit);
/* Bump allocation of `size` bytes aligned to `align` (power of two).
 * -EINVAL bad argument, -ENOSPC does not fit. */
int xsc_c3_alloc(struct xsc_c3 *l, uint64_t size, uint64_t align, uint64_t *off);

/* ---- objects ------------------------------------------------------------- */

struct xsc_core;

/* A fence signals once, with an error code (0 or a negative errno). */
struct xsc_fence {
	uint32_t refs;		/* protected by the core lock */
	uint64_t seq;		/* job identity, armed in submit order; 0 = no job (stub / external) */
	uint64_t rseq;		/* position in the ring = fence-page value; 0 = not in the ring (yet).  == seq unless a job
				 * was pushed past a blocked older one of another queue */
	int err;		/* valid once signaled */
	bool signaled;
	bool foreign;		/* not produced by this ring: signalled by xsc_fence_signal() */
};

/* Binary syncobj: fence == NULL means "no fence yet". */
struct xsc_syncobj {
	uint32_t refs;
	struct xsc_fence *fence;
};

/* One latched MMU fault, decoded. */
struct xsc_mmu_fault {
	bool valid;		/* the slot holds a record */
	bool multi;		/* MMU_FAULT_STATUS.MULTI: later faults were dropped */
	bool used;		/* attached to a failing job */
	uint32_t n;		/* serial number, 1.. */
	uint32_t status;	/* raw MMU_FAULT_STATUS */
	uint32_t code;		/* NICOGPU_MMU_FAULT_* */
	uint32_t access;	/* 0 read, 1 write, 2 exec */
	uint32_t level;
	uint32_t vmid;
	uint64_t va, pte;
	uint64_t t_ms;
};

/* What made a queue GUILTY (the latest reason only). */
struct xsc_queue_fault {
	uint32_t cause;		/* enum xsc_cause */
	uint32_t cp_error;	/* CP_ERROR code, 0 if none */
	uint64_t seq;		/* the job that failed (0: none, e.g. DEV_LOST) */
	int err;		/* its error */
	uint32_t resets;	/* device reset count at that time */
	bool has_mmu;
	struct xsc_mmu_fault mmu;
};

struct xsc_queue {
	struct xsc_core *c;
	uint32_t vmid;		/* 1..15 */
	bool guilty;
	uint64_t guilty_seq;
	uint32_t njobs;		/* armed and not yet retired */
	uint32_t scan_gen;	/* == core->scan_gen: a job of this queue is blocked in the current xsc_run scan */
	uint32_t epoch;		/* context epoch at creation: older than core->epoch = lost */
	struct xsc_queue_fault fault;
};

struct xsc_ib {			/* one user command buffer, == HAL nicogpu_cmdbuf */
	uint64_t va;
	uint32_t size_dw;
	uint32_t flags;		/* must be 0 */
};

enum xsc_job_state {
	XSC_JS_PENDING = 0,	/* armed, not in the ring */
	XSC_JS_PUSHING,		/* ring space reserved, being written */
	XSC_JS_RING,		/* doorbell rung: owned by the CP */
	XSC_JS_FAILED,		/* will be retired with `err` without CP involvement */
};

struct xsc_job {
	struct xsc_job *next;
	struct xsc_queue *q;
	struct xsc_fence *fence;	/* one ref held by the job */
	enum xsc_job_state state;
	uint32_t vmid;
	uint32_t nib;
	struct xsc_ib *ib;		/* nib entries (inside the same allocation) */
	uint32_t ndeps, deps_cap;
	struct xsc_fence **deps;	/* ndeps referenced fences */
	uint32_t dw;			/* ring dwords of the whole job */
	uint32_t ring_start, ring_end;	/* WPTR values [start, end) once pushed */
	uint64_t t_head;		/* ms: since when this job is the in-flight head (or last made progress) */
	uint64_t t_first;		/* ms: since when this job is the in-flight head */
	uint64_t prog;			/* ops->progress when t_head was last set */
	int pre_err;		/* != 0: do not push, retire with this error */
	int err;			/* final error (FAILED) */
};

/* (handle, flags, point) of one wait/signal entry; so == NULL: unknown handle */
struct xsc_sync_ref {
	struct xsc_syncobj *so;
	uint32_t flags;
	uint64_t point;
};

/* ---- environment ---------------------------------------------------------- */

enum xsc_event {		/* ops->event(): diagnostics only */
	XSC_EV_JOB_FAILED,	/* a = seq, b = (u32)-err */
	XSC_EV_LOST_IRQ,	/* a = jobs found retired by polling */
	XSC_EV_BAD_FENCE,	/* a = observed value, b = last armed seq */
	XSC_EV_BAD_RPTR,	/* a = rptr seen, b = wptr */
	XSC_EV_REPLAY,		/* a = jobs requeued after a CP reset */
	XSC_EV_IO_ERROR,	/* a = (u32)-rc of the failing ring/doorbell op, b = seq */
	XSC_EV_CP_ERROR,	/* a = seq of the job the CP error is attributed to, b = CP_ERROR code */
	XSC_EV_MMU_FAULT,	/* a = fault VA, b = raw MMU_FAULT_STATUS */
	XSC_EV_INCIDENT,	/* a = enum xsc_cause, b = seq of the guilty job (0: none): devcoredump trigger */
	XSC_EV_RESET,		/* a = enum xsc_reset_level reached, b = result (0 ok, (u32)-errno) */
	XSC_EV_DEVICE_LOST,	/* every escalation level failed; a = last error */
	XSC_EV_ABORT_TIMEOUT,	/* CP_FW_STATUS.ABORT_TIMEOUT after CP_CTRL.RESET: a = raw CP_FW_STATUS, b = 1 if the CP
				 * also failed to idle in time; the ladder goes straight to the GPU-zone reset */
};

/* Why a job / queue was failed (NICOGPU_FAULT_CAUSE_* in the UAPI have the same values). */
enum xsc_cause {
	XSC_CAUSE_NONE = 0,
	XSC_CAUSE_TIMEOUT,	/* watchdog: the job was the in-flight head for too long */
	XSC_CAUSE_CP_ERROR,	/* the CP halted with an error attributed to the job (CP_ERROR code kept) */
	XSC_CAUSE_MMU_FAULT,	/* CP error / timeout together with an MMU fault record of the job's VMID */
	XSC_CAUSE_FW_PANIC,	/* firmware PANIC / BOOT_ERR; the job was the one at CP_RING_RPTR */
	XSC_CAUSE_DEV_LOST,	/* innocent: its context died with a GPU-zone reset */
};

/* Reset ladder, also the `level` of xsc_force_reset(). */
enum xsc_reset_level {
	XSC_RST_NONE = 0,
	XSC_RST_CP = 1,		/* CP_CTRL = ENABLE|RESET, wait CP_STATUS.IDLE */
	XSC_RST_FW = 2,		/* CORE_RESET + CP bring-up again (firmware dead) */
	XSC_RST_GPU = 3,	/* GPU zone soft reset (ops->gpu_reset) + restore + CP bring-up */
};

struct xsc_ops {
	/* memory */
	void *(*zalloc)(void *ctx, size_t size);	/* zeroed; may sleep; never under the lock */
	void (*free)(void *ctx, void *p);		/* may run under the lock / in IRQ */
	/* locking and time: usable under the lock, never sleep */
	void (*lock)(void *ctx);
	void (*unlock)(void *ctx);
	uint64_t (*now_ms)(void *ctx);
	uint64_t (*read_fence)(void *ctx);		/* the 64-bit seqno in host memory */
	void (*notify)(void *ctx);			/* wake every waiter (wake_up_all) */
	void (*event)(void *ctx, enum xsc_event ev, uint64_t a, uint64_t b);	/* optional */
	/* sleeping ops: lock released, worker context */
	int (*ring_write)(void *ctx, uint32_t dw_idx, const uint32_t *dw, uint32_t n);	/* never wraps */
	int (*doorbell)(void *ctx, uint32_t wptr);
	int (*read_rptr)(void *ctx, uint32_t *rptr);	/* optional (RPTR write-back page) */
	/* The level-1 reset (CP_CTRL.RESET).  0 = the CP is reset and idle with
	 * RPTR == WPTR.  Without XSC_CFG_ESCALATE this is the only recovery hook
	 * (NULL: no recovery, only the guilty job is failed, the basic contract);
	 * with it the core writes CP_CTRL itself (heartbeat probe, RESET, IDLE
	 * wait) when this is NULL, and the hook merely replaces that step. */
	int (*cp_reset)(void *ctx);
	uint32_t (*reg_rd)(void *ctx, uint32_t off);	/* BAR0 */
	void (*reg_wr)(void *ctx, uint32_t off, uint32_t val);
	void (*msleep)(void *ctx, uint32_t ms);
	/* optional: place the firmware image in C3 during the bring-up.
	 * 0 + its PA/size: written to CP_FW_BASE/SIZE; XSC_FW_SKIP: the image is
	 * already there (boot ROM / test RM), write nothing; -errno: fail. */
	int (*fw_load)(void *ctx, uint64_t *base_pa, uint32_t *size);
	/* XSC_CFG_ESCALATE only; sleeping, worker context, lock released.
	 * gpu_reset: reset ladder level 2 (quiesce/isolate/RM_RESET/release,
	 * reload of the last good RM if the soft reset fails); 0 = the RM is back,
	 * released from isolation, IRQ forwarding on.  rm_restore: everything the
	 * RM forgot that only the host knows (C3 window, MMU contexts); NULL = none. */
	int (*gpu_reset)(void *ctx);
	int (*rm_restore)(void *ctx);
	/* optional, usable under the lock, never sleeps: a counter that moves while the GPU works on the head job
	 * (RPTR, retired shader instructions, ...).  With cfg.max_job_ms a head job past timeout_ms whose counter
	 * moved since the last check gets another timeout_ms, up to max_job_ms in total. */
	uint64_t (*progress)(void *ctx);
	/* optional, never sleeps: non-zero while the compute engine still runs
	 * a dispatch.  The reset ladder polls it after CP_CTRL.RESET and after the firmware reload; an engine that does
	 * not stop within XSC_ENGINE_IDLE_MS makes the level fail so the ladder escalates (a reset that left the SP
	 * running a minutes-long dispatch must not be reported as success).  NULL = assume the engine stopped. */
	int (*engine_busy)(void *ctx);
};

/* xsc_cfg.flags */
#define XSC_CFG_ESCALATE	(1u << 0)	/* implement the reset ladder in the core (below) */
#define XSC_CFG_L2_KEEP_CTX	(1u << 1)	/* a GPU-zone reset does not condemn the other queues */

struct xsc_cfg {
	uint32_t ring_log2;	/* CP_RING_SIZE_LOG2: 10..20 */
	uint64_t ring_pa;	/* CP view (C3 physical), 4 KiB aligned */
	uint64_t fence_pa;	/* 8-aligned host-memory address of the u64 fence */
	uint64_t rptr_wb_pa;	/* 4-aligned host address of the RPTR write-back, 0 = off */
	uint32_t timeout_ms;	/* job timeout (10 s default) */
	uint32_t max_job_ms;	/* 0: timeout_ms is final; else >= timeout_ms, the limit for a job that keeps progressing */
	uint32_t flags;		/* XSC_CFG_* (0 = the basic behaviour: only ops->cp_reset, no escalation) */
};

struct xsc_stats {
	uint64_t armed, pushed, retired, failed;	/* jobs */
	uint64_t ring_wraps, ring_full, dep_stalls;
	uint64_t reordered;		/* jobs pushed past a blocked older job of another queue */
	uint64_t doorbells;
	uint64_t lost_irq;		/* retire found by polling instead of the MSI */
	uint64_t late_irq;		/* S5: of those, the MSI came after all (the watchdog just beat the ISR): not a loss */
	uint64_t bad_fence, stale_fence, bad_rptr;
	uint64_t io_errors, resets, replayed;
	uint64_t reentry;		/* xsc_run & co. called concurrently (a glue bug) */
	uint64_t reset_cp, reset_fw, reset_gpu, lost;	/* per ladder level; device-lost count */
	uint64_t mmu_faults, fault_attached, fault_vmid_mismatch;
	uint64_t unattributed;		/* CP errors that belong to no job */
	uint64_t incidents, ctx_lost;	/* devcoredump triggers; queues condemned by a GPU-zone reset */
	uint64_t timeout_ext;		/* timeouts deferred because the head job made progress */
	uint64_t abort_timeout;		/* level-1 resets whose dispatch abort hit the fabric-drain watchdog */
};

/* The last thing that failed a job (devcoredump header), with the failing job's command buffers (it is
 * retired by the time the dump is taken). */
#define XSC_INC_IBS	4u
struct xsc_incident {
	uint32_t n;		/* serial number */
	uint32_t cause;		/* enum xsc_cause */
	uint32_t cp_error;
	uint64_t seq;		/* 0: no job was to blame */
	uint64_t t_ms;
	uint32_t vmid, nib, nib_kept, ring_start, ring_end;
	uint64_t ib_va[XSC_INC_IBS];
	uint32_t ib_dw[XSC_INC_IBS];
};

struct xsc_core {
	const struct xsc_ops *ops;
	void *ctx;
	struct xsc_cfg cfg;
	uint32_t ring_dw;		/* 1 << ring_log2 */
	uint32_t wptr;			/* dwords reserved (free-running, wraps at 2^32) */
	uint32_t wptr_db;		/* last doorbell value written successfully */
	uint32_t rptr;			/* last known consumed count */
	uint64_t last_seq;		/* last armed sequence number */
	uint64_t flight_seq;		/* highest rseq handed to the in-flight list */
	uint64_t done_rseq;		/* rseq of the last retired job */
	uint64_t poll_suspect;		/* S5: jobs the last watchdog poll retired whose MSI has not shown up yet */
	bool ooo;			/* a job was pushed past an older one: done_seq is recomputed on retire */
	uint32_t scan_gen;		/* xsc_run scan generation (queue blocked marker) */
	uint64_t observed;		/* highest fence value accepted from the fence page */
	uint64_t done_seq;		/* every seq <= done_seq is retired (later ones may be too, see ooo) */
	struct xsc_job *pend_head, *pend_tail;	/* armed, not in the ring, in seq order */
	struct xsc_job *fl_head, *fl_tail;	/* in flight, in rseq order */
	uint32_t n_pending, n_inflight;
	bool busy;			/* a serialised entry point is running */
	bool dead;			/* shut down: submit fails */
	bool io_err;			/* sticky: ring/doorbell access failed */
	bool lost;			/* every reset level failed: submit -ENODEV until xsc_cp_bringup() succeeds */
	uint32_t epoch;			/* context epoch (a GPU-zone reset bumps it) */
	struct xsc_mmu_fault faults[XSC_FAULT_LOG];
	uint32_t fault_n;		/* serial of the newest fault record */
	struct xsc_incident inc;
	struct xsc_fence stub;		/* the always-signalled fence of "signaled" syncobjs */
	struct {
		uint64_t seq;
		int err;
	} hist[XSC_ERR_HIST];
	uint32_t hist_n;
	struct xsc_stats st;
	uint32_t pkt[XSC_JOB_MAX_DW];	/* scratch for the (single) pusher */
};

/* ---- setup ------------------------------------------------------------------ */

/* -EINVAL on a bad cfg / missing mandatory op. */
int xsc_init(struct xsc_core *c, const struct xsc_ops *ops, void *ctx, const struct xsc_cfg *cfg);
/* Every job must have retired (see xsc_abort_all).  Returns the number of
 * jobs still alive (0 = clean). */
uint32_t xsc_fini(struct xsc_core *c);

/* CP bring-up (worker context).  The ring cursors
 * are set to what CP_RING_RPTR / CP_RING_WPTR show after the boot (0 on a
 * fresh CP; the firmware core keeps WPTR across CORE_RESET); a ring that is not
 * empty is emptied with CP_CTRL.RESET.  In-flight jobs are replayed into the
 * new ring; clears io_err and lost.  0, -ETIMEDOUT (no FW_READY in 100 ms), -EIO
 * (BOOT_ERR / PANIC / ring not empty), -EPROTO (command-stream version), or the
 * fw_load error. */
int xsc_cp_bringup(struct xsc_core *c);

/* ---- ring entries ------------------------------------------------------------- */

/* Ring dwords of a job with `nib` command buffers. */
static inline uint32_t xsc_job_dw(uint32_t nib)
{
	return nib * XSC_INDIRECT_DW + XSC_FENCE_DW;
}
/* INDIRECT(va,size,vmid)... + FENCE(fence_pa, seq, BIT64|IRQ); returns the
 * dword count.  `out` has room for XSC_JOB_MAX_DW. */
uint32_t xsc_encode_job(const struct xsc_core *c, const struct xsc_job *j, uint32_t *out);

/* ---- queues, syncobjs, fences ---------------------------------------------- */

/* Allocation helpers (call without the lock; NULL = -ENOMEM / bad argument). */
struct xsc_queue *xsc_queue_alloc(struct xsc_core *c, uint32_t vmid);
int xsc_queue_free(struct xsc_core *c, struct xsc_queue *q);	/* -EBUSY while jobs are alive */
bool xsc_queue_idle(struct xsc_core *c, struct xsc_queue *q);	/* njobs == 0 */
/* Cancel the not-yet-pushed jobs of q (they retire with -ECANCELED). */
void xsc_queue_flush(struct xsc_core *c, struct xsc_queue *q);
/* HAL queue_get_status: guilty? */
bool xsc_queue_guilty(struct xsc_core *c, struct xsc_queue *q);

struct xsc_syncobj *xsc_syncobj_alloc(struct xsc_core *c, bool signaled);
void xsc_syncobj_get(struct xsc_core *c, struct xsc_syncobj *so);
void xsc_syncobj_put(struct xsc_core *c, struct xsc_syncobj *so);
/* All entries must be non-NULL (-ENOENT, nothing changed): drop the fences. */
int xsc_syncobj_reset(struct xsc_core *c, struct xsc_syncobj *const *so, uint32_t n);
/* Host signal: attach the already-signalled stub fence. */
int xsc_syncobj_signal(struct xsc_core *c, struct xsc_syncobj *const *so, uint32_t n);
/* Poll once, HAL nicogpu_syncobj_wait semantics: entries are checked in array
 * order (NULL -> -ENOENT, no fence -> -EINVAL unless WAIT_FOR_SUBMIT); 0 when
 * the wait is satisfied (any / all), -ETIME when not yet.  *first = index of
 * the first signalled entry (0 when none / WAIT_ALL), may be NULL.  IRQ-safe. */
int xsc_syncobj_wait_poll(struct xsc_core *c, struct xsc_syncobj *const *so, uint32_t n, uint32_t flags,
			  uint32_t *first);
bool xsc_syncobj_signaled(struct xsc_core *c, struct xsc_syncobj *so);
/* ABI v5 WAIT_AVAILABLE: the same poll, but the condition is "a fence is attached" instead of "the fence has
 * signalled" (submitted / host-signalled, not necessarily complete).  flags: XSC_WAIT_ALL only (XSC_WAIT_FOR_SUBMIT
 * is implied and ignored; anything else -EINVAL).  NULL entry -> -ENOENT, a fence-less entry is never an error.
 * 0 when satisfied, -ETIME when not yet; *first = index of the first available entry (0 when none), may be NULL.  IRQ-safe. */
int xsc_syncobj_wait_avail_poll(struct xsc_core *c, struct xsc_syncobj *const *so, uint32_t n, uint32_t flags,
				uint32_t *first);

/* Fences produced outside the ring (future: imported / cross-device): they
 * gate a job's dependencies until xsc_fence_signal().  A syncobj is attached
 * to one with xsc_syncobj_attach(). */
struct xsc_fence *xsc_fence_alloc_external(struct xsc_core *c);
void xsc_fence_put(struct xsc_core *c, struct xsc_fence *f);
void xsc_fence_signal(struct xsc_core *c, struct xsc_fence *f, int err);
void xsc_syncobj_attach(struct xsc_core *c, struct xsc_syncobj *so, struct xsc_fence *f);
/* dma_fence adapter support (nicogpu_fence.c): take a reference on a fence the caller can already reach, and look at its
 * state without a syncobj.  xsc_fence_poll(): true once the fence has signalled, *err (may be NULL) = its error.  Both
 * only read/count the fence under the core lock; neither changes any existing behaviour.  IRQ-safe. */
void xsc_fence_get(struct xsc_core *c, struct xsc_fence *f);
bool xsc_fence_poll(struct xsc_core *c, struct xsc_fence *f, int *err);

/* Wait-by-seqno (NICOGPU_IOC_FENCE_WAIT): 0 = job `seq` is retired (*err = its
 * error), -ETIME not yet, -EINVAL never issued.  IRQ-safe. */
int xsc_seq_status(struct xsc_core *c, uint64_t seq, int *err);

/* ---- submit ------------------------------------------------------------------ */

/* Job with room for `nib` command buffers and `nwaits` dependencies; the
 * caller fills job->ib[0..nib-1].  NULL on -ENOMEM or nib > XSC_MAX_IBS. */
struct xsc_job *xsc_job_alloc(struct xsc_core *c, uint32_t nib, uint32_t nwaits);
void xsc_job_free(struct xsc_core *c, struct xsc_job *j);	/* only if xsc_submit() failed */

/*
 * Validate and arm (sim_backend submit() order): queue guilty -> -ECANCELED;
 * cmdbufs (flags, size 0 or > 2^20, va%4, va >= 2^48) -> -EINVAL; waits
 * (unknown -> -ENOENT; flags/point/no-fence -> -EINVAL); signals (unknown ->
 * -ENOENT; flags/point/duplicate -> -EINVAL); ring too small for the job ->
 * -EINVAL; sticky io error -> -EIO; shut down or device lost -> -ENODEV.  On success the job
 * belongs to the core, every `signals` syncobj now holds the job's fence and
 * *seq is its sequence number.  On failure nothing changed and the caller
 * still owns the job.  Not IRQ context (takes the lock only).
 */
int xsc_submit(struct xsc_core *c, struct xsc_queue *q, struct xsc_job *j, const struct xsc_sync_ref *waits,
	       uint32_t nwaits, const struct xsc_sync_ref *signals, uint32_t nsignals, uint64_t *seq);

/* ---- run / complete / recover ------------------------------------------------- */

/* Push pending jobs into the ring while dependencies and space allow, ring
 * the doorbell once per job.  Worker only.  >= 0 jobs pushed; -EBUSY if
 * re-entered; -EIO if a ring/doorbell access failed (that job retires with
 * -EIO, the queue is NOT guilty, c->io_err is set). */
int xsc_run(struct xsc_core *c);

/* A fence IRQ (or any poll): read the fence page, retire jobs.  Returns the
 * number of jobs retired.  IRQ-safe. */
unsigned int xsc_fence_irq(struct xsc_core *c);
/* Fresh RPTR value from the write-back page / register.  IRQ-safe. */
void xsc_rptr_update(struct xsc_core *c, uint32_t rptr);

enum xsc_recover {
	XSC_RECOVER_NONE = 0,		/* nothing wrong */
	XSC_RECOVER_LOST_IRQ = 1,	/* completion found by polling (MSI lost/late); no failure */
	XSC_RECOVER_FAILED = 2,		/* the head job failed (guilty); no reset hook: CP untouched */
	XSC_RECOVER_RESET = 3,		/* failed + cp_reset ok + other jobs replayed */
	XSC_RECOVER_FULL = 4,		/* no job to blame (RING_CONFIG / stale CP error / firmware event): whole-device reset done */
};

/* Watchdog tick (worker): poll the fence page, then fail the in-flight head
 * job if it has been running longer than cfg.timeout_ms:
 * fence -EIO, queue GUILTY, call ops->cp_reset and replay.  Returns an
 * enum xsc_recover, or -EIO if cp_reset failed, -EBUSY if re-entered. */
int xsc_check_timeout(struct xsc_core *c);

/* CP_ERROR interrupt (worker): `error_rptr` = CP_ERROR_RPTR.  Attributes the
 * error to the job whose ring range contains it, same
 * handling as a timeout.  Not attributable (RING_CONFIG / stale): without
 * XSC_CFG_ESCALATE -ENOENT (the basic contract: the caller escalates); with it
 * the core resets the whole device itself (RING_CONFIG: rebuild the ring,
 * otherwise CP_CTRL.RESET first), blames no queue and returns
 * XSC_RECOVER_FULL.  `code` is the CP_ERROR value (kept in the queue's fault info). */
int xsc_cp_error(struct xsc_core *c, uint32_t error_rptr, uint32_t code);

/* Fail every job with `err` (device removal / fatal).  Worker only. */
void xsc_abort_all(struct xsc_core *c, int err);

/* ---- faults, reset ladder, lifecycle ---------------------------------- */

/* MMU_FAULT interrupt (worker): the glue read MMU_FAULT_STATUS / _VA / _PTE
 * (and W1Cs the status afterwards) and hands the values over.  The record is
 * attached to the next failing job whose queue VMID equals the fault's VMID
 * (xsc_cp_error / xsc_check_timeout / xsc_cp_fw_event within XSC_FAULT_MATCH_MS)
 * and shows up in that queue's xsc_queue_fault.  Call it BEFORE xsc_cp_error:
 * the fault is latched before the CP halts. */
void xsc_mmu_fault(struct xsc_core *c, uint32_t status, uint64_t va, uint64_t pte);

/* CP_FW interrupt (worker): CP_FW_STATUS.PANIC / BOOT_ERR means the firmware is
 * dead: the job at CP_RING_RPTR is blamed, CORE_RESET + the CP bring-up
 * again and the other jobs are replayed.  XSC_RECOVER_NONE when the status
 * change was benign (READY) or XSC_CFG_ESCALATE is off. */
int xsc_cp_fw_event(struct xsc_core *c);

/* Manual / debugfs reset starting at `level` (XSC_RST_CP: the full ladder,
 * XSC_RST_FW: firmware reload first, XSC_RST_GPU: GPU-zone reset).  Needs
 * XSC_CFG_ESCALATE (-EINVAL otherwise).  No queue is blamed.  Worker only.
 * XSC_RECOVER_FULL or -EIO (device lost).  On a lost device the start level is
 * at least XSC_RST_FW: a successful handshake clears `lost`. */
int xsc_force_reset(struct xsc_core *c, enum xsc_reset_level level);

/* The RM was replaced / reset behind our back and a ring or doorbell access
 * failed (c->io_err): restore (ops->rm_restore) and re-handshake.  0 when
 * there was nothing to do or it worked, else the error. */
int xsc_recover_io(struct xsc_core *c);

/* Queue GUILTY status including a lost context (GPU-zone reset); fills
 * *out with the reason (may be NULL).  true = guilty. */
bool xsc_queue_fault_info(struct xsc_core *c, struct xsc_queue *q, struct xsc_queue_fault *out);

/* Latest incident (devcoredump trigger bookkeeping): serial 0 = none yet. */
void xsc_last_incident(struct xsc_core *c, struct xsc_incident *out);
bool xsc_is_lost(struct xsc_core *c);

/* Consistent copy of everything a devcoredump needs from the core. */
#define XSC_SNAP_JOBS	24u
#define XSC_SNAP_IBS	XSC_INC_IBS
#define XSC_SNAP_HIST	8u
struct xsc_snap_job {
	uint64_t seq;
	uint32_t state;			/* enum xsc_job_state */
	uint32_t vmid, nib, dw;
	uint32_t ring_start, ring_end;
	int err, pre_err;
	uint64_t age_ms;		/* since it became the in-flight head, 0 if it is not */
	uint32_t nib_kept;
	uint64_t ib_va[XSC_SNAP_IBS];
	uint32_t ib_dw[XSC_SNAP_IBS];
	bool queue_guilty;
};
struct xsc_snap {
	uint64_t now_ms;
	struct xsc_cfg cfg;
	uint32_t ring_dw, wptr, wptr_db, rptr;
	uint64_t last_seq, flight_seq, observed, done_seq, fence_mem;
	uint32_t n_pending, n_inflight;
	bool busy, dead, lost, io_err;
	uint32_t epoch;
	struct xsc_stats st;
	struct xsc_incident inc;
	uint32_t nfaults;		/* valid entries of faults[], oldest first */
	struct xsc_mmu_fault faults[XSC_FAULT_LOG];
	uint32_t nhist;
	struct {
		uint64_t seq;
		int err;
	} hist[XSC_SNAP_HIST];		/* newest first */
	uint32_t njobs;			/* in-flight (oldest first) then pending; at most XSC_SNAP_JOBS */
	uint32_t jobs_total;		/* alive in total (njobs may be smaller) */
	struct xsc_snap_job jobs[XSC_SNAP_JOBS];
};
void xsc_snapshot(struct xsc_core *c, struct xsc_snap *out);

/* Number of jobs armed and not retired. */
uint32_t xsc_jobs_alive(struct xsc_core *c);
/* Jobs currently owned by the CP (for the glue's timer arming). */
uint32_t xsc_inflight(struct xsc_core *c);
/* Ring space in dwords as the core currently believes it (tests). */
uint32_t xsc_ring_free(const struct xsc_core *c);
/* Sequence number of the in-flight job whose ring range contains `rptr`:
 * 0, or -ENOENT. */
int xsc_seq_by_rptr(struct xsc_core *c, uint32_t rptr, uint64_t *seq);
/* No new submits (-ENODEV); pair with xsc_abort_all() + waiting for jobs == 0. */
void xsc_shutdown(struct xsc_core *c);

#ifdef __cplusplus
}
#endif

#endif /* NICOGPU_SCHED_CORE_H */
