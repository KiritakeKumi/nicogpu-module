// SPDX-License-Identifier: GPL-2.0
/*
 * nicogpu_sched.c -- kernel glue of the scheduler / synchronisation layer.
 *
 * All semantics (ring, fences, jobs, binary syncobjs, in-order scheduling with
 * dependency tracking, timeout / CP-error handling, CP bring-up) live in
 * nicogpu_sched_core.c, which is plain C and unit-tested on the host (also against
 * the real CP firmware core and the HAL "sim" backend).  This file supplies:
 *   - the core's environment (struct xsc_ops): kzalloc, a spinlock with IRQs
 *     off, ktime, the host-memory fence/RPTR page, BAR0 registers and the C3
 *     window for ring writes;
 *   - an ordered workqueue (one work item pushes jobs and handles CP errors,
 *     one delayed work is the job watchdog) -- the core's "single worker";
 *   - the MSI hook (fence retirement in hard-IRQ context, the rest deferred);
 *   - per-file queue / syncobj handle tables and the ioctls.
 *
 * Why not drm_sched: its dependency/timeout machinery is what nicogpu_sched_core.c re-implements in ~700 testable
 * lines.  The core's fences are exported as dma_fences (nicogpu_fence.c) and SUBMIT accepts DRM core
 * syncobj handles (NICOGPU_SYNC_F_DRM, binary or timeline); the original driver-private syncobj ioctls are kept
 * (deprecated).  The mapping to the drm_sched callbacks is in nicogpu_sched_core.h.
 *
 * It also adds, on the same worker: the MMU fault latch hand-over (before the CP error is attributed), the
 * firmware-event / I/O recovery entry points and manual reset requests, the real CP firmware loader
 * (request_firmware + xim_ngcf_check + the C3 window), the structured queue fault ioctl and the devcoredump trigger.
 * The reset ladder itself (CP_CTRL.RESET -> firmware reload -> GPU-zone reset -> device lost) is in the core.
 *
 * Not compiled on the host: checked only against stub headers.
 */
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/xarray.h>
#include <linux/dma-mapping.h>
#include <linux/property.h>
#include <linux/interrupt.h>
#include <linux/sched.h>	/* MAX_SCHEDULE_TIMEOUT */
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include <linux/devcoredump.h>
#include <linux/capability.h>
#include "nicogpu_drv.h"
#include "nicogpu_img.h"

static unsigned int sched_timeout_ms = 10000;
module_param(sched_timeout_ms, uint, 0444);
MODULE_PARM_DESC(sched_timeout_ms, "job timeout in ms (10000 suggested on FPGA)");
static unsigned int sched_max_job_ms = 60000;
module_param(sched_max_job_ms, uint, 0444);
MODULE_PARM_DESC(sched_max_job_ms, "a job that keeps progressing (RPTR / retired shader instructions) may run this long, "
		 "sched_timeout_ms then only bounds a period without progress; 0 = sched_timeout_ms is final");
static unsigned int sched_ring_log2 = 16;
module_param(sched_ring_log2, uint, 0444);
MODULE_PARM_DESC(sched_ring_log2, "command ring size: 1 << n dwords, 10..20 (default 16 = 256 KiB)");
static bool sched_fw_preloaded;
module_param(sched_fw_preloaded, bool, 0444);
MODULE_PARM_DESC(sched_fw_preloaded, "skip the CP firmware load (the RM carries its own image)");
static char sched_fw_name[64] = "nicogpu/cp_fw.bin";
module_param_string(sched_fw_name, sched_fw_name, sizeof(sched_fw_name), 0444);
MODULE_PARM_DESC(sched_fw_name, "request_firmware() name of the CP firmware image (NGCF)");
static bool sched_l2_keep_ctx;
module_param(sched_l2_keep_ctx, bool, 0444);
MODULE_PARM_DESC(sched_l2_keep_ctx, "after a GPU-zone reset keep the other queues alive and replay their jobs (default: they are lost)");

/* layout of the host status page (one dma_alloc_coherent page, PROVISIONAL) */
#define NICOGPU_PAGE_FENCE		0x00	/* u64: last completed sequence number (FENCE.BIT64 target, 8-aligned) */
#define NICOGPU_PAGE_RPTR		0x40	/* u32: CP_RPTR_WB target */
#define NICOGPU_C3_FW_RESERVE	SZ_1M	/* C3 space kept for the firmware image */
#define NICOGPU_DUMP_BYTES		(128 * 1024)	/* devcoredump text size limit */

/* bits of nicogpu_sched.events beyond the IRQ_STATUS bits (XSC_IRQ_*, 0..4) */
#define NICOGPU_EV_KICK		BIT(31)		/* retired jobs freed ring space: push more */
#define NICOGPU_EV_DUMP		BIT(30)		/* an incident happened: take a devcoredump */
#define NICOGPU_EV_RESET_REQ	BIT(29)		/* nicogpu_sched_reset() asked for a manual reset */
#define NICOGPU_EV_RECOVER_IO	BIT(28)		/* a ring / doorbell access failed: try to restore and re-handshake */

/* the UAPI and the core share one set of values (HAL v0) and one layout */
static_assert(sizeof(struct nicogpu_cmdbuf) == sizeof(struct xsc_ib) &&
	      offsetof(struct nicogpu_cmdbuf, va) == offsetof(struct xsc_ib, va) &&
	      offsetof(struct nicogpu_cmdbuf, size_dw) == offsetof(struct xsc_ib, size_dw) &&
	      offsetof(struct nicogpu_cmdbuf, flags) == offsetof(struct xsc_ib, flags), "cmdbuf == xsc_ib");
static_assert(NICOGPU_SUBMIT_MAX_CMDBUFS == XSC_MAX_IBS && NICOGPU_SUBMIT_MAX_SYNCS == XSC_MAX_SYNCS, "limits");
static_assert(XDD_SH_STATUS == SH_STATUS && XDD_SH_IRQ_STATUS == SH_IRQ_STATUS && XDD_SH_IRQ_ENABLE == SH_IRQ_ENABLE,
	      "nicogpu_dump_core.h shell register offsets == nicogpu_regs.h");
static_assert(NICOGPU_SYNCOBJ_WAIT_ALL == XSC_WAIT_ALL && NICOGPU_SYNCOBJ_WAIT_FOR_SUBMIT == XSC_WAIT_FOR_SUBMIT, "wait flags");

/* ---- struct xsc_ops ------------------------------------------------------- */

static struct nicogpu_sched *ctx_s(void *ctx)
{
	return ctx;
}

static struct nicogpu_dev *ctx_x(void *ctx)
{
	return ((struct nicogpu_sched *)ctx)->x;
}

static void *op_zalloc(void *ctx, size_t size)
{
	return kzalloc(size, GFP_KERNEL_ACCOUNT);
}

static void op_free(void *ctx, void *p)
{
	kfree(p);		/* may run under the spinlock / in IRQ context */
}

static void op_lock(void *ctx)
{
	struct nicogpu_sched *s = ctx_s(ctx);
	unsigned long flags;

	spin_lock_irqsave(&s->lock, flags);
	s->irqflags = flags;
}

static void op_unlock(void *ctx)
{
	struct nicogpu_sched *s = ctx_s(ctx);
	unsigned long flags = s->irqflags;

	spin_unlock_irqrestore(&s->lock, flags);
}

static u64 op_now_ms(void *ctx)
{
	return div_u64(ktime_get_ns(), NSEC_PER_MSEC);
}

/* the CP writes the 64-bit seqno with one posted PCIe write ;
 * little-endian host only, like the rest of the driver */
static u64 op_read_fence(void *ctx)
{
	return READ_ONCE(*(u64 *)(ctx_s(ctx)->page_cpu + NICOGPU_PAGE_FENCE));
}

static void op_notify(void *ctx)
{
	wake_up_all(&ctx_s(ctx)->wq);
}

static void op_event(void *ctx, enum xsc_event ev, u64 a, u64 b)
{
	struct device *d = &ctx_x(ctx)->pdev->dev;

	switch (ev) {
	case XSC_EV_JOB_FAILED:
		dev_warn(d, "job %llu failed (-%llu)\n", a, b);
		break;
	case XSC_EV_LOST_IRQ:
		dev_warn(d, "%llu job(s) found complete by polling: fence MSI lost or late\n", a);
		break;
	case XSC_EV_BAD_FENCE:
		dev_err(d, "fence page says %llu but only %llu was ever submitted\n", a, b);
		break;
	case XSC_EV_BAD_RPTR:
		dev_err(d, "implausible RPTR %llx (committed WPTR %llx)\n", a, b);
		break;
	case XSC_EV_REPLAY:
		dev_warn(d, "CP reset: %llu job(s) replayed\n", a);
		break;
	case XSC_EV_IO_ERROR:
		dev_err(d, "ring/doorbell access failed (%llu) for job %llu\n", a, b);
		break;
	case XSC_EV_CP_ERROR:
		dev_err(d, "CP error %llu attributed to job %llu\n", b, a);
		break;
	case XSC_EV_MMU_FAULT:		/* decoded and logged once by sched_read_mmu_fault */
		break;
	case XSC_EV_INCIDENT:
		/* under the core lock: only note it, the worker takes the dump (sleeping) */
		dev_warn(d, "incident: %s, job %llu\n", xdd_cause_name((u32)a), b);
		atomic_or(NICOGPU_EV_DUMP, &ctx_s(ctx)->events);
		if (READ_ONCE(ctx_s(ctx)->wq_run))
			queue_work(ctx_s(ctx)->wq_run, &ctx_s(ctx)->work);
		break;
	case XSC_EV_RESET:
		dev_warn(d, "reset level %llu (%s): %s\n", a, a == XSC_RST_CP ? "CP_CTRL.RESET" :
			 a == XSC_RST_FW ? "firmware reload" : "GPU zone", b ? "FAILED" : "ok");
		break;
	case XSC_EV_ABORT_TIMEOUT:
		dev_err(d, "CP_CTRL.RESET: dispatch abort timed out (SP fabric requests did not drain, CP_FW_STATUS %08llx%s), escalating to the GPU-zone reset\n",
			a, b ? "; CP not idle either" : "");
		break;
	case XSC_EV_DEVICE_LOST:
		dev_err(d, "every reset level failed (%llu): device lost, reload the driver or reconfigure the RM\n", a);
		nicogpu_note_lost(ctx_x(ctx), NICOGPU_LOST_RESET_FAILED, -(int)a);
		atomic_or(NICOGPU_EV_DUMP, &ctx_s(ctx)->events);
		if (READ_ONCE(ctx_s(ctx)->wq_run))
			queue_work(ctx_s(ctx)->wq_run, &ctx_s(ctx)->work);
		break;
	}
}

static int op_ring_write(void *ctx, u32 dw_idx, const u32 *dw, u32 n)
{
	return nicogpu_mm_c3_write(ctx_x(ctx), ctx_s(ctx)->ring_off + 4ULL * dw_idx, dw, n);
}

/* ring contents are posted BAR2 writes; the doorbell must not pass them
 * (the shell holds it back until the BAR2 writes have landed; wmb() is the CPU side).
 * The core rings the doorbell only right after the ring write of the same batch succeeded, and that write
 * (nicogpu_mm_c3_write -> win_alive) has just read RM_STATUS (not decoupled) and C3_WIN_CTRL: no second RM_STATUS
 * read here.  An RM decoupled in between drops the posted write (an isolated RM ignores writes); the job then times
 * out and the reset ladder / I/O recovery takes over, as for any doorbell lost on the way. */
static int op_doorbell(void *ctx, u32 wptr)
{
	struct nicogpu_dev *x = ctx_x(ctx);

	wmb();
	ng_wr(x, NICOGPU_BAR_CTRL, NICOGPU_REG_DOORBELL0, wptr);
	return 0;
}

static int op_read_rptr(void *ctx, u32 *rptr)
{
	*rptr = READ_ONCE(*(u32 *)(ctx_s(ctx)->page_cpu + NICOGPU_PAGE_RPTR));
	return 0;
}

static u32 op_reg_rd(void *ctx, u32 off)
{
	return ng_rd(ctx_x(ctx), NICOGPU_BAR_CTRL, off);
}

/* Progress of the head job for the timeout: the CP's RPTR, and on rm_sp the debug page's retired-instruction
 * count (COMMIT_TOTAL; it moves while a long dispatch runs, RPTR does not). */
static u64 op_progress(void *ctx)
{
	struct nicogpu_dev *x = ctx_x(ctx);
	u64 p = ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_CP_RING_RPTR);

	if (nicogpu_rm_kind(x) == NGI_RM_KIND_SP)
		p |= (u64)ng_rd(x, NICOGPU_BAR_CTRL, RSP_DBG_COMMIT_TOTAL) << 32;
	return p;
}

static int op_engine_busy(void *ctx)
{
	struct nicogpu_dev *x = ctx_x(ctx);

	if (nicogpu_rm_kind(x) != NGI_RM_KIND_SP)
		return 0;
	return !(ng_rd(x, NICOGPU_BAR_CTRL, RSP_DBG_SP_STATUS) & RSP_DBG_SP_IDLE);
}

static void op_reg_wr(void *ctx, u32 off, u32 val)
{
	ng_wr(ctx_x(ctx), NICOGPU_BAR_CTRL, off, val);
}

/* The CP firmware image is validated (xim_ngcf_check: header, CRC-32, command-stream
 * version) and written to the C3 front region at fw_off through the window.  C3 is static DDR: a later
 * handshake (firmware reload, GPU-zone reset) finds it still there and does not write it again. */
static int op_fw_load(void *ctx, u64 *base_pa, u32 *size)
{
	struct nicogpu_sched *s = ctx_s(ctx);
	struct nicogpu_dev *x = ctx_x(ctx);
	const struct firmware *fw;
	struct xim_ngcf info;
	int rc;

	if (sched_fw_preloaded)
		return XSC_FW_SKIP;
	if (s->fw_pa) {
		*base_pa = s->fw_pa;
		*size = s->fw_size;
		return 0;
	}
	rc = request_firmware(&fw, sched_fw_name, &x->pdev->dev);
	if (rc) {
		dev_err(&x->pdev->dev, "CP firmware \"%s\" not found (%d): install the Ibex image or set sched_fw_preloaded=1\n",
			sched_fw_name, rc);
		return rc;
	}
	rc = xim_ngcf_check(fw->data, fw->size, NICOGPU_C3_FW_RESERVE, XSC_CMD_MAJOR, XSC_CMD_MINOR_MIN, &info);
	if (rc) {
		dev_err(&x->pdev->dev, "CP firmware \"%s\" rejected (%d): not an NGCF image / CRC / command-stream version\n",
			sched_fw_name, rc);
		release_firmware(fw);
		return rc;
	}
	rc = nicogpu_mm_c3_write(x, s->fw_off, (const u32 *)fw->data, fw->size / 4);
	if (!rc) {
		s->fw_pa = NICOGPU_PA_SYS_BASE + s->fw_off;
		s->fw_size = fw->size;
		*base_pa = s->fw_pa;
		*size = fw->size;
		dev_info(&x->pdev->dev, "CP firmware %s: fw %08x cmd %08x, %zu bytes at C3 +0x%llx\n", sched_fw_name,
			 info.fw_version, info.cmd_version, fw->size, s->fw_off);
	}
	release_firmware(fw);
	return rc;
}

static int op_gpu_reset(void *ctx)
{
	return nicogpu_dfx_gpu_reset(ctx_x(ctx));
}

static int op_rm_restore(void *ctx)
{
	return nicogpu_mm_rm_restore(ctx_x(ctx));
}

/* The core runs the reset ladder itself (XSC_CFG_ESCALATE): no cp_reset hook, CP_CTRL is written through reg_wr. */
static const struct xsc_ops nicogpu_sched_ops = {
	.zalloc = op_zalloc,
	.free = op_free,
	.lock = op_lock,
	.unlock = op_unlock,
	.now_ms = op_now_ms,
	.read_fence = op_read_fence,
	.notify = op_notify,
	.event = op_event,
	.ring_write = op_ring_write,
	.doorbell = op_doorbell,
	.read_rptr = op_read_rptr,
	.cp_reset = NULL,
	.reg_rd = op_reg_rd,
	.reg_wr = op_reg_wr,
	.msleep = ng_op_msleep,
	.fw_load = op_fw_load,
	.gpu_reset = op_gpu_reset,
	.rm_restore = op_rm_restore,
	.progress = op_progress,
	.engine_busy = op_engine_busy,
};

/* ---- worker and watchdog ------------------------------------------------- */

/* a foreign dma_fence a job waits for signalled: the worker may be able to push it now */
static void fence_kick(void *ctx)
{
	struct nicogpu_sched *s = ctx;
	struct workqueue_struct *wq = READ_ONCE(s->wq_run);

	if (wq)
		queue_work(wq, &s->work);
}

static void sched_arm_watchdog(struct nicogpu_sched *s)
{
	unsigned int ms = sched_timeout_ms / 4;

	if (ms < 10)
		ms = 10;
	if (ms > 1000)
		ms = 1000;
	if (READ_ONCE(s->ready) && xsc_jobs_alive(&s->core))
		queue_delayed_work(s->wq_run, &s->dwork, msecs_to_jiffies(ms));
}

static void sched_kick(struct nicogpu_sched *s)
{
	queue_work(s->wq_run, &s->work);
	sched_arm_watchdog(s);
}

/* / The fault is latched before the CP halts.  Read the latch, give it to the core (it is
 * attached to the job the CP error / timeout blames), clear it (W1C VALID also clears MULTI). */
static void sched_read_mmu_fault(struct nicogpu_sched *s)
{
	struct nicogpu_dev *x = s->x;
	struct xsc_mmu_fault f;
	u32 st;
	u64 va, pte;

	if (!ng_rm_present(x))
		return;
	st = ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_MMU_FAULT_STATUS);
	if (st == 0xffffffffu || !(st & NICOGPU_REG_MMU_FAULT_STATUS_VALID_MASK))
		return;
	va = ((u64)ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_MMU_FAULT_VA_HI) << 32) | ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_MMU_FAULT_VA_LO);
	pte = ((u64)ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_MMU_FAULT_PTE_HI) << 32) |
	      ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_MMU_FAULT_PTE_LO);
	xsc_mmu_fault(&s->core, st, va, pte);
	if (xsc_mmu_fault_newest(&s->core, &f))		/* the core's decode: logged once, here */
		dev_err(&x->pdev->dev, "MMU fault: vmid %u va %llx %s level %u code %u pte %llx%s\n", f.vmid, f.va,
			f.access == 0 ? "read" : f.access == 1 ? "write" : "exec", f.level, f.code, f.pte,
			f.multi ? " (more faults dropped)" : "");
	ng_wr(x, NICOGPU_BAR_CTRL, NICOGPU_REG_MMU_FAULT_STATUS, NICOGPU_REG_MMU_FAULT_STATUS_VALID_MASK);
}

static void sched_log_recover(struct nicogpu_sched *s, const char *what, int rc)
{
	struct device *d = &s->x->pdev->dev;

	if (rc == XSC_RECOVER_RESET)
		dev_err(d, "%s: job failed, queue marked GUILTY, CP reset, other jobs replayed\n", what);
	else if (rc == XSC_RECOVER_FULL)
		dev_err(d, "%s: not attributable to a job: whole-device reset, nobody blamed\n", what);
	else if (rc == XSC_RECOVER_FAILED)
		dev_err(d, "%s: queue marked GUILTY; no reset configured, queued jobs will time out\n", what);
	else if (rc < 0 && rc != -EBUSY && rc != -ENOENT)
		dev_err(d, "%s: recovery failed (%d)%s\n", what, rc, xsc_is_lost(&s->core) ? ": device lost" : "");
}

static void sched_cp_error(struct nicogpu_sched *s)
{
	struct nicogpu_dev *x = s->x;
	u32 code, rptr, info, ibo;
	int rc;

	if (!ng_rm_present(x))
		return;
	code = ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_CP_ERROR);
	rptr = ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_CP_ERROR_RPTR);
	info = ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_CP_ERROR_INFO);
	ibo = ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_CP_ERROR_IB_OFFSET);
	dev_err(&x->pdev->dev, "CP error %u (%s) at RPTR %u (opcode %02x vmid %u in_ib %u ib_offset %u)\n", code,
		xdd_cp_error_name(code), rptr, info & NICOGPU_REG_CP_ERROR_INFO_OPCODE_MASK,
		(info & NICOGPU_REG_CP_ERROR_INFO_VMID_MASK) >> 8, !!(info & NICOGPU_REG_CP_ERROR_INFO_IN_IB_MASK), ibo);
	rc = xsc_cp_error(&s->core, rptr, code);
	sched_log_recover(s, "CP error", rc);
}

/* a ring / doorbell access failed (RM replaced / decoupled / window lost): once the RM is back, restore what only
 * the host knows and run the handshake again; rate limited, the watchdog and the next submit retry */
static void sched_try_io_recover(struct nicogpu_sched *s)
{
	int rc;

	if (!READ_ONCE(s->core.io_err) || !ng_rm_present(s->x) || time_before(jiffies, s->io_next))
		return;
	s->io_next = jiffies + HZ;
	rc = xsc_recover_io(&s->core);
	if (rc && rc != -EBUSY)
		dev_err(&s->x->pdev->dev, "I/O recovery failed (%d): retrying in 1 s\n", rc);
	else if (!rc)
		dev_info(&s->x->pdev->dev, "RM reachable again: CP handshake repeated, jobs replayed\n");
}

static void sched_do_reset_req(struct nicogpu_sched *s)
{
	u32 lvl = READ_ONCE(s->reset_level);
	int rc;

	if (!lvl)
		return;
	rc = xsc_force_reset(&s->core, (enum xsc_reset_level)lvl);
	s->reset_rc = rc > 0 ? 0 : rc;
	WRITE_ONCE(s->reset_level, 0);
	complete(&s->reset_done);
}

/* what both workers end with: retry a failed ring / doorbell access (io_err is sticky) and signal the dma_fences of the
 * jobs retired meanwhile */
static void sched_settle(struct nicogpu_sched *s)
{
	sched_try_io_recover(s);
	nicogpu_fences_harvest(&s->fences, false);
}

static void sched_work(struct work_struct *w)
{
	struct nicogpu_sched *s = container_of(w, struct nicogpu_sched, work);
	u32 ev = atomic_xchg(&s->events, 0);

	if (!READ_ONCE(s->ready))
		return;
	if (ev & (XSC_IRQ_CP_ERROR | XSC_IRQ_MMU_FAULT))
		sched_read_mmu_fault(s);		/* before the CP error: the fault belongs to the job it blames */
	if (ev & XSC_IRQ_CP_ERROR)
		sched_cp_error(s);
	if (ev & XSC_IRQ_CP_FW) {
		dev_info(&s->x->pdev->dev, "CP firmware status changed: %08x\n",
			 ng_rd(s->x, NICOGPU_BAR_CTRL, NICOGPU_REG_CP_FW_STATUS));
		sched_log_recover(s, "CP firmware", xsc_cp_fw_event(&s->core));
	}
	if (ev & NICOGPU_EV_RESET_REQ)
		sched_do_reset_req(s);
	if (ev & NICOGPU_EV_RECOVER_IO)
		s->io_next = jiffies;			/* a submit saw the failure: try now */
	(void)xsc_run(&s->core);
	sched_settle(s);				/* io_err may be set by the above or by a submit; jobs retired by the above */
	if (ev & NICOGPU_EV_DUMP)
		nicogpu_dump(s->x, xsc_is_lost(&s->core) ? XDD_R_DEVICE_LOST : XDD_R_INCIDENT);
	sched_arm_watchdog(s);
}

static void sched_timeout_work(struct work_struct *w)
{
	struct nicogpu_sched *s = container_of(to_delayed_work(w), struct nicogpu_sched, dwork);
	int rc;

	if (!READ_ONCE(s->ready))
		return;
	rc = xsc_check_timeout(&s->core);
	if (rc != XSC_RECOVER_NONE && rc != XSC_RECOVER_LOST_IRQ)
		sched_log_recover(s, "job timeout", rc);
	sched_settle(s);
	if (rc >= XSC_RECOVER_LOST_IRQ)
		queue_work(s->wq_run, &s->work);	/* freed ring space / replayed jobs */
	sched_arm_watchdog(s);
}

/* MSI (merged vector, hard IRQ): retire fences right here, defer the rest */
void nicogpu_sched_irq(struct nicogpu_dev *x, u32 rm)
{
	struct nicogpu_sched *s = &x->sched;
	u32 ev = rm & (XSC_IRQ_CP_ERROR | XSC_IRQ_MMU_FAULT | XSC_IRQ_CP_FW);

	if (!READ_ONCE(s->ready))
		return;
	if ((rm & XSC_IRQ_FENCE) && xsc_fence_irq(&s->core)) {
		nicogpu_fences_harvest(&s->fences, false);	/* hard IRQ: dma_fence_signal() is IRQ-safe */
		ev |= NICOGPU_EV_KICK;
	}
	if (ev) {
		atomic_or(ev, &s->events);
		queue_work(s->wq_run, &s->work);
	}
}

/* ---- bring-up / tear-down ------------------------------------------------- */

void nicogpu_sched_init(struct nicogpu_dev *x)
{
	struct nicogpu_sched *s = &x->sched;

	s->x = x;
	spin_lock_init(&x->lost.lock);
	spin_lock_init(&s->lock);
	init_waitqueue_head(&s->wq);
	mutex_init(&s->init_lock);
	mutex_init(&s->reset_lock);
	init_completion(&s->reset_done);
	xdd_limiter_init(&s->dump_lim, 0, 1000);	/* at most one devcoredump per second */
	atomic_set(&s->events, 0);
}

static void sched_free_page(struct nicogpu_sched *s)
{
	if (s->page_cpu)
		dma_free_coherent(&s->x->pdev->dev, PAGE_SIZE, s->page_cpu, s->page_dma);
	s->page_cpu = NULL;
}

/* init_lock held, !ready.  The C3 window must exist (RM with the v0.2 aperture)
 * and the CP firmware must come up. */
static int sched_bringup(struct nicogpu_dev *x)
{
	struct nicogpu_sched *s = &x->sched;
	struct xsc_cfg cfg = { };
	int rc;

	if (sched_ring_log2 < XSC_RING_LOG2_MIN || sched_ring_log2 > XSC_RING_LOG2_MAX)
		return -EINVAL;
	if (device_get_dma_attr(&x->pdev->dev) != DEV_DMA_COHERENT)
		/* The fence / RPTR page comes from dma_alloc_coherent(): on a non-coherent host that is an uncached
		 * mapping, so the CPU sees the CP's writes without snooping.  What a non-coherent host cannot give is
		 * cached GTT BOs (reported separately by the mm code); the CP itself works. */
		dev_warn_once(&x->pdev->dev, "PCIe host is not DMA-coherent: CP fence/RPTR page is uncached (dma_alloc_coherent)\n");
	rc = nicogpu_mm_c3_open(x);
	if (rc) {
		dev_err(&x->pdev->dev, "CP bring-up: the C3 window did not open (%d)\n", rc);
		return rc;
	}
	s->page_cpu = dma_alloc_coherent(&x->pdev->dev, PAGE_SIZE, &s->page_dma, GFP_KERNEL);
	if (!s->page_cpu)
		return -ENOMEM;
	if ((u64)s->page_dma + PAGE_SIZE > (1ULL << 47)) {	/* the shell issues 47-bit addresses */
		sched_free_page(s);
		return -ENOMEM;
	}
	xsc_c3_init(&s->c3, XSC_C3_FRONT_SIZE);
	rc = xsc_c3_alloc(&s->c3, 4ULL << sched_ring_log2, SZ_4K, &s->ring_off);
	if (!rc)
		rc = xsc_c3_alloc(&s->c3, NICOGPU_C3_FW_RESERVE, SZ_4K, &s->fw_off);
	if (rc) {
		sched_free_page(s);
		return -ENOSPC;
	}
	cfg.ring_log2 = sched_ring_log2;
	cfg.ring_pa = NICOGPU_PA_SYS_BASE + s->ring_off;
	cfg.fence_pa = NICOGPU_PA_HOST_BASE + s->page_dma + NICOGPU_PAGE_FENCE;
	cfg.rptr_wb_pa = NICOGPU_PA_HOST_BASE + s->page_dma + NICOGPU_PAGE_RPTR;
	cfg.timeout_ms = sched_timeout_ms ? sched_timeout_ms : 10000;
	cfg.max_job_ms = sched_max_job_ms && sched_max_job_ms > cfg.timeout_ms ? sched_max_job_ms : 0;
	cfg.flags = XSC_CFG_ESCALATE | (sched_l2_keep_ctx ? XSC_CFG_L2_KEEP_CTX : 0);
	rc = xsc_init(&s->core, &nicogpu_sched_ops, s, &cfg);
	if (rc) {
		sched_free_page(s);
		return rc;
	}
	s->wq_run = alloc_ordered_workqueue("nicogpu-sched", 0);
	if (!s->wq_run) {
		sched_free_page(s);
		return -ENOMEM;
	}
	INIT_WORK(&s->work, sched_work);
	INIT_DELAYED_WORK(&s->dwork, sched_timeout_work);
	atomic_set(&s->events, 0);
	rc = xsc_cp_bringup(&s->core);
	if (rc) {
		dev_err(&x->pdev->dev, "CP bring-up failed (%d): no command submission\n", rc);
		destroy_workqueue(s->wq_run);
		s->wq_run = NULL;
		sched_free_page(s);
		return rc == -ETIMEDOUT || rc == -EPROTO ? -ENODEV : rc;
	}
	nicogpu_fences_init(&s->fences, &s->core, fence_kick, s);
	WRITE_ONCE(s->ready, true);
	dev_info(&x->pdev->dev, "CP up: ring %u dwords at C3 +0x%llx, fence/RPTR page %llx, job timeout %u ms (max %u ms "
		 "while progressing)\n", 1u << sched_ring_log2, s->ring_off, (u64)s->page_dma, cfg.timeout_ms,
		 cfg.max_job_ms);
	return 0;
}

static int sched_ensure(struct nicogpu_dev *x)
{
	struct nicogpu_sched *s = &x->sched;
	int rc = 0;

	if (READ_ONCE(s->ready))
		return 0;
	mutex_lock(&s->init_lock);
	if (!READ_ONCE(s->ready))
		rc = sched_bringup(x);
	mutex_unlock(&s->init_lock);
	return rc;
}

void nicogpu_sched_fini(struct nicogpu_dev *x)
{
	struct nicogpu_sched *s = &x->sched;
	int i;

	if (!READ_ONCE(s->ready))
		return;
	WRITE_ONCE(s->ready, false);
	for (i = 0; i < x->nvec; i++)
		synchronize_irq(pci_irq_vector(x->pdev, i));	/* no ISR still inside xsc_fence_irq */
	xsc_shutdown(&s->core);
	nicogpu_fences_stop_deps(&s->fences);		/* no foreign-fence callback may touch the core / the work queue afterwards */
	cancel_delayed_work_sync(&s->dwork);
	destroy_workqueue(s->wq_run);			/* drains the work item */
	s->wq_run = NULL;
	xsc_abort_all(&s->core, -EIO);			/* open files were torn down first: nothing should be left */
	nicogpu_fences_harvest(&s->fences, true);		/* the aborted jobs' dma_fences signal with -ENODEV */
	nicogpu_fences_fini(&s->fences);
	if (ng_rm_present(x)) {
		ng_wr(x, NICOGPU_BAR_CTRL, NICOGPU_REG_IRQ_ENABLE, 0);
		ng_wr(x, NICOGPU_BAR_CTRL, NICOGPU_REG_CP_CTRL, NICOGPU_REG_CP_CTRL_CORE_RESET_MASK);
	}
	WARN_ON(xsc_fini(&s->core));
	sched_free_page(s);
}

/* ---- per-file tables ------------------------------------------------------ */

void nicogpu_sched_open(struct nicogpu_file *xf)
{
	xa_init_flags(&xf->queues, XA_FLAGS_ALLOC1);
	xa_init_flags(&xf->syncobjs, XA_FLAGS_ALLOC1);
}

/* cancel what has not reached the ring, wait for the rest (bounded by the
 * watchdog), free.  Process context, no locks. */
static void queue_drain_free(struct nicogpu_dev *x, struct nicogpu_queue *xq)
{
	struct nicogpu_sched *s = &x->sched;

	xsc_queue_flush(&s->core, xq->q);
	sched_kick(s);
	wait_event(s->wq, xsc_queue_idle(&s->core, xq->q));
	xsc_queue_free(&s->core, xq->q);
	nicogpu_fctx_put(xq->fctx);	/* fences of the timeline keep it alive */
	kfree(xq);
}

void nicogpu_sched_file_fini(struct nicogpu_file *xf)
{
	struct nicogpu_sched *s = &xf->x->sched;
	struct nicogpu_queue *xq;
	struct xsc_syncobj *so;
	unsigned long idx;

	/* queues first: their jobs hold fences the syncobjs reference */
	xa_for_each(&xf->queues, idx, xq) {
		xa_erase(&xf->queues, idx);
		queue_drain_free(xf->x, xq);
	}
	xa_destroy(&xf->queues);
	xa_for_each(&xf->syncobjs, idx, so) {
		xa_erase(&xf->syncobjs, idx);
		xsc_syncobj_put(&s->core, so);
	}
	xa_destroy(&xf->syncobjs);
}

/* xf->lock held.  A queue QUEUE_DESTROY unlinked but is still draining counts: its jobs may still run in the VM. */
bool nicogpu_sched_vm_in_use(struct nicogpu_file *xf, u32 vmid)
{
	struct nicogpu_queue *xq;
	unsigned long idx;

	if (vmid < NICOGPU_NUM_VMIDS && xf->vm_draining[vmid])
		return true;
	xa_for_each(&xf->queues, idx, xq)
		if (xq->q->vmid == vmid)
			return true;
	return false;
}

/* ---- ioctls ------------------------------------------------------------- */

static struct xsc_core *core_of(struct nicogpu_file *xf)
{
	return &xf->x->sched.core;
}

long nicogpu_ioc_queue_create(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_queue_create *a = data;
	struct nicogpu_queue *xq = NULL;
	u32 id = 0;
	long rc;

	if (a->flags || a->rsvd[0] || a->rsvd[1])
		return -EINVAL;
	rc = sched_ensure(xf->x);
	if (rc)
		return rc;
	mutex_lock(&xf->lock);
	if (a->vm_id == 0 || a->vm_id >= NICOGPU_NUM_VMIDS || !xf->vms[a->vm_id])
		rc = -ENOENT;		/* HAL order: VM first ... */
	else if (a->priority != NICOGPU_QUEUE_PRIORITY_NORMAL)
		rc = -EINVAL;		/* ... then priority */
	else if (!(xq = kzalloc(sizeof(*xq), GFP_KERNEL_ACCOUNT)))
		rc = -ENOMEM;
	else if (!(xq->q = xsc_queue_alloc(core_of(xf), a->vm_id)) || !(xq->fctx = nicogpu_fctx_create()))
		rc = -ENOMEM;
	else
		rc = xa_alloc(&xf->queues, &id, xq, XA_LIMIT(1, 0xffff), GFP_KERNEL_ACCOUNT);
	if (!rc)
		snprintf(xq->fctx->name, sizeof(xq->fctx->name), "nicogpu-q%u", id);	/* no fence exists yet */
	mutex_unlock(&xf->lock);
	if (rc) {
		if (xq) {
			if (xq->q)
				xsc_queue_free(core_of(xf), xq->q);
			if (xq->fctx)
				nicogpu_fctx_put(xq->fctx);
			kfree(xq);
		}
		return rc == -EBUSY ? -ENOSPC : rc;
	}
	a->queue_id = id;
	return 0;
}

long nicogpu_ioc_queue_destroy(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_queue_destroy *a = data;
	struct nicogpu_queue *xq;
	u32 vmid = 0;

	if (a->pad)
		return -EINVAL;
	mutex_lock(&xf->lock);
	xq = xa_erase(&xf->queues, a->queue_id);	/* no new submit can find it any more */
	if (xq) {
		vmid = xq->q->vmid;
		xf->vm_draining[vmid]++;		/* its VM stays in use until the jobs are gone */
	}
	mutex_unlock(&xf->lock);
	if (!xq)
		return -ENOENT;
	queue_drain_free(xf->x, xq);
	mutex_lock(&xf->lock);
	xf->vm_draining[vmid]--;
	mutex_unlock(&xf->lock);
	return 0;
}

long nicogpu_ioc_queue_status(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_queue_status *a = data;
	struct nicogpu_queue *xq;
	long rc = 0;

	if (a->rsvd)
		return -EINVAL;
	mutex_lock(&xf->lock);
	xq = xa_load(&xf->queues, a->queue_id);
	if (xq)
		a->status = xsc_queue_guilty(core_of(xf), xq->q) ? NICOGPU_QUEUE_STATUS_GUILTY : NICOGPU_QUEUE_STATUS_OK;
	else
		rc = -ENOENT;
	mutex_unlock(&xf->lock);
	if (rc)
		return rc;
	return 0;
}

long nicogpu_ioc_syncobj_create(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_syncobj_create *a = data;
	struct xsc_syncobj *so;
	u32 id;
	long rc;

	if ((a->flags & ~NICOGPU_SYNCOBJ_CREATE_SIGNALED) || a->rsvd)
		return -EINVAL;
	rc = sched_ensure(xf->x);
	if (rc)
		return rc;
	so = xsc_syncobj_alloc(core_of(xf), a->flags & NICOGPU_SYNCOBJ_CREATE_SIGNALED);
	if (!so)
		return -ENOMEM;
	mutex_lock(&xf->lock);
	rc = xa_alloc(&xf->syncobjs, &id, so, XA_LIMIT(1, 0xffffff), GFP_KERNEL_ACCOUNT);
	mutex_unlock(&xf->lock);
	if (rc) {
		xsc_syncobj_put(core_of(xf), so);
		return rc == -EBUSY ? -ENOSPC : rc;
	}
	a->handle = id;
	return 0;
}

long nicogpu_ioc_syncobj_destroy(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_syncobj_destroy *a = data;
	struct xsc_syncobj *so;

	if (a->pad)
		return -EINVAL;
	mutex_lock(&xf->lock);
	so = xa_erase(&xf->syncobjs, a->handle);
	mutex_unlock(&xf->lock);
	if (!so)
		return -ENOENT;
	xsc_syncobj_put(core_of(xf), so);	/* waiters / jobs keep it alive through their own references */
	return 0;
}

/* RESET and SIGNAL: same argument shape, all-or-nothing handle resolution */
static long syncobj_array_op(struct nicogpu_file *xf, void *data, bool reset)
{
	struct nicogpu_syncobj_array *a = data;
	struct xsc_syncobj *so[NICOGPU_SUBMIT_MAX_SYNCS];
	u32 h[NICOGPU_SUBMIT_MAX_SYNCS];
	long rc;
	u32 i;

	if (a->flags || a->count > NICOGPU_SUBMIT_MAX_SYNCS)
		return -EINVAL;
	if (!a->count)
		return 0;
	if (copy_from_user(h, u64_to_user_ptr(a->handles), a->count * sizeof(h[0])))
		return -EFAULT;
	mutex_lock(&xf->lock);
	for (i = 0; i < a->count; i++)
		so[i] = xa_load(&xf->syncobjs, h[i]);	/* NULL: unknown handle, the core says -ENOENT */
	rc = reset ? xsc_syncobj_reset(core_of(xf), so, a->count) : xsc_syncobj_signal(core_of(xf), so, a->count);
	mutex_unlock(&xf->lock);
	return rc;
}

/* relative timeout -> jiffies for wait_event_*_timeout; 0 = poll */
static long timeout_jiffies(s64 timeout_ns)
{
	if (timeout_ns == NICOGPU_TIMEOUT_INFINITE)
		return MAX_SCHEDULE_TIMEOUT;
	if (timeout_ns == 0)
		return 0;
	return nsecs_to_jiffies(timeout_ns) + 1;
}

/* Poll once with `call` (0, -ETIME or an error); while it says -ETIME and `left` jiffies were allowed, sleep on s->wq and
 * poll again.  rc ends up 0 / an error / -ETIME (time ran out) / -ERESTARTSYS (signal). */
#define sched_poll_wait(s, rc, left, call) \
	do { \
		(rc) = (call); \
		if ((rc) == -ETIME && (left)) { \
			long t_ = wait_event_interruptible_timeout((s)->wq, ((rc) = (call)) != -ETIME, (left)); \
\
			if (t_ < 0) \
				(rc) = t_; \
		} \
	} while (0)

/* WAIT_AVAILABLE (ABI v5) waits for "a fence is attached", the other modes for "the fence has signalled" */
static int syncobj_poll(struct xsc_core *c, struct xsc_syncobj *const *so, u32 n, u32 flags, u32 *first)
{
	if (flags & NICOGPU_SYNCOBJ_WAIT_AVAILABLE)
		return xsc_syncobj_wait_avail_poll(c, so, n, flags & ~NICOGPU_SYNCOBJ_WAIT_AVAILABLE, first);
	return xsc_syncobj_wait_poll(c, so, n, flags, first);
}

long nicogpu_ioc_syncobj_wait(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_syncobj_wait *a = data;
	struct nicogpu_sched *s = &xf->x->sched;
	struct xsc_syncobj *so[NICOGPU_SUBMIT_MAX_SYNCS];
	u32 h[NICOGPU_SUBMIT_MAX_SYNCS];
	u32 first = 0, i;
	long rc, left;

	if (a->pad || a->rsvd || a->timeout_ns < 0 ||
	    (a->flags & ~(NICOGPU_SYNCOBJ_WAIT_ALL | NICOGPU_SYNCOBJ_WAIT_FOR_SUBMIT | NICOGPU_SYNCOBJ_WAIT_AVAILABLE)) ||
	    !a->count || a->count > NICOGPU_SUBMIT_MAX_SYNCS)
		return -EINVAL;
	if (copy_from_user(h, u64_to_user_ptr(a->handles), a->count * sizeof(h[0])))
		return -EFAULT;
	mutex_lock(&xf->lock);
	for (i = 0; i < a->count; i++) {
		so[i] = xa_load(&xf->syncobjs, h[i]);
		if (so[i])
			xsc_syncobj_get(&s->core, so[i]);	/* survive a concurrent SYNCOBJ_DESTROY */
	}
	mutex_unlock(&xf->lock);

	left = timeout_jiffies(a->timeout_ns);
	sched_poll_wait(s, rc, left, syncobj_poll(&s->core, so, a->count, a->flags, &first));
	for (i = 0; i < a->count; i++)
		if (so[i])
			xsc_syncobj_put(&s->core, so[i]);
	if (rc)
		return rc;
	a->first_signaled = first;
	return 0;
}

/* Per sync entry state of a SUBMIT.  The core syncobj an entry resolves to lives in refs[i].so, the array xsc_submit()
 * takes: for a DRM entry (NICOGPU_SYNC_F_DRM) a temporary one this submit owns (waits: it carries the dependency's
 * core fence; signals: the placeholder the core arms), for a driver-private handle the handle's own (borrowed, looked
 * up under xf->lock).  ent[] holds what only DRM signal entries need. */
struct sub_ent {
	struct drm_syncobj *so;		/* signals: the DRM syncobj (reference) */
	struct dma_fence_chain *chain;	/* signals with a timeline point: preallocated node */
};

/* sync entries up to this many live on the stack (8 x 56 bytes); more come from one allocation */
#define SUB_STACK_SYNCS		8

static void sub_ents_free(struct nicogpu_sched *s, const struct nicogpu_sync *sy, struct xsc_sync_ref *refs,
			  struct sub_ent *ent, u32 n)
{
	u32 i;

	for (i = 0; i < n; i++) {
		if ((sy[i].flags & NICOGPU_SYNC_F_DRM) && refs[i].so)
			xsc_syncobj_put(&s->core, refs[i].so);	/* ours; a private handle's syncobj is only borrowed */
		if (ent[i].so)
			drm_syncobj_put(ent[i].so);
		if (ent[i].chain)
			dma_fence_chain_free(ent[i].chain);
	}
}

/* Resolve the DRM entries BEFORE any lock is taken (drm_syncobj_find_fence() must not be called with locks held).
 * Semantic failures (unknown handle, no fence yet) are not reported here: the entry becomes a NULL / fence-less core
 * syncobj and xsc_submit() reports -ENOENT / -EINVAL at the position HAL order puts them.  Only -ENOMEM and the like
 * abort early. */
static long sub_resolve(struct nicogpu_file *xf, const struct nicogpu_sync *sy, struct xsc_sync_ref *refs,
			struct sub_ent *ent, u32 nwaits, u32 nsync)
{
	struct nicogpu_sched *s = &xf->x->sched;
	u32 i, k;

	for (i = 0; i < nsync; i++) {
		if (!(sy[i].flags & NICOGPU_SYNC_F_DRM))
			continue;
		if (i < nwaits) {
			struct dma_fence *f = NULL;
			int rc = drm_syncobj_find_fence(xf->file, sy[i].handle, sy[i].point, 0, &f);

			if (rc == -ENOENT) {
				continue;		/* so == NULL: -ENOENT from the core */
			} else if (rc == -EINVAL) {	/* no fence (yet): -EINVAL from the core */
				refs[i].so = xsc_syncobj_alloc(&s->core, false);
				if (!refs[i].so)
					return -ENOMEM;
			} else if (rc) {
				return rc;
			} else {
				struct xsc_syncobj *so = nicogpu_fence_dep(&s->fences, f);

				dma_fence_put(f);
				if (IS_ERR(so))
					return PTR_ERR(so);
				refs[i].so = so;
			}
		} else {
			struct drm_syncobj *so = drm_syncobj_find(xf->file, sy[i].handle);

			if (!so)
				continue;		/* so == NULL: -ENOENT from the core */
			ent[i].so = so;
			if (sy[i].point) {
				ent[i].chain = dma_fence_chain_alloc();
				if (!ent[i].chain)
					return -ENOMEM;
			}
			/* the same syncobj twice shares one placeholder, so the core's duplicate check sees it */
			for (k = nwaits; k < i; k++) {
				if (ent[k].so == so) {
					xsc_syncobj_get(&s->core, refs[k].so);
					refs[i].so = refs[k].so;
					break;
				}
			}
			if (!refs[i].so) {
				refs[i].so = xsc_syncobj_alloc(&s->core, false);
				if (!refs[i].so)
					return -ENOMEM;
			}
		}
	}
	return 0;
}

long nicogpu_ioc_submit(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_submit *a = data;
	struct nicogpu_sched *s = &xf->x->sched;
	struct nicogpu_sync sy_stack[SUB_STACK_SYNCS];
	struct xsc_sync_ref refs_stack[SUB_STACK_SYNCS];
	struct sub_ent ent_stack[SUB_STACK_SYNCS];
	struct nicogpu_sync *sy = sy_stack;
	struct xsc_sync_ref *refs = refs_stack;
	struct sub_ent *ent = ent_stack;
	void *blk = NULL;
	struct xsc_job *job;
	struct nicogpu_queue *xq;
	struct nicogpu_fence *fw = NULL;
	u64 seq = 0;
	u32 nsync, i;
	long rc;

	if (a->flags || a->pad || a->rsvd[0] || a->rsvd[1] || a->num_cmdbufs > NICOGPU_SUBMIT_MAX_CMDBUFS ||
	    a->num_waits > NICOGPU_SUBMIT_MAX_SYNCS || a->num_signals > NICOGPU_SUBMIT_MAX_SYNCS)
		return -EINVAL;
	if (!READ_ONCE(s->ready))
		return -ENOENT;		/* no queue can exist */
	nsync = a->num_waits + a->num_signals;
	/* the job and its core fence are one allocation (xsc_job_alloc) */
	job = xsc_job_alloc(&s->core, a->num_cmdbufs, a->num_waits);
	if (!job)
		return -ENOMEM;
	if (nsync > SUB_STACK_SYNCS) {
		/* one block: refs[] | ent[] | sy[] (refs and ent start zeroed, sy is copied in) */
		blk = kzalloc(nsync * (sizeof(*refs) + sizeof(*ent) + sizeof(*sy)), GFP_KERNEL_ACCOUNT);
		if (!blk) {
			xsc_job_free(&s->core, job);
			return -ENOMEM;
		}
		refs = blk;
		ent = (struct sub_ent *)(refs + nsync);
		sy = (struct nicogpu_sync *)(ent + nsync);
	} else {
		memset(refs_stack, 0, sizeof(refs_stack));
		memset(ent_stack, 0, sizeof(ent_stack));
	}
	if (a->num_cmdbufs &&
	    copy_from_user(job->ib, u64_to_user_ptr(a->cmdbufs), a->num_cmdbufs * sizeof(struct nicogpu_cmdbuf))) {
		rc = -EFAULT;
		nsync = 0;		/* nothing resolved yet: nothing to release */
		goto err;
	}
	if (nsync) {
		if ((a->num_waits &&
		     copy_from_user(sy, u64_to_user_ptr(a->waits), a->num_waits * sizeof(*sy))) ||
		    (a->num_signals &&
		     copy_from_user(sy + a->num_waits, u64_to_user_ptr(a->signals), a->num_signals * sizeof(*sy)))) {
			rc = -EFAULT;
			nsync = 0;	/* sy[] may be partly garbage, nothing resolved yet: nothing to release */
			goto err;
		}
		rc = sub_resolve(xf, sy, refs, ent, a->num_waits, nsync);
		if (rc)
			goto err;
	}
	/* every job gets a dma_fence (the later dma_resv attach point); one kmalloc per job (a dma_fence is freed
	 * through RCU on its own, it cannot share the job's block).  Pin the core fence before the job is armed (it may
	 * retire at once).  Without a DRM signal entry nobody installs it: the pending list holds the only reference
	 * until the harvester signals it. */
	fw = nicogpu_fence_prepare(&s->fences, job->fence);
	if (!fw) {
		rc = -ENOMEM;
		goto err;
	}
	mutex_lock(&xf->lock);
	xq = xa_load(&xf->queues, a->queue_id);
	if (!xq) {
		rc = -ENOENT;
	} else {
		for (i = 0; i < nsync; i++) {
			if (sy[i].flags & NICOGPU_SYNC_F_DRM) {
				/* refs[i].so: resolved above (NULL: unknown handle) */
				refs[i].flags = sy[i].flags & ~NICOGPU_SYNC_F_DRM;	/* unknown bits: -EINVAL from the core */
				refs[i].point = 0;
			} else {
				refs[i].so = xa_load(&xf->syncobjs, sy[i].handle);	/* NULL: -ENOENT in the core */
				refs[i].flags = sy[i].flags;
				refs[i].point = sy[i].point;
			}
		}
		rc = xsc_submit(&s->core, xq->q, job, refs, a->num_waits, refs + a->num_waits, a->num_signals, &seq);
		if (!rc)
			nicogpu_fence_arm(&s->fences, fw, xq->fctx);	/* per-queue seqno: serialised by xf->lock */
	}
	mutex_unlock(&xf->lock);
	if (rc) {
		if (rc == -EIO && READ_ONCE(s->ready)) {	/* the CP / RM was unreachable: let the worker restore it */
			atomic_or(NICOGPU_EV_RECOVER_IO, &s->events);
			queue_work(s->wq_run, &s->work);
		}
		goto err;
	}
	/* The job is armed.  Publish its fence in the DRM syncobjs (neither call can fail; the chain node is consumed). */
	for (i = a->num_waits; i < nsync; i++) {
		if (!ent[i].so)
			continue;
		if (sy[i].point) {
			drm_syncobj_add_point(ent[i].so, ent[i].chain, &fw->base, sy[i].point);
			ent[i].chain = NULL;
		} else {
			drm_syncobj_replace_fence(ent[i].so, &fw->base);
		}
	}
	dma_fence_put(&fw->base);	/* our reference; the syncobjs (if any) and the pending list hold theirs */
	sub_ents_free(s, sy, refs, ent, nsync);
	kfree(blk);
	sched_kick(s);
	a->fence_seq = seq;	/* the job is armed whatever happens to the copy-out */
	return 0;
err:
	if (fw)
		nicogpu_fence_discard(&s->fences, fw);
	sub_ents_free(s, sy, refs, ent, nsync);
	xsc_job_free(&s->core, job);
	kfree(blk);
	return rc;
}

long nicogpu_ioc_fence_wait(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_fence_wait *a = data;
	struct nicogpu_sched *s = &xf->x->sched;
	int err = 0;
	long rc, left;

	if (a->flags || a->rsvd[0] || a->rsvd[1] || a->timeout_ns < 0)
		return -EINVAL;
	if (!READ_ONCE(s->ready))
		return -EINVAL;		/* nothing was ever submitted */
	left = timeout_jiffies(a->timeout_ns);
	sched_poll_wait(s, rc, left, xsc_seq_status(&s->core, a->seq, &err));
	if (rc)
		return rc;
	a->status = err;
	return 0;
}

/* ---- device status (ABI v5) ------------------------------------------------- */

/* Called when the whole device is declared lost: the scheduler worker (reset ladder exhausted) or the DFX flow (PR
 * wedged).  Any context that may take a spinlock with IRQs off. */
void nicogpu_note_lost(struct nicogpu_dev *x, u32 reason, int err)
{
	unsigned long flags;

	spin_lock_irqsave(&x->lost.lock, flags);
	x->lost.reason = reason;
	x->lost.err = err;
	x->lost.ns = ktime_get_ns();
	x->lost.count++;
	spin_unlock_irqrestore(&x->lost.lock, flags);
}

long nicogpu_ioc_dev_status(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_dev *x = xf->x;
	struct nicogpu_sched *s = &x->sched;
	const struct nicogpu_dev_status *in = data;
	struct nicogpu_dev_status a = { };
	unsigned long flags;
	u32 st = 0;

	if (in->flags || in->rsvd[0] || in->rsvd[1])
		return -EINVAL;
	/* the core is only initialised once a queue / syncobj ioctl brought the CP up */
	if (READ_ONCE(s->ready)) {
		op_lock(s);
		if (s->core.lost)
			st |= NICOGPU_DEVSTAT_LOST;
		if (s->core.io_err)
			st |= NICOGPU_DEVSTAT_IO_ERROR;
		a.resets = (u32)s->core.st.resets;
		a.gpu_resets = (u32)s->core.st.reset_gpu;
		op_unlock(s);
		st |= NICOGPU_DEVSTAT_CP_READY;
	}
	if (READ_ONCE(x->dfx.core.wedged))
		st |= NICOGPU_DEVSTAT_PR_WEDGED | NICOGPU_DEVSTAT_LOST;
	a.status = st;
	spin_lock_irqsave(&x->lost.lock, flags);
	a.lost_reason = x->lost.reason;
	a.lost_err = x->lost.err;
	a.lost_time_ns = x->lost.ns;
	a.lost_count = x->lost.count;
	spin_unlock_irqrestore(&x->lost.lock, flags);
	a.now_ns = ktime_get_ns();
	*(struct nicogpu_dev_status *)data = a;
	return 0;
}

/* What a GUILTY queue knows about why. */
long nicogpu_ioc_queue_fault(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_queue_fault *a = data;
	struct nicogpu_sched *s = &xf->x->sched;
	struct xsc_queue_fault qf = { };
	struct nicogpu_queue *xq;
	long rc = 0;
	bool guilty = false;
	u32 resets = 0, gpu_resets = 0;
	bool lost = false;

	if (a->rsvd[0] || a->rsvd[1])
		return -EINVAL;
	mutex_lock(&xf->lock);
	xq = xa_load(&xf->queues, a->queue_id);
	if (!xq) {
		rc = -ENOENT;
	} else {
		guilty = xsc_queue_fault_info(&s->core, xq->q, &qf);
		op_lock(s);
		resets = (u32)s->core.st.resets;
		gpu_resets = (u32)s->core.st.reset_gpu;
		lost = s->core.lost;
		op_unlock(s);
	}
	mutex_unlock(&xf->lock);
	if (rc)
		return rc;
	a->status = guilty ? NICOGPU_QUEUE_STATUS_GUILTY : NICOGPU_QUEUE_STATUS_OK;
	a->cause = qf.cause;			/* XSC_CAUSE_* and NICOGPU_FAULT_CAUSE_* are the same numbers */
	a->cp_error = qf.cp_error;
	a->err = qf.err;
	a->fence_seq = qf.seq;
	a->flags = lost ? NICOGPU_FAULT_F_DEVICE_LOST : 0;
	if (guilty && qf.has_mmu) {
		a->flags |= NICOGPU_FAULT_F_MMU_VALID | (qf.mmu.multi ? NICOGPU_FAULT_F_MMU_MULTI : 0);
		a->mmu_code = qf.mmu.code;
		a->mmu_access = qf.mmu.access;
		a->mmu_level = qf.mmu.level;
		a->mmu_vmid = qf.mmu.vmid;
		a->mmu_va = qf.mmu.va;
		a->mmu_pte = qf.mmu.pte;
	}
	a->resets = resets;
	a->gpu_resets = gpu_resets;
	return 0;
}

long nicogpu_ioc_reset(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_reset *a = data;
	long rc;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	if (a->flags || a->result || a->pad || a->rsvd[0] || a->rsvd[1] || a->level < NICOGPU_RESET_CP || a->level > NICOGPU_RESET_GPU)
		return -EINVAL;
	rc = nicogpu_sched_reset(xf->x, a->level);
	a->result = rc;
	return 0;			/* the outcome of the reset itself is in `result` */
}

long nicogpu_ioc_syncobj_reset(struct nicogpu_file *xf, void *data)
{
	return syncobj_array_op(xf, data, true);
}

long nicogpu_ioc_syncobj_signal(struct nicogpu_file *xf, void *data)
{
	return syncobj_array_op(xf, data, false);
}

/* ---- manual reset, PR hook, devcoredump ------------------------------------------- */

/* Runs the reset on the scheduler worker (it must not overlap xsc_run / the watchdog) and waits for the outcome. */
int nicogpu_sched_reset(struct nicogpu_dev *x, u32 level)
{
	struct nicogpu_sched *s = &x->sched;
	int rc;

	if (level < NICOGPU_RESET_CP || level > NICOGPU_RESET_GPU)
		return -EINVAL;
	if (!READ_ONCE(s->ready))
		return -ENODEV;
	mutex_lock(&s->reset_lock);
	reinit_completion(&s->reset_done);
	WRITE_ONCE(s->reset_level, level);
	atomic_or(NICOGPU_EV_RESET_REQ, &s->events);
	queue_work(s->wq_run, &s->work);
	if (!wait_for_completion_timeout(&s->reset_done, msecs_to_jiffies(60000))) {
		WRITE_ONCE(s->reset_level, 0);
		rc = -ETIMEDOUT;
	} else {
		rc = s->reset_rc;
	}
	mutex_unlock(&s->reset_lock);
	return rc;
}

/* One devcoredump.  Process context.
 * dev_coredumpv() keeps one dump per device until it is read or times out; a second one is dropped by the core. */
void nicogpu_dump(struct nicogpu_dev *x, enum xdd_reason reason)
{
	struct nicogpu_sched *s = &x->sched;
	struct xdd_io io = { .ctx = x, .shell_rd = ng_op_shell_rd, .rm_rd = ng_op_rm_rd };
	struct xsc_snap *snap = NULL;
	struct xdd_regs *regs = NULL;
	struct xdd_input *in = NULL;
	char *buf = NULL;
	size_t n;

	if (reason != XDD_R_MANUAL && !xdd_limiter_allow(&s->dump_lim, op_now_ms(s)))
		return;
	snap = kzalloc(sizeof(*snap), GFP_KERNEL);
	regs = kzalloc(sizeof(*regs), GFP_KERNEL);
	in = kzalloc(sizeof(*in), GFP_KERNEL);
	buf = vzalloc(NICOGPU_DUMP_BYTES);
	if (!snap || !regs || !in || !buf)
		goto out;
	in->reason = reason;
	in->now_ms = op_now_ms(s);
	if (READ_ONCE(s->ready)) {
		xsc_snapshot(&s->core, snap);
		in->snap = snap;
	}
	xdd_collect_regs(&io, regs);
	in->regs = regs;
	if (regs->rm_visible) {
		in->fw_version = ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_CP_FW_VERSION);
		in->cmd_version = ng_rd(x, NICOGPU_BAR_CTRL, NICOGPU_REG_CP_CMD_VERSION);
	}
	nicogpu_dfx_fill_dump(x, &in->dfx);
	n = xdd_format(buf, NICOGPU_DUMP_BYTES, in);
	dev_coredumpv(&x->pdev->dev, buf, n, GFP_KERNEL);	/* takes ownership of buf */
	buf = NULL;
	dev_info(&x->pdev->dev, "devcoredump (%s, %zu bytes): /sys/class/devcoredump/\n",
		 reason == XDD_R_INCIDENT ? "incident" : reason == XDD_R_DEVICE_LOST ? "device lost" : "manual", n);
out:
	vfree(buf);
	kfree(in);
	kfree(regs);
	kfree(snap);
}

/* ---- debugfs -------------------------------------------------------------- */

void nicogpu_sched_debugfs_show(struct nicogpu_dev *x, struct seq_file *m)
{
	struct nicogpu_sched *s = &x->sched;
	struct xsc_stats st;

	if (!READ_ONCE(s->ready)) {
		seq_printf(m, "sched: not initialised (first queue/syncobj ioctl brings the CP up)\n");
		return;
	}
	op_lock(s);
	st = s->core.st;
	seq_printf(m, "ring %u dwords at C3 +%llx (fw reserve +%llx), wptr %u wptr_db %u rptr %u\n", s->core.ring_dw,
		   s->ring_off, s->fw_off, s->core.wptr, s->core.wptr_db, s->core.rptr);
	seq_printf(m, "seq: armed %llu observed %llu done %llu; pending %u in-flight %u; io_err %d\n", s->core.last_seq,
		   s->core.observed, s->core.done_seq, s->core.n_pending, s->core.n_inflight, s->core.io_err);
	op_unlock(s);
	seq_printf(m, "jobs: armed %llu pushed %llu retired %llu failed %llu replayed %llu resets %llu\n", st.armed,
		   st.pushed, st.retired, st.failed, st.replayed, st.resets);
	seq_printf(m, "ring: wraps %llu full-stalls %llu dep-stalls %llu doorbells %llu\n", st.ring_wraps, st.ring_full,
		   st.dep_stalls, st.doorbells);
	seq_printf(m, "anomalies: lost-irq %llu (late %llu) bad-fence %llu stale-fence %llu bad-rptr %llu io-errors %llu reentry %llu\n",
		   st.lost_irq, st.late_irq, st.bad_fence, st.stale_fence, st.bad_rptr, st.io_errors, st.reentry);
	seq_printf(m, "resets: level1 %llu firmware %llu gpu %llu lost %llu; unattributed %llu; ctx-lost queues %llu\n",
		   st.reset_cp, st.reset_fw, st.reset_gpu, st.lost, st.unattributed, st.ctx_lost);
	seq_printf(m, "timeouts deferred (head job progressing): %llu\n", st.timeout_ext);
	seq_printf(m, "dispatch abort timeouts (CP_FW_STATUS.ABORT_TIMEOUT, escalated to the GPU zone): %llu\n",
		   st.abort_timeout);
	{
		struct nicogpu_fence_stats fst;

		nicogpu_fences_stats(&s->fences, &fst);
		seq_printf(m, "dma_fence: created %llu (every job) signalled %llu (errors %llu) pending %u; deps signalled %llu ours %llu foreign %llu (armed %u)\n",
			   fst.created, fst.signalled, fst.errors, fst.pending, fst.dep_signalled, fst.dep_ours, fst.dep_foreign,
			   fst.deps);
	}
	seq_printf(m, "faults: mmu %llu attached %llu vmid-mismatch %llu; incidents %llu\n", st.mmu_faults, st.fault_attached,
		   st.fault_vmid_mismatch, st.incidents);
}
