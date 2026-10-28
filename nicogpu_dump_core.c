// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * nicogpu_dump_core.c -- devcoredump text builder.  See nicogpu_dump_core.h.
 */
#include "nicogpu_dump_core.h"

/* ---- bounded writer ------------------------------------------------------------ */

struct w {
	char *buf;
	size_t cap, n;
	bool trunc;
};

static void w_ch(struct w *w, char c)
{
	if (w->n + 1 < w->cap)
		w->buf[w->n++] = c;
	else
		w->trunc = true;
}

static void w_s(struct w *w, const char *s)
{
	while (*s)
		w_ch(w, *s++);
}

static void w_u(struct w *w, uint64_t v)
{
	char t[24];
	int i = 0;

	do {
		t[i++] = (char)('0' + (v % 10));
		v /= 10;
	} while (v);
	while (i)
		w_ch(w, t[--i]);
}

static void w_i(struct w *w, int64_t v)
{
	if (v < 0) {
		w_ch(w, '-');
		w_u(w, (uint64_t)0 - (uint64_t)v);
	} else {
		w_u(w, (uint64_t)v);
	}
}

/* 0x + `digits` hex digits (0: as many as needed, at least 1) */
static void w_x(struct w *w, uint64_t v, int digits)
{
	static const char hx[] = "0123456789abcdef";
	char t[16];
	int i = 0;

	do {
		t[i++] = hx[v & 15];
		v >>= 4;
	} while (v);
	w_s(w, "0x");
	while (digits > i && digits <= 16) {
		w_ch(w, '0');
		digits--;
	}
	while (i)
		w_ch(w, t[--i]);
}

static void kv_s(struct w *w, const char *k, const char *v)
{
	w_s(w, k);
	w_s(w, ": ");
	w_s(w, v);
	w_ch(w, '\n');
}

static void kv_u(struct w *w, const char *k, uint64_t v)
{
	w_s(w, k);
	w_s(w, ": ");
	w_u(w, v);
	w_ch(w, '\n');
}

static void kv_x(struct w *w, const char *k, uint64_t v, int digits)
{
	w_s(w, k);
	w_s(w, ": ");
	w_x(w, v, digits);
	w_ch(w, '\n');
}

/* ---- names ----------------------------------------------------------------------- */

const char *xdd_cause_name(uint32_t cause)
{
	switch (cause) {
	case XSC_CAUSE_NONE: return "none";
	case XSC_CAUSE_TIMEOUT: return "timeout";
	case XSC_CAUSE_CP_ERROR: return "cp_error";
	case XSC_CAUSE_MMU_FAULT: return "mmu_fault";
	case XSC_CAUSE_FW_PANIC: return "fw_panic";
	case XSC_CAUSE_DEV_LOST: return "device_lost";
	default: return "?";
	}
}

const char *xdd_cp_error_name(uint32_t code)
{
	switch (code) {
	case NICOGPU_CMD_CP_ERROR_NONE: return "NONE";
	case NICOGPU_CMD_CP_ERROR_BAD_OPCODE: return "BAD_OPCODE";
	case NICOGPU_CMD_CP_ERROR_BAD_PACKET: return "BAD_PACKET";
	case NICOGPU_CMD_CP_ERROR_MEM_FAULT: return "MEM_FAULT";
	case NICOGPU_CMD_CP_ERROR_SHADER_FAULT: return "SHADER_FAULT";
	case NICOGPU_CMD_CP_ERROR_RING_CONFIG: return "RING_CONFIG";
	case NICOGPU_CMD_CP_ERROR_NOT_PERMITTED: return "NOT_PERMITTED";
	case NICOGPU_CMD_CP_ERROR_UNSUPPORTED: return "UNSUPPORTED";
	case NICOGPU_CMD_CP_ERROR_NO_SHADER_STATE: return "NO_SHADER_STATE";
	case NICOGPU_CMD_CP_ERROR_WATCHDOG: return "WATCHDOG";
	default: return "?";
	}
}

const char *xdd_mmu_fault_name(uint32_t code)
{
	switch (code) {
	case NICOGPU_MMU_FAULT_NONE: return "NONE";
	case NICOGPU_MMU_FAULT_NOT_PRESENT: return "NOT_PRESENT";
	case NICOGPU_MMU_FAULT_NON_CANONICAL: return "NON_CANONICAL";
	case NICOGPU_MMU_FAULT_PERMISSION: return "PERMISSION";
	case NICOGPU_MMU_FAULT_BAD_PTE: return "BAD_PTE";
	case NICOGPU_MMU_FAULT_BAD_PHYS_ADDR: return "BAD_PHYS_ADDR";
	case NICOGPU_MMU_FAULT_MISALIGNED: return "MISALIGNED";
	default: return "?";
	}
}

static const char *reason_name(enum xdd_reason r)
{
	switch (r) {
	case XDD_R_MANUAL: return "manual";
	case XDD_R_INCIDENT: return "incident";
	case XDD_R_QUIESCE_FORCED: return "quiesce_forced";
	case XDD_R_PR_FAILED: return "pr_failed";
	case XDD_R_DEVICE_LOST: return "device_lost";
	default: return "?";
	}
}

static const char *access_name(uint32_t a)
{
	return a == 0 ? "read" : a == 1 ? "write" : a == 2 ? "exec" : "?";
}

static const char *job_state_name(uint32_t s)
{
	switch (s) {
	case XSC_JS_PENDING: return "pending";
	case XSC_JS_PUSHING: return "pushing";
	case XSC_JS_RING: return "ring";
	case XSC_JS_FAILED: return "failed";
	default: return "?";
	}
}

static const char *outcome_name(enum xdf_outcome o)
{
	switch (o) {
	case XDF_OUT_NONE: return "none";
	case XDF_OUT_LOADED: return "loaded";
	case XDF_OUT_REJECTED: return "rejected";
	case XDF_OUT_RESTORED: return "restored";
	case XDF_OUT_WEDGED: return "wedged";
	default: return "?";
	}
}

static const char *stage_name(enum xdf_stage s)
{
	switch (s) {
	case XDF_STAGE_ISOLATE: return "isolate";
	case XDF_STAGE_PR: return "pr";
	case XDF_STAGE_IDENTIFY: return "identify";
	case XDF_STAGE_RELEASE: return "release";
	case XDF_STAGE_INIT: return "init";
	default: return "-";
	}
}

/* ---- register snapshot ----------------------------------------------------------- */

static void add_reg(struct xdd_regs *o, const char *name, uint8_t bar, uint32_t off, uint32_t val, bool valid)
{
	if (o->n >= XDD_MAX_REGS)
		return;
	o->r[o->n].name = name;
	o->r[o->n].bar = bar;
	o->r[o->n].off = off;
	o->r[o->n].val = val;
	o->r[o->n].valid = valid;
	o->n++;
}

void xdd_collect_regs(const struct xdd_io *io, struct xdd_regs *out)
{
	static const struct {
		const char *name;
		uint32_t off;
	} shell[] = {
		{ "SHELL_VERSION", XDF_SH_VERSION }, { "SHELL_UID_LO", XDF_SH_UID_LO }, { "SHELL_UID_HI", XDF_SH_UID_HI },
		{ "SHELL_STATUS", XDD_SH_STATUS }, { "SHELL_IRQ_STATUS", XDD_SH_IRQ_STATUS }, { "SHELL_IRQ_ENABLE", XDD_SH_IRQ_ENABLE },
		{ "RM_CTRL", XDF_RM_CTRL }, { "RM_STATUS", XDF_RM_STATUS }, { "RM_OUTSTANDING", XDF_RM_OUTSTANDING },
		{ "RM_INFO0", XDF_RM_INFO0 }, { "RM_INFO1", XDF_RM_INFO0 + 4 }, { "RM_INFO2", XDF_RM_INFO0 + 8 },
		{ "RM_INFO3", XDF_RM_INFO0 + 12 }, { "RM_ERR_COUNT", XDF_RM_ERR_COUNT }, { "PR_STATUS", XDF_PR_STATUS },
		{ "PR_WORDS", XDF_PR_WORDS },
	};
	static const struct {
		const char *name;
		uint32_t off;
	} rm[] = {
		{ "IRQ_STATUS", NICOGPU_REG_IRQ_STATUS }, { "IRQ_ENABLE", NICOGPU_REG_IRQ_ENABLE },
		{ "CP_RING_BASE_LO", NICOGPU_REG_CP_RING_BASE_LO }, { "CP_RING_BASE_HI", NICOGPU_REG_CP_RING_BASE_HI },
		{ "CP_RING_SIZE_LOG2", NICOGPU_REG_CP_RING_SIZE_LOG2 }, { "CP_RING_RPTR", NICOGPU_REG_CP_RING_RPTR },
		{ "CP_RING_WPTR", NICOGPU_REG_CP_RING_WPTR }, { "CP_CTRL", NICOGPU_REG_CP_CTRL },
		{ "CP_STATUS", NICOGPU_REG_CP_STATUS }, { "CP_ERROR", NICOGPU_REG_CP_ERROR },
		{ "CP_ERROR_RPTR", NICOGPU_REG_CP_ERROR_RPTR }, { "CP_ERROR_INFO", NICOGPU_REG_CP_ERROR_INFO },
		{ "CP_ERROR_IB_OFFSET", NICOGPU_REG_CP_ERROR_IB_OFFSET }, { "CP_HEARTBEAT", NICOGPU_REG_CP_HEARTBEAT },
		{ "CP_CMD_VERSION", NICOGPU_REG_CP_CMD_VERSION }, { "CP_FW_VERSION", NICOGPU_REG_CP_FW_VERSION },
		{ "CP_FW_STATUS", NICOGPU_REG_CP_FW_STATUS }, { "CP_FW_BASE_LO", NICOGPU_REG_CP_FW_BASE_LO },
		{ "CP_FW_BASE_HI", NICOGPU_REG_CP_FW_BASE_HI }, { "CP_FW_SIZE", NICOGPU_REG_CP_FW_SIZE },
		{ "CP_RPTR_WB_LO", NICOGPU_REG_CP_RPTR_WB_LO }, { "CP_RPTR_WB_HI", NICOGPU_REG_CP_RPTR_WB_HI },
		{ "MMU_FAULT_STATUS", NICOGPU_REG_MMU_FAULT_STATUS }, { "MMU_FAULT_VA_LO", NICOGPU_REG_MMU_FAULT_VA_LO },
		{ "MMU_FAULT_VA_HI", NICOGPU_REG_MMU_FAULT_VA_HI }, { "MMU_FAULT_PTE_LO", NICOGPU_REG_MMU_FAULT_PTE_LO },
		{ "MMU_FAULT_PTE_HI", NICOGPU_REG_MMU_FAULT_PTE_HI }, { "C3_WIN_CTRL", NICOGPU_REG_C3_WIN_CTRL },
		{ "C3_WIN_BASE", NICOGPU_REG_C3_WIN_BASE },
	};
	uint32_t i, st;

	memset(out, 0, sizeof(*out));
	for (i = 0; i < sizeof(shell) / sizeof(shell[0]); i++)
		add_reg(out, shell[i].name, 1, shell[i].off, io->shell_rd(io->ctx, shell[i].off), true);
	st = io->shell_rd(io->ctx, XDF_RM_STATUS);
	out->rm_visible = !(st & XDF_ST_DECOUPLED) && xdf_rm_id_ok(io->rm_rd(io->ctx, XDF_RM_ID));
	for (i = 0; i < sizeof(rm) / sizeof(rm[0]); i++)
		add_reg(out, rm[i].name, 0, rm[i].off, out->rm_visible ? io->rm_rd(io->ctx, rm[i].off) : 0, out->rm_visible);
}

/* ---- sections ---------------------------------------------------------------------- */

static void sec(struct w *w, const char *name)
{
	w_s(w, "\n[");
	w_s(w, name);
	w_s(w, "]\n");
}

static void put_core(struct w *w, const struct xsc_snap *s)
{
	sec(w, "core");
	kv_u(w, "ring_dwords", s->ring_dw);
	w_s(w, "wptr/wptr_db/rptr: ");
	w_u(w, s->wptr);
	w_ch(w, ' ');
	w_u(w, s->wptr_db);
	w_ch(w, ' ');
	w_u(w, s->rptr);
	w_ch(w, '\n');
	w_s(w, "seq last/flight/observed/done: ");
	w_u(w, s->last_seq);
	w_ch(w, ' ');
	w_u(w, s->flight_seq);
	w_ch(w, ' ');
	w_u(w, s->observed);
	w_ch(w, ' ');
	w_u(w, s->done_seq);
	w_ch(w, '\n');
	kv_u(w, "fence_page", s->fence_mem);
	w_s(w, "jobs pending/inflight: ");
	w_u(w, s->n_pending);
	w_ch(w, ' ');
	w_u(w, s->n_inflight);
	w_ch(w, '\n');
	w_s(w, "flags:");
	if (s->busy)
		w_s(w, " busy");
	if (s->dead)
		w_s(w, " dead");
	if (s->lost)
		w_s(w, " LOST");
	if (s->io_err)
		w_s(w, " io_err");
	if (!s->busy && !s->dead && !s->lost && !s->io_err)
		w_s(w, " -");
	w_ch(w, '\n');
	kv_u(w, "ctx_epoch", s->epoch);
	kv_u(w, "timeout_ms", s->cfg.timeout_ms);
	kv_x(w, "cfg_flags", s->cfg.flags, 0);
	w_s(w, "stats armed/pushed/retired/failed: ");
	w_u(w, s->st.armed);
	w_ch(w, ' ');
	w_u(w, s->st.pushed);
	w_ch(w, ' ');
	w_u(w, s->st.retired);
	w_ch(w, ' ');
	w_u(w, s->st.failed);
	w_ch(w, '\n');
	w_s(w, "resets total/cp/fw/gpu/lost: ");
	w_u(w, s->st.resets);
	w_ch(w, ' ');
	w_u(w, s->st.reset_cp);
	w_ch(w, ' ');
	w_u(w, s->st.reset_fw);
	w_ch(w, ' ');
	w_u(w, s->st.reset_gpu);
	w_ch(w, ' ');
	w_u(w, s->st.lost);
	w_ch(w, '\n');
	w_s(w, "replayed/lost_irq/io_errors/ctx_lost: ");
	w_u(w, s->st.replayed);
	w_ch(w, ' ');
	w_u(w, s->st.lost_irq);
	w_ch(w, ' ');
	w_u(w, s->st.io_errors);
	w_ch(w, ' ');
	w_u(w, s->st.ctx_lost);
	w_ch(w, '\n');
	w_s(w, "anomalies bad_fence/stale_fence/bad_rptr/reentry/unattributed: ");
	w_u(w, s->st.bad_fence);
	w_ch(w, ' ');
	w_u(w, s->st.stale_fence);
	w_ch(w, ' ');
	w_u(w, s->st.bad_rptr);
	w_ch(w, ' ');
	w_u(w, s->st.reentry);
	w_ch(w, ' ');
	w_u(w, s->st.unattributed);
	w_ch(w, '\n');
}

static void put_incident(struct w *w, const struct xsc_snap *s)
{
	sec(w, "incident");
	kv_u(w, "serial", s->inc.n);
	kv_s(w, "cause", xdd_cause_name(s->inc.cause));
	kv_u(w, "seq", s->inc.seq);
	w_s(w, "cp_error: ");
	w_u(w, s->inc.cp_error);
	w_ch(w, ' ');
	w_s(w, xdd_cp_error_name(s->inc.cp_error));
	w_ch(w, '\n');
	kv_u(w, "at_ms", s->inc.t_ms);
	if (s->inc.seq) {
		uint32_t k;

		w_s(w, "failing job: seq ");
		w_u(w, s->inc.seq);
		w_s(w, " vmid ");
		w_u(w, s->inc.vmid);
		w_s(w, " ring [");
		w_u(w, s->inc.ring_start);
		w_s(w, ",");
		w_u(w, s->inc.ring_end);
		w_s(w, ") INDIRECTs ");
		w_u(w, s->inc.nib);
		w_ch(w, '\n');
		for (k = 0; k < s->inc.nib_kept; k++) {
			w_s(w, "  INDIRECT ");
			w_u(w, k);
			w_s(w, ": va ");
			w_x(w, s->inc.ib_va[k], 16);
			w_s(w, " size_dw ");
			w_u(w, s->inc.ib_dw[k]);
			w_ch(w, '\n');
		}
	}
}

static void put_faults(struct w *w, const struct xsc_snap *s)
{
	uint32_t i;

	sec(w, "mmu_faults");
	kv_u(w, "count", s->nfaults);
	kv_u(w, "total", s->st.mmu_faults);
	kv_u(w, "attached_to_jobs", s->st.fault_attached);
	kv_u(w, "vmid_mismatch", s->st.fault_vmid_mismatch);
	for (i = 0; i < s->nfaults; i++) {
		const struct xsc_mmu_fault *f = &s->faults[i];

		w_s(w, "fault #");
		w_u(w, f->n);
		w_s(w, ": vmid ");
		w_u(w, f->vmid);
		w_s(w, " va ");
		w_x(w, f->va, 16);
		w_s(w, " ");
		w_s(w, access_name(f->access));
		w_s(w, " ");
		w_s(w, xdd_mmu_fault_name(f->code));
		w_s(w, " level ");
		w_u(w, f->level);
		w_s(w, " pte ");
		w_x(w, f->pte, 16);
		w_s(w, " status ");
		w_x(w, f->status, 8);
		if (f->multi)
			w_s(w, " MULTI");
		w_s(w, f->used ? " attached" : " unattached");
		w_s(w, " age_ms ");
		w_u(w, s->now_ms - f->t_ms);
		w_ch(w, '\n');
	}
}

static void put_jobs(struct w *w, const struct xsc_snap *s)
{
	uint32_t i, k;

	sec(w, "jobs");
	w_s(w, "alive: ");
	w_u(w, s->jobs_total);
	w_s(w, " (listed ");
	w_u(w, s->njobs);
	w_s(w, ")\n");
	for (i = 0; i < s->njobs; i++) {
		const struct xsc_snap_job *j = &s->jobs[i];

		w_s(w, "job seq ");
		w_u(w, j->seq);
		w_s(w, " ");
		w_s(w, job_state_name(j->state));
		w_s(w, " vmid ");
		w_u(w, j->vmid);
		w_s(w, " ring [");
		w_u(w, j->ring_start);
		w_s(w, ",");
		w_u(w, j->ring_end);
		w_s(w, ") dw ");
		w_u(w, j->dw);
		if (j->err) {
			w_s(w, " err ");
			w_i(w, j->err);
		}
		if (j->pre_err) {
			w_s(w, " pre_err ");
			w_i(w, j->pre_err);
		}
		if (j->age_ms) {
			w_s(w, " head_age_ms ");
			w_u(w, j->age_ms);
		}
		if (j->queue_guilty)
			w_s(w, " queue-GUILTY");
		w_ch(w, '\n');
		for (k = 0; k < j->nib_kept; k++) {
			w_s(w, "  INDIRECT ");
			w_u(w, k);
			w_s(w, ": va ");
			w_x(w, j->ib_va[k], 16);
			w_s(w, " size_dw ");
			w_u(w, j->ib_dw[k]);
			w_s(w, " vmid ");
			w_u(w, j->vmid);
			w_ch(w, '\n');
		}
		if (j->nib > j->nib_kept) {
			w_s(w, "  ... ");
			w_u(w, j->nib - j->nib_kept);
			w_s(w, " more INDIRECT\n");
		}
	}
	w_s(w, "ib_contents: not captured (BAR2 is never read by the CPU)\n");
	w_s(w, "recent failures (newest first):");
	if (!s->nhist)
		w_s(w, " -");
	for (i = 0; i < s->nhist; i++) {
		w_s(w, " seq ");
		w_u(w, s->hist[i].seq);
		w_s(w, "=");
		w_i(w, s->hist[i].err);
	}
	w_ch(w, '\n');
}

static void put_regs(struct w *w, const struct xdd_regs *r)
{
	uint32_t i;

	sec(w, "registers");
	kv_s(w, "rm_visible", r->rm_visible ? "yes" : "no (isolated: BAR0 reads skipped)");
	for (i = 0; i < r->n; i++) {
		w_s(w, r->r[i].bar ? "BAR1 " : "BAR0 ");
		w_x(w, r->r[i].off, 4);
		w_ch(w, ' ');
		w_s(w, r->r[i].name);
		w_s(w, " = ");
		if (r->r[i].valid)
			w_x(w, r->r[i].val, 8);
		else
			w_s(w, "-");
		w_ch(w, '\n');
	}
}

static void put_dfx(struct w *w, const struct xdd_dfx *d)
{
	sec(w, "dfx");
	kv_s(w, "last_outcome", outcome_name(d->outcome));
	kv_s(w, "last_stage", stage_name(d->stage));
	w_s(w, "last_err: ");
	w_i(w, d->err);
	w_ch(w, '\n');
	kv_s(w, "wedged", d->wedged ? "yes" : "no");
	w_s(w, "loads/rejected/retries/restored/wedged/gpu_resets/forced_quiesce: ");
	w_u(w, d->st.loads);
	w_ch(w, ' ');
	w_u(w, d->st.rejected);
	w_ch(w, ' ');
	w_u(w, d->st.retries);
	w_ch(w, ' ');
	w_u(w, d->st.restored);
	w_ch(w, ' ');
	w_u(w, d->st.wedged);
	w_ch(w, ' ');
	w_u(w, d->st.gpu_resets);
	w_ch(w, ' ');
	w_u(w, d->st.forced_quiesce);
	w_ch(w, '\n');
}

size_t xdd_format(char *buf, size_t cap, const struct xdd_input *in)
{
	struct w w;
	static const char tr[] = "[truncated]\n";

	if (!cap)
		return 0;
	w.buf = buf;
	w.cap = cap;
	w.n = 0;
	w.trunc = false;
	w_s(&w, "nicogpu devcoredump v1\n");
	kv_s(&w, "reason", reason_name(in->reason));
	kv_u(&w, "time_ms", in->now_ms);
	if (in->fw_version || in->cmd_version) {
		kv_x(&w, "cp_fw_version", in->fw_version, 8);
		kv_x(&w, "cp_cmd_version", in->cmd_version, 8);
	}
	if (in->snap) {
		put_incident(&w, in->snap);
		put_core(&w, in->snap);
		put_faults(&w, in->snap);
		put_jobs(&w, in->snap);
	}
	if (in->dfx.valid)
		put_dfx(&w, &in->dfx);
	if (in->regs)
		put_regs(&w, in->regs);
	if (w.trunc) {		/* rewind so the marker fits in the last bytes */
		size_t t = sizeof(tr) - 1, i;
		size_t keep = cap > t + 1 ? cap - 1 - t : 0;

		if (w.n > keep)
			w.n = keep;
		for (i = 0; i < t && w.n < cap - 1; i++)
			buf[w.n++] = tr[i];
	}
	buf[w.n] = '\0';
	return w.n;
}

void xdd_limiter_init(struct xdd_limiter *l, uint32_t max, uint64_t min_gap_ms)
{
	l->max = max;
	l->min_gap_ms = min_gap_ms;
	l->taken = 0;
	l->last_ms = 0;
}

bool xdd_limiter_allow(struct xdd_limiter *l, uint64_t now_ms)
{
	if (l->max && l->taken >= l->max)
		return false;
	if (l->taken && now_ms - l->last_ms < l->min_gap_ms)
		return false;
	l->taken++;
	l->last_ms = now_ms;
	return true;
}
