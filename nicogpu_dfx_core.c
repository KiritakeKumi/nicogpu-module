// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * nicogpu_dfx_core.c -- shell sequences of the DFX boundary: GPU-zone soft reset
 * and partial reconfiguration.  See nicogpu_dfx_core.h.
 */
#include "nicogpu_dfx_core.h"

#define E_NODEV		19
#define E_BUSY		16
#define E_IO		5
#define E_TIMEDOUT	110
#define E_PROTO		71

static void emit(const struct xdf *d, enum xdf_event ev, uint64_t a, uint64_t b)
{
	if (d->ops->event)
		d->ops->event(d->ctx, ev, a, b);
}

void xdf_cfg_default(struct xdf_cfg *cfg)
{
	cfg->quiesce_ms = 20;			/* the shell's own timeout is 10 ms (RM_QUIESCE_TIMEOUT) */
	cfg->decouple_ms = 10;
	cfg->ready_ms = 100;
	cfg->release_ms = 100;
	cfg->pr_finish_ms = 2000;		/* FIFO drain + ICAP at 400 MB/s after the last word */
	cfg->info_ms = 10;
	cfg->max_payload = 256u << 20;		/* a full-RP partial bitstream is ~70 MB */
	cfg->pr_opt = 0;			/* keep the shell's PR_OPT reset value */
}

int xdf_init(struct xdf *d, const struct xdf_ops *ops, void *ctx, const struct xdf_cfg *cfg)
{
	if (!d || !ops || !cfg || !ops->shell_rd || !ops->shell_wr || !ops->rm_rd || !ops->pr_push || !ops->msleep)
		return -XIM_EINVAL;
	memset(d, 0, sizeof(*d));
	d->ops = ops;
	d->ctx = ctx;
	d->cfg = *cfg;
	if (ops->shell_rd(ctx, XDF_SH_ID) != XDF_SH_ID_MAGIC)
		return -E_NODEV;
	d->shell_uid = (uint64_t)ops->shell_rd(ctx, XDF_SH_UID_LO) | ((uint64_t)ops->shell_rd(ctx, XDF_SH_UID_HI) << 32);
	d->shell_iface = (uint16_t)(ops->shell_rd(ctx, XDF_SH_VERSION) >> 16);
	d->shell_caps = ops->shell_rd(ctx, XDF_SH_CAPS);
	return 0;
}

void xdf_set_good(struct xdf *d, const uint8_t *img, size_t len)
{
	d->good = img;
	d->good_len = img ? len : 0;
}

/* ---- primitives ----------------------------------------------------------------- */

static void wr_ctrl(struct xdf *d, uint32_t v)
{
	d->ctrl = v;
	d->ops->shell_wr(d->ctx, XDF_RM_CTRL, v);
}

/* poll RM_STATUS until (status & mask) == want, 1 ms apart, at most `ms` ms */
static int wait_status(struct xdf *d, uint32_t mask, uint32_t want, uint32_t ms, uint32_t *last)
{
	uint32_t t, v;

	for (t = 0;; t++) {
		v = d->ops->shell_rd(d->ctx, XDF_RM_STATUS);
		if (last)
			*last = v;
		if ((v & mask) == want)
			return 0;
		if (t >= ms)
			return -E_TIMEDOUT;
		d->ops->msleep(d->ctx, 1);
	}
}

/* PR steps 2-4: IRQ forwarding off, quiesce (FORCE on timeout), DECOUPLE, RM_RESET.  Idempotent: an RM
 * that is already isolated (power-up, a failed attempt) only gets the reset. */
static int seq_isolate(struct xdf *d)
{
	uint32_t st;
	int rc;

	st = d->ops->shell_rd(d->ctx, XDF_RM_STATUS);
	if (!(st & XDF_ST_DECOUPLED)) {
		wr_ctrl(d, XDF_CTRL_QUIESCE_REQ);		/* IRQ_FWD_EN = 0 with it */
		rc = wait_status(d, XDF_ST_QUIESCE_ACK | XDF_ST_DRAINED, XDF_ST_QUIESCE_ACK | XDF_ST_DRAINED,
				 d->cfg.quiesce_ms, &st);
		if (rc) {
			d->st.forced_quiesce++;
			emit(d, XDF_EV_QUIESCE_FORCED, st, d->ops->shell_rd(d->ctx, XDF_RM_OUTSTANDING));
			wr_ctrl(d, d->ctrl | XDF_CTRL_FORCE);
		}
		wr_ctrl(d, d->ctrl | XDF_CTRL_DECOUPLE);
		rc = wait_status(d, XDF_ST_DECOUPLED, XDF_ST_DECOUPLED, d->cfg.decouple_ms, &st);
		if (rc && !(d->ctrl & XDF_CTRL_FORCE)) {	/* DECOUPLE only takes effect when DRAINED or FORCE */
			d->st.forced_quiesce++;
			emit(d, XDF_EV_QUIESCE_FORCED, st, d->ops->shell_rd(d->ctx, XDF_RM_OUTSTANDING));
			wr_ctrl(d, d->ctrl | XDF_CTRL_FORCE);
			rc = wait_status(d, XDF_ST_DECOUPLED, XDF_ST_DECOUPLED, d->cfg.decouple_ms, &st);
		}
		if (rc)
			return rc;
	} else {
		wr_ctrl(d, XDF_CTRL_DECOUPLE);
	}
	wr_ctrl(d, d->ctrl | XDF_CTRL_RM_RESET);
	return wait_status(d, XDF_ST_IN_RESET, XDF_ST_IN_RESET, d->cfg.decouple_ms, NULL);
}

/* PR step 8: RM_RESET = 0, RM_READY, DECOUPLE = 0, wait !DECOUPLED.  If the shell keeps the isolation
 * because the RM is not acceptable, say why (-EPROTO) instead of a plain timeout. */
static int seq_release(struct xdf *d)
{
	uint32_t st = 0;
	int rc;

	wr_ctrl(d, d->ctrl & ~XDF_CTRL_RM_RESET);
	rc = wait_status(d, XDF_ST_RM_READY, XDF_ST_RM_READY, d->cfg.ready_ms, NULL);
	if (rc)
		return rc;
	wr_ctrl(d, d->ctrl & ~(XDF_CTRL_DECOUPLE | XDF_CTRL_QUIESCE_REQ | XDF_CTRL_FORCE));
	rc = wait_status(d, XDF_ST_DECOUPLED, 0, d->cfg.release_ms, &st);
	if (rc) {
		const uint32_t need = XDF_ST_INFO_VALID | XDF_ST_IFACE_OK | XDF_ST_MEM_OK;

		return (st & need) != need ? -E_PROTO : rc;	/* RM_INCOMPAT */
	}
	return 0;
}

/* keep the RM away from the host after a failure that happened after the release (or when nothing could be
 * loaded): the full isolation handshake again -- DECOUPLE only takes effect when DRAINED or FORCE, so a plain
 * DECOUPLE write is not enough once the shell has released the RM. */
static void quarantine(struct xdf *d)
{
	(void)seq_isolate(d);
}

/* PR step 9: BAR0 ID (and REV when known), IRQ_FWD_EN = 1, EOI. */
static int seq_init(struct xdf *d)
{
	uint32_t id = d->ops->rm_rd(d->ctx, XDF_RM_ID);

	if (id != XDF_RM_ID_MAGIC) {
		quarantine(d);
		return -E_NODEV;
	}
	if (d->cur_rev_known && (d->ops->rm_rd(d->ctx, XDF_RM_REV) & 0xffffu) != d->cur_rm_rev) {
		quarantine(d);
		return -E_PROTO;
	}
	wr_ctrl(d, XDF_CTRL_IRQ_FWD_EN);
	d->ops->shell_wr(d->ctx, XDF_SH_IRQ_EOI, 3u);	/* re-arm both vectors */
	return 0;
}

/* the PR/ICAP diagnostic registers after an attempt (ICAP_RB shells only: older shells read 0 there) */
static void pr_diag(struct xdf *d)
{
	const struct xdf_ops *o = d->ops;
	struct xdf_pr_diag *g = &d->last_pr;

	if (!(d->shell_caps & XDF_CAPS_ICAP_RB))
		return;
	g->valid = true;
	g->pr_status = o->shell_rd(d->ctx, XDF_PR_STATUS);
	g->pr_words = o->shell_rd(d->ctx, XDF_PR_WORDS);
	g->icap_words = o->shell_rd(d->ctx, XDF_PR_ICAP_WORDS);
	g->icap_o = o->shell_rd(d->ctx, XDF_PR_ICAP_O);
	g->icap_ostat = o->shell_rd(d->ctx, XDF_PR_ICAP_OSTAT);
	g->pr_opt = o->shell_rd(d->ctx, XDF_PR_OPT);
	emit(d, XDF_EV_PR_DIAG, g->pr_status, ((uint64_t)g->icap_words << 32) | g->pr_words);
}

/* PR step 6: PR_EXPECT_WORDS, START, 4 KiB blocks into PR_DATA, FINISH, wait DONE|ERROR. */
static int seq_pr(struct xdf *d, const struct xim_xgpr *x)
{
	const struct xdf_ops *o = d->ops;
	uint32_t off, st = 0, t, nblk = 0;
	int rc = 0;

	if (d->cfg.pr_opt)				/* word order / completion selector for experiments */
		o->shell_wr(d->ctx, XDF_PR_OPT, d->cfg.pr_opt);
	o->shell_wr(d->ctx, XDF_PR_EXPECT_WORDS, x->payload_bytes / 4);
	o->shell_wr(d->ctx, XDF_PR_CTRL, XDF_PR_START);
	st = o->shell_rd(d->ctx, XDF_PR_STATUS);
	if (st & XDF_PRS_ERROR) {			/* ECODE 1: not isolated / not in reset */
		rc = -E_IO;
		goto abort;
	}
	for (off = 0; off < x->payload_bytes; off += XDF_PR_CHUNK) {
		uint32_t n = x->payload_bytes - off < XDF_PR_CHUNK ? x->payload_bytes - off : XDF_PR_CHUNK;

		o->pr_push(d->ctx, x->payload + off, n);
		if (++nblk % 64 == 0 && (o->shell_rd(d->ctx, XDF_PR_STATUS) & XDF_PRS_ERROR)) {
			rc = -E_IO;		/* stop pushing into a failed ICAP */
			goto abort;
		}
	}
	o->shell_wr(d->ctx, XDF_PR_CTRL, XDF_PR_FINISH);
	for (t = 0;; t++) {
		st = o->shell_rd(d->ctx, XDF_PR_STATUS);
		if (st & (XDF_PRS_DONE | XDF_PRS_ERROR))
			break;
		if (t >= d->cfg.pr_finish_ms) {
			rc = -E_TIMEDOUT;
			goto abort;
		}
		o->msleep(d->ctx, 1);
	}
	if ((st & XDF_PRS_ERROR) || !(st & XDF_PRS_DONE) || (st & XDF_PRS_BUSY) ||
	    o->shell_rd(d->ctx, XDF_PR_WORDS) != x->payload_bytes / 4) {
		rc = -E_IO;
		goto abort;
	}
	d->st.pr_bytes += x->payload_bytes;
	pr_diag(d);
	return 0;
abort:
	pr_diag(d);					/* before ABORT: what the ICAP saw */
	o->shell_wr(d->ctx, XDF_PR_CTRL, XDF_PR_ABORT);
	return rc;
}

/* PR step 7: RM_INFO0..3 against the container header. */
static int seq_identify(struct xdf *d, const struct xim_xgpr *x)
{
	const struct xdf_ops *o = d->ops;
	uint32_t i0, i1, i2, i3;
	int rc;

	rc = wait_status(d, XDF_ST_INFO_VALID, XDF_ST_INFO_VALID, d->cfg.info_ms, NULL);
	if (rc)
		return -E_PROTO;
	i0 = o->shell_rd(d->ctx, XDF_RM_INFO0);
	i1 = o->shell_rd(d->ctx, XDF_RM_INFO0 + 4);
	i2 = o->shell_rd(d->ctx, XDF_RM_INFO0 + 8);
	i3 = o->shell_rd(d->ctx, XDF_RM_INFO0 + 12);
	if (i0 != XDF_RMINFO_MAGIC || i1 != (((uint32_t)x->rm_kind << 16) | x->iface_version) || i2 != x->build_id ||
	    i3 != (((uint32_t)x->mem_mask << 16) | x->rm_rev))
		return -E_PROTO;
	return 0;
}

/* one complete load attempt, steps 3-9; the RM is isolated + in reset again on failure */
static int load_once(struct xdf *d, const struct xim_xgpr *x)
{
	enum xdf_stage stage = XDF_STAGE_ISOLATE;
	int rc;

	rc = seq_isolate(d);
	if (!rc) {
		stage = XDF_STAGE_PR;
		rc = seq_pr(d, x);
	}
	if (!rc) {
		stage = XDF_STAGE_IDENTIFY;
		rc = seq_identify(d, x);
	}
	if (!rc) {
		stage = XDF_STAGE_RELEASE;
		rc = seq_release(d);
	}
	if (!rc) {
		d->cur_rm_rev = x->rm_rev;
		d->cur_rev_known = true;
		stage = XDF_STAGE_INIT;
		rc = seq_init(d);
	}
	if (rc) {
		d->last_err = rc;
		d->last_stage = stage;
		emit(d, XDF_EV_PR_ERROR, stage, (uint32_t)-rc);
		if (stage == XDF_STAGE_RELEASE)
			quarantine(d);
	}
	return rc;
}

/* The last known-good container, then the blank fallback.  `bad` is the container that just failed
 * (its twin in `good` would fail again): skipped. */
static int restore(struct xdf *d, const struct xim_xgpr *bad)
{
	const struct xim_xgpr_policy pol = { d->shell_uid, d->shell_iface, d->cfg.max_payload };
	const uint8_t *img[2];
	size_t len[2];
	int i, rc = -E_IO;

	img[0] = d->good;
	len[0] = d->good_len;
	img[1] = NULL;
	len[1] = 0;
	if (d->ops->fallback)
		img[1] = d->ops->fallback(d->ctx, &len[1]);
	for (i = 0; i < 2; i++) {
		struct xim_xgpr x;

		if (!img[i])
			continue;
		rc = xim_xgpr_check(img[i], len[i], &pol, &x);
		if (!rc && bad && !memcmp(x.sha256, bad->sha256, 32) && x.build_id == bad->build_id)
			continue;	/* the same bits again */
		if (!rc)
			rc = load_once(d, &x);
		emit(d, XDF_EV_ROLLBACK, i + 1, (uint32_t)-rc);
		if (!rc)
			return 0;
	}
	return rc ? rc : -E_IO;
}

static int wedge(struct xdf *d)
{
	d->wedged = true;
	d->st.wedged++;
	d->last = XDF_OUT_WEDGED;
	quarantine(d);
	emit(d, XDF_EV_WEDGED, 0, 0);
	return -E_NODEV;
}

int xdf_gpu_reset(struct xdf *d)
{
	int rc;

	if (d->wedged)
		return -E_NODEV;
	d->st.gpu_resets++;
	rc = seq_isolate(d);
	if (!rc)
		rc = seq_release(d);
	if (!rc)
		rc = seq_init(d);
	if (!rc) {
		emit(d, XDF_EV_GPU_RESET, 0, 0);
		return 0;
	}
	/* the soft reset did not bring it back: reload the RM */
	emit(d, XDF_EV_GPU_RESET, (uint32_t)-rc, 1);
	if (!restore(d, NULL)) {
		d->st.restored++;
		d->last = XDF_OUT_RESTORED;
		return 0;
	}
	(void)wedge(d);
	return rc;
}

int xdf_check(const struct xdf *d, const uint8_t *img, size_t len, struct xim_xgpr *out)
{
	const struct xim_xgpr_policy pol = { d->shell_uid, d->shell_iface, d->cfg.max_payload };

	return xim_xgpr_check(img, len, &pol, out);
}

int xdf_pr_load(struct xdf *d, const uint8_t *img, size_t len)
{
	struct xim_xgpr x;
	int rc;

	if (d->wedged)
		return -E_NODEV;
	d->last_err = 0;
	d->last_stage = 0;
	/* step 1: the container.  Nothing below touches the hardware until it passed. */
	rc = xdf_check(d, img, len, &x);
	if (rc) {
		d->st.rejected++;
		d->last = XDF_OUT_REJECTED;
		d->last_err = rc;
		return rc;
	}
	/* step 2: stop the world */
	if (d->ops->prepare) {
		rc = d->ops->prepare(d->ctx);
		if (rc) {
			d->st.rejected++;
			d->last = XDF_OUT_REJECTED;
			d->last_err = rc;
			return rc;
		}
	}
	emit(d, XDF_EV_PR_START, x.payload_bytes, x.build_id);
	d->st.loads++;
	rc = load_once(d, &x);
	if (rc) {			/* retry once */
		d->st.retries++;
		emit(d, XDF_EV_RETRY, 0, 0);
		rc = load_once(d, &x);
	}
	if (!rc) {
		d->last = XDF_OUT_LOADED;
		emit(d, XDF_EV_LOADED, x.build_id, x.rm_kind);
		if (d->ops->done)
			d->ops->done(d->ctx, 0);
		return 0;
	}
	/* back to the last good RM, then the blank one, then wedged */
	if (!restore(d, &x)) {
		d->st.restored++;
		d->last = XDF_OUT_RESTORED;
		rc = -E_IO;
	} else {
		rc = wedge(d);
	}
	if (d->ops->done)
		d->ops->done(d->ctx, rc);
	return rc;
}

/* ---- ICAP readback ------------------------------------------------------------ */

uint32_t xdf_icap_decode(uint32_t order, uint32_t pin)
{
	uint32_t r = 0;
	int i;

	switch (order & XDF_OPT_ORDER_MASK) {
	case 0:					/* bitrev32 */
		for (i = 0; i < 32; i++)
			r |= ((pin >> i) & 1u) << (31 - i);
		return r;
	case 1:					/* bit reversal inside each byte, byte order kept */
		for (i = 0; i < 32; i++)
			r |= ((pin >> i) & 1u) << ((i & ~7) + 7 - (i & 7));
		return r;
	case 2:					/* byte reversal */
		return (pin >> 24) | ((pin >> 8) & 0xff00u) | ((pin << 8) & 0xff0000u) | (pin << 24);
	default:
		return pin;
	}
}

int xdf_icap_readback(struct xdf *d, uint32_t reg, uint32_t count, uint32_t skip, uint32_t out[8])
{
	const struct xdf_ops *o = d->ops;
	uint32_t st, t, i;

	if (!(d->shell_caps & XDF_CAPS_ICAP_RB))
		return -E_NODEV;
	o->shell_wr(d->ctx, XDF_PR_RB_CTRL, XDF_RB_GO | ((skip & 0x3fu) << 16) | ((count & 0xffu) << 8) | (reg & 0x1fu));
	for (t = 0;; t++) {
		st = o->shell_rd(d->ctx, XDF_PR_RB_STATUS);
		if (st & XDF_RBS_REJECTED)
			return -E_BUSY;
		if ((st & (XDF_RBS_BUSY | XDF_RBS_DONE)) == XDF_RBS_DONE)
			break;
		if (t >= 10)
			return -E_TIMEDOUT;		/* the sequence is ~60 ICAP clocks: 10 ms means the ICAP clock is gone */
		o->msleep(d->ctx, 1);
	}
	for (i = 0; i < 8; i++)
		out[i] = o->shell_rd(d->ctx, XDF_PR_RB_DATA0 + 4 * i);
	return 0;
}
