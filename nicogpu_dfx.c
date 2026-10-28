// SPDX-License-Identifier: GPL-2.0
/*
 * nicogpu_dfx.c -- kernel glue of the DFX boundary sequences: GPU-zone soft reset
 * (called by the scheduler core's reset ladder) and partial reconfiguration NICOGPU_IOC_PR_LOAD / debugfs).
 *
 * All sequencing -- register order, timeouts, retry, rollback to the known-good RM -- is in nicogpu_dfx_core.c (pure C,
 * tested on the host against a register-accurate mock shell).  This file supplies the shell/RM register access, the
 * PR_DATA page writes, the world-stopping around a PR (no VM may exist, BAR2 mappings are revoked, the scheduler and
 * the CP are stopped), the known-good container copy, the blank-RM fallback (request_firmware) and the devcoredump
 * after a forced quiesce / failed load.
 *
 * Not compiled on the host: checked only against stub headers.
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/capability.h>
#include <linux/firmware.h>
#include <linux/seq_file.h>
#include "nicogpu_drv.h"
#include "nicogpu_img.h"

#define NICOGPU_PR_DATA		0x1000		/* BAR1: any 32-bit write in this page pushes one word */
#define NICOGPU_RM_BLANK_FW	"nicogpu/rm_blank.ngpr"
#define NICOGPU_RM_BLANK_FW_OLD	"nicogpu/rm_blank.xgpr"	/* name before, tried second */

/* the core carries its own copy of the shell register map: it must be the one of nicogpu_regs.h */
static_assert(XDF_SH_ID_MAGIC == SH_ID_MAGIC && XDF_SH_VERSION == SH_VERSION && XDF_SH_UID_LO == SH_UID_LO &&
	      XDF_SH_UID_HI == SH_UID_HI && XDF_SH_IRQ_EOI == SH_IRQ_EOI && XDF_RM_CTRL == RM_CTRL &&
	      XDF_RM_STATUS == RM_STATUS && XDF_RM_INFO0 == RM_INFO0 && XDF_RM_ERR_COUNT == RM_ERR_COUNT &&
	      XDF_PR_STATUS == PR_STATUS && XDF_ST_DECOUPLED == RM_ST_DECOUPLED && XDF_ST_RM_READY == RM_ST_READY &&
	      XDF_RM_ID == RM_ID && XDF_RM_ID_MAGIC == RM_ID_MAGIC && XDF_RM_ID_MAGIC_LEGACY == RM_ID_MAGIC_LEGACY, "nicogpu_dfx_core.h register map == nicogpu_regs.h");
static_assert(NICOGPU_PR_MAX_BYTES < (1ull << 32), "the container length must fit NGPR.payload_bytes");
static_assert(XDF_SH_CAPS == SH_CAPS && XDF_CAPS_ICAP_RB == SH_CAPS_ICAP_RB && XDF_PR_OPT == PR_OPT &&
	      XDF_PR_ICAP_O == PR_ICAP_O && XDF_PR_ICAP_OSTAT == PR_ICAP_OSTAT && XDF_PR_ICAP_WORDS == PR_ICAP_WORDS &&
	      XDF_PR_RB_CTRL == PR_RB_CTRL && XDF_PR_RB_STATUS == PR_RB_STATUS && XDF_PR_RB_DATA0 == PR_RB_DATA0 &&
	      XDF_PRS_AVAIL_LOST == PR_ST_AVAIL_LOST && XDF_PRS_EOS == PR_ST_EOS, "register map == nicogpu_regs.h");
static_assert(XDF_PRS_BUSY == PR_ST_BUSY && XDF_PRS_DONE == PR_ST_DONE && XDF_PRS_ERROR == PR_ST_ERROR &&
	      XDF_PRS_ECODE_SHIFT == PR_ST_ECODE_SHIFT && XDF_PRS_ICAP_PRERROR == PR_ST_ICAP_PRERR &&
	      XDF_PRS_ICAP_PRDONE == PR_ST_ICAP_PRDONE && XDF_PRS_PRERROR_RAW == PR_ST_PRERR_RAW &&
	      XDF_PRS_PRDONE_RAW == PR_ST_PRDONE_RAW && XDF_PRS_ICAP_AVAIL == PR_ST_ICAP_AVAIL &&
	      XDF_PRS_PRDONE_FELL == PR_ST_PRDONE_FELL && XDF_PRS_EOS_FELL == PR_ST_EOS_FELL, "PR_STATUS bits == nicogpu_regs.h");

/* pr_opt is written to PR_OPT before START when non-zero (bit fields of PR_OPT).  Default (all ones) = auto: ORDER 1 on shells with SHELL_CAPS.ICAP_RB (SHELL_REV 3, b18), where it makes the configuration engine parse the stream (verified on the board 2026-10-26); 0 on older shells.
 * 0 = keep the shell's reset value: ORDER 0 up to SHELL_REV 3 (R1 word order, which the engine ignores, PR is a no-op),
 * ORDER 1 from SHELL_REV 4 (b19).  Auto writes ORDER 1 explicitly on every ICAP_RB shell, so it behaves
 * the same on SHELL_REV 3 and 4. */
static unsigned int pr_opt = ~0u;
module_param(pr_opt, uint, 0644);
MODULE_PARM_DESC(pr_opt, "PR_OPT value written before every PR START (default auto: ORDER 1 on ICAP_RB shells; 0 = shell reset value)");
static unsigned int icap_rb_skip;
module_param(icap_rb_skip, uint, 0644);
MODULE_PARM_DESC(icap_rb_skip, "debugfs icap: read cycles skipped before the 8 captured O words");

/* ---- struct xdf_ops ---------------------------------------------------------------- */

static void op_shell_wr(void *ctx, u32 off, u32 val)
{
	ng_wr(ctx, NICOGPU_BAR_SHELL, off, val);
}

/* 32-bit posted writes into the PR_DATA page, file byte order (the shell restores the bitstream's big-endian words
 * and the ICAP primitive bit-reverses them).  The _rep form keeps every store
 * a separate 32-bit write to the same address, in order. */
static void op_pr_push(void *ctx, const u8 *data, u32 len)
{
	struct nicogpu_dev *x = ctx;

	iowrite32_rep(x->bar[NICOGPU_BAR_SHELL] + NICOGPU_PR_DATA, data, len / 4);
	cond_resched();
}

static void op_event(void *ctx, enum xdf_event ev, u64 a, u64 b)
{
	struct nicogpu_dev *x = ctx;
	struct device *d = &x->pdev->dev;

	switch (ev) {
	case XDF_EV_QUIESCE_FORCED:
		dev_warn(d, "RM did not quiesce (RM_STATUS %08llx, outstanding %08llx): FORCE isolation\n", a, b);
		x->dfx.dump_pending = true;		/* taken after the flow: BAR0 of a hung RM is not read now */
		x->dfx.dump_reason = XDD_R_QUIESCE_FORCED;
		break;
	case XDF_EV_PR_START:
		dev_info(d, "PR: %llu payload bytes, build %07llx\n", a, b);
		break;
	case XDF_EV_PR_ERROR:
		dev_err(d, "PR/GPU reset stage %llu failed (%lld)\n", a, -(s64)(s32)b);
		x->dfx.dump_pending = true;
		x->dfx.dump_reason = XDD_R_PR_FAILED;
		break;
	case XDF_EV_RETRY:
		dev_warn(d, "PR: retrying the same container once\n");
		break;
	case XDF_EV_ROLLBACK:
		dev_warn(d, "PR: rollback to the %s RM: %s\n", a == 1 ? "known-good" : "blank fallback", b ? "FAILED" : "ok");
		break;
	case XDF_EV_WEDGED:
		dev_err(d, "no RM could be loaded: device wedged (reboot / IPROG needed)\n");
		nicogpu_note_lost(x, NICOGPU_LOST_PR_WEDGED, x->dfx.core.last_err);
		x->dfx.dump_pending = true;
		x->dfx.dump_reason = XDD_R_PR_FAILED;
		break;
	case XDF_EV_LOADED:
		dev_info(d, "PR: RM build %07llx kind %04llx loaded and released\n", a, b);
		break;
	case XDF_EV_GPU_RESET:
		if (a)
			dev_err(d, "GPU-zone soft reset failed (%lld)%s\n", -(s64)(s32)a, b ? ": reloading the RM" : "");
		break;
	case XDF_EV_PR_DIAG: {
		const struct xdf_pr_diag *g = &x->dfx.core.last_pr;

		dev_info(d, "PR diag: PR_STATUS %08llx (avail %u avail_lost %u prdone_fell %u eos_fell %u) PR_WORDS %llu "
			 "ICAP_WORDS %llu O %08x OSTAT %08x PR_OPT %03x\n", a, !!(a & PR_ST_ICAP_AVAIL), !!(a & PR_ST_AVAIL_LOST),
			 !!(a & PR_ST_PRDONE_FELL), !!(a & PR_ST_EOS_FELL),
			 b & 0xffffffffull, b >> 32, g->icap_o, g->icap_ostat, g->pr_opt);
		if ((b >> 32) != (b & 0xffffffffull) || (a & XDF_PRS_AVAIL_LOST))
			dev_warn(d, "PR diag: the ICAP did not take every word cleanly\n");
		break;
	}
	}
}

/* the blank RM container: kept until the flow ends (the core hands out pointers into it) */
static const u8 *op_fallback(void *ctx, size_t *len)
{
	struct nicogpu_dev *x = ctx;

	if (!x->dfx.fallback && request_firmware(&x->dfx.fallback, NICOGPU_RM_BLANK_FW, &x->pdev->dev) &&
	    request_firmware(&x->dfx.fallback, NICOGPU_RM_BLANK_FW_OLD, &x->pdev->dev))
		x->dfx.fallback = NULL;
	if (!x->dfx.fallback)
		return NULL;
	*len = x->dfx.fallback->size;
	return x->dfx.fallback->data;
}

static const struct xdf_ops nicogpu_dfx_ops = {
	.shell_rd = ng_op_shell_rd,
	.shell_wr = op_shell_wr,
	.rm_rd = ng_op_rm_rd,
	.pr_push = op_pr_push,
	.msleep = ng_op_msleep,
	.event = op_event,
	.prepare = NULL,		/* the world is stopped by nicogpu_pr_load_buf() before the core runs */
	.done = NULL,
	.fallback = op_fallback,
};

/* ---- lifecycle ----------------------------------------------------------------------- */

void nicogpu_dfx_init(struct nicogpu_dev *x)
{
	mutex_init(&x->dfx.lock);
	mutex_init(&x->dfx.pr_mutex);
}

static void dfx_release_fallback(struct nicogpu_dev *x)
{
	if (x->dfx.fallback)
		release_firmware(x->dfx.fallback);
	x->dfx.fallback = NULL;
}

void nicogpu_dfx_fini(struct nicogpu_dev *x)
{
	dfx_release_fallback(x);
	vfree(x->dfx.good);
	x->dfx.good = NULL;
	x->dfx.good_len = 0;
}

/* dfx.lock held: read SHELL_UID / iface once */
static int dfx_ensure(struct nicogpu_dev *x)
{
	struct xdf_cfg cfg;
	int rc;

	if (x->dfx.inited)
		return 0;
	xdf_cfg_default(&cfg);
	if (cfg.max_payload > NICOGPU_PR_MAX_BYTES - XIM_NGPR_HDR_SIZE)
		cfg.max_payload = NICOGPU_PR_MAX_BYTES - XIM_NGPR_HDR_SIZE;
	rc = xdf_init(&x->dfx.core, &nicogpu_dfx_ops, x, &cfg);
	if (!rc)
		x->dfx.inited = true;
	return rc;
}

void nicogpu_dfx_fill_dump(struct nicogpu_dev *x, struct xdd_dfx *out)
{
	struct xdf *d = &x->dfx.core;

	out->valid = x->dfx.inited;
	if (!out->valid)
		return;
	out->outcome = d->last;
	out->stage = d->last_stage;
	out->err = d->last_err;
	out->wedged = d->wedged;
	out->st = d->st;
}

static void dfx_post_dump(struct nicogpu_dev *x)
{
	enum xdd_reason r;

	if (!x->dfx.dump_pending)
		return;
	r = x->dfx.dump_reason;
	x->dfx.dump_pending = false;
	nicogpu_dump(x, r);
}

/* ---- GPU-zone soft reset (called from the scheduler worker's reset ladder) ---------------- */

int nicogpu_dfx_gpu_reset(struct nicogpu_dev *x)
{
	int rc;

	mutex_lock(&x->dfx.lock);
	rc = dfx_ensure(x);
	if (!rc)
		rc = xdf_gpu_reset(&x->dfx.core);
	mutex_unlock(&x->dfx.lock);
	dfx_post_dump(x);
	return rc;
}

/* ---- partial reconfiguration ------------------------------------------------------------- */

static u32 pr_result(const struct xdf *d)
{
	switch (d->last) {
	case XDF_OUT_LOADED: return NICOGPU_PR_LOADED;
	case XDF_OUT_REJECTED: return NICOGPU_PR_REJECTED;
	case XDF_OUT_RESTORED: return NICOGPU_PR_RESTORED;
	case XDF_OUT_WEDGED: return NICOGPU_PR_WEDGED;
	default: return 0;
	}
}

/* `buf` is a kernel copy of the container (vmalloc); *kept: it became the known-good copy
 * and the caller must not free it.  The caller holds nothing; pr_mutex serialises loads. */
static int pr_load_buf(struct nicogpu_dev *x, u8 *buf, size_t len, struct nicogpu_pr_load *out, bool *kept)
{
	struct nicogpu_dfx *d = &x->dfx;
	int rc;

	*kept = false;
	mutex_lock(&d->pr_mutex);
	/* step 1 before anything is stopped: a bad container must cost nothing */
	mutex_lock(&d->lock);
	rc = dfx_ensure(x);
	if (!rc)
		rc = xdf_check(&d->core, buf, len, NULL);
	mutex_unlock(&d->lock);
	if (rc) {
		out->result = NICOGPU_PR_REJECTED;
		out->error = rc;
		goto out;
	}
	/* step 2: stop the world -- no VM (hence no queue) may exist and none can appear until the end */
	rc = nicogpu_mm_pr_begin(x);
	if (rc) {
		out->result = NICOGPU_PR_REJECTED;
		out->error = rc;
		goto out;
	}
	if (x->mapping)
		unmap_mapping_range(x->mapping, 0, 0, 1);	/* nobody may read all-ones through a BAR2 mapping */
	/* the scheduler and the CP stop (CP off, IRQs off, ring and fence page freed); the next queue / syncobj ioctl
	 * brings them up again */
	nicogpu_sched_fini(x);
	mutex_lock(&d->lock);
	d->core.cfg.pr_opt = READ_ONCE(pr_opt);
	if (d->core.cfg.pr_opt == ~0u)
		d->core.cfg.pr_opt = (d->core.shell_caps & XDF_CAPS_ICAP_RB) ? 1 : 0;
	rc = xdf_pr_load(&d->core, buf, len);
	out->result = pr_result(&d->core);
	out->stage = d->core.last_stage;
	out->error = d->core.last_err;
	if (!rc) {
		u8 *old = d->good;

		d->good = buf;				/* the RM that works: the rollback target of the next load */
		d->good_len = len;
		xdf_set_good(&d->core, buf, len);
		*kept = true;
		vfree(old);
	}
	mutex_unlock(&d->lock);
	nicogpu_mm_pr_end(x, d->core.last != XDF_OUT_REJECTED);
	dfx_release_fallback(x);
	dfx_post_dump(x);
out:
	mutex_unlock(&d->pr_mutex);
	return rc;
}

long nicogpu_ioc_pr_load(struct nicogpu_file *xf, void *data)
{
	struct nicogpu_dev *x = xf->x;
	struct nicogpu_pr_load *a = data;
	u8 *buf;
	bool kept;
	long rc;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	if (a->flags || a->result || a->stage || a->error || a->rsvd[0] || a->rsvd[1])
		return -EINVAL;
	if (a->size < XIM_NGPR_HDR_SIZE)
		return -EINVAL;
	if (a->size > NICOGPU_PR_MAX_BYTES)
		return -E2BIG;
	buf = vmalloc(a->size);
	if (!buf)
		return -ENOMEM;
	if (copy_from_user(buf, u64_to_user_ptr(a->data), a->size)) {
		vfree(buf);
		return -EFAULT;
	}
	rc = pr_load_buf(x, buf, a->size, a, &kept);
	if (!kept)
		vfree(buf);
	return rc;		/* result/stage/error in *a are valid with every return code (DRM copies them out) */
}

/* debugfs `pr_load`: the container comes from request_firmware(name) */
int nicogpu_dfx_pr_load_name(struct nicogpu_dev *x, const char *name)
{
	const struct firmware *fw;
	struct nicogpu_pr_load a = { };
	size_t len;
	u8 *buf;
	bool kept;
	int rc;

	rc = request_firmware(&fw, name, &x->pdev->dev);
	if (rc)
		return rc;
	len = fw->size;
	if (len < XIM_NGPR_HDR_SIZE || len > NICOGPU_PR_MAX_BYTES) {
		release_firmware(fw);
		return -EINVAL;
	}
	buf = vmalloc(len);
	if (!buf) {
		release_firmware(fw);
		return -ENOMEM;
	}
	memcpy(buf, fw->data, len);
	release_firmware(fw);
	rc = pr_load_buf(x, buf, len, &a, &kept);
	if (!kept)
		vfree(buf);
	return rc;
}

/* debugfs `icap`: the PR/ICAP registers and, on a SHELL_CAPS.ICAP_RB shell, an ICAP readback of STAT / IDCODE /
 * BOOTSTS / CTL0 / COR0 of the master SLR (8 raw O words each, and their decode with the current PR_OPT.ORDER; which
 * word carries the register is measured on the board).  Readback writes only sync, a read header, NOPs and DESYNC to
 * the configuration engine (no frames), so it is allowed with a running RM; it is serialised against PR. */
void nicogpu_dfx_icap_show(struct nicogpu_dev *x, struct seq_file *m)
{
	static const struct { u32 reg; const char *name; } regs[] = {
		{ XDF_CFG_STAT, "STAT" }, { XDF_CFG_IDCODE, "IDCODE" }, { XDF_CFG_BOOTSTS, "BOOTSTS" },
		{ XDF_CFG_CTL0, "CTL0" }, { XDF_CFG_COR0, "COR0" },
	};
	struct nicogpu_dfx *d = &x->dfx;
	u32 st, opt, out[8];
	int i, j, rc;

	mutex_lock(&d->pr_mutex);
	mutex_lock(&d->lock);
	rc = dfx_ensure(x);
	if (rc) {
		seq_printf(m, "no shell (%d)\n", rc);
		goto out;
	}
	st = ng_op_shell_rd(x, XDF_PR_STATUS);
	opt = ng_op_shell_rd(x, XDF_PR_OPT);
	seq_printf(m, "SHELL_CAPS     %08x (icap_rb %u)\n", d->core.shell_caps, !!(d->core.shell_caps & XDF_CAPS_ICAP_RB));
	seq_printf(m, "PR_STATUS      %08x (busy %u done %u err %u ecode %u | latched prerr %u prdone %u | raw prerr %u "
		   "prdone %u | eos %u avail %u avail_lost %u prdone_fell %u eos_fell %u)\n", st, !!(st & PR_ST_BUSY),
		   !!(st & PR_ST_DONE), !!(st & PR_ST_ERROR), st >> PR_ST_ECODE_SHIFT & 0xf, !!(st & PR_ST_ICAP_PRERR),
		   !!(st & PR_ST_ICAP_PRDONE), !!(st & PR_ST_PRERR_RAW), !!(st & PR_ST_PRDONE_RAW), !!(st & PR_ST_EOS),
		   !!(st & PR_ST_ICAP_AVAIL), !!(st & PR_ST_AVAIL_LOST), !!(st & PR_ST_PRDONE_FELL), !!(st & PR_ST_EOS_FELL));
	seq_printf(m, "PR_OPT         %03x (order %u done_sel %u avail_gate %u)  module pr_opt %03x\n", opt, opt & PR_OPT_ORDER_MASK,
		   opt >> PR_OPT_DONE_SEL_SHIFT & 3, !!(opt & PR_OPT_AVAIL_GATE), READ_ONCE(pr_opt));
	seq_printf(m, "PR_WORDS       %u  PR_ICAP_WORDS %u\n", ng_op_shell_rd(x, XDF_PR_WORDS), ng_op_shell_rd(x, XDF_PR_ICAP_WORDS));
	seq_printf(m, "PR_ICAP_O      %08x OSTAT %08x\n", ng_op_shell_rd(x, XDF_PR_ICAP_O), ng_op_shell_rd(x, XDF_PR_ICAP_OSTAT));
	if (d->core.last_pr.valid)
		seq_printf(m, "last PR        PR_STATUS %08x PR_WORDS %u ICAP_WORDS %u O %08x\n", d->core.last_pr.pr_status,
			   d->core.last_pr.pr_words, d->core.last_pr.icap_words, d->core.last_pr.icap_o);
	if (!(d->core.shell_caps & XDF_CAPS_ICAP_RB) || (st & XDF_PRS_BUSY))
		goto out;
	for (i = 0; i < (int)ARRAY_SIZE(regs); i++) {
		rc = xdf_icap_readback(&d->core, regs[i].reg, 1, READ_ONCE(icap_rb_skip), out);
		if (rc) {
			seq_printf(m, "RB %-7s    failed (%d)\n", regs[i].name, rc);
			continue;
		}
		seq_printf(m, "RB %-7s    raw", regs[i].name);
		for (j = 0; j < 8; j++)
			seq_printf(m, " %08x", out[j]);
		seq_printf(m, "  decoded");
		for (j = 0; j < 8; j++)
			seq_printf(m, " %08x", xdf_icap_decode(opt, out[j]));
		seq_printf(m, "\n");
	}
out:
	mutex_unlock(&d->lock);
	mutex_unlock(&d->pr_mutex);
}
