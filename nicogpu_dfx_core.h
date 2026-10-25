/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * nicogpu_dfx_core.h -- pure-C shell sequences of the DFX boundary:
 *
 *   xdf_gpu_reset()  reset ladder level 2: quiesce -> isolate -> RM_RESET=1 -> 0 -> RM_READY ->
 *                    release -> re-initialise; worst case reload the last good RM
 *   xdf_pr_load()    validate the XGPR container, stop the world, quiesce, isolate +
 *                    reset, push the partial bitstream through PR_DATA, wait PR_DONE, identify the RM
 *                    (RM_INFO against the header), release, initialise; retry once on any failure after
 *                    the hardware was touched, then the last known-good RM, then the device is "wedged"
 *
 * The hardware is reached through `struct xdf_ops` (BAR1 shell registers, one BAR0 read, the PR_DATA page,
 * sleeping, time), so the same code runs in the kernel module and against a register-accurate mock shell, which
 * pins every register write of every step.  All the ops are
 * "sleeping, worker/process context": no lock is held across them and the core has no locks of its own --
 * the caller serialises (the glue holds one mutex for GPU reset and PR).
 *
 * Register offsets are those of SHELL_REV 2; nicogpu_dfx.c static_asserts them
 * against nicogpu_regs.h.
 *
 * SHELL_REV 3 (SHELL_CAPS.ICAP_RB): xdf_icap_readback() reads a configuration register of the master SLR
 * through the shell's ICAP readback sequencer (PR_RB_*), xdf_icap_decode() undoes the PR_OPT.ORDER pin transform,
 * cfg.pr_opt is written to PR_OPT before START when non-zero (0 = keep the shell's reset value = the SHELL_REV 2
 * behaviour, so the default register sequence is unchanged), and after every PR attempt on an ICAP_RB shell the
 * core snapshots the PR/ICAP diagnostics into d->last_pr and emits XDF_EV_PR_DIAG.
 */
#ifndef NICOGPU_DFX_CORE_H
#define NICOGPU_DFX_CORE_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#else
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#endif
#include "nicogpu_img.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BAR1 (shell) */
#define XDF_SH_ID		0x0000u
#define XDF_SH_ID_MAGIC		0x4C485358u	/* "XSHL" */
#define XDF_SH_VERSION		0x0004u		/* [15:0] shell rev, [31:16] iface version code */
#define XDF_SH_CAPS		0x0008u
#define  XDF_CAPS_ICAP_RB	(1u << 3)	/* PR_RB_*, PR_ICAP_WORDS, PR_STATUS[16:13], PR_OPT[8:6] */
#define XDF_SH_UID_LO		0x0010u
#define XDF_SH_UID_HI		0x0014u
#define XDF_SH_IRQ_EOI		0x0048u		/* [0] vec0 [1] vec1 */
#define XDF_RM_CTRL		0x0100u
#define  XDF_CTRL_QUIESCE_REQ	(1u << 0)
#define  XDF_CTRL_DECOUPLE	(1u << 1)
#define  XDF_CTRL_RM_RESET	(1u << 2)
#define  XDF_CTRL_IRQ_FWD_EN	(1u << 3)
#define  XDF_CTRL_FORCE		(1u << 4)
#define XDF_RM_STATUS		0x0104u
#define  XDF_ST_QUIESCE_ACK	(1u << 0)
#define  XDF_ST_DRAINED		(1u << 1)
#define  XDF_ST_DECOUPLED	(1u << 2)
#define  XDF_ST_IN_RESET	(1u << 3)
#define  XDF_ST_RM_READY	(1u << 4)
#define  XDF_ST_INFO_VALID	(1u << 5)
#define  XDF_ST_IFACE_OK	(1u << 6)
#define  XDF_ST_FORCED		(1u << 7)
#define  XDF_ST_MEM_OK		(1u << 8)
#define XDF_RM_OUTSTANDING	0x010Cu
#define XDF_RM_INFO0		0x0110u
#define XDF_RM_ERR_COUNT	0x0120u
#define XDF_PR_CTRL		0x0200u
#define  XDF_PR_START		(1u << 0)
#define  XDF_PR_FINISH		(1u << 1)
#define  XDF_PR_ABORT		(1u << 2)
#define XDF_PR_STATUS		0x0204u
#define  XDF_PRS_BUSY		(1u << 0)
#define  XDF_PRS_DONE		(1u << 1)
#define  XDF_PRS_ERROR		(1u << 2)
#define  XDF_PRS_ECODE_SHIFT	4
#define  XDF_PRS_ECODE_MASK	(0xfu << 4)
#define XDF_PR_WORDS		0x0208u
#define XDF_PR_EXPECT_WORDS	0x020Cu
#define XDF_PR_FIFO_FREE	0x0210u
#define  XDF_PRS_ICAP_PRERROR	(1u << 8)
#define  XDF_PRS_ICAP_PRDONE	(1u << 9)
#define  XDF_PRS_PRERROR_RAW	(1u << 10)
#define  XDF_PRS_PRDONE_RAW	(1u << 11)
#define  XDF_PRS_EOS		(1u << 12)
#define  XDF_PRS_ICAP_AVAIL	(1u << 13)	/* ICAP_RB shells */
#define  XDF_PRS_AVAIL_LOST	(1u << 14)
#define  XDF_PRS_PRDONE_FELL	(1u << 15)
#define  XDF_PRS_EOS_FELL	(1u << 16)
#define XDF_PR_OPT		0x0214u		/* ORDER[1:0] CSIB_CONT[2] OBS_HOLD[3] CSIB_FORCE[4] STORE[5] DONE_SEL[7:6] AVAIL_GATE[8] */
#define  XDF_OPT_ORDER_MASK	0x3u
#define  XDF_OPT_DONE_SEL_SHIFT	6
#define  XDF_OPT_AVAIL_GATE	(1u << 8)
#define XDF_PR_ICAP_O		0x0218u
#define XDF_PR_ICAP_OSTAT	0x021Cu
#define XDF_PR_ICAP_WORDS	0x0230u
#define XDF_PR_RB_CTRL		0x0234u		/* REG[4:0] COUNT[15:8] SKIP[21:16] NOPS[27:24] GO[31] (read: [31] busy) */
#define  XDF_RB_GO		(1u << 31)
#define XDF_PR_RB_STATUS	0x0238u		/* BUSY[0] DONE[1] REJECTED[2] SEQ[15:8] */
#define  XDF_RBS_BUSY		(1u << 0)
#define  XDF_RBS_DONE		(1u << 1)
#define  XDF_RBS_REJECTED	(1u << 2)
#define XDF_PR_RB_DATA0		0x0240u		/* .. DATA7 0x025C: raw ICAPE3.O of read cycles SKIP..SKIP+7 */
#define XDF_PR_CHUNK		4096u		/* one PR_DATA page */

/* configuration registers (UltraScale+ type-1 addresses) the readback is used for */
#define XDF_CFG_CTL0		0x05u
#define XDF_CFG_STAT		0x07u
#define XDF_CFG_COR0		0x09u
#define XDF_CFG_IDCODE		0x0Cu
#define XDF_CFG_BOOTSTS		0x16u

/* BAR0 (RM) */
#define XDF_RM_ID		0x0000u
#define XDF_RM_ID_MAGIC		0x55504758u	/* "XGPU" */
#define XDF_RM_REV		0x0004u		/* [15:0] RM revision */

/* RM_INFO */
#define XDF_RMINFO_MAGIC	0x4D524758u	/* "XGRM" */

enum xdf_event {
	XDF_EV_QUIESCE_FORCED,	/* a = RM_STATUS, b = RM_OUTSTANDING: devcoredump trigger (PR step 3) */
	XDF_EV_PR_START,	/* a = payload bytes, b = build_id */
	XDF_EV_PR_ERROR,	/* a = stage (enum xdf_stage), b = (u32)-errno */
	XDF_EV_RETRY,		/* the same image is loaded a second time */
	XDF_EV_ROLLBACK,	/* a = 1: known-good image, 2: fallback image; b = 0 ok, else (u32)-errno */
	XDF_EV_WEDGED,		/* nothing could be loaded: only IPROG / reboot remains */
	XDF_EV_LOADED,		/* a = build_id, b = rm_kind */
	XDF_EV_GPU_RESET,	/* a = 0 ok / else (u32)-errno of the soft reset, b = 1 if the RM had to be reloaded */
	XDF_EV_PR_DIAG,		/* ICAP_RB shells only, after every PR attempt: a = PR_STATUS,
				 * b = PR_ICAP_WORDS << 32 | PR_WORDS (d->last_pr has the rest) */
};

enum xdf_stage {		/* where a load attempt stopped */
	XDF_STAGE_ISOLATE = 1,
	XDF_STAGE_PR,
	XDF_STAGE_IDENTIFY,
	XDF_STAGE_RELEASE,
	XDF_STAGE_INIT,
};

enum xdf_outcome {
	XDF_OUT_NONE = 0,
	XDF_OUT_LOADED,		/* the requested RM runs */
	XDF_OUT_REJECTED,	/* refused before the hardware was touched (header, policy, prepare hook) */
	XDF_OUT_RESTORED,	/* the load failed, the known-good (or fallback) RM runs again */
	XDF_OUT_WEDGED,		/* nothing runs: only IPROG / reboot */
};

struct xdf_ops {
	uint32_t (*shell_rd)(void *ctx, uint32_t off);			/* BAR1 */
	void (*shell_wr)(void *ctx, uint32_t off, uint32_t val);
	uint32_t (*rm_rd)(void *ctx, uint32_t off);			/* BAR0: all-ones while isolated */
	/* PR_DATA: `len` bytes (multiple of 4, <= XDF_PR_CHUNK) in file byte order into the PR_DATA page */
	void (*pr_push)(void *ctx, const uint8_t *data, uint32_t len);
	void (*msleep)(void *ctx, uint32_t ms);
	/* optional */
	void (*event)(void *ctx, enum xdf_event ev, uint64_t a, uint64_t b);
	/* xdf_pr_load() step 2 (stop every queue, revoke BAR2 mappings ...): 0 or -errno (-EBUSY); the hardware
	 * is untouched when it fails.  done(result) is called once when the flow ends (result: 0 or the error
	 * xdf_pr_load() returns) to resume.  Both NULL: nothing to stop. */
	int (*prepare)(void *ctx);
	void (*done)(void *ctx, int result);
	/* the blank RM container to fall back to when no known-good image is set; NULL: none */
	const uint8_t *(*fallback)(void *ctx, size_t *len);
};

struct xdf_cfg {		/* all in ms; xdf_cfg_default() */
	uint32_t quiesce_ms;	/* wait QUIESCE_ACK && DRAINED before FORCE (the shell itself times out at 10 ms) */
	uint32_t decouple_ms;	/* DECOUPLE / RM_RESET to take effect */
	uint32_t ready_ms;	/* RM_READY after RM_RESET=0 */
	uint32_t release_ms;	/* !DECOUPLED after DECOUPLE=0 */
	uint32_t pr_finish_ms;	/* PR_STATUS.DONE|ERROR after FINISH */
	uint32_t info_ms;	/* RM_STATUS.INFO_VALID after PR */
	uint32_t max_payload;	/* largest accepted XGPR payload, bytes */
	uint32_t pr_opt;	/* written to PR_OPT before START when non-zero (0 = shell default, R1 order) */
};
void xdf_cfg_default(struct xdf_cfg *cfg);

struct xdf_stats {
	uint64_t loads, rejected, retries, restored, wedged, gpu_resets, forced_quiesce, pr_bytes;
};

struct xdf_pr_diag {		/* PR/ICAP registers right after the last PR attempt (ICAP_RB shells) */
	bool valid;
	uint32_t pr_status, pr_words, icap_words, icap_o, icap_ostat, pr_opt;
};

struct xdf {
	const struct xdf_ops *ops;
	void *ctx;
	struct xdf_cfg cfg;
	uint64_t shell_uid;
	uint16_t shell_iface;		/* SHELL_VERSION[31:16] */
	uint32_t shell_caps;		/* SHELL_CAPS */
	struct xdf_pr_diag last_pr;
	uint32_t ctrl;			/* shadow of RM_CTRL */
	const uint8_t *good;		/* known-good XGPR container (caller owns the memory), NULL */
	size_t good_len;
	bool wedged;
	enum xdf_outcome last;
	int last_err;			/* what the last load attempt failed with (stage in last_stage) */
	enum xdf_stage last_stage;
	uint32_t cur_rm_rev;		/* of the RM we loaded last (0 = unknown: boot-time RM) */
	bool cur_rev_known;
	struct xdf_stats st;
};

/* Reads SHELL_ID / SHELL_UID / SHELL_VERSION: -ENODEV (no shell). */
int xdf_init(struct xdf *d, const struct xdf_ops *ops, void *ctx, const struct xdf_cfg *cfg);
/* The container of the RM that is running and works; the caller keeps the memory alive. */
void xdf_set_good(struct xdf *d, const uint8_t *img, size_t len);

/* PR step 1 alone (no hardware access): the container checks of xim_xgpr_check() against this
 * shell's UID and interface version.  0 or -XIM_*; the glue calls it before it stops anything. */
int xdf_check(const struct xdf *d, const uint8_t *img, size_t len, struct xim_xgpr *out);

/* Reset ladder level 2.  0 = the RM runs again (it was reset, or reloaded from the good image);
 * -ETIMEDOUT / -EIO / -EPROTO when it did not come back and nothing could be reloaded (wedged); -ENODEV
 * when already wedged. */
int xdf_gpu_reset(struct xdf *d);

/* Returns 0 when the new RM runs.  Otherwise a negative errno and d->last tells
 * which of: REJECTED (-XIM_* from the container checks, -EBUSY from prepare; the hardware was not touched),
 * RESTORED (-EIO: the requested RM failed twice, the old one is back), WEDGED (-ENODEV). */
int xdf_pr_load(struct xdf *d, const uint8_t *img, size_t len);

/* One ICAP readback of configuration register `reg` (XDF_CFG_*) of the master SLR: `count` words in the read
 * packet header (0 = 1), the O bus of read cycles skip..skip+7 into out[0..7] (raw pin words; xdf_icap_decode() with
 * the current PR_OPT.ORDER gives the register value -- which of the 8 words carries it is a property of the silicon,
 * to be measured on the board).  The caller serialises against PR (the shell refuses while the PR FIFO holds words).
 * 0, -ENODEV (shell without SHELL_CAPS.ICAP_RB), -EBUSY (refused), -ETIMEDOUT. */
int xdf_icap_readback(struct xdf *d, uint32_t reg, uint32_t count, uint32_t skip, uint32_t out[8]);
/* the inverse of the PR_OPT.ORDER pin transform (each transform is its own inverse) */
uint32_t xdf_icap_decode(uint32_t order, uint32_t pin);

#ifdef __cplusplus
}
#endif

#endif /* NICOGPU_DFX_CORE_H */
