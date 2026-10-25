/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * nicogpu_dump_core.h -- devcoredump content builder, pure C.
 *
 * Collects what a devcoredump is asked to contain --
 * CP_ERROR*, MMU_FAULT_* (VMID, VA, access, level, PTE), RM_STATUS / RM_OUTSTANDING / RM_ERR_COUNT /
 * RM_INFO* / PR_STATUS / SHELL_STATUS, the ring / fence / job state of the scheduler core (a consistent
 * struct xsc_snap), the failing jobs' command buffers (VA, size: the contents are NOT copied -- the CPU never
 * reads BAR2; a copy-engine DMA capture is a follow-up) -- into one text buffer.
 *
 * The text format is PROVISIONAL (human first: "key: value" lines in [sections]); it is not an ABI.  The
 * formatter does its own number formatting so the output is byte-identical on every host (the host test
 * compares it with golden fragments) and never overruns `cap`.
 */
#ifndef NICOGPU_DUMP_CORE_H
#define NICOGPU_DUMP_CORE_H

#include "nicogpu_sched_core.h"
#include "nicogpu_dfx_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define XDD_MAX_REGS	64u

enum xdd_reason {
	XDD_R_MANUAL = 0,	/* debugfs / test */
	XDD_R_INCIDENT,		/* a job failed: cause in the snapshot's incident */
	XDD_R_QUIESCE_FORCED,	/* PR step 3 had to FORCE */
	XDD_R_PR_FAILED,	/* a PR attempt failed (xdd_dfx.stage / err) */
	XDD_R_DEVICE_LOST,	/* every reset level failed */
};

struct xdd_reg {
	const char *name;
	uint32_t off;
	uint32_t val;
	uint8_t bar;		/* 0 = BAR0 (RM), 1 = BAR1 (shell) */
	bool valid;		/* false: not read (RM isolated) */
};

struct xdd_regs {
	uint32_t n;
	bool rm_visible;	/* BAR0 was reachable and answered "XGPU" */
	struct xdd_reg r[XDD_MAX_REGS];
};

struct xdd_io {
	void *ctx;
	uint32_t (*shell_rd)(void *ctx, uint32_t off);
	uint32_t (*rm_rd)(void *ctx, uint32_t off);
};

/* Read the register set (the shell's always; the RM's only if RM_STATUS says it is not decoupled and BAR0 ID
 * reads "XGPU").  Worker context. */
void xdd_collect_regs(const struct xdd_io *io, struct xdd_regs *out);

struct xdd_dfx {		/* shell-level state of the DFX core, optional */
	bool valid;
	enum xdf_outcome outcome;
	enum xdf_stage stage;
	int err;
	bool wedged;
	struct xdf_stats st;
};

struct xdd_input {
	enum xdd_reason reason;
	uint64_t now_ms;
	const struct xsc_snap *snap;	/* may be NULL (a pure DFX dump) */
	const struct xdd_regs *regs;	/* may be NULL */
	struct xdd_dfx dfx;
	uint32_t fw_version, cmd_version;	/* CP_FW_VERSION / CP_CMD_VERSION as read, 0 if unknown */
};

/* Format into buf[cap]; returns the number of bytes used (excluding any NUL), at most cap - 1 (buf is NUL
 * terminated when cap > 0).  When the text did not fit the output ends with "[truncated]\n". */
size_t xdd_format(char *buf, size_t cap, const struct xdd_input *in);

/* Rate limiter for dump triggers: at most `max` dumps and not closer than `min_gap_ms` apart (a kernel
 * devcoredump slot holds one dump until it is read or times out; a hang storm must not flood the log). */
struct xdd_limiter {
	uint32_t max, taken;
	uint64_t min_gap_ms, last_ms;
};
void xdd_limiter_init(struct xdd_limiter *l, uint32_t max, uint64_t min_gap_ms);
bool xdd_limiter_allow(struct xdd_limiter *l, uint64_t now_ms);

/* names (for logs and the dump) */
const char *xdd_cause_name(uint32_t cause);
const char *xdd_cp_error_name(uint32_t code);
const char *xdd_mmu_fault_name(uint32_t code);

#ifdef __cplusplus
}
#endif

#endif /* NICOGPU_DUMP_CORE_H */
