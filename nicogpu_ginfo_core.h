/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * nicogpu_ginfo_core.h -- what GET_GPU_INFO / GET_TIMESTAMP report (ABI v5), pure C.
 *
 * The kernel glue reads the registers and the allocator state (nicogpu_drm.c / nicogpu_mm.c) and hands them over in
 * struct ngi_hw; this file decides what is reported and which numbers are driver defaults.  Only the CAPS, REV and
 * TIMESTAMP_FREQ registers of the RM exist in the regmap; everything else (warp
 * slots, register file sizes, workgroup limits ...) is an architectural constant of ISA v0.1 and is PROVISIONAL until
 * the RM publishes it.
 */
#ifndef NICOGPU_GINFO_CORE_H
#define NICOGPU_GINFO_CORE_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#else
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* info flags: the same bit values as NICOGPU_GPU_INFO_F_* of nicogpu_uapi.h (the glue static-asserts it) */
#define NGI_F_RM_CAPS	(1u << 0)
#define NGI_F_RM_REV	(1u << 1)
#define NGI_F_TS_RM	(1u << 2)
#define NGI_F_TS_SHELL	(1u << 3)
#define NGI_F_GTT_UNCACHED (1u << 4)	/* non-coherent host: GTT BOs are CPU-uncached / write-combined */
#define NGI_F_NO_VRAM	(1u << 5)	/* the loaded RM has no VRAM path: VRAM BOs are refused, vram_size = 0 */
#define NGI_F_SP_UNITS	(1u << 6)	/* sp_units come from CAPS[26:24] (CAPS[27] UNITS_VALID set); clear = unknown, not "none" */

/* RM CAPS unit bits: [26:24] optional SP units (NGI_SPU_* = NICOGPU_SP_UNIT_*), [27] UNITS_VALID */
#define NGI_CAPS_UNITS_SHIFT	24
#define NGI_CAPS_UNITS_MASK	0x7u
#define NGI_CAPS_UNITS_VALID	(1u << 27)
#define NGI_SPU_SFU		(1u << 0)
#define NGI_SPU_XLANE		(1u << 1)
#define NGI_SPU_FP64		(1u << 2)
/* Ruled 2026-10-24: CAPS[29] F16CVT = the VALU executes the binary16 <-> binary32 conversions; reported as
 * sp_units bit 3 (NGI_SPU_F16CVT = NICOGPU_SP_UNIT_F16CVT), read only with CAPS[27] UNITS_VALID like [26:24] */
#define NGI_CAPS_F16CVT		(1u << 29)
#define NGI_SPU_F16CVT		(1u << 3)

/* RM kinds (RM_INFO1[31:16]) that are known to have NO device path to VRAM: rm_cp 0x8002 (CP + MMU,
 * no SP) and rm_sp 0x8003 (M1: CP + one SP; its fabric answers a VRAM PA with ERR_BAD_PA).  Any other kind (blank/test
 * RM, the future full RM) is assumed to serve VRAM.  rm_kind 0 = not identified. */
#define NGI_RM_KIND_CP		0x8002u
#define NGI_RM_KIND_SP		0x8003u
#define NGI_RMINFO_MAGIC	0x4D52474Eu	/* "NGRM" in RM_INFO0 */
#define NGI_RMINFO_MAGIC_LEGACY	0x4D524758u	/* "XGRM": RMs built before b20 */

/* driver defaults (PROVISIONAL, same values the sim backend and the Mesa drm backend used before ABI v5) */
#define NGI_DEF_CORES		4u
#define NGI_DEF_SP_PER_CORE	4u
#define NGI_DEF_WARP_SIZE	32u
#define NGI_DEF_WARP_SLOTS	16u
#define NGI_DEF_MAX_VREGS	128u
#define NGI_DEF_NUM_UREGS	64u
#define NGI_DEF_MAX_WG_THREADS	1024u
#define NGI_DEF_MAX_SHARED	(64u * 1024u)
#define NGI_DEF_MAX_USER_UREGS	26u
#define NGI_DEF_VA_BITS		48u
#define NGI_DEF_MAX_VMS		15u
#define NGI_DEF_TS_FREQ_HZ	100000000u	/* nominal */

/* plausibility window of a reported timestamp frequency */
#define NGI_TS_MIN_HZ		1000000u
#define NGI_TS_MAX_HZ		1000000000u

struct ngi_hw {
	bool rm_regs;			/* the RM is present and RM_ID == "NGPU": CAPS / REV / TIMESTAMP_* are the regmap's */
	uint32_t rm_rev;		/* BAR0 REV */
	uint32_t rm_kind;		/* RM_INFO1[31:16] when RM_INFO0 is "NGRM", else 0 (unknown) */
	uint32_t rm_caps;		/* BAR0 CAPS: [3:0] cores [7:4] SP/core [15:8] warp size [23:16] VMID count [26:24] SP units [27] UNITS_VALID */
	uint32_t rm_ts_freq_hz;		/* BAR0 TIMESTAMP_FREQ */
	uint32_t shell_ts_khz;		/* shell CLK_FREQ_TS, 0 = no meter */
	uint32_t max_vms_limit;		/* the driver's VMID table size - 1 */
	uint64_t va_user_start, va_user_end;
	uint64_t vram_heap_bytes;	/* what the VRAM allocator can serve */
	uint64_t gtt_total_max, gtt_bo_max;
	uint64_t vram_used, gtt_used;
	bool gtt_uncached;		/* the host is not dma-coherent: GTT is CPU-uncached (WC) memory */
};

struct ngi_info {
	uint32_t info_flags;		/* NGI_F_* */
	uint32_t sp_units;		/* NGI_SPU_*; 0 unless info_flags has NGI_F_SP_UNITS */
	uint32_t revision;
	uint32_t num_cores, sp_per_core, warp_size, warp_slots_per_sp;
	uint32_t max_vregs, num_uregs, max_workgroup_threads, max_shared_bytes, max_user_uregs;
	uint32_t va_bits, page_sizes, max_vms;
	uint64_t va_user_start, va_user_end;
	uint64_t vram_size, vram_visible_size, gtt_size, gtt_bo_max, vram_used, gtt_used;
	uint64_t timestamp_freq_hz;
};

void ngi_build(const struct ngi_hw *hw, struct ngi_info *out);

/* false for the RM kinds without a VRAM path (NGI_RM_KIND_CP / _SP), true for everything else incl. unknown (0) */
bool ngi_rm_kind_has_vram(uint32_t rm_kind);

/* timestamp frequency choice alone: RM register, else shell meter (kHz), else nominal; *src gets NGI_F_TS_* or 0 */
uint32_t ngi_ts_freq(const struct ngi_hw *hw, uint32_t *src);

/* LO is read first (it latches HI), then HI */
static inline uint64_t ngi_ts_join(uint32_t lo, uint32_t hi)
{
	return ((uint64_t)hi << 32) | lo;
}

#ifdef __cplusplus
}
#endif

#endif /* NICOGPU_GINFO_CORE_H */
