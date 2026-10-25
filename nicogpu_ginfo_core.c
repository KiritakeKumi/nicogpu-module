// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * nicogpu_ginfo_core.c -- GET_GPU_INFO content.  See nicogpu_ginfo_core.h.
 */
#include "nicogpu_ginfo_core.h"

#define PAGE_4K_BIT	(1u << 0)
#define PAGE_64K_BIT	(1u << 1)
#define PAGE_2M_BIT	(1u << 2)

static bool is_pow2(uint32_t v)
{
	return v && !(v & (v - 1));
}

uint32_t xgi_ts_freq(const struct xgi_hw *hw, uint32_t *src)
{
	uint64_t hz;

	if (hw->rm_regs && hw->rm_ts_freq_hz >= XGI_TS_MIN_HZ && hw->rm_ts_freq_hz <= XGI_TS_MAX_HZ) {
		*src = XGI_F_TS_RM;
		return hw->rm_ts_freq_hz;
	}
	hz = (uint64_t)hw->shell_ts_khz * 1000u;
	if (hz >= XGI_TS_MIN_HZ && hz <= XGI_TS_MAX_HZ) {
		*src = XGI_F_TS_SHELL;
		return (uint32_t)hz;
	}
	*src = 0;
	return XGI_DEF_TS_FREQ_HZ;
}

bool xgi_rm_kind_has_vram(uint32_t rm_kind)
{
	return rm_kind != XGI_RM_KIND_CP && rm_kind != XGI_RM_KIND_SP;
}

void xgi_build(const struct xgi_hw *hw, struct xgi_info *o)
{
	uint32_t cores = XGI_DEF_CORES, sp = XGI_DEF_SP_PER_CORE, warp = XGI_DEF_WARP_SIZE, vms = XGI_DEF_MAX_VMS;
	uint32_t src = 0;

	memset(o, 0, sizeof(*o));
	if (hw->rm_regs) {
		uint32_t c = hw->rm_caps & 0xF, s = (hw->rm_caps >> 4) & 0xF, w = (hw->rm_caps >> 8) & 0xFF;
		uint32_t nv = (hw->rm_caps >> 16) & 0xFF;

		/* a CAPS word that cannot describe a GPU (all-ones of an isolated RM, a blank RM's zero) is ignored whole */
		if (c >= 1 && s >= 1 && w >= 8 && w <= 128 && is_pow2(w)) {
			cores = c;
			sp = s;
			warp = w;
			o->info_flags |= XGI_F_RM_CAPS;
			if (nv >= 2 && nv - 1 < vms)
				vms = nv - 1;
			/* the unit bits count only with a CAPS word that describes a GPU and the RM saying it reports them */
			if (hw->rm_caps & XGI_CAPS_UNITS_VALID) {
				o->sp_units = (hw->rm_caps >> XGI_CAPS_UNITS_SHIFT) & XGI_CAPS_UNITS_MASK;
				if (hw->rm_caps & XGI_CAPS_F16CVT)
					o->sp_units |= XGI_SPU_F16CVT;
				o->info_flags |= XGI_F_SP_UNITS;
			}
		}
		if (hw->rm_rev && hw->rm_rev != 0xFFFFFFFFu) {
			o->revision = hw->rm_rev;
			o->info_flags |= XGI_F_RM_REV;
		}
	}
	if (vms > hw->max_vms_limit)
		vms = hw->max_vms_limit;
	o->num_cores = cores;
	o->sp_per_core = sp;
	o->warp_size = warp;
	o->warp_slots_per_sp = XGI_DEF_WARP_SLOTS;
	o->max_vregs = XGI_DEF_MAX_VREGS;
	o->num_uregs = XGI_DEF_NUM_UREGS;
	/* a workgroup lives in one half core (<= 2 SPs): min(1024, min(sp_per_core, 2) * warp_slots * warp_size) */
	{
		uint32_t half_sp = sp < 2 ? sp : 2;
		uint64_t t = (uint64_t)half_sp * o->warp_slots_per_sp * warp;

		o->max_workgroup_threads = t < XGI_DEF_MAX_WG_THREADS ? (uint32_t)t : XGI_DEF_MAX_WG_THREADS;
	}
	o->max_shared_bytes = XGI_DEF_MAX_SHARED;
	o->max_user_uregs = XGI_DEF_MAX_USER_UREGS;
	o->va_bits = XGI_DEF_VA_BITS;
	o->page_sizes = PAGE_4K_BIT | PAGE_64K_BIT | PAGE_2M_BIT;
	o->max_vms = vms;
	o->va_user_start = hw->va_user_start;
	o->va_user_end = hw->va_user_end;
	o->vram_size = hw->vram_heap_bytes;
	o->vram_visible_size = hw->vram_heap_bytes;	/* ABI v5: the whole heap is behind BAR2 */
	o->gtt_size = hw->gtt_total_max;
	o->gtt_bo_max = hw->gtt_bo_max;
	o->vram_used = hw->vram_used;
	if (!xgi_rm_kind_has_vram(hw->rm_kind)) {
		/* the RM cannot reach VRAM: advertise no VRAM heap (user space exposes host memory types only) */
		o->info_flags |= XGI_F_NO_VRAM;
		o->vram_size = o->vram_visible_size = o->vram_used = 0;
	}
	o->gtt_used = hw->gtt_used;
	o->timestamp_freq_hz = xgi_ts_freq(hw, &src);
	o->info_flags |= src;
	if (hw->gtt_uncached)
		o->info_flags |= XGI_F_GTT_UNCACHED;
}
