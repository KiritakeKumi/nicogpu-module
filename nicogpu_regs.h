/* SPDX-License-Identifier: GPL-2.0 */
/* nicogpu_regs.h -- BAR1 shell register map (SHELL_REV 2; additions = SHELL_REV 3)
 * and the few BAR0 (RM) registers the driver touches. */
#ifndef NICOGPU_REGS_H
#define NICOGPU_REGS_H

#define NICOGPU_PCI_VENDOR	0x10ee
#define NICOGPU_PCI_DEVICE	0x9031		/* pf0_device_id of the shell */

#define NICOGPU_BAR_CTRL	0		/* 1 MiB, RM registers (isolated => reads all-ones) */
#define NICOGPU_BAR_SHELL	1		/* 64 KiB, shell registers */
#define NICOGPU_BAR_MEM	2		/* 256 MiB, 64-bit prefetchable, RM memory window */
#define NICOGPU_NUM_BARS	3

/* BAR1 */
#define SH_ID		0x0000		/* "XSHL" */
#define SH_ID_MAGIC	0x4C485358
#define SH_VERSION	0x0004		/* [15:0] shell rev, [31:16] iface_version_code */
#define SH_CAPS		0x0008		/* ICAP[0] CLK_DRP[1] JTAG_AXI[2] ICAP_RB[3] MEM_PRESENT[11:8] MSI_VECTORS[15:12] */
#define  SH_CAPS_ICAP		BIT(0)
#define  SH_CAPS_CLK_DRP	BIT(1)
#define  SH_CAPS_JTAG_AXI	BIT(2)
#define  SH_CAPS_MEM_SHIFT	8		/* MEM_PRESENT[11:8] */
#define  SH_CAPS_MSI_SHIFT	12		/* MSI_VECTORS[15:12] */
#define  SH_CAPS_ICAP_RB	BIT(3)		/* SHELL_REV 3: PR_ICAP_WORDS, PR_RB_*, PR_STATUS[16:13], PR_OPT[8:6] */
#define SH_SCRATCH	0x000C
#define SH_UID_LO	0x0010
#define SH_UID_HI	0x0014
#define SH_STATUS	0x0020		/* SI_DONE[0] SI_ERR[1] MMCM[2] CALIB[7:4] LINK[8] FENCE_STUCK[9] */
#define  SH_ST_SI_DONE		BIT(0)
#define  SH_ST_SI_ERR		BIT(1)
#define  SH_ST_MMCM		BIT(2)
#define  SH_ST_CALIB_SHIFT	4		/* CALIB[7:4] */
#define  SH_ST_LINK		BIT(8)
#define  SH_ST_FENCE_STUCK	BIT(9)
#define SH_IRQ_STATUS	0x0040		/* W1C: PR_DONE PR_ERROR QUIESCE_TIMEOUT RM_INCOMPAT MEM_ERROR */
#define SH_IRQ_ENABLE	0x0044
#define SH_IRQ_EOI	0x0048		/* [0] vec0 [1] vec1 re-arm */
#define  SH_IRQ_ALL	0x1F
#define RM_CTRL		0x0100
#define RM_STATUS	0x0104
#define  RM_ST_DECOUPLED	BIT(2)
#define  RM_ST_IN_RESET		BIT(3)
#define  RM_ST_READY		BIT(4)
#define  RM_ST_INFO_VALID	BIT(5)
#define  RM_ST_IFACE_OK		BIT(6)
#define  RM_ST_MEM_OK		BIT(8)
#define RM_INFO0	0x0110
#define RM_INFO1	0x0114
#define RM_INFO2	0x0118
#define RM_INFO3	0x011C
#define RM_ERR_COUNT	0x0120
#define PR_STATUS	0x0204		/* BUSY[0] DONE[1] ERROR[2] ECODE[7:4] ICAP_PRERROR[8] ICAP_PRDONE[9] raw PRERROR[10] raw PRDONE[11] EOS[12] */
/* PR diagnostics (shell build with PR_OPT) */
#define PR_OPT		0x0214		/* RW: ORDER[1:0] CSIB_CONT[2] OBS_HOLD[3] CSIB_FORCE[4] STORE[5]; reset 0, ORDER resets to 1 from SHELL_REV 4 */
#define  PR_OPT_ORDER_MASK	0x3
#define  PR_OPT_CSIB_CONT	BIT(2)
#define  PR_OPT_OBS_HOLD	BIT(3)
#define  PR_OPT_CSIB_FORCE	BIT(4)
#define  PR_OPT_STORE		BIT(5)
#define  PR_OPT_DONE_SEL_SHIFT	6		/* 0 latched PRDONE, 1 PRDONE edge, 2 EOS edge, 3 drain only */
#define  PR_OPT_AVAIL_GATE	BIT(8)
#define PR_ICAP_O	0x0218		/* RO: last non-zero ICAPE3.O word */
#define PR_ICAP_OSTAT	0x021C		/* RO: O_VALID[0] O_CHG[15:8] */
#define PR_ICAP_O_H0	0x0220		/* RO: O change log, newest H0, H3 oldest */
#define PR_ICAP_O_H1	0x0224
#define PR_ICAP_O_H2	0x0228
#define PR_ICAP_O_H3	0x022C
#define  PR_ST_BUSY		BIT(0)
#define  PR_ST_DONE		BIT(1)
#define  PR_ST_ERROR		BIT(2)
#define  PR_ST_ECODE_SHIFT	4		/* ECODE[7:4] */
#define  PR_ST_ICAP_PRERR	BIT(8)
#define  PR_ST_ICAP_PRDONE	BIT(9)
#define  PR_ST_PRERR_RAW	BIT(10)
#define  PR_ST_PRDONE_RAW	BIT(11)
#define  PR_ST_EOS		BIT(12)
#define  PR_ST_ICAP_AVAIL	BIT(13)		/* SH_CAPS_ICAP_RB shells */
#define  PR_ST_AVAIL_LOST	BIT(14)
#define  PR_ST_PRDONE_FELL	BIT(15)
#define  PR_ST_EOS_FELL		BIT(16)
#define PR_ICAP_WORDS	0x0230		/* RO: data words strobed into ICAPE3 since START */
#define PR_RB_CTRL	0x0234		/* RW: REG[4:0] COUNT[15:8] SKIP[21:16] NOPS[27:24] GO[31] */
#define PR_RB_STATUS	0x0238		/* RO: BUSY[0] DONE[1] REJECTED[2] SEQ[15:8] */
#define PR_RB_DATA0	0x0240		/* RO: .. DATA7 0x025C, raw ICAPE3.O of read cycles SKIP..SKIP+7 */
#define CLK_CTRL	0x2000
#define CLK_STATUS	0x2004
#define CLK_DRP_ADDR	0x2008
#define CLK_DRP_DATA	0x200C
#define CLK_FREQ_CORE	0x2010		/* kHz */
#define CLK_FREQ_L2	0x2014
#define CLK_FREQ_TS	0x2018
#define CLK_FREQ_CORE2X	0x201C

/* BAR0 (RM) -- only valid while RM_STATUS.DECOUPLED == 0 */
#define RM_ID		0x0000		/* "NGPU" 0x5550474E; RMs built before b20 read "XGPU" 0x55504758 */
#define  RM_ID_MAGIC	0x5550474E
#define  RM_ID_MAGIC_LEGACY	0x55504758
#define RM_REV		0x0004		/* [15:0] RTL/model revision, [31:16] regmap version */
#define RM_CAPS		0x0008		/* [3:0] cores [7:4] SP/core [15:8] warp size [23:16] VMID count [24] SFU [25] XLANE [26] FP64 [27] UNITS_VALID [28] SCRATCH_ILV, informational [29] F16CVT */
#define RM_TIMESTAMP_LO	0x0050		/* RO: reading LO latches HI */
#define RM_TIMESTAMP_HI	0x0054
#define RM_TIMESTAMP_FREQ 0x0058	/* RO, Hz */
#define RM_SCRATCH	0x0010		/* blank/test RM byte-strobed scratch */
#define RM_IRQ_STATUS	0x0040		/* W1C; 0x10 is SCRATCH */

/* BAR0 BAR2 host window / C3 aperture.
 * MMU context/invalidate registers come from gen/nicogpu_pte.h (ptegen). */
#define RM_BAR2_INFO	0x0400		/* [7:0] BAR2 log2 (28) [15:8] min win log2 (20) [23:16] max (27) */
#define  RM_BAR2_INFO_V02	0x001B141C	/* exact value of the v0.2 layout */
#define RM_C3_WIN_CTRL	0x0404		/* [0] ENABLE [12:8] SIZE_LOG2 [31] ERR (RO) */
#define  C3_WIN_ENABLE		BIT(0)
#define  C3_WIN_SIZE_SHIFT	8
#define  C3_WIN_ERR		BIT(31)
#define RM_C3_WIN_BASE	0x0408		/* C3 byte offset, window-size aligned */

/* BAR0, rm_sp debug page (only on an rm_sp RM: nicogpu_rm_kind() == NGI_RM_KIND_SP) */
#define RSP_DBG_SP_STATUS	0x0804		/* bit 30 = SP idle (no dispatch in the work distributor / warps) */
#define  RSP_DBG_SP_IDLE	BIT(30)
#define RSP_DBG_COMMIT_TOTAL	0x0824		/* retired-instruction count; moves while a long dispatch runs */

#endif
