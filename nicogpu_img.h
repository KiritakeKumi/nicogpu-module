/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * nicogpu_img.h -- pure-C validation of the two image containers the driver loads:
 * the CP firmware image (`NGCF`) and the partial
 * bitstream container (`NGPR`), plus the CRC-32 / SHA-256
 * they need.  No libc beyond memcpy/memset; the same file builds in the kernel
 * module and in the host tests.
 *
 * Everything here is a function of the bytes: the caller (nicogpu_dfx.c, nicogpu_sched.c)
 * obtains them with request_firmware() or from a user buffer, and for NGPR passes
 * the two numbers it read from the shell (SHELL_UID, the iface version code).
 */
#ifndef NICOGPU_IMG_H
#define NICOGPU_IMG_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#else
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#endif
#include "nicogpu_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

/* result codes: negative Linux errno values (aliases of the shared table, nicogpu_errno.h) */
#define XIM_EINVAL	NGE_INVAL	/* malformed: magic, version, sizes, reserved bits, offsets */
#define XIM_ENXIO	NGE_NXIO	/* NGPR: built for another static region (SHELL_UID) -- never write ICAP */
#define XIM_EPROTO	NGE_PROTO	/* version / interface incompatible with this driver / shell */
#define XIM_EBADMSG	NGE_BADMSG	/* CRC-32 / SHA-256 mismatch */
#define XIM_E2BIG	NGE_2BIG	/* larger than the limit the caller allows */

/* ---- CRC-32 (IEEE 802.3, reflected, as zlib's crc32()) --------------------- */
/* Incremental: crc = xim_crc32(0, a, na); crc = xim_crc32(crc, b, nb); ... */
uint32_t xim_crc32(uint32_t crc, const void *buf, size_t len);

/* ---- SHA-256 ----------------------------------------------------------------- */
/* The module uses the kernel's sha256() (crypto/sha2.h) and the kernel's crc32_le(); the portable implementations
 * below are only built for the host tests (and any non-Linux user of this core). */
#ifndef __KERNEL__
struct xim_sha256 {
	uint32_t h[8];
	uint64_t len;		/* bytes hashed so far */
	uint8_t buf[64];
	uint32_t n;		/* bytes in buf */
};
void xim_sha256_init(struct xim_sha256 *s);
void xim_sha256_update(struct xim_sha256 *s, const void *data, size_t len);
void xim_sha256_final(struct xim_sha256 *s, uint8_t out[32]);
#endif

/* ---- NGCF: CP firmware image ----------------------------------- */
#define XIM_NGCF_MAGIC		0x4643474Eu	/* bytes "NGCF", parsed by the CP boot ROM (ng_cp_boot.sv) */
#define XIM_NGCF_MAGIC_LEGACY	0x46434758u	/* bytes "XGCF": images packed before b20; still accepted for one version */
#define XIM_NGCF_HDR_SIZE	64u
/* Mirror ng_cp_boot.sv IMEM_WORDS*4 / DMEM_WORDS*4 (PROVISIONAL): text must fit IMEM,
 * data + bss (rounded up to words) must fit DMEM.  The entry's low byte must be 0x80 (Ibex reset vector). */
#define XIM_NGCF_IMEM_BYTES	(16384u * 4u)
#define XIM_NGCF_DMEM_BYTES	(8192u * 4u)
#define XIM_NGCF_ENTRY_LOW	0x80u

struct xim_ngcf {
	uint32_t fw_version, cmd_version, image_size;
	uint32_t text_off, text_size, data_off, data_size, bss_size, entry;
};

/*
 * Validate a firmware image: magic, hdr_version == 1, flags == 0, reserved
 * [52,64) == 0, image_size == len (a multiple of 4; the 4 KiB-aligned base is
 * the caller's business), image_size <= max_size, text/data segments inside [64, image_size),
 * 4-byte aligned and not overlapping, entry inside the text segment with entry[7:0] == 0x80, text <= IMEM, data + bss <= DMEM, CRC-32 over
 * [64, image_size), cmd_version major == cmd_major and minor >= cmd_minor_min.
 * 0 or -XIM_*.  *out may be NULL.
 */
int xim_ngcf_check(const uint8_t *img, size_t len, size_t max_size, uint32_t cmd_major, uint32_t cmd_minor_min,
		   struct xim_ngcf *out);

/* ---- NGPR: partial bitstream container --------------------- */
#define XIM_NGPR_MAGIC		0x5250474eu	/* bytes "NGPR" */
#define XIM_NGPR_MAGIC_LEGACY	0x52504758u	/* bytes "XGPR": containers packed before b20 stay accepted (one version) */
#define XIM_NGPR_HDR_SIZE	128u
#define XIM_NGPR_RM_MEM_MASK	0x0fu

struct xim_ngpr {
	uint16_t hdr_version, hdr_size;
	uint64_t shell_uid;
	uint16_t iface_version;		/* [15:8] major, [7:0] minor */
	uint16_t rm_kind;
	uint32_t build_id;
	uint16_t rm_rev;
	uint8_t mem_mask, flags;
	uint32_t payload_bytes;
	uint8_t sha256[32];
	uint32_t clk_core_khz, clk_l2_khz;
	const uint8_t *payload;		/* into the caller's buffer */
};

/* What the container must match: the shell's SHELL_UID and iface version code
 * (SHELL_VERSION[31:16]), the largest payload the driver accepts. */
struct xim_ngpr_policy {
	uint64_t shell_uid;
	uint16_t shell_iface;
	uint32_t max_payload;
};

/* Header and length checks only (magic, hdr_version == 1, hdr_size == 128, flags
 * == 0, mem_mask <= 0xf, reserved bytes == 0, payload_bytes non-zero multiple of
 * 4 and len == 128 + payload_bytes).  0 or -XIM_EINVAL / -XIM_E2BIG. */
int xim_ngpr_parse(const uint8_t *img, size_t len, uint32_t max_payload, struct xim_ngpr *out);

/* PR check in full: parse, shell_uid == policy (-XIM_ENXIO),
 * iface major equal and minor <= the shell's (-XIM_EPROTO), SHA-256 of the
 * payload (-XIM_EBADMSG).  The order is the order hardware could be harmed
 * in: the UID is checked before any hash work. */
int xim_ngpr_check(const uint8_t *img, size_t len, const struct xim_ngpr_policy *pol, struct xim_ngpr *out);

#ifdef __cplusplus
}
#endif

#endif /* NICOGPU_IMG_H */
