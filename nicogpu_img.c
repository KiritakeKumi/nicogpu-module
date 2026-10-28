#include "nicogpu_img.h"

#ifdef __KERNEL__
#include <crypto/sha2.h>
#include <linux/crc32.h>
#if __has_include(<linux/unaligned.h>)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#define le16(p)	get_unaligned_le16(p)
#define le32(p)	get_unaligned_le32(p)
#define le64(p)	get_unaligned_le64(p)

/* ---- CRC-32 -------------------------------------------------------------------- */

uint32_t xim_crc32(uint32_t crc, const void *buf, size_t len)
{
	/* zlib convention: the kernel's crc32_le() works on the raw (un-inverted) register */
	return ~crc32_le(~crc, buf, len);
}
#else	/* host build: portable C, no kernel helpers */

static uint16_t le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t le64(const uint8_t *p)
{
	return (uint64_t)le32(p) | ((uint64_t)le32(p + 4) << 32);
}

/* ---- CRC-32 -------------------------------------------------------------------- */

uint32_t xim_crc32(uint32_t crc, const void *buf, size_t len)
{
	const uint8_t *p = buf;
	size_t i;
	int k;

	crc = ~crc;
	for (i = 0; i < len; i++) {
		crc ^= p[i];
		for (k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
	}
	return ~crc;
}

/* ---- SHA-256 -------------------------------------------------------------------- */

static const uint32_t K256[64] = {
	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
	0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
	0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
	0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
	0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
	0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

#define ROR32(x, n)	(((x) >> (n)) | ((x) << (32 - (n))))

static void sha256_block(uint32_t h[8], const uint8_t *p)
{
	uint32_t w[64], a, b, c, d, e, f, g, hh, t1, t2;
	int i;

	for (i = 0; i < 16; i++)
		w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) | ((uint32_t)p[4 * i + 2] << 8) |
		       (uint32_t)p[4 * i + 3];
	for (i = 16; i < 64; i++) {
		uint32_t s0 = ROR32(w[i - 15], 7) ^ ROR32(w[i - 15], 18) ^ (w[i - 15] >> 3);
		uint32_t s1 = ROR32(w[i - 2], 17) ^ ROR32(w[i - 2], 19) ^ (w[i - 2] >> 10);

		w[i] = w[i - 16] + s0 + w[i - 7] + s1;
	}
	a = h[0]; b = h[1]; c = h[2]; d = h[3];
	e = h[4]; f = h[5]; g = h[6]; hh = h[7];
	for (i = 0; i < 64; i++) {
		uint32_t S1 = ROR32(e, 6) ^ ROR32(e, 11) ^ ROR32(e, 25);
		uint32_t ch = (e & f) ^ (~e & g);
		uint32_t S0 = ROR32(a, 2) ^ ROR32(a, 13) ^ ROR32(a, 22);
		uint32_t maj = (a & b) ^ (a & c) ^ (b & c);

		t1 = hh + S1 + ch + K256[i] + w[i];
		t2 = S0 + maj;
		hh = g; g = f; f = e; e = d + t1;
		d = c; c = b; b = a; a = t1 + t2;
	}
	h[0] += a; h[1] += b; h[2] += c; h[3] += d;
	h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}

void xim_sha256_init(struct xim_sha256 *s)
{
	s->h[0] = 0x6a09e667; s->h[1] = 0xbb67ae85; s->h[2] = 0x3c6ef372; s->h[3] = 0xa54ff53a;
	s->h[4] = 0x510e527f; s->h[5] = 0x9b05688c; s->h[6] = 0x1f83d9ab; s->h[7] = 0x5be0cd19;
	s->len = 0;
	s->n = 0;
}

void xim_sha256_update(struct xim_sha256 *s, const void *data, size_t len)
{
	const uint8_t *p = data;

	s->len += len;
	if (s->n) {
		size_t take = 64 - s->n;

		if (take > len)
			take = len;
		memcpy(s->buf + s->n, p, take);
		s->n += (uint32_t)take;
		p += take;
		len -= take;
		if (s->n == 64) {
			sha256_block(s->h, s->buf);
			s->n = 0;
		}
	}
	while (len >= 64) {
		sha256_block(s->h, p);
		p += 64;
		len -= 64;
	}
	if (len) {
		memcpy(s->buf, p, len);
		s->n = (uint32_t)len;
	}
}

void xim_sha256_final(struct xim_sha256 *s, uint8_t out[32])
{
	uint64_t bits = s->len * 8;
	uint8_t pad[72];
	uint32_t padlen = (s->n < 56) ? 56 - s->n : 120 - s->n;
	int i;

	memset(pad, 0, sizeof(pad));
	pad[0] = 0x80;
	for (i = 0; i < 8; i++)
		pad[padlen + i] = (uint8_t)(bits >> (56 - 8 * i));
	xim_sha256_update(s, pad, padlen + 8);
	for (i = 0; i < 8; i++) {
		out[4 * i] = (uint8_t)(s->h[i] >> 24);
		out[4 * i + 1] = (uint8_t)(s->h[i] >> 16);
		out[4 * i + 2] = (uint8_t)(s->h[i] >> 8);
		out[4 * i + 3] = (uint8_t)s->h[i];
	}
}

#endif /* !__KERNEL__ */

/* ---- NGCF ----------------------------------------------------------------------- */

static bool seg_ok(uint32_t off, uint32_t size, uint32_t image_size)
{
	return (off & 3u) == 0 && (size & 3u) == 0 && off >= XIM_NGCF_HDR_SIZE && off <= image_size &&
	       size <= image_size - off;
}

int xim_ngcf_check(const uint8_t *img, size_t len, size_t max_size, uint32_t cmd_major, uint32_t cmd_minor_min,
		   struct xim_ngcf *out)
{
	struct xim_ngcf f;
	uint32_t i;

	if (!img || len < XIM_NGCF_HDR_SIZE)
		return -XIM_EINVAL;
	if ((le32(img) != XIM_NGCF_MAGIC && le32(img) != XIM_NGCF_MAGIC_LEGACY) || le32(img + 4) != 1u || le32(img + 16) != 0)
		return -XIM_EINVAL;
	for (i = 52; i < 64; i++)
		if (img[i])
			return -XIM_EINVAL;
	f.fw_version = le32(img + 8);
	f.cmd_version = le32(img + 12);
	f.image_size = le32(img + 20);
	f.text_off = le32(img + 24);
	f.text_size = le32(img + 28);
	f.data_off = le32(img + 32);
	f.data_size = le32(img + 36);
	f.bss_size = le32(img + 40);
	f.entry = le32(img + 44);
	if (f.image_size != len || (len & 3u))		/* the loader copies dwords */
		return -XIM_EINVAL;
	if (len > max_size)
		return -XIM_E2BIG;
	if (!f.text_size || !seg_ok(f.text_off, f.text_size, f.image_size) ||
	    (f.data_size && !seg_ok(f.data_off, f.data_size, f.image_size)))
		return -XIM_EINVAL;
	if (f.data_size && f.text_off < f.data_off + f.data_size && f.data_off < f.text_off + f.text_size)
		return -XIM_EINVAL;		/* overlapping segments */
	if (f.entry >= f.text_size || (f.entry & 1u))
		return -XIM_EINVAL;
	if ((f.entry & 0xffu) != XIM_NGCF_ENTRY_LOW)	/* Ibex reset vector = boot_addr + 0x80 */
		return -XIM_EINVAL;
	if (f.text_size > XIM_NGCF_IMEM_BYTES)
		return -XIM_EINVAL;
	/* data + bss rounded up to words, in 64 bits (no wrap) */
	if ((uint64_t)f.data_size + (((uint64_t)f.bss_size + 3u) & ~(uint64_t)3u) > XIM_NGCF_DMEM_BYTES)
		return -XIM_EINVAL;
	if (xim_crc32(0, img + XIM_NGCF_HDR_SIZE, len - XIM_NGCF_HDR_SIZE) != le32(img + 48))
		return -XIM_EBADMSG;
	if ((f.cmd_version >> 16) != cmd_major || (f.cmd_version & 0xffffu) < cmd_minor_min)
		return -XIM_EPROTO;
	if (out)
		*out = f;
	return 0;
}

/* ---- NGPR ----------------------------------------------------------------------- */

int xim_ngpr_parse(const uint8_t *img, size_t len, uint32_t max_payload, struct xim_ngpr *out)
{
	struct xim_ngpr x;
	uint32_t i;

	if (!img || len < XIM_NGPR_HDR_SIZE)
		return -XIM_EINVAL;
	x.hdr_version = le16(img + 4);
	x.hdr_size = le16(img + 6);
	if ((le32(img) != XIM_NGPR_MAGIC && le32(img) != XIM_NGPR_MAGIC_LEGACY) || x.hdr_version != 1 || x.hdr_size != XIM_NGPR_HDR_SIZE)
		return -XIM_EINVAL;
	x.shell_uid = le64(img + 8);
	x.iface_version = le16(img + 0x10);
	x.rm_kind = le16(img + 0x12);
	x.build_id = le32(img + 0x14);
	x.rm_rev = le16(img + 0x18);
	x.mem_mask = img[0x1a];
	x.flags = img[0x1b];
	x.payload_bytes = le32(img + 0x1c);
	memcpy(x.sha256, img + 0x20, 32);
	x.clk_core_khz = le32(img + 0x40);
	x.clk_l2_khz = le32(img + 0x44);
	if (x.flags || (x.mem_mask & ~XIM_NGPR_RM_MEM_MASK))
		return -XIM_EINVAL;
	for (i = 0x48; i < XIM_NGPR_HDR_SIZE; i++)
		if (img[i])
			return -XIM_EINVAL;
	if (!x.payload_bytes || (x.payload_bytes & 3u))
		return -XIM_EINVAL;
	if (x.payload_bytes > max_payload)
		return -XIM_E2BIG;
	if ((uint64_t)len != (uint64_t)XIM_NGPR_HDR_SIZE + x.payload_bytes)
		return -XIM_EINVAL;
	x.payload = img + XIM_NGPR_HDR_SIZE;
	if (out)
		*out = x;
	return 0;
}

int xim_ngpr_check(const uint8_t *img, size_t len, const struct xim_ngpr_policy *pol, struct xim_ngpr *out)
{
	struct xim_ngpr x;
	uint8_t d[32];
	int rc;

	rc = xim_ngpr_parse(img, len, pol->max_payload, &x);
	if (rc)
		return rc;
	if (x.shell_uid != pol->shell_uid)
		return -XIM_ENXIO;
	if ((x.iface_version >> 8) != (pol->shell_iface >> 8) ||
	    (x.iface_version & 0xffu) > (pol->shell_iface & 0xffu))
		return -XIM_EPROTO;
#ifdef __KERNEL__
	sha256(x.payload, x.payload_bytes, d);
#else
	{
		struct xim_sha256 s;

		xim_sha256_init(&s);
		xim_sha256_update(&s, x.payload, x.payload_bytes);
		xim_sha256_final(&s, d);
	}
#endif
	if (memcmp(d, x.sha256, 32) != 0)
		return -XIM_EBADMSG;
	if (out)
		*out = x;
	return 0;
}
