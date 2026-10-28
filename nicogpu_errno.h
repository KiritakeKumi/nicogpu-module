/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * nicogpu_errno.h -- the one errno table of the portable cores (scheduler, VM, image
 * validation, DFX sequences).  Cores return these values negated, the glue passes them on
 * as kernel return codes.
 *
 * Kernel flavour: the kernel's own <linux/errno.h> values, so the cores can never drift
 * from what the glue compares against (and stay right on architectures whose errno
 * numbering differs from the generic one).  Host flavour (tests, any non-Linux user): the
 * generic Linux numbers, independent of the host libc's <errno.h>.
 *
 * The per-core prefixed names (XSC_E*, XVC_E*, XIM_E*) are aliases of this table.
 */
#ifndef NICOGPU_ERRNO_H
#define NICOGPU_ERRNO_H

#ifdef __KERNEL__
#include <linux/errno.h>
#define NGE_NOENT	ENOENT
#define NGE_IO		EIO
#define NGE_NXIO	ENXIO
#define NGE_2BIG	E2BIG
#define NGE_NOMEM	ENOMEM
#define NGE_BUSY	EBUSY
#define NGE_NODEV	ENODEV
#define NGE_INVAL	EINVAL
#define NGE_NOSPC	ENOSPC
#define NGE_TIME	ETIME
#define NGE_PROTO	EPROTO
#define NGE_BADMSG	EBADMSG
#define NGE_TIMEDOUT	ETIMEDOUT
#define NGE_CANCELED	ECANCELED
#else
#define NGE_NOENT	2
#define NGE_IO		5
#define NGE_NXIO	6
#define NGE_2BIG	7
#define NGE_NOMEM	12
#define NGE_BUSY	16
#define NGE_NODEV	19
#define NGE_INVAL	22
#define NGE_NOSPC	28
#define NGE_TIME	62
#define NGE_PROTO	71
#define NGE_BADMSG	74
#define NGE_TIMEDOUT	110
#define NGE_CANCELED	125
#endif

#endif /* NICOGPU_ERRNO_H */
