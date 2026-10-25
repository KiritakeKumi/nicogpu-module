/* SPDX-License-Identifier: GPL-2.0 */
/* Kernel shim for the <stdint.h> that gen/nicogpu_pte.h (ptegen output, vendored
 * unmodified) and nicogpu_vm_core.[ch] include.  Only on the kernel include path
 * (Makefile ccflags); the host tests use the real header. */
#ifndef NICOGPU_KCOMPAT_STDINT_H
#define NICOGPU_KCOMPAT_STDINT_H

#include <linux/types.h>
#include <linux/limits.h>

#ifndef UINT64_C
#define UINT64_C(x)	x##ULL
#endif
#ifndef UINT32_C
#define UINT32_C(x)	x##U
#endif
#ifndef UINT64_MAX
#define UINT64_MAX	U64_MAX
#endif

#endif
