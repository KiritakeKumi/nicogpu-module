/* SPDX-License-Identifier: GPL-2.0 */
/*
 * nicogpu_fence.h -- dma_fence adapter of the nicogpu scheduler core.
 *
 * The pure-C core (nicogpu_sched_core.c) keeps its own fences (`struct xsc_fence`: a ring sequence number, signalled once,
 * with an error code).  This file is the kernel-only bridge to the DRM/dma-buf world:
 *
 *   job fence  -> dma_fence       struct nicogpu_fence wraps one xsc_fence.  One fence context per queue (struct
 *                                 nicogpu_fctx, "timeline"), seqno 1, 2, ... in submission order; the core retires jobs
 *                                 strictly in order, so the dma_fences of one context signal in seqno order.
 *   dma_fence  -> job dependency  nicogpu_fence_dep(): a dma_fence a job has to wait for becomes a core syncobj holding
 *                                 an xsc_fence: an already signalled fence is the core's stub, a fence of this device
 *                                 is the core fence itself (the in-order ring does the rest), any other fence gets an
 *                                 "external" core fence signalled from a dma_fence callback (xsc_fence_signal).
 *
 * Signalling is push based: whoever makes the core retire jobs (fence IRQ, the scheduler worker, abort at remove)
 * calls nicogpu_fences_harvest() afterwards.  It polls the core fences of the pending dma_fences (kept ordered by core
 * sequence number, so it stops at the first unsignalled one) and signals the ones that are done -- never under the
 * core lock, because dma_fence callbacks may re-enter the core (nicogpu_dep_cb -> xsc_fence_signal).
 *
 * Only one harvester signals at a time (a concurrent caller leaves a note and returns; the active one repeats), which
 * keeps the dma_fences of one context signalling in seqno order even with the IRQ and the worker racing.
 *
 * Lock order: [foreign fence->lock] -> fs->lock -> core lock; a context lock (nicogpu_fctx.lock, the dma_fence lock of our
 * own fences) is only taken with nothing else held.  nicogpu_fences_harvest() is IRQ-safe; nicogpu_fence_dep() / _prepare()
 * allocate (process context), nicogpu_fences_stop_deps() sleeps.
 *
 * Kernel-only.
 */
#ifndef NICOGPU_FENCE_H
#define NICOGPU_FENCE_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/kref.h>
#include <linux/spinlock.h>
#include <linux/dma-fence.h>
#include "nicogpu_sched_core.h"

/* One queue = one dma_fence timeline.  Refcounted: fences outlive the queue (a syncobj / sync_file keeps them). */
struct nicogpu_fctx {
	struct kref ref;
	spinlock_t lock;	/* dma_fence::lock of every fence of the timeline */
	u64 context;		/* dma_fence_context_alloc(1) */
	u64 seqno;		/* last seqno handed out; the caller serialises (the submit holds the file lock) */
	char name[24];		/* "nicogpu-q<id>": timeline name for debugfs / sync_file info */
};

struct nicogpu_fences;

struct nicogpu_fence {
	struct dma_fence base;	/* first: ops->release frees through dma_fence_free */
	struct nicogpu_fctx *ctx;
	struct nicogpu_fences *fs;	/* != NULL: armed and not yet harvested (identity of the owning device); fs->lock */
	struct xsc_fence *xf;	/* the core fence; one reference held until the fence is signalled */
	struct list_head node;	/* fs->pending / the harvester's private list */
	int err;		/* core error, scratch of the harvester */
};

struct nicogpu_fence_stats {
	u64 created, signalled, errors;			/* job fences exported as dma_fence; ... with an error */
	u64 dep_signalled, dep_ours, dep_foreign;	/* nicogpu_fence_dep() outcomes */
	u32 pending, deps;				/* now */
};

struct nicogpu_fences {
	struct xsc_core *core;
	spinlock_t lock;		/* pending, deps, shut, stats; leaf below the core lock */
	struct list_head pending;	/* armed fences, ordered by core sequence number */
	struct list_head deps;		/* foreign fences a job waits for (struct nicogpu_dep) */
	bool shut;			/* nicogpu_fences_stop_deps() ran: no new foreign dependency */
	bool harvesting, again, gone;	/* one harvester at a time, so the dma_fences of a context signal in seqno order */
	void (*kick)(void *ctx);	/* a foreign dependency signalled: let the worker push jobs (IRQ-safe) */
	void *kick_ctx;
	struct nicogpu_fence_stats st;
};

void nicogpu_fences_init(struct nicogpu_fences *fs, struct xsc_core *core, void (*kick)(void *ctx), void *kick_ctx);
/* Signal every pending dma_fence whose core fence has signalled.  IRQ-safe.  `gone`: the device is being torn down,
 * every error becomes -ENODEV. */
void nicogpu_fences_harvest(struct nicogpu_fences *fs, bool gone);
/* Teardown step 1 (process context, no submit in flight): detach every foreign-fence callback and wait for the ones
 * that are running, so none touches the core afterwards. */
void nicogpu_fences_stop_deps(struct nicogpu_fences *fs);
/* Teardown step 2, after the core retired everything and nicogpu_fences_harvest(fs, true): nothing may be left. */
void nicogpu_fences_fini(struct nicogpu_fences *fs);
void nicogpu_fences_stats(struct nicogpu_fences *fs, struct nicogpu_fence_stats *out);

struct nicogpu_fctx *nicogpu_fctx_create(void);
void nicogpu_fctx_get(struct nicogpu_fctx *ctx);
void nicogpu_fctx_put(struct nicogpu_fctx *ctx);

/* Export a job fence: prepare BEFORE xsc_submit() (takes a core reference on the job's fence), then arm() after it
 * succeeded (assigns context / seqno, queues the fence for harvesting; the caller serialises per queue) or discard()
 * when it failed.  The armed fence comes with one reference for the caller (dma_fence_put when done). */
struct nicogpu_fence *nicogpu_fence_prepare(struct nicogpu_fences *fs, struct xsc_fence *xf);
void nicogpu_fence_discard(struct nicogpu_fences *fs, struct nicogpu_fence *w);
void nicogpu_fence_arm(struct nicogpu_fences *fs, struct nicogpu_fence *w, struct nicogpu_fctx *ctx);

/* A dma_fence the job must wait for -> a core syncobj (one reference, give it to xsc_syncobj_put) that carries the
 * matching core fence.  ERR_PTR(-ENOMEM), ERR_PTR(-ENODEV) after nicogpu_fences_stop_deps().  `f` is only borrowed. */
struct xsc_syncobj *nicogpu_fence_dep(struct nicogpu_fences *fs, struct dma_fence *f);

/* core error -> dma_fence error: 0 stays 0; on a lost device (or `gone`) every error is -ENODEV, otherwise the core's
 * errno is kept (-EIO: the job was the guilty one / aborted, -ECANCELED: dropped because its queue is GUILTY or its
 * context was lost) */
int nicogpu_fence_errno(int core_err, bool lost);

#endif
