// SPDX-License-Identifier: GPL-2.0
/*
 * nicogpu_fence.c -- dma_fence adapter of the nicogpu scheduler core.  Design and lock order: nicogpu_fence.h.
 * Kernel-only.
 */
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include "nicogpu_fence.h"

/* 64-bit seqnos: dma_fence_ops.use_64bit_seqno up to 6.16; a per-fence flag set by dma_fence_init64() since 6.17
 * (checked on 7.0.11; 6.12.90 has the ops field) */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 17, 0)
#define NICOGPU_FENCE_INIT64 1
#endif

#define NICOGPU_DEP_MAGIC	0x58444550u	/* "XDEP": a use after free shows up as a bad magic */
#define NICOGPU_FENCE_MAGIC_FREE 0xdeadf3ceu

/* A foreign fence a job waits for: the dma_fence callback signals the core's external fence.  Two references: one for
 * the armed callback, one for whoever owns the entry (the registering thread, later the teardown). */
struct nicogpu_dep {
	u32 magic;
	struct dma_fence_cb cb;
	struct list_head node;		/* fs->deps; fs->lock */
	struct dma_fence *fence;	/* reference, dropped with the last dep reference */
	struct xsc_fence *ext;		/* the core's external fence; the creation reference lives here until the signal */
	struct nicogpu_fences *fs;
	atomic_t refs;
	bool done;			/* the callback is finished with `fs` for good (teardown waits for it) */
};

static const struct dma_fence_ops nicogpu_fence_ops;

static struct nicogpu_fence *to_nicogpu_fence(struct dma_fence *f)
{
	return container_of(f, struct nicogpu_fence, base);
}

/* ---- timelines ------------------------------------------------------------- */

struct nicogpu_fctx *nicogpu_fctx_create(void)
{
	struct nicogpu_fctx *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL_ACCOUNT);

	if (!ctx)
		return NULL;
	kref_init(&ctx->ref);
	spin_lock_init(&ctx->lock);
	ctx->context = dma_fence_context_alloc(1);
	return ctx;
}

void nicogpu_fctx_get(struct nicogpu_fctx *ctx)
{
	kref_get(&ctx->ref);
}

static void fctx_release(struct kref *k)
{
	kfree(container_of(k, struct nicogpu_fctx, ref));
}

void nicogpu_fctx_put(struct nicogpu_fctx *ctx)
{
	kref_put(&ctx->ref, fctx_release);
}

/* ---- the fence ops ------------------------------------------------------------ */

static const char *nicogpu_fence_driver_name(struct dma_fence *f)
{
	return "nicogpu";
}

static const char *nicogpu_fence_timeline_name(struct dma_fence *f)
{
	return to_nicogpu_fence(f)->ctx->name;
}

/* the core signals through nicogpu_fences_harvest(), nothing to switch on */
static bool nicogpu_fence_enable_signaling(struct dma_fence *f)
{
	return true;
}

static void nicogpu_fence_release(struct dma_fence *f)
{
	struct nicogpu_fence *w = to_nicogpu_fence(f);

	WARN_ON(w->xf || w->fs);	/* the harvester drops both when it signals; releasing earlier is a leak */
	nicogpu_fctx_put(w->ctx);
	dma_fence_free(f);		/* kfree_rcu: readers under rcu_read_lock() may still look at the fence */
}

static const struct dma_fence_ops nicogpu_fence_ops = {
#ifndef NICOGPU_FENCE_INIT64
	.use_64bit_seqno = true,
#endif
	.get_driver_name = nicogpu_fence_driver_name,
	.get_timeline_name = nicogpu_fence_timeline_name,
	.enable_signaling = nicogpu_fence_enable_signaling,
	.release = nicogpu_fence_release,
};

int nicogpu_fence_errno(int core_err, bool lost)
{
	if (!core_err)
		return 0;
	if (lost)
		return -ENODEV;
	if (core_err > 0 || core_err < -4095)
		return -EIO;
	return core_err;
}

/* ---- the set ----------------------------------------------------------------- */

void nicogpu_fences_init(struct nicogpu_fences *fs, struct xsc_core *core, void (*kick)(void *ctx), void *kick_ctx)
{
	memset(fs, 0, sizeof(*fs));
	fs->core = core;
	spin_lock_init(&fs->lock);
	INIT_LIST_HEAD(&fs->pending);
	INIT_LIST_HEAD(&fs->deps);
	fs->kick = kick;
	fs->kick_ctx = kick_ctx;
}

void nicogpu_fences_stats(struct nicogpu_fences *fs, struct nicogpu_fence_stats *out)
{
	unsigned long fl;
	struct list_head *p;

	spin_lock_irqsave(&fs->lock, fl);
	*out = fs->st;
	out->pending = 0;
	out->deps = 0;
	for (p = fs->pending.next; p != &fs->pending; p = p->next)
		out->pending++;
	for (p = fs->deps.next; p != &fs->deps; p = p->next)
		out->deps++;
	spin_unlock_irqrestore(&fs->lock, fl);
}

void nicogpu_fences_fini(struct nicogpu_fences *fs)
{
	WARN_ON(!list_empty(&fs->pending) || !list_empty(&fs->deps));
}

/* ---- exporting a job fence ------------------------------------------------------- */

struct nicogpu_fence *nicogpu_fence_prepare(struct nicogpu_fences *fs, struct xsc_fence *xf)
{
	struct nicogpu_fence *w = kzalloc(sizeof(*w), GFP_KERNEL_ACCOUNT);

	if (!w)
		return NULL;
	xsc_fence_get(fs->core, xf);
	w->xf = xf;
	INIT_LIST_HEAD(&w->node);
	return w;
}

/* never armed: no dma_fence exists yet */
void nicogpu_fence_discard(struct nicogpu_fences *fs, struct nicogpu_fence *w)
{
	xsc_fence_put(fs->core, w->xf);
	kfree(w);
}

void nicogpu_fence_arm(struct nicogpu_fences *fs, struct nicogpu_fence *w, struct nicogpu_fctx *ctx)
{
	struct nicogpu_fence *p;
	unsigned long fl;
	u64 seq = w->xf->seq;		/* assigned by xsc_submit() on this thread */

	nicogpu_fctx_get(ctx);
	w->ctx = ctx;
#ifdef NICOGPU_FENCE_INIT64
	dma_fence_init64(&w->base, &nicogpu_fence_ops, &ctx->lock, ctx->context, ++ctx->seqno);
#else
	dma_fence_init(&w->base, &nicogpu_fence_ops, &ctx->lock, ctx->context, ++ctx->seqno);
#endif
	dma_fence_get(&w->base);	/* the pending list's reference */
	spin_lock_irqsave(&fs->lock, fl);
	w->fs = fs;
	/* ordered by core sequence number: walk back from the tail (threads arm in nearly that order) */
	for (p = list_last_entry(&fs->pending, struct nicogpu_fence, node); &p->node != &fs->pending;
	     p = list_last_entry(&p->node, struct nicogpu_fence, node)) {
		if (p->xf->seq < seq)
			break;
	}
	list_add(&w->node, &p->node);	/* after p (or at the head: p is the list head's container) */
	fs->st.created++;
	spin_unlock_irqrestore(&fs->lock, fl);
	/* the job may have retired between xsc_submit() and here */
	nicogpu_fences_harvest(fs, false);
}

void nicogpu_fences_harvest(struct nicogpu_fences *fs, bool gone)
{
	struct xsc_core *c = fs->core;
	struct nicogpu_fence *w, *n;
	LIST_HEAD(done);
	unsigned long fl;

	spin_lock_irqsave(&fs->lock, fl);
	if (gone)
		fs->gone = true;
	if (fs->harvesting) {		/* the active harvester will look again before it leaves */
		fs->again = true;
		spin_unlock_irqrestore(&fs->lock, fl);
		return;
	}
	fs->harvesting = true;
	do {
		bool lost = fs->gone || xsc_is_lost(c);

		fs->again = false;
		list_for_each_entry_safe(w, n, &fs->pending, node) {
			if (!xsc_fence_poll(c, w->xf, &w->err))
				break;	/* retirement is in order: nothing behind it is done either */
			w->fs = NULL;	/* from now on nicogpu_fence_dep() treats it as signalled */
			w->err = nicogpu_fence_errno(w->err, lost);
			list_move_tail(&w->node, &done);
			fs->st.signalled++;
			if (w->err)
				fs->st.errors++;
		}
		spin_unlock_irqrestore(&fs->lock, fl);

		list_for_each_entry_safe(w, n, &done, node) {
			struct xsc_fence *xf = w->xf;

			list_del_init(&w->node);
			w->xf = NULL;
			if (w->err)
				dma_fence_set_error(&w->base, w->err);
			dma_fence_signal(&w->base);
			xsc_fence_put(c, xf);
			dma_fence_put(&w->base);	/* the pending list's reference */
		}
		spin_lock_irqsave(&fs->lock, fl);
	} while (fs->again);
	fs->harvesting = false;
	spin_unlock_irqrestore(&fs->lock, fl);
}

/* ---- waiting for a dma_fence -------------------------------------------------------- */

static void dep_put(struct nicogpu_dep *dep)
{
	if (atomic_dec_and_test(&dep->refs)) {
		dma_fence_put(dep->fence);
		dep->magic = NICOGPU_FENCE_MAGIC_FREE;
		kfree(dep);
	}
}

/* may run in IRQ context, under the foreign fence's lock */
static void nicogpu_dep_cb(struct dma_fence *f, struct dma_fence_cb *cb)
{
	struct nicogpu_dep *dep = container_of(cb, struct nicogpu_dep, cb);
	struct nicogpu_fences *fs = dep->fs;
	struct xsc_core *c = fs->core;
	unsigned long fl;

	WARN_ON(dep->magic != NICOGPU_DEP_MAGIC);
	xsc_fence_signal(c, dep->ext, f->error);
	xsc_fence_put(c, dep->ext);
	if (fs->kick)
		fs->kick(fs->kick_ctx);
	/* unlink last: nicogpu_fences_stop_deps() waits for every entry it still finds on the list, so the callback must
	 * be finished with the core and `fs` (all but this unlock) once the entry is gone */
	spin_lock_irqsave(&fs->lock, fl);
	list_del_init(&dep->node);
	spin_unlock_irqrestore(&fs->lock, fl);
	WRITE_ONCE(dep->done, true);	/* touches only the dep, which the waiter keeps alive */
	dep_put(dep);			/* the callback's reference */
}

struct xsc_syncobj *nicogpu_fence_dep(struct nicogpu_fences *fs, struct dma_fence *f)
{
	struct xsc_core *c = fs->core;
	struct xsc_syncobj *so;
	struct nicogpu_dep *dep;
	struct xsc_fence *ext;
	unsigned long fl;
	int rc;

	if (dma_fence_is_signaled(f)) {
		so = xsc_syncobj_alloc(c, true);
		if (so) {
			spin_lock_irqsave(&fs->lock, fl);
			fs->st.dep_signalled++;
			spin_unlock_irqrestore(&fs->lock, fl);
		}
		return so ? so : ERR_PTR(-ENOMEM);
	}

	if (f->ops == &nicogpu_fence_ops) {
		struct nicogpu_fence *w = to_nicogpu_fence(f);
		struct nicogpu_fences *owner;
		struct xsc_fence *xf = NULL;

		spin_lock_irqsave(&fs->lock, fl);
		owner = w->fs;
		if (owner == fs) {
			xf = w->xf;
			xsc_fence_get(c, xf);
			fs->st.dep_ours++;
		} else if (!owner) {
			fs->st.dep_signalled++;	/* harvested: the core fence is signalled, the dma_fence is about to be */
		}
		spin_unlock_irqrestore(&fs->lock, fl);
		if (xf) {
			so = xsc_syncobj_alloc(c, false);
			if (!so) {
				xsc_fence_put(c, xf);
				return ERR_PTR(-ENOMEM);
			}
			xsc_syncobj_attach(c, so, xf);	/* the syncobj takes its own reference */
			xsc_fence_put(c, xf);
			return so;
		}
		if (!owner) {
			so = xsc_syncobj_alloc(c, true);
			return so ? so : ERR_PTR(-ENOMEM);
		}
		/* a fence of another nicogpu device: foreign */
	}

	ext = xsc_fence_alloc_external(c);
	dep = kzalloc(sizeof(*dep), GFP_KERNEL_ACCOUNT);
	so = xsc_syncobj_alloc(c, false);
	if (!ext || !dep || !so) {
		if (so)
			xsc_syncobj_put(c, so);
		kfree(dep);
		if (ext)
			xsc_fence_put(c, ext);
		return ERR_PTR(-ENOMEM);
	}
	xsc_syncobj_attach(c, so, ext);		/* the syncobj's own reference; the creation reference stays with dep */
	dep->magic = NICOGPU_DEP_MAGIC;
	dep->fs = fs;
	dep->ext = ext;
	dep->fence = dma_fence_get(f);
	atomic_set(&dep->refs, 2);		/* callback + owner */
	INIT_LIST_HEAD(&dep->node);
	spin_lock_irqsave(&fs->lock, fl);
	if (fs->shut) {
		spin_unlock_irqrestore(&fs->lock, fl);
		xsc_syncobj_put(c, so);
		xsc_fence_put(c, ext);
		dma_fence_put(dep->fence);
		kfree(dep);
		return ERR_PTR(-ENODEV);
	}
	list_add_tail(&dep->node, &fs->deps);
	fs->st.dep_foreign++;
	spin_unlock_irqrestore(&fs->lock, fl);

	/* never with fs->lock held: the callback takes it under the foreign fence's lock */
	rc = dma_fence_add_callback(f, &dep->cb, nicogpu_dep_cb);
	if (rc) {				/* -ENOENT: signalled in the meantime, the callback will never run */
		spin_lock_irqsave(&fs->lock, fl);
		list_del_init(&dep->node);
		spin_unlock_irqrestore(&fs->lock, fl);
		xsc_fence_signal(c, ext, f->error);
		xsc_fence_put(c, ext);
		dep_put(dep);			/* the callback's reference */
	}
	dep_put(dep);				/* the owner's reference: the callback (if armed) holds its own */
	return so;
}

void nicogpu_fences_stop_deps(struct nicogpu_fences *fs)
{
	struct xsc_core *c = fs->core;
	unsigned long fl;

	spin_lock_irqsave(&fs->lock, fl);
	fs->shut = true;
	while (!list_empty(&fs->deps)) {
		struct nicogpu_dep *dep = list_first_entry(&fs->deps, struct nicogpu_dep, node);

		atomic_inc(&dep->refs);		/* keeps it alive while the lock is dropped */
		spin_unlock_irqrestore(&fs->lock, fl);
		if (dma_fence_remove_callback(dep->fence, &dep->cb)) {
			/* the callback will never run: do what it would have done, minus the signal */
			spin_lock_irqsave(&fs->lock, fl);
			list_del_init(&dep->node);
			spin_unlock_irqrestore(&fs->lock, fl);
			xsc_fence_put(c, dep->ext);
			dep_put(dep);		/* the callback's reference */
		} else {
			/* the fence signalled: the callback is running (or finished): it unlinks itself */
			while (!READ_ONCE(dep->done))
				cond_resched();
		}
		dep_put(dep);			/* our temporary reference */
		spin_lock_irqsave(&fs->lock, fl);
	}
	spin_unlock_irqrestore(&fs->lock, fl);
}
