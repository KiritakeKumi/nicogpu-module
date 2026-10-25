// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * nicogpu_sched_core.c -- pure-C core of the nicogpu scheduler / sync layer.
 * See nicogpu_sched_core.h.  No kernel or libc dependency beyond memset/memcpy.
 */
#include "nicogpu_sched_core.h"

#define XSC_LOCK(c)		((c)->ops->lock((c)->ctx))
#define XSC_UNLOCK(c)	((c)->ops->unlock((c)->ctx))

/* ------------------------------------------------------------------------ */
/* C3 front-region layout                                                   */

void xsc_c3_init(struct xsc_c3 *l, uint64_t limit)
{
	l->limit = limit;
	l->next = 0;
}

int xsc_c3_alloc(struct xsc_c3 *l, uint64_t size, uint64_t align, uint64_t *off)
{
	uint64_t start;

	if (!size || !align || (align & (align - 1)))
		return -XSC_EINVAL;
	start = (l->next + align - 1) & ~(align - 1);
	if (start < l->next || size > l->limit || start > l->limit - size)
		return -XSC_ENOSPC;
	l->next = start + size;
	*off = start;
	return 0;
}

/* ------------------------------------------------------------------------ */
/* setup                                                                    */

int xsc_init(struct xsc_core *c, const struct xsc_ops *ops, void *ctx, const struct xsc_cfg *cfg)
{
	if (!c || !ops || !cfg || !ops->zalloc || !ops->free || !ops->lock || !ops->unlock || !ops->now_ms ||
	    !ops->read_fence || !ops->notify || !ops->ring_write || !ops->doorbell)
		return -XSC_EINVAL;
	if (cfg->ring_log2 < XSC_RING_LOG2_MIN || cfg->ring_log2 > XSC_RING_LOG2_MAX || (cfg->ring_pa & 0xfff) ||
	    !cfg->fence_pa || (cfg->fence_pa & 7) || (cfg->rptr_wb_pa & 3) || !cfg->timeout_ms ||
	    (cfg->max_job_ms && cfg->max_job_ms < cfg->timeout_ms))
		return -XSC_EINVAL;
	if ((cfg->flags & ~(XSC_CFG_ESCALATE | XSC_CFG_L2_KEEP_CTX)) ||
	    ((cfg->flags & XSC_CFG_ESCALATE) && (!ops->reg_rd || !ops->reg_wr || !ops->msleep)))
		return -XSC_EINVAL;
	memset(c, 0, sizeof(*c));
	c->ops = ops;
	c->ctx = ctx;
	c->cfg = *cfg;
	c->ring_dw = 1u << cfg->ring_log2;
	c->stub.refs = 1;		/* the core's own reference: never freed */
	c->stub.signaled = true;
	c->stub.foreign = true;
	return 0;
}

uint32_t xsc_fini(struct xsc_core *c)
{
	uint32_t n;

	XSC_LOCK(c);
	n = c->n_pending + c->n_inflight;
	XSC_UNLOCK(c);
	return n;
}

void xsc_shutdown(struct xsc_core *c)
{
	XSC_LOCK(c);
	c->dead = true;
	XSC_UNLOCK(c);
}

static void emit(const struct xsc_core *c, enum xsc_event ev, uint64_t a, uint64_t b)
{
	if (c->ops->event)
		c->ops->event(c->ctx, ev, a, b);
}

/* ------------------------------------------------------------------------ */
/* serialised entry points                                                  */

static int xsc_enter(struct xsc_core *c)
{
	int rc = 0;

	XSC_LOCK(c);
	if (c->busy) {
		c->st.reentry++;
		rc = -XSC_EBUSY;
	} else {
		c->busy = true;
	}
	XSC_UNLOCK(c);
	return rc;
}

static void xsc_leave(struct xsc_core *c)
{
	XSC_LOCK(c);
	c->busy = false;
	XSC_UNLOCK(c);
}

/* ------------------------------------------------------------------------ */
/* fences                                                                   */

static void fence_get(struct xsc_fence *f)
{
	f->refs++;
}

/* lock held */
static void fence_put_locked(struct xsc_core *c, struct xsc_fence *f)
{
	if (--f->refs == 0 && f != &c->stub)
		c->ops->free(c->ctx, f);
}

struct xsc_fence *xsc_fence_alloc_external(struct xsc_core *c)
{
	struct xsc_fence *f = c->ops->zalloc(c->ctx, sizeof(*f));

	if (f) {
		f->refs = 1;
		f->foreign = true;
	}
	return f;
}

void xsc_fence_put(struct xsc_core *c, struct xsc_fence *f)
{
	XSC_LOCK(c);
	fence_put_locked(c, f);
	XSC_UNLOCK(c);
}

void xsc_fence_get(struct xsc_core *c, struct xsc_fence *f)
{
	XSC_LOCK(c);
	fence_get(f);
	XSC_UNLOCK(c);
}

bool xsc_fence_poll(struct xsc_core *c, struct xsc_fence *f, int *err)
{
	bool s;

	XSC_LOCK(c);
	s = f->signaled;
	if (s && err)
		*err = f->err;
	XSC_UNLOCK(c);
	return s;
}

void xsc_fence_signal(struct xsc_core *c, struct xsc_fence *f, int err)
{
	XSC_LOCK(c);
	if (!f->signaled) {
		f->signaled = true;
		f->err = err;
		c->ops->notify(c->ctx);
	}
	XSC_UNLOCK(c);
}

/* ------------------------------------------------------------------------ */
/* syncobjs                                                                 */

struct xsc_syncobj *xsc_syncobj_alloc(struct xsc_core *c, bool signaled)
{
	struct xsc_syncobj *so = c->ops->zalloc(c->ctx, sizeof(*so));

	if (!so)
		return NULL;
	so->refs = 1;
	if (signaled) {
		XSC_LOCK(c);		/* the shared stub's refcount is under the core lock like every fence's */
		so->fence = &c->stub;
		fence_get(so->fence);
		XSC_UNLOCK(c);
	}
	return so;
}

void xsc_syncobj_get(struct xsc_core *c, struct xsc_syncobj *so)
{
	XSC_LOCK(c);
	so->refs++;
	XSC_UNLOCK(c);
}

static void syncobj_set_locked(struct xsc_core *c, struct xsc_syncobj *so, struct xsc_fence *f)
{
	struct xsc_fence *old = so->fence;

	if (f)
		fence_get(f);
	so->fence = f;
	if (old)
		fence_put_locked(c, old);
}

void xsc_syncobj_put(struct xsc_core *c, struct xsc_syncobj *so)
{
	XSC_LOCK(c);
	if (--so->refs == 0) {
		syncobj_set_locked(c, so, NULL);
		c->ops->free(c->ctx, so);
	}
	XSC_UNLOCK(c);
}

int xsc_syncobj_reset(struct xsc_core *c, struct xsc_syncobj *const *so, uint32_t n)
{
	uint32_t i;

	for (i = 0; i < n; i++)
		if (!so[i])
			return -XSC_ENOENT;
	XSC_LOCK(c);
	for (i = 0; i < n; i++)
		syncobj_set_locked(c, so[i], NULL);
	XSC_UNLOCK(c);
	return 0;
}

int xsc_syncobj_signal(struct xsc_core *c, struct xsc_syncobj *const *so, uint32_t n)
{
	uint32_t i;

	for (i = 0; i < n; i++)
		if (!so[i])
			return -XSC_ENOENT;
	XSC_LOCK(c);
	for (i = 0; i < n; i++)
		syncobj_set_locked(c, so[i], &c->stub);
	c->ops->notify(c->ctx);		/* a WAIT_FOR_SUBMIT waiter may be sleeping */
	XSC_UNLOCK(c);
	return 0;
}

void xsc_syncobj_attach(struct xsc_core *c, struct xsc_syncobj *so, struct xsc_fence *f)
{
	XSC_LOCK(c);
	syncobj_set_locked(c, so, f);
	c->ops->notify(c->ctx);
	XSC_UNLOCK(c);
}

int xsc_syncobj_wait_poll(struct xsc_core *c, struct xsc_syncobj *const *so, uint32_t n, uint32_t flags,
			  uint32_t *first)
{
	bool all = true;
	int first_idx = -1, rc = 0;
	uint32_t i;

	if (!n || (flags & ~XSC_WAIT_FLAGS_ALL))
		return -XSC_EINVAL;
	XSC_LOCK(c);
	for (i = 0; i < n; i++) {
		const struct xsc_fence *f;
		bool sig;

		if (!so[i]) {
			rc = -XSC_ENOENT;
			break;
		}
		f = so[i]->fence;
		if (!f && !(flags & XSC_WAIT_FOR_SUBMIT)) {
			rc = -XSC_EINVAL;	/* no WAIT_FOR_SUBMIT in HAL v0 */
			break;
		}
		sig = f && f->signaled;
		all = all && sig;
		if (sig && first_idx < 0)
			first_idx = (int)i;
	}
	if (!rc) {
		bool ok = (flags & XSC_WAIT_ALL) ? all : first_idx >= 0;

		if (ok && first)
			*first = (uint32_t)(first_idx < 0 ? 0 : first_idx);
		rc = ok ? 0 : -XSC_ETIME;
	}
	XSC_UNLOCK(c);
	return rc;
}

int xsc_syncobj_wait_avail_poll(struct xsc_core *c, struct xsc_syncobj *const *so, uint32_t n, uint32_t flags,
				uint32_t *first)
{
	bool all = true;
	int first_idx = -1, rc = 0;
	uint32_t i;

	if (!n || (flags & ~(XSC_WAIT_ALL | XSC_WAIT_FOR_SUBMIT)))
		return -XSC_EINVAL;
	XSC_LOCK(c);
	for (i = 0; i < n; i++) {
		bool avail;

		if (!so[i]) {
			rc = -XSC_ENOENT;
			break;
		}
		avail = so[i]->fence != NULL;
		all = all && avail;
		if (avail && first_idx < 0)
			first_idx = (int)i;
	}
	if (!rc) {
		bool ok = (flags & XSC_WAIT_ALL) ? all : first_idx >= 0;

		if (ok && first)
			*first = (uint32_t)(first_idx < 0 ? 0 : first_idx);
		rc = ok ? 0 : -XSC_ETIME;
	}
	XSC_UNLOCK(c);
	return rc;
}

bool xsc_syncobj_signaled(struct xsc_core *c, struct xsc_syncobj *so)
{
	bool s;

	XSC_LOCK(c);
	s = so->fence && so->fence->signaled;
	XSC_UNLOCK(c);
	return s;
}

int xsc_seq_status(struct xsc_core *c, uint64_t seq, int *err)
{
	int rc;
	uint32_t i, n;

	if (err)
		*err = 0;
	if (!seq)
		return 0;
	XSC_LOCK(c);
	if (seq > c->last_seq) {
		rc = -XSC_EINVAL;
	} else if (seq > c->done_seq) {
		rc = -XSC_ETIME;
	} else {
		rc = 0;
		n = c->hist_n < XSC_ERR_HIST ? c->hist_n : XSC_ERR_HIST;
		for (i = 0; i < n && err; i++) {
			uint32_t k = (c->hist_n - 1 - i) % XSC_ERR_HIST;

			if (c->hist[k].seq == seq) {
				*err = c->hist[k].err;
				break;
			}
		}
	}
	XSC_UNLOCK(c);
	return rc;
}

/* ------------------------------------------------------------------------ */
/* queues                                                                   */

struct xsc_queue *xsc_queue_alloc(struct xsc_core *c, uint32_t vmid)
{
	struct xsc_queue *q;

	if (vmid == 0 || vmid >= XSC_NUM_VMIDS)
		return NULL;
	q = c->ops->zalloc(c->ctx, sizeof(*q));
	if (q) {
		q->c = c;
		q->vmid = vmid;
		XSC_LOCK(c);
		q->epoch = c->epoch;
		XSC_UNLOCK(c);
	}
	return q;
}

int xsc_queue_free(struct xsc_core *c, struct xsc_queue *q)
{
	int rc = 0;

	XSC_LOCK(c);
	if (q->njobs)
		rc = -XSC_EBUSY;
	XSC_UNLOCK(c);
	if (!rc)
		c->ops->free(c->ctx, q);
	return rc;
}

bool xsc_queue_idle(struct xsc_core *c, struct xsc_queue *q)
{
	bool idle;

	XSC_LOCK(c);
	idle = q->njobs == 0;
	XSC_UNLOCK(c);
	return idle;
}

/* GUILTY, including the queues a GPU-zone reset condemned (their epoch is older
 * than the core's): latch the reason once.  Lock held. */
static bool qguilty_locked(struct xsc_core *c, struct xsc_queue *q)
{
	if (!q->guilty && q->epoch != c->epoch) {
		q->guilty = true;
		q->guilty_seq = 0;
		memset(&q->fault, 0, sizeof(q->fault));
		q->fault.cause = XSC_CAUSE_DEV_LOST;
		q->fault.resets = (uint32_t)c->st.resets;
		c->st.ctx_lost++;
	}
	return q->guilty;
}

bool xsc_queue_guilty(struct xsc_core *c, struct xsc_queue *q)
{
	bool g;

	XSC_LOCK(c);
	g = qguilty_locked(c, q);
	XSC_UNLOCK(c);
	return g;
}

bool xsc_queue_fault_info(struct xsc_core *c, struct xsc_queue *q, struct xsc_queue_fault *out)
{
	bool g;

	XSC_LOCK(c);
	g = qguilty_locked(c, q);
	if (out) {
		if (g)
			*out = q->fault;
		else
			memset(out, 0, sizeof(*out));
	}
	XSC_UNLOCK(c);
	return g;
}

void xsc_queue_flush(struct xsc_core *c, struct xsc_queue *q)
{
	struct xsc_job *j;

	XSC_LOCK(c);
	for (j = c->pend_head; j; j = j->next)
		if (j->q == q && !j->pre_err)
			j->pre_err = -XSC_ECANCELED;
	XSC_UNLOCK(c);
}

/* ------------------------------------------------------------------------ */
/* jobs                                                                     */

struct xsc_job *xsc_job_alloc(struct xsc_core *c, uint32_t nib, uint32_t nwaits)
{
	struct xsc_job *j;
	struct xsc_fence *f;
	size_t sz;

	if (nib > XSC_MAX_IBS || nwaits > XSC_MAX_SYNCS)
		return NULL;
	sz = sizeof(*j) + (size_t)nib * sizeof(struct xsc_ib) + (size_t)nwaits * sizeof(struct xsc_fence *);
	j = c->ops->zalloc(c->ctx, sz);
	if (!j)
		return NULL;
	f = c->ops->zalloc(c->ctx, sizeof(*f));
	if (!f) {
		c->ops->free(c->ctx, j);
		return NULL;
	}
	f->refs = 1;			/* the job's reference */
	j->fence = f;
	j->nib = nib;
	j->ib = (struct xsc_ib *)(j + 1);
	j->deps_cap = nwaits;
	j->deps = (struct xsc_fence **)(j->ib + nib);
	return j;
}

/* drops the job's fence and dependency references, frees the job; lock held */
/* j became the in-flight head at `now`: its timeout period (and the progress baseline) starts here */
static void head_start_locked(struct xsc_core *c, struct xsc_job *j, uint64_t now)
{
	j->t_head = now;
	j->t_first = now;
	j->prog = c->ops->progress ? c->ops->progress(c->ctx) : 0;
}

static void job_release_locked(struct xsc_core *c, struct xsc_job *j)
{
	uint32_t i;

	for (i = 0; i < j->ndeps; i++)
		fence_put_locked(c, j->deps[i]);
	fence_put_locked(c, j->fence);
	c->ops->free(c->ctx, j);
}

void xsc_job_free(struct xsc_core *c, struct xsc_job *j)
{
	XSC_LOCK(c);
	job_release_locked(c, j);
	XSC_UNLOCK(c);
}

/* a dependency never blocks if it is signalled or is a job already in the
 * (in-order) ring, i.e. ahead of any job pushed from now on.  A job still
 * pending is not: it may itself be blocked and be passed by others. */
static bool dep_ready(const struct xsc_fence *d)
{
	return d->signaled || (!d->foreign && d->rseq);
}

int xsc_submit(struct xsc_core *c, struct xsc_queue *q, struct xsc_job *j, const struct xsc_sync_ref *waits,
	       uint32_t nwaits, const struct xsc_sync_ref *signals, uint32_t nsignals, uint64_t *seq)
{
	uint32_t i, k;
	int rc = 0;

	XSC_LOCK(c);
	if (c->dead || c->lost) {
		rc = -XSC_ENODEV;
		goto out;
	}
	if (qguilty_locked(c, q)) {
		rc = -XSC_ECANCELED;
		goto out;
	}
	for (i = 0; i < j->nib; i++) {
		const struct xsc_ib *ib = &j->ib[i];

		if (ib->flags || !ib->size_dw || ib->size_dw > XSC_IB_MAX_DW || (ib->va & 3) ||
		    (ib->va >> XSC_VA_BITS)) {
			rc = -XSC_EINVAL;
			goto out;
		}
	}
	if (nwaits > j->deps_cap || nwaits > XSC_MAX_SYNCS || nsignals > XSC_MAX_SYNCS) {
		rc = -XSC_EINVAL;
		goto out;
	}
	for (i = 0; i < nwaits; i++) {
		if (!waits[i].so) {
			rc = -XSC_ENOENT;
			goto out;
		}
		if (waits[i].flags || waits[i].point || !waits[i].so->fence) {
			rc = -XSC_EINVAL;
			goto out;
		}
	}
	for (i = 0; i < nsignals; i++) {
		if (!signals[i].so) {
			rc = -XSC_ENOENT;
			goto out;
		}
		if (signals[i].flags || signals[i].point) {
			rc = -XSC_EINVAL;
			goto out;
		}
		for (k = 0; k < i; k++) {
			if (signals[k].so == signals[i].so) {
				rc = -XSC_EINVAL;
				goto out;
			}
		}
	}
	if (xsc_job_dw(j->nib) >= c->ring_dw) {		/* sim_backend: pk.size() > ring_mask */
		rc = -XSC_EINVAL;
		goto out;
	}
	if (c->io_err) {
		rc = -XSC_EIO;
		goto out;
	}

	/* arm */
	j->fence->seq = ++c->last_seq;
	j->q = q;
	j->vmid = q->vmid;
	j->dw = xsc_job_dw(j->nib);
	for (i = 0; i < nwaits; i++) {
		struct xsc_fence *d = waits[i].so->fence;

		if (!dep_ready(d)) {
			fence_get(d);
			j->deps[j->ndeps++] = d;
		}
	}
	for (i = 0; i < nsignals; i++)
		syncobj_set_locked(c, signals[i].so, j->fence);
	q->njobs++;
	j->state = XSC_JS_PENDING;
	j->next = NULL;
	if (c->pend_tail)
		c->pend_tail->next = j;
	else
		c->pend_head = j;
	c->pend_tail = j;
	c->n_pending++;
	c->st.armed++;
	if (seq)
		*seq = j->fence->seq;
out:
	XSC_UNLOCK(c);
	return rc;
}

/* ------------------------------------------------------------------------ */
/* ring                                                                     */

uint32_t xsc_ring_free(const struct xsc_core *c)
{
	return c->ring_dw - (c->wptr - c->rptr);
}

uint32_t xsc_encode_job(const struct xsc_core *c, const struct xsc_job *j, uint32_t *out)
{
	uint32_t i, n = 0;
	uint64_t seq = j->fence->rseq;

	for (i = 0; i < j->nib; i++) {
		out[n++] = NICOGPU_CMD_HEADER(NICOGPU_CMD_OP_INDIRECT, 4);
		out[n++] = (uint32_t)j->ib[i].va;
		out[n++] = (uint32_t)(j->ib[i].va >> 32);
		out[n++] = j->ib[i].size_dw;
		out[n++] = j->vmid & NICOGPU_CMD_INDIRECT_IB_CTRL_VMID_MASK;
	}
	out[n++] = NICOGPU_CMD_HEADER(NICOGPU_CMD_OP_FENCE, 5);
	out[n++] = (uint32_t)c->cfg.fence_pa;
	out[n++] = (uint32_t)(c->cfg.fence_pa >> 32);
	out[n++] = (uint32_t)seq;
	out[n++] = (uint32_t)(seq >> 32);
	out[n++] = NICOGPU_CMD_FENCE_FLAGS_BIT64_MASK | NICOGPU_CMD_FENCE_FLAGS_IRQ_MASK;
	return n;
}

/* write n dwords at free-running position `start`, splitting at the ring end;
 * *wrapped is set when a split happened.  Lock released. */
static int ring_put(struct xsc_core *c, uint32_t start, const uint32_t *dw, uint32_t n, bool *wrapped)
{
	uint32_t idx = start & (c->ring_dw - 1);
	uint32_t first = c->ring_dw - idx;
	int rc;

	if (first > n)
		first = n;
	rc = c->ops->ring_write(c->ctx, idx, dw, first);
	*wrapped = false;
	if (!rc && first < n) {
		*wrapped = true;
		rc = c->ops->ring_write(c->ctx, 0, dw + first, n - first);
	}
	return rc;
}

static void fl_append_locked(struct xsc_core *c, struct xsc_job *j)
{
	j->next = NULL;
	if (c->fl_tail)
		c->fl_tail->next = j;
	else
		c->fl_head = j;
	c->fl_tail = j;
	c->n_inflight++;
	j->fence->rseq = ++c->flight_seq;
}

/* unlink j (whose predecessor in the pending list is prev, NULL at the head) */
static void pend_unlink_locked(struct xsc_core *c, struct xsc_job *prev, struct xsc_job *j)
{
	if (prev)
		prev->next = j->next;
	else
		c->pend_head = j->next;
	if (c->pend_tail == j)
		c->pend_tail = prev;
	j->next = NULL;
	c->n_pending--;
}

static void pend_pop_locked(struct xsc_core *c)
{
	pend_unlink_locked(c, NULL, c->pend_head);
}

static void hist_add_locked(struct xsc_core *c, uint64_t seq, int err)
{
	uint32_t k = c->hist_n % XSC_ERR_HIST;

	c->hist[k].seq = seq;
	c->hist[k].err = err;
	c->hist_n++;
}

/* Retire jobs from the head of the in-flight list, in order, while they are
 * complete (fence value reached) or failed.  Lock held.  */
static unsigned int retire_locked(struct xsc_core *c)
{
	unsigned int n = 0;
	uint64_t now = 0;
	bool have_now = false;

	for (;;) {
		struct xsc_job *j = c->fl_head;
		int err;

		if (!j)
			break;
		if (j->state == XSC_JS_FAILED)
			err = j->err;
		else if (j->state == XSC_JS_RING && j->fence->rseq <= c->observed)
			err = 0;
		else
			break;
		c->fl_head = j->next;
		if (!c->fl_head)
			c->fl_tail = NULL;
		c->n_inflight--;
		j->fence->err = err;
		j->fence->signaled = true;
		if (err)
			hist_add_locked(c, j->fence->seq, err);
		c->done_rseq = j->fence->rseq;
		if (!c->ooo)
			c->done_seq = j->fence->seq;
		j->q->njobs--;
		c->st.retired++;
		n++;
		job_release_locked(c, j);
		if (c->fl_head && c->fl_head->state == XSC_JS_RING) {
			if (!have_now) {
				now = c->ops->now_ms(c->ctx);
				have_now = true;
			}
			head_start_locked(c, c->fl_head, now);
		}
	}
	if (n && c->ooo) {
		/* jobs retired out of seq order: done_seq is just below the oldest job still alive */
		uint64_t m = c->last_seq + 1;
		const struct xsc_job *k;

		if (c->pend_head)
			m = c->pend_head->fence->seq;	/* the pending list is in seq order */
		for (k = c->fl_head; k; k = k->next)
			if (k->fence->seq < m)
				m = k->fence->seq;
		c->done_seq = m - 1;
		if (!c->pend_head && !c->fl_head)
			c->ooo = false;
	}
	if (n)
		c->ops->notify(c->ctx);
	return n;
}

/* take a fence value read from the fence page; lock held */
static unsigned int observe_locked(struct xsc_core *c, uint64_t v)
{
	if (v > c->flight_seq) {
		/* more than anything handed to the CP: corrupt or stale page */
		c->st.bad_fence++;
		emit(c, XSC_EV_BAD_FENCE, v, c->flight_seq);
		return 0;
	}
	if (v < c->observed)
		c->st.stale_fence++;	/* the page went backwards: keep what we have */
	else
		c->observed = v;
	/* also when nothing is new: a job may have been waiting in PUSHING while
	 * an interrupt raised `observed` past it */
	return retire_locked(c);
}

unsigned int xsc_fence_irq(struct xsc_core *c)
{
	unsigned int n;

	XSC_LOCK(c);
	n = observe_locked(c, c->ops->read_fence(c->ctx));
	/* S5: a fence MSI that finds nothing new right after the watchdog retired jobs by polling is the MSI of those
	 * jobs arriving late (the poll read the fence page before the ISR got to it), not a lost one */
	if (!n && c->poll_suspect) {
		c->st.late_irq += c->poll_suspect;
		c->poll_suspect = 0;
	}
	XSC_UNLOCK(c);
	return n;
}

static void rptr_update_locked(struct xsc_core *c, uint32_t rptr)
{
	/* the CP can only have consumed what was committed by a doorbell */
	if ((int)(rptr - c->rptr) < 0 || (int)(c->wptr_db - rptr) < 0) {
		c->st.bad_rptr++;
		emit(c, XSC_EV_BAD_RPTR, rptr, c->wptr_db);
		return;
	}
	c->rptr = rptr;
}

void xsc_rptr_update(struct xsc_core *c, uint32_t rptr)
{
	XSC_LOCK(c);
	rptr_update_locked(c, rptr);
	XSC_UNLOCK(c);
}

static void fail_noguilt_locked(struct xsc_core *c, struct xsc_job *j, int err)
{
	j->state = XSC_JS_FAILED;
	j->err = err;
	c->st.failed++;
	emit(c, XSC_EV_JOB_FAILED, j->fence->seq, (uint32_t)-err);
}

/* devcoredump trigger bookkeeping; lock held */
static void note_incident_locked(struct xsc_core *c, uint32_t cause, const struct xsc_job *j, uint32_t code)
{
	uint32_t i, n = c->inc.n + 1;

	memset(&c->inc, 0, sizeof(c->inc));
	c->inc.n = n;
	c->inc.cause = cause;
	c->inc.cp_error = code;
	c->inc.t_ms = c->ops->now_ms(c->ctx);
	if (j) {
		c->inc.seq = j->fence->seq;
		c->inc.vmid = j->vmid;
		c->inc.nib = j->nib;
		c->inc.ring_start = j->ring_start;
		c->inc.ring_end = j->ring_end;
		for (i = 0; i < j->nib && i < XSC_INC_IBS; i++) {
			c->inc.ib_va[i] = j->ib[i].va;
			c->inc.ib_dw[i] = j->ib[i].size_dw;
		}
		c->inc.nib_kept = i;
	}
	c->st.incidents++;
	emit(c, XSC_EV_INCIDENT, cause, c->inc.seq);
}

/* the newest not yet attached MMU fault record of `vmid` that is recent enough; lock held */
static struct xsc_mmu_fault *match_fault_locked(struct xsc_core *c, uint32_t vmid, bool *other)
{
	uint64_t now = c->ops->now_ms(c->ctx);
	uint32_t n, lowest;

	*other = false;
	lowest = c->fault_n > XSC_FAULT_LOG ? c->fault_n - XSC_FAULT_LOG + 1 : 1;
	for (n = c->fault_n; n >= lowest && n; n--) {
		struct xsc_mmu_fault *f = &c->faults[(n - 1) % XSC_FAULT_LOG];

		if (!f->valid || f->used || now - f->t_ms > XSC_FAULT_MATCH_MS)
			continue;
		if (f->vmid == vmid)
			return f;
		*other = true;
	}
	return NULL;
}

/* A job of a queue failed for a reason that is the queue's fault: GUILTY, with
 * the structured reason (the first one wins), the MMU fault record of its VMID
 * if there is one, and a devcoredump trigger.  Lock held. */
static void fail_guilty_locked(struct xsc_core *c, struct xsc_job *j, int err, uint32_t cause, uint32_t code)
{
	struct xsc_queue *q = j->q;

	if (!qguilty_locked(c, q)) {
		struct xsc_mmu_fault *f;
		bool other;

		q->guilty = true;
		q->guilty_seq = j->fence->seq;
		memset(&q->fault, 0, sizeof(q->fault));
		q->fault.cause = cause;
		q->fault.cp_error = code;
		q->fault.seq = j->fence->seq;
		q->fault.err = err;
		q->fault.resets = (uint32_t)c->st.resets;
		f = match_fault_locked(c, j->vmid, &other);
		if (f) {
			f->used = true;
			q->fault.has_mmu = true;
			q->fault.mmu = *f;
			c->st.fault_attached++;
			if (cause == XSC_CAUSE_TIMEOUT ||
			    (cause == XSC_CAUSE_CP_ERROR && (code == NICOGPU_CMD_CP_ERROR_MEM_FAULT ||
							      code == NICOGPU_CMD_CP_ERROR_SHADER_FAULT)))
				q->fault.cause = XSC_CAUSE_MMU_FAULT;
		} else if (other && (code == NICOGPU_CMD_CP_ERROR_MEM_FAULT || code == NICOGPU_CMD_CP_ERROR_SHADER_FAULT)) {
			c->st.fault_vmid_mismatch++;	/* a fault of another VMID: blame by RPTR only */
		}
		note_incident_locked(c, q->fault.cause, j, code);
	}
	fail_noguilt_locked(c, j, err);
}

int xsc_run(struct xsc_core *c)
{
	int pushed = 0, rc = 0;
	bool stalled = false;
	struct xsc_job *prev = NULL;

	if (xsc_enter(c))
		return -XSC_EBUSY;
	XSC_LOCK(c);
	if (!++c->scan_gen)
		c->scan_gen = 1;
	/* Walk the pending list in seq order.  A job whose dependencies are not ready blocks its own queue
	 * (later jobs of that queue are skipped, they must stay behind it) but not the other queues. */
	for (;;) {
		struct xsc_job *j = prev ? prev->next : c->pend_head;
		uint32_t i;
		bool wrapped = false;

		if (!j)
			break;
		if (j->pre_err || qguilty_locked(c, j->q) || c->io_err) {
			if (j->q->scan_gen == c->scan_gen) {	/* behind a blocked job of its queue */
				prev = j;
				continue;
			}
			/* never reaches the CP: retire with an error, in order */
			pend_unlink_locked(c, prev, j);
			fail_noguilt_locked(c, j, j->pre_err ? j->pre_err :
					    c->io_err && !j->q->guilty ? -XSC_EIO : -XSC_ECANCELED);
			fl_append_locked(c, j);
			retire_locked(c);
			continue;
		}
		if (j->q->scan_gen == c->scan_gen) {
			prev = j;
			continue;
		}
		for (i = 0; i < j->ndeps; i++)
			if (!dep_ready(j->deps[i]))
				break;
		if (i < j->ndeps) {
			if (!stalled)
				c->st.dep_stalls++;
			stalled = true;
			j->q->scan_gen = c->scan_gen;
			prev = j;
			continue;
		}
		if (xsc_ring_free(c) < j->dw && c->ops->read_rptr) {
			uint32_t v = 0;
			int r;

			XSC_UNLOCK(c);
			r = c->ops->read_rptr(c->ctx, &v);
			XSC_LOCK(c);
			if (!r)
				rptr_update_locked(c, v);
		}
		if (xsc_ring_free(c) < j->dw) {
			c->st.ring_full++;
			break;
		}

		/* reserve; from here the job is visible to retire/timeouts */
		pend_unlink_locked(c, prev, j);
		if (prev) {			/* passed a blocked older job */
			c->ooo = true;
			c->st.reordered++;
		}
		j->ring_start = c->wptr;
		c->wptr += j->dw;
		j->ring_end = c->wptr;
		j->state = XSC_JS_PUSHING;
		fl_append_locked(c, j);
		XSC_UNLOCK(c);

		xsc_encode_job(c, j, c->pkt);
		rc = ring_put(c, j->ring_start, c->pkt, j->dw, &wrapped);
		if (!rc)
			rc = c->ops->doorbell(c->ctx, j->ring_end);

		XSC_LOCK(c);
		if (wrapped)
			c->st.ring_wraps++;
		if (rc) {
			/* nothing was committed to the CP (the doorbell did not move):
			 * give the space back and retire the job with -EIO */
			c->wptr = j->ring_start;
			j->ring_start = j->ring_end = 0;
			c->io_err = true;
			c->st.io_errors++;
			emit(c, XSC_EV_IO_ERROR, (uint32_t)-rc, j->fence->seq);
			fail_noguilt_locked(c, j, -XSC_EIO);
			retire_locked(c);
			rc = -XSC_EIO;
			break;
		}
		c->wptr_db = j->ring_end;
		c->st.doorbells++;
		c->st.pushed++;
		pushed++;
		j->state = XSC_JS_RING;
		if (c->fl_head == j)
			head_start_locked(c, j, c->ops->now_ms(c->ctx));
		/* the CP may have finished it before we got the lock back */
		observe_locked(c, c->ops->read_fence(c->ctx));
	}
	c->busy = false;
	XSC_UNLOCK(c);
	return rc ? rc : pushed;
}

/* ------------------------------------------------------------------------ */
/* timeouts, CP errors, replay                                              */

/* Move every in-flight job back to the front of the pending list (a CP reset
 * or reload discarded the ring).  Jobs of a guilty queue and jobs that had
 * already failed keep an error and are retired by xsc_run without being
 * replayed (never replay a guilty queue).  Lock held;
 * returns the number of jobs that will really be replayed. */
static uint32_t requeue_inflight_locked(struct xsc_core *c)
{
	struct xsc_job *j;
	uint32_t n = 0;

	retire_locked(c);
	for (j = c->fl_head; j; j = j->next) {
		if (j->state == XSC_JS_FAILED)
			j->pre_err = j->err;
		else if (qguilty_locked(c, j->q))
			j->pre_err = -XSC_ECANCELED;
		else
			n++;
		j->state = XSC_JS_PENDING;
		j->ring_start = j->ring_end = 0;
		j->fence->rseq = 0;
	}
	if (c->fl_head) {
		if (!c->pend_head || (!c->ooo && c->fl_tail->fence->seq < c->pend_head->fence->seq)) {
			c->fl_tail->next = c->pend_head;	/* in order: the whole list goes in front */
			c->pend_head = c->fl_head;
			if (!c->pend_tail)
				c->pend_tail = c->fl_tail;
		} else {
			/* jobs were pushed out of seq order: merge back into the seq-ordered pending list */
			struct xsc_job *nx;

			for (j = c->fl_head; j; j = nx) {
				struct xsc_job *p = NULL, *k = c->pend_head;

				nx = j->next;
				while (k && k->fence->seq < j->fence->seq) {
					p = k;
					k = k->next;
				}
				j->next = k;
				if (p)
					p->next = j;
				else
					c->pend_head = j;
				if (!k)
					c->pend_tail = j;
			}
		}
		c->n_pending += c->n_inflight;
	}
	c->fl_head = c->fl_tail = NULL;
	c->n_inflight = 0;
	c->flight_seq = c->done_rseq;
	return n;
}

static int bringup_inner(struct xsc_core *c);
static void abort_all_locked(struct xsc_core *c, int err);

/* ---- reset ladder ----------------------------------------------------------- */

/* A job that is not done must be able to run again after a reset: bring the
 * bookkeeping to "CP restarted, RPTR == WPTR" and replay.  Lock released. */
static void finish_cp_reset(struct xsc_core *c)
{
	uint32_t n;

	XSC_LOCK(c);
	c->st.resets++;
	c->io_err = false;
	c->rptr = c->wptr;		/* CP_CTRL.RESET: RPTR := WPTR */
	c->wptr_db = c->wptr;
	/* completions the CP finished before the reset took effect */
	(void)observe_locked(c, c->ops->read_fence(c->ctx));
	n = requeue_inflight_locked(c);
	c->st.replayed += n;
	XSC_UNLOCK(c);
	emit(c, XSC_EV_REPLAY, n, 0);
}

#define XSC_L1_FWDEAD	1	/* level_cp(): the firmware is dead, CP_CTRL.RESET would do nothing */
#define XSC_L1_ENGBUSY	2	/* CP_CTRL.RESET went through but the compute engine still runs */
#define XSC_L1_ABTMO	3	/* CP_CTRL.RESET went through but its dispatch abort hit the fabric-drain watchdog */

/* PROVISIONAL: the firmware latches CP_FW_STATUS.ABORT_TIMEOUT when the dispatch abort of a CP_CTRL.RESET
 * ended on the hardware watchdog (WDIST_STATUS[4]: SP fabric requests still out).  CORE_RESET does not reset the
 * fabric, so the firmware reload cannot help: the caller goes straight to the GPU-zone reset.  The bit is sticky until
 * CORE_RESET; old firmware / RMs read 0.  `late` = the CP did not idle in time either (the ladder escalates anyway;
 * this only counts and reports).  Worker, lock released. */
static bool abort_timed_out(struct xsc_core *c, bool late)
{
	uint32_t st = c->ops->reg_rd(c->ctx, NICOGPU_REG_CP_FW_STATUS);

	if (!(st & NICOGPU_REG_CP_FW_STATUS_ABORT_TIMEOUT_MASK))
		return false;
	XSC_LOCK(c);
	c->st.abort_timeout++;
	XSC_UNLOCK(c);
	emit(c, XSC_EV_ABORT_TIMEOUT, st, late ? 1 : 0);
	return true;
}

/* a reset level only counts when the compute engine really stopped.  0 = idle (or no probe),
 * -ETIMEDOUT = still busy after XSC_ENGINE_IDLE_MS.  Worker, lock released. */
static int engines_stopped(struct xsc_core *c)
{
	const struct xsc_ops *o = c->ops;
	uint32_t t;

	if (!o->engine_busy)
		return 0;
	for (t = 0;; t++) {
		if (!o->engine_busy(c->ctx))
			return 0;
		if (t >= XSC_ENGINE_IDLE_MS)
			return -XSC_ETIMEDOUT;
		o->msleep(c->ctx, 1);
	}
}

/* Is the firmware alive (no PANIC, CP_HEARTBEAT
 * moving within XSC_HB_STALL_MS), then CP_CTRL = ENABLE|RESET and wait for
 * CP_STATUS.IDLE for XSC_RESET_IDLE_MS.  0, XSC_L1_FWDEAD, -ETIMEDOUT (not idle),
 * -EIO (the cp_reset hook failed).  Worker, lock released. */
static int level_cp(struct xsc_core *c)
{
	const struct xsc_ops *o = c->ops;
	uint32_t hb0, t;
	bool alive = false;

	if (o->cp_reset)		/* the glue's own implementation wins */
		return o->cp_reset(c->ctx) ? -XSC_EIO : 0;
	if (o->reg_rd(c->ctx, NICOGPU_REG_CP_FW_STATUS) & NICOGPU_REG_CP_FW_STATUS_PANIC_MASK)
		return XSC_L1_FWDEAD;
	hb0 = o->reg_rd(c->ctx, NICOGPU_REG_CP_HEARTBEAT);
	for (t = 0; t < XSC_HB_STALL_MS; t++) {
		o->msleep(c->ctx, 1);
		if (o->reg_rd(c->ctx, NICOGPU_REG_CP_HEARTBEAT) != hb0) {
			alive = true;
			break;
		}
	}
	if (!alive)
		return XSC_L1_FWDEAD;
	o->reg_wr(c->ctx, NICOGPU_REG_CP_CTRL, NICOGPU_REG_CP_CTRL_ENABLE_MASK | NICOGPU_REG_CP_CTRL_RESET_MASK);
	for (t = 0;; t++) {
		if (o->reg_rd(c->ctx, NICOGPU_REG_CP_STATUS) & NICOGPU_REG_CP_STATUS_IDLE_MASK)
			return 0;
		if (t >= XSC_RESET_IDLE_MS)
			return -XSC_ETIMEDOUT;
		o->msleep(c->ctx, 1);
	}
}

/* Level 2: reset the whole GPU zone (ops->gpu_reset), let
 * the glue restore what only the host knows (ops->rm_restore), run the CP bring-up
 * handshake again.  The RM's L2 dirty data is gone, so unless
 * XSC_CFG_L2_KEEP_CTX the other queues' contexts are declared lost (epoch bump:
 * their jobs retire with -ECANCELED, the queues turn GUILTY with cause
 * DEV_LOST).  Worker, lock released. */
static int level_gpu(struct xsc_core *c)
{
	const struct xsc_ops *o = c->ops;
	int rc;

	if (!o->gpu_reset)
		return -XSC_ENODEV;
	rc = o->gpu_reset(c->ctx);
	if (!rc && o->rm_restore)
		rc = o->rm_restore(c->ctx);
	if (rc)
		return rc;
	if (!(c->cfg.flags & XSC_CFG_L2_KEEP_CTX)) {
		XSC_LOCK(c);
		c->epoch++;
		XSC_UNLOCK(c);
	}
	return bringup_inner(c);
}

/* every level failed: the device is lost */
static int declare_lost(struct xsc_core *c, int rc)
{
	XSC_LOCK(c);
	c->lost = true;
	c->st.lost++;
	abort_all_locked(c, -XSC_EIO);
	note_incident_locked(c, XSC_CAUSE_DEV_LOST, NULL, 0);
	XSC_UNLOCK(c);
	emit(c, XSC_EV_DEVICE_LOST, (uint32_t)-rc, 0);
	return -XSC_EIO;
}

/* Recovery after a failure was attributed (or none: `full`).  busy held, lock
 * released.  Without XSC_CFG_ESCALATE this is the basic contract: one
 * ops->cp_reset, no escalation.  With it the reset ladder
 * starting at `from`:
 *   CP    CP_CTRL.RESET (firmware alive) -> replay the other jobs
 *         (CP_FW_STATUS.ABORT_TIMEOUT afterwards -> straight to GPU, no L1 retry, no FW)
 *   FW    CORE_RESET + CP bring-up (firmware dead, or the CP did not idle) -> replay
 *   GPU   GPU-zone reset + restore + CP bring-up
 * and when everything fails: abort every job with -EIO and mark the device lost. */
static int recover_from(struct xsc_core *c, enum xsc_reset_level from, bool full)
{
	int rc = 0;
	int ok = full ? XSC_RECOVER_FULL : XSC_RECOVER_RESET;

	if (!(c->cfg.flags & XSC_CFG_ESCALATE)) {
		if (!c->ops->cp_reset)
			return XSC_RECOVER_FAILED;
		rc = c->ops->cp_reset(c->ctx);
		if (rc)
			return -XSC_EIO;
		finish_cp_reset(c);
		return ok;
	}

	if (from <= XSC_RST_CP) {
		rc = level_cp(c);
		if (!rc && abort_timed_out(c, false))
			rc = XSC_L1_ABTMO;	/* SP fabric requests still out: neither L1 nor CORE_RESET clears them */
		else if (rc == -XSC_ETIMEDOUT)
			(void)abort_timed_out(c, true);	/* goes to the GPU zone anyway; only counted if already visible */
		if (!rc && engines_stopped(c))
			rc = XSC_L1_ENGBUSY;	/* the dispatch survived CP_CTRL.RESET: the SP needs CORE_RESET */
		if (!rc) {
			XSC_LOCK(c);
			c->st.reset_cp++;
			XSC_UNLOCK(c);
			finish_cp_reset(c);
			emit(c, XSC_EV_RESET, XSC_RST_CP, 0);
			return ok;
		}
		emit(c, XSC_EV_RESET, XSC_RST_CP, (uint32_t)(rc > 0 ? XSC_EIO : -rc));
		/* FWDEAD / ENGBUSY: the firmware reload (CORE_RESET) can cure it; ABTMO and a CP that does not idle:
		 * the GPU-zone reset */
		from = (rc == XSC_L1_FWDEAD || rc == XSC_L1_ENGBUSY) ? XSC_RST_FW : XSC_RST_GPU;
	}
	if (from <= XSC_RST_FW) {
		rc = bringup_inner(c);
		if (!rc && engines_stopped(c))
			rc = -XSC_ETIMEDOUT;	/* CORE_RESET did not stop the SP: only the GPU-zone reset is left */
		if (!rc) {
			XSC_LOCK(c);
			c->st.reset_fw++;
			c->st.resets++;
			XSC_UNLOCK(c);
			emit(c, XSC_EV_RESET, XSC_RST_FW, 0);
			return ok;
		}
		emit(c, XSC_EV_RESET, XSC_RST_FW, (uint32_t)-rc);
	}
	rc = level_gpu(c);
	if (!rc) {
		XSC_LOCK(c);
		c->st.reset_gpu++;
		c->st.resets++;
		XSC_UNLOCK(c);
		emit(c, XSC_EV_RESET, XSC_RST_GPU, 0);
		return ok;
	}
	emit(c, XSC_EV_RESET, XSC_RST_GPU, (uint32_t)-rc);
	return declare_lost(c, rc);
}

/* ---- timeouts, CP errors, MMU faults ----------------------------------------- */

int xsc_check_timeout(struct xsc_core *c)
{
	struct xsc_job *j;
	unsigned int n;
	uint64_t now;
	int rc;

	rc = xsc_enter(c);
	if (rc)
		return rc;
	XSC_LOCK(c);
	n = observe_locked(c, c->ops->read_fence(c->ctx));
	c->poll_suspect = n;		/* an unmatched suspect of the previous poll is a real loss: stays in lost_irq only */
	if (n) {
		c->st.lost_irq += n;
		emit(c, XSC_EV_LOST_IRQ, n, 0);
	}
	now = c->ops->now_ms(c->ctx);
	j = c->fl_head;
	if (j && j->state == XSC_JS_RING && now - j->t_head >= c->cfg.timeout_ms && c->ops->progress &&
	    c->cfg.max_job_ms && now - j->t_first < c->cfg.max_job_ms) {
		uint64_t p = c->ops->progress(c->ctx);

		if (p != j->prog) {		/* still working: another period (long kernels, large copies) */
			j->prog = p;
			j->t_head = now;
			c->st.timeout_ext++;
		}
	}
	if (!j || j->state != XSC_JS_RING || now - j->t_head < c->cfg.timeout_ms) {
		XSC_UNLOCK(c);
		rc = n ? XSC_RECOVER_LOST_IRQ : XSC_RECOVER_NONE;
		goto out;
	}
	fail_guilty_locked(c, j, -XSC_EIO, XSC_CAUSE_TIMEOUT, 0);
	retire_locked(c);
	XSC_UNLOCK(c);
	rc = recover_from(c, XSC_RST_CP, false);
out:
	xsc_leave(c);
	return rc;
}

static struct xsc_job *find_job_by_rptr(struct xsc_core *c, uint32_t rptr)
{
	struct xsc_job *j;

	for (j = c->fl_head; j; j = j->next)
		if (j->state == XSC_JS_RING && (uint32_t)(rptr - j->ring_start) < (uint32_t)(j->ring_end - j->ring_start))
			return j;
	return NULL;
}

int xsc_seq_by_rptr(struct xsc_core *c, uint32_t rptr, uint64_t *seq)
{
	struct xsc_job *j;
	int rc = -XSC_ENOENT;

	XSC_LOCK(c);
	j = find_job_by_rptr(c, rptr);
	if (j) {
		*seq = j->fence->seq;
		rc = 0;
	}
	XSC_UNLOCK(c);
	return rc;
}

int xsc_cp_error(struct xsc_core *c, uint32_t error_rptr, uint32_t code)
{
	struct xsc_job *j;
	int rc;

	rc = xsc_enter(c);
	if (rc)
		return rc;
	XSC_LOCK(c);
	(void)observe_locked(c, c->ops->read_fence(c->ctx));	/* earlier jobs may have finished */
	j = find_job_by_rptr(c, error_rptr);
	if (!j) {
		if (!(c->cfg.flags & XSC_CFG_ESCALATE)) {
			XSC_UNLOCK(c);
			rc = -XSC_ENOENT;
			goto out;
		}
		/* belongs to no job (RING_CONFIG: a kernel bug; or a stale interrupt): reset
		 * everything, blame nobody */
		c->st.unattributed++;
		note_incident_locked(c, XSC_CAUSE_CP_ERROR, NULL, code);
		XSC_UNLOCK(c);
		rc = recover_from(c, code == NICOGPU_CMD_CP_ERROR_RING_CONFIG ? XSC_RST_FW : XSC_RST_CP, true);
		goto out;
	}
	emit(c, XSC_EV_CP_ERROR, j->fence->seq, code);
	fail_guilty_locked(c, j, -XSC_EIO, XSC_CAUSE_CP_ERROR, code);
	retire_locked(c);
	XSC_UNLOCK(c);
	rc = recover_from(c, XSC_RST_CP, false);
out:
	xsc_leave(c);
	return rc;
}

void xsc_mmu_fault(struct xsc_core *c, uint32_t status, uint64_t va, uint64_t pte)
{
	struct xsc_mmu_fault *f;

	if (!(status & NICOGPU_REG_MMU_FAULT_STATUS_VALID_MASK))
		return;
	XSC_LOCK(c);
	f = &c->faults[c->fault_n % XSC_FAULT_LOG];
	memset(f, 0, sizeof(*f));
	f->valid = true;
	f->n = ++c->fault_n;
	f->multi = (status & NICOGPU_REG_MMU_FAULT_STATUS_MULTI_MASK) != 0;
	f->status = status;
	f->code = (status & NICOGPU_REG_MMU_FAULT_STATUS_CODE_MASK) >> NICOGPU_REG_MMU_FAULT_STATUS_CODE_SHIFT;
	f->access = (status & NICOGPU_REG_MMU_FAULT_STATUS_ACCESS_MASK) >> NICOGPU_REG_MMU_FAULT_STATUS_ACCESS_SHIFT;
	f->level = (status & NICOGPU_REG_MMU_FAULT_STATUS_LEVEL_MASK) >> NICOGPU_REG_MMU_FAULT_STATUS_LEVEL_SHIFT;
	f->vmid = (status & NICOGPU_REG_MMU_FAULT_STATUS_VMID_MASK) >> NICOGPU_REG_MMU_FAULT_STATUS_VMID_SHIFT;
	f->va = va;
	f->pte = pte;
	f->t_ms = c->ops->now_ms(c->ctx);
	c->st.mmu_faults++;
	emit(c, XSC_EV_MMU_FAULT, va, status);
	XSC_UNLOCK(c);
}

int xsc_cp_fw_event(struct xsc_core *c)
{
	const struct xsc_ops *o = c->ops;
	struct xsc_job *j;
	uint32_t st, rptr;
	int rc;

	if (!(c->cfg.flags & XSC_CFG_ESCALATE))
		return XSC_RECOVER_NONE;
	rc = xsc_enter(c);
	if (rc)
		return rc;
	st = o->reg_rd(c->ctx, NICOGPU_REG_CP_FW_STATUS);
	if (!(st & (NICOGPU_REG_CP_FW_STATUS_PANIC_MASK | NICOGPU_REG_CP_FW_STATUS_BOOT_ERR_MASK))) {
		rc = XSC_RECOVER_NONE;		/* READY and friends */
		goto out;
	}
	rptr = o->reg_rd(c->ctx, NICOGPU_REG_CP_RING_RPTR);
	XSC_LOCK(c);
	(void)observe_locked(c, c->ops->read_fence(c->ctx));
	j = find_job_by_rptr(c, rptr);
	if (j) {
		fail_guilty_locked(c, j, -XSC_EIO, XSC_CAUSE_FW_PANIC, st >> 16);
		retire_locked(c);
	} else {
		c->st.unattributed++;
		note_incident_locked(c, XSC_CAUSE_FW_PANIC, NULL, st >> 16);
	}
	XSC_UNLOCK(c);
	rc = recover_from(c, XSC_RST_FW, !j);
out:
	xsc_leave(c);
	return rc;
}

int xsc_force_reset(struct xsc_core *c, enum xsc_reset_level level)
{
	int rc;

	if (!(c->cfg.flags & XSC_CFG_ESCALATE) || level < XSC_RST_CP || level > XSC_RST_GPU)
		return -XSC_EINVAL;
	rc = xsc_enter(c);
	if (rc)
		return rc;
	XSC_LOCK(c);
	if (c->lost && level < XSC_RST_FW)
		level = XSC_RST_FW;		/* a lost device needs the CP bring-up handshake, not just CP_CTRL.RESET */
	XSC_UNLOCK(c);
	rc = recover_from(c, level, true);
	xsc_leave(c);
	return rc;
}

int xsc_recover_io(struct xsc_core *c)
{
	int rc;
	bool need;

	rc = xsc_enter(c);
	if (rc)
		return rc;
	XSC_LOCK(c);
	need = c->io_err && !c->dead;
	XSC_UNLOCK(c);
	if (need) {
		rc = c->ops->rm_restore ? c->ops->rm_restore(c->ctx) : 0;
		if (!rc)
			rc = bringup_inner(c);	/* clears io_err, replays */
	}
	xsc_leave(c);
	return rc;
}

bool xsc_is_lost(struct xsc_core *c)
{
	bool l;

	XSC_LOCK(c);
	l = c->lost;
	XSC_UNLOCK(c);
	return l;
}

void xsc_last_incident(struct xsc_core *c, struct xsc_incident *out)
{
	XSC_LOCK(c);
	*out = c->inc;
	XSC_UNLOCK(c);
}

static void snap_job(const struct xsc_core *c, const struct xsc_job *j, uint64_t now, struct xsc_snap_job *o)
{
	uint32_t i;

	memset(o, 0, sizeof(*o));
	o->seq = j->fence->seq;
	o->state = (uint32_t)j->state;
	o->vmid = j->vmid;
	o->nib = j->nib;
	o->dw = j->dw;
	o->ring_start = j->ring_start;
	o->ring_end = j->ring_end;
	o->err = j->err;
	o->pre_err = j->pre_err;
	o->age_ms = (c->fl_head == j && j->state == XSC_JS_RING) ? now - j->t_first : 0;
	o->queue_guilty = j->q->guilty || j->q->epoch != c->epoch;
	for (i = 0; i < j->nib && i < XSC_SNAP_IBS; i++) {
		o->ib_va[i] = j->ib[i].va;
		o->ib_dw[i] = j->ib[i].size_dw;
	}
	o->nib_kept = i;
}

void xsc_snapshot(struct xsc_core *c, struct xsc_snap *out)
{
	const struct xsc_job *j;
	uint32_t i, n, first;

	memset(out, 0, sizeof(*out));
	XSC_LOCK(c);
	out->now_ms = c->ops->now_ms(c->ctx);
	out->cfg = c->cfg;
	out->ring_dw = c->ring_dw;
	out->wptr = c->wptr;
	out->wptr_db = c->wptr_db;
	out->rptr = c->rptr;
	out->last_seq = c->last_seq;
	out->flight_seq = c->flight_seq;
	out->observed = c->observed;
	out->done_seq = c->done_seq;
	out->fence_mem = c->ops->read_fence(c->ctx);
	out->n_pending = c->n_pending;
	out->n_inflight = c->n_inflight;
	out->busy = c->busy;
	out->dead = c->dead;
	out->lost = c->lost;
	out->io_err = c->io_err;
	out->epoch = c->epoch;
	out->st = c->st;
	out->inc = c->inc;
	n = c->fault_n < XSC_FAULT_LOG ? c->fault_n : XSC_FAULT_LOG;
	first = c->fault_n - n;		/* serials first+1 .. fault_n */
	for (i = 0; i < n; i++)
		out->faults[i] = c->faults[(first + i) % XSC_FAULT_LOG];
	out->nfaults = n;
	n = c->hist_n < XSC_SNAP_HIST ? c->hist_n : XSC_SNAP_HIST;
	for (i = 0; i < n; i++) {
		uint32_t k = (c->hist_n - 1 - i) % XSC_ERR_HIST;

		out->hist[i].seq = c->hist[k].seq;
		out->hist[i].err = c->hist[k].err;
	}
	out->nhist = n;
	out->jobs_total = c->n_inflight + c->n_pending;
	for (j = c->fl_head; j && out->njobs < XSC_SNAP_JOBS; j = j->next)
		snap_job(c, j, out->now_ms, &out->jobs[out->njobs++]);
	for (j = c->pend_head; j && out->njobs < XSC_SNAP_JOBS; j = j->next)
		snap_job(c, j, out->now_ms, &out->jobs[out->njobs++]);
	XSC_UNLOCK(c);
}

/* Fail every job with `err`; busy held, lock held. */
static void abort_all_locked(struct xsc_core *c, int err)
{
	struct xsc_job *j;

	while ((j = c->pend_head) != NULL) {
		pend_pop_locked(c);
		fail_noguilt_locked(c, j, err);
		fl_append_locked(c, j);
	}
	for (j = c->fl_head; j; j = j->next)
		if (j->state != XSC_JS_FAILED)
			fail_noguilt_locked(c, j, err);
	retire_locked(c);
}

void xsc_abort_all(struct xsc_core *c, int err)
{
	if (xsc_enter(c))
		return;
	XSC_LOCK(c);
	abort_all_locked(c, err);
	c->busy = false;
	XSC_UNLOCK(c);
}

uint32_t xsc_jobs_alive(struct xsc_core *c)
{
	return xsc_fini(c);
}

uint32_t xsc_inflight(struct xsc_core *c)
{
	uint32_t n;

	XSC_LOCK(c);
	n = c->n_inflight;
	XSC_UNLOCK(c);
	return n;
}

/* ------------------------------------------------------------------------ */
/* CP bring-up                                                              */

/* busy held, lock released */
static int bringup_inner(struct xsc_core *c)
{
	const struct xsc_ops *o = c->ops;
	uint64_t fw_pa = 0;
	uint32_t fw_sz = 0, v, ver, waited, rptr0, wptr0;
	int rc;

	if (!o->reg_rd || !o->reg_wr || !o->msleep)
		return -XSC_EINVAL;

	/* 1. Ibex in reset, ring disabled; interrupts on */
	o->reg_wr(c->ctx, NICOGPU_REG_CP_CTRL, NICOGPU_REG_CP_CTRL_CORE_RESET_MASK);
	o->reg_wr(c->ctx, NICOGPU_REG_IRQ_ENABLE, XSC_IRQ_FENCE | XSC_IRQ_CP_ERROR | XSC_IRQ_MMU_FAULT | XSC_IRQ_CP_FW);
	/* 2. firmware image */
	if (o->fw_load) {
		rc = o->fw_load(c->ctx, &fw_pa, &fw_sz);
		if (rc < 0)
			return rc;
		if (rc == 0) {
			o->reg_wr(c->ctx, NICOGPU_REG_CP_FW_BASE_LO, (uint32_t)fw_pa);
			o->reg_wr(c->ctx, NICOGPU_REG_CP_FW_BASE_HI, (uint32_t)(fw_pa >> 32));
			o->reg_wr(c->ctx, NICOGPU_REG_CP_FW_SIZE, fw_sz);
		}
	}
	/* 3. release the core: the boot ROM loads the image */
	o->reg_wr(c->ctx, NICOGPU_REG_CP_CTRL, 0);
	/* 4. wait for READY, 100 ms */
	for (waited = 0;; waited++) {
		v = o->reg_rd(c->ctx, NICOGPU_REG_CP_FW_STATUS);
		if ((v & NICOGPU_REG_CP_FW_STATUS_PANIC_MASK) || (v & NICOGPU_REG_CP_FW_STATUS_BOOT_ERR_MASK))
			return -XSC_EIO;
		if (v & NICOGPU_REG_CP_FW_STATUS_READY_MASK)
			break;
		if (waited >= XSC_FW_READY_TIMEOUT_MS)
			return -XSC_ETIMEDOUT;
		o->msleep(c->ctx, 1);
	}
	/* 5. command-stream version */
	ver = o->reg_rd(c->ctx, NICOGPU_REG_CP_CMD_VERSION);
	if ((ver >> 16) != XSC_CMD_MAJOR || (ver & 0xffff) < XSC_CMD_MINOR_MIN)
		return -XSC_EPROTO;
	/* 6. ring (the CP is idle: ENABLE=0) and enable */
	o->reg_wr(c->ctx, NICOGPU_REG_CP_RING_BASE_LO, (uint32_t)c->cfg.ring_pa);
	o->reg_wr(c->ctx, NICOGPU_REG_CP_RING_BASE_HI, (uint32_t)(c->cfg.ring_pa >> 32));
	o->reg_wr(c->ctx, NICOGPU_REG_CP_RING_SIZE_LOG2, c->cfg.ring_log2);
	if (c->cfg.rptr_wb_pa) {
		o->reg_wr(c->ctx, NICOGPU_REG_CP_RPTR_WB_HI, (uint32_t)(c->cfg.rptr_wb_pa >> 32));
		o->reg_wr(c->ctx, NICOGPU_REG_CP_RPTR_WB_LO,
			  ((uint32_t)c->cfg.rptr_wb_pa & NICOGPU_REG_CP_RPTR_WB_LO_ADDR_MASK) |
			  NICOGPU_REG_CP_RPTR_WB_LO_ENABLE_MASK);
	} else {
		o->reg_wr(c->ctx, NICOGPU_REG_CP_RPTR_WB_LO, 0);
	}
	/* The CP starts with an empty ring, but the position is not specified: power-up is
	 * 0, while the firmware core keeps WPTR across CORE_RESET (RPTR := WPTR).  Adopt what the
	 * registers say; the doorbell may only advance from here.  If RPTR != WPTR the ring is not empty:
	 * CP_CTRL.RESET (RPTR := WPTR; ENABLE is still 0) empties it, anything else is refused. */
	rptr0 = o->reg_rd(c->ctx, NICOGPU_REG_CP_RING_RPTR);
	wptr0 = o->reg_rd(c->ctx, NICOGPU_REG_CP_RING_WPTR);
	if (rptr0 != wptr0) {
		o->reg_wr(c->ctx, NICOGPU_REG_CP_CTRL, NICOGPU_REG_CP_CTRL_RESET_MASK);
		rptr0 = o->reg_rd(c->ctx, NICOGPU_REG_CP_RING_RPTR);
		wptr0 = o->reg_rd(c->ctx, NICOGPU_REG_CP_RING_WPTR);
		if (rptr0 != wptr0)
			return -XSC_EIO;
	}
	o->reg_wr(c->ctx, NICOGPU_REG_CP_CTRL, NICOGPU_REG_CP_CTRL_ENABLE_MASK);

	/* a freshly (re)loaded CP starts with an empty ring: whatever was in flight is replayed into it */
	XSC_LOCK(c);
	c->wptr = c->wptr_db = c->rptr = wptr0;
	c->io_err = false;
	c->lost = false;
	c->st.replayed += requeue_inflight_locked(c);
	XSC_UNLOCK(c);
	return 0;
}

int xsc_cp_bringup(struct xsc_core *c)
{
	int rc;

	if (!c->ops->reg_rd || !c->ops->reg_wr || !c->ops->msleep)
		return -XSC_EINVAL;
	rc = xsc_enter(c);
	if (rc)
		return rc;
	rc = bringup_inner(c);
	xsc_leave(c);
	return rc;
}
