// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * nicogpu_vm_core.c -- pure-C core of the nicogpu memory manager.
 * See nicogpu_vm_core.h.  No kernel or libc dependency beyond memset/memmove.
 */
#include "nicogpu_vm_core.h"

#define PTE_V		NICOGPU_PTE_V_MASK
#define PTE_LEAF	NICOGPU_PTE_LEAF_MASK
#define PTE_R		NICOGPU_PTE_R_MASK
#define PTE_W		NICOGPU_PTE_W_MASK
#define PTE_X		NICOGPU_PTE_X_MASK
#define PTE_MTYPE	NICOGPU_PTE_MTYPE_MASK
#define PTE_F64K	NICOGPU_PTE_F64K_MASK
#define PTE_SPARSE	NICOGPU_PTE_SPARSE_MASK
#define PTE_PA		NICOGPU_PTE_PA_MASK
#define PTE_RSVD	NICOGPU_PTE_RSVD_MASK
#define PTE_ATTR	(PTE_R | PTE_W | PTE_X | PTE_MTYPE)
#define SPARSE_NULL	NICOGPU_PTE_SPARSE_NULL
#define ROOT_LEVEL	(XVC_LEVELS - 1)

/* ------------------------------------------------------------------------ */
/* BO arguments: same checks and order as SimDevice::bo_create.             */

int xvc_bo_args_check(uint64_t size, uint64_t alignment, uint32_t domain, uint32_t flags,
		      uint64_t *out_size, uint64_t *out_align)
{
	uint64_t sz, al;

	if (size == 0 || size > UINT64_MAX - (XVC_PAGE - 1) ||
	    (alignment && (alignment < XVC_PAGE || (alignment & (alignment - 1)))) ||
	    (flags & ~XVC_BO_HOST_VISIBLE))
		return -XVC_EINVAL;
	if (domain != XVC_DOMAIN_VRAM && domain != XVC_DOMAIN_GTT)
		return -XVC_EINVAL;
	if (domain == XVC_DOMAIN_GTT && flags)
		return -XVC_EINVAL;
	sz = (size + XVC_PAGE - 1) & ~(XVC_PAGE - 1);
	/* default alignment lets the page-table writer use 64K / 2M pages */
	al = alignment ? alignment : sz >= XVC_PAGE_2M ? XVC_PAGE_2M :
	     sz >= XVC_PAGE_64K ? XVC_PAGE_64K : XVC_PAGE;
	*out_size = sz;
	*out_align = al;
	return 0;
}

/* ------------------------------------------------------------------------ */
/* Bitmap page pool.                                                        */

size_t xvc_pool_words(uint64_t npages)
{
	return (size_t)((npages + 63) / 64);
}

void xvc_pool_init(struct xvc_pool *p, uint64_t base, uint64_t npages, uint64_t *bits)
{
	p->base = base;
	p->npages = npages;
	p->used = 0;
	p->bits = bits;
	memset(bits, 0, xvc_pool_words(npages) * sizeof(uint64_t));
}

static bool pool_bit(const struct xvc_pool *p, uint64_t i)
{
	return (p->bits[i >> 6] >> (i & 63)) & 1;
}

bool xvc_pool_alloc(struct xvc_pool *p, uint64_t size, uint64_t align, uint64_t *out)
{
	uint64_t n, pos = 0;

	if (!size || size > p->npages * XVC_PAGE || (size & (XVC_PAGE - 1)))
		return false;
	if (align < XVC_PAGE)
		align = XVC_PAGE;
	if (align & (align - 1))
		return false;
	n = size / XVC_PAGE;
	for (;;) {
		uint64_t addr = p->base + pos * XVC_PAGE;
		uint64_t start = (addr + align - 1) & ~(align - 1), i, j;

		if (start < addr)
			return false;
		i = (start - p->base) / XVC_PAGE;
		if (i > p->npages || p->npages - i < n)
			return false;
		for (j = i; j < i + n; j++) {
			if (!(j & 63) && j + 64 <= i + n && p->bits[j >> 6] == 0) {
				j += 63;	/* whole free word */
				continue;
			}
			if (pool_bit(p, j))
				break;
		}
		if (j == i + n) {
			for (j = i; j < i + n; j++)
				p->bits[j >> 6] |= UINT64_C(1) << (j & 63);
			p->used += n;
			*out = start;
			return true;
		}
		pos = j + 1;
	}
}

void xvc_pool_free(struct xvc_pool *p, uint64_t addr, uint64_t size)
{
	uint64_t i = (addr - p->base) / XVC_PAGE, n = size / XVC_PAGE, j;

	for (j = i; j < i + n && j < p->npages; j++) {
		if (pool_bit(p, j)) {
			p->bits[j >> 6] &= ~(UINT64_C(1) << (j & 63));
			p->used--;
		}
	}
}

/* ------------------------------------------------------------------------ */
/* Entry decoding (pte.h decode_entry, rules D1..D15 in order).             */

static bool table_pa_ok(uint64_t pa)
{
	return pa >= NICOGPU_PA_SYS_BASE && pa - NICOGPU_PA_SYS_BASE < NICOGPU_PA_SYS_SIZE;
}

static unsigned int level_shift(unsigned int level)
{
	return 12 + 9 * level;
}

static unsigned int level_index(uint64_t va, unsigned int level)
{
	return (unsigned int)((va >> level_shift(level)) & 511);
}

struct xvc_decoded xvc_decode(uint64_t e, unsigned int level)
{
	struct xvc_decoded d = { XVC_NOT_PRESENT, 1, 0, 0 };
	uint64_t pa = e & PTE_PA, span = UINT64_C(1) << level_shift(level);

	if (!(e & PTE_V) && !(e & PTE_SPARSE))
		return d;
	d.kind = XVC_BAD_PTE;
	if (e & PTE_RSVD) {
		d.rule = 2;
		return d;
	}
	if (e & PTE_SPARSE) {
		if (e & NICOGPU_PTE_SPARSE_MBZ) {
			d.rule = 3;
			return d;
		}
		d.kind = XVC_SPARSE_NULL;
		d.rule = 4;
		d.size = span;
		return d;
	}
	if (level > 0 && !(e & PTE_LEAF)) {
		if (e & NICOGPU_PTE_PDE_MBZ) {
			d.rule = 5;
			return d;
		}
		if (!table_pa_ok(pa)) {
			d.rule = 6;
			return d;
		}
		d.kind = XVC_TABLE;
		d.rule = 7;
		d.base = pa;
		return d;
	}
	if (level == 0 && !(e & PTE_LEAF)) {
		d.rule = 8;
		return d;
	}
	if (level >= 2) {
		d.rule = 9;
		return d;
	}
	if (((e & PTE_MTYPE) >> NICOGPU_PTE_MTYPE_SHIFT) == NICOGPU_MTYPE_RSVD3) {
		d.rule = 10;
		return d;
	}
	if (level == 1) {
		if ((e & PTE_F64K) || (pa & (XVC_PAGE_2M - 1))) {
			d.rule = 11;
			return d;
		}
		d.kind = XVC_LEAF;
		d.rule = 12;
		d.size = XVC_PAGE_2M;
		d.base = pa;
		return d;
	}
	if (e & PTE_F64K) {
		if (pa & (XVC_PAGE_64K - 1)) {
			d.rule = 13;
			return d;
		}
		d.kind = XVC_LEAF;
		d.rule = 14;
		d.size = XVC_PAGE_64K;
		d.base = pa;
		return d;
	}
	d.kind = XVC_LEAF;
	d.rule = 15;
	d.size = XVC_PAGE;
	d.base = pa;
	return d;
}

/* ------------------------------------------------------------------------ */
/* Shadow page tables and the transaction journal.                          */

struct xvc_pt {
	uint64_t e[XVC_ENTRIES];	/* shadow of the device table */
	struct xvc_pt **kids;		/* level > 0: child of each PDE, else NULL */
	uint64_t pa;
	struct xvc_pt *rnext;		/* retired stack link */
	struct xvc_pt *anext;		/* txn-allocated list link */
	bool txn_new;
};

#define JCHUNK 255
struct xvc_jent {
	struct xvc_pt *t;
	struct xvc_pt *old_kid;
	uint64_t old;
	uint32_t idx;
};
struct xvc_jchunk {
	struct xvc_jchunk *next;	/* older chunk */
	uint32_t n;
	struct xvc_jent ent[JCHUNK];
};

static uint64_t make_pte(uint64_t pa, uint64_t attrs, bool f64k)
{
	return (pa & PTE_PA) | PTE_V | PTE_LEAF | (attrs & PTE_ATTR) | (f64k ? PTE_F64K : 0);
}

static uint64_t make_pde(uint64_t pa)
{
	return (pa & PTE_PA) | PTE_V;
}

static bool is_table(uint64_t e, unsigned int level)
{
	return level > 0 && (e & PTE_V) && !(e & PTE_LEAF);
}

static bool is_sparse(uint64_t e)
{
	return !(e & PTE_V) && (e & PTE_SPARSE);
}

static bool is_absent(uint64_t e)
{
	return !(e & PTE_V) && !(e & PTE_SPARSE);
}

uint64_t xvc_leaf_attrs(uint32_t f)
{
	return (f & XVC_VM_READ ? PTE_R : 0) | (f & XVC_VM_WRITE ? PTE_W : 0) |
	       (f & XVC_VM_EXEC ? PTE_X : 0) |
	       ((uint64_t)(f & XVC_VM_UNCACHED ? NICOGPU_MTYPE_UC : NICOGPU_MTYPE_CC) << NICOGPU_PTE_MTYPE_SHIFT);
}

static void pt_destroy(struct xvc_vm *vm, struct xvc_pt *t)
{
	vm->ops->pt_free(vm->ctx, t->pa);
	if (t->kids)
		vm->ops->free(vm->ctx, t->kids);
	vm->ops->free(vm->ctx, t);
}

/* One entry update: journal first (may fail -> nothing written), then the
 * shadow and one 64-bit device store. */
static int wr(struct xvc_vm *vm, struct xvc_pt *t, unsigned int idx, uint64_t e, struct xvc_pt *kid)
{
	if (vm->txn) {
		struct xvc_jchunk *c = vm->journal;

		if (!c || c->n == JCHUNK) {
			c = vm->ops->zalloc(vm->ctx, sizeof(*c));
			if (!c)
				return -XVC_ENOMEM;
			c->next = vm->journal;
			vm->journal = c;
		}
		c->ent[c->n].t = t;
		c->ent[c->n].idx = idx;
		c->ent[c->n].old = t->e[idx];
		c->ent[c->n].old_kid = t->kids ? t->kids[idx] : NULL;
		c->n++;
	}
	t->e[idx] = e;
	if (t->kids)
		t->kids[idx] = kid;
	if (vm->ops->pt_write)
		vm->ops->pt_write(vm->ctx, t->pa + 8ull * idx, e);
	return 0;
}

/* PageTableBuilder::new_table: allocate and fully initialise a table of
 * level `child_level` that replicates `from` (sparse span or 2 MiB leaf),
 * before anything links it. */
static struct xvc_pt *new_table(struct xvc_vm *vm, uint64_t from, unsigned int child_level)
{
	struct xvc_pt *t;
	uint64_t pa = vm->ops->pt_alloc(vm->ctx);
	unsigned int i;

	if (!pa)
		return NULL;
	t = vm->ops->zalloc(vm->ctx, sizeof(*t));
	if (!t)
		goto fail_pa;
	if (child_level > 0) {
		t->kids = vm->ops->zalloc(vm->ctx, XVC_ENTRIES * sizeof(*t->kids));
		if (!t->kids)
			goto fail_t;
	}
	t->pa = pa;
	if (is_sparse(from)) {
		for (i = 0; i < XVC_ENTRIES; i++)
			t->e[i] = SPARSE_NULL;
	} else if ((from & PTE_V) && (from & PTE_LEAF) && child_level == 0) {
		uint64_t base = from & PTE_PA, attrs = from & PTE_ATTR;

		for (i = 0; i < XVC_ENTRIES; i++)
			t->e[i] = vm->large ? make_pte(base + (uint64_t)(i & ~15u) * XVC_PAGE, attrs, true)
					    : make_pte(base + (uint64_t)i * XVC_PAGE, attrs, false);
	}
	if (vm->ops->pt_fill)
		vm->ops->pt_fill(vm->ctx, pa, t->e);
	vm->tables++;
	if (vm->txn) {
		t->txn_new = true;
		t->anext = vm->txn_alloc;
		vm->txn_alloc = t;
	}
	return t;
fail_t:
	vm->ops->free(vm->ctx, t);
fail_pa:
	vm->ops->pt_free(vm->ctx, pa);
	return NULL;
}

static void retire_subtree(struct xvc_vm *vm, struct xvc_pt *t, unsigned int level)
{
	unsigned int i;

	if (level > 0)
		for (i = 0; i < XVC_ENTRIES; i++)
			if (is_table(t->e[i], level))
				retire_subtree(vm, t->kids[i], level - 1);
	t->rnext = vm->retired;
	vm->retired = t;
	vm->tables--;
}

static void free_subtree(struct xvc_vm *vm, struct xvc_pt *t, unsigned int level)
{
	unsigned int i;

	if (level > 0)
		for (i = 0; i < XVC_ENTRIES; i++)
			if (is_table(t->e[i], level))
				free_subtree(vm, t->kids[i], level - 1);
	pt_destroy(vm, t);
	vm->tables--;
}

/* A child left entirely not-present (or entirely sparse) collapses into one
 * entry of its parent.  Such a child has no sub-tables, so retiring it alone
 * is enough (model: retire_subtree(child, 0)). */
static int maybe_collapse(struct xvc_vm *vm, struct xvc_pt *t, unsigned int idx, struct xvc_pt *child)
{
	uint64_t first = child->e[0];
	unsigned int i;
	int rc;

	if (!(first == 0 || first == SPARSE_NULL))
		return 0;
	for (i = 1; i < XVC_ENTRIES; i++)
		if (child->e[i] != first)
			return 0;
	rc = wr(vm, t, idx, first, NULL);
	if (rc)
		return rc;
	retire_subtree(vm, child, 0);	/* no sub-tables: every entry is 0 or sparse */
	return 0;
}

static int demote(struct xvc_vm *vm, struct xvc_pt *t, uint64_t g, uint64_t range_start, uint64_t end)
{
	unsigned int first, j;

	if (g >= range_start && g + XVC_PAGE_64K <= end)
		return 0;
	first = level_index(g, 0);
	for (j = 0; j < 16; j++) {
		uint64_t x = t->e[first + j];

		if ((x & PTE_V) && (x & PTE_F64K)) {
			int rc = wr(vm, t, first + j,
				    make_pte((x & PTE_PA) + j * XVC_PAGE, x & PTE_ATTR, false), NULL);
			if (rc)
				return rc;
		}
	}
	return 0;
}

/* PageTableBuilder::set_level, statement for statement. */
static int set_level(struct xvc_vm *vm, struct xvc_pt *t, unsigned int level, uint64_t va, uint64_t end,
		     uint32_t kind, uint64_t pa, uint64_t va0, uint64_t flags)
{
	const uint64_t span = UINT64_C(1) << level_shift(level);
	const uint64_t range_start = va;
	uint64_t cur;
	int rc;

	for (cur = va; cur < end;) {
		unsigned int idx = level_index(cur, level);
		uint64_t e_start = cur & ~(span - 1), e_end = e_start + span;
		uint64_t sub_end = end < e_end ? end : e_end;
		bool full = cur == e_start && sub_end == e_end;
		uint64_t epa = pa + (cur - va0);
		uint64_t e = t->e[idx];

		if (level == 0) {
			uint64_t ne = 0;

			if (kind == XVC_OP_MAP) {
				uint64_t g = e_start & ~(XVC_PAGE_64K - 1);
				bool f64k = vm->large && g >= range_start && g + XVC_PAGE_64K <= end &&
					    ((pa + (g - va0)) & (XVC_PAGE_64K - 1)) == 0;

				ne = f64k ? make_pte(pa + (g - va0), flags, true) : make_pte(epa, flags, false);
			} else if (kind == XVC_OP_SPARSE_NULL) {
				ne = SPARSE_NULL;
			}
			if (ne != e) {
				rc = wr(vm, t, idx, ne, NULL);
				if (rc)
					return rc;
			}
		} else if ((kind == XVC_OP_UNMAP && is_absent(e)) ||
			   (kind == XVC_OP_SPARSE_NULL && is_sparse(e))) {
			/* already what was asked for over the whole span */
		} else {
			bool leaf_ok = full && (kind != XVC_OP_MAP ||
				(level == 1 && vm->large && (epa & (XVC_PAGE_2M - 1)) == 0));

			if (leaf_ok) {
				struct xvc_pt *old = is_table(e, level) ? t->kids[idx] : NULL;
				uint64_t ne = kind == XVC_OP_MAP ? make_pte(epa, flags, false) :
					      kind == XVC_OP_SPARSE_NULL ? SPARSE_NULL : 0;

				rc = wr(vm, t, idx, ne, NULL);
				if (rc)
					return rc;
				if (old)
					retire_subtree(vm, old, level - 1);
			} else {
				struct xvc_pt *child;

				if (is_table(e, level)) {
					child = t->kids[idx];
				} else {
					child = new_table(vm, e, level - 1);
					if (!child)
						return -XVC_ENOMEM;
					rc = wr(vm, t, idx, make_pde(child->pa), child);
					if (rc)
						return rc;	/* child is in txn_alloc (txn) */
				}
				rc = set_level(vm, child, level - 1, cur, sub_end, kind, pa, va0, flags);
				if (rc)
					return rc;
				if (kind != XVC_OP_MAP) {
					rc = maybe_collapse(vm, t, idx, child);
					if (rc)
						return rc;
				}
			}
		}
		cur = sub_end;
	}
	if (level == 0) {
		/* A partly rewritten 64K group must not keep F64K anywhere:
		 * demote the untouched members to 4K entries. */
		rc = demote(vm, t, range_start & ~(XVC_PAGE_64K - 1), range_start, end);
		if (rc)
			return rc;
		rc = demote(vm, t, (end - 1) & ~(XVC_PAGE_64K - 1), range_start, end);
		if (rc)
			return rc;
	}
	return 0;
}

int xvc_pt_set(struct xvc_vm *vm, uint64_t va, uint64_t size, uint32_t kind, uint64_t pa, uint64_t attrs)
{
	if (!vm->root)
		return -XVC_ENOMEM;
	if (!size)
		return 0;
	return set_level(vm, vm->root, ROOT_LEVEL, va, va + size, kind, pa, va, attrs & PTE_ATTR);
}

static void journal_free(struct xvc_vm *vm)
{
	while (vm->journal) {
		struct xvc_jchunk *c = vm->journal;

		vm->journal = c->next;
		vm->ops->free(vm->ctx, c);
	}
}

void xvc_pt_begin(struct xvc_vm *vm)
{
	vm->txn = true;
	journal_free(vm);
	vm->txn_alloc = NULL;
	vm->txn_mark = vm->retired;
}

static void txn_clear_new(struct xvc_vm *vm)
{
	struct xvc_pt *t, *n;

	for (t = vm->txn_alloc; t; t = n) {
		n = t->anext;
		t->anext = NULL;
		t->txn_new = false;
	}
	vm->txn_alloc = NULL;
}

void xvc_pt_commit(struct xvc_vm *vm)
{
	vm->txn = false;
	journal_free(vm);
	txn_clear_new(vm);
}

void xvc_pt_rollback(struct xvc_vm *vm)
{
	struct xvc_jchunk *c;
	struct xvc_pt *t, *n;

	if (!vm->txn)
		return;
	/* restore every journaled entry, newest first */
	for (c = vm->journal; c; c = c->next) {
		uint32_t k = c->n;

		while (k--) {
			struct xvc_jent *j = &c->ent[k];

			j->t->e[j->idx] = j->old;
			if (j->t->kids)
				j->t->kids[j->idx] = j->old_kid;
			if (vm->ops->pt_write)
				vm->ops->pt_write(vm->ctx, j->t->pa + 8ull * j->idx, j->old);
		}
	}
	/* tables unlinked in this transaction are linked again */
	while (vm->retired != vm->txn_mark) {
		t = vm->retired;
		vm->retired = t->rnext;
		t->rnext = NULL;
		vm->tables++;
	}
	/* tables allocated in this transaction are unreachable again but may
	 * have been seen by the walker: retire them (freed after invalidation) */
	for (t = vm->txn_alloc; t; t = n) {
		n = t->anext;
		t->anext = NULL;
		t->txn_new = false;
		t->rnext = vm->retired;
		vm->retired = t;
		vm->tables--;
	}
	vm->txn_alloc = NULL;
	vm->txn = false;
	journal_free(vm);
}

size_t xvc_pt_reclaim(struct xvc_vm *vm)
{
	size_t n = 0;

	if (vm->txn)
		return 0;
	while (vm->retired) {
		struct xvc_pt *t = vm->retired;

		vm->retired = t->rnext;
		pt_destroy(vm, t);
		n++;
	}
	return n;
}

size_t xvc_pt_retired(const struct xvc_vm *vm)
{
	const struct xvc_pt *t;
	size_t n = 0;

	for (t = vm->retired; t; t = t->rnext)
		n++;
	return n;
}

/* ------------------------------------------------------------------------ */
/* VM lifetime, lookup, verify, digest.                                     */

int xvc_vm_init(struct xvc_vm *vm, const struct xvc_ops *ops, void *ctx)
{
	memset(vm, 0, sizeof(*vm));
	vm->ops = ops;
	vm->ctx = ctx;
	vm->large = true;
	vm->max_maps = XVC_MAX_MAPPINGS;
	vm->root = new_table(vm, 0, ROOT_LEVEL);	/* model: new_table(0, kLevels - 1) */
	return vm->root ? 0 : -XVC_ENOMEM;
}

static void put_maps(struct xvc_vm *vm, struct xvc_map *m, uint32_t n)
{
	uint32_t i;

	if (!vm->ops->bo_put)
		return;
	for (i = 0; i < n; i++)
		if (m[i].bo)
			vm->ops->bo_put(vm->ctx, m[i].bo);
}

void xvc_vm_fini(struct xvc_vm *vm)
{
	if (vm->txn)
		xvc_pt_rollback(vm);
	if (vm->root)
		free_subtree(vm, vm->root, ROOT_LEVEL);
	vm->root = NULL;
	xvc_pt_reclaim(vm);
	put_maps(vm, vm->maps, vm->nmaps);
	if (vm->maps)
		vm->ops->free(vm->ctx, vm->maps);
	vm->maps = NULL;
	vm->nmaps = 0;
}

uint64_t xvc_vm_root_pa(const struct xvc_vm *vm)
{
	return vm->root ? vm->root->pa : 0;
}

void xvc_vm_lookup(const struct xvc_vm *vm, uint64_t va, struct xvc_xlate *out)
{
	const struct xvc_pt *t = vm->root;
	unsigned int level = ROOT_LEVEL;

	memset(out, 0, sizeof(*out));
	out->kind = XVC_NOT_PRESENT;
	if (!t || (va >> NICOGPU_VA_BITS))
		return;
	for (;;) {
		unsigned int idx = level_index(va, level);
		uint64_t e = t->e[idx];
		struct xvc_decoded d = xvc_decode(e, level);

		out->level = level;
		out->entry = e;
		if (d.kind == XVC_TABLE) {
			t = t->kids[idx];
			level--;
			continue;
		}
		out->kind = d.kind;
		if (d.kind == XVC_LEAF) {
			out->page_size = d.size;
			out->pa = d.base + (va & (d.size - 1));
		} else if (d.kind == XVC_SPARSE_NULL) {
			out->page_size = d.size;
		}
		return;
	}
}

static const char *verify_table(const struct xvc_pt *t, unsigned int level, size_t *count)
{
	unsigned int i, j;

	(*count)++;
	for (i = 0; i < XVC_ENTRIES; i++) {
		uint64_t e = t->e[i];
		struct xvc_decoded d = xvc_decode(e, level);

		if (d.kind == XVC_BAD_PTE)
			return "BAD_PTE entry";
		if (d.kind == XVC_TABLE) {
			const char *why;

			if (!t->kids || !t->kids[i] || t->kids[i]->pa != d.base)
				return "shadow child does not match PDE";
			why = verify_table(t->kids[i], level - 1, count);
			if (why)
				return why;
		} else if (t->kids && t->kids[i]) {
			return "stale shadow child pointer";
		}
		if (level == 0 && (i % 16) == 0) {
			bool any = false;

			for (j = 0; j < 16; j++)
				any |= (t->e[i + j] & (PTE_V | PTE_F64K)) == (PTE_V | PTE_F64K);
			if (any)
				for (j = 1; j < 16; j++)
					if (t->e[i + j] != e)
						return "inconsistent 64K group";
		}
	}
	return NULL;
}

const char *xvc_vm_verify(const struct xvc_vm *vm)
{
	size_t count = 0;
	const char *why;
	uint32_t i;

	if (!vm->root)
		return "no root";
	if (!table_pa_ok(vm->root->pa))
		return "root outside SYS";
	why = verify_table(vm->root, ROOT_LEVEL, &count);
	if (why)
		return why;
	/* a table linked twice would be counted twice */
	if (count != vm->tables)
		return "live table count mismatch";
	for (i = 0; i < vm->nmaps; i++) {
		if (!vm->maps[i].size)
			return "empty mapping record";
		if (i && vm->maps[i - 1].va + vm->maps[i - 1].size > vm->maps[i].va)
			return "mapping records overlap or unsorted";
	}
	return NULL;
}

uint64_t xvc_fnv_mix(uint64_t h, uint64_t v)
{
	unsigned int i;

	for (i = 0; i < 8; i++) {
		h ^= (uint8_t)(v >> (8 * i));
		h *= UINT64_C(1099511628211);
	}
	return h;
}

static uint64_t digest_walk(const struct xvc_pt *t, unsigned int level, uint64_t h)
{
	unsigned int i;

	h = xvc_fnv_mix(h, t->pa);
	for (i = 0; i < XVC_ENTRIES; i++) {
		uint64_t e = t->e[i];

		h = xvc_fnv_mix(h, e);
		if (level > 0 && (e & PTE_V) && !(e & PTE_LEAF))
			h = digest_walk(t->kids[i], level - 1, h);
	}
	return h;
}

uint64_t xvc_vm_digest(const struct xvc_vm *vm, uint64_t h)
{
	return vm->root ? digest_walk(vm->root, ROOT_LEVEL, h) : h;
}

/* ------------------------------------------------------------------------ */
/* VM_BIND.                                                                 */

/* Remove [va, va+size) from the sorted mapping array, splitting partial
 * overlaps (model carve()).  The array has room for one more record.
 * Returns the index where a record starting at va belongs. */
static uint32_t carve(struct xvc_map *m, uint32_t *pn, uint64_t va, uint64_t size)
{
	uint64_t end = va + size;
	uint32_t n = *pn, lo = 0, hi = n, keep;
	struct xvc_map l, r;
	bool keep_l, keep_r;

	/* first record ending after va (records are sorted and disjoint) */
	while (lo < hi) {
		uint32_t mid = lo + (hi - lo) / 2;

		if (m[mid].va + m[mid].size > va)
			hi = mid;
		else
			lo = mid + 1;
	}
	for (hi = lo; hi < n && m[hi].va < end; hi++)
		;
	if (lo == hi)
		return lo;
	l = m[lo];
	r = m[hi - 1];
	keep_l = l.va < va;
	keep_r = r.va + r.size > end;
	keep = (uint32_t)keep_l + (uint32_t)keep_r;
	memmove(&m[lo + keep], &m[hi], (size_t)(n - hi) * sizeof(*m));
	if (keep_l) {
		l.size = va - l.va;
		m[lo] = l;
	}
	if (keep_r) {
		uint64_t cut = end - r.va;

		r.va = end;
		r.size -= cut;
		r.bo_offset += cut;
		m[lo + keep_l] = r;
	}
	*pn = n - (hi - lo) + keep;
	return lo + keep_l;
}

static void insert_at(struct xvc_map *m, uint32_t *pn, uint32_t at, const struct xvc_map *rec)
{
	memmove(&m[at + 1], &m[at], (size_t)(*pn - at) * sizeof(*m));
	m[at] = *rec;
	(*pn)++;
}

/* Validate the whole array first (SimDevice::vm_bind order) so a bad op
 * leaves the VM untouched.  `bos` (may be NULL) receives the MAP BOs. */
static int validate(struct xvc_vm *vm, const struct xvc_bind_op *ops, uint32_t n, struct xvc_bo_info *bos)
{
	uint32_t i;

	for (i = 0; i < n; i++) {
		const struct xvc_bind_op *o = &ops[i];

		if (o->op > XVC_OP_SPARSE_NULL || o->pad || ((o->va | o->range | o->bo_offset) & (XVC_PAGE - 1)) ||
		    !o->range)
			return -XVC_EINVAL;
		if (o->va < NICOGPU_VA_USER_START || o->va + o->range > NICOGPU_VA_USER_END || o->va + o->range < o->va)
			return -XVC_EINVAL;
		if (o->op == XVC_OP_MAP) {
			struct xvc_bo_info b;
			int rc = vm->ops->bo_lookup(vm->ctx, o->bo_handle, &b);

			if (rc)
				return rc < 0 ? rc : -XVC_ENOENT;
			if (o->bo_offset + o->range > b.size || o->bo_offset + o->range < o->bo_offset)
				return -XVC_EINVAL;
			if (o->flags & ~XVC_VM_FLAGS_ALL)
				return -XVC_EINVAL;
			if (bos)
				bos[i] = b;
		} else if (o->flags || o->bo_handle || o->bo_offset) {
			return -XVC_EINVAL;
		}
	}
	return 0;
}

static void finish(struct xvc_vm *vm)
{
	/* Synchronous TLB invalidation (HAL v0): the walker may have seen
	 * transient entries on either path.  Then free unlinked tables --
	 * only once an invalidation has completed. */
	bool ok = vm->ops->invalidate ? vm->ops->invalidate(vm->ctx) : true;

	vm->reclaim_blocked = !ok;
	if (ok)
		xvc_pt_reclaim(vm);
}

int xvc_vm_bind(struct xvc_vm *vm, const struct xvc_bind_op *ops, uint32_t n)
{
	struct xvc_bo_info *bos = NULL;
	struct xvc_map *staged = NULL;
	uint32_t i, ns, cap;
	int rc;

	if (n > XVC_MAX_BIND_OPS)
		return -XVC_EINVAL;
	rc = validate(vm, ops, n, NULL);
	if (rc || n == 0)
		return rc;

	/* Everything below is one transaction: any failure restores the exact
	 * previous tables and mapping table (whole-call atomic). */
	rc = -XVC_ENOMEM;
	cap = vm->nmaps + 2 * n;	/* each op adds at most 2 records */
	bos = vm->ops->zalloc(vm->ctx, (size_t)n * sizeof(*bos));
	staged = vm->ops->zalloc(vm->ctx, (size_t)cap * sizeof(*staged));
	xvc_pt_begin(vm);
	if (!bos || !staged)
		goto fail;
	rc = validate(vm, ops, n, bos);	/* resolve BOs; caller holds the lock */
	if (rc)
		goto fail;
	if (vm->nmaps)
		memcpy(staged, vm->maps, (size_t)vm->nmaps * sizeof(*staged));
	ns = vm->nmaps;
	for (i = 0; i < n; i++) {
		const struct xvc_bind_op *o = &ops[i];
		uint32_t at = carve(staged, &ns, o->va, o->range);
		struct xvc_map rec;

		memset(&rec, 0, sizeof(rec));
		rec.va = o->va;
		rec.size = o->range;
		if (o->op == XVC_OP_MAP) {
			rec.kind = XVC_OP_MAP;
			rec.bo = bos[i].bo;
			rec.bo_pa = bos[i].pa;
			rec.bo_offset = o->bo_offset;
			rec.flags = o->flags;
			rc = xvc_pt_set(vm, o->va, o->range, XVC_OP_MAP, bos[i].pa + o->bo_offset,
					xvc_leaf_attrs(o->flags));
			insert_at(staged, &ns, at, &rec);
		} else if (o->op == XVC_OP_SPARSE_NULL) {
			rec.kind = XVC_OP_SPARSE_NULL;
			rc = xvc_pt_set(vm, o->va, o->range, XVC_OP_SPARSE_NULL, 0, 0);
			insert_at(staged, &ns, at, &rec);
		} else {
			rc = xvc_pt_set(vm, o->va, o->range, XVC_OP_UNMAP, 0, 0);
		}
		if (rc)
			goto fail;
	}
	if (ns > vm->max_maps) {
		rc = -XVC_ENOSPC;
		goto fail;
	}
	xvc_pt_commit(vm);
	/* references: take the new ones before dropping the old ones */
	if (vm->ops->bo_get)
		for (i = 0; i < ns; i++)
			if (staged[i].bo)
				vm->ops->bo_get(vm->ctx, staged[i].bo);
	finish(vm);
	put_maps(vm, vm->maps, vm->nmaps);
	if (vm->maps)
		vm->ops->free(vm->ctx, vm->maps);
	vm->maps = staged;
	vm->nmaps = ns;
	vm->ops->free(vm->ctx, bos);
	return 0;

fail:
	xvc_pt_rollback(vm);
	finish(vm);
	if (staged)
		vm->ops->free(vm->ctx, staged);
	if (bos)
		vm->ops->free(vm->ctx, bos);
	return rc;
}
