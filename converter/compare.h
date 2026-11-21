// Comparing two versions of a drawing (D116, 판 비교): two caches this
// converter made (old, new) in, one cache out with every primitive on one of
// three layers - unchanged (grey), only in the new one (red), only in the old
// one (blue) - so the viewer's layer switches show or hide each kind.
//
// Both caches come from the same converter, so an unchanged entity gives the
// same world coordinates in both, up to each cache's fixed-point step (the
// two can have different origins and steps when the drawing's extent
// changed). Primitives are matched with that tolerance: a spatial hash on an
// anchor point (midpoint, centre, first vertex ...), then every coordinate is
// checked. Each old primitive matches at most one new one (a multiset).
// Linetypes are not kept (drawn continuous); colours are replaced by the
// three kinds.

#define CMP_SAME 0
#define CMP_ADDED 1
#define CMP_REMOVED 2

typedef struct {
  uint64_t cell;  // spatial hash of the anchor
  uint32_t kind;  // 0 line, 1 arc, 2 poly, 3 text, 4 fill, 5 hatch line
  uint32_t idx;   // primitive index in its view
  uint32_t first; // poly / fill: first point index
  uint8_t used;
} CmpItem;

typedef struct {
  const UmgView *v;
  double ox, oy, q;
} CmpSide;

#define CX(s, x) ((s)->ox + (double)(x) * (s)->q)
#define CY(s, y) ((s)->oy + (double)(y) * (s)->q)

static uint64_t cmp_cell(double x, double y, double c) {
  int64_t i = (int64_t)floor(x / c), j = (int64_t)floor(y / c);
  return (uint64_t)i * 0x9E3779B97F4A7C15ull ^ (uint64_t)j * 0xC2B2AE3D27D4EB4Full;
}

static int cmp_items(const void *a, const void *b) {
  const CmpItem *x = (const CmpItem *)a, *y = (const CmpItem *)b;
  return x->cell < y->cell ? -1 : x->cell > y->cell;
}

// Anchor point of a primitive in world coordinates.
static void cmp_anchor(const CmpSide *s, uint32_t kind, uint32_t i, uint32_t first, double *ax, double *ay) {
  const UmgView *v = s->v;
  switch (kind) {
  case 0: case 5: {
    const UmgLine *l = kind ? &v->hlines[i] : &v->lines[i];
    *ax = (CX(s, l->x1) + CX(s, l->x2)) / 2, *ay = (CY(s, l->y1) + CY(s, l->y2)) / 2;
  } break;
  case 1: *ax = CX(s, v->arcs[i].cx), *ay = CY(s, v->arcs[i].cy); break;
  case 2: {  // the mean of the vertices: the same whichever way it runs
    uint32_t n = v->polys[i] & UMG_COUNT;
    double sx = 0, sy = 0;
    for (uint32_t k = 0; k < n; k++) sx += CX(s, v->pts[first + k].x), sy += CY(s, v->pts[first + k].y);
    *ax = n ? sx / n : 0, *ay = n ? sy / n : 0;
  } break;
  case 3: *ax = CX(s, v->texts[i].x), *ay = CY(s, v->texts[i].y); break;
  case 4: {
    uint32_t n = v->fills[i];
    double sx = 0, sy = 0;
    for (uint32_t k = 0; k < n; k++) sx += CX(s, v->fpts[first + k].x), sy += CY(s, v->fpts[first + k].y);
    *ax = n ? sx / n : 0, *ay = n ? sy / n : 0;
  } break;
  default: *ax = *ay = 0;
  }
}

static int cmp_near(double a, double b, double tol) { return fabs(a - b) <= tol; }

static int cmp_angle(double a, double b) {
  double d = fmod(fabs(a - b), 2 * M_PI);
  return d < 1e-4 || 2 * M_PI - d < 1e-4;
}

// Is old primitive (A, i, fi) the same as new primitive (B, j, fj)?
static int cmp_same(const CmpSide *A, uint32_t kind, uint32_t i, uint32_t fi,
                    const CmpSide *B, uint32_t j, uint32_t fj, double tol) {
  const UmgView *a = A->v, *b = B->v;
  switch (kind) {
  case 0: case 5: {
    const UmgLine *p = kind ? &a->hlines[i] : &a->lines[i], *r = kind ? &b->hlines[j] : &b->lines[j];
    double ax1 = CX(A, p->x1), ay1 = CY(A, p->y1), ax2 = CX(A, p->x2), ay2 = CY(A, p->y2);
    double bx1 = CX(B, r->x1), by1 = CY(B, r->y1), bx2 = CX(B, r->x2), by2 = CY(B, r->y2);
    return (cmp_near(ax1, bx1, tol) && cmp_near(ay1, by1, tol) && cmp_near(ax2, bx2, tol) && cmp_near(ay2, by2, tol)) ||
           (cmp_near(ax1, bx2, tol) && cmp_near(ay1, by2, tol) && cmp_near(ax2, bx1, tol) && cmp_near(ay2, by1, tol));
  }
  case 1: {
    const UmgArc *p = &a->arcs[i], *r = &b->arcs[j];
    int fp = p->a1 - p->a0 >= 2 * M_PI - 1e-6 || p->a1 - p->a0 <= -2 * M_PI + 1e-6;
    int fr = r->a1 - r->a0 >= 2 * M_PI - 1e-6 || r->a1 - r->a0 <= -2 * M_PI + 1e-6;
    return cmp_near(CX(A, p->cx), CX(B, r->cx), tol) && cmp_near(CY(A, p->cy), CY(B, r->cy), tol) &&
           cmp_near(p->r * A->q, r->r * B->q, tol) && fp == fr &&
           (fp || (cmp_angle(p->a0, r->a0) && cmp_angle(p->a1, r->a1)));
  }
  case 2: {
    uint32_t n = a->polys[i] & UMG_COUNT, m = b->polys[j] & UMG_COUNT;
    if (n != m || (a->polys[i] & UMG_CLOSED) != (b->polys[j] & UMG_CLOSED)) return 0;
    int fwd = 1, rev = 1;
    for (uint32_t k = 0; k < n && (fwd || rev); k++) {
      const UmgPt *p = &a->pts[fi + k], *r = &b->pts[fj + k], *rr = &b->pts[fj + n - 1 - k];
      if (fwd && !(cmp_near(CX(A, p->x), CX(B, r->x), tol) && cmp_near(CY(A, p->y), CY(B, r->y), tol) &&
                   fabs(p->bulge - r->bulge) < 1e-5))
        fwd = 0;
      // Reversed: the bulge of a segment sits on its other end, with the sign turned.
      const UmgPt *rb = &b->pts[fj + (n + n - 2 - k) % n];
      if (rev && !(cmp_near(CX(A, p->x), CX(B, rr->x), tol) && cmp_near(CY(A, p->y), CY(B, rr->y), tol) &&
                   (k + 1 == n || fabs(p->bulge + rb->bulge) < 1e-5)))
        rev = 0;
    }
    return fwd || rev;
  }
  case 3: {
    const UmgText *p = &a->texts[i], *r = &b->texts[j];
    return p->len == r->len && !memcmp(a->pool + p->off, b->pool + r->off, p->len) &&
           cmp_near(CX(A, p->x), CX(B, r->x), tol) && cmp_near(CY(A, p->y), CY(B, r->y), tol) &&
           fabs(p->h - r->h) <= tol + 1e-6 * fabs(p->h) && cmp_angle(p->rot, r->rot);
  }
  case 4: {
    uint32_t n = a->fills[i], m = b->fills[j];
    if (n != m) return 0;
    for (uint32_t k = 0; k < n; k++)
      if (!cmp_near(CX(A, a->fpts[fi + k].x), CX(B, b->fpts[fj + k].x), tol) ||
          !cmp_near(CY(A, a->fpts[fi + k].y), CY(B, b->fpts[fj + k].y), tol))
        return 0;
    return 1;
  }
  }
  return 0;
}

// All primitives of one side as items (first point indices filled in).
static CmpItem *cmp_list(const CmpSide *s, double cell, size_t *count) {
  const UmgHeader *h = &s->v->h;
  size_t n = (size_t)h->n_lines + h->n_arcs + h->n_polys + h->n_texts + h->n_fills + h->n_hlines;
  CmpItem *it = (CmpItem *)calloc(n ? n : 1, sizeof(CmpItem));
  size_t k = 0;
  uint32_t counts[6] = {h->n_lines, h->n_arcs, h->n_polys, h->n_texts, h->n_fills, h->n_hlines};
  uint32_t p0 = 0, f0 = 0;
  for (uint32_t kind = 0; kind < 6; kind++)
    for (uint32_t i = 0; i < counts[kind]; i++, k++) {
      it[k].kind = kind, it[k].idx = i;
      if (kind == 2) it[k].first = p0, p0 += s->v->polys[i] & UMG_COUNT;
      if (kind == 4) it[k].first = f0, f0 += s->v->fills[i];
      double x, y;
      cmp_anchor(s, kind, i, it[k].first, &x, &y);
      it[k].cell = cmp_cell(x, y, cell);
    }
  *count = n;
  return it;
}

typedef struct { double x0, y0, x1, y1; } CmpBox;

// 바뀐 곳: boxes around groups of changed primitives, for the viewer's
// "next change" buttons and revision clouds (at most CMP_ZONES).
#define CMP_ZONES 60

typedef struct {
  unsigned long same, added, removed;
  double x0, y0, x1, y1;   // everything output
  CmpBox *ch;              // one box per changed primitive
  size_t nch, cch;
  CmpBox zone[CMP_ZONES];  // in reading order: top rows first, left to right
  int nzone;
} CmpStats;

static void cmp_grow(CmpBox *b, double x, double y) {
  if (!isfinite(x) || !isfinite(y)) return;
  if (x < b->x0) b->x0 = x;
  if (x > b->x1) b->x1 = x;
  if (y < b->y0) b->y0 = y;
  if (y > b->y1) b->y1 = y;
}

// Puts one primitive of side s into g with the style of its kind.
static void cmp_emit(Geom *g, const CmpSide *s, const CmpItem *it, uint32_t style, CmpStats *st, int changed) {
  const UmgView *v = s->v;
  CmpBox bb = {INFINITY, INFINITY, -INFINITY, -INFINITY}, *b = &bb;
  g->cur = style;
  OrdKey k0 = {0, 0};
  g->ord = k0;
  switch (it->kind) {
  case 0: case 5: {
    const UmgLine *l = it->kind ? &v->hlines[it->idx] : &v->lines[it->idx];
    double x1 = CX(s, l->x1), y1 = CY(s, l->y1), x2 = CX(s, l->x2), y2 = CY(s, l->y2);
    if (it->kind) {
      g->lod = v->hline_lod[it->idx];
      add_hline(g, XF_ID, x1, y1, x2, y2);
      g->lod = INFINITY;
    } else {
      add_line(g, XF_ID, x1, y1, x2, y2);
    }
    cmp_grow(b, x1, y1), cmp_grow(b, x2, y2);
  } break;
  case 1: {
    const UmgArc *a = &v->arcs[it->idx];
    double cx = CX(s, a->cx), cy = CY(s, a->cy), r = a->r * s->q;
    add_arc(g, XF_ID, cx, cy, r, a->a0, a->a1);
    cmp_grow(b, cx - r, cy - r), cmp_grow(b, cx + r, cy + r);
  } break;
  case 2: {
    uint32_t head = v->polys[it->idx], n = head & UMG_COUNT;
    begin_poly(g, n, head & (UMG_CLOSED | UMG_LTCONT));
    for (uint32_t k = 0; k < n; k++) {
      const UmgPt *p = &v->pts[it->first + k];
      add_pt(g, XF_ID, CX(s, p->x), CY(s, p->y), p->bulge);
      cmp_grow(b, CX(s, p->x), CY(s, p->y));
    }
  } break;
  case 3: {
    UmgText r = v->texts[it->idx];
    if ((uint64_t)r.off + r.len > s->v->h.pool_bytes) break;
    push_text(g, r, CX(s, r.x), CY(s, r.y), CX(s, r.x2), CY(s, r.y2), v->pool + r.off, r.len);
    cmp_grow(b, CX(s, r.x), CY(s, r.y));
  } break;
  case 4: {
    uint32_t n = v->fills[it->idx];
    double *xy = (double *)malloc((n ? n : 1) * 2 * sizeof(double));
    uint32_t *tri = (uint32_t *)malloc((n ? n : 1) * sizeof(uint32_t));
    for (uint32_t k = 0; k < n; k++) {
      xy[2 * k] = CX(s, v->fpts[it->first + k].x), xy[2 * k + 1] = CY(s, v->fpts[it->first + k].y);
      tri[k] = k;
      cmp_grow(b, xy[2 * k], xy[2 * k + 1]);
    }
    add_fill(g, XF_ID, xy, tri, n);
    free(xy), free(tri);
  } break;
  }
  if (!(bb.x1 >= bb.x0)) return;
  st->x0 = fmin(st->x0, bb.x0), st->y0 = fmin(st->y0, bb.y0);
  st->x1 = fmax(st->x1, bb.x1), st->y1 = fmax(st->y1, bb.y1);
  if (!changed) return;
  if (st->nch == st->cch) {
    size_t c = st->cch ? st->cch * 2 : 1024;
    CmpBox *p = (CmpBox *)realloc(st->ch, c * sizeof *p);
    if (!p) return;
    st->ch = p, st->cch = c;
  }
  st->ch[st->nch++] = bb;
}

// Groups the changed primitives into at most CMP_ZONES boxes. Grouped by
// their centres (a long changed line must not join everything it crosses):
// each centre into a cell of a grid 1/30 of the drawing (a room or two on
// a floor plan), and touching cells
// (sides or corners) are one group; a coarser grid while there are too
// many. A group's box covers its primitives whole.
static void cmp_zones(CmpStats *st) {
  st->nzone = 0;
  double ext = fmax(st->x1 - st->x0, st->y1 - st->y0);
  if (!st->nch || !(ext > 0) || !isfinite(ext)) return;
  for (int div = 30; div >= 1; div /= 2) {
    double G = ext / div;
    int N = div + 1;
    size_t NN = (size_t)N * N;
    CmpBox *full = (CmpBox *)malloc(NN * sizeof *full);
    int *stack = (int *)malloc(NN * sizeof *stack);
    CmpBox *zones = (CmpBox *)malloc(NN * sizeof *zones);
    if (!full || !stack || !zones) { free(full), free(stack), free(zones); return; }
    for (size_t k = 0; k < NN; k++) full[k].x0 = full[k].y0 = INFINITY, full[k].x1 = full[k].y1 = -INFINITY;
    for (size_t k = 0; k < st->nch; k++) {
      const CmpBox *c = &st->ch[k];
      int i = (int)(((c->x0 + c->x1) / 2 - st->x0) / G), j = (int)(((c->y0 + c->y1) / 2 - st->y0) / G);
      i = i < 0 ? 0 : i >= N ? N - 1 : i, j = j < 0 ? 0 : j >= N ? N - 1 : j;
      cmp_grow(&full[j * N + i], c->x0, c->y0), cmp_grow(&full[j * N + i], c->x1, c->y1);
    }
    // Groups of touching cells; a visited cell is emptied.
    size_t n = 0;
    for (size_t k = 0; k < NN; k++) {
      if (!(full[k].x1 >= full[k].x0)) continue;
      CmpBox z = full[k];
      full[k].x0 = INFINITY, full[k].x1 = -INFINITY;
      size_t top = 0;
      stack[top++] = (int)k;
      while (top) {
        int c = stack[--top], ci = c % N, cj = c / N;
        for (int dj = -1; dj <= 1; dj++)
          for (int di = -1; di <= 1; di++) {
            int i = ci + di, j = cj + dj;
            if (i < 0 || j < 0 || i >= N || j >= N) continue;
            CmpBox *b = &full[j * N + i];
            if (!(b->x1 >= b->x0)) continue;
            cmp_grow(&z, b->x0, b->y0), cmp_grow(&z, b->x1, b->y1);
            b->x0 = INFINITY, b->x1 = -INFINITY;
            stack[top++] = j * N + i;
          }
      }
      zones[n++] = z;
    }
    free(full), free(stack);
    // Groups that touch (the edges of one changed rectangle land in
    // different cells) are joined while the joined box stays within ten
    // cells: geometry that connects is one change, a long line is not
    // allowed to string everything together.
    for (int merged = 1; merged;) {
      merged = 0;
      for (size_t a = 0; a < n; a++)
        for (size_t b = a + 1; b < n; b++) {
          const CmpBox *p = &zones[a], *q = &zones[b];
          double gap = G / 4;
          if (p->x0 - gap > q->x1 || q->x0 - gap > p->x1 || p->y0 - gap > q->y1 || q->y0 - gap > p->y1) continue;
          CmpBox u = *p;
          cmp_grow(&u, q->x0, q->y0), cmp_grow(&u, q->x1, q->y1);
          if (u.x1 - u.x0 > 10 * G || u.y1 - u.y0 > 10 * G) continue;
          zones[a] = u, zones[b--] = zones[--n], merged = 1;
        }
    }
    if (n <= CMP_ZONES || div == 1) {
      // Reading order: bands a twentieth of the drawing from the top, then left to right.
      double band = ext / 20;
      for (size_t a = 1; a < n; a++)
        for (size_t b = a; b > 0; b--) {
          CmpBox *p = &zones[b - 1], *q = &zones[b];
          double rp = floor((st->y1 - p->y1) / band), rq = floor((st->y1 - q->y1) / band);
          if (rp < rq || (rp == rq && p->x0 <= q->x0)) break;
          CmpBox t = *p;
          *p = *q, *q = t;
        }
      for (size_t a = 0; a < n && a < CMP_ZONES; a++) st->zone[st->nzone++] = zones[a];
      free(zones);
      return;
    }
    free(zones);
  }
}

// The counts inside the sections agree with the section sizes (a cache
// damaged on the phone's storage must not send the reads past them).
static int cmp_valid(const UmgView *v) {
  uint64_t np = 0, nf = 0;
  for (uint32_t i = 0; i < v->h.n_polys; i++) np += v->polys[i] & UMG_COUNT;
  for (uint32_t i = 0; i < v->h.n_fills; i++) nf += v->fills[i];
  if (np > v->h.n_pts || nf > v->h.n_fpts) return 0;
  for (uint32_t i = 0; i < v->h.n_texts; i++)
    if ((uint64_t)v->texts[i].off + v->texts[i].len > v->h.pool_bytes) return 0;
  return 1;
}

// Compares old (a) and new (b) into g / t. Returns 0, or 3 when a cache is
// not a cache of this format.
static int cmp_run(const void *raw_a, size_t na, const void *raw_b, size_t nb, Geom *g, Tables *t, CmpStats *st) {
  UmgView va, vb;
  if (!umg_view(raw_a, na, &va) || !umg_view(raw_b, nb, &vb) || !cmp_valid(&va) || !cmp_valid(&vb)) return 3;
  CmpSide A = {&va, va.h.ox, va.h.oy, va.h.q}, B = {&vb, vb.h.ox, vb.h.oy, vb.h.q};
  if (!(A.q > 0) || !(B.q > 0) || !isfinite(A.ox + A.oy + B.ox + B.oy)) return 3;
  // A few fixed-point steps of either cache; the cell is wider than any
  // match can be apart, and the 3 x 3 cells around an anchor are searched.
  double tol = 4 * fmax(A.q, B.q), cell = 4 * tol;
  size_t nA, nB;
  CmpItem *ia = cmp_list(&A, cell, &nA), *ib = cmp_list(&B, cell, &nB);
  qsort(ia, nA, sizeof *ia, cmp_items);
  // For each new primitive, an unused old one that is the same.
  uint8_t *same_b = (uint8_t *)calloc(nB ? nB : 1, 1);
  for (size_t j = 0; j < nB; j++) {
    double x, y;
    cmp_anchor(&B, ib[j].kind, ib[j].idx, ib[j].first, &x, &y);
    int found = 0;
    for (int dx = -1; dx <= 1 && !found; dx++)
      for (int dy = -1; dy <= 1 && !found; dy++) {
        CmpItem key = {0};
        key.cell = cmp_cell(x + dx * cell, y + dy * cell, cell);
        // first item with this cell
        size_t lo = 0, hi = nA;
        while (lo < hi) {
          size_t mid = (lo + hi) / 2;
          if (ia[mid].cell < key.cell) lo = mid + 1;
          else hi = mid;
        }
        for (size_t k = lo; k < nA && ia[k].cell == key.cell; k++) {
          if (ia[k].used || ia[k].kind != ib[j].kind) continue;
          if (cmp_same(&A, ia[k].kind, ia[k].idx, ia[k].first, &B, ib[j].idx, ib[j].first, tol)) {
            ia[k].used = 1, found = 1;
            break;
          }
        }
      }
    same_b[j] = (uint8_t)found;
  }
  // Output: origin of the new drawing, the coarser step of the two.
  g->ox = B.ox, g->oy = B.oy, g->q = fmax(A.q, B.q), g->origin_set = 1;
  t->units = vb.h.units, t->ltscale = 1;
  static const char *names[3] = {"그대로", "새로 생김", "없어짐"};
  static const uint32_t rgb[3] = {0x8C8C8C, 0xFF453A, 0x0A84FF};
  uint32_t style[3];
  for (int c = 0; c < 3; c++) {
    uint64_t lh = tables_new_layer(t, names[c], 0);
    t->layers[t->n_layers - 1].rgb = rgb[c];
    // What changed is drawn bold: one-pixel red and blue lines vanish next to grey.
    RawStyle k = {lh, LT_CONT, (c == CMP_SAME ? COL_TRUE : COL_BOLD) | rgb[c], 1.0f};
    style[c] = style_intern(k);
  }
  t->layer0 = t->layers[0].h;
  st->x0 = st->y0 = INFINITY, st->x1 = st->y1 = -INFINITY;
  // Unchanged first (underneath), then removed, then added on top.
  for (size_t j = 0; j < nB; j++)
    if (same_b[j]) cmp_emit(g, &B, &ib[j], style[CMP_SAME], st, 0), st->same++;
  for (size_t k = 0; k < nA; k++)
    if (!ia[k].used) cmp_emit(g, &A, &ia[k], style[CMP_REMOVED], st, 1), st->removed++;
  for (size_t j = 0; j < nB; j++)
    if (!same_b[j]) cmp_emit(g, &B, &ib[j], style[CMP_ADDED], st, 1), st->added++;
  cmp_zones(st);
  free(st->ch), st->ch = NULL, st->nch = st->cch = 0;
  // The new drawing's dimensions ("도면 숫자" while measuring).
  for (uint32_t d = 0; d < vb.h.n_dims; d++) {
    UmgDim r = vb.dims[d];
    double x1 = CX(&B, r.x1), y1 = CY(&B, r.y1), x2 = CX(&B, r.x2), y2 = CY(&B, r.y2);
    r.x1 = QX(g, x1), r.y1 = QY(g, y1), r.x2 = QX(g, x2), r.y2 = QY(g, y2);
    *(UmgDim *)vec_push(&g->dims) = r;
  }
  free(ia), free(ib), free(same_b);
  return 0;
}

#undef CX
#undef CY

// A whole cache file, decompressed; NULL when unreadable. Caps: 256 MB
// compressed, 2 GB raw.
static void *cmp_load(const char *path, size_t *n) {
  FILE *f = fopen(path, "rb");
  if (!f) return NULL;
  fseek(f, 0, SEEK_END);
  long len = ftell(f);
  fseek(f, 0, SEEK_SET);
  if (len <= 0 || len > (256L << 20)) { fclose(f); return NULL; }
  char *z = (char *)malloc((size_t)len);
  size_t got = z ? fread(z, 1, (size_t)len, f) : 0;
  fclose(f);
  unsigned long long rn = got == (size_t)len ? ZSTD_getFrameContentSize(z, got) : 0;
  if (!z || rn == ZSTD_CONTENTSIZE_UNKNOWN || rn == ZSTD_CONTENTSIZE_ERROR || rn == 0 || rn > (2ull << 30)) {
    free(z);
    return NULL;
  }
  void *raw = malloc((size_t)rn);
  if (!raw || ZSTD_isError(ZSTD_decompress(raw, (size_t)rn, z, got))) { free(z), free(raw); return NULL; }
  free(z);
  *n = (size_t)rn;
  return raw;
}
