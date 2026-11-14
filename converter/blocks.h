// Block store: keeps converted block-definition primitives per
// BLOCK_HEADER handle and expands INSERT / MINSERT / DIMENSION references
// into world-space primitives after decoding. Self-contained block entities
// are converted (and, when streaming, freed) as they are decoded; only their
// primitives are kept.
#pragma once
#include <stdint.h>
#include <string.h>

#include "convert.h"

#define MAX_DEPTH 16

typedef struct {
  uint64_t block;  // BLOCK_HEADER handle
  Xf xf;           // block coordinates (base point already removed) -> parent
  uint32_t style;  // raw style of the INSERT / DIMENSION
  uint64_t handle;  // of the INSERT / DIMENSION (draw order)
  double *clip;    // XCLIP boundary in block coordinates (x, y pairs), or NULL
  uint32_t nclip;
} Ref;

typedef struct {
  Ref *v;
  size_t n, cap;
} Refs;

static Ref *refs_push(Refs *r, uint64_t block, Xf xf, uint32_t style,
                      uint64_t handle) {
  if (r->n == r->cap) {
    r->cap = r->cap ? r->cap * 2 : 16;
    r->v = (Ref *)realloc(r->v, r->cap * sizeof(Ref));
  }
  Ref f = {block, xf, style, handle, NULL, 0};
  r->v[r->n] = f;
  return &r->v[r->n++];
}

static void refs_free(Refs *r) {
  for (size_t k = 0; k < r->n; k++) free(r->v[k].clip);
  free(r->v);
}

typedef struct {
  uint64_t handle;
  double bx, by;  // base point
  Geom geom;      // primitives relative to geom.ox/oy (lazy origin)
  Refs kids;      // nested INSERTs / DIMENSIONs inside this block
} Block;

typedef struct {
  Block *v;
  size_t n, cap;
  int32_t *slot;  // open addressing: handle -> index into v, -1 = empty
  size_t nslot;
  Refs roots;     // model-space references, in decode order
  double q;       // fixed-point step, same as the output Geom
  uint64_t layer0;  // LAYER "0" handle, for style inheritance
  unsigned long n_insert, n_dim, n_missing, n_deep, n_mirrored_text, n_xclip;
  unsigned long n_minsert_capped;  // MINSERT arrays cut to MINSERT_MAX cells
} Blocks;

static size_t slot_of(const Blocks *s, uint64_t h) {
  size_t i = (size_t)(h * 0x9E3779B97F4A7C15ull) & (s->nslot - 1);
  while (s->slot[i] >= 0 && s->v[s->slot[i]].handle != h)
    i = (i + 1) & (s->nslot - 1);
  return i;
}

static Block *block_find(const Blocks *s, uint64_t h) {
  if (!s->nslot) return NULL;
  int32_t k = s->slot[slot_of(s, h)];
  return k < 0 ? NULL : &s->v[k];
}

static Block *block_get(Blocks *s, uint64_t h) {
  if ((s->n + 1) * 2 > s->nslot) {  // keep load factor <= 0.5
    size_t ns = s->nslot ? s->nslot * 2 : 1024;
    free(s->slot);
    s->slot = (int32_t *)malloc(ns * sizeof(int32_t));
    memset(s->slot, 0xff, ns * sizeof(int32_t));
    s->nslot = ns;
    for (size_t k = 0; k < s->n; k++) s->slot[slot_of(s, s->v[k].handle)] = k;
  }
  size_t i = slot_of(s, h);
  if (s->slot[i] >= 0) return &s->v[s->slot[i]];
  if (s->n == s->cap) {
    s->cap = s->cap ? s->cap * 2 : 256;
    s->v = (Block *)realloc(s->v, s->cap * sizeof(Block));
  }
  Block *b = &s->v[s->n];
  memset(b, 0, sizeof *b);
  b->handle = h;
  geom_init(&b->geom, s->q);
  b->geom.lazy_origin = 1;
  s->slot[i] = s->n++;
  return b;
}

static int is_dimension(const Dwg_Object *o) {
  switch (o->fixedtype) {
  case DWG_TYPE_DIMENSION_LINEAR: case DWG_TYPE_DIMENSION_ALIGNED:
  case DWG_TYPE_DIMENSION_RADIUS: case DWG_TYPE_DIMENSION_DIAMETER:
  case DWG_TYPE_DIMENSION_ANG2LN: case DWG_TYPE_DIMENSION_ANG3PT:
  case DWG_TYPE_DIMENSION_ORDINATE:
    return 1;
  default:
    return 0;
  }
}

// Where a reference found in entity e goes: model space roots, or the kids of
// its owning block. Paper space is dropped.
static Refs *refs_for(Blocks *s, const Dwg_Object_Entity *e) {
  if (e->entmode == 2) return &s->roots;
  if (e->entmode == 1) return NULL;
  return &block_get(s, ref_handle(e->ownerhandle))->kids;
}

// DIMENSION geometry lives in an anonymous *D block already in world
// coordinates, so it is placed with the identity transform.
// Without a *D block the dimension is drawn from its own data instead
// (add_dimension), into model space or its owning block.
static void blocks_add_dimension(Blocks *s, Geom *ms, const Dwg_Object *o) {
  Refs *r = refs_for(s, o->tio.entity);
  uint64_t h = ref_handle(o->tio.entity->tio.DIMENSION_LINEAR->block);
#ifdef UNIMOG_DIM_SELF  // test build: draw angular dimensions from their data
  if (o->fixedtype == DWG_TYPE_DIMENSION_ANG3PT || o->fixedtype == DWG_TYPE_DIMENSION_ANG2LN) h = 0;
#endif
  if (r && !h) {
    const Dwg_Object_Entity *e = o->tio.entity;
    Geom *g = e->entmode == 2 ? ms : &block_get(s, ref_handle(e->ownerhandle))->geom;
    g->cur = entity_style(e);
    OrdKey k = {g->model ? o->handle.value : 0, g->model ? 0 : o->handle.value};
    g->ord = k;
    add_dimension(g, o);
    return;
  }
  if (r && h)
    refs_push(r, h, XF_ID, entity_style(o->tio.entity), o->handle.value),
        s->n_dim++;
}

// Entry `name` of DICTIONARY d, or NULL.
static Dwg_Object *dict_item(Dwg_Object *d, const char *name) {
  if (!d || d->fixedtype != DWG_TYPE_DICTIONARY) return NULL;
  const Dwg_Object_DICTIONARY *x = d->tio.object->tio.DICTIONARY;
  for (BITCODE_BL i = 0; i < x->numitems && x->texts && x->itemhandles; i++) {
    char *k = dwg_utf8(d, x->texts[i]);
    int hit = k && !strcmp(k, name);
    free(k);
    if (hit) return x->itemhandles[i] ? dwg_ref_object(d->parent, x->itemhandles[i]) : NULL;
  }
  return NULL;
}

// XCLIP of an INSERT: its extension dictionary's ACAD_FILTER / SPATIAL
// SPATIAL_FILTER, when clipping is on. The boundary (two points = a
// rectangle) is in world coordinates as they were when it was clipped;
// the stored inverse of that INSERT transform takes it to block coordinates
// (base point included), so it follows the INSERT when moved later.
// Front / back (3D) clips and inverted clips are not handled.
static double *insert_clip(const Dwg_Object *o, uint32_t *n) {
  const Dwg_Object_Entity *e = o->tio.entity;
  Dwg_Object *x = e->xdicobjhandle ? dwg_ref_object(o->parent, e->xdicobjhandle) : NULL;
  x = dict_item(dict_item(x, "ACAD_FILTER"), "SPATIAL");
  if (!x || x->fixedtype != DWG_TYPE_SPATIAL_FILTER) return NULL;
  const Dwg_Object_SPATIAL_FILTER *f = x->tio.object->tio.SPATIAL_FILTER;
  const double *m = f->inverse_transform;
  if (!f->display_boundary_on || !m || f->num_clip_verts < 2) return NULL;
  uint32_t k = f->num_clip_verts == 2 ? 4 : f->num_clip_verts;
  double *p = (double *)malloc(k * 2 * sizeof(double));
  const BITCODE_2RD *v = f->clip_verts;
  for (uint32_t i = 0; i < k; i++) {
    // Two points: corners of the rectangle, in order.
    double cx = f->num_clip_verts == 2 ? v[i == 1 || i == 2].x : v[i].x;
    double cy = f->num_clip_verts == 2 ? v[i >= 2].y : v[i].y;
    p[2 * i] = m[0] * cx + m[1] * cy + m[3];
    p[2 * i + 1] = m[4] * cx + m[5] * cy + m[7];
  }
  *n = k;
  return p;
}

#define ADD_INSERT(F, cols, rows, csp, rsp)                                   \
  do {                                                                        \
    __typeof__(e->tio.F) in = e->tio.F;                                     \
    uint64_t h = ref_handle(in->block_header);                                \
    if (!h) break;                                                            \
    double cr = cos(in->rotation), sr = sin(in->rotation);                    \
    double mx = in->extrusion.z < 0 ? -1 : 1; /* OCS mirror */                \
    for (int ri = 0; ri < (rows); ri++)                                       \
      for (int ci = 0; ci < (cols); ci++) {                                   \
        double ox = ci * (csp), oy = ri * (rsp); /* array offset, rotated */  \
        double px = in->ins_pt.x + cr * ox - sr * oy;                         \
        double py = in->ins_pt.y + sr * ox + cr * oy;                         \
        Xf xf = {mx * cr * in->scale.x, -mx * sr * in->scale.y,               \
                 sr * in->scale.x, cr * in->scale.y, mx * px, py};            \
        Ref *f_ = refs_push(r, h, xf, st, o->handle.value);                   \
        if ((cols) == 1 && (rows) == 1 && (f_->clip = insert_clip(o, &f_->nclip))) \
          s->n_xclip++;                                                       \
        s->n_insert++;                                                        \
      }                                                                       \
  } while (0)

#define MINSERT_MAX 10000

static void blocks_add_insert(Blocks *s, const Dwg_Object *o) {
  Dwg_Object_Entity *e = o->tio.entity;
  Refs *r = refs_for(s, e);
  if (!r) return;
  uint32_t st = entity_style(e);
  if (o->fixedtype == DWG_TYPE_INSERT)
    ADD_INSERT(INSERT, 1, 1, 0, 0);
  else {
    // A damaged file can ask for 65535 x 65535 copies (fuzzing: 5.9 GB of
    // references, minutes of work); real arrays are small. Keep at most
    // MINSERT_MAX cells, whole rows first.
    const Dwg_Entity_MINSERT *mi = e->tio.MINSERT;
    int nc = mi->num_cols ? mi->num_cols : 1, nr = mi->num_rows ? mi->num_rows : 1;
    if ((long)nc * nr > MINSERT_MAX) {
      if (nc > MINSERT_MAX) nc = MINSERT_MAX;
      nr = MINSERT_MAX / nc;
      s->n_minsert_capped++;
    }
    ADD_INSERT(MINSERT, nc, nr, in->col_spacing, in->row_spacing);
  }
}

static void blocks_add_attrib(Blocks *s, Geom *ms, const Dwg_Object *o) {
  Dwg_Object *ins = dwg_resolve_handle(
      o->parent, ref_handle(o->tio.entity->ownerhandle));
  if (!ins || ins->supertype != DWG_SUPERTYPE_ENTITY) {
    s->n_missing++;
    return;
  }
  Dwg_Object_Entity *ie = ins->tio.entity;
  if (ie->entmode == 2) convert_attrib(ms, o);
  else if (ie->entmode != 1)
    convert_attrib(&block_get(s, ref_handle(ie->ownerhandle))->geom, o);
}

static void blocks_set_base(Blocks *s, const Dwg_Object *o) {
  Dwg_Object_BLOCK_HEADER *bh = o->tio.object->tio.BLOCK_HEADER;
  Block *b = block_get(s, o->handle.value);
  b->bx = bh->base_pt.x, b->by = bh->base_pt.y;
}

// ---- XCLIP ------------------------------------------------------------
// Clip boundaries in force while a block is emitted, in world coordinates;
// a primitive must lie inside all of them (nested clipped INSERTs).

typedef struct {
  int n;
  const double *p[MAX_DEPTH];
  uint32_t np[MAX_DEPTH];
  int convex[MAX_DEPTH];
} Clips;

static int clips_inside(const Clips *c, double x, double y) {
  for (int k = 0; k < c->n; k++)
    if (!ring_contains(c->p[k], 0, c->np[k], x, y)) return 0;
  return 1;
}

// Signed area sign (+1 CCW) when ring p is convex, else 0.
static int ring_convex(const double *p, uint32_t n) {
  int sign = 0;
  for (uint32_t i = 0; i < n; i++) {
    const double *a = p + 2 * i, *b = p + 2 * ((i + 1) % n), *c = p + 2 * ((i + 2) % n);
    double z = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
    if (z == 0) continue;
    if (!sign) sign = z > 0 ? 1 : -1;
    else if ((z > 0) != (sign > 0)) return 0;
  }
  return sign;
}


// Calls emit(ctx, ...) for each part of segment (x1, y1)-(x2, y2) inside
// the clips: cut at every boundary crossing, keep pieces whose middle is in.
static void clips_segment(const Clips *c, double x1, double y1, double x2,
                          double y2, void (*emit)(void *, double, double, double, double),
                          void *ctx) {
  double ts[2 + 4 * 64];
  size_t nt = 0;
  ts[nt++] = 0, ts[nt++] = 1;
  double dx = x2 - x1, dy = y2 - y1;
  for (int k = 0; k < c->n; k++)
    for (uint32_t i = 0; i < c->np[k] && nt < sizeof ts / sizeof *ts; i++) {
      const double *a = c->p[k] + 2 * i, *b = c->p[k] + 2 * ((i + 1) % c->np[k]);
      double ex = b[0] - a[0], ey = b[1] - a[1], den = dx * ey - dy * ex;
      if (den == 0) continue;
      double t = ((a[0] - x1) * ey - (a[1] - y1) * ex) / den;
      double u = ((a[0] - x1) * dy - (a[1] - y1) * dx) / den;
      if (t > 0 && t < 1 && u >= 0 && u <= 1) ts[nt++] = t;
    }
  qsort(ts, nt, sizeof *ts, cmp_double);
  double run = -1;  // start of the current kept piece
  for (size_t i = 0; i + 1 < nt; i++) {
    double m = (ts[i] + ts[i + 1]) / 2;
    int in = ts[i + 1] > ts[i] && clips_inside(c, x1 + m * dx, y1 + m * dy);
    if (in && run < 0) run = ts[i];
    if (!in && run >= 0 && ts[i + 1] > ts[i]) {
      emit(ctx, x1 + run * dx, y1 + run * dy, x1 + ts[i] * dx, y1 + ts[i] * dy);
      run = -1;
    }
  }
  if (run >= 0) emit(ctx, x1 + run * dx, y1 + run * dy, x2, y2);
}

// Clipped pieces shorter than a fixed-point step are dropped (they would
// be stored as dots).
static void emit_line(void *g, double a, double b, double c, double d) {
  if (hypot(c - a, d - b) > ((Geom *)g)->q) add_line((Geom *)g, XF_ID, a, b, c, d);
}

static void emit_hline(void *g, double a, double b, double c, double d) {
  if (hypot(c - a, d - b) > ((Geom *)g)->q) add_hline((Geom *)g, XF_ID, a, b, c, d);
}

// Polyline through world points pts (n, closed or not): kept whole when all
// points are inside, else drawn as clipped line pieces. Returns 1 if whole.
static int clips_path(Geom *out, const Clips *c, const double *pts, size_t n,
                      int closed) {
  size_t in = 0;
  for (size_t i = 0; i < n; i++) in += clips_inside(c, pts[2 * i], pts[2 * i + 1]);
  if (in == n) return 1;
  size_t ns = closed ? n : n - 1;
  for (size_t i = 0; i < ns; i++) {
    const double *a = pts + 2 * i, *b = pts + 2 * ((i + 1) % n);
    clips_segment(c, a[0], a[1], b[0], b[1], emit_line, out);
  }
  return 0;
}

// Points of an arc (local), mapped through m into v.
static void arc_points(Vec *v, Xf m, double cx, double cy, double r, double a0,
                       double sweep) {
  int k = umg_arc_segs(sweep);
  for (int i = 0; i <= k; i++) {
    double a = a0 + sweep * i / k, x = cx + r * cos(a), y = cy + r * sin(a);
    double *o = (double *)vec_push(v);
    o[0] = m.a * x + m.b * y + m.tx, o[1] = m.c * x + m.d * y + m.ty;
  }
}

// Convex polygon (n points in a) cut by convex ring r (orientation sign
// rs), Sutherland-Hodgman; result in b, returns its size.
static size_t poly_cut(const double *a, size_t n, const double *r, uint32_t rn,
                       int rs, double *b, double *tmp) {
  const double *src = a;
  double *dst = tmp;
  size_t m = n;
  for (uint32_t e = 0; e < rn && m; e++) {
    const double *p = r + 2 * e, *q = r + 2 * ((e + 1) % rn);
    double ex = q[0] - p[0], ey = q[1] - p[1];
#define SIDE(v) (rs * (ex * ((v)[1] - p[1]) - ey * ((v)[0] - p[0])))
    size_t k = 0;
    for (size_t i = 0; i < m; i++) {
      const double *u = src + 2 * i, *w = src + 2 * ((i + 1) % m);
      double su = SIDE(u), sw = SIDE(w);
      if (su >= 0) dst[2 * k] = u[0], dst[2 * k + 1] = u[1], k++;
      if ((su >= 0) != (sw >= 0)) {
        double t = su / (su - sw);
        dst[2 * k] = u[0] + t * (w[0] - u[0]), dst[2 * k + 1] = u[1] + t * (w[1] - u[1]);
        k++;
      }
    }
#undef SIDE
    m = k;
    src = dst;
    dst = dst == tmp ? b : tmp;
  }
  if (src != b) memcpy(b, src, m * 2 * sizeof(double));
  return m;
}

// xf maps this block's coordinates, after removing its base point, to world;
// style is the (already composed) raw style of the reference placing it.
// Draw order: root is the model-space reference; below the first level
// every primitive takes the first-level entity `inner`. clip / nclip: this
// reference's XCLIP in block coordinates; cin: clips of the references
// around it.
static void emit_block(Blocks *s, Geom *out, uint64_t h, Xf xf, uint32_t style,
                       int depth, uint64_t root, uint64_t inner,
                       const double *clip, uint32_t nclip, const Clips *cin) {
  Block *b = block_find(s, h);
  if (!b) { s->n_missing++; return; }
  if (depth >= MAX_DEPTH) { s->n_deep++; return; }
  Xf base = {1, 0, 0, 1, -b->bx, -b->by};
  Xf m = xf_mul(xf, base);
  const Geom *g = &b->geom;
  Clips cs = *cin;
  double *own = NULL;
  if (nclip && cs.n < MAX_DEPTH) {
    own = (double *)malloc(nclip * 2 * sizeof(double));
    for (uint32_t i = 0; i < nclip; i++) {  // block coordinates, like g's
      own[2 * i] = m.a * clip[2 * i] + m.b * clip[2 * i + 1] + m.tx;
      own[2 * i + 1] = m.c * clip[2 * i] + m.d * clip[2 * i + 1] + m.ty;
    }
    cs.p[cs.n] = own, cs.np[cs.n] = nclip, cs.convex[cs.n] = ring_convex(own, nclip);
    cs.n++;
  }
  const int clipped = cs.n > 0;
#define STYLE(vec, i) \
  (out->cur = style_compose(((const uint32_t *)g->vec.p)[i], style, s->layer0, m))
#define ORD(keys, i)                                                      \
  do {                                                                    \
    OrdKey k_ = {root, depth ? inner : ((const OrdKey *)g->keys.p)[i].inner}; \
    out->ord = k_;                                                        \
  } while (0)
#define MX(x, y) (m.a * (x) + m.b * (y) + m.tx)
#define MY(x, y) (m.c * (x) + m.d * (y) + m.ty)
  Vec pts = {NULL, 0, 0, 2 * sizeof(double)};
  const UmgLine *l = (const UmgLine *)g->lines.p;
  for (size_t i = 0; i < g->lines.n; i++, l++) {
    STYLE(line_st, i);
    double x1 = WX(g, l->x1), y1 = WY(g, l->y1), x2 = WX(g, l->x2), y2 = WY(g, l->y2);
    if (clipped)
      clips_segment(&cs, MX(x1, y1), MY(x1, y1), MX(x2, y2), MY(x2, y2), emit_line, out);
    else
      add_line(out, m, x1, y1, x2, y2);
  }
  l = (const UmgLine *)g->hlines.p;
  const double lod_scale = sqrt(fabs(xf_det(m)));
  for (size_t i = 0; i < g->hlines.n; i++, l++) {
    STYLE(hline_st, i);
    ORD(hline_key, i);
    out->lod = (float)(((const float *)g->hline_lod.p)[i] * lod_scale);
    double x1 = WX(g, l->x1), y1 = WY(g, l->y1), x2 = WX(g, l->x2), y2 = WY(g, l->y2);
    if (!clipped)
      add_hline(out, m, x1, y1, x2, y2);
    else if (x1 == x2 && y1 == y2) {  // dot
      if (clips_inside(&cs, MX(x1, y1), MY(x1, y1))) add_hline(out, m, x1, y1, x2, y2);
    } else
      clips_segment(&cs, MX(x1, y1), MY(x1, y1), MX(x2, y2), MY(x2, y2), emit_hline, out);
  }
  out->lod = INFINITY;
  const UmgArc *a = (const UmgArc *)g->arcs.p;
  for (size_t i = 0; i < g->arcs.n; i++, a++) {
    STYLE(arc_st, i);
    double cx = WX(g, a->cx), cy = WY(g, a->cy), r = a->r * g->q;
    if (clipped) {
      double sw = a->a1 - a->a0;
      if (sw <= 0) sw += 2 * M_PI;
      pts.n = 0;
      arc_points(&pts, m, cx, cy, r, a->a0, sw);
      if (!clips_path(out, &cs, (const double *)pts.p, pts.n, 0)) continue;
    }
    add_arc(out, m, cx, cy, r, a->a0, a->a1);
  }
  for (size_t i = 0, p0 = 0; i < g->polys.n; i++) {
    STYLE(poly_st, i);
    uint32_t head = ((const uint32_t *)g->polys.p)[i], cnt = head & UMG_COUNT;
    int whole = 1;
    if (clipped && cnt >= 2) {
      const UmgPt *v = (const UmgPt *)g->pts.p + p0;
      int closed = (head & UMG_CLOSED) != 0;
      uint32_t ns = closed ? cnt : cnt - 1;
      pts.n = 0;
      for (uint32_t j = 0; j < ns; j++) {
        const UmgPt *p = v + j, *q = v + (j + 1) % cnt;
        double px = WX(g, p->x), py = WY(g, p->y), qx = WX(g, q->x), qy = WY(g, q->y);
        if (p->bulge != 0 && (px != qx || py != qy)) {
          double ccx, ccy, rr, a0, sw;
          umg_bulge_arc(px, py, qx, qy, p->bulge, &ccx, &ccy, &rr, &a0, &sw);
          arc_points(&pts, m, ccx, ccy, rr, a0, sw);
          pts.n--;  // its end is the next segment's start
        } else {
          double *o = (double *)vec_push(&pts);
          o[0] = MX(px, py), o[1] = MY(px, py);
        }
      }
      if (!closed) {
        double *o = (double *)vec_push(&pts);
        o[0] = MX(WX(g, v[cnt - 1].x), WY(g, v[cnt - 1].y));
        o[1] = MY(WX(g, v[cnt - 1].x), WY(g, v[cnt - 1].y));
      }
      whole = clips_path(out, &cs, (const double *)pts.p, pts.n, closed);
    }
    if (whole) copy_poly(out, m, g, i, p0);
    p0 += cnt;
  }
  for (size_t i = 0, f0 = 0; i < g->fills.n; i++) {
    STYLE(fill_st, i);
    ORD(fill_key, i);
    uint32_t nv = ((const uint32_t *)g->fills.p)[i];
    if (!clipped) {
      copy_fill(out, m, g, i, f0);
    } else {  // triangle by triangle
      const UmgFPt *v = (const UmgFPt *)g->fpts.p + f0;
      Vec xy = {NULL, 0, 0, 2 * sizeof(double)}, tri = {NULL, 0, 0, sizeof(uint32_t)};
      double t[2 * 64], u[2 * 64], w[2 * 64];
      for (uint32_t k = 0; k + 2 < nv; k += 3) {
        size_t n = 3;
        for (int j = 0; j < 3; j++) {
          double x = WX(g, v[k + j].x), y = WY(g, v[k + j].y);
          t[2 * j] = MX(x, y), t[2 * j + 1] = MY(x, y);
        }
        for (int c = 0; c < cs.n && n; c++) {
          if (cs.convex[c] && cs.np[c] <= 16) {
            n = poly_cut(t, n, cs.p[c], cs.np[c], cs.convex[c], u, w);
            memcpy(t, u, n * 2 * sizeof(double));
          } else {  // concave boundary: keep or drop by the centroid
            double mx = 0, my = 0;
            for (size_t j = 0; j < n; j++) mx += t[2 * j], my += t[2 * j + 1];
            if (!ring_contains(cs.p[c], 0, cs.np[c], mx / n, my / n)) n = 0;
          }
        }
        uint32_t b0 = (uint32_t)xy.n;
        for (size_t j = 0; n >= 3 && j < n; j++) {
          double *o = (double *)vec_push(&xy);
          o[0] = t[2 * j], o[1] = t[2 * j + 1];
        }
        for (size_t j = 1; n >= 3 && j + 1 < n; j++) {  // fan
          uint32_t f[3] = {b0, b0 + (uint32_t)j, b0 + (uint32_t)j + 1};
          for (int z = 0; z < 3; z++) *(uint32_t *)vec_push(&tri) = f[z];
        }
      }
      add_fill(out, XF_ID, (const double *)xy.p, (const uint32_t *)tri.p, tri.n);
      free(xy.p), free(tri.p);
    }
    f0 += nv;
  }
  // Text: anchor points go through m; rotation, height and width factor
  // pick up m's rotation and axis scales. Mirrored inserts keep text
  // readable (spike: not AutoCAD's MIRRTEXT behaviour). Clipped: kept when
  // the anchor is inside.
  double rot = atan2(m.c, m.a), sx = hypot(m.a, m.c), sy = hypot(m.b, m.d);
  if (xf_det(m) < 0) s->n_mirrored_text += g->texts.n;
  const UmgText *t = (const UmgText *)g->texts.p;
  for (size_t i = 0; i < g->texts.n; i++) {
    STYLE(text_st, i);
    UmgText r = t[i];
    double x = WX(g, r.x), y = WY(g, r.y), x2 = WX(g, r.x2), y2 = WY(g, r.y2);
    if (clipped && !clips_inside(&cs, MX(x, y), MY(x, y))) continue;
    r.rot += (float)rot, r.h *= (float)sy, r.wf *= (float)(sx / sy);
    r.width *= (float)sx;
    push_text(out, r, MX(x, y), MY(x, y), MX(x2, y2), MY(x2, y2),
              (const char *)g->pool.p + r.off, r.len);
  }
  free(pts.p);
  for (size_t k = 0; k < b->kids.n; k++) {
    const Ref *f = &b->kids.v[k];
    // The nested reference is itself an entity of this block. Its pattern
    // scale is applied by the full transform when its primitives are emitted.
    uint32_t st = style_compose(f->style, style, s->layer0, XF_ID);
    emit_block(s, out, f->block, xf_mul(m, f->xf), st, depth + 1, root,
               depth ? inner : f->handle, f->clip, f->nclip, &cs);
  }
  free(own);
#undef STYLE
#undef ORD
#undef MX
#undef MY
}

static void blocks_expand(Blocks *s, Geom *out) {
  Clips none = {0};
  for (size_t k = 0; k < s->roots.n; k++) {
    const Ref *f = &s->roots.v[k];
    emit_block(s, out, f->block, f->xf, f->style, 0, f->handle, 0, f->clip,
               f->nclip, &none);
  }
}

static size_t geom_bytes(const Geom *g) {
  return g->lines.cap * g->lines.elem + g->arcs.cap * g->arcs.elem +
         g->polys.cap * g->polys.elem + g->pts.cap * g->pts.elem +
         g->texts.cap * g->texts.elem + g->pool.cap +
         g->fills.cap * g->fills.elem + g->fpts.cap * g->fpts.elem +
         g->hlines.cap * g->hlines.elem;
}

static size_t blocks_bytes(const Blocks *s) {
  size_t t = s->cap * sizeof(Block) + s->nslot * sizeof(int32_t) +
             s->roots.cap * sizeof(Ref);
  for (size_t k = 0; k < s->n; k++)
    t += geom_bytes(&s->v[k].geom) + s->v[k].kids.cap * sizeof(Ref);
  return t;
}

static void blocks_free(Blocks *s) {
  for (size_t k = 0; k < s->n; k++)
    geom_free(&s->v[k].geom), refs_free(&s->v[k].kids);
  free(s->v), free(s->slot), refs_free(&s->roots);
}
