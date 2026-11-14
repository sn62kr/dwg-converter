// Converter core: one decoded entity -> exact Unimog primitives (lines,
// arcs, polylines with bulges, plain text, filled triangles) in UMG8 fixed
// point, each tagged
// with the entity's raw style (layer, colour, linetype; resolved at write). Used
// by the baseline and streaming benches and by block expansion, so every
// path produces the same bytes. Splines and ellipses are tessellated here (no
// exact form kept); arcs and bulges stay exact and are tessellated at load
// time (tess.h).
#pragma once
#include <dwg.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/resource.h>
#include <time.h>

#include "tess.h"

static double now_s(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec / 1e9;
}

static double peak_mb(void) {
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
#ifdef __APPLE__
  return ru.ru_maxrss / (1024.0 * 1024.0);
#else
  return ru.ru_maxrss / 1024.0;
#endif
}

typedef struct {
  void *p;
  size_t n, cap, elem;
} Vec;

static void vec_reserve(Vec *v, size_t n) {
  if (v->n + n <= v->cap) return;
  while (v->n + n > v->cap) v->cap = v->cap ? v->cap * 2 : 16;
  v->p = realloc(v->p, v->cap * v->elem);
}

static void *vec_push(Vec *v) {
  vec_reserve(v, 1);
  return (char *)v->p + v->elem * v->n++;
}

static void vec_append(Vec *v, const void *src, size_t n) {
  if (!n) return;
  vec_reserve(v, n);
  memcpy((char *)v->p + v->elem * v->n, src, n * v->elem);
  v->n += n;
}

// Affine 2D transform: x' = a*x + b*y + tx, y' = c*x + d*y + ty
typedef struct { double a, b, c, d, tx, ty; } Xf;

static const Xf XF_ID = {1, 0, 0, 1, 0, 0};
static const Xf XF_MIRROR_X = {-1, 0, 0, 1, 0, 0};  // OCS (0,0,-1) -> WCS

static Xf xf_mul(Xf p, Xf q) {  // p after q
  Xf r = {p.a * q.a + p.b * q.c, p.a * q.b + p.b * q.d,
          p.c * q.a + p.d * q.c, p.c * q.b + p.d * q.d,
          p.a * q.tx + p.b * q.ty + p.tx, p.c * q.tx + p.d * q.ty + p.ty};
  return r;
}

static double xf_det(Xf m) { return m.a * m.d - m.b * m.c; }

// Similarity (rotation + uniform scale, optionally mirrored): arcs stay arcs.
static int xf_similar(Xf m) {
  double sx = hypot(m.a, m.c), sy = hypot(m.b, m.d);
  return fabs(sx - sy) <= 1e-9 * (sx + sy) &&
         fabs(m.a * m.b + m.c * m.d) <= 1e-9 * sx * sy;
}

// ---- raw styles -----------------------------------------------------------

// An entity's drawing properties as stored, before ByLayer / ByBlock are
// resolved (that needs the layer table and, inside blocks, the INSERT).
// Colours: COL_BYLAYER, COL_BYBLOCK, COL_ACI | index, COL_TRUE | rgb.
#define COL_BYLAYER 0u
#define COL_TRUE 0x01000000u
#define COL_ACI 0x02000000u
#define COL_BYBLOCK 0x03000000u
#define COL_BG 0x04000000u  // WIPEOUT: background colour
// Linetypes: LT_BYLAYER, LT_BYBLOCK, LT_CONT, else the LTYPE handle.
#define LT_BYLAYER 0u
#define LT_BYBLOCK 1u
#define LT_CONT 2u

typedef struct {
  uint64_t layer, ltype;  // LAYER handle, LT_* or LTYPE handle
  uint32_t color;         // COL_*
  float scale;            // entity linetype scale (times block scale)
} RawStyle;

static struct {
  RawStyle *v;
  size_t n, cap;
  int32_t *slot;  // open addressing over v, -1 = empty
  size_t nslot;
} g_styles;

static uint64_t style_hash(const RawStyle *k) {
  uint64_t h = k->layer * 0x9E3779B97F4A7C15ull ^ k->ltype * 0xC2B2AE3D27D4EB4Full;
  uint32_t sc;
  memcpy(&sc, &k->scale, 4);
  return (h ^ ((uint64_t)k->color << 32 | sc)) * 0x165667B19E3779F9ull;
}

static size_t style_slot(const RawStyle *k) {
  size_t i = (size_t)(style_hash(k) >> 17) & (g_styles.nslot - 1);
  while (g_styles.slot[i] >= 0 &&
         memcmp(&g_styles.v[g_styles.slot[i]], k, sizeof *k) != 0)
    i = (i + 1) & (g_styles.nslot - 1);
  return i;
}

// Id of k in the global raw style table, adding it if new.
static uint32_t style_intern(RawStyle k) {
  if ((g_styles.n + 1) * 2 > g_styles.nslot) {
    size_t ns = g_styles.nslot ? g_styles.nslot * 2 : 256;
    free(g_styles.slot);
    g_styles.slot = (int32_t *)malloc(ns * sizeof(int32_t));
    memset(g_styles.slot, 0xff, ns * sizeof(int32_t));
    g_styles.nslot = ns;
    for (size_t j = 0; j < g_styles.n; j++)
      g_styles.slot[style_slot(&g_styles.v[j])] = (int32_t)j;
  }
  size_t i = style_slot(&k);
  if (g_styles.slot[i] >= 0) return (uint32_t)g_styles.slot[i];
  if (g_styles.n == g_styles.cap) {
    g_styles.cap = g_styles.cap ? g_styles.cap * 2 : 256;
    g_styles.v = (RawStyle *)realloc(g_styles.v, g_styles.cap * sizeof(RawStyle));
  }
  g_styles.v[g_styles.n] = k;
  g_styles.slot[i] = (int32_t)g_styles.n;
  return (uint32_t)g_styles.n++;
}

static uint64_t ref_handle(BITCODE_H r) { return r ? r->absolute_ref : 0; }

// R2004+ colours carry a method byte: 0xc0 ByLayer, 0xc1 ByBlock, 0xc2 true
// colour, 0xc3 colour index in the low byte. Older files have the index only
// (negative = layer off).
static uint32_t raw_color(const Dwg_Color *c, int has_rgb) {
  if (has_rgb) {
    switch (c->rgb >> 24) {
    case 0xc0: return COL_BYLAYER;
    case 0xc1: return COL_BYBLOCK;
    case 0xc2: return COL_TRUE | (c->rgb & 0xffffff);
    case 0xc3: return COL_ACI | (c->rgb & 0xff);
    }
  }
  int i = c->index < 0 ? -c->index : c->index;
  return i == 0 ? COL_BYBLOCK : i >= 256 ? COL_BYLAYER : COL_ACI | (uint32_t)i;
}

static uint32_t entity_style(const Dwg_Object_Entity *e) {
  RawStyle k = {ref_handle(e->layer), LT_BYLAYER, 0, 1};
  k.color = raw_color(&e->color, (e->color.flag & 0x80) != 0);
  switch (e->ltype_flags) {
  case 1: k.ltype = LT_BYBLOCK; break;
  case 2: k.ltype = LT_CONT; break;
  case 3: k.ltype = ref_handle(e->ltype); break;
  default:  // R13/R14 store an explicit handle unless isbylayerlt
    if (e->dwg && e->dwg->header.from_version < R_2000 && !e->isbylayerlt)
      k.ltype = ref_handle(e->ltype);
  }
  if (k.ltype == 0 && e->ltype_flags == 3) k.ltype = LT_BYLAYER;
  k.scale = (float)(e->ltype_scale > 0 ? e->ltype_scale : 1);
  return style_intern(k);
}

// Style of an entity inside a block placed by a reference with style
// `outer` through m: layer "0" and ByBlock take the reference's values, and
// the linetype pattern scales with the block.
static uint32_t style_compose(uint32_t inner, uint32_t outer, uint64_t layer0,
                              Xf m) {
  RawStyle k = g_styles.v[inner], o = g_styles.v[outer];
  if (k.layer == layer0 || k.layer == 0) k.layer = o.layer;
  if (k.color == COL_BYBLOCK) k.color = o.color;
  if (k.ltype == LT_BYBLOCK) k.ltype = o.ltype, k.scale *= o.scale;
  k.scale *= (float)sqrt(fabs(xf_det(m)));
  return style_intern(k);
}

// Draw order of a fill or hatch line: the model-space entity it comes from
// (`root`: its own handle, or the INSERT's) and, inside a block, the
// first-level block entity (`inner`). Both go through the SORTENTSTABLE
// map; the image stores the rank.
typedef struct { uint64_t root, inner; } OrdKey;

// Primitives as fixed point relative to (ox, oy) in steps of q.
typedef struct {
  double ox, oy, q;
  int lazy_origin, origin_set;  // block geometry: origin = first point added
  Vec lines;  // UmgLine
  Vec arcs;   // UmgArc
  Vec polys;  // uint32: point count | UMG_CLOSED
  Vec pts;    // UmgPt
  Vec texts;  // UmgText
  Vec fills;  // uint32: triangle vertex count
  Vec fpts;   // UmgFPt
  Vec hlines;  // UmgLine: hatch pattern lines (drawn, not snapped)
  Vec fill_key, hline_key;  // OrdKey per fill, per hline
  Vec hline_lod;  // float per hline: hidden when its lines get this close
  float lod;      // ... for the hlines being added (INFINITY = always drawn)
  int model;                // model space (else a block definition)
  OrdKey ord;               // key of the area primitives being added
  Vec pool;   // char
  Vec line_st, arc_st, poly_st, text_st, fill_st, hline_st;  // uint32 style
  uint32_t cur;  // raw style of the primitives being added
  unsigned long n_skipped, n_text_bad, n_clamped;
} Geom;

static void geom_init(Geom *g, double q) {
  memset(g, 0, sizeof *g);
  g->q = q;
  g->lines.elem = sizeof(UmgLine), g->arcs.elem = sizeof(UmgArc);
  g->polys.elem = 4, g->pts.elem = sizeof(UmgPt);
  g->texts.elem = sizeof(UmgText), g->pool.elem = 1;
  g->fills.elem = 4, g->fpts.elem = sizeof(UmgFPt);
  g->line_st.elem = g->arc_st.elem = g->poly_st.elem = g->text_st.elem = 4;
  g->fill_st.elem = g->hline_st.elem = 4;
  g->hlines.elem = sizeof(UmgLine);
  g->fill_key.elem = g->hline_key.elem = sizeof(OrdKey);
  g->hline_lod.elem = sizeof(float);
  g->lod = INFINITY;
}

// Quantizes a length or an origin-relative coordinate to steps of q.
static int32_t qz(Geom *g, double v) {
  double t = floor(v / g->q + 0.5);
  if (t != t) {  // NaN from a damaged file
    g->n_clamped++;
    return 0;
  }
  if (t > INT32_MAX || t < INT32_MIN) {
    g->n_clamped++;
    return t > 0 ? INT32_MAX : INT32_MIN;
  }
  return (int32_t)t;
}

#define QX(g, X) qz(g, (X) - (g)->ox)
#define QY(g, Y) qz(g, (Y) - (g)->oy)
// World coordinates back from a Geom's fixed point.
#define WX(g, v) ((g)->ox + (v) * (g)->q)
#define WY(g, v) ((g)->oy + (v) * (g)->q)

static void geom_free(Geom *g) {
  free(g->lines.p), free(g->arcs.p), free(g->polys.p), free(g->pts.p);
  free(g->texts.p), free(g->pool.p), free(g->fills.p), free(g->fpts.p);
  free(g->line_st.p), free(g->arc_st.p), free(g->poly_st.p), free(g->text_st.p);
  free(g->fill_st.p), free(g->hlines.p), free(g->hline_st.p);
  free(g->fill_key.p), free(g->hline_key.p), free(g->hline_lod.p);
}

static void geom_origin(Geom *g, double x, double y) {
  if (g->lazy_origin && !g->origin_set) g->ox = x, g->oy = y;
  g->origin_set = 1;
}

static void push_line(Geom *g, Vec *lines, Vec *st, Xf m, double x1,
                      double y1, double x2, double y2) {
  double X1 = m.a * x1 + m.b * y1 + m.tx, Y1 = m.c * x1 + m.d * y1 + m.ty;
  double X2 = m.a * x2 + m.b * y2 + m.tx, Y2 = m.c * x2 + m.d * y2 + m.ty;
  if (!g->origin_set) geom_origin(g, X1, Y1);
  UmgLine *l = (UmgLine *)vec_push(lines);
  l->x1 = QX(g, X1), l->y1 = QY(g, Y1), l->x2 = QX(g, X2), l->y2 = QY(g, Y2);
  *(uint32_t *)vec_push(st) = g->cur;
}

static void add_line(Geom *g, Xf m, double x1, double y1, double x2,
                     double y2) {
  push_line(g, &g->lines, &g->line_st, m, x1, y1, x2, y2);
}

// Hatch pattern line: drawn like a line, but kept apart so measurement does
// not snap to it (AutoCAD's OSNAPHATCH off).
static void add_hline(Geom *g, Xf m, double x1, double y1, double x2,
                      double y2) {
  push_line(g, &g->hlines, &g->hline_st, m, x1, y1, x2, y2);
  *(OrdKey *)vec_push(&g->hline_key) = g->ord;
  *(float *)vec_push(&g->hline_lod) = g->lod;
}

// Level of detail for the k-th of evenly spaced items `step` apart (hatch
// lines across a family, pattern repeats along a line): kept while items
// 2^j apart stay visible, j = trailing zero bits of k, so zooming out halves
// the count step by step and what stays is still even. k = 0 always stays.
static float lod_key(double step, double k) {
  if (!(fabs(k) < 9e15)) return (float)fabs(step);  // NaN or past exact integers
  long long n = llabs((long long)k);
  if (!n) return INFINITY;
  int j = 0;
  while (!(n & 1) && j < 40) n >>= 1, j++;
  return (float)(fabs(step) * ldexp(1, j));
}

// Polyline: begin_poly, then add_pt per vertex. Bulge is tan(sweep / 4) of
// the segment leaving the vertex, positive = counter-clockwise; a mirroring
// transform flips its sign. flags: UMG_CLOSED, UMG_LTCONT.
static void begin_poly(Geom *g, uint32_t n, uint32_t flags) {
  *(uint32_t *)vec_push(&g->polys) = n | flags;
  *(uint32_t *)vec_push(&g->poly_st) = g->cur;
}

static void add_pt(Geom *g, Xf m, double x, double y, double bulge) {
  double X = m.a * x + m.b * y + m.tx, Y = m.c * x + m.d * y + m.ty;
  if (!g->origin_set) geom_origin(g, X, Y);
  UmgPt *p = (UmgPt *)vec_push(&g->pts);
  p->x = QX(g, X), p->y = QY(g, Y);
  p->bulge = (float)(xf_det(m) < 0 ? -bulge : bulge);
}

// Arc a0 -> a1 counter-clockwise. Under a similarity it stays an arc (angles
// re-derived from the mapped end points; a mirror reverses the direction);
// otherwise it becomes an elliptic arc and is tessellated into a polyline.
static void add_arc(Geom *g, Xf m, double cx, double cy, double r, double a0,
                    double a1) {
  if (a1 < a0) a1 += 2 * M_PI;
  int full = a1 - a0 >= 2 * M_PI - 1e-12;
  if (xf_similar(m)) {
    double X = m.a * cx + m.b * cy + m.tx, Y = m.c * cx + m.d * cy + m.ty;
    double b0 = 0, b1 = 2 * M_PI;
    if (!full) {
      double p0x = cx + r * cos(a0), p0y = cy + r * sin(a0);
      double p1x = cx + r * cos(a1), p1y = cy + r * sin(a1);
      b0 = atan2(m.c * p0x + m.d * p0y + m.ty - Y,
                 m.a * p0x + m.b * p0y + m.tx - X);
      b1 = atan2(m.c * p1x + m.d * p1y + m.ty - Y,
                 m.a * p1x + m.b * p1y + m.tx - X);
      if (xf_det(m) < 0) {
        double t = b0;
        b0 = b1, b1 = t;
      }
    }
    if (!g->origin_set) geom_origin(g, X, Y);
    UmgArc *f = (UmgArc *)vec_push(&g->arcs);
    f->cx = QX(g, X), f->cy = QY(g, Y), f->r = qz(g, r * hypot(m.a, m.c));
    f->a0 = (float)b0, f->a1 = (float)b1;
    *(uint32_t *)vec_push(&g->arc_st) = g->cur;
    return;
  }
  int n = umg_arc_segs(a1 - a0);
  begin_poly(g, full ? n : n + 1, (full ? UMG_CLOSED : 0) | UMG_LTCONT);
  for (int i = 0; i < (full ? n : n + 1); i++) {
    double a = a0 + (a1 - a0) * i / n;
    add_pt(g, m, cx + r * cos(a), cy + r * sin(a), 0);
  }
}

static int bulge_is_arc(const double *xy, uint32_t i, uint32_t j, double b) {
  return b != 0 && (xy[2 * i] != xy[2 * j] || xy[2 * i + 1] != xy[2 * j + 1]);
}

// Polyline vertices with bulges, through m. A non-similar transform turns
// bulged segments into elliptic arcs, so those are tessellated first (and
// the linetype then runs on across the extra vertices).
static void add_bulged_poly(Geom *g, Xf m, const double *xy, const double *bul,
                            uint32_t n, uint32_t flags) {
  int similar = xf_similar(m), closed = (flags & UMG_CLOSED) != 0;
  uint32_t nseg = closed ? n : n - 1, total = n;
  if (!similar)
    for (uint32_t i = 0; i < nseg; i++)
      if (bulge_is_arc(xy, i, (i + 1) % n, bul[i]))
        total += umg_arc_segs(4 * atan(bul[i])) - 1;
  if (total != n) flags |= UMG_LTCONT;
  begin_poly(g, total, flags);
  for (uint32_t i = 0; i < n; i++) {
    uint32_t j = (i + 1) % n;
    double x = xy[2 * i], y = xy[2 * i + 1];
    if (i >= nseg || !bulge_is_arc(xy, i, j, bul[i])) {
      add_pt(g, m, x, y, 0);
    } else if (similar) {
      add_pt(g, m, x, y, bul[i]);
    } else {
      double cx, cy, r, a0, sweep;
      umg_bulge_arc(x, y, xy[2 * j], xy[2 * j + 1], bul[i], &cx, &cy, &r, &a0,
                    &sweep);
      int k = umg_arc_segs(sweep);
      for (int s = 0; s < k; s++) {
        double a = a0 + sweep * s / k;
        add_pt(g, m, cx + r * cos(a), cy + r * sin(a), 0);
      }
    }
  }
}

// Copies polyline `pi` (first point index `p0`) of src into g through m.
static void copy_poly(Geom *g, Xf m, const Geom *src, size_t pi, size_t p0) {
  uint32_t head = ((const uint32_t *)src->polys.p)[pi];
  uint32_t n = head & UMG_COUNT;
  const UmgPt *pts = (const UmgPt *)src->pts.p + p0;
  double *xy = (double *)malloc(n * 2 * sizeof(double));
  double *bul = (double *)malloc(n * sizeof(double));
  for (uint32_t i = 0; i < n; i++) {
    xy[2 * i] = WX(src, pts[i].x), xy[2 * i + 1] = WY(src, pts[i].y);
    bul[i] = pts[i].bulge;
  }
  add_bulged_poly(g, m, xy, bul, n, head & (UMG_CLOSED | UMG_LTCONT));
  free(xy), free(bul);
}

// ---- splines --------------------------------------------------------------

// Rational B-spline point at parameter u (de Boor, homogeneous).
static void spline_eval(const Dwg_Entity_SPLINE *s, double u, double *x,
                        double *y) {
  int p = s->degree, nc = (int)s->num_ctrl_pts;
  const double *U = s->knots;
  int k = p;
  while (k < nc - 1 && u >= U[k + 1]) k++;
  double dx[16], dy[16], dw[16];
  for (int j = 0; j <= p; j++) {
    const Dwg_SPLINE_control_point *c = &s->ctrl_pts[k - p + j];
    double w = s->rational && c->w > 0 ? c->w : 1;
    dx[j] = c->x * w, dy[j] = c->y * w, dw[j] = w;
  }
  for (int r = 1; r <= p; r++)
    for (int j = p; j >= r; j--) {
      int i = k - p + j;
      double den = U[i + p - r + 1] - U[i];
      double a = den > 0 ? (u - U[i]) / den : 0;
      dx[j] = (1 - a) * dx[j - 1] + a * dx[j];
      dy[j] = (1 - a) * dy[j - 1] + a * dy[j];
      dw[j] = (1 - a) * dw[j - 1] + a * dw[j];
    }
  *x = dx[p] / dw[p], *y = dy[p] / dw[p];
}

// Adaptive sampling: each knot span is split until the curve midpoint lies
// within tol of the chord (tol = 0.5% of the control polygon size, about
// the sagitta of a 32-segment circle, so splines look as smooth as arcs).
#define SPLINE_MAX_PTS 4096

static void add_spline(Geom *g, const Dwg_Entity_SPLINE *s) {
  int p = s->degree, nc = (int)s->num_ctrl_pts;
  if (nc > p && p >= 1 && p < 16 && s->knots &&
      s->num_knots == (BITCODE_BL)(nc + p + 1)) {
    double minx = INFINITY, miny = INFINITY, maxx = -INFINITY, maxy = -INFINITY;
    for (int i = 0; i < nc; i++) {
      minx = fmin(minx, s->ctrl_pts[i].x), maxx = fmax(maxx, s->ctrl_pts[i].x);
      miny = fmin(miny, s->ctrl_pts[i].y), maxy = fmax(maxy, s->ctrl_pts[i].y);
    }
    double tol = 0.005 * hypot(maxx - minx, maxy - miny);
    static double xs[SPLINE_MAX_PTS], ys[SPLINE_MAX_PTS];
    spline_eval(s, s->knots[p], &xs[0], &ys[0]);
    int n = 1;
    for (int k = p; k < nc; k++) {  // knot spans [U[k], U[k+1])
      double ua = s->knots[k], ub = s->knots[k + 1];
      if (ub <= ua) continue;
      // Depth-first bisection of [ua, ub]; the stack holds pending right
      // ends with their evaluated points, so each split costs one eval.
      double su[32], sx[32], sy[32];
      int sd[32], top = 0;
      su[0] = ub, sd[0] = 0, top = 1;
      spline_eval(s, ub, &sx[0], &sy[0]);
      double u0 = ua;
      while (top) {
        int t = top - 1;
        double x0 = xs[n - 1], y0 = ys[n - 1], xm, ym, um = (u0 + su[t]) / 2;
        spline_eval(s, um, &xm, &ym);
        double dev = hypot(xm - (x0 + sx[t]) / 2, ym - (y0 + sy[t]) / 2);
        // Always split once per span so S-shaped spans are not missed.
        if ((sd[t] == 0 || dev > tol) && sd[t] < 10 && top < 32 &&
            n < SPLINE_MAX_PTS - 1) {
          sd[t]++;
          su[top] = um, sx[top] = xm, sy[top] = ym, sd[top] = sd[t], top++;
          continue;
        }
        if (n < SPLINE_MAX_PTS) xs[n] = sx[t], ys[n] = sy[t], n++;
        u0 = su[t];
        top--;
      }
    }
    begin_poly(g, n, UMG_LTCONT);
    for (int i = 0; i < n; i++) add_pt(g, XF_ID, xs[i], ys[i], 0);
  } else if (s->num_fit_pts > 1) {  // fit-point splines: fit polyline
    begin_poly(g, s->num_fit_pts, UMG_LTCONT);
    for (BITCODE_BS i = 0; i < s->num_fit_pts; i++)
      add_pt(g, XF_ID, s->fit_pts[i].x, s->fit_pts[i].y, 0);
  } else {
    g->n_skipped++;
  }
}

// ---- fills ----------------------------------------------------------------

#include "earcut.h"

// One filled area: n vertices (triangles) picked from xy by tri, through m.
static void add_fill(Geom *g, Xf m, const double *xy, const uint32_t *tri,
                     size_t n) {
  if (!n) return;
  *(uint32_t *)vec_push(&g->fills) = (uint32_t)n;
  *(uint32_t *)vec_push(&g->fill_st) = g->cur;
  *(OrdKey *)vec_push(&g->fill_key) = g->ord;
  for (size_t k = 0; k < n; k++) {
    double x = xy[2 * tri[k]], y = xy[2 * tri[k] + 1];
    double X = m.a * x + m.b * y + m.tx, Y = m.c * x + m.d * y + m.ty;
    if (!g->origin_set) geom_origin(g, X, Y);
    UmgFPt *p = (UmgFPt *)vec_push(&g->fpts);
    p->x = QX(g, X), p->y = QY(g, Y);
  }
}

// Copies fill `fi` (first vertex index `f0`) of src into g through m.
static void copy_fill(Geom *g, Xf m, const Geom *src, size_t fi, size_t f0) {
  uint32_t n = ((const uint32_t *)src->fills.p)[fi];
  const UmgFPt *v = (const UmgFPt *)src->fpts.p + f0;
  double *xy = (double *)malloc(n * 2 * sizeof(double));
  uint32_t *tri = (uint32_t *)malloc(n * sizeof(uint32_t));
  for (uint32_t k = 0; k < n; k++)
    xy[2 * k] = WX(src, v[k].x), xy[2 * k + 1] = WY(src, v[k].y), tri[k] = k;
  add_fill(g, m, xy, tri, n);
  free(xy), free(tri);
}

// Polyline width (LWPOLYLINE / POLYLINE 2D, drawing units, so it zooms with
// the drawing; not lineweight): the band as a fill, under the centreline
// the caller still draws, which keeps it visible when the band is thinner
// than a pixel. Segment i runs from vertex i to i + 1 with width w0[i] ->
// w1[i]. Straight segments meeting at an equal width are mitred (AutoCAD
// joins them so); sharp corners and arcs are left butt-ended.
static void add_wide_poly(Geom *g, Xf m, const double *xy, const double *bul,
                          const double *w0, const double *w1, uint32_t n,
                          int closed) {
  uint32_t ns = closed ? n : n - 1;
  Vec V = {NULL, 0, 0, 2 * sizeof(double)}, T = {NULL, 0, 0, sizeof(uint32_t)};
  double(*c)[8] = (double(*)[8])calloc(ns, sizeof *c);  // Ls Rs Le Re
  double(*nm)[2] = (double(*)[2])calloc(ns, sizeof *nm);
  int *straight = (int *)calloc(ns, sizeof(int));
  for (uint32_t i = 0; i < ns; i++) {
    const double *p = xy + 2 * i, *q = xy + 2 * ((i + 1) % n);
    double len = hypot(q[0] - p[0], q[1] - p[1]);
    if (bul[i] != 0 || len == 0) continue;
    double nx = -(q[1] - p[1]) / len, ny = (q[0] - p[0]) / len;
    double hs = w0[i] / 2, he = w1[i] / 2;
    straight[i] = 1, nm[i][0] = nx, nm[i][1] = ny;
    c[i][0] = p[0] + nx * hs, c[i][1] = p[1] + ny * hs;
    c[i][2] = p[0] - nx * hs, c[i][3] = p[1] - ny * hs;
    c[i][4] = q[0] + nx * he, c[i][5] = q[1] + ny * he;
    c[i][6] = q[0] - nx * he, c[i][7] = q[1] - ny * he;
  }
  for (uint32_t i = 0; i + (closed ? 0 : 1) < ns; i++) {
    uint32_t k = (i + 1) % ns;
    if (!straight[i] || !straight[k] || w1[i] != w0[k] || w1[i] <= 0) continue;
    double sx = nm[i][0] + nm[k][0], sy = nm[i][1] + nm[k][1], sl = hypot(sx, sy);
    if (sl < 1e-9) continue;
    sx /= sl, sy /= sl;
    double cosh = sx * nm[i][0] + sy * nm[i][1];
    if (cosh < 0.25) continue;  // sharper than ~150 deg: miter too long
    double L = w1[i] / 2 / cosh;
    const double *j = xy + 2 * ((i + 1) % n);
    c[i][4] = c[k][0] = j[0] + sx * L, c[i][5] = c[k][1] = j[1] + sy * L;
    c[i][6] = c[k][2] = j[0] - sx * L, c[i][7] = c[k][3] = j[1] - sy * L;
  }
  for (uint32_t i = 0; i < ns; i++) {
    if (w0[i] <= 0 && w1[i] <= 0) continue;
    uint32_t b = (uint32_t)V.n;
    if (straight[i]) {
      static const int corner[4] = {0, 1, 3, 2};  // Ls Rs Re Le
      static const uint32_t quad[6] = {0, 1, 2, 0, 2, 3};
      for (int v = 0; v < 4; v++) {
        double *p = (double *)vec_push(&V);
        p[0] = c[i][2 * corner[v]], p[1] = c[i][2 * corner[v] + 1];
      }
      for (int t = 0; t < 6; t++) *(uint32_t *)vec_push(&T) = b + quad[t];
    } else if (bul[i] != 0) {
      const double *p = xy + 2 * i, *q = xy + 2 * ((i + 1) % n);
      if (p[0] == q[0] && p[1] == q[1]) continue;
      double cx, cy, r, a0, sw;
      umg_bulge_arc(p[0], p[1], q[0], q[1], bul[i], &cx, &cy, &r, &a0, &sw);
      int k = umg_arc_segs(sw);
      for (int s = 0; s <= k; s++) {
        double a = a0 + sw * s / k, h = (w0[i] + (w1[i] - w0[i]) * s / k) / 2;
        double ri = fmax(0, r - h), ro = r + h;
        double *v = (double *)vec_push(&V);
        v[0] = cx + ri * cos(a), v[1] = cy + ri * sin(a);
        v = (double *)vec_push(&V);
        v[0] = cx + ro * cos(a), v[1] = cy + ro * sin(a);
        if (s) {
          uint32_t t0 = b + 2 * (s - 1);
          uint32_t tri[6] = {t0, t0 + 1, t0 + 3, t0, t0 + 3, t0 + 2};
          for (int t = 0; t < 6; t++) *(uint32_t *)vec_push(&T) = tri[t];
        }
      }
    }
  }
  if (T.n) add_fill(g, m, (const double *)V.p, (const uint32_t *)T.p, T.n);
  free(V.p), free(T.p), free(c), free(nm), free(straight);
}

// Boundary loops of one hatch: points (x, y pairs) and where each loop
// starts. Every edge adds its points except the last, which is where the
// next edge starts.
typedef struct {
  Vec xy;     // double[2]
  Vec start;  // size_t, first point of each loop
} Loops;

static void loops_pt(Loops *L, double x, double y) {
  double *p = (double *)vec_push(&L->xy);
  p[0] = x, p[1] = y;
}

static void loops_arc(Loops *L, double cx, double cy, double r, double a0,
                      double sweep) {
  int k = umg_arc_segs(sweep);
  for (int i = 0; i < k; i++) {
    double a = a0 + sweep * i / k;
    loops_pt(L, cx + r * cos(a), cy + r * sin(a));
  }
}

// Hatch edge arcs and elliptic arcs store a clockwise arc as the mirrored
// counter-clockwise one: angles negated (checked on sample01: every loop
// closes this way).
static void hatch_sweep(const Dwg_HATCH_PathSeg *s, double *a0, double *sw) {
  double d = s->end_angle - s->start_angle;
  if (!(d > 0)) d = fmod(d, 2 * M_PI) + 2 * M_PI;  // fmod: no loop on 1e300
  if (!(d > 0) || d > 2 * M_PI) d = 2 * M_PI;
  *a0 = s->is_ccw ? s->start_angle : -s->start_angle;
  *sw = s->is_ccw ? d : -d;
}

#define HATCH_SPAN_PTS 8  // points per knot span of a spline edge

static void hatch_edge(Loops *L, const Dwg_HATCH_PathSeg *s) {
  switch (s->curve_type) {
  case 1:
    loops_pt(L, s->first_endpoint.x, s->first_endpoint.y);
    break;
  case 2: {
    double a0, sw;
    hatch_sweep(s, &a0, &sw);
    loops_arc(L, s->center.x, s->center.y, s->radius, a0, sw);
  } break;
  case 3: {
    double a0, sw, mx = s->endpoint.x, my = s->endpoint.y;
    double rr = s->minor_major_ratio;  // minor axis = major turned 90 deg
    hatch_sweep(s, &a0, &sw);
    int k = umg_arc_segs(sw);
    for (int i = 0; i < k; i++) {
      double t = a0 + sw * i / k, c = cos(t), n = sin(t) * rr;
      loops_pt(L, s->center.x + mx * c - my * n, s->center.y + my * c + mx * n);
    }
  } break;
  case 4: {
    int p = (int)s->degree, nc = (int)s->num_control_points;
    if (nc > p && p >= 1 && p < 16 && s->knots &&
        s->num_knots == (BITCODE_BL)(nc + p + 1)) {
      // Through spline_eval, with the edge dressed up as a SPLINE entity.
      Dwg_Entity_SPLINE sp;
      memset(&sp, 0, sizeof sp);
      sp.degree = (BITCODE_BS)p, sp.rational = s->is_rational;
      sp.num_knots = s->num_knots, sp.knots = s->knots;
      sp.num_ctrl_pts = (BITCODE_BL)nc;
      sp.ctrl_pts = (Dwg_SPLINE_control_point *)calloc(nc, sizeof *sp.ctrl_pts);
      for (int i = 0; i < nc; i++) {
        sp.ctrl_pts[i].x = s->control_points[i].point.x;
        sp.ctrl_pts[i].y = s->control_points[i].point.y;
        sp.ctrl_pts[i].w = s->control_points[i].weight;
      }
      for (int k = p; k < nc; k++) {
        double ua = s->knots[k], ub = s->knots[k + 1];
        for (int i = 0; ub > ua && i < HATCH_SPAN_PTS; i++) {
          double x, y;
          spline_eval(&sp, ua + (ub - ua) * i / HATCH_SPAN_PTS, &x, &y);
          loops_pt(L, x, y);
        }
      }
      free(sp.ctrl_pts);
    } else {
      for (BITCODE_BL i = 0; i + 1 < s->num_fitpts; i++)
        loops_pt(L, s->fitpts[i].x, s->fitpts[i].y);
    }
  } break;
  }
}

// Start and end of a line, arc or elliptic edge; 0 for splines (their
// control polygon need not end on the curve).
static int hatch_edge_ends(const Dwg_HATCH_PathSeg *s, double *a, double *b) {
  double a0, sw, c, d, mx = s->endpoint.x, my = s->endpoint.y;
  switch (s->curve_type) {
  case 1:
    a[0] = s->first_endpoint.x, a[1] = s->first_endpoint.y;
    b[0] = s->second_endpoint.x, b[1] = s->second_endpoint.y;
    return 1;
  case 2:
    hatch_sweep(s, &a0, &sw);
    a[0] = s->center.x + s->radius * cos(a0), a[1] = s->center.y + s->radius * sin(a0);
    b[0] = s->center.x + s->radius * cos(a0 + sw), b[1] = s->center.y + s->radius * sin(a0 + sw);
    return 1;
  case 3:
    hatch_sweep(s, &a0, &sw);
    c = cos(a0), d = sin(a0) * s->minor_major_ratio;
    a[0] = s->center.x + mx * c - my * d, a[1] = s->center.y + my * c + mx * d;
    c = cos(a0 + sw), d = sin(a0 + sw) * s->minor_major_ratio;
    b[0] = s->center.x + mx * c - my * d, b[1] = s->center.y + my * c + mx * d;
    return 1;
  }
  return 0;
}

// Whether each edge of an edge-type path ends where the next one starts,
// the last one closing on the first. Paths flagged not closed (0x20) can
// be loose edges in any order with dangling ends (sample01 72672E): taken
// point by point they make a large bogus polygon, so they are left out.
// On sample01 and the LibreDWG test files every other edge path is in order.
static int hatch_path_closed(const Dwg_HATCH_Path *P) {
  uint32_t n = P->num_segs_or_paths;
  double span = 0, (*a)[2], (*b)[2];
  int *known, ok = 1;
  if (!n) return 0;
  a = (double(*)[2])malloc(n * sizeof *a), b = (double(*)[2])malloc(n * sizeof *b);
  known = (int *)malloc(n * sizeof(int));
  for (uint32_t i = 0; i < n; i++)
    if ((known[i] = hatch_edge_ends(&P->segs[i], a[i], b[i])))
      span = fmax(span, fmax(fabs(a[i][0] - b[i][0]), fabs(a[i][1] - b[i][1])));
  double tol = 1e-4 * span + 1e-9;
  for (uint32_t i = 0; i < n && ok; i++) {
    uint32_t j = (i + 1) % n;
    if (known[i] && known[j])
      ok = fabs(b[i][0] - a[j][0]) <= tol && fabs(b[i][1] - a[j][1]) <= tol;
  }
  free(a), free(b), free(known);
  return ok;
}

static void hatch_loops(Loops *L, const Dwg_Entity_HATCH *h) {
  for (BITCODE_BL k = 0; k < h->num_paths; k++) {
    const Dwg_HATCH_Path *P = &h->paths[k];
    if (!(P->flag & 2) && !hatch_path_closed(P)) continue;
    *(size_t *)vec_push(&L->start) = L->xy.n;
    if (P->flag & 2) {  // polyline loop, closed
      uint32_t n = P->num_segs_or_paths;
      for (uint32_t i = 0; i < n; i++) {
        const Dwg_HATCH_PolylinePath *a = &P->polyline_paths[i];
        const Dwg_HATCH_PolylinePath *b = &P->polyline_paths[(i + 1) % n];
        loops_pt(L, a->point.x, a->point.y);
        if (P->bulges_present && a->bulge != 0 &&
            (a->point.x != b->point.x || a->point.y != b->point.y)) {
          double cx, cy, r, a0, sw;
          umg_bulge_arc(a->point.x, a->point.y, b->point.x, b->point.y,
                        a->bulge, &cx, &cy, &r, &a0, &sw);
          int m = umg_arc_segs(sw);
          for (int j = 1; j < m; j++) {
            double t = a0 + sw * j / m;
            loops_pt(L, cx + r * cos(t), cy + r * sin(t));
          }
        }
      }
    } else {
      for (BITCODE_BL i = 0; i < P->num_segs_or_paths; i++)
        hatch_edge(L, &P->segs[i]);
    }
  }
}

// Even-odd point in ring [a, b) of xy.
static int ring_contains(const double *xy, size_t a, size_t b, double px,
                         double py) {
  int in = 0;
  for (size_t i = a, j = b - 1; i < b; j = i++) {
    double xi = xy[2 * i], yi = xy[2 * i + 1], xj = xy[2 * j], yj = xy[2 * j + 1];
    if ((yi > py) != (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi)
      in = !in;
  }
  return in;
}

// Whether ring i lies inside ring j: majority of its first three points
// (one point can sit on a shared boundary).
static int ring_inside(const double *xy, const size_t *st, size_t i, size_t j) {
  size_t a = st[i], n = st[i + 1] - a, votes = 0, k;
  for (k = 0; k < 3 && k < n; k++)
    votes += ring_contains(xy, st[j], st[j + 1], xy[2 * (a + k)], xy[2 * (a + k) + 1]);
  return 2 * votes > k;
}

// Loop starts with an end sentinel, and each loop's nesting depth (number of
// loops around it). Loops under 3 points get depth -1 (unused).
static size_t *loops_depth(const Loops *L, int **depth) {
  size_t nl = L->start.n;
  const double *xy = (const double *)L->xy.p;
  size_t *st = (size_t *)malloc((nl + 1) * sizeof(size_t));
  if (nl) memcpy(st, L->start.p, nl * sizeof(size_t));
  st[nl] = L->xy.n;
  int *d = (int *)calloc(nl ? nl : 1, sizeof(int));
  for (size_t i = 0; i < nl; i++) {
    if (st[i + 1] - st[i] < 3) { d[i] = -1; continue; }
    for (size_t j = 0; j < nl; j++)
      if (j != i && st[j + 1] - st[j] >= 3 && ring_inside(xy, st, i, j)) d[i]++;
  }
  *depth = d;
  return st;
}

// Deepest loop a hatch style uses: 0 (normal) alternates filled and empty
// at every depth, 1 (outer) only the outermost loops and their direct
// holes, 2 (ignore) the outermost loops whole.
static int style_max_depth(int style) {
  return style == 2 ? 0 : style == 1 ? 1 : INT32_MAX;
}

static void fill_loops(Geom *g, Xf m, Loops *L, int style) {
  size_t nl = L->start.n;
  if (!nl) return;
  const double *xy = (const double *)L->xy.p;
  int *depth;
  size_t *st = loops_depth(L, &depth);
  int maxd = style_max_depth(style);
  Vec ring = {NULL, 0, 0, 2 * sizeof(double)}, tri = {NULL, 0, 0, 4};
  Vec holes = {NULL, 0, 0, sizeof(size_t)};
  for (size_t i = 0; i < nl; i++) {
    if (depth[i] < 0 || depth[i] % 2 || depth[i] > maxd) continue;
    ring.n = tri.n = holes.n = 0;
    vec_append(&ring, xy + 2 * st[i], st[i + 1] - st[i]);
    for (size_t j = 0; j < nl && depth[i] < maxd; j++)
      if (depth[j] == depth[i] + 1 && ring_inside(xy, st, j, i)) {
        *(size_t *)vec_push(&holes) = ring.n;
        vec_append(&ring, xy + 2 * st[j], st[j + 1] - st[j]);
      }
    ec_triangulate((const double *)ring.p, ring.n, (const size_t *)holes.p,
                   holes.n, &tri);
    add_fill(g, m, (const double *)ring.p, (const uint32_t *)tri.p, tri.n);
  }
  free(ring.p), free(tri.p), free(holes.p), free(depth), free(st);
}

static int cmp_double(const void *a, const void *b) {
  double x = *(const double *)a, y = *(const double *)b;
  return x < y ? -1 : x > y;
}

#define HATCH_MAX_SEGS 2000000  // work for one hatch; denser ones are dropped
// Lines x loop edges visited, per hatch and per drawing. Real drawings:
// at most 5e5 for one hatch (field), so these only stop damaged files
// (fuzzing: one file ran for minutes).
#define HATCH_MAX_WORK 2e7
#define HATCH_MAX_WORK_TOTAL 4e8
static double g_hatch_work_max, g_hatch_work_total;  // reported on stderr

// Pattern hatch: each pattern line family (already scaled and rotated in the
// DWG) is cut by the loops even-odd, then dashed (dots: zero-length lines).
// With count != NULL only counts the dashes into *count (stops past the cap).
static void pattern_lines(Geom *g, Xf m, const Dwg_Entity_HATCH *h,
                          const Loops *L, size_t *count) {
  int *depth;
  size_t *st = loops_depth(L, &depth);
  const double *xy = (const double *)L->xy.p;
  size_t nl = L->start.n, ne = 0;
  int maxd = style_max_depth(h->style);
  for (size_t i = 0; i < nl; i++)
    if (depth[i] >= 0 && depth[i] <= maxd) ne += st[i + 1] - st[i];
  double *ts = (double *)malloc((ne ? ne : 1) * sizeof(double));
  for (BITCODE_BS k = 0; k < h->num_deflines; k++) {
    const Dwg_HATCH_DefLine *D = &h->deflines[k];
    double ux = cos(D->angle), uy = sin(D->angle), nx = -uy, ny = ux;
    double sp = D->offset.x * nx + D->offset.y * ny;  // spacing across lines
    if (fabs(sp) < 1e-9) continue;
    double pat = 0;
    BITCODE_BS nd = D->dashes ? D->num_dashes : 0;  // count without data: solid
    for (BITCODE_BS j = 0; j < nd; j++) pat += fabs(D->dashes[j]);
    double vmin = INFINITY, vmax = -INFINITY;  // loops across the lines
    for (size_t i = 0; i < L->xy.n; i++) {
      double v = (xy[2 * i] - D->pt0.x) * nx + (xy[2 * i + 1] - D->pt0.y) * ny;
      vmin = fmin(vmin, v), vmax = fmax(vmax, v);
    }
    double k0 = ceil(fmin(vmin / sp, vmax / sp)), k1 = floor(fmax(vmin / sp, vmax / sp));
    // Line numbers past exact doubles (2^53) would not advance with kk++
    // (fuzzing: endless loop); no real hatch is that far from its origin.
    if (!(fabs(k0) < 9e15 && fabs(k1) < 9e15)) continue;
    // Work is bounded by lines and pattern repeats, not only by output, and
    // by lines times the edges each line is cut against.
    double nlines = fmax(0, k1 - k0 + 1);
    double work = nlines * (double)ne;
    if (count && work > g_hatch_work_max) g_hatch_work_max = work;
    if (count && (!(nlines <= HATCH_MAX_SEGS) || work > HATCH_MAX_WORK ||
                  (g_hatch_work_total += work) > HATCH_MAX_WORK_TOTAL)) {
      *count = HATCH_MAX_SEGS + 1;
      goto done;
    }
    if (count && (*count += (size_t)nlines) > HATCH_MAX_SEGS) goto done;
    for (double kk = k0; kk <= k1; kk++) {
      double ox = D->pt0.x + kk * D->offset.x, oy = D->pt0.y + kk * D->offset.y;
      size_t nt = 0;
      for (size_t i = 0; i < nl; i++) {
        if (depth[i] < 0 || depth[i] > maxd) continue;
        for (size_t a = st[i], b = st[i + 1] - 1; a < st[i + 1]; b = a++) {
          double da = (xy[2 * a] - ox) * nx + (xy[2 * a + 1] - oy) * ny;
          double db = (xy[2 * b] - ox) * nx + (xy[2 * b + 1] - oy) * ny;
          if ((da > 0) == (db > 0)) continue;
          double f = da / (da - db);
          double px = xy[2 * a] + f * (xy[2 * b] - xy[2 * a]);
          double py = xy[2 * a + 1] + f * (xy[2 * b + 1] - xy[2 * a + 1]);
          ts[nt++] = (px - ox) * ux + (py - oy) * uy;
        }
      }
      qsort(ts, nt, sizeof(double), cmp_double);
      const float across = lod_key(sp, kk);
      g->lod = across;
      for (size_t i = 0; i + 1 < nt; i += 2) {
        double t0 = ts[i], t1 = ts[i + 1];
#define EMIT(a, b)                                                           \
  do {                                                                       \
    if (count) { if (++*count > HATCH_MAX_SEGS) goto done; }                 \
    else add_hline(g, m, ox + (a) * ux, oy + (a) * uy, ox + (b) * ux,       \
                   oy + (b) * uy);                                           \
  } while (0)
        // Pieces under one fixed-point step would be stored with equal
        // ends, i.e. as dots: an edge only grazed at a vertex gives such
        // a sliver, and whether it does depends on libm rounding (Mac and
        // phone differed by one stray dot on f3).
        if (pat <= 0) {
          if (t1 - t0 > g->q) EMIT(t0, t1);
          continue;
        }
        double reps = (t1 - t0) / pat;
        if (count && (!(reps <= HATCH_MAX_SEGS) ||
                      (*count += (size_t)reps) > HATCH_MAX_SEGS)) {
          *count = HATCH_MAX_SEGS + 1;
          goto done;
        }
        double base = floor(t0 / pat) * pat;
        // Far from the pattern origin a short pattern stops advancing
        // (base + pat == base in double): bound the repeats instead of
        // trusting the sum (fuzzing: endless loop).
        if (!(base + pat > base)) continue;
        for (long rep = 0; base < t1 && rep <= (long)reps + 1; base += pat, rep++) {
          double acc = base;
          g->lod = fminf(across, lod_key(pat, floor(base / pat + 0.5)));
          for (BITCODE_BS j = 0; j < nd; j++) {
            double dl = D->dashes[j];
            if (dl > 0) {
              double a = fmax(acc, t0), b = fmin(acc + dl, t1);
              if (b - a > g->q) EMIT(a, b);
            } else if (dl == 0 && acc >= t0 && acc < t1) {
              EMIT(acc, acc);  // dot: a zero-length line, drawn as a point
            }
            acc += fabs(dl);
          }
        }
#undef EMIT
      }
    }
  }
done:
  g->lod = INFINITY;
  free(ts), free(st), free(depth);
}

static unsigned long n_hatch_dense;  // pattern hatches over HATCH_MAX_SEGS

static void add_hatch(Geom *g, const Dwg_Entity_HATCH *h) {
  Xf m = h->extrusion.z < 0 ? XF_MIRROR_X : XF_ID;
  Loops L = {{NULL, 0, 0, 2 * sizeof(double)}, {NULL, 0, 0, sizeof(size_t)}};
  hatch_loops(&L, h);
  if (!L.start.n) {  // every path left out: nothing to fill or bound lines
  } else if (h->is_solid_fill) {
    fill_loops(g, m, &L, h->style);
  } else {
    size_t n = 0;
    pattern_lines(g, m, h, &L, &n);
    if (n > HATCH_MAX_SEGS) {
      n_hatch_dense++;
    } else {
      // Pattern lines are drawn continuous whatever the hatch's linetype.
      RawStyle k = g_styles.v[g->cur];
      uint32_t saved = g->cur;
      k.ltype = LT_CONT;
      g->cur = style_intern(k);
      pattern_lines(g, m, h, &L, NULL);
      g->cur = saved;
    }
  }
  free(L.xy.p), free(L.start.p);
}

// SOLID: corners 1 2 3 4 outline the quad 1 2 4 3 (3 == 4: a triangle).
static void add_solid(Geom *g, const Dwg_Entity_SOLID *s) {
  double xy[8] = {s->corner1.x, s->corner1.y, s->corner2.x, s->corner2.y,
                  s->corner3.x, s->corner3.y, s->corner4.x, s->corner4.y};
  static const uint32_t tri[6] = {0, 1, 2, 1, 3, 2};
  int quad = xy[4] != xy[6] || xy[5] != xy[7];
  add_fill(g, s->extrusion.z < 0 ? XF_MIRROR_X : XF_ID, xy, tri, quad ? 6 : 3);
}

// WIPEOUT: a polygon in the background colour that hides the fills and
// hatches before it in draw order. Clip vertices are image pixels with the
// origin at the top-left (as ezdxf maps them); a WIPEOUT image is 1 x 1.
static void add_wipeout(Geom *g, const Dwg_Entity_WIPEOUT *w) {
  double sx = w->image_size.x, sy = w->image_size.y;
  BITCODE_2RD box[4] = {{-0.5, -0.5}, {sx - 0.5, -0.5}, {sx - 0.5, sy - 0.5},
                        {-0.5, sy - 0.5}};
  const BITCODE_2RD *v = box;
  uint32_t n = 4;
  if (w->clipping && w->num_clip_verts >= 3) {
    v = w->clip_verts, n = w->num_clip_verts;
  } else if (w->clipping && w->num_clip_verts == 2) {  // rectangle corners
    const BITCODE_2RD *c = w->clip_verts;
    box[0].x = box[3].x = c[0].x, box[1].x = box[2].x = c[1].x;
    box[0].y = box[1].y = c[0].y, box[2].y = box[3].y = c[1].y;
  }
  double *xy = (double *)malloc(n * 2 * sizeof(double));
  for (uint32_t i = 0; i < n; i++) {
    double a = v[i].x + 0.5, b = sy - 0.5 - v[i].y;
    xy[2 * i] = w->pt0.x + w->uvec.x * a + w->vvec.x * b;
    xy[2 * i + 1] = w->pt0.y + w->uvec.y * a + w->vvec.y * b;
  }
  Vec tri = {NULL, 0, 0, 4};
  ec_triangulate(xy, n, NULL, 0, &tri);
  RawStyle k = g_styles.v[g->cur];
  k.color = COL_BG, k.ltype = LT_CONT;
  g->cur = style_intern(k);
  add_fill(g, XF_ID, xy, (const uint32_t *)tri.p, tri.n);
  free(xy), free(tri.p);
}

// ---- old-style polylines and leaders --------------------------------------

// VERTEX objects of an old-style POLYLINE, in order: R2004+ lists their
// handles, older files chain them from first_vertex to last_vertex.
static size_t polyline_vertices(const Dwg_Object *o, BITCODE_BL num_owned,
                                BITCODE_H *vertex, BITCODE_H first,
                                BITCODE_H last, Vec *out) {
  Dwg_Data *d = o->parent;
  out->elem = sizeof(Dwg_Object *), out->n = 0;
  if (vertex && num_owned) {
    for (BITCODE_BL i = 0; i < num_owned; i++) {
      Dwg_Object *x = vertex[i] ? dwg_ref_object(d, vertex[i]) : NULL;
      if (x) *(Dwg_Object **)vec_push(out) = x;
    }
  } else if (first) {
    Dwg_Object *x = dwg_ref_object(d, first), *l = last ? dwg_ref_object(d, last) : NULL;
    for (size_t k = 0; x && k < 10000000 && x->fixedtype != DWG_TYPE_SEQEND;
         k++, x = dwg_next_object(x)) {
      *(Dwg_Object **)vec_push(out) = x;
      if (x == l) break;
    }
  }
  return out->n;
}

// Whether a closed-flagged polyline of n vertices has a closing segment
// worth drawing: two vertices close into a real shape when bulged (dimension
// dot arrowheads and DONUTs are two half-circle arcs of width).
static int poly_closes(uint32_t n, const double *bul) {
  return n > 2 || (n == 2 && (bul[0] != 0 || bul[1] != 0));
}

// POLYLINE (2D): OCS vertices with bulges. Spline-fit polylines draw their
// fitted vertices, never the frame control points.
static void add_polyline_2d(Geom *g, const Dwg_Object *o) {
  const Dwg_Entity_POLYLINE_2D *p = o->tio.entity->tio.POLYLINE_2D;
  Vec v = {NULL, 0, 0, 0};
  size_t nv = polyline_vertices(o, p->num_owned, p->vertex, p->first_vertex,
                                p->last_vertex, &v);
  double *xy = (double *)malloc((nv ? nv : 1) * 2 * sizeof(double));
  double *bul = (double *)calloc(nv ? nv : 1, sizeof(double));
  double *w0 = (double *)calloc(nv ? nv : 1, sizeof(double));
  double *w1 = (double *)calloc(nv ? nv : 1, sizeof(double));
  uint32_t n = 0;
  int wide = 0;
  for (size_t i = 0; i < nv; i++) {
    const Dwg_Object *x = ((Dwg_Object **)v.p)[i];
    if (x->fixedtype != DWG_TYPE_VERTEX_2D) continue;
    const Dwg_Entity_VERTEX_2D *t = x->tio.entity->tio.VERTEX_2D;
    if (t->flag & 16) continue;                     // spline frame point
    if ((p->flag & 4) && !(t->flag & 8)) continue;  // spline: fitted only
    xy[2 * n] = t->point.x, xy[2 * n + 1] = t->point.y, bul[n] = t->bulge;
    // A vertex without widths of its own takes the polyline's default.
    int own = t->start_width != 0 || t->end_width != 0;
    w0[n] = own ? t->start_width : p->start_width;
    w1[n] = own ? t->end_width : p->end_width;
    wide |= w0[n] > 0 || w1[n] > 0;
    n++;
  }
  if (n >= 2) {
    Xf m = p->extrusion.z < 0 ? XF_MIRROR_X : XF_ID;
    int closed = (p->flag & 1) && poly_closes(n, bul);
    add_bulged_poly(g, m, xy, bul, n, (closed ? UMG_CLOSED : 0) |
                                          (p->flag & 0x80 ? UMG_LTCONT : 0));
    if (wide) add_wide_poly(g, m, xy, bul, w0, w1, n, closed);
  } else {
    g->n_skipped++;
  }
  free(xy), free(bul), free(w0), free(w1), free(v.p);
}

// POLYLINE (3D): WCS vertices, drawn in plan (z dropped).
static void add_polyline_3d(Geom *g, const Dwg_Object *o) {
  const Dwg_Entity_POLYLINE_3D *p = o->tio.entity->tio.POLYLINE_3D;
  Vec v = {NULL, 0, 0, 0};
  size_t nv = polyline_vertices(o, p->num_owned, p->vertex, p->first_vertex,
                                p->last_vertex, &v);
  uint32_t n = 0;
  for (size_t i = 0; i < nv; i++) {
    const Dwg_Object *x = ((Dwg_Object **)v.p)[i];
    if (x->fixedtype == DWG_TYPE_VERTEX_3D && !(x->tio.entity->tio.VERTEX_3D->flag & 16)) n++;
  }
  if (n >= 2) {
    begin_poly(g, n, (p->flag & 1) && n > 2 ? UMG_CLOSED : 0);
    for (size_t i = 0; i < nv; i++) {
      const Dwg_Object *x = ((Dwg_Object **)v.p)[i];
      if (x->fixedtype != DWG_TYPE_VERTEX_3D) continue;
      const Dwg_Entity_VERTEX_3D *t = x->tio.entity->tio.VERTEX_3D;
      if (!(t->flag & 16)) add_pt(g, XF_ID, t->point.x, t->point.y, 0);
    }
  } else {
    g->n_skipped++;
  }
  free(v.p);
}

// LEADER: its path (WCS), with a closed filled arrowhead (length DIMASZ,
// width a third of it, AutoCAD's default) at the first point.
static void add_leader(Geom *g, const Dwg_Object *o) {
  const Dwg_Entity_LEADER *l = o->tio.entity->tio.LEADER;
  uint32_t n = l->num_points;
  if (n < 2 || !l->points) { g->n_skipped++; return; }
  begin_poly(g, n, UMG_LTCONT);
  for (uint32_t i = 0; i < n; i++) add_pt(g, XF_ID, l->points[i].x, l->points[i].y, 0);
  const Dwg_Header_Variables *hv = &o->parent->header_vars;
  double sc = hv->DIMSCALE > 0 ? hv->DIMSCALE : 1;
  double a = l->dimasz > 0 ? l->dimasz : hv->DIMASZ * sc;
  double dx = l->points[1].x - l->points[0].x, dy = l->points[1].y - l->points[0].y;
  double len = hypot(dx, dy);
  if (!l->arrowhead_on || !(a > 0) || len < a) return;
  dx /= len, dy /= len;
  double bx = l->points[0].x + dx * a, by = l->points[0].y + dy * a, w = a / 6;
  double xy[6] = {l->points[0].x, l->points[0].y, bx - dy * w, by + dx * w,
                  bx + dy * w, by - dx * w};
  static const uint32_t tri[3] = {0, 1, 2};
  add_fill(g, XF_ID, xy, tri, 3);
}

// POINT, shaped by PDMODE: low 3 bits 0 dot, 1 nothing, 2 plus, 3 cross,
// 4 tick up; +32 circle, +64 square around it. PDSIZE > 0 is the size, else
// a percentage (0: 5) of the drawing height (AutoCAD: of the view at regen).
// The dot is a zero-length hatch line, drawn as a point like pattern dots.
static void add_point(Geom *g, const Dwg_Object *o) {
  const Dwg_Entity_POINT *p = o->tio.entity->tio.POINT;
  const Dwg_Header_Variables *hv = &o->parent->header_vars;
  int mode = hv->PDMODE, shape = mode & 7;
  double s = hv->PDSIZE;
  // PDSIZE <= 0 sizes the symbol relative to the screen, which a static
  // picture cannot do; scaled to the drawing extents it grew huge (public
  // drawing: PDMODE 35, PDSIZE -10%, dimension definition points became
  // circles and crosses many times the plan). Such points are dots.
  if (!(s > 0)) mode = 0, shape = 0;
  double x = p->x, y = p->y, h = s / 2;
  double c = cos(p->x_ang), sn = sin(p->x_ang);
  Xf m = {c, -sn, sn, c, x, y};  // symbol axes turned by x_ang
  if (shape == 0) add_hline(g, XF_ID, x, y, x, y);
  else if (shape == 2) add_line(g, m, -h, 0, h, 0), add_line(g, m, 0, -h, 0, h);
  else if (shape == 3) add_line(g, m, -h, -h, h, h), add_line(g, m, -h, h, h, -h);
  else if (shape == 4) add_line(g, m, 0, 0, 0, h);
  if (mode & 32) add_arc(g, XF_ID, x, y, h, 0, 2 * M_PI);
  if (mode & 64) {
    begin_poly(g, 4, UMG_CLOSED | UMG_LTCONT);
    add_pt(g, m, -h, -h, 0), add_pt(g, m, h, -h, 0);
    add_pt(g, m, h, h, 0), add_pt(g, m, -h, h, 0);
  }
}

// 3DFACE seen from above: its edges, minus those flagged invisible (bit i
// for the edge leaving corner i + 1); a triangle repeats its third corner.
static void add_3dface(Geom *g, const Dwg_Entity__3DFACE *f) {
  const BITCODE_3BD *c[4] = {&f->corner1, &f->corner2, &f->corner3, &f->corner4};
  int n = c[3]->x == c[2]->x && c[3]->y == c[2]->y ? 3 : 4;
  for (int i = 0; i < n; i++) {
    if (!f->has_no_flags && (f->invis_flags >> i & 1)) continue;
    const BITCODE_3BD *a = c[i], *b = c[(i + 1) % n];
    if (a->x != b->x || a->y != b->y) add_line(g, XF_ID, a->x, a->y, b->x, b->y);
  }
}

// XLINE (both ways) and RAY (forward from its point): infinite, so cut to the
// header extents box, which AutoCAD computes without them. Lines that miss
// it are left out; so is everything with unusable extents.
static void add_xline(Geom *g, const Dwg_Object *o, int both) {
  const Dwg_Entity_RAY *r = o->tio.entity->tio.RAY;
  const Dwg_Header_Variables *hv = &o->parent->header_vars;
  double x0 = hv->EXTMIN.x, y0 = hv->EXTMIN.y, x1 = hv->EXTMAX.x, y1 = hv->EXTMAX.y;
  double px = r->point.x, py = r->point.y, dx = r->vector.x, dy = r->vector.y;
  if (!(x1 > x0 && y1 > y0 && x1 - x0 < 1e12 && y1 - y0 < 1e12) ||
      !(fabs(dx) + fabs(dy) > 0)) {
    g->n_skipped++;
    return;
  }
  double lo = both ? -INFINITY : 0, hi = INFINITY;  // Liang-Barsky
  double pp[4] = {-dx, dx, -dy, dy}, qq[4] = {px - x0, x1 - px, py - y0, y1 - py};
  for (int i = 0; i < 4; i++) {
    if (pp[i] == 0) {
      if (qq[i] < 0) return;
    } else {
      double t = qq[i] / pp[i];
      if (pp[i] < 0) lo = fmax(lo, t);
      else hi = fmin(hi, t);
    }
  }
  if (lo < hi)
    add_line(g, XF_ID, px + lo * dx, py + lo * dy, px + hi * dx, py + hi * dy);
}

// MLINE: parallel elements along the vertices. Element j at vertex i starts
// segparms[0] along the miter; the rest are visible-run boundaries along the
// segment (start, break, start, break, ...; an open last run reaches the
// next vertex). Colours, square caps and the fill come from the MLINESTYLE.
static void add_mline(Geom *g, const Dwg_Object *o) {
  const Dwg_Entity_MLINE *ml = o->tio.entity->tio.MLINE;
  uint32_t nv = ml->num_verts, ne = ml->num_lines;
  if (nv < 2 || !ne || !ml->verts) { g->n_skipped++; return; }
  for (uint32_t i = 0; i < nv; i++)
    if (ml->verts[i].num_lines < ne || !ml->verts[i].lines) { g->n_skipped++; return; }
  Dwg_Object *so = ml->mlinestyle ? dwg_ref_object(o->parent, ml->mlinestyle) : NULL;
  const Dwg_Object_MLINESTYLE *st =
      so && so->fixedtype == DWG_TYPE_MLINESTYLE ? so->tio.object->tio.MLINESTYLE : NULL;
  int has_rgb = o->parent->header.from_version >= R_2004;
  int closed = (ml->flags & 2) != 0;
  uint32_t nseg = closed ? nv : nv - 1, base = g->cur;
#define MP(i, j, X, Y)                                                    \
  do {                                                                    \
    const Dwg_MLINE_vertex *v_ = &ml->verts[i];                           \
    double m_ = v_->lines[j].num_segparms ? v_->lines[j].segparms[0] : 0; \
    X = v_->vertex.x + v_->miter_direction.x * m_;                        \
    Y = v_->vertex.y + v_->miter_direction.y * m_;                        \
  } while (0)
  RawStyle k = g_styles.v[base];
  for (uint32_t j = 0; j < ne; j++) {
    if (st && j < st->num_lines) {  // element colour, else the entity's
      k.color = raw_color(&st->lines[j].color, has_rgb);
      g->cur = style_intern(k);
    }
    for (uint32_t i = 0; i < nseg; i++) {
      double ax, ay, bx, by;
      MP(i, j, ax, ay);
      MP((i + 1) % nv, j, bx, by);
      double len = hypot(bx - ax, by - ay);
      if (len <= 0) continue;
      const Dwg_MLINE_line *L = &ml->verts[i].lines[j];
      uint32_t np = L->num_segparms > 1 ? L->num_segparms - 1 : 0;
      const double *t = np ? L->segparms + 1 : NULL;
      if (!np) {
        add_line(g, XF_ID, ax, ay, bx, by);
        continue;
      }
      for (uint32_t r = 0; r < np; r += 2) {
        double t0 = fmax(0, t[r]), t1 = fmin(len, r + 1 < np ? t[r + 1] : len);
        if (t1 > t0)
          add_line(g, XF_ID, ax + (bx - ax) * t0 / len, ay + (by - ay) * t0 / len,
                   ax + (bx - ax) * t1 / len, ay + (by - ay) * t1 / len);
      }
    }
  }
  g->cur = base;
  // Outermost elements: square caps and the fill between them.
  uint32_t lo = 0, hi = 0;
  for (uint32_t j = 1; j < ne; j++) {
    double sj = ml->verts[0].lines[j].num_segparms ? ml->verts[0].lines[j].segparms[0] : 0;
    double sl = ml->verts[0].lines[lo].num_segparms ? ml->verts[0].lines[lo].segparms[0] : 0;
    double sh = ml->verts[0].lines[hi].num_segparms ? ml->verts[0].lines[hi].segparms[0] : 0;
    if (sj < sl) lo = j;
    if (sj > sh) hi = j;
  }
  int sflag = st ? st->flag : 0;
  if (!closed && lo != hi) {
    double ax, ay, bx, by;
    if ((sflag & 16) && !(ml->flags & 4)) {
      MP(0, lo, ax, ay);
      MP(0, hi, bx, by);
      add_line(g, XF_ID, ax, ay, bx, by);
    }
    if ((sflag & 256) && !(ml->flags & 8)) {
      MP(nv - 1, lo, ax, ay);
      MP(nv - 1, hi, bx, by);
      add_line(g, XF_ID, ax, ay, bx, by);
    }
  }
  if ((sflag & 1) && lo != hi) {
    k = g_styles.v[base];
    k.color = raw_color(&st->fill_color, has_rgb);
    g->cur = style_intern(k);
    for (uint32_t i = 0; i < nseg; i++) {
      double xy[8];
      MP(i, lo, xy[0], xy[1]);
      MP((i + 1) % nv, lo, xy[2], xy[3]);
      MP((i + 1) % nv, hi, xy[4], xy[5]);
      MP(i, hi, xy[6], xy[7]);
      static const uint32_t tri[6] = {0, 1, 2, 0, 2, 3};
      add_fill(g, XF_ID, xy, tri, 6);
    }
    g->cur = base;
  }
#undef MP
}

// ---- text -----------------------------------------------------------------

// LibreDWG string helpers (src/bits.h is not installed).
char *bit_convert_TU(const BITCODE_TU wstr);
char *bit_TV_to_utf8(const char *src, const BITCODE_RS codepage);

static void pool_cp(Vec *pool, unsigned cp) {  // one code point as UTF-8
  char u[4];
  size_t n;
  if (cp < 0x80) u[0] = cp, n = 1;
  else if (cp < 0x800) u[0] = 0xC0 | cp >> 6, u[1] = 0x80 | (cp & 63), n = 2;
  else if (cp < 0x10000)
    u[0] = 0xE0 | cp >> 12, u[1] = 0x80 | (cp >> 6 & 63),
    u[2] = 0x80 | (cp & 63), n = 3;
  else
    u[0] = 0xF0 | cp >> 18, u[1] = 0x80 | (cp >> 12 & 63),
    u[2] = 0x80 | (cp >> 6 & 63), u[3] = 0x80 | (cp & 63), n = 4;
  vec_append(pool, u, n);
}

// Appends s as plain text: %% control codes for both kinds, plus MTEXT
// inline formatting (\P paragraphs, {\f..;} groups, \S stacks).
static void pool_clean(Vec *pool, const char *s, int mtext) {
  while (*s) {
    if (s[0] == '%' && s[1] == '%' && s[2]) {
      char c = s[2] | 0x20;
      if (c == 'c') pool_cp(pool, 0x2300);       // diameter
      else if (c == 'd') pool_cp(pool, 0xB0);    // degree
      else if (c == 'p') pool_cp(pool, 0xB1);    // plus-minus
      else if (s[2] == '%') vec_append(pool, "%", 1);
      else if (s[2] >= '0' && s[2] <= '9') {  // %%nnn
        unsigned v = 0;
        int k = 2;
        while (k < 5 && s[k] >= '0' && s[k] <= '9') v = v * 10 + s[k++] - '0';
        pool_cp(pool, v);
        s += k;
        continue;
      }  // %%u, %%o (under/overline toggles) are dropped
      s += 3;
      continue;
    }
    if (mtext && *s == '\\' && s[1]) {
      char c = s[1];
      s += 2;
      if (c == 'P' || c == 'X') vec_append(pool, "\n", 1);
      else if (c == '~') vec_append(pool, " ", 1);
      else if (c == '\\' || c == '{' || c == '}') vec_append(pool, &c, 1);
      else if ((c == 'U' || c == 'u') && *s == '+') {
        pool_cp(pool, (unsigned)strtoul(s + 1, NULL, 16));
        s += 5;
      } else if (c == 'S') {  // stacked fraction a^b, a/b, a#b up to ';'
        for (; *s && *s != ';'; s++)
          vec_append(pool, (*s == '^' || *s == '#') ? "/" : s, 1);
        if (*s) s++;
      } else if (strchr("fFHCcTQWAp", c)) {  // parameter up to ';'
        while (*s && *s != ';') s++;
        if (*s) s++;
      }  // L l O o K k N: toggles without parameter, dropped
      continue;
    }
    if (mtext && (*s == '{' || *s == '}')) {
      s++;
      continue;
    }
    vec_append(pool, s, 1);
    s++;
  }
}

// MTEXT inline colour (\C<aci>; within { } groups) when one colour covers
// all visible text, as raw colour; otherwise -1 (the entity colour stands).
// Titles are often written {\C256;...} on a coloured MTEXT (field drawing
// f1: yellow entity, white text). Per-run colours would need text runs in
// UMG; none of the sample drawings mix colours in one MTEXT.
static int64_t mtext_inline_color(const char *s) {
  int64_t stack[32], cur = -1, first = -2;
  int sp = 0;
  while (*s) {
    if (*s == '\\' && s[1]) {
      char c = s[1];
      s += 2;
      if (c == 'C') cur = strtol(s, NULL, 10);
      else if (c == 'c') cur = -3;  // true colour: not handled
      if (strchr("fFHCcTQWApS", c)) {
        while (*s && *s != ';') s++;
        if (*s) s++;
        continue;
      }
      if (c == 'U' || c == 'u') s += *s == '+' ? 5 : 0;
      else if (!strchr("~\\{}", c)) continue;  // \P and toggles: not text
    } else if (*s == '{' || *s == '}') {
      if (*s == '{' && sp < 32) stack[sp++] = cur;
      else if (*s == '}' && sp) cur = stack[--sp];
      s++;
      continue;
    } else if (*s == ' ' || *s == '\t') {
      s++;
      continue;
    } else {
      s++;
    }
    if (first == -2) first = cur;
    else if (cur != first) return -1;
  }
  if (first < 0) return -1;
  return first == 0 ? COL_BYBLOCK : first >= 256 ? COL_BYLAYER
                                                 : (COL_ACI | (uint32_t)first);
}

// Returns a UTF-8 copy of a DWG string (caller frees), or NULL.
static char *dwg_utf8(const Dwg_Object *o, BITCODE_T str) {
  if (!str) return NULL;
  Dwg_Data *dwg = o->parent;
  if (dwg->header.from_version >= R_2007) return bit_convert_TU((BITCODE_TU)str);
  char *r = bit_TV_to_utf8(str, dwg->header.codepage);
  return r == str ? strdup(str) : r;
}

// Records a text whose anchor points are given in world coordinates.
static void push_text(Geom *g, UmgText r, double x, double y, double x2,
                      double y2, const char *str, uint32_t len) {
  if (!g->origin_set) geom_origin(g, x, y);
  r.x = QX(g, x), r.y = QY(g, y), r.x2 = QX(g, x2), r.y2 = QY(g, y2);
  r.off = (uint32_t)g->pool.n, r.len = len;
  vec_append(&g->pool, str, len);
  *(UmgText *)vec_push(&g->texts) = r;
  *(uint32_t *)vec_push(&g->text_st) = g->cur;
}

static void add_text(Geom *g, const Dwg_Object *o, BITCODE_T str, int mtext,
                     UmgText r, double x, double y, double x2, double y2) {
  char *u = dwg_utf8(o, str);
  if (!u || r.h <= 0) {
    g->n_text_bad++;
    free(u);
    return;
  }
  Vec tmp = {NULL, 0, 0, 1};
  pool_clean(&tmp, u, mtext);
  free(u);
  if (tmp.n)
    push_text(g, r, x, y, x2, y2, (const char *)tmp.p, (uint32_t)tmp.n);
  free(tmp.p);
}

// TEXT and ATTRIB share their layout fields.
#define ADD_SINGLE_LINE(T)                                                    \
  do {                                                                        \
    UmgText r = {0};                                                          \
    r.h = (float)T->height, r.rot = (float)T->rotation;                       \
    r.wf = (float)(T->width_factor > 0 ? T->width_factor : 1);                \
    r.obl = (float)T->oblique_angle;                                          \
    r.ha = (uint8_t)T->horiz_alignment, r.va = (uint8_t)T->vert_alignment;    \
    add_text(g, o, T->text_value, 0, r, T->ins_pt.x, T->ins_pt.y,             \
             T->alignment_pt.x, T->alignment_pt.y);                           \
  } while (0)

static void convert_attrib(Geom *g, const Dwg_Object *o) {
  Dwg_Entity_ATTRIB *a = o->tio.entity->tio.ATTRIB;
  if (a->flags & 1) return;  // invisible
  g->cur = entity_style(o->tio.entity);
  ADD_SINGLE_LINE(a);
}

// MULTILEADER arrowhead at tip (x, y), pointing from (ux, uy) (unit) towards
// the tip, size a. Arrow block by name: none (_NONE), filled dot (_DOT...),
// tick (_OBLIQUE, _ARCHTICK); anything else as the default closed filled.
static void add_ml_arrow(Geom *g, const Dwg_Object *o, BITCODE_H blk, double x,
                         double y, double ux, double uy, double a) {
  if (!(a > 0)) return;
  char *nm = NULL;
  Dwg_Object *b = blk ? dwg_ref_object(o->parent, blk) : NULL;
  if (b && b->fixedtype == DWG_TYPE_BLOCK_HEADER)
    nm = dwg_utf8(o, b->tio.object->tio.BLOCK_HEADER->name);
  const char *n = nm ? nm : "";
  if (!strcasecmp(n, "_NONE")) {
  } else if (!strncasecmp(n, "_DOT", 4)) {
    double xy[2 * 16];  // 16-gon of radius a / 4, as a triangle fan
    uint32_t tri[3 * 14];
    for (int i = 0; i < 16; i++)
      xy[2 * i] = x + a / 4 * cos(i * M_PI / 8), xy[2 * i + 1] = y + a / 4 * sin(i * M_PI / 8);
    for (uint32_t i = 0; i < 14; i++) tri[3 * i] = 0, tri[3 * i + 1] = i + 1, tri[3 * i + 2] = i + 2;
    add_fill(g, XF_ID, xy, tri, 3 * 14);
  } else if (!strcasecmp(n, "_OBLIQUE") || !strcasecmp(n, "_ARCHTICK")) {
    double h = a / 2, c = M_SQRT1_2;  // 45 degrees to the leader
    double tx = (ux - uy) * c * h, ty = (uy + ux) * c * h;
    add_line(g, XF_ID, x - tx, y - ty, x + tx, y + ty);
  } else {
    double bx = x - ux * a, by = y - uy * a, w = a / 6;
    double xy[6] = {x, y, bx - uy * w, by + ux * w, bx + uy * w, by - ux * w};
    static const uint32_t tri[3] = {0, 1, 2};
    add_fill(g, XF_ID, xy, tri, 3);
  }
  free(nm);
}

// MULTILEADER: per leader line, tip -> vertices -> the node's connection
// point, then the dogleg; MTEXT content hangs from its location by the top
// edge (left/center/right from its alignment). Block content is not drawn.
static void add_multileader(Geom *g, const Dwg_Object *o) {
  const Dwg_Entity_MULTILEADER *m = o->tio.entity->tio.MULTILEADER;
  const Dwg_MLEADER_AnnotContext *x = &m->ctx;
  int r2010 = o->parent->header.from_version >= R_2010;
  for (uint32_t k = 0; k < x->num_leaders && x->leaders; k++) {
    const Dwg_LEADER_Node *nd = &x->leaders[k];
    for (uint32_t j = 0; j < nd->num_lines && nd->lines; j++) {
      const Dwg_LEADER_Line *l = &nd->lines[j];
      int type = r2010 && (l->flags & 1) ? l->type : m->type;
      if (type == 0 || !l->num_points || !l->points) continue;
      uint32_t n = l->num_points + 1;
      begin_poly(g, n, UMG_LTCONT);
      for (uint32_t i = 0; i < l->num_points; i++) add_pt(g, XF_ID, l->points[i].x, l->points[i].y, 0);
      add_pt(g, XF_ID, nd->lastleaderlinepoint.x, nd->lastleaderlinepoint.y, 0);
      double nx = l->num_points > 1 ? l->points[1].x : nd->lastleaderlinepoint.x;
      double ny = l->num_points > 1 ? l->points[1].y : nd->lastleaderlinepoint.y;
      double dx = l->points[0].x - nx, dy = l->points[0].y - ny, len = hypot(dx, dy);
      double a = r2010 && (l->flags & 16) ? l->arrow_size : x->arrow_size;
      BITCODE_H ah = r2010 && (l->flags & 32) ? l->arrow_handle : m->arrow_handle;
      if (len > a) add_ml_arrow(g, o, ah, l->points[0].x, l->points[0].y, dx / len, dy / len, a);
    }
    if (nd->has_dogleg && m->has_dogleg && nd->dogleg_length > 0)
      add_line(g, XF_ID, nd->lastleaderlinepoint.x, nd->lastleaderlinepoint.y,
               nd->lastleaderlinepoint.x + nd->dogleg_vector.x * nd->dogleg_length,
               nd->lastleaderlinepoint.y + nd->dogleg_vector.y * nd->dogleg_length);
  }
  if (x->has_content_txt) {
    const Dwg_MLEADER_Content_MText *t = &x->content.txt;
    UmgText r = {0};
    r.kind = 1;
    r.h = (float)(t->height > 0 ? t->height : x->text_height);
    r.rot = (float)atan2(t->direction.y, t->direction.x);
    r.wf = 1;
    r.att = (uint8_t)(t->alignment >= 1 && t->alignment <= 3 ? t->alignment : 1);
    r.width = (float)t->width;
    r.lsf = (float)(t->line_spacing_factor > 0 ? t->line_spacing_factor : 1);
    add_text(g, o, t->default_text, 1, r, t->location.x, t->location.y,
             t->location.x, t->location.y);
  } else if (x->has_content_blk) {
    g->n_skipped++;
  }
}

// ---- dimensions without a *D block ----------------------------------------
// AutoCAD normally stores a dimension's picture in an anonymous *D block and
// we draw that (blocks_add_dimension). Some files carry the dimension data
// only (Autodesk's own R2010 sample drawings: 44 dimensions, no *D blocks),
// so we draw it here from the definition points: extension lines, dimension
// line, arrows or ticks, and the measurement text. Style values come from
// the dimension's DIMSTYLE when it is already decoded, else from the header.
static unsigned long g_dim_drawn;  // dimensions drawn without a *D block

typedef struct {
  double asz, txt, scale, exo, exe, gap, tsz, lfac, dle;
  int dec, adec, lunit, zin;
  BITCODE_H blk;
} DimVars;

#define DIMVARS_FROM(V, S)                                                    \
  do {                                                                        \
    V.asz = S.DIMASZ, V.txt = S.DIMTXT, V.scale = S.DIMSCALE;                 \
    V.exo = S.DIMEXO, V.exe = S.DIMEXE, V.gap = S.DIMGAP, V.tsz = S.DIMTSZ;   \
    V.lfac = S.DIMLFAC, V.dle = S.DIMDLE, V.dec = S.DIMDEC;                   \
    V.adec = S.DIMADEC, V.lunit = S.DIMLUNIT, V.zin = S.DIMZIN;               \
    V.blk = S.DIMBLK;                                                         \
  } while (0)

static DimVars dim_vars(const Dwg_Object *o, BITCODE_H style) {
  DimVars v;
  Dwg_Data *dwg = o->parent;
  DIMVARS_FROM(v, dwg->header_vars);
  Dwg_Object *ds = style ? dwg_ref_object(dwg, style) : NULL;
  if (ds && ds->fixedtype == DWG_TYPE_DIMSTYLE && ds->tio.object &&
      ds->tio.object->tio.DIMSTYLE) {
    const Dwg_Object_DIMSTYLE *d = ds->tio.object->tio.DIMSTYLE;
    DIMVARS_FROM(v, (*d));
  }
  if (!(v.scale > 0)) v.scale = 1;
  if (!(v.lfac != 0)) v.lfac = 1;
  if (!(v.txt > 0)) v.txt = 2.5;
  if (!(v.asz >= 0)) v.asz = v.txt;
  if (v.dec < 0 || v.dec > 8) v.dec = 0;
  if (v.adec < 0 || v.adec > 8) v.adec = 0;
  return v;
}

// Number as decimal with dec places; DIMZIN 8 drops trailing zeros.
static void dim_decimal(char *out, size_t n, double v, int dec, int zin) {
  snprintf(out, n, "%.*f", dec, v);
  if ((zin & 8) && strchr(out, '.')) {
    char *e = out + strlen(out) - 1;
    while (*e == '0') *e-- = 0;
    if (*e == '.') *e = 0;
  }
  if (!strcmp(out, "-0")) strcpy(out, "0");
}

// Architectural units (DIMLUNIT 4): inches as 12'-6 1/2", fraction 1/2^dec.
static void dim_arch(char *out, size_t n, double v, int dec) {
  int den = 1 << (dec > 0 ? dec : 0);
  long t = lround(fabs(v) * den);  // in 1/den inch
  long ft = t / (12L * den), rem = t % (12L * den);
  long in = rem / den, fr = rem % den;
  while (fr && !(fr & 1)) fr >>= 1, den >>= 1;
  char frac[24] = "";
  if (fr) snprintf(frac, sizeof frac, " %ld/%d", fr, den);
  if (ft) snprintf(out, n, "%s%ld'-%ld%s\"", v < 0 ? "-" : "", ft, in, frac);
  else snprintf(out, n, "%s%ld%s\"", v < 0 ? "-" : "", in, frac);
}

static void dim_value(char *out, size_t n, const DimVars *v, double meas,
                      int angular) {
  if (angular) {
    char num[48];
    dim_decimal(num, sizeof num, meas * 180 / M_PI, v->adec, v->zin);
    snprintf(out, n, "%s%%%%d", num);
  } else if (v->lunit == 4) {
    dim_arch(out, n, meas * v->lfac, v->dec);
  } else {
    dim_decimal(out, n, meas * v->lfac, v->dec, v->zin);
  }
}

// The text shown: user text with "<>" replaced by the value; "" = value
// alone, " " = no text.
static char *dim_text(const Dwg_Object *o, BITCODE_T user, const char *val,
                      const char *prefix) {
  char *u = dwg_utf8(o, user);
  char *r;
  if (u && !strcmp(u, " ")) {
    r = NULL;
  } else if (!u || !*u) {
    size_t n = strlen(prefix) + strlen(val) + 1;
    r = malloc(n);
    snprintf(r, n, "%s%s", prefix, val);
  } else {
    char *at = strstr(u, "<>");
    if (!at) {
      r = strdup(u);
    } else {
      size_t n = strlen(u) + strlen(prefix) + strlen(val) + 1;
      r = malloc(n);
      snprintf(r, n, "%.*s%s%s%s", (int)(at - u), u, prefix, val, at + 2);
    }
  }
  free(u);
  return r;
}

// Reading direction: angles that would print upside down turn by 180.
static double dim_readable(double a) {
  a = remainder(a, 2 * M_PI);
  if (a > M_PI / 2 + 1e-9) a -= M_PI;
  else if (a <= -M_PI / 2 + 1e-9) a += M_PI;
  return a;
}

// Arrow (or tick when DIMTSZ > 0) at (x, y), (ux, uy) the unit direction
// pointing into the tip.
static void dim_arrow(Geom *g, const Dwg_Object *o, const DimVars *v,
                      double x, double y, double ux, double uy) {
  double s = v->scale;
  if (v->tsz > 0) {
    // DIMTSZ is the stroke's reach along and across the line (public
    // drawings' *D blocks: half-length DIMTSZ * sqrt 2).
    double h = v->tsz * s;
    double tx = (ux - uy) * h, ty = (uy + ux) * h;
    add_line(g, XF_ID, x - tx, y - ty, x + tx, y + ty);
  } else {
    add_ml_arrow(g, o, v->blk, x, y, ux, uy, v->asz * s);
  }
}

static void dim_put_text(Geom *g, const Dwg_Object *o, const char *str,
                         double h, double rot, double x, double y) {
  if (!str || !*str) return;
  Vec tmp = {NULL, 0, 0, 1};
  pool_clean(&tmp, str, 1);
  if (tmp.n) {
    UmgText r = {0};
    r.h = (float)h, r.rot = (float)rot, r.wf = 1;
    r.ha = 1, r.va = 2;  // middle center on text_midpt
    push_text(g, r, x, y, x, y, (const char *)tmp.p, (uint32_t)tmp.n);
  }
  free(tmp.p);
}

// Linear / aligned: dim line through def_pt at angle `ang`.
static void dim_linear(Geom *g, const Dwg_Object *o, const DimVars *v,
                       const Dwg_DIMENSION_common *d, double x1, double y1,
                       double x2, double y2, double ang, double *mx, double *my) {
  double ux = cos(ang), uy = sin(ang), nx = -uy, ny = ux;
  double px = d->def_pt.x, py = d->def_pt.y;
  // Feet of the extension lines on the dimension line.
  double t1 = (x1 - px) * ux + (y1 - py) * uy, t2 = (x2 - px) * ux + (y2 - py) * uy;
  double a1x = px + ux * t1, a1y = py + uy * t1, a2x = px + ux * t2, a2y = py + uy * t2;
  double s = v->scale;
  // Extension lines: from DIMEXO off the object to DIMEXE past the dim line.
  double xs[2] = {x1, x2}, ys[2] = {y1, y2}, ax[2] = {a1x, a2x}, ay[2] = {a1y, a2y};
  for (int i = 0; i < 2; i++) {
    double dx = ax[i] - xs[i], dy = ay[i] - ys[i], len = hypot(dx, dy);
    if (len < 1e-9) continue;
    dx /= len, dy /= len;
    double o0 = fmin(v->exo * s, len);
    add_line(g, XF_ID, xs[i] + dx * o0, ys[i] + dy * o0, ax[i] + dx * v->exe * s,
             ay[i] + dy * v->exe * s);
  }
  *mx = (a1x + a2x) / 2, *my = (a1y + a2y) / 2;
  double len = hypot(a2x - a1x, a2y - a1y);
  if (len < 1e-9) return;
  double dx = (a2x - a1x) / len, dy = (a2y - a1y) / len;
  double ext = v->tsz > 0 ? v->dle * s : 0;
  add_line(g, XF_ID, a1x - dx * ext, a1y - dy * ext, a2x + dx * ext, a2y + dy * ext);
  if (len > 2 * v->asz * s || v->tsz > 0) {  // arrows inside
    dim_arrow(g, o, v, a1x, a1y, -dx, -dy);
    dim_arrow(g, o, v, a2x, a2y, dx, dy);
  } else {  // too short: arrows outside, pointing in
    dim_arrow(g, o, v, a1x, a1y, dx, dy);
    dim_arrow(g, o, v, a2x, a2y, -dx, -dy);
  }
  (void)nx, (void)ny;
}

// Extension line from the measured point (px, py) to (ax, ay) on the
// dimension line or arc: DIMEXO short of the point, DIMEXE past the end.
static void dim_ext(Geom *g, const DimVars *v, double px, double py,
                    double ax, double ay) {
  double dx = ax - px, dy = ay - py, len = hypot(dx, dy), s = v->scale;
  if (len < 1e-9) return;
  dx /= len, dy /= len;
  double o0 = fmin(v->exo * s, len);
  add_line(g, XF_ID, px + dx * o0, py + dy * o0, ax + dx * v->exe * s,
           ay + dy * v->exe * s);
}

// Angular dimension arc around (cx, cy), radius r, counter-clockwise from
// a0 to a1, with its arrows; *mid = the arc's middle angle.
static void dim_arc(Geom *g, const Dwg_Object *o, const DimVars *v, double cx,
                    double cy, double r, double a0, double a1, double *mid) {
  if (a1 <= a0) a1 = a0 + fmod(a1 - a0, 2 * M_PI) + 2 * M_PI;
  *mid = (a0 + a1) / 2;
  add_arc(g, XF_ID, cx, cy, r, a0, a1);
  // Each arrow lies along the chord from the tip to the arc point one arrow
  // length further on (inside) or back (outside), as AutoCAD draws it.
  double L = (v->tsz > 0 ? v->tsz : v->asz) * v->scale;
  double phi = L > 0 ? 2 * asin(fmin(1, L / (2 * r))) : 1e-6;
  int inside = v->tsz > 0 || r * (a1 - a0) > 2 * L;
  double ends[2] = {a0, a1}, toward[2] = {inside ? phi : -phi, inside ? -phi : phi};
  for (int i = 0; i < 2; i++) {
    double a = ends[i], b = a + toward[i];
    double tx = cx + r * cos(a), ty = cy + r * sin(a);
    double dx = r * (cos(a) - cos(b)), dy = r * (sin(a) - sin(b)), n = hypot(dx, dy);
    if (n > 0) dim_arrow(g, o, v, tx, ty, dx / n, dy / n);
  }
}

// Angle a moved into [b, b + 2 pi).
static double ang_from(double a, double b) {
  a = fmod(a - b, 2 * M_PI);
  return a < 0 ? a + 2 * M_PI : a;
}

// 3-point angular: vertex c, measured points p1 and p2, arc through `at`.
// The arc spans the side of the two rays that holds `at` (may exceed 180).
static int dim_ang3pt(Geom *g, const Dwg_Object *o, const DimVars *v,
                      const Dwg_Entity_DIMENSION_ANG3PT *e, double *cx,
                      double *cy, double *r, double *mid) {
  *cx = e->center_pt.x, *cy = e->center_pt.y;
  double ax = e->def_pt.x - *cx, ay = e->def_pt.y - *cy;
  *r = hypot(ax, ay);
  double p[2][2] = {{e->xline1_pt.x, e->xline1_pt.y}, {e->xline2_pt.x, e->xline2_pt.y}};
  if (*r < 1e-9 || hypot(p[0][0] - *cx, p[0][1] - *cy) < 1e-9 ||
      hypot(p[1][0] - *cx, p[1][1] - *cy) < 1e-9)
    return 0;
  double a1 = atan2(p[0][1] - *cy, p[0][0] - *cx), a2 = atan2(p[1][1] - *cy, p[1][0] - *cx);
  int swap = ang_from(atan2(ay, ax), a1) > ang_from(a2, a1);
  double b0 = swap ? a2 : a1, b1 = swap ? a1 : a2;
  for (int i = 0; i < 2; i++) {
    double a = i == 0 ? a1 : a2;
    dim_ext(g, v, p[i][0], p[i][1], *cx + *r * cos(a), *cy + *r * sin(a));
  }
  dim_arc(g, o, v, *cx, *cy, *r, b0, b1, mid);
  return 1;
}

// 2-line angular: lines (13 -> 14) and (15 -> 10), arc through DXF 16
// around their intersection, in the quadrant of the four the lines make
// that holds that point. R13+ files store 16 first (a 2RD) and 10 last:
// LibreDWG reads them into def_pt and xline2end_pt, the other way round from
// their DXF names (public drawings: the vertex came out right only so).
static int dim_ang2ln(Geom *g, const Dwg_Object *o, const DimVars *v,
                      const Dwg_DIMENSION_common *d,
                      const Dwg_Entity_DIMENSION_ANG2LN *e, double *cx,
                      double *cy, double *r, double *mid) {
  int r13 = o->parent->header.version >= R_13b1;
  BITCODE_3BD end2 = r13 ? e->xline2end_pt : d->def_pt;
  BITCODE_3BD arc = r13 ? d->def_pt : e->xline2end_pt;
  double q[2][2][2] = {{{e->xline1start_pt.x, e->xline1start_pt.y}, {e->xline1end_pt.x, e->xline1end_pt.y}},
                       {{e->xline2start_pt.x, e->xline2start_pt.y}, {end2.x, end2.y}}};
  double u[2][2];
  for (int i = 0; i < 2; i++) {
    double dx = q[i][1][0] - q[i][0][0], dy = q[i][1][1] - q[i][0][1], len = hypot(dx, dy);
    if (len < 1e-9) return 0;
    u[i][0] = dx / len, u[i][1] = dy / len;
  }
  double cr = u[0][0] * u[1][1] - u[0][1] * u[1][0];
  if (fabs(cr) < 1e-9) return 0;  // parallel lines
  double wx = q[1][0][0] - q[0][0][0], wy = q[1][0][1] - q[0][0][1];
  double t = (wx * u[1][1] - wy * u[1][0]) / cr;  // along line 1
  *cx = q[0][0][0] + u[0][0] * t, *cy = q[0][0][1] + u[0][1] * t;
  double px = arc.x - *cx, py = arc.y - *cy;
  *r = hypot(px, py);
  if (*r < 1e-9) return 0;
  // Ray signs: the arc point lies between rays s1*u1 and s2*u2.
  double s1 = (px * u[1][1] - py * u[1][0]) * cr >= 0 ? 1 : -1;
  double s2 = (u[0][0] * py - u[0][1] * px) * cr >= 0 ? 1 : -1;
  double r1x = s1 * u[0][0], r1y = s1 * u[0][1], r2x = s2 * u[1][0], r2y = s2 * u[1][1];
  double a1 = atan2(r1y, r1x), a2 = atan2(r2y, r2x);
  int ccw = r1x * r2y - r1y * r2x > 0;
  double rays[2][2] = {{r1x, r1y}, {r2x, r2y}};
  for (int i = 0; i < 2; i++) {  // extension lines only off the line's ends
    double ta = (q[i][0][0] - *cx) * rays[i][0] + (q[i][0][1] - *cy) * rays[i][1];
    double tb = (q[i][1][0] - *cx) * rays[i][0] + (q[i][1][1] - *cy) * rays[i][1];
    double lo = fmin(ta, tb), hi = fmax(ta, tb), from = *r > hi ? hi : *r < lo ? lo : *r;
    if (from == *r) continue;
    dim_ext(g, v, *cx + rays[i][0] * from, *cy + rays[i][1] * from,
            *cx + rays[i][0] * *r, *cy + rays[i][1] * *r);
  }
  dim_arc(g, o, v, *cx, *cy, *r, ccw ? a1 : a2, ccw ? a2 : a1, mid);
  return 1;
}

static void add_dimension(Geom *g, const Dwg_Object *o) {
  const Dwg_DIMENSION_common *d = o->tio.entity->tio.DIMENSION_LINEAR
                                      ? (const Dwg_DIMENSION_common *)o->tio.entity->tio.DIMENSION_LINEAR
                                      : NULL;
  if (!d) return;
  DimVars v = dim_vars(o, d->dimstyle);
  // Annotative dimensions keep paper sizes (1/8") and are scaled by the
  // annotation scale, which lives in extension dictionaries we do not read.
  // Keep the text at least 1/200 of the drawing so it stays readable.
  // Only for unscaled styles (DIMSCALE 1, as annotative ones are) whose
  // text is clearly paper-sized (Autodesk samples: 7 to 58 times too
  // small): a real DIMSCALE (public drawings: 50) is right, and extents can
  // be far larger than the sheet, which blew arrows up many times.
  if (v.scale <= 1) {
    Dwg_Bitcode_3BD lo = o->parent->header_vars.EXTMIN, hi = o->parent->header_vars.EXTMAX;
    double span = fmax(hi.x - lo.x, hi.y - lo.y);
    double want = isfinite(span) && span > 0 && span < 1e12 ? span / 200 : 0;
    if (want > 5 * v.txt * v.scale) v.scale = want / v.txt;
  }
  double h = v.txt * v.scale, rot = 0;
  double mx = d->text_midpt.x, my = d->text_midpt.y;  // default text spot
  int place = d->text_midpt.x == 0 && d->text_midpt.y == 0;  // not stored
  char val[64];
  const char *prefix = "";
  int angular = 0;
  double outx = 0, outy = 0;  // angular: away from the vertex at the arc middle
  switch (o->fixedtype) {
  case DWG_TYPE_DIMENSION_LINEAR: {
    const Dwg_Entity_DIMENSION_LINEAR *l = o->tio.entity->tio.DIMENSION_LINEAR;
    rot = l->dim_rotation;
    dim_linear(g, o, &v, d, l->xline1_pt.x, l->xline1_pt.y, l->xline2_pt.x,
               l->xline2_pt.y, rot, &mx, &my);
  } break;
  case DWG_TYPE_DIMENSION_ALIGNED: {
    const Dwg_Entity_DIMENSION_ALIGNED *l = o->tio.entity->tio.DIMENSION_ALIGNED;
    rot = atan2(l->xline2_pt.y - l->xline1_pt.y, l->xline2_pt.x - l->xline1_pt.x);
    dim_linear(g, o, &v, d, l->xline1_pt.x, l->xline1_pt.y, l->xline2_pt.x,
               l->xline2_pt.y, rot, &mx, &my);
  } break;
  case DWG_TYPE_DIMENSION_RADIUS: {
    const Dwg_Entity_DIMENSION_RADIUS *r = o->tio.entity->tio.DIMENSION_RADIUS;
    double cx = r->def_pt.x, cy = r->def_pt.y, ex = r->first_arc_pt.x, ey = r->first_arc_pt.y;
    double len = hypot(ex - cx, ey - cy);
    if (len > 1e-9) {
      add_line(g, XF_ID, cx, cy, ex, ey);
      dim_arrow(g, o, &v, ex, ey, (ex - cx) / len, (ey - cy) / len);
    }
    rot = atan2(ey - cy, ex - cx);
    mx = (cx + ex) / 2, my = (cy + ey) / 2;
    prefix = "R";
  } break;
  case DWG_TYPE_DIMENSION_DIAMETER: {
    const Dwg_Entity_DIMENSION_DIAMETER *r = o->tio.entity->tio.DIMENSION_DIAMETER;
    double ax = r->def_pt.x, ay = r->def_pt.y, bx = r->first_arc_pt.x, by = r->first_arc_pt.y;
    double len = hypot(bx - ax, by - ay);
    if (len > 1e-9) {
      double ux = (bx - ax) / len, uy = (by - ay) / len;
      add_line(g, XF_ID, ax, ay, bx, by);
      dim_arrow(g, o, &v, ax, ay, -ux, -uy);
      dim_arrow(g, o, &v, bx, by, ux, uy);
    }
    rot = atan2(by - ay, bx - ax);
    mx = (ax + bx) / 2, my = (ay + by) / 2;
    prefix = "%%c";
  } break;
  case DWG_TYPE_DIMENSION_ANG3PT:
  case DWG_TYPE_DIMENSION_ANG2LN: {
    double cx, cy, r, mid;
    int ok = o->fixedtype == DWG_TYPE_DIMENSION_ANG3PT
                 ? dim_ang3pt(g, o, &v, o->tio.entity->tio.DIMENSION_ANG3PT, &cx, &cy, &r, &mid)
                 : dim_ang2ln(g, o, &v, d, o->tio.entity->tio.DIMENSION_ANG2LN, &cx, &cy, &r, &mid);
    angular = ok ? 2 : 1;  // 1: degenerate, text only
    if (ok) {  // unstored text: just outside the arc's middle
      outx = cos(mid), outy = sin(mid);
      mx = cx + r * outx, my = cy + r * outy;
    }
  } break;
  default:  // ordinate: text only
    break;
  }
  dim_value(val, sizeof val, &v, d->act_measurement, angular);
  char *t = dim_text(o, d->user_text, val, prefix);
  double trot = d->text_rotation != 0 ? d->text_rotation : dim_readable(rot);
  if (angular || o->fixedtype == DWG_TYPE_DIMENSION_ORDINATE) trot = d->text_rotation;
  if (place) {  // above the dimension line (DIMTAD 1 look), else as stored
    if (angular == 1 || o->fixedtype == DWG_TYPE_DIMENSION_ORDINATE) { free(t); g_dim_drawn++; return; }
    double off = h / 2 + v.gap * v.scale + h * 0.2;
    if (angular) mx += outx * off, my += outy * off;
    else mx += -sin(trot) * off, my += cos(trot) * off;
  } else {
    mx = d->text_midpt.x, my = d->text_midpt.y;
  }
  dim_put_text(g, o, t, h, trot, mx, my);
  free(t);
  g_dim_drawn++;
}

// ---- entities -------------------------------------------------------------

// Types whose geometry is fully contained in the object itself, so the
// streaming decoder may free them right after conversion.
static int is_self_contained(const Dwg_Object *o) {
  switch (o->fixedtype) {
  case DWG_TYPE_LINE: case DWG_TYPE_ARC: case DWG_TYPE_CIRCLE:
  case DWG_TYPE_LWPOLYLINE: case DWG_TYPE_SPLINE: case DWG_TYPE_ELLIPSE:
  case DWG_TYPE_TEXT: case DWG_TYPE_MTEXT: case DWG_TYPE_POINT:
  case DWG_TYPE_XLINE: case DWG_TYPE_RAY: case DWG_TYPE__3DFACE:
  case DWG_TYPE_SOLID: case DWG_TYPE_TRACE: case DWG_TYPE_HATCH:
  case DWG_TYPE_WIPEOUT: case DWG_TYPE_DIMENSION_LINEAR:
  case DWG_TYPE_DIMENSION_ALIGNED: case DWG_TYPE_LEADER:
    return 1;
  default:
    return 0;
  }
}

// Entity types we do not draw, counted by name for the app's drawing info
// (D86): "3DSOLID", "IMAGE", ... Block contents count once per insert.
typedef struct { const char *name; unsigned long n; } Tally;
static Tally g_unsup[32];
static int g_n_unsup;
static unsigned long g_unsup_other;

static void unsupported_tally(const char *name) {
  if (!name) name = "?";
  // Not drawn on purpose or drawn another way: block markers, attribute
  // definitions, paper-space viewports, dimensions (their *D block),
  // polyline vertices and sequence ends, block references.
  static const char *skip[] = {"BLOCK", "ENDBLK", "SEQEND", "ATTDEF", "VIEWPORT",
                               "INSERT", "MINSERT"};
  for (size_t i = 0; i < sizeof skip / sizeof *skip; i++)
    if (!strcmp(name, skip[i])) return;
  if (!strncmp(name, "DIMENSION", 9) || !strncmp(name, "VERTEX", 6)) return;
  for (int i = 0; i < g_n_unsup; i++)
    if (!strcmp(g_unsup[i].name, name)) { g_unsup[i].n++; return; }
  if (g_n_unsup < 32) g_unsup[g_n_unsup++] = (Tally){name, 1};
  else g_unsup_other++;
}

static void convert_entity(Geom *g, const Dwg_Object *o) {
  Dwg_Object_Entity *e = o->tio.entity;
  g->cur = entity_style(e);
  OrdKey k = {g->model ? o->handle.value : 0, g->model ? 0 : o->handle.value};
  g->ord = k;
  switch (o->fixedtype) {
  case DWG_TYPE_LINE: {
    Dwg_Entity_LINE *l = e->tio.LINE;
    add_line(g, XF_ID, l->start.x, l->start.y, l->end.x, l->end.y);
  } break;
  case DWG_TYPE_ARC: {
    Dwg_Entity_ARC *a = e->tio.ARC;
    add_arc(g, a->extrusion.z < 0 ? XF_MIRROR_X : XF_ID, a->center.x,
            a->center.y, a->radius, a->start_angle, a->end_angle);
  } break;
  case DWG_TYPE_CIRCLE: {
    Dwg_Entity_CIRCLE *c = e->tio.CIRCLE;
    add_arc(g, c->extrusion.z < 0 ? XF_MIRROR_X : XF_ID, c->center.x,
            c->center.y, c->radius, 0, 2 * M_PI);
  } break;
  case DWG_TYPE_LWPOLYLINE: {
    Dwg_Entity_LWPOLYLINE *p = e->tio.LWPOLYLINE;
    uint32_t n = p->points ? p->num_points : 0;
    if (n < 2) break;
    double *xy = (double *)malloc(n * 2 * sizeof(double));
    double *bul = (double *)calloc(n, sizeof(double));
    for (uint32_t i = 0; i < n; i++) {
      xy[2 * i] = p->points[i].x, xy[2 * i + 1] = p->points[i].y;
      if (p->bulges && i < p->num_bulges) bul[i] = p->bulges[i];
    }
    uint32_t flags = ((p->flag & 512) && poly_closes(n, bul) ? UMG_CLOSED : 0) |
                     (p->flag & 128 ? UMG_LTCONT : 0);  // plinegen
    add_bulged_poly(g, p->extrusion.z < 0 ? XF_MIRROR_X : XF_ID, xy, bul, n,
                    flags);
    int wide = p->const_width > 0;
    uint32_t nw = p->widths ? p->num_widths : 0;
    for (uint32_t i = 0; i < nw && !wide; i++)
      wide = p->widths[i].start > 0 || p->widths[i].end > 0;
    if (wide) {
      double *w0 = (double *)malloc(n * sizeof(double));
      double *w1 = (double *)malloc(n * sizeof(double));
      for (uint32_t i = 0; i < n; i++) {
        int own = p->const_width <= 0 && i < nw;
        w0[i] = own ? p->widths[i].start : p->const_width;
        w1[i] = own ? p->widths[i].end : p->const_width;
      }
      add_wide_poly(g, p->extrusion.z < 0 ? XF_MIRROR_X : XF_ID, xy, bul, w0,
                    w1, n, (flags & UMG_CLOSED) != 0);
      free(w0), free(w1);
    }
    free(xy), free(bul);
  } break;
  case DWG_TYPE_SPLINE:
    add_spline(g, e->tio.SPLINE);
    break;
  case DWG_TYPE_ELLIPSE: {
    Dwg_Entity_ELLIPSE *el = e->tio.ELLIPSE;
    double mx = el->sm_axis.x, my = el->sm_axis.y;
    double R = hypot(mx, my), r = R * el->axis_ratio, rot = atan2(my, mx);
    double a0 = el->start_angle, a1 = el->end_angle;
    if (a1 < a0) a1 += 2 * M_PI;
    int n = umg_arc_segs(a1 - a0);
    begin_poly(g, n + 1, UMG_LTCONT);
    for (int i = 0; i <= n; i++) {
      double a = a0 + (a1 - a0) * i / n;
      double lx = R * cos(a), ly = r * sin(a);
      add_pt(g, XF_ID, el->center.x + lx * cos(rot) - ly * sin(rot),
             el->center.y + lx * sin(rot) + ly * cos(rot), 0);
    }
  } break;
  case DWG_TYPE_TEXT: {
    Dwg_Entity_TEXT *t = e->tio.TEXT;
    ADD_SINGLE_LINE(t);
  } break;
  case DWG_TYPE_MTEXT: {
    Dwg_Entity_MTEXT *mt = e->tio.MTEXT;
    UmgText r = {0};
    r.kind = 1;
    r.h = (float)mt->text_height;
    r.rot = (float)atan2(mt->x_axis_dir.y, mt->x_axis_dir.x);
    r.wf = 1;
    r.att = (uint8_t)mt->attachment;
    r.width = (float)mt->rect_width;
    r.lsf = (float)(mt->linespace_factor > 0 ? mt->linespace_factor : 1);
    char *u = dwg_utf8(o, mt->text);
    int64_t col = u ? mtext_inline_color(u) : -1;
    free(u);
    if (col >= 0) {
      RawStyle k = g_styles.v[g->cur];
      k.color = (uint32_t)col;
      g->cur = style_intern(k);
    }
    add_text(g, o, mt->text, 1, r, mt->ins_pt.x, mt->ins_pt.y, mt->ins_pt.x,
             mt->ins_pt.y);
  } break;
  case DWG_TYPE_HATCH:
    add_hatch(g, e->tio.HATCH);
    break;
  case DWG_TYPE_SOLID:
    add_solid(g, e->tio.SOLID);
    break;
  case DWG_TYPE_TRACE:  // same fields as SOLID
    add_solid(g, (const Dwg_Entity_SOLID *)e->tio.TRACE);
    break;
  case DWG_TYPE_WIPEOUT:
    add_wipeout(g, e->tio.WIPEOUT);
    break;
  case DWG_TYPE_POLYLINE_2D:
    add_polyline_2d(g, o);
    break;
  case DWG_TYPE_POLYLINE_3D:
    add_polyline_3d(g, o);
    break;
  case DWG_TYPE_LEADER:
    add_leader(g, o);
    break;
  case DWG_TYPE_MLINE:
    add_mline(g, o);
    break;
  case DWG_TYPE_POINT:
    add_point(g, o);
    break;
  case DWG_TYPE_XLINE:
  case DWG_TYPE_RAY:
    add_xline(g, o, o->fixedtype == DWG_TYPE_XLINE);
    break;
  case DWG_TYPE__3DFACE:
    add_3dface(g, e->tio._3DFACE);
    break;
  case DWG_TYPE_MULTILEADER:  // not streamed: its arrow blocks are resolved
    add_multileader(g, o);
    break;
  default:
    g->n_skipped++;
    unsupported_tally(o->name);
  }
}

// ---- layer and linetype tables -------------------------------------------

typedef struct {
  uint64_t h, ltype;  // LAYER handle, its LTYPE handle
  char *name;
  uint32_t rgb, flags;  // resolved colour, UMG_LAYER_*
} LayerInfo;

typedef struct {
  uint64_t h;
  char *name;
  float *dash;  // > 0 dash, < 0 gap, 0 dot
  uint32_t n;
  float len;
} LtypeInfo;

typedef struct {
  LayerInfo *layers;
  size_t n_layers, cap_layers;
  LtypeInfo *ltypes;
  size_t n_ltypes, cap_ltypes;
  uint64_t layer0;  // handle of layer "0" (block entities on it inherit)
  double ltscale;   // header LTSCALE
  uint32_t units;   // header INSUNITS, into UmgHeader.units (D80)
  Vec sorts;        // uint64[2]: entity handle, its SORTENTSTABLE sort handle
} Tables;

static uint32_t aci_rgb(uint32_t i) {
  const Dwg_RGB_Palette *p = dwg_rgb_palette();
  if (i == 7 || i > 255) return 0xffffff;  // black/white: white on our dark
  return (uint32_t)p[i].r << 16 | (uint32_t)p[i].g << 8 | p[i].b;
}

static uint32_t color_rgb(uint32_t raw) {
  switch (raw & 0xff000000u) {
  case COL_BG: return UMG_RGB_BACKGROUND;
  case COL_TRUE: return raw & 0xffffff;
  case COL_ACI: return aci_rgb(raw & 0xff);
  default: return 0xffffff;
  }
}

// Collects LAYER and LTYPE table entries (call for every decoded object).
static void tables_add(Tables *t, const Dwg_Object *o) {
  if (o->fixedtype == DWG_TYPE_SORTENTSTABLE) {
    const Dwg_Object_SORTENTSTABLE *st = o->tio.object->tio.SORTENTSTABLE;
    t->sorts.elem = 2 * sizeof(uint64_t);
    for (BITCODE_BL k = 0; k < st->num_ents; k++) {
      if (!st->ents || !st->sort_ents || !st->ents[k] || !st->sort_ents[k]) continue;
      uint64_t *p = (uint64_t *)vec_push(&t->sorts);
      p[0] = st->ents[k]->absolute_ref, p[1] = st->sort_ents[k]->absolute_ref;
    }
  } else if (o->fixedtype == DWG_TYPE_LAYER) {
    Dwg_Object_LAYER *l = o->tio.object->tio.LAYER;
    if (t->n_layers == t->cap_layers) {
      t->cap_layers = t->cap_layers ? t->cap_layers * 2 : 64;
      t->layers = (LayerInfo *)realloc(t->layers, t->cap_layers * sizeof(LayerInfo));
    }
    LayerInfo *L = &t->layers[t->n_layers++];
    L->h = o->handle.value;
    L->ltype = ref_handle(l->ltype);
    L->name = dwg_utf8(o, l->name);
    if (!L->name) L->name = strdup("?");
    int has_rgb = o->parent->header.from_version >= R_2004;
    uint32_t c = raw_color(&l->color, has_rgb);
    L->rgb = c == COL_BYLAYER || c == COL_BYBLOCK ? 0xffffff : color_rgb(c);
    L->flags = (l->off || l->color.index < 0 ? UMG_LAYER_OFF : 0) |
               (l->frozen ? UMG_LAYER_FROZEN : 0);
    if (!strcmp(L->name, "0")) t->layer0 = L->h;
  } else if (o->fixedtype == DWG_TYPE_LTYPE) {
    Dwg_Object_LTYPE *lt = o->tio.object->tio.LTYPE;
    if (t->n_ltypes == t->cap_ltypes) {
      t->cap_ltypes = t->cap_ltypes ? t->cap_ltypes * 2 : 16;
      t->ltypes = (LtypeInfo *)realloc(t->ltypes, t->cap_ltypes * sizeof(LtypeInfo));
    }
    LtypeInfo *T = &t->ltypes[t->n_ltypes++];
    memset(T, 0, sizeof *T);
    T->h = o->handle.value;
    T->name = dwg_utf8(o, lt->name);
    if (!T->name) T->name = strdup("?");
    uint32_t n = lt->numdashes;
    if (n > 12) n = 12;
    double len = 0;
    T->dash = (float *)malloc((n ? n : 1) * sizeof(float));
    for (uint32_t k = 0; k < n; k++) {
      double d = lt->dashes ? lt->dashes[k].length : lt->dashes_r11[k];
      T->dash[k] = (float)d, len += fabs(d);
    }
    // Patterns without a gap, or degenerate ones, draw as continuous.
    int gap = 0;
    for (uint32_t k = 0; k < n; k++) gap |= T->dash[k] < 0;
    T->n = gap && len > 0 ? n : 0;
    T->len = (float)len;
  }
}

static void tables_free(Tables *t) {
  for (size_t k = 0; k < t->n_layers; k++) free(t->layers[k].name);
  for (size_t k = 0; k < t->n_ltypes; k++)
    free(t->ltypes[k].name), free(t->ltypes[k].dash);
  free(t->layers), free(t->ltypes), free(t->sorts.p);
}

static int layer_index(const Tables *t, uint64_t h) {
  for (size_t k = 0; k < t->n_layers; k++)
    if (t->layers[k].h == h) return (int)k;
  return -1;
}

static int ltype_index(const Tables *t, uint64_t h) {
  for (size_t k = 0; k < t->n_ltypes; k++)
    if (t->ltypes[k].h == h) return t->ltypes[k].n ? (int)k : -1;
  return -1;
}

// Final style of a model-space raw style. ByBlock at the top level draws as
// white / continuous, like AutoCAD.
static UmgStyle style_resolve(const Tables *t, RawStyle k) {
  UmgStyle r;
  int li = layer_index(t, k.layer);
  if (li < 0) li = layer_index(t, t->layer0);
  if (li < 0) li = 0;
  const LayerInfo *L = &t->layers[li];
  r.layer = (uint16_t)li;
  r.rgb = k.color == COL_BYLAYER ? L->rgb : color_rgb(k.color);
  uint64_t lt = k.ltype == LT_BYLAYER ? L->ltype : k.ltype;
  int ti = lt == LT_BYBLOCK || lt == LT_CONT ? -1 : ltype_index(t, lt);
  r.ltype = ti < 0 ? UMG_NO_LTYPE : (uint16_t)ti;
  r.lt_scale = ti < 0 ? 1 : (float)(k.scale * t->ltscale);
  return r;
}

// ---- draw order -----------------------------------------------------------

static int cmp_u64x2(const void *a, const void *b) {
  const uint64_t *p = (const uint64_t *)a, *q = (const uint64_t *)b;
  if (p[0] != q[0]) return p[0] < q[0] ? -1 : 1;
  return p[1] < q[1] ? -1 : p[1] > q[1];
}

// Sort handle of h: its SORTENTSTABLE entry, else h itself (AutoCAD draws
// in handle order unless the table says otherwise). sorts must be sorted.
static uint64_t sort_handle(const Vec *sorts, uint64_t h) {
  const uint64_t *v = (const uint64_t *)sorts->p;
  size_t lo = 0, hi = sorts->n;
  while (lo < hi) {
    size_t mid = (lo + hi) / 2;
    if (v[2 * mid] < h) lo = mid + 1;
    else hi = mid;
  }
  return lo < sorts->n && v[2 * lo] == h ? v[2 * lo + 1] : h;
}

typedef struct { uint64_t a, b; uint32_t i; } OrdItem;

static int cmp_ord(const void *x, const void *y) {
  const OrdItem *p = (const OrdItem *)x, *q = (const OrdItem *)y;
  if (p->a != q->a) return p->a < q->a ? -1 : 1;
  if (p->b != q->b) return p->b < q->b ? -1 : 1;
  return p->i < q->i ? -1 : p->i > q->i;
}

// Dense ranks (from 1) of the fills' and hatch lines' order keys.
static void ord_ranks(const Geom *g, Tables *t, Vec *fill_ord, Vec *hline_ord,
                      uint32_t *n_ord) {
  if (t->sorts.n) qsort(t->sorts.p, t->sorts.n, t->sorts.elem, cmp_u64x2);
  size_t nf = g->fill_key.n, n = nf + g->hline_key.n;
  OrdItem *it = (OrdItem *)malloc((n ? n : 1) * sizeof(OrdItem));
  for (size_t i = 0; i < n; i++) {
    const OrdKey *k = i < nf ? (const OrdKey *)g->fill_key.p + i
                             : (const OrdKey *)g->hline_key.p + (i - nf);
    it[i].a = sort_handle(&t->sorts, k->root);
    it[i].b = k->inner ? sort_handle(&t->sorts, k->inner) : 0;
    it[i].i = (uint32_t)i;
  }
  qsort(it, n, sizeof(OrdItem), cmp_ord);
  vec_reserve(fill_ord, nf), vec_reserve(hline_ord, n - nf);
  fill_ord->n = nf, hline_ord->n = n - nf;
  uint32_t r = 0;
  for (size_t j = 0; j < n; j++) {
    if (!j || it[j].a != it[j - 1].a || it[j].b != it[j - 1].b) r++;
    uint32_t *dst = it[j].i < nf ? (uint32_t *)fill_ord->p + it[j].i
                                 : (uint32_t *)hline_ord->p + (it[j].i - nf);
    *dst = r;
  }
  *n_ord = r;
  free(it);
}

// ---- UMG8 image -----------------------------------------------------------

// Raw (uncompressed) UMG8 image of g with resolved styles and the tables.
static char *geom_serialize(const Geom *g, Tables *t, size_t *size,
                            unsigned long *n_style_overflow) {
  if (!t->n_layers) {  // no layer table (should not happen): one layer "0"
    LayerInfo L = {0, LT_CONT, strdup("0"), 0xffffff, 0};
    t->layers = (LayerInfo *)malloc(sizeof L), t->cap_layers = 1;
    t->layers[t->n_layers++] = L;
  }
  // Raw style id -> final style index, resolved on first use and deduped.
  uint32_t *map = (uint32_t *)malloc((g_styles.n ? g_styles.n : 1) * 4);
  memset(map, 0xff, (g_styles.n ? g_styles.n : 1) * 4);
  Vec styles = {NULL, 0, 0, sizeof(UmgStyle)};
  const Vec *sts[] = {&g->line_st, &g->arc_st, &g->poly_st, &g->text_st,
                       &g->fill_st, &g->hline_st};
  uint16_t *idx[6];
  for (int a = 0; a < 6; a++) {
    idx[a] = (uint16_t *)malloc((sts[a]->n ? sts[a]->n : 1) * 2);
    for (size_t i = 0; i < sts[a]->n; i++) {
      uint32_t raw = ((const uint32_t *)sts[a]->p)[i];
      if (map[raw] == 0xffffffffu) {
        UmgStyle r = style_resolve(t, g_styles.v[raw]);
        size_t k = 0;
        const UmgStyle *sv = (const UmgStyle *)styles.p;
        while (k < styles.n && memcmp(&sv[k], &r, sizeof r)) k++;
        if (k == styles.n) {
          if (k >= UMG_NO_LTYPE) k = 0, ++*n_style_overflow;
          else *(UmgStyle *)vec_push(&styles) = r;
        }
        map[raw] = (uint32_t)k;
      }
      idx[a][i] = (uint16_t)map[raw];
    }
  }
  if (!styles.n) {  // empty drawing: still one style
    UmgStyle r = {0xffffff, 0, UMG_NO_LTYPE, 1};
    *(UmgStyle *)vec_push(&styles) = r;
  }
  free(map);

  // Tables; names go to the end of the pool.
  Vec pool = {NULL, 0, 0, 1};
  vec_append(&pool, g->pool.p, g->pool.n);
  Vec layers = {NULL, 0, 0, sizeof(UmgLayer)};
  for (size_t k = 0; k < t->n_layers; k++) {
    const LayerInfo *L = &t->layers[k];
    UmgLayer u = {(uint32_t)pool.n, (uint32_t)strlen(L->name), L->rgb, L->flags};
    vec_append(&pool, L->name, u.name_len);
    *(UmgLayer *)vec_push(&layers) = u;
  }
  Vec ltypes = {NULL, 0, 0, sizeof(UmgLtype)};
  Vec dashes = {NULL, 0, 0, sizeof(float)};
  for (size_t k = 0; k < t->n_ltypes; k++) {
    const LtypeInfo *T = &t->ltypes[k];
    UmgLtype u = {(uint32_t)pool.n, (uint32_t)strlen(T->name),
                  (uint32_t)dashes.n, T->n, T->len};
    vec_append(&pool, T->name, u.name_len);
    vec_append(&dashes, T->dash, T->n);
    *(UmgLtype *)vec_push(&ltypes) = u;
  }

  // Draw-order ranks of fills and hatch lines.
  uint32_t n_ord = 0;
  Vec fill_ord = {NULL, 0, 0, 4}, hline_ord = {NULL, 0, 0, 4};
  ord_ranks(g, t, &fill_ord, &hline_ord, &n_ord);

  // Padding before the doubles is written too: zero it so the same drawing
  // always gives the same bytes (the regression pins the SHA).
  UmgHeader h;
  memset(&h, 0, sizeof h);
  UmgHeader hv = {UMG_MAGIC, (uint32_t)g->lines.n, (uint32_t)g->arcs.n,
                  (uint32_t)g->polys.n, (uint32_t)g->pts.n,
                  (uint32_t)g->texts.n, (uint32_t)pool.n,
                  (uint32_t)styles.n, (uint32_t)layers.n, (uint32_t)ltypes.n,
                  (uint32_t)dashes.n, (uint32_t)g->fills.n, (uint32_t)g->fpts.n,
                  (uint32_t)g->hlines.n, n_ord, t->units, g->ox, g->oy, g->q};
  memcpy(&h.magic, &hv.magic, offsetof(UmgHeader, units) + sizeof h.units);
  h.ox = hv.ox, h.oy = hv.oy, h.q = hv.q;
  Vec st16[6];
  for (int a = 0; a < 6; a++) {
    Vec v = {idx[a], sts[a]->n, sts[a]->n, 2};
    st16[a] = v;
  }
  const Vec *parts[] = {&g->lines, &g->arcs, &g->polys, &g->pts, &g->texts,
                        &g->fills, &g->fpts, &g->hlines, &fill_ord,
                        &hline_ord, &g->hline_lod, &styles, &layers,
                        &ltypes, &dashes, &st16[0], &st16[1], &st16[2],
                        &st16[3], &st16[4], &st16[5], &pool};
  const int np = sizeof parts / sizeof *parts;
  size_t n = sizeof h;
  for (int i = 0; i < np; i++) n += parts[i]->n * parts[i]->elem;
  char *raw = (char *)malloc(n), *w = raw;
  memcpy(w, &h, sizeof h), w += sizeof h;
  for (int i = 0; i < np; i++) {
    if (parts[i]->n) memcpy(w, parts[i]->p, parts[i]->n * parts[i]->elem);
    w += parts[i]->n * parts[i]->elem;
  }
  *size = n;
  for (int a = 0; a < 6; a++) free(idx[a]);
  free(styles.p), free(pool.p), free(layers.p), free(ltypes.p), free(dashes.p);
  free(fill_ord.p), free(hline_ord.p);
  return raw;
}
