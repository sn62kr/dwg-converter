// UMG8 cache layout and load-time tessellation, shared by the converter, the
// snap index and the C++ renderer. A UMG8 image (zstd-compressed on disk) is:
//   UmgHeader, lines UmgLine, arcs UmgArc, polys uint32, pts UmgPt,
//   texts UmgText, fills uint32, fpts UmgFPt, hlines UmgLine,
//   fill_ord uint32, hline_ord uint32, hline_lod float, styles UmgStyle, layers UmgLayer, ltypes UmgLtype, dashes float, then
//   one uint16 style index per line, arc, poly, text, fill and hline, then
//   the UTF-8 pool (text strings, layer and linetype names). hlines are
//   hatch pattern lines: drawn like lines, never snapped to; zero-length
//   ones are pattern dots. Fills and hlines carry their draw-order rank
//   (1..n_ord, later = on top); lines, arcs, polys and text are drawn above
//   all of them. hline_lod: the hline is left out once its pattern lines
//   are drawn closer than that (drawing units; see lod_key in convert.h).
// Coordinates are int32 fixed point: world = (ox, oy) + value * q, with q a
// power of two chosen per file from the drawing extents (sample01: 1/512 mm),
// so measurements keep sub-micrometre precision. Angles and bulges are float.
#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define UMG_MAGIC 0x38474D55u /* "UMG8" */
#define UMG_CLOSED 0x80000000u
#define UMG_LTCONT 0x40000000u  // linetype pattern runs on across vertices
#define UMG_COUNT 0x3FFFFFFFu   // point count bits of a poly header
#define UMG_NO_LTYPE 0xFFFFu    // UmgStyle.ltype: continuous

typedef struct {
  uint32_t magic, n_lines, n_arcs, n_polys, n_pts, n_texts, pool_bytes;
  uint32_t n_styles, n_layers, n_ltypes, n_dashes, n_fills, n_fpts;
  uint32_t n_hlines, n_ord;
  uint32_t units;  // header INSUNITS (D80): 0 unset, 1 inch, 4 mm, 6 m, ...
  double ox, oy, q;
} UmgHeader;

// Millimetres per drawing unit for UmgHeader.units (INSUNITS); unset or
// unknown units count as mm, the Korean default (D80).
static inline double umg_mm_per_unit(uint32_t u) {
  switch (u) {
  case 1: return 25.4;           // inch
  case 2: return 304.8;          // foot
  case 4: return 1;              // mm
  case 5: return 10;             // cm
  case 6: return 1000;           // m
  case 7: return 1e6;            // km
  case 9: return 0.0254;         // mil
  case 10: return 914.4;         // yard
  case 13: return 0.001;         // micron
  case 14: return 100;           // dm
  default: return 1;
  }
}

typedef struct { int32_t x1, y1, x2, y2; } UmgLine;
typedef struct { int32_t cx, cy, r; float a0, a1; } UmgArc;  // CCW a0 -> a1
typedef struct { int32_t x, y; float bulge; } UmgPt;  // bulge of next segment
// Filled area (solid hatch, SOLID) as triangles: a fill is a uint32 vertex
// count (a multiple of 3), its vertices follow in fpts.
typedef struct { int32_t x, y; } UmgFPt;

// One text item, already reduced to plain UTF-8 ('\n' = MTEXT paragraph).
// Layout (font metrics, justification) happens on the device.
typedef struct {
  int32_t x, y;    // TEXT/MTEXT ins_pt
  int32_t x2, y2;  // TEXT alignment_pt (used when ha/va != 0)
  float h, rot, wf, obl;
  float width, lsf;  // MTEXT: rect width, line spacing factor
  uint32_t off, len;  // into the string pool
  uint8_t kind, ha, va, att;  // kind 0 = TEXT/ATTRIB, 1 = MTEXT
} UmgText;

// How a primitive is drawn, fully resolved (ByLayer / ByBlock already
// applied): colour, owning layer (for show / hide) and linetype.
typedef struct {
  uint32_t rgb;     // 0xRRGGBB
  uint16_t layer;   // index into layers
  uint16_t ltype;   // index into ltypes, or UMG_NO_LTYPE
  float lt_scale;   // pattern scale in drawing units (LTSCALE * entity)
} UmgStyle;

// UmgStyle.rgb flag: WIPEOUT, filled with the background colour.
#define UMG_RGB_BACKGROUND 0x01000000u

#define UMG_LAYER_OFF 1u
#define UMG_LAYER_FROZEN 2u

typedef struct {
  uint32_t name_off, name_len;  // into the pool
  uint32_t rgb, flags;          // UMG_LAYER_*
} UmgLayer;

// Dash pattern: > 0 dash, < 0 gap, 0 dot (lengths before lt_scale).
typedef struct {
  uint32_t name_off, name_len;
  uint32_t dash_off, n_dash;  // into dashes
  float pattern_len;
} UmgLtype;

// Pointers into a raw UMG8 image.
typedef struct {
  UmgHeader h;
  const UmgLine *lines;
  const UmgArc *arcs;
  const uint32_t *polys;
  const UmgPt *pts;
  const UmgText *texts;
  const uint32_t *fills;
  const UmgFPt *fpts;
  const UmgLine *hlines;
  const uint32_t *fill_ord, *hline_ord;
  const float *hline_lod;
  const UmgStyle *styles;
  const UmgLayer *layers;
  const UmgLtype *ltypes;
  const float *dashes;
  const uint16_t *line_st, *arc_st, *poly_st, *text_st, *fill_st, *hline_st;
  const char *pool;
} UmgView;

static inline int umg_view(const void *raw, size_t size, UmgView *v) {
  if (size < sizeof(UmgHeader)) return 0;
  const UmgHeader *h = (const UmgHeader *)raw;
  if (h->magic != UMG_MAGIC) return 0;
  v->h = *h;
  const char *p = (const char *)raw + sizeof(UmgHeader);
  const char *end = (const char *)raw + size;
#define UMG_TAKE(field, type, count)                                \
  if ((size_t)(end - p) / sizeof(type) < (size_t)(count)) return 0; \
  v->field = (const type *)p, p += (size_t)(count) * sizeof(type)
  UMG_TAKE(lines, UmgLine, h->n_lines);
  UMG_TAKE(arcs, UmgArc, h->n_arcs);
  UMG_TAKE(polys, uint32_t, h->n_polys);
  UMG_TAKE(pts, UmgPt, h->n_pts);
  UMG_TAKE(texts, UmgText, h->n_texts);
  UMG_TAKE(fills, uint32_t, h->n_fills);
  UMG_TAKE(fpts, UmgFPt, h->n_fpts);
  UMG_TAKE(hlines, UmgLine, h->n_hlines);
  UMG_TAKE(fill_ord, uint32_t, h->n_fills);
  UMG_TAKE(hline_ord, uint32_t, h->n_hlines);
  UMG_TAKE(hline_lod, float, h->n_hlines);
  UMG_TAKE(styles, UmgStyle, h->n_styles);
  UMG_TAKE(layers, UmgLayer, h->n_layers);
  UMG_TAKE(ltypes, UmgLtype, h->n_ltypes);
  UMG_TAKE(dashes, float, h->n_dashes);
  UMG_TAKE(line_st, uint16_t, h->n_lines);
  UMG_TAKE(arc_st, uint16_t, h->n_arcs);
  UMG_TAKE(poly_st, uint16_t, h->n_polys);
  UMG_TAKE(text_st, uint16_t, h->n_texts);
  UMG_TAKE(fill_st, uint16_t, h->n_fills);
  UMG_TAKE(hline_st, uint16_t, h->n_hlines);
  UMG_TAKE(pool, char, h->pool_bytes);
#undef UMG_TAKE
  return p == end && h->n_styles > 0;
}

// Colour as shown on the viewer's near-black background: a colour darker
// than luma UMG_MIN_LUMA is mixed towards white up to that luma, so dark
// greys and pure blue stay visible and keep their hue (AutoCAD draws them
// as they are, only swapping colour 7). Solid fills keep their colour.
#define UMG_MIN_LUMA 0.4
static inline uint32_t umg_readable_rgb(uint32_t rgb) {
  double r = (rgb >> 16 & 255) / 255.0, g = (rgb >> 8 & 255) / 255.0,
         b = (rgb & 255) / 255.0, y = 0.299 * r + 0.587 * g + 0.114 * b;
  if (y >= UMG_MIN_LUMA) return rgb;
  double k = (UMG_MIN_LUMA - y) / (1 - y);
  r += (1 - r) * k, g += (1 - g) * k, b += (1 - b) * k;
  return (rgb & 0xff000000u) | (uint32_t)(r * 255 + 0.5) << 16 |
         (uint32_t)(g * 255 + 0.5) << 8 | (uint32_t)(b * 255 + 0.5);
}

// Colour as shown on a light background (D82, setting 흰 배경): white and
// near-white (AutoCAD's colour 7) turn black; lines and text brighter than
// luma UMG_MAX_LUMA_LIGHT are darkened to it so yellow and the like stay
// visible on white. Fills keep their colour apart from white -> black.
#define UMG_MAX_LUMA_LIGHT 0.6
static inline uint32_t umg_display_rgb(uint32_t rgb, int light, int fill) {
  if (!light) return fill ? rgb : umg_readable_rgb(rgb);
  uint32_t r8 = rgb >> 16 & 255, g8 = rgb >> 8 & 255, b8 = rgb & 255;
  if (r8 >= 0xF0 && g8 >= 0xF0 && b8 >= 0xF0) return rgb & 0xff000000u;
  if (fill) return rgb;
  double r = r8 / 255.0, g = g8 / 255.0, b = b8 / 255.0,
         y = 0.299 * r + 0.587 * g + 0.114 * b;
  if (y <= UMG_MAX_LUMA_LIGHT) return rgb;
  double k = UMG_MAX_LUMA_LIGHT / y;
  return (rgb & 0xff000000u) | (uint32_t)(r * k * 255 + 0.5) << 16 |
         (uint32_t)(g * k * 255 + 0.5) << 8 | (uint32_t)(b * k * 255 + 0.5);
}

static inline int umg_arc_segs(double sweep) {
  double n = ceil(fabs(sweep) / (2 * M_PI) * 32);  // clamp before int: NaN, 1e58
  return !(n >= 4) ? 4 : n > 64 ? 64 : (int)n;
}

// Arc through the bulged segment p1 -> p2 (bulge = tan(sweep / 4)).
static inline void umg_bulge_arc(double x1, double y1, double x2, double y2,
                                 double b, double *cx, double *cy, double *r,
                                 double *a0, double *sweep) {
  double dx = x2 - x1, dy = y2 - y1, d = hypot(dx, dy), s = d / 2;
  double h = s * (1 - b * b) / (2 * b);  // signed: + = center left of chord
  *cx = (x1 + x2) / 2 - h * dy / d;
  *cy = (y1 + y2) / 2 + h * dx / d;
  *r = s * (1 + b * b) / (2 * fabs(b));
  *a0 = atan2(y1 - *cy, x1 - *cx);
  *sweep = 4 * atan(b);
}

// Tessellation into GL_LINES segments (float[4] each, world units relative
// to (ox, oy); float is plenty for drawing), optionally with the distance
// along the linetype run at each vertex (float[2] per segment) for dashed
// styles. A run is one line, arc or poly segment, or a whole poly when
// UMG_LTCONT is set; a run shorter than one pattern gets distance -1, which
// the shader draws solid (as AutoCAD does). With xy == NULL only counts.
// With arc_max_r > 0, arcs up to that radius are not split into segments
// but recorded as one instance each (UMG_ARC_FLOATS floats in arc), which
// the viewer draws exactly at any zoom (D69); larger ones stay segments, as
// a float ring test loses precision far from the centre.
#define UMG_ARC_FLOATS 10  // bbox x0 y0 x1 y1, cx cy r a0 sweep, run distance
typedef struct {
  float *xy, *dist;  // output, or NULL
  size_t n;          // segments written so far
  float pat;         // scaled pattern length (dist only)
  double d;          // distance along the current run
  size_t run;        // first segment of the current run
  float *arc;        // arc instances, or NULL (counting)
  size_t na;         // arc instances so far
  size_t arun;       // first arc instance of the current run
  double arc_max_r;  // 0 = no instances
} UmgTess;

static inline void umg_run_begin(UmgTess *t) {
  t->d = 0, t->run = t->n, t->arun = t->na;
}

static inline void umg_run_end(UmgTess *t) {
  if (t->dist && t->d < t->pat) {
    for (size_t k = 2 * t->run; k < 2 * t->n; k++) t->dist[k] = -1;
    if (t->arc)
      for (size_t k = t->arun; k < t->na; k++) t->arc[k * UMG_ARC_FLOATS + 9] = -1;
  }
}

// Bounding box of the arc from a0 through sweep (either sign).
static inline void umg_arc_bbox(double cx, double cy, double r, double a0,
                                double sweep, double *b) {
  double a1 = a0 + sweep, lo = sweep < 0 ? a1 : a0, hi = sweep < 0 ? a0 : a1;
  double x0 = cos(a0), y0 = sin(a0), x1 = cos(a1), y1 = sin(a1);
  b[0] = fmin(x0, x1), b[1] = fmin(y0, y1), b[2] = fmax(x0, x1), b[3] = fmax(y0, y1);
  // Axis directions k * 90 deg inside [lo, hi] extend the box.
  for (double k = ceil(lo / (M_PI / 2)); k * (M_PI / 2) <= hi && k <= ceil(lo / (M_PI / 2)) + 4; k++) {
    int q = ((int)k % 4 + 4) % 4;
    if (q == 0) b[2] = 1;
    else if (q == 1) b[3] = 1;
    else if (q == 2) b[0] = -1;
    else b[1] = -1;
  }
  b[0] = cx + r * b[0], b[1] = cy + r * b[1], b[2] = cx + r * b[2], b[3] = cy + r * b[3];
}

static inline void umg_seg(UmgTess *t, double x1, double y1, double x2,
                           double y2) {
  if (t->xy) {
    float *o = t->xy + t->n * 4;
    o[0] = (float)x1, o[1] = (float)y1, o[2] = (float)x2, o[3] = (float)y2;
  }
  if (t->dist) {
    double len = hypot(x2 - x1, y2 - y1);
    t->dist[2 * t->n] = (float)t->d, t->dist[2 * t->n + 1] = (float)(t->d + len);
    t->d += len;
  }
  t->n++;
}

static inline void umg_seg_arc(UmgTess *t, double cx, double cy, double r,
                               double a0, double sweep) {
  if (r <= t->arc_max_r) {
    if (t->arc) {
      float *o = t->arc + t->na * UMG_ARC_FLOATS;
      double b[4];
      umg_arc_bbox(cx, cy, r, a0, sweep, b);
      o[0] = (float)b[0], o[1] = (float)b[1], o[2] = (float)b[2], o[3] = (float)b[3];
      o[4] = (float)cx, o[5] = (float)cy, o[6] = (float)r;
      o[7] = (float)a0, o[8] = (float)sweep, o[9] = t->dist ? (float)t->d : -1;
    }
    if (t->dist) t->d += fabs(sweep) * r;
    t->na++;
    return;
  }
  int k = umg_arc_segs(sweep);
  double px = cx + r * cos(a0), py = cy + r * sin(a0);
  for (int i = 1; i <= k; i++) {
    double a = a0 + sweep * i / k, x = cx + r * cos(a), y = cy + r * sin(a);
    umg_seg(t, px, py, x, y);
    px = x, py = y;
  }
}

static inline void umg_tess_line(UmgTess *t, const UmgView *v, uint32_t i) {
  const double q = v->h.q;
  const UmgLine *l = &v->lines[i];
  umg_run_begin(t);
  umg_seg(t, l->x1 * q, l->y1 * q, l->x2 * q, l->y2 * q);
  umg_run_end(t);
}

static inline int umg_hline_is_dot(const UmgLine *l) {
  return l->x1 == l->x2 && l->y1 == l->y2;
}

// Hatch pattern line; dots (zero length) are left to the caller (points).
static inline void umg_tess_hline(UmgTess *t, const UmgView *v, uint32_t i) {
  const double q = v->h.q;
  const UmgLine *l = &v->hlines[i];
  if (umg_hline_is_dot(l)) return;
  umg_run_begin(t);
  umg_seg(t, l->x1 * q, l->y1 * q, l->x2 * q, l->y2 * q);
  umg_run_end(t);
}

static inline void umg_tess_arc(UmgTess *t, const UmgView *v, uint32_t i) {
  const double q = v->h.q;
  const UmgArc *a = &v->arcs[i];
  double a0 = a->a0, a1 = a->a1;
  if (a1 < a0) a1 += 2 * M_PI;
  umg_run_begin(t);
  umg_seg_arc(t, a->cx * q, a->cy * q, a->r * q, a0, a1 - a0);
  umg_run_end(t);
}

// Poly i, whose points start at pts[p0].
static inline void umg_tess_poly(UmgTess *t, const UmgView *v, uint32_t i,
                                 size_t p0) {
  const double q = v->h.q;
  uint32_t head = v->polys[i], cnt = head & UMG_COUNT;
  int closed = (head & UMG_CLOSED) != 0, cont = (head & UMG_LTCONT) != 0;
  uint32_t nseg = closed ? cnt : cnt - 1;
  if (cnt < 2) return;
  if (cont) umg_run_begin(t);
  for (uint32_t j = 0; j < nseg; j++) {
    const UmgPt *a = &v->pts[p0 + j], *b = &v->pts[p0 + (j + 1) % cnt];
    if (!cont) umg_run_begin(t);
    if (a->bulge == 0 || (a->x == b->x && a->y == b->y)) {
      umg_seg(t, a->x * q, a->y * q, b->x * q, b->y * q);
    } else {
      double cx, cy, r, a0, sw;
      umg_bulge_arc(a->x * q, a->y * q, b->x * q, b->y * q, a->bulge, &cx,
                    &cy, &r, &a0, &sw);
      umg_seg_arc(t, cx, cy, r, a0, sw);
    }
    if (!cont) umg_run_end(t);
  }
  if (cont) umg_run_end(t);
}
