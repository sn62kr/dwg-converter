// unimog-convert: DWG -> UMG8 cache (exact fixed-point primitives + text,
// styles, layers, linetypes; zstd). Runs as a separate process from the app
// (GPL boundary, D31): the app only reads the UMG8 file this writes.
//   unimog-convert [-s] in.dwg out.umg   (in.dwg may also be a DXF or a PDF,
//                                          by content; a PDF: its first page)
//   -p N  page N (0-based) of a PDF (D105); the default is the first
//   -s  sandbox (the app always passes it): write out.umg directly (the
//       caller renames it into place) and, once the DWG is in memory, lock
//       the process down (sandbox.h). INFO then carries sandbox=on|failed.
//   unimog-convert --sandbox-test   checks the lockdown on this device
// stdout protocol, one line each:
//   PROGRESS <objects decoded>      every 100k objects
//   INFO key=value ...               drawing facts for the app (D86), before DONE
//   DONE <ms> <primitives> <texts>  on success
// Exit codes: 0 ok, 2 usage, 3 unreadable DWG, 4 cannot write output.
// Without -s the output is written to out.umg.tmp and renamed, so a killed
// process never leaves a half-written cache behind (with -s the caller
// does that: a locked-down process cannot rename).

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zstd.h>

#include "blocks.h"
#include "sandbox.h"
#include "pdf.h"
#include "compare.h"

static int g_sandbox = SANDBOX_NONE;
static void enter_sandbox(void *unused) {
  (void)unused;
  g_sandbox = sandbox_enter();
}

typedef struct {
  Geom *b;
  Blocks *blk;
  Tables *tab;
  int origin_set;
  unsigned long n_seen;
  unsigned long n_paper;  // entities drawn only on layouts (paper space), D87
  unsigned long n_paper_active;  // of those, in the active layout
  int dxf;  // reading a DXF (its entities are offered as they are read too)
  Vec vps;  // Vp: every viewport, when a layout is wanted
  // Closed polylines and circles outside model space, when a layout is wanted:
  // one of them may be a viewport's clip boundary (looked up by handle once
  // the viewports are known; the entity itself is freed after conversion).
  Vec shapes;     // Shape
  Vec shape_pts;  // double x, y
} Ctx;

typedef struct { uint64_t h; uint32_t off, n; } Shape;

static void shape_record(Ctx *c, const Dwg_Object *o) {
  const Dwg_Object_Entity *e = o->tio.entity;
  if (c->shapes.n >= 200000 || c->shape_pts.n >= 4000000) return;
  c->shapes.elem = sizeof(Shape), c->shape_pts.elem = 2 * sizeof(double);
  uint32_t off = (uint32_t)c->shape_pts.n;
#define SHAPE_PT(X, Y) \
  do { double *q_ = (double *)vec_push(&c->shape_pts); q_[0] = (X), q_[1] = (Y); } while (0)
  if (o->fixedtype == DWG_TYPE_CIRCLE) {
    const Dwg_Entity_CIRCLE *ci = e->tio.CIRCLE;
    if (!(ci->radius > 0) || !isfinite(ci->radius + ci->center.x + ci->center.y)) return;
    double mx = ci->extrusion.z < 0 ? -1 : 1;
    for (int k = 0; k < 64; k++)
      SHAPE_PT(mx * (ci->center.x + ci->radius * cos(2 * M_PI * k / 64)), ci->center.y + ci->radius * sin(2 * M_PI * k / 64));
  } else if (o->fixedtype == DWG_TYPE_LWPOLYLINE) {
    const Dwg_Entity_LWPOLYLINE *p = e->tio.LWPOLYLINE;
    uint32_t n = p->points ? p->num_points : 0;
    if (n < 3 || n > 256 || !(p->flag & 512)) return;  // closed outlines only
    double mx = p->extrusion.z < 0 ? -1 : 1;
    for (uint32_t i = 0; i < n; i++) {
      double x1 = p->points[i].x, y1 = p->points[i].y;
      double x2 = p->points[(i + 1) % n].x, y2 = p->points[(i + 1) % n].y;
      double b = p->bulges && i < p->num_bulges ? p->bulges[i] : 0;
      if (!isfinite(x1 + y1 + x2 + y2 + b)) { c->shape_pts.n = off; return; }
      SHAPE_PT(mx * x1, y1);
      if (b != 0 && (x1 != x2 || y1 != y2)) {  // an arc: its points in between
        double cx, cy, r, a0, sw;
        umg_bulge_arc(x1, y1, x2, y2, b, &cx, &cy, &r, &a0, &sw);
        int k = umg_arc_segs(sw);
        for (int j = 1; j < k; j++)
          SHAPE_PT(mx * (cx + r * cos(a0 + sw * j / k)), cy + r * sin(a0 + sw * j / k));
      }
    }
  } else {
    return;
  }
#undef SHAPE_PT
  Shape *s = (Shape *)vec_push(&c->shapes);
  s->h = o->handle.value, s->off = off, s->n = (uint32_t)c->shape_pts.n - off;
}

// A paper-space viewport: a w x h window at (cx, cy) on the paper showing
// the model around (vcx, vcy) (after the turn by twist about the target),
// vsize model units high.
typedef struct {
  uint64_t owner, handle;  // its layout's block (PS_KEY: the active one)
  uint32_t style;
  double cx, cy, w, h, tgx, tgy, vcx, vcy, vsize, twist;
  int ok;  // on, seen from straight above, sizes make sense
  uint32_t n_frozen;  // layers frozen in this viewport only
  uint64_t *frozen;   // their handles
  int shaped;         // clipped to a shape other than its rectangle
  uint64_t clip_h;    // that shape: an entity on the paper
} Vp;

static void vp_record(Ctx *c, const Dwg_Object *o) {
  const Dwg_Object_Entity *e = o->tio.entity;
  const Dwg_Entity_VIEWPORT *v = e->tio.VIEWPORT;
  if (!v || c->vps.n >= 4096) return;
  c->vps.elem = sizeof(Vp);
  Vp *p = (Vp *)vec_push(&c->vps);
  memset(p, 0, sizeof *p);
  p->owner = e->entmode == 1 ? PS_KEY : ref_handle(e->ownerhandle);
  p->handle = o->handle.value, p->style = entity_style(e);
  p->cx = v->center.x, p->cy = v->center.y, p->w = v->width, p->h = v->height;
  p->tgx = v->view_target.x, p->tgy = v->view_target.y;
  p->vcx = v->VIEWCTR.x, p->vcy = v->VIEWCTR.y, p->vsize = v->VIEWSIZE, p->twist = v->VIEWTWIST;
  p->n_frozen = v->frozen_layers && v->num_frozen_layers <= 100000 ? v->num_frozen_layers : 0;
  if (p->n_frozen) {
    p->frozen = (uint64_t *)malloc(p->n_frozen * sizeof(uint64_t));
    for (uint32_t i = 0; i < p->n_frozen; i++) p->frozen[i] = ref_handle(v->frozen_layers[i]);
  }
  p->shaped = (v->status_flag & 0x10000) != 0;
  p->clip_h = p->shaped ? ref_handle(v->clip_boundary) : 0;
  double dl = sqrt(v->VIEWDIR.x * v->VIEWDIR.x + v->VIEWDIR.y * v->VIEWDIR.y + v->VIEWDIR.z * v->VIEWDIR.z);
  double s = p->h / p->vsize;
  p->ok = !(v->status_flag & 0x20001) &&  // off, perspective
          dl > 0 && v->VIEWDIR.z > 0.999999 * dl &&
          isfinite(p->cx + p->cy + p->tgx + p->tgy + p->vcx + p->vcy + p->twist) &&
          p->w > 0 && p->h > 0 && p->w < 1e9 && p->h < 1e9 && s > 1e-9 && s < 1e9;
}

// Model -> paper through a viewport.
static Xf vp_xf(const Vp *p) {
  double s = p->h / p->vsize, co = cos(p->twist), si = sin(p->twist);
  Xf m = {s * co, -s * si, s * si, s * co, 0, 0};
  m.tx = p->cx - s * p->vcx - (m.a * p->tgx + m.b * p->tgy);
  m.ty = p->cy - s * p->vcy - (m.c * p->tgx + m.d * p->tgy);
  return m;
}

static int block_has_content(const Blocks *s, uint64_t key) {
  const Block *b = block_find(s, key);
  if (!b) return 0;
  const Geom *g = &b->geom;
  return g->lines.n + g->arcs.n + g->polys.n + g->texts.n + g->fills.n + g->hlines.n + b->kids.n > 0;
}

// The drawing's layouts that have something drawn on the paper (a title
// block, notes; an untouched "Layout1" with only its default viewport is
// left out), in tab order. Printed as "LAYOUT <n> <name>" for the app.
typedef struct { int order; uint64_t key; char *name; double x0, y0, x1, y1; } Lay;

static int cmp_lay(const void *a, const void *b) {
  return ((const Lay *)a)->order - ((const Lay *)b)->order;
}

static size_t layouts_list(Ctx *c, Dwg_Data *dwg, Lay **out) {
  uint64_t ms = dwg->header_vars.BLOCK_RECORD_MSPACE ? dwg->header_vars.BLOCK_RECORD_MSPACE->absolute_ref : 0;
  uint64_t ps = dwg->header_vars.BLOCK_RECORD_PSPACE ? dwg->header_vars.BLOCK_RECORD_PSPACE->absolute_ref : 0;
  Lay *v = NULL;
  size_t n = 0, cap = 0;
  for (BITCODE_BL i = 0; i < dwg->num_objects && n < 1000; i++) {
    Dwg_Object *o = &dwg->object[i];
    if (o->fixedtype != DWG_TYPE_LAYOUT || !o->tio.object || !o->tio.object->tio.LAYOUT) continue;
    const Dwg_Object_LAYOUT *L = o->tio.object->tio.LAYOUT;
    uint64_t bh = ref_handle(L->block_header);
    if (!bh || bh == ms) continue;  // "Model"
    int active = bh == ps;
    // The active layout's entities are only kept when a layout is wanted;
    // otherwise they were counted.
    int has = active ? (g_layout ? block_has_content(c->blk, PS_KEY) : c->n_paper_active > 0)
                     : block_has_content(c->blk, bh);
    if (!has) continue;
    if (n == cap) cap = cap ? cap * 2 : 8, v = (Lay *)realloc(v, cap * sizeof(Lay));
    Lay *l = &v[n++];
    l->order = L->tab_order, l->key = active ? PS_KEY : bh;
    l->name = dwg_utf8(o, L->layout_name);
    if (!l->name) l->name = strdup("");
    for (char *q = l->name; *q; q++)
      if ((unsigned char)*q < ' ') *q = ' ';
    l->x0 = L->EXTMIN.x, l->y0 = L->EXTMIN.y, l->x1 = L->EXTMAX.x, l->y1 = L->EXTMAX.y;
  }
  if (n) qsort(v, n, sizeof(Lay), cmp_lay);
  *out = v;
  return n;
}

// Puts layout l together as the output's roots: the model through every
// viewport, then the paper on top. Everything is placed so that the largest
// viewport shows the model at its real place and size (measuring and
// coordinates are then right inside it, like a PDF with one scale); the
// paper and the other viewports are scaled around it. Sets the output's
// origin to the middle of the sheet. Gives the box of the sheet and the main
// viewport's scale (0: no viewport, the sheet is in paper units).
static void layout_roots(Ctx *c, const Lay *l, uint64_t layer0, double box[4], double *scale) {
  Blocks *s = c->blk;
  const Vp *vp = (const Vp *)c->vps.p;
  // The first viewport of a layout is the sheet itself (the paper view).
  long first = -1, main = -1;
  for (size_t i = 0; i < c->vps.n; i++) {
    if (vp[i].owner != l->key) continue;
    if (first < 0) { first = (long)i; continue; }
    if (vp[i].ok && (main < 0 || vp[i].w * vp[i].h > vp[main].w * vp[main].h)) main = (long)i;
  }
  Xf T = main >= 0 ? xf_inv(vp_xf(&vp[main])) : XF_ID;
  *scale = main >= 0 ? vp[main].h / vp[main].vsize : 0;  // paper per model unit
  // References that change nothing: layer "0" stays "0", ByBlock stays.
  RawStyle neutral = {layer0, LT_BYBLOCK, COL_BYBLOCK, 1};
  uint32_t st = style_intern(neutral);
  double x0 = INFINITY, y0 = INFINITY, x1 = -INFINITY, y1 = -INFINITY;
#define GROW(X, Y)                                                    \
  do {                                                                \
    double gx = T.a * (X) + T.b * (Y) + T.tx, gy = T.c * (X) + T.d * (Y) + T.ty; \
    x0 = fmin(x0, gx), x1 = fmax(x1, gx), y0 = fmin(y0, gy), y1 = fmax(y1, gy);  \
  } while (0)
  if (isfinite(l->x0 + l->y0 + l->x1 + l->y1) && l->x1 > l->x0 && l->y1 > l->y0 &&
      l->x1 - l->x0 < 1e7 && l->y1 - l->y0 < 1e7) {
    GROW(l->x0, l->y0);
    GROW(l->x1, l->y0);
    GROW(l->x1, l->y1);
    GROW(l->x0, l->y1);
  }
  size_t shown = 0, skipped = 0, n_frozen_vp = 0, n_shaped = 0, n_shape_used = 0;
  for (size_t i = 0; i < c->vps.n; i++) {
    const Vp *p = &vp[i];
    if (p->owner == l->key && (long)i != first && !p->ok) skipped++;
    if (p->owner != l->key || (long)i == first || !p->ok || shown >= 64) continue;
    shown++;
    if (p->n_frozen) n_frozen_vp++;
    if (p->shaped) n_shaped++;
    Xf V = vp_xf(p), Vi = xf_inv(V);
    double cx[4] = {p->cx - p->w / 2, p->cx + p->w / 2, p->cx + p->w / 2, p->cx - p->w / 2};
    double cy[4] = {p->cy - p->h / 2, p->cy - p->h / 2, p->cy + p->h / 2, p->cy + p->h / 2};
    Ref *f = refs_push(&s->roots, MS_KEY, xf_mul(T, V), st, p->handle);
    // Clipped to a shape drawn on the paper (a closed polyline or a circle):
    // that outline instead of the rectangle. The shape is an entity of the
    // sheet and is drawn as such, so no frame is added.
    const Shape *sh = NULL;
    for (size_t k = 0; p->clip_h && k < c->shapes.n && !sh; k++)
      if (((const Shape *)c->shapes.p)[k].h == p->clip_h) sh = (const Shape *)c->shapes.p + k;
    if (sh && sh->n >= 3 && sh->n <= 4096) {
      const double *sp = (const double *)c->shape_pts.p + 2 * sh->off;
      f->clip = (double *)malloc(sh->n * 2 * sizeof(double)), f->nclip = sh->n;
      for (uint32_t k = 0; k < sh->n; k++) {
        f->clip[2 * k] = Vi.a * sp[2 * k] + Vi.b * sp[2 * k + 1] + Vi.tx;
        f->clip[2 * k + 1] = Vi.c * sp[2 * k] + Vi.d * sp[2 * k + 1] + Vi.ty;
        GROW(sp[2 * k], sp[2 * k + 1]);
      }
      n_shape_used++;
      if (p->n_frozen) {
        f->hide = (uint64_t *)malloc(p->n_frozen * sizeof(uint64_t)), f->nhide = p->n_frozen;
        memcpy(f->hide, p->frozen, p->n_frozen * sizeof(uint64_t));
        qsort(f->hide, f->nhide, sizeof(uint64_t), cmp_u64);
      }
      s->n_xclip++;
      continue;
    }
    f->clip = (double *)malloc(8 * sizeof(double)), f->nclip = 4;
    if (p->n_frozen) {  // a copy: the list of viewports is freed before the output is put together
      f->hide = (uint64_t *)malloc(p->n_frozen * sizeof(uint64_t)), f->nhide = p->n_frozen;
      memcpy(f->hide, p->frozen, p->n_frozen * sizeof(uint64_t));
      qsort(f->hide, f->nhide, sizeof(uint64_t), cmp_u64);
    }
    Geom *pg = &block_get(s, l->key)->geom;  // the frame, on the viewport's layer
    pg->cur = p->style;
    begin_poly(pg, 4, UMG_CLOSED | UMG_LTCONT);
    for (int k = 0; k < 4; k++) {
      f->clip[2 * k] = Vi.a * cx[k] + Vi.b * cy[k] + Vi.tx;
      f->clip[2 * k + 1] = Vi.c * cx[k] + Vi.d * cy[k] + Vi.ty;
      add_pt(pg, XF_ID, cx[k], cy[k], 0);
      GROW(cx[k], cy[k]);
    }
    s->n_xclip++;
  }
#undef GROW
  refs_push(&s->roots, l->key, T, st, UINT64_MAX);
  fprintf(stderr, "layout viewports=%zu skipped=%zu frozen=%zu shaped=%zu shape_clips=%zu scale=%.6g\n", shown, skipped,
          n_frozen_vp, n_shaped, n_shape_used, main >= 0 ? vp[main].h / vp[main].vsize : 0.0);
  if (!(x1 > x0 && y1 > y0)) x0 = y0 = 0, x1 = 420, y1 = 297;
  box[0] = x0, box[1] = y0, box[2] = x1, box[3] = y1;
  Geom *b = c->b;  // empty so far: everything went into blocks
  b->ox = (x0 + x1) / 2, b->oy = (y0 + y1) / 2, b->origin_set = 1;
}

// Paper space is not drawn (D87): count what is there so the app can say
// "this drawing lives on its layouts". Viewports are frames, not content.
static void count_paper(Ctx *c, const Dwg_Object *o) {
  if (o->fixedtype != DWG_TYPE_VIEWPORT) c->n_paper++;
  // The paper-space block's own BLOCK / ENDBLK markers are not content.
  if (o->fixedtype != DWG_TYPE_VIEWPORT && o->fixedtype != DWG_TYPE_BLOCK &&
      o->fixedtype != DWG_TYPE_ENDBLK)
    c->n_paper_active++;
}

// Origin = center of the header extents; q = the smallest power of two that
// keeps +-2x the extents inside int32. Stale or missing extents fall back to
// a 10 km span around 0; overflow is counted as clamped.
static void set_origin(Ctx *c, Dwg_Data *dwg) {
  Dwg_Bitcode_3BD lo = dwg->header_vars.EXTMIN, hi = dwg->header_vars.EXTMAX;
  double span = fmax(hi.x - lo.x, hi.y - lo.y);
  int ok = isfinite(span) && span > 1e-9 && span < 1e12 && fabs(lo.x) < 1e15 &&
           fabs(lo.y) < 1e15;
  // A layout is drawn around its main viewport, in model units: room for a
  // sheet a few times larger than a small model.
  if (ok && g_layout) span = fmax(span, 1e5);
  Geom *b = c->b;
  b->ox = ok ? (lo.x + hi.x) / 2 : 0, b->oy = ok ? (lo.y + hi.y) / 2 : 0;
  b->q = c->blk->q = ldexp(1, (int)ceil(log2((ok ? span : 1e7) / 1073741824.0)));
  b->origin_set = 1;
}

// Self-contained entity: converted straight into model space or its block.
static void handle_self_contained(Ctx *c, const Dwg_Object *o) {
  Dwg_Object_Entity *e = o->tio.entity;
  if (e->invisible & 1) return;  // hidden
  if (e->entmode == 1) count_paper(c, o);  // active layout
  if (g_layout && e->entmode != 2 &&
      (o->fixedtype == DWG_TYPE_LWPOLYLINE || o->fixedtype == DWG_TYPE_CIRCLE))
    shape_record(c, o);
  Geom *g = geom_for(c->blk, c->b, e);
  if (!g) return;
  convert_entity(g, o);
  if (is_dimension(o)) blocks_add_dimension(c->blk, c->b, o);
}

// Everything the streaming decoder keeps: block headers and references.
static void handle_kept(Ctx *c, const Dwg_Object *o) {
  if (o->fixedtype == DWG_TYPE_BLOCK_HEADER) {
    blocks_set_base(c->blk, o);
    // Other layouts' contents sit in *Paper_Space0, 1, ... blocks.
    const Dwg_Object_BLOCK_HEADER *bh = o->tio.object->tio.BLOCK_HEADER;
    char *nm = dwg_utf8(o, bh->name);
    if (nm && !strncasecmp(nm, "*Paper_Space", 12) && nm[12]) c->n_paper += bh->num_owned;
    free(nm);
  } else if (o->supertype != DWG_SUPERTYPE_ENTITY) {
    tables_add(c->tab, o);
  } else if (o->tio.entity->invisible & 1) {
    return;
  } else if (o->fixedtype == DWG_TYPE_ATTRIB) {
    blocks_add_attrib(c->blk, c->b, o);
  } else if (o->fixedtype == DWG_TYPE_INSERT ||
             o->fixedtype == DWG_TYPE_MINSERT) {
    blocks_add_insert(c->blk, o);
  } else if (is_dimension(o)) {  // radius, angular, ...: not streamed
    blocks_add_dimension(c->blk, c->b, o);
  } else if (g_layout && o->fixedtype == DWG_TYPE_VIEWPORT) {
    vp_record(c, o);
  } else if (o->tio.entity->entmode == 1) {
    count_paper(c, o);
    if (g_layout) convert_entity(&block_get(c->blk, PS_KEY)->geom, o);
  } else if (o->tio.entity->entmode == 2) {
    // unsupported types are counted as skipped
    convert_entity(g_layout ? &block_get(c->blk, MS_KEY)->geom : c->b, o);
  } else if (o->tio.entity->entmode == 0 &&
             !(o->fixedtype >= DWG_TYPE_VERTEX_2D && o->fixedtype <= DWG_TYPE_VERTEX_PFACE_FACE) &&
             o->fixedtype != DWG_TYPE_SEQEND && o->fixedtype != DWG_TYPE_ATTDEF) {
    // Non-streamed entity inside a block (old-style POLYLINE, ...).
    convert_entity(&block_get(c->blk, ref_handle(o->tio.entity->ownerhandle))->geom, o);
  }
}

static int on_object(Dwg_Data *dwg, Dwg_Object *o, void *ud) {
  Ctx *c = (Ctx *)ud;
  if (++c->n_seen % 100000 == 0) {
    printf("PROGRESS %lu\n", c->n_seen);
    fflush(stdout);
  }
  if (o->supertype != DWG_SUPERTYPE_ENTITY || !is_self_contained(o)) return 0;
  // A DXF names a dimension's block, and the name is turned into a handle
  // only after the whole file is read: dimensions wait until then.
  if (c->dxf && is_dimension(o)) return 0;
  if (!c->origin_set) {  // header is decoded before the object stream
    set_origin(c, dwg);
    c->origin_set = 1;
  }
  handle_self_contained(c, o);
  return 1;
}

// DXF instead of DWG (by content: apps send both as octet-stream, and the
// app stores every drawing as .dwg). A DWG starts with its version "AC1...";
// a DXF with "AutoCAD Binary DXF" or group-code text ("0" / "999").
// LibreDWG's DXF reader leaves entmode 3 ("owner unknown") with the owner
// handle set; a DWG read says 2 model space, 1 paper space, 0 in a block.
// Old-style polylines in blocks were left out (handle_kept wants 0).
static void dxf_fix_entmode(Dwg_Data *d) {
  uint64_t ms = d->header_vars.BLOCK_RECORD_MSPACE ? d->header_vars.BLOCK_RECORD_MSPACE->absolute_ref : 0;
  uint64_t ps = d->header_vars.BLOCK_RECORD_PSPACE ? d->header_vars.BLOCK_RECORD_PSPACE->absolute_ref : 0;
  // R12 has no owner handles: only a BLOCK names its block header, and
  // what follows it up to ENDBLK belongs to that block. Outside BLOCK ...
  // ENDBLK, an entity without an owner is in model space (ENTITIES).
  BITCODE_H blk = NULL;
  for (BITCODE_BL i = 0; i < d->num_objects; i++) {
    Dwg_Object *o = &d->object[i];
    if (o->supertype != DWG_SUPERTYPE_ENTITY || !o->tio.entity) continue;
    Dwg_Object_Entity *e = o->tio.entity;
    if (o->fixedtype == DWG_TYPE_BLOCK) blk = e->ownerhandle;
    else if (o->fixedtype == DWG_TYPE_ENDBLK) { if (!e->ownerhandle) e->ownerhandle = blk; blk = NULL; }
    else if (!e->ownerhandle && blk) e->ownerhandle = blk;
    if (e->entmode != 3) continue;
    uint64_t ow = e->ownerhandle ? e->ownerhandle->absolute_ref : 0;
    e->entmode = !ow || ow == ms ? 2 : ow == ps ? 1 : 0;
  }
}

static int looks_dxf(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) return 0;
  char b[32] = {0};
  size_t n = fread(b, 1, sizeof b - 1, f);
  fclose(f);
  if (n < 6 || !memcmp(b, "AC1", 3) || !memcmp(b, "AC2", 3) || !memcmp(b, "MC0", 3)) return 0;
  if (!memcmp(b, "AutoCAD Binary DXF", 18)) return 1;
  const char *p = b;
  if (!memcmp(p, "\xEF\xBB\xBF", 3)) p += 3;  // UTF-8 BOM
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
  return (p[0] == '0' || !strncmp(p, "999", 3)) && (p[1] == '\r' || p[1] == '\n' || p[1] == '9');
}

int main(int argc, char **argv) {
  if (argc == 2 && !strcmp(argv[1], "--sandbox-test")) {
    int st = sandbox_enter();
    printf("SANDBOX %s\n", SANDBOX_NAME[st]);
    return st == SANDBOX_ON ? (sandbox_selftest() ? 1 : 0) : 1;
  }
  int sandboxed = 0, page = 0;
  double img_w = 0, img_h = 0;  // -i W H: a picture, not a drawing (D111)
  const char *cmp_old = NULL;   // -c OLD.umg: compare caches, argv[1] = the new one (D116)
  while (argc > 3 && argv[1][0] == '-') {
    if (!strcmp(argv[1], "-s")) sandboxed = 1, argv++, argc--;
    else if (!strcmp(argv[1], "-p") && argc > 4) page = atoi(argv[2]), argv += 2, argc -= 2;
    else if (!strcmp(argv[1], "-l") && argc > 4) g_layout = atoi(argv[2]), argv += 2, argc -= 2;
    else if (!strcmp(argv[1], "-i") && argc > 5) img_w = atof(argv[2]), img_h = atof(argv[3]), argv += 3, argc -= 3;
    else if (!strcmp(argv[1], "-c") && argc > 4) cmp_old = argv[2], argv += 2, argc -= 2;
    else break;
  }
  if (page < 0 || page > 100000) page = 0;
  if (g_layout < 0 || g_layout > 1000) g_layout = 0;
  if (argc != 3) {
    fprintf(stderr, "usage: %s [-s] [-p page | -l layout | -i width height | -c old.umg] in.dwg out.umg\n", argv[0]);
    return 2;
  }
  // Sandboxed: the output must be open before the lockdown.
  FILE *out = NULL;
  if (sandboxed && !(out = fopen(argv[2], "wb"))) {
    fprintf(stderr, "cannot write %s\n", argv[2]);
    return 4;
  }
  double t0 = now_s();
  Dwg_Data dwg;
  memset(&dwg, 0, sizeof(dwg));
  Geom b;
  geom_init(&b, 1);  // q is set from the header extents in set_origin
  b.model = 1;
  Blocks blk = {0};
  Tables tab = {0};
  Ctx ctx = {&b, &blk, &tab, 0, 0};

  const char *ver = NULL;
  Dwg_Bitcode_3BD emin = {0, 0, 0}, emax = {0, 0, 0};
  unsigned long n_xclip = 0;
  PdfInfo pinfo;
  memset(&pinfo, 0, sizeof pinfo);
  int img = img_w >= 1 && img_h >= 1 && img_w <= 1e6 && img_h <= 1e6;
  int dxf = 0, pdf = !cmp_old && !img && !looks_dxf(argv[1]) && looks_pdf(argv[1]);
  size_t n_lay = 0;
  double lay_scale = 0;
  CmpStats cst;
  memset(&cst, 0, sizeof cst);
  if (cmp_old) {
    // Two caches made earlier (D116): read both, then lock down and compare.
    g_layout = 0;
    size_t na = 0, nb = 0;
    void *ra = cmp_load(cmp_old, &na), *rb = cmp_load(argv[1], &nb);
    if (sandboxed) enter_sandbox(NULL);
    int rc = ra && rb ? cmp_run(ra, na, rb, nb, &b, &tab, &cst) : 3;
    free(ra), free(rb);
    if (rc) {
      fprintf(stderr, "cannot compare\n");
      tables_free(&tab), geom_free(&b);
      return rc;
    }
    ver = "CMP";
    if (cst.x1 >= cst.x0) emin.x = cst.x0, emin.y = cst.y0, emax.x = cst.x1, emax.y = cst.y1;
  } else if (img) {
    // A photo or scan opened as a drawing: the app shows the picture itself;
    // the cache holds only its frame, one unit per pixel (the input file is
    // not read here).
    g_layout = 0;
    if (sandboxed) enter_sandbox(NULL);
    b.ox = img_w / 2, b.oy = img_h / 2, b.origin_set = 1;
    b.q = ldexp(1, (int)ceil(log2(fmax(img_w, img_h) / 1073741824.0)));
    tab.units = 4, tab.ltscale = 1;
    tab.layer0 = tables_new_layer(&tab, "0", 0);
    RawStyle k = {tab.layer0, LT_CONT, COL_TRUE | 0x808080, 1.0f};
    b.cur = style_intern(k);
    double fx[4] = {0, img_w, img_w, 0}, fy[4] = {0, 0, img_h, img_h};
    begin_poly(&b, 4, UMG_CLOSED | UMG_LTCONT);
    for (int i = 0; i < 4; i++) add_pt(&b, XF_ID, fx[i], fy[i], 0);
    ver = "IMG";
    emax.x = img_w, emax.y = img_h;
  } else if (pdf) {
    g_layout = 0;
    int rc = pdf_read(argv[1], &b, &tab, sandboxed ? enter_sandbox : NULL, page, &pinfo);
    if (rc) {
      fprintf(stderr, "pdf_read failed\n");
      tables_free(&tab), geom_free(&b);
      return rc;
    }
    ver = "PDF";
    emin.x = pinfo.x0, emin.y = pinfo.y0, emax.x = pinfo.x1, emax.y = pinfo.y1;
  } else {
  dwg_set_skip_eed(1);  // never read; some drawings carry hundreds of MB
  dwg_set_stream_callback(on_object, &ctx);
  if (sandboxed) dwg_set_before_decode(enter_sandbox, NULL);
  dxf = looks_dxf(argv[1]);
  ctx.dxf = dxf;
  // A DXF is read for an R2000 target: LibreDWG then keeps all its strings
  // as bytes (for R2007+ it stored table names as UTF-16 but other strings
  // as UTF-8, and looked names up wrongly: blocks and layers were lost).
  if (dxf) dwg.header.version = R_2000;
  int err = dxf ? dxf_read_file(argv[1], &dwg) : dwg_read_file(argv[1], &dwg);
  dwg_set_stream_callback(NULL, NULL);
  if (dxf && err < DWG_ERR_CRITICAL) dxf_fix_entmode(&dwg);
  if (err >= DWG_ERR_CRITICAL) {
    fprintf(stderr, "dwg_read_file failed: 0x%x\n", err);
    return 3;
  }
  if (!ctx.origin_set) set_origin(&ctx, &dwg);  // no self-contained entities
  // Streamed entities are gone from dwg.object; any self-contained one left
  // here was not streamed (pre-R2004 files have no object stream).
  for (BITCODE_BL i = 0; i < dwg.num_objects; i++) {
    Dwg_Object *o = &dwg.object[i];
    if (o->supertype == DWG_SUPERTYPE_ENTITY && is_self_contained(o))
      handle_self_contained(&ctx, o);
    else
      handle_kept(&ctx, o);
  }
  tab.ltscale = dwg.header_vars.LTSCALE > 0 ? dwg.header_vars.LTSCALE : 1;
  tab.units = (uint32_t)dwg.header_vars.INSUNITS;
  ver = dwg_version_type(dwg.header.from_version);
  emin = dwg.header_vars.EXTMIN, emax = dwg.header_vars.EXTMAX;
  // Layouts with something on the paper, for the app's list; -l N draws one.
  Lay *lays = NULL;
  n_lay = layouts_list(&ctx, &dwg, &lays);
  for (size_t i = 0; i < n_lay; i++) printf("LAYOUT %zu %s\n", i + 1, lays[i].name);
  if (g_layout) {
    if ((size_t)g_layout > n_lay) {
      fprintf(stderr, "no layout %d (of %zu)\n", g_layout, n_lay);
      return 5;
    }
    double box[4];
    layout_roots(&ctx, &lays[g_layout - 1], tab.layer0, box, &lay_scale);
    emin.x = box[0], emin.y = box[1], emax.x = box[2], emax.y = box[3];
  }
  for (size_t i = 0; i < n_lay; i++) free(lays[i].name);
  for (size_t i = 0; i < ctx.vps.n; i++) free(((Vp *)ctx.vps.p)[i].frozen);
  free(lays), free(ctx.vps.p), free(ctx.shapes.p), free(ctx.shape_pts.p);
  // A DXF read can leave pointers dwg_free must not free (LibreDWG 0.14:
  // an invalid free in dwg_free_LINE on one public drawing). The process
  // ends right after; let the system take the memory back.
  if (!dxf) dwg_free(&dwg);
  blk.layer0 = tab.layer0;
  blocks_expand(&blk, &b);
  n_xclip = blk.n_xclip;
  blocks_free(&blk);
  }  // DWG / DXF

  unsigned long overflow = 0;
  Serial ser;
  geom_serialize(&b, &tab, &ser, &overflow);
  if (overflow) fprintf(stderr, "styles over 65535: %lu\n", overflow);
  fprintf(stderr,
          "styles raw=%zu layers=%zu ltypes=%zu fills=%zu triangles=%zu "
          "hatch_lines=%zu dense_hatches=%lu sort_entries=%zu xclips=%lu dims_drawn=%lu "
          "hatch_work_max=%.0f hatch_work_total=%.0f fill_work_max=%.0f fill_work_total=%.0f\n",
          g_styles.n, tab.n_layers, tab.n_ltypes, b.fills.n, b.fpts.n / 3,
          b.hlines.n, n_hatch_dense, tab.sorts.n, n_xclip, g_dim_drawn, g_hatch_work_max, g_hatch_work_total,
          ec_work_max, ec_work_total);
  size_t n_layers = tab.n_layers;
  tables_free(&tab);

  // Compressed as it is written, part by part; the size goes in the frame
  // header (readers allocate from it).
  char tmp[4096];
  snprintf(tmp, sizeof tmp, "%s.tmp", argv[2]);
  FILE *f = sandboxed ? out : fopen(tmp, "wb");
  int ok = f != NULL;
  ZSTD_CCtx *cc = ZSTD_createCCtx();
  size_t zcap = ZSTD_CStreamOutSize();
  char *z = (char *)malloc(zcap);
  ok = ok && cc && z &&
       !ZSTD_isError(ZSTD_CCtx_setParameter(cc, ZSTD_c_compressionLevel, 3)) &&
       !ZSTD_isError(ZSTD_CCtx_setPledgedSrcSize(cc, ser.total));
  for (int i = 0; ok && i <= ser.n; i++) {
    int last = i == ser.n;
    ZSTD_inBuffer in = {last ? NULL : ser.p[i], last ? 0 : ser.len[i], 0};
    size_t left;
    do {
      ZSTD_outBuffer o = {z, zcap, 0};
      left = ZSTD_compressStream2(cc, &o, &in, last ? ZSTD_e_end : ZSTD_e_continue);
      if (ZSTD_isError(left) || fwrite(z, 1, o.pos, f) != o.pos) ok = 0;
    } while (ok && (last ? left != 0 : in.pos < in.size));
  }
  ZSTD_freeCCtx(cc);
  free(z);
  serial_free(&ser);
  if (f) ok = (fclose(f) == 0) && ok;
  if (!ok || (!sandboxed && rename(tmp, argv[2]) != 0)) {
    if (!sandboxed) remove(tmp);  // sandboxed: the caller deletes it
    fprintf(stderr, "cannot write %s\n", argv[2]);
    return 4;
  }
  // Unsupported types as NAME:count; names are LibreDWG's (no spaces).
  char skipped[1024] = "";
  for (int i = 0; i < g_n_unsup; i++) {
    size_t at = strlen(skipped);
    snprintf(skipped + at, sizeof skipped - at, "%s%s:%lu", i ? "," : "",
             g_unsup[i].name, g_unsup[i].n);
  }
  if (dxf && dxf_broken_count()) {  // DXF objects LibreDWG could not read (left out)
    size_t at = strlen(skipped);
    snprintf(skipped + at, sizeof skipped - at, "%sDXF_BROKEN:%lu", at ? "," : "", dxf_broken_count());
  }
  if (pdf && pinfo.texts_lost) {  // text in fonts we cannot map to characters
    size_t at = strlen(skipped);
    snprintf(skipped + at, sizeof skipped - at, "%sPDF_TEXT:%lu", at ? "," : "", pinfo.texts_lost);
  }
  if (pdf && pinfo.images) {  // pictures in a PDF (a scan is only that)
    size_t at = strlen(skipped);
    snprintf(skipped + at, sizeof skipped - at, "%sPDF_IMAGE:%lu", at ? "," : "", pinfo.images);
  }
  if (g_unsup_other) {
    size_t at = strlen(skipped);
    snprintf(skipped + at, sizeof skipped - at, "%sOTHER:%lu", at ? "," : "", g_unsup_other);
  }
  char pdfinfo[160] = "";
  if (pdf)  // pdf=measure: scale from the PDF; pdf=paper: millimetres on paper
    snprintf(pdfinfo, sizeof pdfinfo, " pdf=%s pages=%d units_per_pt=%.6g%s", pinfo.measured ? "measure" : "paper",
             pinfo.pages, pinfo.scale, pinfo.scan ? " scan=1" : "");
  // cmp=same,added,removed zones=x0,y0,x1,y1;... (바뀐 곳, reading order)
  char cmpinfo[96 + CMP_ZONES * 4 * 18] = "";
  if (cmp_old) {
    snprintf(cmpinfo, sizeof cmpinfo, " cmp=%lu,%lu,%lu", cst.same, cst.added, cst.removed);
    for (int z = 0; z < cst.nzone; z++) {
      size_t at = strlen(cmpinfo);
      snprintf(cmpinfo + at, sizeof cmpinfo - at, "%s%.10g,%.10g,%.10g,%.10g", z ? ";" : " zones=",
               cst.zone[z].x0, cst.zone[z].y0, cst.zone[z].x1, cst.zone[z].y1);
    }
  }
  char layinfo[96] = "";  // lscale: paper per model unit in the layout's main viewport
  if (n_lay) snprintf(layinfo, sizeof layinfo, " layouts=%zu layout=%d", n_lay, g_layout);
  if (n_lay && g_layout) snprintf(layinfo + strlen(layinfo), sizeof layinfo - strlen(layinfo), " lscale=%.6g", lay_scale);
  printf("INFO ver=%s units=%u ext=%.1f,%.1f,%.1f,%.1f layers=%zu paper=%lu "
         "prims=%zu texts=%zu skipped=%s%s%s%s%s%s\n",
         ver ? ver : "?", tab.units, emin.x, emin.y, emax.x, emax.y, n_layers,
         ctx.n_paper, b.lines.n + b.arcs.n + b.polys.n + b.fills.n + b.hlines.n,
         b.texts.n, skipped[0] ? skipped : "-", sandboxed ? " sandbox=" : "",
         sandboxed ? SANDBOX_NAME[g_sandbox] : "", pdfinfo, layinfo, cmpinfo);
  printf("DONE %.0f %zu %zu\n", (now_s() - t0) * 1e3,
         b.lines.n + b.arcs.n + b.polys.n + b.fills.n + b.hlines.n, b.texts.n);
  geom_free(&b);
  return 0;
}
