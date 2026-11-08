// unimog-convert: DWG -> UMG8 cache (exact fixed-point primitives + text,
// styles, layers, linetypes; zstd). Runs as a separate process from the app
// (GPL boundary, D31): the app only reads the UMG8 file this writes.
//   unimog-convert in.dwg out.umg
// stdout protocol, one line each:
//   PROGRESS <objects decoded>      every 100k objects
//   INFO key=value ...               drawing facts for the app (D86), before DONE
//   DONE <ms> <primitives> <texts>  on success
// Exit codes: 0 ok, 2 usage, 3 unreadable DWG, 4 cannot write output.
// The output is written to out.umg.tmp and renamed, so a killed process
// never leaves a half-written cache behind.
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zstd.h>

#include "blocks.h"

typedef struct {
  Geom *b;
  Blocks *blk;
  Tables *tab;
  int origin_set;
  unsigned long n_seen;
  unsigned long n_paper;  // entities drawn only on layouts (paper space), D87
} Ctx;

// Paper space is not drawn (D87): count what is there so the app can say
// "this drawing lives on its layouts". Viewports are frames, not content.
static void count_paper(Ctx *c, const Dwg_Object *o) {
  if (o->fixedtype != DWG_TYPE_VIEWPORT) c->n_paper++;
}

// Origin = center of the header extents; q = the smallest power of two that
// keeps +-2x the extents inside int32. Stale or missing extents fall back to
// a 10 km span around 0; overflow is counted as clamped.
static void set_origin(Ctx *c, Dwg_Data *dwg) {
  Dwg_Bitcode_3BD lo = dwg->header_vars.EXTMIN, hi = dwg->header_vars.EXTMAX;
  double span = fmax(hi.x - lo.x, hi.y - lo.y);
  int ok = isfinite(span) && span > 0 && span < 1e12 && fabs(lo.x) < 1e15 &&
           fabs(lo.y) < 1e15;
  Geom *b = c->b;
  b->ox = ok ? (lo.x + hi.x) / 2 : 0, b->oy = ok ? (lo.y + hi.y) / 2 : 0;
  b->q = c->blk->q = ldexp(1, (int)ceil(log2((ok ? span : 1e7) / 1073741824.0)));
  b->origin_set = 1;
}

// Self-contained entity: converted straight into model space or its block.
static void handle_self_contained(Ctx *c, const Dwg_Object *o) {
  Dwg_Object_Entity *e = o->tio.entity;
  if (e->invisible & 1) return;  // hidden
  if (e->entmode == 1) { count_paper(c, o); return; }  // active layout
  if (e->entmode == 2)
    convert_entity(c->b, o);
  else
    convert_entity(&block_get(c->blk, ref_handle(e->ownerhandle))->geom, o);
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
  } else if (o->tio.entity->entmode == 1) {
    count_paper(c, o);
  } else if (o->tio.entity->entmode == 2) {
    convert_entity(c->b, o);  // unsupported types are counted as skipped
  } else if (o->tio.entity->entmode == 0 && o->fixedtype != DWG_TYPE_VERTEX_2D &&
             o->fixedtype != DWG_TYPE_VERTEX_3D && o->fixedtype != DWG_TYPE_SEQEND &&
             o->fixedtype != DWG_TYPE_ATTDEF) {
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
  if (!c->origin_set) {  // header is decoded before the object stream
    set_origin(c, dwg);
    c->origin_set = 1;
  }
  handle_self_contained(c, o);
  return 1;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "usage: %s in.dwg out.umg\n", argv[0]);
    return 2;
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

  dwg_set_skip_eed(1);  // never read; some drawings carry hundreds of MB
  dwg_set_stream_callback(on_object, &ctx);
  int err = dwg_read_file(argv[1], &dwg);
  dwg_set_stream_callback(NULL, NULL);
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
  const char *ver = dwg_version_type(dwg.header.from_version);
  Dwg_Bitcode_3BD emin = dwg.header_vars.EXTMIN, emax = dwg.header_vars.EXTMAX;
  dwg_free(&dwg);
  blk.layer0 = tab.layer0;
  blocks_expand(&blk, &b);
  unsigned long n_xclip = blk.n_xclip;
  blocks_free(&blk);

  size_t raw_n;
  unsigned long overflow = 0;
  char *raw = geom_serialize(&b, &tab, &raw_n, &overflow);
  if (overflow) fprintf(stderr, "styles over 65535: %lu\n", overflow);
  fprintf(stderr,
          "styles raw=%zu layers=%zu ltypes=%zu fills=%zu triangles=%zu "
          "hatch_lines=%zu dense_hatches=%lu sort_entries=%zu xclips=%lu dims_drawn=%lu\n",
          g_styles.n, tab.n_layers, tab.n_ltypes, b.fills.n, b.fpts.n / 3,
          b.hlines.n, n_hatch_dense, tab.sorts.n, n_xclip, g_dim_drawn);
  size_t n_layers = tab.n_layers;
  tables_free(&tab);
  size_t zcap = ZSTD_compressBound(raw_n);
  char *z = (char *)malloc(zcap);
  size_t z_n = ZSTD_compress(z, zcap, raw, raw_n, 3);
  free(raw);

  char tmp[4096];
  snprintf(tmp, sizeof tmp, "%s.tmp", argv[2]);
  FILE *f = fopen(tmp, "wb");
  int ok = f && !ZSTD_isError(z_n) && fwrite(z, 1, z_n, f) == z_n;
  if (f) ok = (fclose(f) == 0) && ok;
  free(z);
  if (!ok || rename(tmp, argv[2]) != 0) {
    remove(tmp);
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
  if (g_unsup_other) {
    size_t at = strlen(skipped);
    snprintf(skipped + at, sizeof skipped - at, "%sOTHER:%lu", at ? "," : "", g_unsup_other);
  }
  printf("INFO ver=%s units=%u ext=%.1f,%.1f,%.1f,%.1f layers=%zu paper=%lu "
         "prims=%zu texts=%zu skipped=%s\n",
         ver ? ver : "?", tab.units, emin.x, emin.y, emax.x, emax.y, n_layers,
         ctx.n_paper, b.lines.n + b.arcs.n + b.polys.n + b.fills.n + b.hlines.n,
         b.texts.n, skipped[0] ? skipped : "-");
  printf("DONE %.0f %zu %zu\n", (now_s() - t0) * 1e3,
         b.lines.n + b.arcs.n + b.polys.n + b.fills.n + b.hlines.n, b.texts.n);
  geom_free(&b);
  return 0;
}
