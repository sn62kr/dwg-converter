// PDF drawings (D105): the vector content of one page into the same Geom /
// Tables a DWG fills, so a PDF opens, snaps, measures and hides layers like
// a DWG. Read entirely in memory, then (with -s) the process is locked down
// before any parsing, exactly like a DWG.
//
// What is read: the cross-reference table or stream (with /Prev chains and
// object streams; a damaged file falls back to scanning for "N G obj"), the
// page tree, content streams (FlateDecode, PNG predictors), paths (lines,
// Bezier curves flattened, rectangles), stroke and fill colours, filled
// areas (even-odd via the hatch filler), Form XObjects, optional content
// (OCG) as layers, text with ToUnicode / simple encodings, and the page's
// measuring viewport (/VP /Measure, written by AutoCAD's PDF plotter) as the
// drawing scale. Images are counted (a scanned PDF has nothing to measure).
//
// Snapping: CAD plotters write SHX text, hatch dots and dash pieces as tiny
// separate paths. Paths whose bounding box is under PDF_SMALL_PT on paper
// are drawn as hatch lines (not snapped), like DWG hatches (OSNAPHATCH off).
//
// Hostile input: every count, depth and size is capped; the decompressed
// total has a budget; operators have a budget (Form XObjects can nest).

#include <zlib.h>

#define PDF_MAX_DEPTH 32          // nested arrays / dictionaries
#define PDF_MAX_FORM_DEPTH 8      // nested Form XObjects
#define PDF_MAX_STREAM (512u << 20)       // one decoded stream
#define PDF_MAX_INFLATE_TOTAL (1536ull << 20)  // all decoded streams
#define PDF_MAX_OPS 60000000ull   // content operators, forms included
#define PDF_MAX_OBJS 8000000u
#define PDF_MAX_OPERANDS 2048  // on the stack, per Form XObject level
#define PDF_SMALL_PT 6.0          // paths smaller than this (pt) are not snapped

// ---- values ---------------------------------------------------------------

enum { PV_NULL, PV_BOOL, PV_NUM, PV_NAME, PV_STR, PV_ARR, PV_DICT, PV_REF };

typedef struct Pv Pv;
struct Pv {
  int t;
  double num;      // NUM, BOOL
  uint32_t ref;    // REF object number
  char *s;         // NAME (no '/'), STR (decoded bytes)
  size_t len;      // STR length, ARR / DICT item count
  Pv **a;          // ARR items, DICT values
  char **k;        // DICT keys
  size_t soff;     // DICT followed by "stream": data offset in the file
  int has_stream;
};

typedef struct ArenaBlk { struct ArenaBlk *next; size_t used, cap; } ArenaBlk;

typedef struct {
  const uint8_t *b;
  size_t n;
  ArenaBlk *arena;
  // xref: per object number, type (0 none, 1 offset, 2 compressed),
  // offset or object-stream number, index in the object stream
  uint8_t *xt;
  size_t *xo;
  uint32_t *xi;
  uint32_t nobj;
  Pv **cache;
  uint8_t *loading;
  Pv *trailer;
  unsigned long long inflated;
  unsigned long n_bad_streams, n_filters_unsup;
} Pdf;

static Pv PV_NULL_V = {PV_NULL, 0, 0, NULL, 0, NULL, NULL, 0, 0};

static void *ar_alloc(Pdf *P, size_t n) {
  n = (n + 15) & ~(size_t)15;
  ArenaBlk *a = P->arena;
  if (!a || a->used + n > a->cap) {
    size_t cap = n > 65536 ? n : 65536;
    a = (ArenaBlk *)malloc(sizeof(ArenaBlk) + cap);
    if (!a) { fprintf(stderr, "out of memory\n"); exit(5); }
    a->next = P->arena, a->used = 0, a->cap = cap;
    P->arena = a;
  }
  void *r = (char *)(a + 1) + a->used;
  a->used += n;
  return r;
}

static Pv *pv_new(Pdf *P, int t) {
  Pv *v = (Pv *)ar_alloc(P, sizeof(Pv));
  memset(v, 0, sizeof *v);
  v->t = t;
  return v;
}

static Pv *dict_get(const Pv *d, const char *key) {
  if (!d || d->t != PV_DICT) return NULL;
  for (size_t i = 0; i < d->len; i++)
    if (!strcmp(d->k[i], key)) return d->a[i];
  return NULL;
}

// ---- lexer ----------------------------------------------------------------

static int pdf_ws(int c) { return c == 0 || c == 9 || c == 10 || c == 12 || c == 13 || c == 32; }
static int pdf_delim(int c) {
  return c == '(' || c == ')' || c == '<' || c == '>' || c == '[' || c == ']' ||
         c == '{' || c == '}' || c == '/' || c == '%';
}

static size_t skip_ws(const uint8_t *b, size_t n, size_t i) {
  while (i < n) {
    if (pdf_ws(b[i])) i++;
    else if (b[i] == '%') { while (i < n && b[i] != '\n' && b[i] != '\r') i++; }
    else break;
  }
  return i;
}

static int hexv(int c) {
  return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
}

// Literal string at b[i] == '(' -> decoded bytes into out (len); returns end.
static size_t lex_lit(const uint8_t *b, size_t n, size_t i, char *out, size_t *len) {
  int depth = 1;
  size_t o = 0;
  i++;
  while (i < n && depth > 0) {
    int c = b[i++];
    if (c == '\\' && i < n) {
      c = b[i++];
      switch (c) {
      case 'n': c = '\n'; break;
      case 'r': c = '\r'; break;
      case 't': c = '\t'; break;
      case 'b': c = '\b'; break;
      case 'f': c = '\f'; break;
      case '\r': if (i < n && b[i] == '\n') i++; continue;
      case '\n': continue;
      default:
        if (c >= '0' && c <= '7') {
          int v = c - '0';
          for (int k = 0; k < 2 && i < n && b[i] >= '0' && b[i] <= '7'; k++) v = v * 8 + (b[i++] - '0');
          c = v & 0xff;
        }
      }
    } else if (c == '(') depth++;
    else if (c == ')' && --depth == 0) break;
    if (out) out[o] = (char)c;
    o++;
  }
  *len = o;
  return i;
}

static size_t lex_hex(const uint8_t *b, size_t n, size_t i, char *out, size_t *len) {
  size_t o = 0;
  int hi = -1;
  i++;
  while (i < n && b[i] != '>') {
    int v = hexv(b[i++]);
    if (v < 0) continue;
    if (hi < 0) hi = v;
    else { if (out) out[o] = (char)(hi << 4 | v); o++, hi = -1; }
  }
  if (hi >= 0) { if (out) out[o] = (char)(hi << 4); o++; }
  *len = o;
  return i < n ? i + 1 : i;
}

static size_t lex_name(const uint8_t *b, size_t n, size_t i, char *out, size_t cap) {
  size_t o = 0;
  i++;  // '/'
  while (i < n && !pdf_ws(b[i]) && !pdf_delim(b[i])) {
    int c = b[i++];
    if (c == '#' && i + 1 < n && hexv(b[i]) >= 0 && hexv(b[i + 1]) >= 0)
      c = hexv(b[i]) << 4 | hexv(b[i + 1]), i += 2;
    if (o + 1 < cap) out[o++] = (char)c;
  }
  out[o] = 0;
  return i;
}

static size_t lex_num(const uint8_t *b, size_t n, size_t i, double *v) {
  // PDF numbers: [sign] digits [. digits] (no exponent). Own parser: the C
  // library's strtod aborted on some malformed runs (macOS, found by fuzzing).
  // Digits are gathered as an integer and divided once by an exact power of
  // ten (correctly rounded, as strtod for these). A malformed run (several
  // signs or points) is consumed whole, like viewers do.
  static const double P10[19] = {1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10,
                                 1e11, 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18};
  int neg = 0, seen_digit = 0, frac = 0, nd = 0, fd = 0, ex = 0;
  uint64_t m = 0;
  size_t s = i;
  while (i < n && (b[i] == '+' || b[i] == '-')) { if (i == s) neg = b[i] == '-'; i++; }
  for (; i < n; i++) {
    int c = b[i];
    if (c >= '0' && c <= '9') {
      seen_digit = 1;
      if (nd < 18 && (nd || c != '0')) m = m * 10 + (uint64_t)(c - '0'), nd++, fd += frac;
      else if (!nd && frac) fd++;          // leading zeros after the point
      else if (nd >= 18 && !frac) ex++;     // integer part beyond 18 digits
    } else if (c == '.' && !frac) {
      frac = 1;
    } else if (c == '.' || c == '+' || c == '-') {
      while (i < n && ((b[i] >= '0' && b[i] <= '9') || b[i] == '.' || b[i] == '+' || b[i] == '-')) i++;
      break;
    } else break;
  }
  double r = (double)m;
  if (fd > 0) r = fd <= 18 ? r / P10[fd] : r / P10[18] / pow(10, fd - 18);
  if (ex > 0) r *= pow(10, ex);
  *v = seen_digit && isfinite(r) ? (neg ? -r : r) : 0;
  return i > s ? i : s + 1;
}

static int is_numstart(int c) { return (c >= '0' && c <= '9') || c == '+' || c == '-' || c == '.'; }

// Parses one value at *pos (file-level syntax, refs allowed).
static Pv *parse_val(Pdf *P, const uint8_t *b, size_t n, size_t *pos, int depth) {
  size_t i = skip_ws(b, n, *pos);
  if (i >= n || depth > PDF_MAX_DEPTH) { *pos = n; return &PV_NULL_V; }
  int c = b[i];
  if (c == '/') {
    char nm[256];
    i = lex_name(b, n, i, nm, sizeof nm);
    Pv *v = pv_new(P, PV_NAME);
    v->s = (char *)ar_alloc(P, strlen(nm) + 1);
    strcpy(v->s, nm);
    *pos = i;
    return v;
  }
  if (c == '(' || (c == '<' && !(i + 1 < n && b[i + 1] == '<'))) {
    size_t len, e;
    e = c == '(' ? lex_lit(b, n, i, NULL, &len) : lex_hex(b, n, i, NULL, &len);
    Pv *v = pv_new(P, PV_STR);
    v->s = (char *)ar_alloc(P, len + 1);
    if (c == '(') lex_lit(b, n, i, v->s, &len); else lex_hex(b, n, i, v->s, &len);
    v->s[len] = 0, v->len = len;
    *pos = e;
    return v;
  }
  if (c == '[') {
    Pv *v = pv_new(P, PV_ARR);
    size_t cap = 0;
    i++;
    for (;;) {
      i = skip_ws(b, n, i);
      if (i >= n) break;
      if (b[i] == ']') { i++; break; }
      size_t before = i;
      Pv *it = parse_val(P, b, n, &i, depth + 1);
      if (i == before) i++;  // unparsable byte
      if (v->len == cap) {
        size_t nc = cap ? cap * 2 : 8;
        Pv **na = (Pv **)ar_alloc(P, nc * sizeof(Pv *));
        if (cap) memcpy(na, v->a, cap * sizeof(Pv *));
        v->a = na, cap = nc;
      }
      v->a[v->len++] = it;
    }
    *pos = i;
    return v;
  }
  if (c == '<') {  // "<<"
    Pv *v = pv_new(P, PV_DICT);
    size_t cap = 0;
    i += 2;
    for (;;) {
      i = skip_ws(b, n, i);
      if (i >= n) break;
      if (b[i] == '>' && i + 1 < n && b[i + 1] == '>') { i += 2; break; }
      if (b[i] != '/') {  // junk: skip a value
        size_t before = i;
        parse_val(P, b, n, &i, depth + 1);
        if (i == before) i++;
        continue;
      }
      char nm[256];
      i = lex_name(b, n, i, nm, sizeof nm);
      size_t before = i;
      Pv *it = parse_val(P, b, n, &i, depth + 1);
      if (i == before) i++;
      if (v->len == cap) {
        size_t nc = cap ? cap * 2 : 8;
        Pv **na = (Pv **)ar_alloc(P, nc * sizeof(Pv *));
        char **nk = (char **)ar_alloc(P, nc * sizeof(char *));
        if (cap) memcpy(na, v->a, cap * sizeof(Pv *)), memcpy(nk, v->k, cap * sizeof(char *));
        v->a = na, v->k = nk, cap = nc;
      }
      v->k[v->len] = (char *)ar_alloc(P, strlen(nm) + 1);
      strcpy(v->k[v->len], nm);
      v->a[v->len++] = it;
    }
    // "stream" right after the dictionary
    size_t j = skip_ws(b, n, i);
    if (j + 6 <= n && !memcmp(b + j, "stream", 6)) {
      j += 6;
      if (j < n && b[j] == '\r') j++;
      if (j < n && b[j] == '\n') j++;
      v->has_stream = 1, v->soff = j;
    }
    *pos = i;
    return v;
  }
  if (is_numstart(c)) {
    double a;
    i = lex_num(b, n, i, &a);
    // "N G R"?
    size_t j = skip_ws(b, n, i);
    if (a >= 0 && a == floor(a) && j < n && b[j] >= '0' && b[j] <= '9') {
      double g;
      size_t k = lex_num(b, n, j, &g);
      k = skip_ws(b, n, k);
      if (k < n && b[k] == 'R' && (k + 1 >= n || pdf_ws(b[k + 1]) || pdf_delim(b[k + 1]))) {
        Pv *v = pv_new(P, PV_REF);
        v->ref = a < 4e9 ? (uint32_t)a : 0;
        *pos = k + 1;
        return v;
      }
    }
    Pv *v = pv_new(P, PV_NUM);
    v->num = a;
    *pos = i;
    return v;
  }
  // keyword
  size_t s = i;
  while (i < n && !pdf_ws(b[i]) && !pdf_delim(b[i])) i++;
  if (i == s) i++;
  *pos = i;
  if (i - s == 4 && !memcmp(b + s, "true", 4)) { Pv *v = pv_new(P, PV_BOOL); v->num = 1; return v; }
  if (i - s == 5 && !memcmp(b + s, "false", 5)) return pv_new(P, PV_BOOL);
  return &PV_NULL_V;
}

// ---- objects ----------------------------------------------------------------

static Pv *pdf_obj(Pdf *P, uint32_t num);

static Pv *res(Pdf *P, Pv *v) {
  for (int k = 0; v && v->t == PV_REF && k < 8; k++) v = pdf_obj(P, v->ref);
  return v ? v : &PV_NULL_V;
}

static Pv *dget(Pdf *P, const Pv *d, const char *key) { return res(P, dict_get(d, key)); }

static double pv_num(const Pv *v, double def) { return v && (v->t == PV_NUM || v->t == PV_BOOL) ? v->num : def; }

static int name_is(const Pv *v, const char *s) { return v && v->t == PV_NAME && !strcmp(v->s, s); }

static uint8_t *stream_data(Pdf *P, Pv *d, size_t *len);

// Object at a file offset ("N G obj ..."): its value.
static Pv *obj_at(Pdf *P, size_t off) {
  if (off >= P->n) return &PV_NULL_V;
  size_t i = off;
  double a, g;
  i = skip_ws(P->b, P->n, i);
  if (i >= P->n || !is_numstart(P->b[i])) return &PV_NULL_V;
  i = lex_num(P->b, P->n, i, &a);
  i = skip_ws(P->b, P->n, i);
  i = lex_num(P->b, P->n, i, &g);
  i = skip_ws(P->b, P->n, i);
  if (i + 3 > P->n || memcmp(P->b + i, "obj", 3)) return &PV_NULL_V;
  i += 3;
  return parse_val(P, P->b, P->n, &i, 0);
}

static Pv *pdf_obj(Pdf *P, uint32_t num) {
  if (num >= P->nobj || !P->xt[num]) return &PV_NULL_V;
  if (P->cache[num]) return P->cache[num];
  if (P->loading[num]) return &PV_NULL_V;  // cycle
  P->loading[num] = 1;
  Pv *v = &PV_NULL_V;
  if (P->xt[num] == 1) {
    v = obj_at(P, P->xo[num]);
  } else if (P->xt[num] == 2) {
    Pv *st = pdf_obj(P, (uint32_t)P->xo[num]);
    if (st->t == PV_DICT && st->has_stream) {
      size_t len;
      uint8_t *d = stream_data(P, st, &len);
      if (d) {
        long nn = (long)pv_num(dget(P, st, "N"), 0), first = (long)pv_num(dget(P, st, "First"), 0);
        size_t pos = 0;
        for (long k = 0; k < nn && k < 1000000; k++) {
          double on, oo;
          pos = skip_ws(d, len, pos);
          if (pos >= len) break;
          pos = lex_num(d, len, pos, &on);
          pos = skip_ws(d, len, pos);
          pos = lex_num(d, len, pos, &oo);
          if ((uint32_t)on == num) {
            size_t at = (size_t)first + (size_t)oo;
            if (first >= 0 && oo >= 0 && at < len) v = parse_val(P, d, len, &at, 0);
            break;
          }
        }
        free(d);
      }
    }
  }
  P->loading[num] = 0;
  P->cache[num] = v;
  return v;
}

// ---- streams ------------------------------------------------------------------

static uint8_t *inflate_all(Pdf *P, const uint8_t *src, size_t n, size_t *out_len) {
  z_stream z;
  memset(&z, 0, sizeof z);
  if (inflateInit(&z) != Z_OK) return NULL;
  size_t cap = n * 4 + 1024, len = 0;
  if (cap > PDF_MAX_STREAM) cap = PDF_MAX_STREAM;
  uint8_t *o = (uint8_t *)malloc(cap);
  if (!o) { inflateEnd(&z); return NULL; }
  z.next_in = (Bytef *)src, z.avail_in = (uInt)(n > 0x7fffffff ? 0x7fffffff : n);
  for (;;) {
    if (len == cap) {
      if (cap >= PDF_MAX_STREAM || P->inflated + len >= PDF_MAX_INFLATE_TOTAL) break;
      size_t nc = cap * 2 > PDF_MAX_STREAM ? PDF_MAX_STREAM : cap * 2;
      uint8_t *no = (uint8_t *)realloc(o, nc);
      if (!no) break;
      o = no, cap = nc;
    }
    z.next_out = o + len, z.avail_out = (uInt)(cap - len);
    int r = inflate(&z, Z_NO_FLUSH);
    len = cap - z.avail_out;
    if (r == Z_STREAM_END) break;
    if (r != Z_OK) break;  // damaged: keep what came out (viewers do the same)
    if (z.avail_in == 0 && len < cap) break;
  }
  inflateEnd(&z);
  P->inflated += len;
  *out_len = len;
  return o;
}

static uint8_t *unpredict(uint8_t *d, size_t *len, int pred, int colors, int bpc, int cols) {
  if (pred < 10) return d;  // 1 none; 2 TIFF (not used by content/xref in practice)
  if (colors < 1 || colors > 32 || bpc < 1 || bpc > 16 || cols < 1 || cols > 1 << 20) return d;
  size_t bpp = ((size_t)colors * bpc + 7) / 8, row = ((size_t)colors * bpc * cols + 7) / 8;
  size_t nrows = *len / (row + 1);
  uint8_t *o = (uint8_t *)calloc(nrows ? nrows * row : 1, 1);
  if (!o) return d;
  for (size_t r = 0; r < nrows; r++) {
    const uint8_t *in = d + r * (row + 1);
    uint8_t *cur = o + r * row, *prev = r ? o + (r - 1) * row : NULL;
    int t = in[0];
    for (size_t x = 0; x < row; x++) {
      int a = x >= bpp ? cur[x - bpp] : 0, b = prev ? prev[x] : 0, c = prev && x >= bpp ? prev[x - bpp] : 0;
      int v = in[1 + x];
      switch (t) {
      case 1: v += a; break;
      case 2: v += b; break;
      case 3: v += (a + b) / 2; break;
      case 4: {
        int p = a + b - c, pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);
        v += pa <= pb && pa <= pc ? a : pb <= pc ? b : c;
        break;
      }
      }
      cur[x] = (uint8_t)v;
    }
  }
  free(d);
  *len = nrows * row;
  return o;
}

// Decoded data of a stream object (malloc'd), or NULL.
static uint8_t *stream_data(Pdf *P, Pv *d, size_t *len) {
  if (!d || d->t != PV_DICT || !d->has_stream || d->soff >= P->n) return NULL;
  size_t off = d->soff, n;
  Pv *L = dget(P, d, "Length");
  double ln = pv_num(L, -1);
  if (ln >= 0 && ln <= (double)(P->n - off))
    n = (size_t)ln;
  else {  // wrong or missing /Length: up to "endstream"
    const uint8_t *e = (const uint8_t *)memmem(P->b + off, P->n - off, "endstream", 9);
    n = e ? (size_t)(e - (P->b + off)) : P->n - off;
    while (n && (P->b[off + n - 1] == '\n' || P->b[off + n - 1] == '\r')) n--;
  }
  Pv *f = dget(P, d, "Filter"), *dp = dget(P, d, "DecodeParms");
  Pv *filters[8];
  Pv *parms[8];
  int nf = 0;
  if (f->t == PV_NAME) filters[nf] = f, parms[nf++] = dp;
  else if (f->t == PV_ARR)
    for (size_t k = 0; k < f->len && nf < 8; k++) {
      filters[nf] = res(P, f->a[k]);
      parms[nf++] = dp->t == PV_ARR && k < dp->len ? res(P, dp->a[k]) : (dp->t == PV_DICT ? dp : &PV_NULL_V);
    }
  uint8_t *cur = (uint8_t *)malloc(n ? n : 1);
  if (!cur) return NULL;
  memcpy(cur, P->b + off, n);
  size_t cl = n;
  for (int k = 0; k < nf; k++) {
    if (name_is(filters[k], "FlateDecode") || name_is(filters[k], "Fl")) {
      size_t ol;
      uint8_t *o = inflate_all(P, cur, cl, &ol);
      free(cur);
      if (!o) { P->n_bad_streams++; return NULL; }
      cur = o, cl = ol;
      Pv *pp = parms[k];
      int pred = (int)pv_num(dget(P, pp, "Predictor"), 1);
      if (pred > 1)
        cur = unpredict(cur, &cl, pred, (int)pv_num(dget(P, pp, "Colors"), 1),
                        (int)pv_num(dget(P, pp, "BitsPerComponent"), 8), (int)pv_num(dget(P, pp, "Columns"), 1));
    } else if (name_is(filters[k], "ASCIIHexDecode") || name_is(filters[k], "AHx")) {
      uint8_t *o = (uint8_t *)malloc(cl / 2 + 2);
      if (!o) { free(cur); return NULL; }
      size_t ol = 0;
      int hi = -1;
      for (size_t x = 0; x < cl && cur[x] != '>'; x++) {
        int v = hexv(cur[x]);
        if (v < 0) continue;
        if (hi < 0) hi = v; else o[ol++] = (uint8_t)(hi << 4 | v), hi = -1;
      }
      if (hi >= 0) o[ol++] = (uint8_t)(hi << 4);
      free(cur);
      cur = o, cl = ol;
    } else {
      P->n_filters_unsup++;
      free(cur);
      return NULL;
    }
  }
  *len = cl;
  return cur;
}

// ---- cross-reference ------------------------------------------------------------

static void xref_grow(Pdf *P, uint32_t need) {
  if (need <= P->nobj) return;
  if (need > PDF_MAX_OBJS) need = PDF_MAX_OBJS;
  uint32_t nn = P->nobj ? P->nobj : 64;
  while (nn < need) nn = nn * 2 > PDF_MAX_OBJS ? PDF_MAX_OBJS : nn * 2;
  P->xt = (uint8_t *)realloc(P->xt, nn);
  P->xo = (size_t *)realloc(P->xo, nn * sizeof(size_t));
  P->xi = (uint32_t *)realloc(P->xi, nn * sizeof(uint32_t));
  memset(P->xt + P->nobj, 0, nn - P->nobj);
  P->nobj = nn;
}

static void xref_set(Pdf *P, uint64_t num, int type, size_t off, uint32_t idx) {
  if (num >= PDF_MAX_OBJS) return;
  xref_grow(P, (uint32_t)num + 1);
  if (num >= P->nobj || P->xt[num]) return;  // newer section already set it
  P->xt[num] = (uint8_t)type, P->xo[num] = off, P->xi[num] = idx;
}

// One xref section at off (table or stream); returns its trailer dict.
static Pv *xref_section(Pdf *P, size_t off) {
  if (off >= P->n) return NULL;
  size_t i = skip_ws(P->b, P->n, off);
  if (i + 4 <= P->n && !memcmp(P->b + i, "xref", 4)) {
    i += 4;
    for (int guard = 0; guard < 100000; guard++) {
      i = skip_ws(P->b, P->n, i);
      if (i >= P->n || !(P->b[i] >= '0' && P->b[i] <= '9')) break;
      double s, c;
      i = lex_num(P->b, P->n, i, &s);
      i = skip_ws(P->b, P->n, i);
      i = lex_num(P->b, P->n, i, &c);
      if (s < 0 || c < 0 || c > PDF_MAX_OBJS) break;
      for (uint64_t k = 0; k < (uint64_t)c; k++) {
        double o, g;
        i = skip_ws(P->b, P->n, i);
        if (i >= P->n) break;
        i = lex_num(P->b, P->n, i, &o);
        i = skip_ws(P->b, P->n, i);
        i = lex_num(P->b, P->n, i, &g);
        i = skip_ws(P->b, P->n, i);
        int used = i < P->n && P->b[i] == 'n';
        if (i < P->n) i++;
        if (used && o > 0 && o < (double)P->n) xref_set(P, (uint64_t)s + k, 1, (size_t)o, 0);
        else if (!used) xref_set(P, (uint64_t)s + k, 0, 0, 0);
      }
    }
    i = skip_ws(P->b, P->n, i);
    if (i + 7 <= P->n && !memcmp(P->b + i, "trailer", 7)) {
      i += 7;
      Pv *t = parse_val(P, P->b, P->n, &i, 0);
      return t->t == PV_DICT ? t : NULL;
    }
    return NULL;
  }
  // xref stream
  Pv *d = obj_at(P, i);
  if (d->t != PV_DICT || !name_is(dict_get(d, "Type"), "XRef")) return NULL;
  size_t len;
  uint8_t *data = stream_data(P, d, &len);
  if (!data) return d;
  Pv *W = dget(P, d, "W"), *I = dget(P, d, "Index");
  int w[3] = {0, 0, 0};
  if (W->t == PV_ARR && W->len >= 3)
    for (int k = 0; k < 3; k++) w[k] = (int)pv_num(res(P, W->a[k]), 0);
  size_t rec = (size_t)w[0] + w[1] + w[2];
  if (w[0] < 0 || w[1] < 0 || w[2] < 0 || w[0] > 8 || w[1] > 8 || w[2] > 8 || !rec) { free(data); return d; }
  double size = pv_num(dget(P, d, "Size"), 0);
  size_t pos = 0;
  size_t nsub = I->t == PV_ARR ? I->len / 2 : 1;
  for (size_t sct = 0; sct < nsub; sct++) {
    double start = I->t == PV_ARR ? pv_num(res(P, I->a[2 * sct]), 0) : 0;
    double cnt = I->t == PV_ARR ? pv_num(res(P, I->a[2 * sct + 1]), 0) : size;
    if (start < 0 || cnt < 0) break;
    for (uint64_t k = 0; k < (uint64_t)cnt && pos + rec <= len; k++) {
      uint64_t f[3];
      for (int c = 0; c < 3; c++) {
        f[c] = 0;
        for (int b = 0; b < w[c]; b++) f[c] = f[c] << 8 | data[pos++];
      }
      int type = w[0] ? (int)f[0] : 1;
      if (type == 1 && f[1] < P->n) xref_set(P, (uint64_t)start + k, 1, (size_t)f[1], 0);
      else if (type == 2) xref_set(P, (uint64_t)start + k, 2, (size_t)f[1], (uint32_t)f[2]);
      else if (type == 0) xref_set(P, (uint64_t)start + k, 0, 0, 0);
    }
  }
  free(data);
  return d;
}

// Damaged file: every "N G obj" (the last one wins), trailer from "trailer"
// or from an XRef stream with /Root.
static void xref_scan(Pdf *P) {
  memset(P->xt, 0, P->nobj);
  P->trailer = NULL;
  for (size_t i = 0; i + 5 < P->n; i++) {
    if (P->b[i] != 'o' || memcmp(P->b + i, "obj", 3)) continue;
    if (i + 3 < P->n && !pdf_ws(P->b[i + 3]) && !pdf_delim(P->b[i + 3])) continue;
    // back over "N G "
    size_t j = i;
    while (j > 0 && pdf_ws(P->b[j - 1])) j--;
    size_t ge = j;
    while (j > 0 && P->b[j - 1] >= '0' && P->b[j - 1] <= '9') j--;
    if (j == ge) continue;
    while (j > 0 && pdf_ws(P->b[j - 1])) j--;
    size_t ne = j;
    while (j > 0 && P->b[j - 1] >= '0' && P->b[j - 1] <= '9') j--;
    if (j == ne || ne - j > 9) continue;
    unsigned long long num = strtoull((const char *)P->b + j, NULL, 10);
    if (num >= PDF_MAX_OBJS) continue;
    xref_grow(P, (uint32_t)num + 1);
    P->xt[num] = 1, P->xo[num] = j;
  }
  const uint8_t *t = NULL;
  for (size_t i = P->n; i > 7; i--)
    if (!memcmp(P->b + i - 7, "trailer", 7)) { t = P->b + i; break; }
  if (t) {
    size_t i = (size_t)(t - P->b);
    Pv *tr = parse_val(P, P->b, P->n, &i, 0);
    if (tr->t == PV_DICT && dict_get(tr, "Root")) { P->trailer = tr; return; }
  }
  for (uint32_t k = 0; k < P->nobj; k++) {
    if (P->xt[k] != 1) continue;
    Pv *d = obj_at(P, P->xo[k]);
    if (d->t == PV_DICT && dict_get(d, "Root")) { P->trailer = d; return; }
  }
}

static int pdf_open(Pdf *P) {
  size_t from = P->n > 2048 ? P->n - 2048 : 0;
  const uint8_t *sx = NULL;
  for (size_t i = P->n; i >= from + 9; i--)
    if (!memcmp(P->b + i - 9, "startxref", 9)) { sx = P->b + i; break; }
  xref_grow(P, 64);
  if (sx) {
    double off;
    size_t i = skip_ws(P->b, P->n, (size_t)(sx - P->b));
    lex_num(P->b, P->n, i, &off);
    size_t seen[32];
    int ns = 0;
    while (off > 0 && off < (double)P->n && ns < 32) {
      int dup = 0;
      for (int k = 0; k < ns; k++) dup |= seen[k] == (size_t)off;
      if (dup) break;
      seen[ns++] = (size_t)off;
      Pv *tr = xref_section(P, (size_t)off);
      if (!tr) break;
      if (!P->trailer) P->trailer = tr;
      double xs = pv_num(dict_get(tr, "XRefStm"), -1);  // hybrid file
      if (xs > 0 && xs < (double)P->n) xref_section(P, (size_t)xs);
      off = pv_num(dict_get(tr, "Prev"), -1);
    }
  }
  P->cache = (Pv **)calloc(P->nobj, sizeof(Pv *));
  P->loading = (uint8_t *)calloc(P->nobj, 1);
  int ok = P->trailer && dget(P, P->trailer, "Root")->t == PV_DICT;
  if (!ok) {
    xref_scan(P);
    free(P->cache), free(P->loading);
    P->cache = (Pv **)calloc(P->nobj, sizeof(Pv *));
    P->loading = (uint8_t *)calloc(P->nobj, 1);
    ok = P->trailer && dget(P, P->trailer, "Root")->t == PV_DICT;
  }
  return ok;
}

static void pdf_close(Pdf *P) {
  while (P->arena) { ArenaBlk *n = P->arena->next; free(P->arena); P->arena = n; }
  free(P->xt), free(P->xo), free(P->xi), free(P->cache), free(P->loading);
  memset(P, 0, sizeof *P);
}

// ---- pages ----------------------------------------------------------------------

typedef struct {
  Pv *page, *resources, *box;
  int rotate;
} PageRef;

// The want-th page (0-based); *count gets the total. Inherited attributes
// (Resources, MediaBox, CropBox, Rotate) come down the tree.
static void find_page(Pdf *P, Pv *node, int depth, long want, long *count, PageRef *out,
                      Pv *res_in, Pv *box_in, int rot_in) {
  if (!node || node->t != PV_DICT || depth > 32 || *count > 100000) return;
  Pv *r = dget(P, node, "Resources");
  Pv *box = dget(P, node, "CropBox");
  if (box->t != PV_ARR) box = dget(P, node, "MediaBox");
  Pv *rot = dget(P, node, "Rotate");
  Pv *rs = r->t == PV_DICT ? r : res_in;
  Pv *bx = box->t == PV_ARR ? box : box_in;
  int ro = rot->t == PV_NUM ? (int)rot->num : rot_in;
  Pv *kids = dget(P, node, "Kids");
  if (kids->t == PV_ARR && !name_is(dict_get(node, "Type"), "Page")) {
    for (size_t k = 0; k < kids->len; k++) find_page(P, res(P, kids->a[k]), depth + 1, want, count, out, rs, bx, ro);
    return;
  }
  if (*count == want) out->page = node, out->resources = rs, out->box = bx, out->rotate = ro;
  (*count)++;
}

// ---- content --------------------------------------------------------------------

enum { TK_NUM, TK_NAME, TK_STR, TK_ARR_B, TK_ARR_E, TK_DICT, TK_OP, TK_END };

typedef struct {
  int t;
  double v;
  const uint8_t *s;  // NAME: chars; STR: raw (with '(' or '<'); OP: chars
  size_t n;
} Tok;

static size_t next_tok(const uint8_t *b, size_t n, size_t i, Tok *t) {
  i = skip_ws(b, n, i);
  if (i >= n) { t->t = TK_END; return n; }
  int c = b[i];
  if (c == '/') {
    size_t s = ++i;
    while (i < n && !pdf_ws(b[i]) && !pdf_delim(b[i])) i++;
    t->t = TK_NAME, t->s = b + s, t->n = i - s;
    return i;
  }
  if (c == '(') {
    size_t len, e = lex_lit(b, n, i, NULL, &len);
    t->t = TK_STR, t->s = b + i, t->n = e - i;
    return e;
  }
  if (c == '<' && i + 1 < n && b[i + 1] == '<') {  // inline dict (BDC, ...): skipped
    int depth = 0;
    while (i < n) {
      if (b[i] == '<' && i + 1 < n && b[i + 1] == '<') depth++, i += 2;
      else if (b[i] == '>' && i + 1 < n && b[i + 1] == '>') { i += 2; if (--depth == 0) break; }
      else if (b[i] == '(') { size_t len; i = lex_lit(b, n, i, NULL, &len); }
      else i++;
    }
    t->t = TK_DICT;
    return i;
  }
  if (c == '<') {
    size_t s = i;
    while (i < n && b[i] != '>') i++;
    if (i < n) i++;
    t->t = TK_STR, t->s = b + s, t->n = i - s;
    return i;
  }
  if (c == '[') { t->t = TK_ARR_B; return i + 1; }
  if (c == ']') { t->t = TK_ARR_E; return i + 1; }
  if (is_numstart(c)) {
    i = lex_num(b, n, i, &t->v);
    t->t = TK_NUM;
    return i;
  }
  size_t s = i;
  while (i < n && !pdf_ws(b[i]) && !pdf_delim(b[i])) i++;
  if (i == s) i++;  // stray delimiter
  t->t = TK_OP, t->s = b + s, t->n = i - s;
  return i;
}

typedef struct { double a, b, c, d, e, f; } M6;
static M6 m6_mul(M6 p, M6 q) {  // p then q
  M6 r = {p.a * q.a + p.b * q.c, p.a * q.b + p.b * q.d, p.c * q.a + p.d * q.c,
          p.c * q.b + p.d * q.d, p.e * q.a + p.f * q.c + q.e, p.e * q.b + p.f * q.d + q.f};
  return r;
}

typedef struct {
  M6 ctm;
  double lw;
  uint32_t srgb, frgb;  // 0xRRGGBB
  int fill_n, stroke_n; // colour components of the current colour space
  // text state
  double tc, tw, th, tl, tfs, trise;
  int tr;
  Pv *font;
} GState;

typedef struct {
  uint32_t *map;   // code -> first code point (0 = none)
  // Type0 fonts: CID widths (/W, 1/1000 em; 0 = /DW) and, for a code that
  // maps to no character, the embedded TrueType outlines to draw instead
  uint16_t *cw;
  double dw;
  uint8_t *tt;
  size_t ttn;
  uint32_t glyf_off, glyf_len, loca_off, loca_len, nglyphs;
  int loca_long;
  double upm;
  uint16_t *cid2gid;
  size_t ncid;
  uint32_t nmap;
  int two_byte;    // Type0 / Identity: 2-byte codes
  double w[256];   // simple font widths (1/1000 em), 0 = unknown
  int has_w;
  Pv *dict;
} FontInfo;

typedef struct {
  Pdf *P;
  Geom *g;
  Tables *tab;
  Xf out;            // page points -> drawing units (rotation, scale)
  double scale;      // drawing units per point
  int rot;           // page rotation (degrees, 0/90/180/270)
  // layers: OCG object number -> layer handle
  uint32_t *ocg_num;
  uint64_t *ocg_h;
  size_t n_ocg;
  uint64_t oc_stack[64];
  int oc_n;
  uint64_t cur_layer;
  // path in page space
  Vec xy;      // double[2]
  Vec sub;     // size_t: first point of each subpath
  Vec closed;  // uint8 per subpath
  Vec curved;  // uint8 per subpath: has Bezier segments
  double cx, cy, sx, sy;  // current point, subpath start (user space)
  int has_cur;
  GState gs[64];
  int gn;
  // text
  M6 tm, tlm;
  FontInfo fonts[64];
  Pv *font_keys[64];
  int nfonts;
  unsigned long long ops;
  unsigned long n_images, n_paths, n_small, n_text, n_text_lost, n_glyphs;
  int form_depth;
  uint64_t ord;
} Interp;

static uint32_t rgb3(double r, double g, double b) {
  int R = (int)(fmin(fmax(r, 0), 1) * 255 + .5), G = (int)(fmin(fmax(g, 0), 1) * 255 + .5), B = (int)(fmin(fmax(b, 0), 1) * 255 + .5);
  return (uint32_t)R << 16 | (uint32_t)G << 8 | (uint32_t)B;
}

static uint32_t comps_rgb(const double *v, int n) {
  if (n == 1) return rgb3(v[0], v[0], v[0]);
  if (n == 3) return rgb3(v[0], v[1], v[2]);
  if (n == 4) return rgb3((1 - v[0]) * (1 - v[3]), (1 - v[1]) * (1 - v[3]), (1 - v[2]) * (1 - v[3]));
  return 0;
}

// Paper colours onto the dark canvas: black ink draws white (like ACI 7),
// white paint is the background (it covers, like a WIPEOUT).
static uint32_t ink_color(uint32_t rgb) {
  int r = rgb >> 16 & 255, g = rgb >> 8 & 255, b = rgb & 255;
  if (r < 48 && g < 48 && b < 48) return COL_TRUE | 0xffffff;
  if (r > 240 && g > 240 && b > 240) return COL_BG;
  return COL_TRUE | rgb;
}

static void set_style(Interp *I, uint32_t color) {
  RawStyle k = {I->cur_layer, LT_CONT, color, 1.0f};
  I->g->cur = style_intern(k);
}

static void path_reset(Interp *I) { I->xy.n = I->sub.n = I->closed.n = I->curved.n = 0; I->has_cur = 0; }

static void path_pt(Interp *I, double x, double y) {
  M6 m = I->gs[I->gn].ctm;
  double *p = (double *)vec_push(&I->xy);
  p[0] = m.a * x + m.c * y + m.e, p[1] = m.b * x + m.d * y + m.f;
  if (!isfinite(p[0]) || !isfinite(p[1])) p[0] = p[1] = 0;
}

static void path_move(Interp *I, double x, double y) {
  // a lone moveto replaces the previous empty subpath
  if (I->sub.n && I->xy.n - ((size_t *)I->sub.p)[I->sub.n - 1] == 1) {
    I->xy.n--, I->sub.n--, I->closed.n--, I->curved.n--;
  }
  *(size_t *)vec_push(&I->sub) = I->xy.n;
  *(uint8_t *)vec_push(&I->closed) = 0;
  *(uint8_t *)vec_push(&I->curved) = 0;
  path_pt(I, x, y);
  I->cx = I->sx = x, I->cy = I->sy = y, I->has_cur = 1;
}

static void path_line(Interp *I, double x, double y) {
  if (!I->has_cur) { path_move(I, x, y); return; }
  path_pt(I, x, y);
  I->cx = x, I->cy = y;
}

static void path_curve(Interp *I, double x1, double y1, double x2, double y2, double x3, double y3) {
  if (!I->has_cur) path_move(I, x1, y1);
  double x0 = I->cx, y0 = I->cy;
  // segments from the control polygon's length in page points
  M6 m = I->gs[I->gn].ctm;
  double sc = sqrt(fabs(m.a * m.d - m.b * m.c));
  double L = (hypot(x1 - x0, y1 - y0) + hypot(x2 - x1, y2 - y1) + hypot(x3 - x2, y3 - y2)) * sc;
  int k = L < 2 ? 2 : L < 20 ? 6 : L < 200 ? 16 : 32;
  for (int i = 1; i <= k; i++) {
    double t = (double)i / k, u = 1 - t;
    double x = u * u * u * x0 + 3 * u * u * t * x1 + 3 * u * t * t * x2 + t * t * t * x3;
    double y = u * u * u * y0 + 3 * u * u * t * y1 + 3 * u * t * t * y2 + t * t * t * y3;
    path_pt(I, x, y);
  }
  if (I->curved.n) ((uint8_t *)I->curved.p)[I->curved.n - 1] = 1;
  I->cx = x3, I->cy = y3;
}

static void path_close(Interp *I) {
  if (!I->sub.n) return;
  ((uint8_t *)I->closed.p)[I->closed.n - 1] = 1;
  I->cx = I->sx, I->cy = I->sy;
}

static void paint(Interp *I, int stroke, int fill) {
  const double *xy = (const double *)I->xy.p;
  size_t ns = I->sub.n;
  if (!ns || (!stroke && !fill)) { path_reset(I); return; }
  const size_t *st = (const size_t *)I->sub.p;
  const uint8_t *cl = (const uint8_t *)I->closed.p;
  double x0 = INFINITY, y0 = INFINITY, x1 = -INFINITY, y1 = -INFINITY;
  for (size_t k = 0; k < I->xy.n; k++) {
    x0 = fmin(x0, xy[2 * k]), x1 = fmax(x1, xy[2 * k]);
    y0 = fmin(y0, xy[2 * k + 1]), y1 = fmax(y1, xy[2 * k + 1]);
  }
  int small = fmax(x1 - x0, y1 - y0) < PDF_SMALL_PT;
  Geom *g = I->g;
  I->n_paths++;
  g->ord.root = ++I->ord, g->ord.inner = 0;
  if (fill) {
    uint32_t c = ink_color(I->gs[I->gn].frgb);
    set_style(I, c);
    Loops L;
    memset(&L, 0, sizeof L);
    L.xy.elem = 2 * sizeof(double), L.start.elem = sizeof(size_t);
    for (size_t s = 0; s < ns; s++) {
      size_t a = st[s], b = s + 1 < ns ? st[s + 1] : I->xy.n;
      if (b - a < 3) continue;
      *(size_t *)vec_push(&L.start) = L.xy.n;
      vec_append(&L.xy, xy + 2 * a, b - a);
    }
    if (L.start.n) fill_loops(g, I->out, &L, 0);
    free(L.xy.p), free(L.start.p);
  }
  if (stroke) {
    uint32_t c = ink_color(I->gs[I->gn].srgb);
    if (c == COL_BG) { path_reset(I); return; }  // white on paper: not drawn
    set_style(I, c);
    if (small) I->n_small++;
    for (size_t s = 0; s < ns; s++) {
      size_t a = st[s], b = s + 1 < ns ? st[s + 1] : I->xy.n;
      size_t np = b - a;
      if (np < 2) continue;
      if (small) {
        for (size_t k = a; k + 1 < b; k++)
          add_hline(g, I->out, xy[2 * k], xy[2 * k + 1], xy[2 * k + 2], xy[2 * k + 3]);
        if (cl[s]) add_hline(g, I->out, xy[2 * (b - 1)], xy[2 * (b - 1) + 1], xy[2 * a], xy[2 * a + 1]);
      } else if (np == 2 && !cl[s]) {
        add_line(g, I->out, xy[2 * a], xy[2 * a + 1], xy[2 * a + 2], xy[2 * a + 3]);
      } else {
        // drop a closing point equal to the start; the flag closes it
        size_t e = b;
        if (cl[s] && np > 2 && xy[2 * a] == xy[2 * (b - 1)] && xy[2 * a + 1] == xy[2 * (b - 1) + 1]) e--;
        begin_poly(g, (uint32_t)(e - a), cl[s] ? UMG_CLOSED : 0);
        for (size_t k = a; k < e; k++) add_pt(g, I->out, xy[2 * k], xy[2 * k + 1], 0);
      }
    }
  }
  path_reset(I);
}

// ---- layers (optional content) ---------------------------------------------------

static uint64_t layer_for(Interp *I, Pv *ref) {
  if (!ref || ref->t != PV_REF) return I->cur_layer;
  Pv *d = res(I->P, ref);
  uint32_t num = ref->ref;
  if (name_is(dict_get(d, "Type"), "OCMD")) {  // membership: its first OCG
    Pv *o = dict_get(d, "OCGs");
    Pv *ro = res(I->P, o);
    if (o && o->t == PV_REF) num = o->ref;
    else if (ro->t == PV_ARR && ro->len && ro->a[0]->t == PV_REF) num = ro->a[0]->ref;
    else return I->cur_layer;
  }
  for (size_t k = 0; k < I->n_ocg; k++)
    if (I->ocg_num[k] == num) return I->ocg_h[k];
  return I->cur_layer;
}

static uint64_t tables_new_layer(Tables *t, const char *name, uint32_t flags) {
  if (t->n_layers == t->cap_layers) {
    t->cap_layers = t->cap_layers ? t->cap_layers * 2 : 64;
    t->layers = (LayerInfo *)realloc(t->layers, t->cap_layers * sizeof(LayerInfo));
  }
  LayerInfo *L = &t->layers[t->n_layers];
  L->h = t->n_layers + 1;
  L->ltype = LT_CONT;
  L->name = strdup(name && *name ? name : "?");
  L->rgb = 0xffffff;
  L->flags = flags;
  t->n_layers++;
  return L->h;
}

// PDF text (PDFDocEncoding or UTF-16BE with BOM) as UTF-8.
static void pdf_text_utf8(const Pv *s, char *out, size_t cap) {
  size_t o = 0;
  if (!s || s->t != PV_STR) { out[0] = 0; return; }
  const uint8_t *p = (const uint8_t *)s->s;
  if (s->len >= 2 && p[0] == 0xFE && p[1] == 0xFF) {
    for (size_t i = 2; i + 1 < s->len; i += 2) {
      unsigned cp = (unsigned)p[i] << 8 | p[i + 1];
      if (cp >= 0xD800 && cp < 0xDC00 && i + 3 < s->len) {
        unsigned lo = (unsigned)p[i + 2] << 8 | p[i + 3];
        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00), i += 2;
      }
      char u[4];
      int k = cp < 0x80 ? (u[0] = (char)cp, 1) : cp < 0x800 ? (u[0] = (char)(0xC0 | cp >> 6), u[1] = (char)(0x80 | (cp & 63)), 2)
              : cp < 0x10000 ? (u[0] = (char)(0xE0 | cp >> 12), u[1] = (char)(0x80 | (cp >> 6 & 63)), u[2] = (char)(0x80 | (cp & 63)), 3)
              : (u[0] = (char)(0xF0 | cp >> 18), u[1] = (char)(0x80 | (cp >> 12 & 63)), u[2] = (char)(0x80 | (cp >> 6 & 63)), u[3] = (char)(0x80 | (cp & 63)), 4);
      if (o + k + 1 > cap) break;
      memcpy(out + o, u, k), o += k;
    }
  } else {
    for (size_t i = 0; i < s->len; i++) {
      unsigned c = p[i];
      if (c < 0x80) { if (o + 2 > cap) break; out[o++] = (char)c; }
      else { if (o + 3 > cap) break; out[o++] = (char)(0xC0 | c >> 6), out[o++] = (char)(0x80 | (c & 63)); }
    }
  }
  out[o] = 0;
}

static void load_layers(Interp *I, Pv *catalog) {
  Pv *ocp = dget(I->P, catalog, "OCProperties");
  Pv *ocgs = dget(I->P, ocp, "OCGs");
  Pv *off = dget(I->P, dget(I->P, ocp, "D"), "OFF");
  if (ocgs->t != PV_ARR) return;
  size_t n = ocgs->len > 60000 ? 60000 : ocgs->len;
  I->ocg_num = (uint32_t *)calloc(n ? n : 1, sizeof(uint32_t));
  I->ocg_h = (uint64_t *)calloc(n ? n : 1, sizeof(uint64_t));
  for (size_t k = 0; k < n; k++) {
    Pv *r = ocgs->a[k];
    if (r->t != PV_REF) continue;
    int dup = 0;
    for (size_t j = 0; j < I->n_ocg; j++) dup |= I->ocg_num[j] == r->ref;
    if (dup) continue;
    char nm[512];
    pdf_text_utf8(dget(I->P, res(I->P, r), "Name"), nm, sizeof nm);
    uint32_t flags = 0;
    if (off->t == PV_ARR)
      for (size_t j = 0; j < off->len; j++)
        if (off->a[j]->t == PV_REF && off->a[j]->ref == r->ref) flags = UMG_LAYER_OFF;
    I->ocg_num[I->n_ocg] = r->ref;
    I->ocg_h[I->n_ocg++] = tables_new_layer(I->tab, nm, flags);
  }
}

// ---- fonts and text ------------------------------------------------------------------

static uint32_t hexcode(const char *s, size_t n) {
  uint32_t v = 0;
  for (size_t i = 0; i < n && i < 8; i++) v = v << 8 | (uint8_t)s[i];
  return v;
}

// UTF-16BE bytes (ToUnicode destination) -> first code point.
static uint32_t utf16_first(const char *s, size_t n) {
  if (n < 2) return n ? (uint8_t)s[0] : 0;
  uint32_t hi = (uint8_t)s[0] << 8 | (uint8_t)s[1];
  if (hi >= 0xD800 && hi < 0xDC00 && n >= 4) {
    uint32_t lo = (uint8_t)s[2] << 8 | (uint8_t)s[3];
    return 0x10000 + ((hi - 0xD800) << 10) + (lo - 0xDC00);
  }
  return hi;
}

static void map_set(FontInfo *f, uint32_t code, uint32_t cp) {
  if (code >= 0x10000) return;
  if (code >= f->nmap) {
    uint32_t nn = f->two_byte ? 0x10000 : 256;
    if (code >= nn) return;
    f->map = (uint32_t *)realloc(f->map, nn * sizeof(uint32_t));
    memset(f->map + f->nmap, 0, (nn - f->nmap) * sizeof(uint32_t));
    f->nmap = nn;
  }
  f->map[code] = cp;
}

static void parse_tounicode(Interp *I, FontInfo *f, Pv *st) {
  size_t len;
  uint8_t *d = stream_data(I->P, st, &len);
  if (!d) return;
  Tok t;
  size_t i = 0;
  Tok q[3];
  int nq = 0;
  int mode = 0;  // 1 bfchar, 2 bfrange
  long guard = 0;
  while (i < len && guard++ < 5000000) {
    i = next_tok(d, len, i, &t);
    if (t.t == TK_END) break;
    if (t.t == TK_OP) {
      if (t.n == 11 && !memcmp(t.s, "beginbfchar", 11)) mode = 1, nq = 0;
      else if (t.n == 12 && !memcmp(t.s, "beginbfrange", 12)) mode = 2, nq = 0;
      else if (t.n >= 9 && !memcmp(t.s, "end", 3)) mode = 0;
      continue;
    }
    if (!mode) continue;
    if (t.t == TK_ARR_B) {  // bfrange with an array of destinations
      if (mode == 2 && nq == 2) {
        char a[16], b[16];
        size_t la, lb;
        lex_hex(q[0].s, q[0].n, 0, a, &la);
        lex_hex(q[1].s, q[1].n, 0, b, &lb);
        if (la > 8) la = 8;
        if (lb > 8) lb = 8;
        uint32_t lo = hexcode(a, la), hi = hexcode(b, lb);
        for (uint32_t c = lo; c <= hi && c - lo < 65536; c++) {
          i = next_tok(d, len, i, &t);
          if (t.t != TK_STR) break;
          char u[64];
          size_t lu;
          if (t.n > 2 * sizeof u + 2) continue;
          lex_hex(t.s, t.n, 0, u, &lu);
          map_set(f, c, utf16_first(u, lu));
        }
        while (i < len && t.t != TK_ARR_E && t.t != TK_END) i = next_tok(d, len, i, &t);
      }
      nq = 0;
      continue;
    }
    if (t.t != TK_STR || t.s[0] != '<' || t.n > 160) { nq = 0; continue; }
    q[nq++] = t;
    if (mode == 1 && nq == 2) {
      char a[16], u[80];
      size_t la, lu;
      if (q[0].n > 2 * sizeof a + 2) { nq = 0; continue; }
      lex_hex(q[0].s, q[0].n, 0, a, &la);
      lex_hex(q[1].s, q[1].n, 0, u, &lu);
      map_set(f, hexcode(a, la), utf16_first(u, lu));
      nq = 0;
    } else if (mode == 2 && nq == 3) {
      char a[16], b[16], u[80];
      size_t la, lb, lu;
      if (q[0].n > 2 * sizeof a + 2 || q[1].n > 2 * sizeof b + 2) { nq = 0; continue; }
      lex_hex(q[0].s, q[0].n, 0, a, &la);
      lex_hex(q[1].s, q[1].n, 0, b, &lb);
      lex_hex(q[2].s, q[2].n, 0, u, &lu);
      uint32_t lo = hexcode(a, la), hi = hexcode(b, lb), cp = utf16_first(u, lu);
      for (uint32_t c = lo; c <= hi && c - lo < 65536; c++) map_set(f, c, cp + (c - lo));
      nq = 0;
    }
  }
  free(d);
}

// Embedded TrueType font program: glyph id -> code point from its 'cmap'
// (format 4 or 12, Unicode or Windows symbol subtable). For Type0 fonts
// with Identity encoding and no ToUnicode (AutoCAD writes these): the codes
// are CIDs = glyph ids when CIDToGIDMap is Identity.
static uint32_t be16(const uint8_t *p) { return (uint32_t)p[0] << 8 | p[1]; }
static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }

static void tt_cmap_reverse(FontInfo *f, const uint8_t *d, size_t n, const uint16_t *cid2gid, size_t ncid) {
  if (n < 12) return;
  uint32_t nt = be16(d + 4);
  const uint8_t *cmap = NULL;
  size_t clen = 0;
  for (uint32_t k = 0; k < nt && 12 + 16 * (k + 1) <= n; k++) {
    const uint8_t *r = d + 12 + 16 * k;
    if (!memcmp(r, "cmap", 4)) {
      uint32_t off = be32(r + 8), len = be32(r + 12);
      if (off < n && len <= n - off) cmap = d + off, clen = len;
    }
  }
  if (!cmap || clen < 4) return;
  // best subtable: (3,10) / (0,4) format 12, else (3,1) / (0,3) / (3,0) format 4
  const uint8_t *best = NULL;
  int best_rank = 0;
  uint32_t ns = be16(cmap + 2);
  for (uint32_t k = 0; k < ns && 4 + 8 * (k + 1) <= clen; k++) {
    uint32_t pid = be16(cmap + 4 + 8 * k), eid = be16(cmap + 6 + 8 * k), off = be32(cmap + 8 + 8 * k);
    if (off + 4 > clen) continue;
    uint32_t fmt = be16(cmap + off);
    int rank = fmt == 12 && (pid == 0 || (pid == 3 && eid == 10)) ? 4 : fmt == 4 && pid == 3 && eid == 1 ? 3
             : fmt == 4 && pid == 0 ? 2 : fmt == 4 && pid == 3 && eid == 0 ? 1 : 0;
    if (rank > best_rank) best_rank = rank, best = cmap + off;
  }
  if (!best) return;
  size_t avail = clen - (size_t)(best - cmap);
  // gid -> code point, then code (CID) -> code point through cid2gid
  uint32_t *g2u = (uint32_t *)calloc(65536, sizeof(uint32_t));
  if (!g2u) return;
  int symbol = best_rank == 1;
  if (be16(best) == 4 && avail >= 14) {
    uint32_t segx2 = be16(best + 6);
    if (14 + 4 * (size_t)segx2 + 2 <= avail) {
      const uint8_t *ends = best + 14, *starts = ends + segx2 + 2, *deltas = starts + segx2, *ranges = deltas + segx2;
      for (uint32_t s2 = 0; s2 < segx2; s2 += 2) {
        uint32_t st = be16(starts + s2), en = be16(ends + s2), dl = be16(deltas + s2), ro = be16(ranges + s2);
        for (uint32_t c = st; c <= en && c != 0xFFFF; c++) {
          uint32_t gid;
          if (!ro) gid = (c + dl) & 0xFFFF;
          else {
            size_t at = (size_t)(ranges + s2 - best) + ro + 2 * (c - st);
            if (at + 2 > avail) break;
            gid = be16(best + at);
            if (gid) gid = (gid + dl) & 0xFFFF;
          }
          uint32_t cp = symbol && c >= 0xF020 && c <= 0xF0FF ? c - 0xF000 : c;
          if (gid && !g2u[gid]) g2u[gid] = cp;
        }
      }
    }
  } else if (be16(best) == 12 && avail >= 16) {
    uint32_t ng = be32(best + 12);
    for (uint32_t k = 0; k < ng && 16 + 12 * (size_t)(k + 1) <= avail; k++) {
      const uint8_t *gp = best + 16 + 12 * k;
      uint32_t sc = be32(gp), ec = be32(gp + 4), sg = be32(gp + 8);
      for (uint32_t c = sc; c <= ec && c - sc < 65536; c++)
        if (sg + (c - sc) < 65536 && !g2u[sg + (c - sc)]) g2u[sg + (c - sc)] = c;
    }
  }
  for (uint32_t code = 0; code < 65536; code++) {
    if (code < f->nmap && f->map[code]) continue;
    uint32_t gid = cid2gid ? (code < ncid ? cid2gid[code] : 0) : code;
    if (gid && g2u[gid]) map_set(f, code, g2u[gid]);
  }
  free(g2u);
}

// TrueType outlines (glyf / loca / head): tables found and kept in f.
static int tt_open(FontInfo *f, uint8_t *d, size_t n) {
  if (n < 12) return 0;
  uint32_t nt = be16(d + 4), head = 0, maxp = 0;
  for (uint32_t k = 0; k < nt && 12 + 16 * (size_t)(k + 1) <= n; k++) {
    const uint8_t *r = d + 12 + 16 * k;
    uint32_t off = be32(r + 8), len = be32(r + 12);
    if (off >= n || len > n - off) continue;
    if (!memcmp(r, "glyf", 4)) f->glyf_off = off, f->glyf_len = len;
    else if (!memcmp(r, "loca", 4)) f->loca_off = off, f->loca_len = len;
    else if (!memcmp(r, "head", 4) && len >= 54) head = off;
    else if (!memcmp(r, "maxp", 4) && len >= 6) maxp = off;
  }
  if (!head || !f->glyf_len || !f->loca_len) return 0;
  f->upm = be16(d + head + 18);
  if (f->upm < 16 || f->upm > 16384) f->upm = 1000;
  f->loca_long = be16(d + head + 50) != 0;
  f->nglyphs = maxp ? be16(d + maxp + 4) : 0;
  f->tt = d, f->ttn = n;
  return 1;
}

static int tt_range(const FontInfo *f, uint32_t gid, uint32_t *a, uint32_t *b) {
  const uint8_t *loca = f->tt + f->loca_off;
  size_t need = (size_t)(gid + 2) * (f->loca_long ? 4 : 2);
  if (need > f->loca_len) return 0;
  *a = f->loca_long ? be32(loca + 4 * gid) : 2 * be16(loca + 2 * gid);
  *b = f->loca_long ? be32(loca + 4 * gid + 4) : 2 * be16(loca + 2 * gid + 2);
  return *a < *b && *b <= f->glyf_len;
}

static void tt_pt(Loops *L, M6 m, double x, double y) {
  loops_pt(L, m.a * x + m.c * y + m.e, m.b * x + m.d * y + m.f);
}

// Contours of glyph gid (font units through m) into L.
static void tt_glyph(const FontInfo *f, uint32_t gid, M6 m, Loops *L, int depth) {
  uint32_t a, b;
  if (depth > 4 || !tt_range(f, gid, &a, &b) || b - a < 10) return;
  const uint8_t *g = f->tt + f->glyf_off + a, *end = f->tt + f->glyf_off + b;
  int nc = (int16_t)be16(g);
  const uint8_t *p = g + 10;
  if (nc >= 0) {
    if (p + 2 * nc + 2 > end) return;
    const uint8_t *eps = p;
    uint32_t npts = nc ? be16(eps + 2 * (nc - 1)) + 1 : 0;
    if (npts > 20000) return;
    p += 2 * nc;
    uint32_t il = be16(p);
    p += 2 + il;
    if (p > end) return;
    uint8_t *fl = (uint8_t *)malloc(npts ? npts : 1);
    double *xy = (double *)malloc((npts ? npts : 1) * 2 * sizeof(double));
    uint32_t k = 0;
    while (k < npts && p < end) {
      uint8_t c = *p++;
      fl[k++] = c;
      if (c & 8 && p < end) for (uint32_t r = *p++; r && k < npts; r--) fl[k++] = c;
    }
    int ok = k == npts;
    long x = 0, y = 0;
    for (k = 0; ok && k < npts; k++) {
      if (fl[k] & 2) { if (p >= end) { ok = 0; break; } x += (fl[k] & 16) ? *p : -(long)*p; p++; }
      else if (!(fl[k] & 16)) { if (p + 2 > end) { ok = 0; break; } x += (int16_t)be16(p); p += 2; }
      xy[2 * k] = (double)x;
    }
    for (k = 0; ok && k < npts; k++) {
      if (fl[k] & 4) { if (p >= end) { ok = 0; break; } y += (fl[k] & 32) ? *p : -(long)*p; p++; }
      else if (!(fl[k] & 32)) { if (p + 2 > end) { ok = 0; break; } y += (int16_t)be16(p); p += 2; }
      xy[2 * k + 1] = (double)y;
    }
    uint32_t s0 = 0;
    for (int c = 0; ok && c < nc; c++) {
      uint32_t e = be16(eps + 2 * c);
      if (e < s0 || e >= npts) break;
      uint32_t cnt = e - s0 + 1;
      if (cnt >= 3) {
        *(size_t *)vec_push(&L->start) = L->xy.n;
        // start at an on-curve point (or the midpoint of the first two)
        uint32_t st = 0;
        while (st < cnt && !(fl[s0 + st] & 1)) st++;
        double sx, sy;
        if (st == cnt) sx = (xy[2 * s0] + xy[2 * (s0 + 1)]) / 2, sy = (xy[2 * s0 + 1] + xy[2 * (s0 + 1) + 1]) / 2, st = 0;
        else sx = xy[2 * (s0 + st)], sy = xy[2 * (s0 + st) + 1];
        tt_pt(L, m, sx, sy);
        double px = sx, py = sy;
        for (uint32_t j = 1; j <= cnt; j++) {
          uint32_t i = s0 + (st + j) % cnt;
          double qx = xy[2 * i], qy = xy[2 * i + 1];
          if (fl[i] & 1) {
            if (j < cnt || qx != sx || qy != sy) tt_pt(L, m, qx, qy);
            px = qx, py = qy;
          } else {
            uint32_t i2 = s0 + (st + j + 1) % cnt;
            double nx = xy[2 * i2], ny = xy[2 * i2 + 1];
            if (!(fl[i2] & 1)) nx = (qx + nx) / 2, ny = (qy + ny) / 2;
            for (int t = 1; t <= 4; t++) {
              double u = t / 4.0, v = 1 - u;
              tt_pt(L, m, v * v * px + 2 * v * u * qx + u * u * nx, v * v * py + 2 * v * u * qy + u * u * ny);
            }
            px = nx, py = ny;
            if (fl[i2] & 1) j++;
          }
        }
      }
      s0 = e + 1;
    }
    free(fl), free(xy);
  } else {  // composite
    for (int guard = 0; guard < 64 && p + 4 <= end; guard++) {
      uint32_t fl = be16(p), cg = be16(p + 2);
      p += 4;
      double dx = 0, dy = 0, a2 = 1, b2 = 0, c2 = 0, d2 = 1;
      if (fl & 1) { if (p + 4 > end) return; if (fl & 2) dx = (int16_t)be16(p), dy = (int16_t)be16(p + 2); p += 4; }
      else { if (p + 2 > end) return; if (fl & 2) dx = (int8_t)p[0], dy = (int8_t)p[1]; p += 2; }
      if (fl & 8) { if (p + 2 > end) return; a2 = d2 = (int16_t)be16(p) / 16384.0; p += 2; }
      else if (fl & 0x40) { if (p + 4 > end) return; a2 = (int16_t)be16(p) / 16384.0, d2 = (int16_t)be16(p + 2) / 16384.0; p += 4; }
      else if (fl & 0x80) { if (p + 8 > end) return; a2 = (int16_t)be16(p) / 16384.0, b2 = (int16_t)be16(p + 2) / 16384.0, c2 = (int16_t)be16(p + 4) / 16384.0, d2 = (int16_t)be16(p + 6) / 16384.0; p += 8; }
      M6 cm = {a2, b2, c2, d2, dx, dy};
      tt_glyph(f, cg, m6_mul(cm, m), L, depth + 1);
      if (!(fl & 0x20)) break;
    }
  }
}

// cp1252 0x80..0x9F
static const uint16_t WIN_HI[32] = {0x20AC, 0, 0x201A, 0x0192, 0x201E, 0x2026, 0x2020, 0x2021, 0x02C6, 0x2030, 0x0160, 0x2039, 0x0152, 0, 0x017D, 0,
                                    0, 0x2018, 0x2019, 0x201C, 0x201D, 0x2022, 0x2013, 0x2014, 0x02DC, 0x2122, 0x0161, 0x203A, 0x0153, 0, 0x017E, 0x0178};

static FontInfo *font_get(Interp *I, Pv *fdict) {
  if (!fdict || fdict->t != PV_DICT) return NULL;
  for (int k = 0; k < I->nfonts; k++)
    if (I->font_keys[k] == fdict) return &I->fonts[k];
  if (I->nfonts == 64) return &I->fonts[63];
  FontInfo *f = &I->fonts[I->nfonts];
  memset(f, 0, sizeof *f);
  I->font_keys[I->nfonts++] = fdict;
  f->dict = fdict;
  f->two_byte = name_is(dget(I->P, fdict, "Subtype"), "Type0");
  Pv *tu = dget(I->P, fdict, "ToUnicode");
  if (tu->t == PV_DICT && tu->has_stream) parse_tounicode(I, f, tu);
  if (f->two_byte && !f->nmap) {  // no ToUnicode: the embedded TrueType's cmap
    Pv *df = dget(I->P, fdict, "DescendantFonts");
    Pv *cf = df->t == PV_ARR && df->len ? res(I->P, df->a[0]) : &PV_NULL_V;
    Pv *ff = dget(I->P, dget(I->P, cf, "FontDescriptor"), "FontFile2");
    Pv *c2g = dget(I->P, cf, "CIDToGIDMap");
    uint16_t *cid2gid = NULL;
    size_t ncid = 0, glen;
    if (c2g->t == PV_DICT && c2g->has_stream) {
      uint8_t *m = stream_data(I->P, c2g, &glen);
      if (m) {
        ncid = glen / 2 > 65536 ? 65536 : glen / 2;
        cid2gid = (uint16_t *)malloc((ncid ? ncid : 1) * sizeof(uint16_t));
        for (size_t k = 0; k < ncid; k++) cid2gid[k] = (uint16_t)be16(m + 2 * k);
        free(m);
      }
    }
    size_t tl;
    uint8_t *tt = ff->t == PV_DICT && ff->has_stream ? stream_data(I->P, ff, &tl) : NULL;
    if (tt) tt_cmap_reverse(f, tt, tl, cid2gid, ncid), free(tt);
    free(cid2gid);
  }
  if (f->two_byte) {
    Pv *df = dget(I->P, fdict, "DescendantFonts");
    Pv *cf = df->t == PV_ARR && df->len ? res(I->P, df->a[0]) : &PV_NULL_V;
    f->dw = pv_num(dget(I->P, cf, "DW"), 1000);
    Pv *W = dget(I->P, cf, "W");
    if (W->t == PV_ARR && W->len) {
      f->cw = (uint16_t *)calloc(65536, sizeof(uint16_t));
      for (size_t k = 0; f->cw && k + 1 < W->len;) {
        double c0 = pv_num(res(I->P, W->a[k]), -1);
        Pv *nx = res(I->P, W->a[k + 1]);
        if (nx->t == PV_ARR) {
          for (size_t j = 0; j < nx->len && c0 >= 0 && c0 + j < 65536; j++)
            f->cw[(size_t)c0 + j] = (uint16_t)fmin(fmax(pv_num(res(I->P, nx->a[j]), 0), 0), 65535);
          k += 2;
        } else if (k + 2 < W->len) {
          double c1 = pv_num(nx, -1), w = pv_num(res(I->P, W->a[k + 2]), 0);
          for (double c = c0; c >= 0 && c <= c1 && c < 65536 && c - c0 < 65536; c++)
            f->cw[(size_t)c] = (uint16_t)fmin(fmax(w, 0), 65535);
          k += 3;
        } else break;
      }
    }
    // outlines, for codes no character could be found for
    Pv *ff = dget(I->P, dget(I->P, cf, "FontDescriptor"), "FontFile2");
    size_t tl;
    uint8_t *tt = ff->t == PV_DICT && ff->has_stream ? stream_data(I->P, ff, &tl) : NULL;
    if (tt && !tt_open(f, tt, tl)) free(tt);
    Pv *c2g = dget(I->P, cf, "CIDToGIDMap");
    if (f->tt && c2g->t == PV_DICT && c2g->has_stream) {
      size_t glen;
      uint8_t *m = stream_data(I->P, c2g, &glen);
      if (m) {
        f->ncid = glen / 2 > 65536 ? 65536 : glen / 2;
        f->cid2gid = (uint16_t *)malloc((f->ncid ? f->ncid : 1) * sizeof(uint16_t));
        for (size_t k = 0; k < f->ncid; k++) f->cid2gid[k] = (uint16_t)be16(m + 2 * k);
        free(m);
      }
    }
  }
  if (!f->two_byte) {
    Pv *w = dget(I->P, fdict, "Widths");
    int fc = (int)pv_num(dget(I->P, fdict, "FirstChar"), 0);
    if (w->t == PV_ARR && fc >= 0 && fc < 256)
      for (size_t k = 0; k < w->len && fc + k < 256; k++) f->w[fc + k] = pv_num(res(I->P, w->a[k]), 0), f->has_w = 1;
    // simple encoding without ToUnicode: cp1252, /Differences by glyph name for a few
    for (int c = 0; c < 256; c++) {
      if (f->nmap && f->map[c]) continue;
      uint32_t cp = c < 0x80 ? (uint32_t)c : c < 0xA0 ? WIN_HI[c - 0x80] : (uint32_t)c;
      if (c < 32) cp = 0;
      if (cp) map_set(f, (uint32_t)c, cp);
    }
  }
  return f;
}

static size_t put_utf8(char *o, size_t at, size_t cap, uint32_t cp) {
  if (cp < 0x80) { if (at + 1 < cap) o[at++] = (char)cp; }
  else if (cp < 0x800) { if (at + 2 < cap) o[at++] = (char)(0xC0 | cp >> 6), o[at++] = (char)(0x80 | (cp & 63)); }
  else if (cp < 0x10000) { if (at + 3 < cap) o[at++] = (char)(0xE0 | cp >> 12), o[at++] = (char)(0x80 | (cp >> 6 & 63)), o[at++] = (char)(0x80 | (cp & 63)); }
  else if (cp < 0x110000) { if (at + 4 < cap) o[at++] = (char)(0xF0 | cp >> 18), o[at++] = (char)(0x80 | (cp >> 12 & 63)), o[at++] = (char)(0x80 | (cp >> 6 & 63)), o[at++] = (char)(0x80 | (cp & 63)); }
  return at;
}

// Shows decoded bytes; appends UTF-8 to out and advances the text matrix.
static size_t show_bytes(Interp *I, const char *s, size_t n, char *out, size_t at, size_t cap, int *drew) {
  GState *G = &I->gs[I->gn];
  FontInfo *f = font_get(I, G->font);
  int two = f && f->two_byte;
  for (size_t i = 0; i < n;) {
    uint32_t code = two && i + 1 < n ? (uint8_t)s[i] << 8 | (uint8_t)s[i + 1] : (uint8_t)s[i];
    i += two && i + 1 < n ? 2 : 1;
    uint32_t cp = f && code < f->nmap ? f->map[code] : 0;
    if (!f) cp = code < 0x80 && code >= 32 ? code : 0;
#ifdef PDF_FORCE_GLYPHS  // test build: draw every glyph with an outline
    if (f && f->tt) cp = 0;
#endif
    if (cp && cp != 0xFFFD) at = put_utf8(out, at, cap, cp);
    else if (f && f->tt && G->tr != 3 && G->tr != 7 && I->n_glyphs < 2000000) {
      // no character for it: draw the glyph's outline (not searchable)
      uint32_t gid = f->cid2gid ? (code < f->ncid ? f->cid2gid[code] : 0) : code;
      if (gid && (!f->nglyphs || gid < f->nglyphs)) {
        M6 gm = {G->tfs * G->th / f->upm, 0, 0, G->tfs / f->upm, 0, G->trise};
        gm = m6_mul(m6_mul(gm, I->tm), G->ctm);
        Loops L;
        memset(&L, 0, sizeof L);
        L.xy.elem = 2 * sizeof(double), L.start.elem = sizeof(size_t);
        tt_glyph(f, gid, gm, &L, 0);
        if (L.start.n) {
          set_style(I, ink_color(G->frgb));
          I->g->ord.root = ++I->ord;
          fill_loops(I->g, I->out, &L, 0);
          I->n_glyphs++;
          *drew = 1;
        }
        free(L.xy.p), free(L.start.p);
      }
    }
    double w = !two && f && f->has_w && f->w[code & 255] > 0 ? f->w[code & 255] / 1000
             : two && f ? ((f->cw && f->cw[code & 0xFFFF] ? f->cw[code & 0xFFFF] : f->dw) / 1000) : 0.55;
    double adv = (w * G->tfs + G->tc + (!two && code == 32 ? G->tw : 0)) * G->th;
    M6 t = {1, 0, 0, 1, adv, 0};
    I->tm = m6_mul(t, I->tm);
  }
  return at;
}

static void emit_text(Interp *I, M6 start, const char *u, size_t len) {
  GState *G = &I->gs[I->gn];
  if (!len || G->tr == 3 || G->tr == 7) return;  // invisible (OCR layer of a scan)
  while (len && u[len - 1] == ' ') len--;
  size_t lead = 0;
  while (lead < len && u[lead] == ' ') lead++;
  if (lead == len) return;
  M6 trm0 = {G->tfs * G->th, 0, 0, G->tfs, 0, G->trise};
  M6 m = m6_mul(m6_mul(trm0, start), G->ctm);
  double x = m.e, y = m.f;
  double hx = m.c, hy = m.d, wx = m.a, wy = m.b;
  double h = hypot(hx, hy), w = hypot(wx, wy);
  if (!(h > 1e-6) || !isfinite(h)) return;
  // page -> output (rotation + scale)
  Xf o = I->out;
  double X = o.a * x + o.b * y + o.tx, Y = o.c * x + o.d * y + o.ty;
  double ax = o.a * wx + o.b * wy, ay = o.c * wx + o.d * wy;
  UmgText r;
  memset(&r, 0, sizeof r);
  r.h = (float)(h * I->scale);
  r.rot = (float)atan2(ay, ax);
  r.wf = (float)(G->th > 0 && h > 0 ? w / h : 1);
  if (!(r.wf > 0.05f && r.wf < 20)) r.wf = 1;
  r.kind = 0;
  set_style(I, ink_color(G->frgb));
  push_text(I->g, r, X, Y, X, Y, u + lead, (uint32_t)(len - lead));
  I->n_text++;
}

// ---- interpreter -----------------------------------------------------------------------

static void run_content(Interp *I, const uint8_t *b, size_t n, Pv *resources);

static void do_xobject(Interp *I, Pv *resources, const Tok *name) {
  Pv *xo = dget(I->P, resources, "XObject");
  char nm[256];
  size_t l = name->n < 255 ? name->n : 255;
  memcpy(nm, name->s, l), nm[l] = 0;
  Pv *x = dget(I->P, xo, nm);
  if (x->t != PV_DICT) return;
  Pv *st = dget(I->P, x, "Subtype");
  if (name_is(st, "Image")) { I->n_images++; return; }
  if (!name_is(st, "Form") || !x->has_stream || I->form_depth >= PDF_MAX_FORM_DEPTH || I->gn >= 63) return;
  size_t len;
  uint8_t *d = stream_data(I->P, x, &len);
  if (!d) return;
  I->gs[I->gn + 1] = I->gs[I->gn];
  I->gn++;
  Pv *mx = dget(I->P, x, "Matrix");
  if (mx->t == PV_ARR && mx->len >= 6) {
    M6 m = {pv_num(res(I->P, mx->a[0]), 1), pv_num(res(I->P, mx->a[1]), 0), pv_num(res(I->P, mx->a[2]), 0),
            pv_num(res(I->P, mx->a[3]), 1), pv_num(res(I->P, mx->a[4]), 0), pv_num(res(I->P, mx->a[5]), 0)};
    I->gs[I->gn].ctm = m6_mul(m, I->gs[I->gn].ctm);
  }
  uint64_t saved = I->cur_layer;
  Pv *oc = dict_get(x, "OC");
  if (oc) I->cur_layer = layer_for(I, oc);
  Pv *r = dget(I->P, x, "Resources");
  I->form_depth++;
  M6 tm = I->tm, tlm = I->tlm;
  run_content(I, d, len, r->t == PV_DICT ? r : resources);
  I->tm = tm, I->tlm = tlm;
  I->form_depth--;
  I->cur_layer = saved;
  I->gn--;
  free(d);
}

static void run_content(Interp *I, const uint8_t *b, size_t n, Pv *resources) {
  Tok ops[PDF_MAX_OPERANDS];
  int no = 0;
  size_t i = 0;
  Tok t;
  path_reset(I);
  while (i < n) {
    i = next_tok(b, n, i, &t);
    if (t.t == TK_END) break;
    if (t.t != TK_OP) {
      if (no < PDF_MAX_OPERANDS) ops[no++] = t;
      continue;
    }
    if (++I->ops > PDF_MAX_OPS) return;
    const char *o = (const char *)t.s;
    size_t on = t.n;
#define OP(s) (on == sizeof(s) - 1 && !memcmp(o, s, sizeof(s) - 1))
#define NUM(k) (no >= (k) && ops[no - (k)].t == TK_NUM ? ops[no - (k)].v : 0)
    GState *G = &I->gs[I->gn];
    if (OP("m") && no >= 2) path_move(I, NUM(2), NUM(1));
    else if (OP("l") && no >= 2) path_line(I, NUM(2), NUM(1));
    else if (OP("c") && no >= 6) path_curve(I, NUM(6), NUM(5), NUM(4), NUM(3), NUM(2), NUM(1));
    else if (OP("v") && no >= 4) path_curve(I, I->cx, I->cy, NUM(4), NUM(3), NUM(2), NUM(1));
    else if (OP("y") && no >= 4) path_curve(I, NUM(4), NUM(3), NUM(2), NUM(1), NUM(2), NUM(1));
    else if (OP("h")) path_close(I);
    else if (OP("re") && no >= 4) {
      double x = NUM(4), y = NUM(3), w = NUM(2), h = NUM(1);
      path_move(I, x, y), path_line(I, x + w, y), path_line(I, x + w, y + h), path_line(I, x, y + h), path_close(I);
    }
    else if (OP("S")) paint(I, 1, 0);
    else if (OP("s")) path_close(I), paint(I, 1, 0);
    else if (OP("f") || OP("F") || OP("f*")) paint(I, 0, 1);
    else if (OP("B") || OP("B*")) paint(I, 1, 1);
    else if (OP("b") || OP("b*")) path_close(I), paint(I, 1, 1);
    else if (OP("n")) path_reset(I);
    else if (OP("W") || OP("W*")) {}  // clipping: not applied
    else if (OP("q")) { if (I->gn < 63) I->gs[I->gn + 1] = *G, I->gn++; }
    else if (OP("Q")) { if (I->gn > 0) I->gn--; }
    else if (OP("cm") && no >= 6) {
      M6 m = {NUM(6), NUM(5), NUM(4), NUM(3), NUM(2), NUM(1)};
      G->ctm = m6_mul(m, G->ctm);
    }
    else if (OP("w") && no >= 1) G->lw = NUM(1);
    else if (OP("G") && no >= 1) G->srgb = rgb3(NUM(1), NUM(1), NUM(1)), G->stroke_n = 1;
    else if (OP("g") && no >= 1) G->frgb = rgb3(NUM(1), NUM(1), NUM(1)), G->fill_n = 1;
    else if (OP("RG") && no >= 3) G->srgb = rgb3(NUM(3), NUM(2), NUM(1)), G->stroke_n = 3;
    else if (OP("rg") && no >= 3) G->frgb = rgb3(NUM(3), NUM(2), NUM(1)), G->fill_n = 3;
    else if (OP("K") && no >= 4) { double v[4] = {NUM(4), NUM(3), NUM(2), NUM(1)}; G->srgb = comps_rgb(v, 4), G->stroke_n = 4; }
    else if (OP("k") && no >= 4) { double v[4] = {NUM(4), NUM(3), NUM(2), NUM(1)}; G->frgb = comps_rgb(v, 4), G->fill_n = 4; }
    else if (OP("CS") || OP("cs")) {
      int nc = 1;
      if (no >= 1 && ops[no - 1].t == TK_NAME) {
        if (ops[no - 1].n == 9 && !memcmp(ops[no - 1].s, "DeviceRGB", 9)) nc = 3;
        else if (ops[no - 1].n == 10 && !memcmp(ops[no - 1].s, "DeviceCMYK", 10)) nc = 4;
        else if (!(ops[no - 1].n == 10 && !memcmp(ops[no - 1].s, "DeviceGray", 10))) {
          // named colour space: ICCBased /N, else assume RGB
          char nm[128];
          size_t l = ops[no - 1].n < 127 ? ops[no - 1].n : 127;
          memcpy(nm, ops[no - 1].s, l), nm[l] = 0;
          Pv *cs = dget(I->P, dget(I->P, resources, "ColorSpace"), nm);
          nc = 3;
          if (cs->t == PV_ARR && cs->len >= 2 && name_is(res(I->P, cs->a[0]), "ICCBased"))
            nc = (int)pv_num(dget(I->P, res(I->P, cs->a[1]), "N"), 3);
          else if (cs->t == PV_ARR && cs->len >= 1 && name_is(res(I->P, cs->a[0]), "Separation")) nc = 1;
        }
      }
      if (o[0] == 'C') G->stroke_n = nc, G->srgb = 0;  // initial colour: black
      else G->fill_n = nc, G->frgb = 0;
    }
    else if (OP("SC") || OP("SCN") || OP("sc") || OP("scn")) {
      double v[4];
      int k = 0;
      for (int j = no - 1; j >= 0 && k < 4 && ops[j].t == TK_NUM; j--) k++;
      int want = o[0] == 'S' ? G->stroke_n : G->fill_n;
      if (want < 1 || want > 4) want = k;
      if (k >= want && want >= 1) {
        for (int j = 0; j < want; j++) v[j] = ops[no - want + j].v;
        uint32_t c = comps_rgb(v, want == 2 ? 1 : want);
        if (o[0] == 'S') G->srgb = c; else G->frgb = c;
      }
    }
    // marked content (optional content = layers)
    else if (OP("BDC")) {
      uint64_t L = I->cur_layer;
      if (no >= 2 && ops[no - 2].t == TK_NAME && ops[no - 2].n == 2 && !memcmp(ops[no - 2].s, "OC", 2) && ops[no - 1].t == TK_NAME) {
        char nm[256];
        size_t l = ops[no - 1].n < 255 ? ops[no - 1].n : 255;
        memcpy(nm, ops[no - 1].s, l), nm[l] = 0;
        Pv *props = dget(I->P, resources, "Properties");
        L = layer_for(I, dict_get(props, nm));
      }
      if (I->oc_n < 64) I->oc_stack[I->oc_n] = I->cur_layer;
      I->oc_n++;
      I->cur_layer = L;
    }
    else if (OP("BMC")) {
      if (I->oc_n < 64) I->oc_stack[I->oc_n] = I->cur_layer;
      I->oc_n++;
    }
    else if (OP("EMC")) {
      if (I->oc_n > 0) {
        I->oc_n--;
        if (I->oc_n < 64) I->cur_layer = I->oc_stack[I->oc_n];
      }
    }
    else if (OP("Do") && no >= 1 && ops[no - 1].t == TK_NAME) do_xobject(I, resources, &ops[no - 1]);
    else if (OP("BI")) {  // inline image: skip to "EI"
      I->n_images++;
      const uint8_t *e = NULL;
      for (size_t j = i; j + 2 < n; j++)
        if (b[j] == 'E' && b[j + 1] == 'I' && pdf_ws(b[j - 1]) && (j + 2 >= n || pdf_ws(b[j + 2]))) { e = b + j + 2; break; }
      i = e ? (size_t)(e - b) : n;
    }
    // text
    else if (OP("BT")) { M6 id = {1, 0, 0, 1, 0, 0}; I->tm = I->tlm = id; }
    else if (OP("ET")) {}
    else if (OP("Tf") && no >= 2) {
      G->tfs = NUM(1);
      if (ops[no - 2].t == TK_NAME) {
        char nm[256];
        size_t l = ops[no - 2].n < 255 ? ops[no - 2].n : 255;
        memcpy(nm, ops[no - 2].s, l), nm[l] = 0;
        Pv *fd = dget(I->P, dget(I->P, resources, "Font"), nm);
        G->font = fd->t == PV_DICT ? fd : NULL;
      }
    }
    else if (OP("Tc") && no >= 1) G->tc = NUM(1);
    else if (OP("Tw") && no >= 1) G->tw = NUM(1);
    else if (OP("Tz") && no >= 1) G->th = NUM(1) / 100;
    else if (OP("TL") && no >= 1) G->tl = NUM(1);
    else if (OP("Ts") && no >= 1) G->trise = NUM(1);
    else if (OP("Tr") && no >= 1) G->tr = (int)NUM(1);
    else if (OP("Td") && no >= 2) { M6 t2 = {1, 0, 0, 1, NUM(2), NUM(1)}; I->tlm = I->tm = m6_mul(t2, I->tlm); }
    else if (OP("TD") && no >= 2) { G->tl = -NUM(1); M6 t2 = {1, 0, 0, 1, NUM(2), NUM(1)}; I->tlm = I->tm = m6_mul(t2, I->tlm); }
    else if (OP("Tm") && no >= 6) { M6 m = {NUM(6), NUM(5), NUM(4), NUM(3), NUM(2), NUM(1)}; I->tlm = I->tm = m; }
    else if (OP("T*")) { M6 t2 = {1, 0, 0, 1, 0, -G->tl}; I->tlm = I->tm = m6_mul(t2, I->tlm); }
    else if (OP("Tj") || OP("'") || OP("\"") || OP("TJ")) {
      if (o[0] == '\'' || o[0] == '"') {
        if (o[0] == '"' && no >= 3) G->tw = NUM(3), G->tc = NUM(2);
        M6 t2 = {1, 0, 0, 1, 0, -G->tl};
        I->tlm = I->tm = m6_mul(t2, I->tlm);
      }
      char u[2048];
      size_t ul = 0;
      int drew = 0;
      M6 start = I->tm;
      int from = no - 1;
      if (OP("TJ")) {
        while (from >= 0 && ops[from].t != TK_ARR_B) from--;
        if (from < 0) from = no;
      }
      for (int j = OP("TJ") ? from + 1 : no - 1; j >= 0 && j < no; j++) {
        if (ops[j].t == TK_STR) {
          char raw[4096];
          size_t rl;
          if (ops[j].s[0] == '(') {
            size_t need;
            lex_lit(ops[j].s, ops[j].n, 0, NULL, &need);
            if (need > sizeof raw) continue;
            lex_lit(ops[j].s, ops[j].n, 0, raw, &rl);
          } else {
            if (ops[j].n / 2 > sizeof raw) continue;
            lex_hex(ops[j].s, ops[j].n, 0, raw, &rl);
          }
          ul = show_bytes(I, raw, rl, u, ul, sizeof u, &drew);
        } else if (ops[j].t == TK_NUM && OP("TJ")) {
          double adv = -ops[j].v / 1000 * G->tfs * G->th;
          M6 t2 = {1, 0, 0, 1, adv, 0};
          I->tm = m6_mul(t2, I->tm);
          if (ops[j].v < -250 && ul && ul + 1 < sizeof u && u[ul - 1] != ' ') u[ul++] = ' ';
        }
        if (!OP("TJ")) break;
      }
      if (!ul && !drew && no > 0 && G->tr != 3 && G->tr != 7) I->n_text_lost++;
      emit_text(I, start, u, ul);
    }
    no = 0;
#undef OP
#undef NUM
  }
  path_reset(I);
}

// ---- page -> drawing ----------------------------------------------------------------------

typedef struct {
  int pages, measured;
  double scale;     // drawing units per point
  double x0, y0, x1, y1;  // extents (drawing units)
  uint32_t units;
  unsigned long images, paths, small, texts, texts_lost, bad_streams, unsup_filters;
  int scan;  // a scanned page: pictures and next to no lines; only its frame is drawn
} PdfInfo;

static uint32_t units_from_label(const char *u) {
  while (*u == ' ') u++;
  if (!strcmp(u, "mm")) return 4;
  if (!strcmp(u, "cm")) return 5;
  if (!strcmp(u, "m")) return 6;
  if (!strcmp(u, "km")) return 7;
  if (!strcmp(u, "in") || !strcmp(u, "\"")) return 1;
  if (!strcmp(u, "ft") || !strcmp(u, "'")) return 2;
  return 0;
}

static int looks_pdf(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) return 0;
  char b[1024];
  size_t n = fread(b, 1, sizeof b, f);
  fclose(f);
  return memmem(b, n, "%PDF-", 5) != NULL;  // the header may follow junk
}

// Reads page `want` (0-based) of the PDF at path into g / t. Calls lock()
// once the file is in memory. Returns 0, or 3 when it is not a usable PDF.
static int pdf_read(const char *path, Geom *g, Tables *t, void (*lock)(void *), int want, PdfInfo *info) {
  memset(info, 0, sizeof *info);
  FILE *f = fopen(path, "rb");
  if (!f) return 3;
  fseek(f, 0, SEEK_END);
  long fl = ftell(f);
  fseek(f, 0, SEEK_SET);
  if (fl <= 0) { fclose(f); return 3; }
  uint8_t *buf = (uint8_t *)malloc((size_t)fl);
  if (!buf || fread(buf, 1, (size_t)fl, f) != (size_t)fl) { fclose(f); free(buf); return 3; }
  fclose(f);
  if (lock) lock(NULL);
  Pdf P;
  memset(&P, 0, sizeof P);
  P.b = buf, P.n = (size_t)fl;
  int rc = 3;
  Interp *I = (Interp *)calloc(1, sizeof(Interp));
  if (!pdf_open(&P)) goto done;
  {
    Pv *cat = dget(&P, P.trailer, "Root");
    PageRef pr;
    memset(&pr, 0, sizeof pr);
    long count = 0;
    find_page(&P, dget(&P, cat, "Pages"), 0, want, &count, &pr, &PV_NULL_V, &PV_NULL_V, 0);
    info->pages = (int)count;
    if (!pr.page) goto done;
    double bx0 = 0, by0 = 0, bx1 = 612, by1 = 792;
    if (pr.box && pr.box->t == PV_ARR && pr.box->len >= 4) {
      double v[4];
      for (int k = 0; k < 4; k++) v[k] = pv_num(res(&P, pr.box->a[k]), 0);
      bx0 = fmin(v[0], v[2]), by0 = fmin(v[1], v[3]), bx1 = fmax(v[0], v[2]), by1 = fmax(v[1], v[3]);
      if (!(bx1 - bx0 > 1 && by1 - by0 > 1 && bx1 - bx0 < 1e7 && by1 - by0 < 1e7)) bx0 = 0, by0 = 0, bx1 = 612, by1 = 792;
    }
    // scale: the largest measuring viewport (AutoCAD writes one per layout viewport)
    double scale = 25.4 / 72;  // no scale: millimetres on paper
    uint32_t units = 4;
    Pv *vp = dget(&P, pr.page, "VP");
    double best = 0;
    if (vp->t == PV_ARR)
      for (size_t k = 0; k < vp->len && k < 1000; k++) {
        Pv *v = res(&P, vp->a[k]);
        Pv *ms = dget(&P, v, "Measure");
        if (!name_is(dget(&P, ms, "Subtype"), "RL")) continue;
        Pv *X = dget(&P, ms, "X");
        if (X->t != PV_ARR || !X->len) continue;
        Pv *x0 = res(&P, X->a[0]);
        double c = pv_num(dget(&P, x0, "C"), 0);
        Pv *bb = dget(&P, v, "BBox");
        double area = 1;
        if (bb->t == PV_ARR && bb->len >= 4)
          area = fabs((pv_num(res(&P, bb->a[2]), 0) - pv_num(res(&P, bb->a[0]), 0)) * (pv_num(res(&P, bb->a[3]), 0) - pv_num(res(&P, bb->a[1]), 0)));
        if (c > 1e-9 && c < 1e9 && area > best) {
          best = area, scale = c, info->measured = 1;
          char ul[64];
          pdf_text_utf8(dget(&P, x0, "U"), ul, sizeof ul);
          units = units_from_label(ul);
        }
      }
    int rot = ((pr.rotate % 360) + 360) % 360;
    rot = rot / 90 * 90;
    // page point -> drawing: shown as the viewer shows the page (Rotate is clockwise)
    Xf R = rot == 90 ? (Xf){0, 1, -1, 0, 0, 0} : rot == 180 ? (Xf){-1, 0, 0, -1, 0, 0} : rot == 270 ? (Xf){0, -1, 1, 0, 0, 0} : XF_ID;
    Xf out = {R.a * scale, R.b * scale, R.c * scale, R.d * scale, 0, 0};
    // origin / q from the page box, like set_origin
    double cxp = (bx0 + bx1) / 2, cyp = (by0 + by1) / 2;
    double span = fmax(bx1 - bx0, by1 - by0) * scale;
    g->ox = out.a * cxp + out.b * cyp, g->oy = out.c * cxp + out.d * cyp;
    g->q = ldexp(1, (int)ceil(log2(span / 1073741824.0)));
    g->origin_set = 1;
    double cx[4] = {bx0, bx1, bx1, bx0}, cy[4] = {by0, by0, by1, by1};
    info->x0 = info->y0 = INFINITY, info->x1 = info->y1 = -INFINITY;
    for (int k = 0; k < 4; k++) {
      double X = out.a * cx[k] + out.b * cy[k], Y = out.c * cx[k] + out.d * cy[k];
      info->x0 = fmin(info->x0, X), info->x1 = fmax(info->x1, X);
      info->y0 = fmin(info->y0, Y), info->y1 = fmax(info->y1, Y);
    }
    info->scale = scale, info->units = units;
    t->units = units;
    t->ltscale = 1;
    t->layer0 = tables_new_layer(t, "0", 0);
    I->P = &P, I->g = g, I->tab = t, I->out = out, I->scale = scale, I->rot = rot;
    I->xy.elem = 2 * sizeof(double), I->sub.elem = sizeof(size_t), I->closed.elem = I->curved.elem = 1;
    I->cur_layer = t->layer0;
    M6 id = {1, 0, 0, 1, 0, 0};
    I->gs[0].ctm = id, I->gs[0].lw = 1, I->gs[0].th = 1, I->gs[0].fill_n = I->gs[0].stroke_n = 1;
    load_layers(I, cat);
    Pv *cont = dget(&P, pr.page, "Contents");
    Pv *one[1] = {dict_get(pr.page, "Contents")};
    Pv **parts = cont->t == PV_ARR ? cont->a : one;
    size_t np = cont->t == PV_ARR ? cont->len : (cont->t == PV_DICT ? 1 : 0);
    // the parts form one stream (an operator may span two parts)
    Vec all = {NULL, 0, 0, 1};
    for (size_t k = 0; k < np && k < 10000; k++) {
      size_t len;
      uint8_t *d = stream_data(&P, res(&P, parts[k]), &len);
      if (!d) continue;
      vec_append(&all, d, len);
      *(char *)vec_push(&all) = '\n';
      free(d);
    }
    if (all.n) run_content(I, (const uint8_t *)all.p, all.n, pr.resources);
    free(all.p);
    info->images = I->n_images, info->paths = I->n_paths, info->small = I->n_small, info->texts = I->n_text, info->texts_lost = I->n_text_lost;
    // A scanned page (pictures, next to no lines): the page's edge as a frame,
    // so there is something to open and the app can lay the page's picture
    // inside it (D111).
    if (I->n_images && g->lines.n + g->arcs.n + g->polys.n + g->fills.n + g->hlines.n < 20) {
      I->cur_layer = t->layer0;
      set_style(I, COL_TRUE | 0x808080);
      double fx[4] = {info->x0, info->x1, info->x1, info->x0}, fy[4] = {info->y0, info->y0, info->y1, info->y1};
      begin_poly(g, 4, UMG_CLOSED | UMG_LTCONT);
      for (int k = 0; k < 4; k++) add_pt(g, XF_ID, fx[k], fy[k], 0);
      info->scan = 1;
    }
    rc = 0;
  }
done:
  info->bad_streams = P.n_bad_streams, info->unsup_filters = P.n_filters_unsup;
  if (I) {
    free(I->xy.p), free(I->sub.p), free(I->closed.p), free(I->curved.p);
    free(I->ocg_num), free(I->ocg_h);
    for (int k = 0; k < I->nfonts; k++)
      free(I->fonts[k].map), free(I->fonts[k].cw), free(I->fonts[k].tt), free(I->fonts[k].cid2gid);
    free(I);
  }
  pdf_close(&P);
  free(buf);
  return rc;
}
