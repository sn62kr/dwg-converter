// ACIS bodies (REGION, 3DSOLID, BODY, the surface entities) as their edges.
//
// A DWG stores these as an ACIS model: text (SAT, "version 1", up to R2004
// style) or binary (SAB, "version 2"). AutoCAD's 2D wireframe shows the
// model's edges, so that is what is drawn: every `edge` record, between its
// two vertices, along its curve - straight, or an ellipse/circle arc; any
// other curve (splines) as its chord. Seen from above (z dropped) like the
// other 3D entities. Faces, loops and surfaces are not needed for that.
//
// Both formats give the same records: a name, then pointers to other records
// (by number), numbers and flags. Layouts differ a little by ACIS version
// (an id from 700, a pattern pointer later), so fields are taken from the end
// of the pointer list where the versions agree, and every pointer is checked
// to land on the kind of record expected - a file that does not fit draws
// nothing rather than garbage.
//
// Input is untrusted: every read is bounds checked, sizes are capped.

#define ACIS_MAX_BYTES (64u << 20)
#define ACIS_MAX_TOK 4000000u
#define ACIS_MAX_EDGES 500000u

enum { AC_OTHER, AC_BODY, AC_TRANSFORM, AC_EDGE, AC_VERTEX, AC_POINT, AC_STRAIGHT, AC_ELLIPSE };

typedef struct {
  uint8_t t;  // 'P' pointer (i), 'N' number (d), 'B' sense: 1 = reversed (i), 'S' anything else
  int64_t i;
  double d;
} AcTok;

typedef struct {
  uint32_t t0, nt;  // tokens after the name
  uint8_t kind;
} AcRec;

typedef struct {
  Vec tok, rec;
  long ver;
  uint32_t t0;     // first token of the record being read
  char name[40];   // its name while reading ("straight-curve")
  int named, full;
} Acis;

static void ac_tok(Acis *a, uint8_t t, int64_t i, double d) {
  if (a->tok.n >= ACIS_MAX_TOK) { a->full = 1; return; }
  AcTok *k = (AcTok *)vec_push(&a->tok);
  k->t = t, k->i = i, k->d = d;
}

static void ac_name(Acis *a, const char *s, size_t n) {
  size_t l = strlen(a->name);
  if (l && l + 1 < sizeof a->name) a->name[l++] = '-';
  for (size_t i = 0; i < n && l + 1 < sizeof a->name; i++) a->name[l++] = s[i];
  a->name[l] = 0;
  a->named = 1;
}

static void ac_end_record(Acis *a) {
  static const struct { const char *n; uint8_t k; } kinds[] = {
    {"body", AC_BODY}, {"transform", AC_TRANSFORM}, {"edge", AC_EDGE},
    {"vertex", AC_VERTEX}, {"point", AC_POINT}, {"straight-curve", AC_STRAIGHT},
    {"ellipse-curve", AC_ELLIPSE}};
  AcRec *r = (AcRec *)vec_push(&a->rec);
  r->t0 = a->t0, r->nt = (uint32_t)a->tok.n - a->t0, r->kind = AC_OTHER;
  for (size_t i = 0; i < sizeof kinds / sizeof *kinds; i++)
    if (!strcmp(a->name, kinds[i].n)) r->kind = kinds[i].k;
  a->t0 = (uint32_t)a->tok.n, a->name[0] = 0, a->named = 0;
}

// [+-]digits[.digits][e[+-]digits] only (or .digits), at most 63 characters:
// the C library's strtod is given nothing else (it aborted on odd input, D105).
static int ac_number(const char *s, size_t n, double *out) {
  char buf[64];
  if (!n || n >= sizeof buf) return 0;
  size_t i = 0, d = 0;
  if (s[i] == '+' || s[i] == '-') i++;
  while (i < n && s[i] >= '0' && s[i] <= '9') i++, d++;
  if (i < n && s[i] == '.') {
    i++;
    while (i < n && s[i] >= '0' && s[i] <= '9') i++, d++;
  }
  if (!d) return 0;
  if (i < n && (s[i] == 'e' || s[i] == 'E')) {
    i++;
    if (i < n && (s[i] == '+' || s[i] == '-')) i++;
    size_t e = 0;
    while (i < n && s[i] >= '0' && s[i] <= '9') i++, e++;
    if (!e || e > 4) return 0;
  }
  if (i != n) return 0;
  memcpy(buf, s, n), buf[n] = 0;
  *out = strtod(buf, NULL);
  return isfinite(*out);
}

// Text tokens of a SAT stream (also the numbers of a SAB transform, which
// are stored as one text string).
static void ac_text(Acis *a, const char *s, size_t n, int records) {
  size_t i = 0;
  while (i < n && !a->full) {
    while (i < n && (unsigned char)s[i] <= ' ') i++;
    if (i >= n) break;
    size_t b = i;
    while (i < n && (unsigned char)s[i] > ' ') i++;
    const char *w = s + b;
    size_t l = i - b;
    double v;
    if (records && l == 1 && w[0] == '#') {
      ac_end_record(a);  // also without a name: pointers count every record
    } else if (w[0] == '$') {
      if (ac_number(w + 1, l - 1, &v) && fabs(v) < 1e9) ac_tok(a, 'P', (int64_t)v, 0);
      else ac_tok(a, 'S', 0, 0);
    } else if (w[0] == '@') {  // "@7 unknown": that many bytes after one space
      if (ac_number(w + 1, l - 1, &v) && v >= 0 && v < (double)n) {
        size_t skip = (size_t)v;
        i = i < n ? i + 1 : i;
        i = skip > n - i ? n : i + skip;
      }
      ac_tok(a, 'S', 0, 0);
    } else if (ac_number(w, l, &v)) {
      // Before the name: "-12 body ...", a record number (old files).
      if (records && !a->named) continue;
      ac_tok(a, 'N', 0, v);
    } else if (records && !a->named) {
      ac_name(a, w, l);
    } else if (l == 8 && !memcmp(w, "reversed", 8)) {
      ac_tok(a, 'B', 1, 0);
    } else if (l == 7 && !memcmp(w, "forward", 7)) {
      ac_tok(a, 'B', 0, 0);
    } else {
      ac_tok(a, 'S', 0, 0);
    }
  }
}

static void ac_parse_sat(Acis *a, const char *s, size_t n) {
  // Header: "700 0 1 0", and from ACIS 2.0 two more lines (product, units).
  size_t i = 0, e = 0;
  while (e < n && s[e] != '\n') e++;
  while (i < e && (unsigned char)s[i] <= ' ') i++;
  size_t b = i;
  while (i < e && (unsigned char)s[i] > ' ') i++;
  double v;
  a->ver = ac_number(s + b, i - b, &v) && v > 0 && v < 1e7 ? (long)v : 0;
  if (!a->ver) return;
  i = e < n ? e + 1 : n;
  for (int k = 0; k < 2 && a->ver >= 200; k++) {
    while (i < n && s[i] != '\n') i++;
    if (i < n) i++;
  }
  ac_text(a, s + i, n - i, 1);
}

static void ac_parse_sab(Acis *a, const uint8_t *b, size_t n) {
  // "ACIS BinaryFile" with 4-byte integers, "ASM BinaryFile4" with 8-byte ones.
  size_t isz = !memcmp(b, "ASM BinaryFile4", 15) ? 8 : 4, pos = 15;
  if (n < pos + 4 * isz) return;
  a->ver = (long)(b[pos] | b[pos + 1] << 8 | b[pos + 2] << 16 | (uint32_t)(b[pos + 3] & 0x7f) << 24);
  pos += 4 * isz;
#define AC_NEED(k) if ((size_t)(k) > n - pos) return
  while (pos < n && !a->full) {
    uint8_t t = b[pos++];
    switch (t) {
    case 0x04: case 0x0C: case 0x15: {  // integer, pointer, enumeration
      AC_NEED(isz);
      int64_t v = 0;
      for (size_t k = 0; k < isz; k++) v |= (int64_t)((uint64_t)b[pos + k] << (8 * k));
      if (isz == 4) v = (int32_t)v;
      pos += isz;
      if (t == 0x0C) ac_tok(a, 'P', v, 0);
      else ac_tok(a, 'N', 0, (double)v);
    } break;
    case 0x06: case 0x13: case 0x14: case 0x16: {  // number; position, direction; u v
      int k = t == 0x06 ? 1 : t == 0x16 ? 2 : 3;
      AC_NEED(8 * k);
      for (int j = 0; j < k; j++, pos += 8) {
        double d;
        memcpy(&d, b + pos, 8);
        ac_tok(a, 'N', 0, isfinite(d) ? d : 0);
      }
    } break;
    case 0x07: case 0x0D: case 0x0E: {  // string, name, name prefix: 1-byte length
      AC_NEED(1);
      size_t l = b[pos++];
      AC_NEED(l);
      if (t != 0x07 && a->tok.n == a->t0) ac_name(a, (const char *)b + pos, l);
      else ac_tok(a, 'S', 0, 0);
      pos += l;
    } break;
    case 0x08: {  // string, 2-byte length
      AC_NEED(2);
      size_t l = b[pos] | b[pos + 1] << 8;
      pos += 2;
      AC_NEED(l);
      pos += l;
      ac_tok(a, 'S', 0, 0);
    } break;
    case 0x09: case 0x12: {  // string, 4-byte length (a transform's numbers are one)
      AC_NEED(4);
      size_t l = b[pos] | b[pos + 1] << 8 | b[pos + 2] << 16 | (size_t)b[pos + 3] << 24;
      pos += 4;
      AC_NEED(l);
      if (!strcmp(a->name, "transform")) ac_text(a, (const char *)b + pos, l, 0);
      else ac_tok(a, 'S', 0, 0);
      pos += l;
    } break;
    case 0x0A: ac_tok(a, 'B', 1, 0); break;
    case 0x0B: ac_tok(a, 'B', 0, 0); break;
    case 0x0F: case 0x10: ac_tok(a, 'S', 0, 0); break;  // subtype { }
    case 0x11:
      ac_end_record(a);  // the first one (asmheader) follows the file header's values
      break;
    default: return;  // unknown tag: what was read so far is still good
    }
  }
#undef AC_NEED
}

static const AcRec *ac_rec(const Acis *a, int64_t i, int kind) {
  if (i < 0 || (uint64_t)i >= a->rec.n) return NULL;
  const AcRec *r = (const AcRec *)a->rec.p + i;
  return r->kind == kind ? r : NULL;
}

// The record's fields after the attribute pointer (and the id, from 700).
static void ac_fields(const Acis *a, const AcRec *r, const AcTok **t, uint32_t *n) {
  uint32_t skip = a->ver >= 700 ? 2 : 1;
  if (skip > r->nt) skip = r->nt;
  *t = (const AcTok *)a->tok.p + r->t0 + skip, *n = r->nt - skip;
}

// The first k numbers of the record; 0 when there are fewer.
static int ac_nums(const Acis *a, const AcRec *r, double *out, int k) {
  const AcTok *t;
  uint32_t n;
  ac_fields(a, r, &t, &n);
  int m = 0;
  for (uint32_t i = 0; i < n && m < k; i++)
    if (t[i].t == 'N') out[m++] = t[i].d;
  return m == k;
}

static int64_t ac_last_ptr(const Acis *a, const AcRec *r) {
  const AcTok *t;
  uint32_t n;
  ac_fields(a, r, &t, &n);
  for (uint32_t i = n; i-- > 0;)
    if (t[i].t == 'P') return t[i].i;
  return -1;
}

typedef struct { int on; double m[9], t[3], s; } AcXf;

static void ac_apply(const AcXf *x, double *p, int vector) {
  if (!x->on) return;
  double q[3];
  for (int k = 0; k < 3; k++)
    q[k] = x->s * (p[0] * x->m[k] + p[1] * x->m[3 + k] + p[2] * x->m[6 + k]) + (vector ? 0 : x->t[k]);
  p[0] = q[0], p[1] = q[1], p[2] = q[2];
}

static int ac_vertex(const Acis *a, int64_t v, const AcXf *x, double *p) {
  const AcRec *vr = ac_rec(a, v, AC_VERTEX);
  const AcRec *pr = vr ? ac_rec(a, ac_last_ptr(a, vr), AC_POINT) : NULL;
  if (!pr || !ac_nums(a, pr, p, 3)) return 0;
  ac_apply(x, p, 0);
  return 1;
}

// Draws the model's edges into g with the current style. Returns how many.
static unsigned long acis_edges(Geom *g, const uint8_t *data, size_t n, int binary) {
  if (!data || n < 16 || n > ACIS_MAX_BYTES) return 0;
  Acis a = {0};
  a.tok.elem = sizeof(AcTok), a.rec.elem = sizeof(AcRec);
  if (binary) {
    if (memcmp(data, "ACIS BinaryFile", 15) && memcmp(data, "ASM BinaryFile4", 15)) return 0;
    ac_parse_sab(&a, data, n);
  } else {
    ac_parse_sat(&a, (const char *)data, n);
  }
  // The body's transform (its last pointer), when it has one: 3 x 3 matrix
  // (row vectors), shift, scale.
  AcXf xf = {0};
  for (size_t i = 0; i < a.rec.n && !xf.on; i++) {
    const AcRec *r = (const AcRec *)a.rec.p + i;
    const AcRec *tr = r->kind == AC_BODY ? ac_rec(&a, ac_last_ptr(&a, r), AC_TRANSFORM) : NULL;
    double v[13];
    if (!tr || !ac_nums(&a, tr, v, 13)) continue;
    memcpy(xf.m, v, sizeof xf.m), memcpy(xf.t, v + 9, sizeof xf.t);
    xf.s = v[12] > 0 && isfinite(v[12]) ? v[12] : 1;
    xf.on = 1;
  }
  unsigned long drawn = 0;
  for (size_t i = 0; i < a.rec.n && drawn < ACIS_MAX_EDGES; i++) {
    const AcRec *r = (const AcRec *)a.rec.p + i;
    if (r->kind != AC_EDGE) continue;
    const AcTok *t;
    uint32_t nt;
    ac_fields(&a, r, &t, &nt);
    int64_t ptr[4] = {-1, -1, -1, -1};  // the last four: vertex, vertex, coedge, curve
    int np = 0, rev = 0, has_sense = 0, after_ptr = 0;
    double num_after_last = -1;  // ACIS 1.x writes the sense as 0 / 1 after the curve
    for (uint32_t k = 0; k < nt; k++) {
      if (t[k].t == 'P') {
        ptr[0] = ptr[1], ptr[1] = ptr[2], ptr[2] = ptr[3], ptr[3] = t[k].i;
        np++, after_ptr = 1, num_after_last = -1;
        continue;
      }
      if (t[k].t == 'B' && !has_sense) rev = (int)t[k].i, has_sense = 1;
      if (t[k].t == 'N' && after_ptr) num_after_last = t[k].d;
      after_ptr = 0;
    }
    if (!has_sense && num_after_last == 1) rev = 1;
    double p0[3], p1[3];
    if (np < 4 || !ac_vertex(&a, ptr[0], &xf, p0) || !ac_vertex(&a, ptr[1], &xf, p1)) continue;
    const AcRec *el = ac_rec(&a, ptr[3], AC_ELLIPSE);
    double e[10];
    if (el && ac_nums(&a, el, e, 10)) {
      // P(u) = c + M cos u + m sin u, m = (normal x M) * ratio.
      double *c = e, *nv = e + 3, *M = e + 6, ratio = e[9];
      double nl = sqrt(nv[0] * nv[0] + nv[1] * nv[1] + nv[2] * nv[2]);
      double m[3] = {0, 0, 0};
      if (nl > 1e-12 && ratio > 1e-9 && ratio < 1e9) {
        m[0] = (nv[1] * M[2] - nv[2] * M[1]) / nl * ratio;
        m[1] = (nv[2] * M[0] - nv[0] * M[2]) / nl * ratio;
        m[2] = (nv[0] * M[1] - nv[1] * M[0]) / nl * ratio;
      }
      ac_apply(&xf, c, 0), ac_apply(&xf, M, 1), ac_apply(&xf, m, 1);
      double R2 = M[0] * M[0] + M[1] * M[1] + M[2] * M[2];
      double m2 = m[0] * m[0] + m[1] * m[1] + m[2] * m[2];
      if (R2 > 1e-24 && m2 > 1e-24 && isfinite(R2) && isfinite(m2)) {
        double d0[3] = {p0[0] - c[0], p0[1] - c[1], p0[2] - c[2]};
        double d1[3] = {p1[0] - c[0], p1[1] - c[1], p1[2] - c[2]};
        double t0 = atan2((d0[0] * m[0] + d0[1] * m[1] + d0[2] * m[2]) / m2,
                          (d0[0] * M[0] + d0[1] * M[1] + d0[2] * M[2]) / R2);
        double t1 = atan2((d1[0] * m[0] + d1[1] * m[1] + d1[2] * m[2]) / m2,
                          (d1[0] * M[0] + d1[1] * M[1] + d1[2] * M[2]) / R2);
        // Along the curve when forward, against it when reversed; the same
        // vertex at both ends is the whole ellipse.
        if (!rev && t1 <= t0 + 1e-9) t1 += 2 * M_PI;
        if (rev && t1 >= t0 - 1e-9) t1 -= 2 * M_PI;
        double R = sqrt(R2), cross = M[0] * m[1] - M[1] * m[0];
        int flat = fabs(M[2]) < 1e-9 * R && fabs(m[2]) < 1e-9 * R &&
                   fabs(sqrt(m2) - R) < 1e-9 * R && cross != 0;
        if (flat) {  // a circle seen from straight above: a true arc
          int ccw = (cross > 0) == !rev;
          double b0 = atan2(d0[1], d0[0]), b1 = atan2(d1[1], d1[0]);
          if (fabs(fabs(t1 - t0) - 2 * M_PI) < 1e-9) b0 = 0, b1 = 2 * M_PI;
          else if (!ccw) { double w = b0; b0 = b1, b1 = w; }
          add_arc(g, XF_ID, c[0], c[1], R, b0, b1);
        } else {
          int k = umg_arc_segs(fabs(t1 - t0));
          begin_poly(g, (uint32_t)k + 1, UMG_LTCONT);
          for (int j = 0; j <= k; j++) {
            double u = t0 + (t1 - t0) * j / k;
            add_pt(g, XF_ID, c[0] + M[0] * cos(u) + m[0] * sin(u),
                   c[1] + M[1] * cos(u) + m[1] * sin(u), 0);
          }
        }
        drawn++;
        continue;
      }
    }
    // A straight edge; any other curve (spline) as its chord.
    if (p0[0] != p1[0] || p0[1] != p1[1]) {
      add_line(g, XF_ID, p0[0], p0[1], p1[0], p1[1]);
      drawn++;
    }
  }
  free(a.tok.p), free(a.rec.p);
  return drawn;
}
