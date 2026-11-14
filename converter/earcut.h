// Polygon-with-holes triangulation for filled hatches: a C port of mapbox
// earcut (ISC licence, https://github.com/mapbox/earcut) without the z-order
// hash (boundaries here are at most a few thousand points).
//   ec_triangulate(xy, n, holes, nholes, out) appends vertex index triples
//   to out (uint32). Points [0, holes[0]) are the outer ring, each hole runs
//   to the next hole start (or n). Rings may have any orientation.
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct EcNode {
  double x, y;
  uint32_t i;
  int steiner;
  struct EcNode *prev, *next;
} EcNode;

typedef struct {
  EcNode *pool;
  size_t n, cap;
  Vec *out;
  int full;  // pool exhausted or work budget spent: give up on this polygon
} Ec;

// Work budget: on damaged hatches (crossed or corrupt rings) the ear search
// can loop for ever. Every loop step counts; past EC_WORK_MAX the polygon is
// given up like an exhausted pool, past EC_WORK_TOTAL (per drawing) no more
// fills are triangulated. Real drawings (484, 2026-11-12) need at most 1.3e6
// per polygon and 1.4e7 per drawing; about 1.5 s per 1e9 steps on a Mac.
// Maxima are reported on stderr (fill_work_max, fill_work_total).
#ifndef EC_WORK_MAX
#define EC_WORK_MAX 5e7
#endif
#ifndef EC_WORK_TOTAL
#define EC_WORK_TOTAL 5e8
#endif
static double ec_work, ec_work_max, ec_work_total;
#define EC_SPENT() (++ec_work > EC_WORK_MAX)

static EcNode *ec_new(Ec *c, uint32_t i, double x, double y) {
  if (c->n == c->cap) { c->full = 1; return NULL; }
  EcNode *p = &c->pool[c->n++];
  p->x = x, p->y = y, p->i = i, p->steiner = 0, p->prev = p->next = NULL;
  return p;
}

static EcNode *ec_insert(Ec *c, uint32_t i, double x, double y, EcNode *last) {
  EcNode *p = ec_new(c, i, x, y);
  if (!p) return last;
  if (!last) {
    p->prev = p->next = p;
  } else {
    p->next = last->next, p->prev = last;
    last->next->prev = p, last->next = p;
  }
  return p;
}

static void ec_remove(EcNode *p) { p->next->prev = p->prev, p->prev->next = p->next; }

static double ec_area(const EcNode *p, const EcNode *q, const EcNode *r) {
  return (q->y - p->y) * (r->x - q->x) - (q->x - p->x) * (r->y - q->y);
}

static int ec_equals(const EcNode *a, const EcNode *b) { return a->x == b->x && a->y == b->y; }

static int ec_sign(double v) { return v > 0 ? 1 : v < 0 ? -1 : 0; }

static int ec_on_segment(const EcNode *p, const EcNode *q, const EcNode *r) {
  return q->x <= fmax(p->x, r->x) && q->x >= fmin(p->x, r->x) &&
         q->y <= fmax(p->y, r->y) && q->y >= fmin(p->y, r->y);
}

static int ec_intersects(const EcNode *p1, const EcNode *q1, const EcNode *p2,
                         const EcNode *q2) {
  int o1 = ec_sign(ec_area(p1, q1, p2)), o2 = ec_sign(ec_area(p1, q1, q2));
  int o3 = ec_sign(ec_area(p2, q2, p1)), o4 = ec_sign(ec_area(p2, q2, q1));
  if (o1 != o2 && o3 != o4) return 1;
  if (o1 == 0 && ec_on_segment(p1, p2, q1)) return 1;
  if (o2 == 0 && ec_on_segment(p1, q2, q1)) return 1;
  if (o3 == 0 && ec_on_segment(p2, p1, q2)) return 1;
  if (o4 == 0 && ec_on_segment(p2, q1, q2)) return 1;
  return 0;
}

static int ec_point_in_triangle(double ax, double ay, double bx, double by,
                                double cx, double cy, double px, double py) {
  return (cx - px) * (ay - py) >= (ax - px) * (cy - py) &&
         (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
         (bx - px) * (cy - py) >= (cx - px) * (by - py);
}

static int ec_locally_inside(const EcNode *a, const EcNode *b) {
  return ec_area(a->prev, a, a->next) < 0
             ? ec_area(a, b, a->next) >= 0 && ec_area(a, a->prev, b) >= 0
             : ec_area(a, b, a->prev) < 0 || ec_area(a, a->next, b) < 0;
}

static int ec_middle_inside(const EcNode *a, const EcNode *b) {
  const EcNode *p = a;
  int inside = 0;
  double px = (a->x + b->x) / 2, py = (a->y + b->y) / 2;
  do {
    if (EC_SPENT()) break;
    if (((p->y > py) != (p->next->y > py)) && p->next->y != p->y &&
        (px < (p->next->x - p->x) * (py - p->y) / (p->next->y - p->y) + p->x))
      inside = !inside;
    p = p->next;
  } while (p != a);
  return inside;
}

static int ec_intersects_polygon(const EcNode *a, const EcNode *b) {
  const EcNode *p = a;
  do {
    if (EC_SPENT()) break;
    if (p->i != a->i && p->next->i != a->i && p->i != b->i &&
        p->next->i != b->i && ec_intersects(p, p->next, a, b))
      return 1;
    p = p->next;
  } while (p != a);
  return 0;
}

static int ec_valid_diagonal(const EcNode *a, const EcNode *b) {
  return a->next->i != b->i && a->prev->i != b->i &&
         !ec_intersects_polygon(a, b) &&
         ((ec_locally_inside(a, b) && ec_locally_inside(b, a) &&
           ec_middle_inside(a, b) &&
           (ec_area(a->prev, a, b->prev) != 0 || ec_area(a, b->prev, b) != 0)) ||
          (ec_equals(a, b) && ec_area(a->prev, a, a->next) > 0 &&
           ec_area(b->prev, b, b->next) > 0));
}

// Links a and b with a bridge, splitting the ring in two (or joining a hole
// to the outer ring). Returns the copy of b on the other side.
static EcNode *ec_split(Ec *c, EcNode *a, EcNode *b) {
  EcNode *a2 = ec_new(c, a->i, a->x, a->y), *b2 = ec_new(c, b->i, b->x, b->y);
  if (!a2 || !b2) return NULL;
  EcNode *an = a->next, *bp = b->prev;
  a->next = b, b->prev = a;
  a2->next = an, an->prev = a2;
  b2->next = a2, a2->prev = b2;
  bp->next = b2, b2->prev = bp;
  return b2;
}

// Removes duplicate and collinear points.
static EcNode *ec_filter(EcNode *start, EcNode *end) {
  if (!start) return start;
  if (!end) end = start;
  EcNode *p = start;
  int again;
  do {
    if (EC_SPENT()) break;
    again = 0;
    if (!p->steiner && (ec_equals(p, p->next) || ec_area(p->prev, p, p->next) == 0)) {
      ec_remove(p);
      p = end = p->prev;
      if (p == p->next) break;
      again = 1;
    } else {
      p = p->next;
    }
  } while (again || p != end);
  return end;
}

static void ec_emit(Ec *c, const EcNode *a, const EcNode *b, const EcNode *d) {
  uint32_t *t = (uint32_t *)vec_push(c->out);
  t[0] = a->i;
  t = (uint32_t *)vec_push(c->out), t[0] = b->i;
  t = (uint32_t *)vec_push(c->out), t[0] = d->i;
}

static int ec_is_ear(const EcNode *ear) {
  const EcNode *a = ear->prev, *b = ear, *d = ear->next;
  if (ec_area(a, b, d) >= 0) return 0;  // reflex
  double x0 = fmin(a->x, fmin(b->x, d->x)), y0 = fmin(a->y, fmin(b->y, d->y));
  double x1 = fmax(a->x, fmax(b->x, d->x)), y1 = fmax(a->y, fmax(b->y, d->y));
  for (const EcNode *p = d->next; p != a && !EC_SPENT(); p = p->next)
    if (p->x >= x0 && p->x <= x1 && p->y >= y0 && p->y <= y1 &&
        ec_point_in_triangle(a->x, a->y, b->x, b->y, d->x, d->y, p->x, p->y) &&
        ec_area(p->prev, p, p->next) >= 0)
      return 0;
  return 1;
}

static EcNode *ec_cure_local(Ec *c, EcNode *start) {
  EcNode *p = start;
  do {
    if (EC_SPENT()) break;
    EcNode *a = p->prev, *b = p->next->next;
    if (!ec_equals(a, b) && ec_intersects(a, p, p->next, b) &&
        ec_locally_inside(a, b) && ec_locally_inside(b, a)) {
      ec_emit(c, a, p, b);
      ec_remove(p), ec_remove(p->next);
      p = start = b;
    }
    p = p->next;
  } while (p != start);
  return ec_filter(p, NULL);
}

static void ec_linked(Ec *c, EcNode *ear, int pass);

static void ec_split_earcut(Ec *c, EcNode *start) {
  EcNode *a = start;
  do {
    if (EC_SPENT()) break;
    for (EcNode *b = a->next->next; b != a->prev && !EC_SPENT(); b = b->next) {
      if (a->i != b->i && ec_valid_diagonal(a, b)) {
        EcNode *d = ec_split(c, a, b);
        if (!d) return;
        a = ec_filter(a, a->next), d = ec_filter(d, d->next);
        ec_linked(c, a, 0), ec_linked(c, d, 0);
        return;
      }
    }
    a = a->next;
  } while (a != start);
}

static void ec_linked(Ec *c, EcNode *ear, int pass) {
  if (!ear || c->full) return;
  EcNode *stop = ear;
  while (ear->prev != ear->next) {
    if (EC_SPENT()) { c->full = 1; return; }
    EcNode *prev = ear->prev, *next = ear->next;
    if (ec_is_ear(ear)) {
      ec_emit(c, prev, ear, next);
      ec_remove(ear);
      ear = stop = next->next;
      continue;
    }
    ear = next;
    if (ear == stop) {  // no ear found: progressively more forceful passes
      if (pass == 0) ec_linked(c, ec_filter(ear, NULL), 1);
      else if (pass == 1) ec_linked(c, ec_cure_local(c, ec_filter(ear, NULL)), 2);
      else ec_split_earcut(c, ear);
      break;
    }
  }
}

// Ring [a, b) of xy as a circular list in the requested orientation.
static EcNode *ec_ring(Ec *c, const double *xy, size_t a, size_t b, int clockwise) {
  double sum = 0;
  for (size_t i = a, j = b - 1; i < b; j = i++)
    sum += (xy[2 * j] - xy[2 * i]) * (xy[2 * i + 1] + xy[2 * j + 1]);
  EcNode *last = NULL;
  if (clockwise == (sum > 0)) {
    for (size_t i = a; i < b; i++) last = ec_insert(c, (uint32_t)i, xy[2 * i], xy[2 * i + 1], last);
  } else {
    for (size_t i = b; i-- > a;) last = ec_insert(c, (uint32_t)i, xy[2 * i], xy[2 * i + 1], last);
  }
  if (last && ec_equals(last, last->next)) {
    ec_remove(last);
    last = last->next;
  }
  return last;
}

static EcNode *ec_leftmost(EcNode *start) {
  EcNode *p = start, *l = start;
  do {
    if (EC_SPENT()) break;
    if (p->x < l->x || (p->x == l->x && p->y < l->y)) l = p;
    p = p->next;
  } while (p != start);
  return l;
}

static int ec_sector_contains(const EcNode *m, const EcNode *p) {
  return ec_area(m->prev, m, p->prev) < 0 && ec_area(p->next, m, m->next) < 0;
}

// Outer-ring vertex the hole's leftmost point can be bridged to.
static EcNode *ec_bridge(EcNode *hole, EcNode *outer) {
  EcNode *p = outer, *m = NULL;
  double hx = hole->x, hy = hole->y, qx = -INFINITY;
  do {  // nearest edge to the left of the hole point
    if (EC_SPENT()) break;
    if (hy <= p->y && hy >= p->next->y && p->next->y != p->y) {
      double x = p->x + (hy - p->y) * (p->next->x - p->x) / (p->next->y - p->y);
      if (x <= hx && x > qx) {
        qx = x;
        m = p->x < p->next->x ? p : p->next;
        if (x == hx) return m;
      }
    }
    p = p->next;
  } while (p != outer);
  if (!m) return NULL;
  // A reflex vertex inside the triangle (hole, intersection, m) would make
  // the bridge cross the ring: take the one with the smallest angle.
  EcNode *stop = m;
  double mx = m->x, my = m->y, tan_min = INFINITY;
  p = m;
  do {
    if (EC_SPENT()) break;
    if (hx >= p->x && p->x >= mx && hx != p->x &&
        ec_point_in_triangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx,
                             hy, p->x, p->y)) {
      double tan = fabs(hy - p->y) / (hx - p->x);
      if (ec_locally_inside(p, hole) &&
          (tan < tan_min ||
           (tan == tan_min &&
            (p->x > m->x || (p->x == m->x && ec_sector_contains(m, p)))))) {
        m = p;
        tan_min = tan;
      }
    }
    p = p->next;
  } while (p != stop);
  return m;
}

static int ec_cmp_x(const void *a, const void *b) {
  const EcNode *p = *(EcNode *const *)a, *q = *(EcNode *const *)b;
  // Full order so the result does not depend on the platform's qsort.
  if (p->x != q->x) return p->x < q->x ? -1 : 1;
  if (p->y != q->y) return p->y < q->y ? -1 : 1;
  return p->i < q->i ? -1 : p->i > q->i;
}

static void ec_triangulate(const double *xy, size_t n, const size_t *holes,
                           size_t nholes, Vec *out) {
  if (ec_work_total > EC_WORK_TOTAL) return;
  Ec c = {NULL, 0, 3 * n + 2 * nholes + 16, out, 0};
  ec_work = 0;
  c.pool = (EcNode *)malloc(c.cap * sizeof(EcNode));
  EcNode *outer = ec_ring(&c, xy, 0, nholes ? holes[0] : n, 1);
  if (outer && outer->next != outer->prev) {
    if (nholes) {
      EcNode **q = (EcNode **)malloc(nholes * sizeof(EcNode *));
      size_t nq = 0;
      for (size_t k = 0; k < nholes; k++) {
        EcNode *r = ec_ring(&c, xy, holes[k], k + 1 < nholes ? holes[k + 1] : n, 0);
        if (!r) continue;
        if (r == r->next) r->steiner = 1;
        q[nq++] = ec_leftmost(r);
      }
      qsort(q, nq, sizeof *q, ec_cmp_x);
      for (size_t k = 0; k < nq && !c.full; k++) {
        EcNode *b = ec_bridge(q[k], outer);
        if (!b) continue;
        EcNode *r = ec_split(&c, b, q[k]);
        if (!r) break;
        ec_filter(r, r->next);
        outer = ec_filter(b, b->next);
      }
      free(q);
    }
    if (!c.full) ec_linked(&c, outer, 0);
  }
  free(c.pool);
  if (ec_work > ec_work_max) ec_work_max = ec_work;
  ec_work_total += ec_work;
}
