/*
 * plan.c - the show planner.  See plan.h for what it does and why.
 */
#include <string.h>
#include "plan.h"
#include "fmath.h"
#include "fmt.h"
#include "trig.h"

#define MIN_ALT     2.0f         /* m: no slot, and no repaired path, lower    */
#define SHORTEST    2.0f         /* s: the quickest any move is allowed to be  */
#define HOLD_PADS   1.0f         /* s on the pads before the climb             */
#define HOLD_HOME   1.0f
#define HOLD_LAND   8.0f         /* s the plan allows for landing              */

static float dist2(v3_t a, v3_t b)                     /* squared: no root */
{
    float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

static float dist(v3_t a, v3_t b) { return t_sqrt(dist2(a, b)); }

/* A squared distance above this is surely farther than d - no square root
 * needed to know.  The margin covers t_sqrt()'s rounding (1e-7) many times
 * over, so a pair it skips could never have been the closer one. */
static float beyond(float d) { return d * d * 1.00002f; }

/* metres as "2.35" - fmt.c prints no floats */
static const char *metres(char *buf, float m)          /* buf: 16 bytes */
{
    int cm = (int)(m * 100.0f + (m >= 0.0f ? 0.5f : -0.5f)), a = cm < 0 ? -cm : cm;
    fmt(buf, 16, "%s%d.%02d", cm < 0 ? "-" : "", a / 100, a % 100);
    return buf;
}

/* ============================================================================
 *  Shapes: n points (u, w) in a local plane, centred on 0, `s` metres across
 * ============================================================================ */
static void shape_line(int n, float s, float *u, float *w)
{
    for (int k = 0; k < n; k++) { u[k] = n > 1 ? -s / 2 + s * (float)k / (float)(n - 1) : 0.0f;  w[k] = 0.0f; }
}

static void shape_v(int n, float s, float *u, float *w)
{
    /* the apex, then pairs further out on alternate arms */
    int ranks = n / 2 > 0 ? n / 2 : 1;
    float du = (s / 2) / (float)ranks, top = 0.0f;
    u[0] = w[0] = 0.0f;
    for (int k = 1; k < n; k++) {
        int r = (k + 1) / 2;
        u[k] = du * (float)r * (k % 2 ? 1.0f : -1.0f);
        w[k] = 1.2f * du * (float)r;
        if (w[k] > top) { top = w[k]; }
    }
    for (int k = 0; k < n; k++) { w[k] -= top / 2; }
}

static void shape_circle(int n, float s, float *u, float *w)
{
    for (int k = 0; k < n; k++) {
        float a = F_PI_2 + F_2PI * (float)k / (float)n, sn, cs;
        t_sincos(a, &sn, &cs);
        u[k] = (s / 2) * cs;  w[k] = (s / 2) * sn;
    }
}

static void shape_cross(int n, float s, float *u, float *w)
{
    static const int8_t dir[4][2] = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } };
    int per_arm = (n - 1 + 3) / 4 > 0 ? (n - 1 + 3) / 4 : 1;
    float step = (s / 2) / (float)per_arm;
    u[0] = w[0] = 0.0f;
    for (int k = 1; k < n; k++) {
        int a = (k - 1) % 4, r = (k - 1) / 4 + 1;
        u[k] = dir[a][0] * (float)r * step;  w[k] = dir[a][1] * (float)r * step;
    }
}

static void shape_grid(int n, float s, float *u, float *w)
{
    int cols = 1;
    while (cols * cols < n) { cols++; }
    int rows = (n + cols - 1) / cols;
    float d = s / (float)(cols > 1 ? cols - 1 : 1);
    for (int k = 0; k < n; k++) {
        int r = k / cols, c = k % cols;
        int in_row = n - r * cols < cols ? n - r * cols : cols;     /* a short last row is centred */
        u[k] = ((float)c - (float)(in_row - 1) / 2) * d;
        w[k] = ((float)(rows - 1) / 2 - (float)r) * d;
    }
}

/* ---- the heart ----------------------------------------------------------
 * The classic heart curve.  Drones go evenly ALONG it (even steps of t would
 * bunch them at the tip and the notch).  Tip and notch are sharp corners -
 * two drones either side of one would nearly touch - so one drone sits ON
 * each, the rest share the sides.  An odd count makes that lopsided, so the
 * symmetric layout (notch left empty) is used when it costs little spacing. */
static void heart_at(float t, float *x, float *y)
{
    /* x = 16 sin^3 t,  y = 13 cos t - 5 cos 2t - 2 cos 3t - cos 4t.
     * One sin and one cos; the multiple angles follow from them
     * (cos 2t = 2 cos^2 t - 1, and so on) for a few multiplies each.
     * The curve is walked ~2000 times at power-up: it adds up. */
    float s, c;
    t_sincos(t, &s, &c);
    float c2 = 2.0f * c * c - 1.0f;                /* cos 2t */
    float c3 = c * (2.0f * c2 - 1.0f);             /* cos 3t */
    float c4 = 2.0f * c2 * c2 - 1.0f;              /* cos 4t */
    *x = 16.0f * s * s * s;
    *y = 13.0f * c - 5.0f * c2 - 2.0f * c3 - c4;
}

#define HEART_STEPS 240

/* m points strictly between heart(t0) and heart(t1), evenly by length */
static void heart_along(float t0, float t1, int m, float *u, float *w)
{
    float total = 0.0f, px, py, x, y;
    heart_at(t0, &px, &py);
    for (int k = 1; k <= HEART_STEPS; k++) {                     /* pass 1: length */
        heart_at(t0 + (t1 - t0) * (float)k / HEART_STEPS, &x, &y);
        total += t_sqrt((x - px) * (x - px) + (y - py) * (y - py));
        px = x;  py = y;
    }
    float run = 0.0f;
    int placed = 0;
    heart_at(t0, &px, &py);
    for (int k = 1; k <= HEART_STEPS && placed < m; k++) {       /* pass 2: place */
        heart_at(t0 + (t1 - t0) * (float)k / HEART_STEPS, &x, &y);
        run += t_sqrt((x - px) * (x - px) + (y - py) * (y - py));
        px = x;  py = y;
        while (placed < m && run >= total * (float)(placed + 1) / (float)(m + 1)) {
            u[placed] = x;  w[placed] = y;  placed++;
        }
    }
    while (placed < m) { u[placed] = x;  w[placed] = y;  placed++; }
}

static float min_gap(int n, const float *u, const float *w)
{
    float g = 1e9f;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            float d = t_sqrt((u[i] - u[j]) * (u[i] - u[j]) + (w[i] - w[j]) * (w[i] - w[j]));
            if (d < g) { g = d; }
        }
    }
    return g;
}

static void shape_heart(int n, float s, float *u, float *w)
{
    float a[SHOW_MAX_DRONES], b[SHOW_MAX_DRONES];
    heart_at(F_PI, &u[0], &w[0]);                                /* the tip   */
    if (n > 1) {
        int left = (n - 2) / 2, right = n - 2 - left;
        heart_along(F_PI, F_2PI, right, &u[1], &w[1]);
        heart_at(0.0f, &u[1 + right], &w[1 + right]);            /* the notch */
        heart_along(0.0f, F_PI, left, &u[2 + right], &w[2 + right]);
        if (n % 2) {
            int half = (n - 1) / 2;
            a[0] = u[0];  b[0] = w[0];
            heart_along(F_PI, F_2PI, half, &a[1], &b[1]);
            heart_along(0.0f, F_PI, half, &a[1 + half], &b[1 + half]);
            if (min_gap(n, a, b) >= 0.9f * min_gap(n, u, w)) {
                memcpy(u, a, sizeof(float) * (size_t)n);
                memcpy(w, b, sizeof(float) * (size_t)n);
            }
        }
    }
    /* scale the curve's bounding box to `s` across, centred */
    float lo_x = 1e9f, hi_x = -1e9f, lo_y = 1e9f, hi_y = -1e9f, x, y;
    for (int k = 0; k < 200; k++) {
        heart_at(F_2PI * (float)k / 200.0f, &x, &y);
        if (x < lo_x) { lo_x = x; }
        if (x > hi_x) { hi_x = x; }
        if (y < lo_y) { lo_y = y; }
        if (y > hi_y) { hi_y = y; }
    }
    float span = (hi_x - lo_x) > (hi_y - lo_y) ? hi_x - lo_x : hi_y - lo_y;
    float k = s / span, cu = (hi_x + lo_x) / 2, cw = (hi_y + lo_y) / 2;
    for (int i = 0; i < n; i++) { u[i] = (u[i] - cu) * k;  w[i] = (w[i] - cw) * k; }
}

void plan_shape(const form_t *f, int n, v3_t *out)
{
    if (f->shape == SH_POINTS) {
        for (int i = 0; i < n; i++) { out[i] = f->points ? f->points[i] : (v3_t){ 0 }; }
        return;
    }
    float u[SHOW_MAX_DRONES], w[SHOW_MAX_DRONES];
    switch (f->shape) {
    case SH_V:      shape_v(n, f->size, u, w);      break;
    case SH_CIRCLE: shape_circle(n, f->size, u, w); break;
    case SH_CROSS:  shape_cross(n, f->size, u, w);  break;
    case SH_GRID:   shape_grid(n, f->size, u, w);   break;
    case SH_HEART:  shape_heart(n, f->size, u, w);  break;
    default:        shape_line(n, f->size, u, w);   break;
    }
    float c, s;
    t_sincos(f->turn * (F_PI / 180.0f), &s, &c);
    for (int i = 0; i < n; i++) {
        if (f->plane == WALL) {          /* standing up, facing south */
            out[i] = (v3_t){ f->x + u[i] * c, f->y + u[i] * s, f->z + w[i] };
        } else {                         /* lying flat                */
            out[i] = (v3_t){ f->x + u[i] * c - w[i] * s, f->y + u[i] * s + w[i] * c, f->z };
        }
    }
}

/* ============================================================================
 *  Who goes where: the Hungarian algorithm (Kuhn 1955, Munkres 1957)
 * ============================================================================
 * The potentials form, O(n^3).  u[] and v[] are prices on rows (drones) and
 * columns (slots), kept so every cost minus its two prices stays >= 0; rows
 * join one at a time, each along the cheapest augmenting path in those
 * reduced costs.  Index 0 is a sentinel, so arrays run 0..n.               */
void plan_assign(const v3_t *from, const v3_t *to, int n, uint8_t *who)
{
    float cost[SHOW_MAX_DRONES][SHOW_MAX_DRONES];
    float u[SHOW_MAX_DRONES + 1] = { 0 }, v[SHOW_MAX_DRONES + 1] = { 0 }, best[SHOW_MAX_DRONES + 1];
    int owner[SHOW_MAX_DRONES + 1] = { 0 }, way[SHOW_MAX_DRONES + 1] = { 0 };
    uint8_t used[SHOW_MAX_DRONES + 1];

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            float dx = from[i].x - to[j].x, dy = from[i].y - to[j].y, dz = from[i].z - to[j].z;
            cost[i][j] = dx * dx + dy * dy + dz * dz;            /* SQUARED: see plan.h */
        }
    }
    for (int row = 1; row <= n; row++) {
        owner[0] = row;
        int col0 = 0;
        for (int c = 0; c <= n; c++) { best[c] = 1e30f;  used[c] = 0; }
        do {
            used[col0] = 1;
            int r0 = owner[col0], col1 = 0;
            float delta = 1e30f;
            for (int c = 1; c <= n; c++) {
                if (used[c]) { continue; }
                float cur = cost[r0 - 1][c - 1] - u[r0] - v[c];
                if (cur < best[c]) { best[c] = cur;  way[c] = col0; }
                if (best[c] < delta) { delta = best[c];  col1 = c; }
            }
            for (int c = 0; c <= n; c++) {
                if (used[c]) { u[owner[c]] += delta;  v[c] -= delta; }
                else         { best[c] -= delta; }
            }
            col0 = col1;
        } while (owner[col0] != 0);
        do {                                                   /* flip the path */
            int col1 = way[col0];
            owner[col0] = owner[col1];
            col0 = col1;
        } while (col0);
    }
    for (int c = 1; c <= n; c++) { who[owner[c] - 1] = (uint8_t)(c - 1); }
}

/* ============================================================================
 *  How fast: one trapezoidal profile per move
 * ============================================================================
 * Distance along a path of length L: speed up at a to vp, cruise, brake.
 * Every drone in a move uses the SAME profile as a fraction of L, times its
 * own path length - so all start and arrive together.                      */
static profile_t profile_make(float L, float vmax, float amax)
{
    profile_t p = { L, 0.0f, SHORTEST / 2, SHORTEST };
    if (L < 1e-6f) { return p; }
    if (L >= vmax * vmax / amax) {                 /* reaches cruise speed      */
        p.vp = vmax;  p.ta = vmax / amax;  p.T = L / vmax + vmax / amax;
    } else {                                       /* a triangle                */
        p.vp = t_sqrt(L * amax);  p.ta = p.vp / amax;  p.T = 2 * p.ta;
    }
    if (p.T < SHORTEST) {                          /* stretch very short moves  */
        float k = p.T / SHORTEST;
        p.vp *= k;  p.ta /= k;  p.T = SHORTEST;
    }
    return p;
}

/* fraction done (0..1) and fraction per second, t s into the move */
static void profile_at(const profile_t *p, float t, float *frac, float *rate)
{
    if (p->L < 1e-6f) { *frac = f_clamp(t / p->T, 0.0f, 1.0f);  *rate = 0.0f;  return; }
    t = f_clamp(t, 0.0f, p->T);
    float a = p->vp / p->ta, d, v;
    if (t < p->ta)              { d = 0.5f * a * t * t;  v = a * t; }
    else if (t < p->T - p->ta)  { d = 0.5f * a * p->ta * p->ta + p->vp * (t - p->ta);  v = p->vp; }
    else                        { float r = p->T - t;  d = p->L - 0.5f * a * r * r;  v = a * r; }
    *frac = d / p->L;  *rate = v / p->L;
}

/* ============================================================================
 *  Paths and sampling
 * ============================================================================ */
static int path(const plan_t *p, const stage_t *s, int i, v3_t pt[4])
{
    v3_t a = p->spot[s->from][i], b = p->spot[s->to][i];
    float h = (s->kind == ST_MOVE) ? (float)p->lift[s->from][i] * 0.1f : 0.0f;
    if (s->from == s->to) { pt[0] = a;  return 1; }
    if (h == 0.0f)        { pt[0] = a;  pt[1] = b;  return 2; }
    pt[0] = a;  pt[1] = (v3_t){ a.x, a.y, a.z + h };
    pt[2] = (v3_t){ b.x, b.y, b.z + h };  pt[3] = b;
    return 4;
}

static float path_len(const v3_t *pt, int n)
{
    float L = 0.0f;
    for (int k = 0; k + 1 < n; k++) { L += dist(pt[k], pt[k + 1]); }
    return L;
}

/* One profile for the whole move, sized so NO drone breaks a limit: planned
 * for the longest path - or longer, if some path is so steep that vmax
 * along it would climb or sink faster than vzmax. */
static void retime(plan_t *p, stage_t *s)
{
    const show_t *sh = p->show;
    float eff = 0.0f;
    for (int i = 0; i < p->n; i++) {
        v3_t pt[4];
        int m = path(p, s, i, pt);
        float total = path_len(pt, m), steep = 0.0f;
        for (int k = 0; k + 1 < m; k++) {
            float L = dist(pt[k], pt[k + 1]);
            if (L > 1e-6f) {
                float r = f_abs(pt[k + 1].z - pt[k].z) / L;
                if (r > steep) { steep = r; }
            }
        }
        float need = total * steep * sh->vmax / sh->vzmax;
        if (total > eff) { eff = total; }
        if (need > eff)  { eff = need; }
    }
    s->prof = profile_make(eff, sh->vmax, sh->amax);
    s->T = s->prof.T;
}

/* does anything move in this stage?  On the pads, in a hold, landing: no. */
static int moving(const stage_t *s) { return s->kind == ST_TAKEOFF || s->kind == ST_MOVE; }

/* drone i, `frac` of the way along its path, covering `rate` paths a second */
static void along(const plan_t *p, const stage_t *s, int i, float frac, float rate, v3_t *pos, v3_t *vel)
{
    v3_t pt[4];
    int m = path(p, s, i, pt);
    *vel = (v3_t){ 0 };
    if (m == 1) { *pos = pt[0];  return; }
    float len[3], total = 0.0f;                      /* each leg measured once */
    for (int k = 0; k + 1 < m; k++) { len[k] = dist(pt[k], pt[k + 1]);  total += len[k]; }
    float d = frac * total, speed = rate * total;
    for (int k = 0; k + 1 < m; k++) {
        float L = len[k];
        if (d <= L || k + 2 == m) {
            if (L < 1e-6f) { *pos = pt[k + 1];  return; }
            /* two divisions, then only multiplies: a division costs three */
            float f = d / L, v = speed / L;
            if (f > 1.0f) { f = 1.0f; }
            v3_t leg = { pt[k + 1].x - pt[k].x, pt[k + 1].y - pt[k].y, pt[k + 1].z - pt[k].z };
            *pos = (v3_t){ pt[k].x + leg.x * f, pt[k].y + leg.y * f, pt[k].z + leg.z * f };
            *vel = (v3_t){ leg.x * v, leg.y * v, leg.z * v };
            return;
        }
        d -= L;
    }
    *pos = pt[m - 1];
}

void plan_sample(const plan_t *p, const stage_t *s, int i, float t, v3_t *pos, v3_t *vel)
{
    if (!moving(s)) { *pos = p->spot[s->to][i];  *vel = (v3_t){ 0 };  return; }
    float frac, rate;
    profile_at(&s->prof, t - s->t0, &frac, &rate);
    along(p, s, i, frac, rate, pos, vel);
}

/* Every drone at once.  They all fly the SAME speed profile, so how far
 * along the move is at time t is worked out once, not once per drone. */
void plan_sample_all(const plan_t *p, const stage_t *s, float t, v3_t *pos, v3_t *vel)
{
    float frac = 0.0f, rate = 0.0f;
    int go = moving(s);
    if (go) { profile_at(&s->prof, t - s->t0, &frac, &rate); }
    for (int i = 0; i < p->n; i++) {
        if (go) { along(p, s, i, frac, rate, &pos[i], &vel[i]); }
        else    { pos[i] = p->spot[s->to][i];  vel[i] = (v3_t){ 0 }; }
    }
}

v3_t plan_stage_end(const plan_t *p, const stage_t *s, int i) { return p->spot[s->to][i]; }

float plan_duration(const plan_t *p)
{
    const stage_t *s = &p->st[p->n_stages - 1];
    return s->t0 + s->T;
}

const stage_t *plan_stage(const plan_t *p, float t, int *index)
{
    int k = 0;                         /* a stage ends where the next begins */
    while (k + 1 < p->n_stages && t >= p->st[k + 1].t0) { k++; }
    if (index) { *index = k; }
    return &p->st[k];
}

static const char *const kind_text[ST_COUNT] = { "pads", "takeoff", "move", "hold", "home", "land" };

const char *plan_kind_name(uint8_t kind) { return kind < ST_COUNT ? kind_text[kind] : "?"; }

const char *plan_stage_name(const plan_t *p, const stage_t *s)
{
    if ((s->kind == ST_MOVE || s->kind == ST_HOLD) && s->form >= 0) { return p->show->forms[s->form].name; }
    if (s->kind == ST_MOVE) { return "home"; }
    return kind_text[s->kind];
}

/* ============================================================================
 *  Check and repair
 * ============================================================================ */
typedef struct { int bad; float d, t; uint8_t i, j; } check_t;

/* Every pair, every PLAN_DT_CHECK s: how many pairs come closer than sep,
 * and the closest pass of all.
 *
 * Why 0.2 s is fine-grained enough: no drone flies faster than vmax (3 m/s),
 * so two drones close on each other at most 6 m/s, 1.2 m between samples.
 * If the true closest moment falls halfway, the sample before it is at most
 * 0.6 m of relative travel away, and at a 3 m pass that reads
 * sqrt(3^2 + 0.6^2) = 3.06 m: 6 cm too generous.  host/test_show.c re-checks
 * every plan at 20 ms.  Checking costs the chip the most at power-up (in
 * Wokwi, seconds), so every halving counts. */
static check_t check(const plan_t *p, const stage_t *s, float sep, uint8_t *close /* n*n or 0 */)
{
    check_t c = { 0, 1e9f, 0.0f, 0, 0 };
    float least[SHOW_MAX_DRONES * SHOW_MAX_DRONES], bound[SHOW_MAX_DRONES * SHOW_MAX_DRONES];
    int n = p->n, steps = (int)(s->T / PLAN_DT_CHECK) + 1;
    for (int k = 0; k < n * n; k++) { least[k] = 1e9f;  bound[k] = beyond(1e9f); }
    for (int k = 0; k <= steps; k++) {
        float t = s->t0 + (k < steps ? (float)k * PLAN_DT_CHECK : s->T);
        v3_t at[SHOW_MAX_DRONES], v[SHOW_MAX_DRONES];
        plan_sample_all(p, s, t, at, v);
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                /* most pairs, most moments, are no closer than they have been:
                 * the squared distance says so without a square root */
                float q = dist2(at[i], at[j]);
                if (q > bound[i * n + j]) { continue; }
                float d = t_sqrt(q);
                if (d < least[i * n + j]) { least[i * n + j] = d;  bound[i * n + j] = beyond(d); }
                if (d < c.d) { c.d = d;  c.t = t;  c.i = (uint8_t)i;  c.j = (uint8_t)j; }
            }
        }
    }
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            int b = least[i * n + j] < sep;
            c.bad += b;
            if (close) { close[i * n + j] = (uint8_t)b; }
        }
    }
    return c;
}

/* Fly one of the closest too-close pair over or under the other, trying both
 * drones and four heights, and keep the try that leaves the fewest problems
 * (then the smallest climb, then the shortest move).  Repeat until clean. */
static int repair(plan_t *p, stage_t *s, check_t *last)
{
    const float sep = p->show->sep;
    const float heights[4] = { sep, -sep, 2 * sep, -2 * sep };   /* <= 12.7 m: an int8 in dm */
    for (int tries = 0; tries < 4 + p->n; tries++) {
        check_t c = check(p, s, sep, 0);
        *last = c;                                    /* the caller keeps the closest pass */
        if (c.bad == 0) { return 0; }
        int best_who = -1;
        float best_h = 0.0f, best_score[4] = { 0 };
        for (int w = 0; w < 2; w++) {
            int who = w == 0 ? c.j : c.i;
            int8_t old = p->lift[s->from][who];
            v3_t a = p->spot[s->from][who], b = p->spot[s->to][who];
            for (int k = 0; k < 4; k++) {
                float h = heights[k];
                if ((a.z < b.z ? a.z : b.z) + h < MIN_ALT) { continue; }
                p->lift[s->from][who] = (int8_t)(h * 10.0f + (h > 0.0f ? 0.5f : -0.5f));
                retime(p, s);
                uint8_t close[SHOW_MAX_DRONES * SHOW_MAX_DRONES];
                check_t after = check(p, s, sep, close);
                float score[4] = { (float)close[c.i * p->n + c.j], (float)after.bad, f_abs(h), s->T };
                int better = best_who < 0;
                for (int q = 0; q < 4 && !better; q++) {
                    if (score[q] < best_score[q]) { better = 1; }
                    else if (score[q] > best_score[q]) { break; }
                }
                if (better) { best_who = who;  best_h = h;  memcpy(best_score, score, sizeof score); }
                p->lift[s->from][who] = old;
                retime(p, s);
            }
        }
        if (best_who < 0) { return -1; }
        p->lift[s->from][best_who] = (int8_t)(best_h * 10.0f + (best_h > 0.0f ? 0.5f : -0.5f));
        retime(p, s);
        p->repairs++;
    }
    *last = check(p, s, sep, 0);
    return last->bad ? -1 : 0;
}

/* the closest planned pass so far: kept as the plan is built, not found by a
 * second pass over the whole show (that doubled the chip's planning time) */
static void closer(plan_t *p, float d, float t, int i, int j)
{
    if (d < p->closest) { p->closest = d;  p->closest_t = t;  p->ci = (uint8_t)i;  p->cj = (uint8_t)j; }
}

/* ============================================================================
 *  The whole plan
 * ============================================================================ */
static stage_t *add(plan_t *p, uint8_t kind, int from, int to, int form, float hold)
{
    stage_t *s = &p->st[p->n_stages];
    float t0 = p->n_stages ? p->st[p->n_stages - 1].t0 + p->st[p->n_stages - 1].T : 0.0f;
    *s = (stage_t){ .kind = kind, .from = (uint8_t)from, .to = (uint8_t)to, .form = (int8_t)form,
                    .t0 = t0, .T = hold };
    p->n_stages++;
    if (kind == ST_TAKEOFF || kind == ST_MOVE) { retime(p, s); }
    return s;
}

int plan_build(plan_t *p, const show_t *sh, plan_progress_fn progress)
{
    char m1[16], m2[16];
    memset(p, 0, sizeof *p);
    p->show = sh;
    p->n = sh->drones;
    int n = p->n;
    if (n < 1 || n > SHOW_MAX_DRONES) {
        fmt(p->why, sizeof p->why, "drones: 1 .. %d on this build", SHOW_MAX_DRONES);
        return -1;
    }
    if (sh->n_forms > SHOW_MAX_FORMS) {
        fmt(p->why, sizeof p->why, "at most %d formations", SHOW_MAX_FORMS);
        return -1;
    }
    if (sh->pads < sh->sep) {
        fmt(p->why, sizeof p->why, "pads %s m apart, closer than sep %s m", metres(m1, sh->pads), metres(m2, sh->sep));
        return -1;
    }

    for (int i = 0; i < n; i++) {                                   /* spot 0, 1 */
        float x = ((float)i - (float)(n - 1) / 2) * sh->pads;
        p->spot[0][i] = (v3_t){ x, 0.0f, 0.0f };
        p->spot[1][i] = (v3_t){ x, 0.0f, sh->takeoff };
    }
    add(p, ST_PADS, 0, 0, -1, HOLD_PADS);
    add(p, ST_TAKEOFF, 0, 1, -1, 0.0f);
    p->closest = 1e9f;
    if (n > 1) { closer(p, sh->pads, p->st[1].t0, 0, 1); }       /* straight up, side by side */

    int here = 1;
    for (int f = 0; f <= sh->n_forms; f++) {
        v3_t goal[SHOW_MAX_DRONES];
        int home = f == sh->n_forms;
        const char *name = home ? "home" : sh->forms[f].name;
        if (home) { memcpy(goal, p->spot[1], sizeof goal); }
        else      { plan_shape(&sh->forms[f], n, goal); }

        float tight = 1e9f;                        /* the tightest slot pair */
        for (int i = 0; i < n; i++) {                               /* the slots */
            if (goal[i].z < MIN_ALT) {
                fmt(p->why, sizeof p->why, "%s: a slot at %s m - keep every slot above 2 m", name, metres(m1, goal[i].z));
                return -1;
            }
            for (int j = i + 1; j < n; j++) {
                float d = dist(goal[i], goal[j]);
                if (d < sh->sep - 1e-4f) {
                    fmt(p->why, sizeof p->why, "%s: slots %d and %d are %s m apart (sep %s) - make it bigger",
                             name, i, j, metres(m1, d), metres(m2, sh->sep));
                    return -1;
                }
                if (d < tight) { tight = d; }
            }
        }
        uint8_t who[SHOW_MAX_DRONES];
        plan_assign(p->spot[here], goal, n, who);
        for (int i = 0; i < n; i++) { p->spot[here + 1][i] = goal[who[i]]; }

        stage_t *mv = add(p, ST_MOVE, here, here + 1, home ? -1 : f, 0.0f);
        check_t c;
        if (repair(p, mv, &c) != 0) {
            fmt(p->why, sizeof p->why, "move to %s: drones %d and %d pass %s m apart, no repair worked",
                     name, c.i, c.j, metres(m1, c.d));
            return -1;
        }
        if (n > 1) { closer(p, c.d, c.t, c.i, c.j); }
        if (progress) { progress(f + 1, sh->n_forms + 1); }
        here++;
        stage_t *hold = home ? add(p, ST_HOME, here, here, -1, HOLD_HOME)
                             : add(p, ST_HOLD, here, here, f, sh->forms[f].hold);
        if (n > 1) {                                  /* a hold: who stands where, ordered as flown */
            for (int i = 0; i < n; i++) {
                for (int j = i + 1; j < n; j++) {
                    float d = dist(p->spot[here][i], p->spot[here][j]);
                    if (d <= tight + 1e-4f) { closer(p, d, hold->t0, i, j); }
                }
            }
        }
    }
    add(p, ST_LAND, here, here, -1, HOLD_LAND);
    p->n_spots = (uint8_t)(here + 1);
    return 0;
}
