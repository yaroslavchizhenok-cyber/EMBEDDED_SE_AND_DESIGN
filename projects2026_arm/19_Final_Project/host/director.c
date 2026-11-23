/*
 * director.c - runs the show.  See director.h.
 */
#include <stdarg.h>
#include <string.h>
#include "director.h"
#include "craft.h"           /* GPS_PLAIN, GPS_RTK */
#include "pilot.h"
#include "proto.h"
#include "fmath.h"
#include "fmt.h"
#include "trig.h"

#define TICK_S        0.01f          /* one fleet_step()                         */
#define EVERY         10u            /* ticks: targets and frames at 10 Hz       */
#define WARM_TICKS    130u           /* 1.3 s on the pads: the 10th GPS fix, which
                                        pilot_arm() needs, arrives at tick 110    */
#define GIVE_UP_S     30.0f          /* past the plan's end: stop waiting         */

static const char *const state_text[DR_COUNT] = { "REFUSED", "WARMUP", "READY", "FLYING", "LANDING", "DONE" };

const char *director_state_name(uint8_t s) { return s < DR_COUNT ? state_text[s] : "?"; }

/* ---- output: lesson 08's frames, and plain text --------------------------- */
static long cm(float m) { return (long)(m * 100.0f + (m >= 0.0f ? 0.5f : -0.5f)); }

/* metres as "2.35": fmt.c prints no floats.  buf: 16 B */
static const char *fm(char *buf, float m)
{
    long c = cm(m), a = c < 0 ? -c : c;
    fmt(buf, 16, "%s%ld.%02ld", c < 0 ? "-" : "", a / 100, a % 100);
    return buf;
}

static void frame(director_t *d, int must, const char *f, ...) __attribute__((format(printf, 3, 4)));
static void say(director_t *d, const char *f, ...) __attribute__((format(printf, 2, 3)));

static void frame(director_t *d, int must, const char *f, ...)
{
    /* The body is formatted straight into its place behind the '$', and only
     * the checksum is added after it.  (Formatting it into a buffer of its
     * own and then "$%s*%02X" copied every character a second time - a
     * tenth of a tick - and kept 112 more bytes on fly's stack.) */
    enum { BODY = 112 };
    char line[BODY + 12];
    va_list ap;
    va_start(ap, f);
    int n = fmt_v(line + 1, BODY, f, ap);
    va_end(ap);
    if (n <= 0 || n >= BODY) { return; }
    line[0] = '$';
    fmt(line + 1 + n, 8, "*%02X\r\n", proto_checksum(line + 1, (size_t)n));
    d->emit(line, must);
}

static void say(director_t *d, const char *f, ...)
{
    char line[124];
    va_list ap;
    va_start(ap, f);
    int n = fmt_v(line, sizeof line - 2, f, ap);
    va_end(ap);
    if (n < 0) { return; }
    if (n > (int)sizeof line - 3) { n = (int)sizeof line - 3; }
    line[n] = '\r';  line[n + 1] = '\n';  line[n + 2] = '\0';
    d->emit(line, 1);
}

static float dist2(v3_t a, v3_t b)                     /* squared: no root */
{
    float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

static float dist(v3_t a, v3_t b) { return t_sqrt(dist2(a, b)); }

/* A squared distance above this is surely farther than d, so the pair cannot
 * be the new closest - no square root needed to know (plan.c does the same) */
static float beyond(float d) { return d * d * 1.00002f; }

/* ---- set-up ---------------------------------------------------------------- */
void director_reset(director_t *d)
{
    const plan_t *p = &d->plan;
    if (d->state == DR_REFUSED) { return; }
    fleet_init(p->n, d->seed);
    for (int i = 0; i < p->n; i++) { fleet_place(i, p->spot[0][i].x, p->spot[0][i].y); }
    fleet_wind(d->wind, d->wind_deg * (F_PI / 180.0f));
    gps_grade_t g = d->rtk ? GPS_RTK : GPS_PLAIN;
    fleet_gps(g.wander, g.jitter);
    memset(d->sc, 0, sizeof d->sc);
    memset(d->hold_seen, 0, sizeof d->hold_seen);
    for (int k = 0; k < PLAN_STAGES; k++) { d->last_bad[k] = -1.0f; }
    d->closest = 1e9f;  d->closest_t = 0.0f;  d->ci = d->cj = 0;
    d->state = DR_WARMUP;
    d->warm = d->ticks = 0;
    d->paused = d->aborted = d->landing_sent = d->landed = d->passed = 0;
    d->aborted_at = -1.0f;
    d->rms = d->max_err = 0.0f;
}

/* plan_build()'s progress, as text: the station shows it while it waits */
static director_t *planning;

static void progress(int done, int total)
{
    say(planning, "planning: %d of %d moves checked", done, total);
}

int director_init(director_t *d, const show_t *s, emit_fn emit)
{
    memset(d, 0, sizeof *d);
    d->emit = emit;
    d->seed = 1u;
    d->stride = 1;
    d->wind = s->wind;  d->wind_deg = s->wind_deg;
    d->rtk = s->rtk;
    d->feedforward = 1;
    planning = d;
    if (plan_build(&d->plan, s, progress) != 0) {
        d->state = DR_REFUSED;
        say(d, "REFUSED before take-off: %s", d->plan.why);
        return -1;
    }
    d->state = DR_WARMUP;            /* so reset() runs */
    director_reset(d);
    return 0;
}

uint32_t director_show_ms(const director_t *d) { return (uint32_t)(plan_duration(&d->plan) * 1000.0f); }

/* ---- commands --------------------------------------------------------------- */
static void land_all(director_t *d)
{
    for (int i = 0; i < d->plan.n; i++) { fleet_land(i); }
}

static void arm_all(director_t *d)
{
    for (int i = 0; i < d->plan.n; i++) {
        int r = fleet_arm(i);
        if (r != 0) {
            say(d, "drone %d refused to arm (%s)", i, r == 2 ? "no GPS yet" : "not on the ground");
            return;
        }
    }
    d->state = DR_FLYING;
    d->ticks = 0;
    d->start_pending = 0;
    say(d, "show '%s': %d drones, seed %lu - go", d->plan.show->name, d->plan.n, (unsigned long)d->seed);
}

void director_start(director_t *d)
{
    switch (d->state) {
    case DR_READY:   arm_all(d);                                break;
    case DR_WARMUP:  d->start_pending = 1;                      break;   /* when ready */
    case DR_DONE:    director_reset(d);  d->start_pending = 1;  break;
    case DR_FLYING:
    case DR_LANDING: director_go(d);                            break;
    default:         say(d, "REFUSED: %s", d->plan.why);        break;
    }
}

void director_pause(director_t *d)
{
    if ((d->state == DR_FLYING || d->state == DR_LANDING) && !d->paused) {
        d->paused = 1;
        say(d, "paused - the show clock stands still; start or go carries on");
    }
}

void director_go(director_t *d)
{
    if (d->paused) { d->paused = 0;  say(d, "go"); }
}

void director_abort(director_t *d)
{
    if ((d->state != DR_FLYING && d->state != DR_LANDING) || d->aborted) { return; }
    d->aborted = 1;
    d->paused = 0;
    d->aborted_at = (float)d->ticks * TICK_S;
    d->state = DR_LANDING;
    land_all(d);
    say(d, "abort: every drone lands where it is");
}

void director_set_wind(director_t *d, float speed, float deg)
{
    d->wind = speed < 0.0f ? 0.0f : speed;
    d->wind_deg = deg;
    if (d->state != DR_REFUSED) { fleet_wind(d->wind, deg * (F_PI / 180.0f)); }
}

static int grounded_state(const director_t *d)
{
    return d->state == DR_WARMUP || d->state == DR_READY || d->state == DR_DONE;
}

int director_set_seed(director_t *d, uint32_t seed)
{
    if (!grounded_state(d)) { return -1; }
    d->seed = seed;
    director_reset(d);
    return 0;
}

int director_set_gps(director_t *d, int rtk)
{
    if (!grounded_state(d)) { return -1; }
    d->rtk = (uint8_t)(rtk != 0);
    director_reset(d);
    return 0;
}

int director_set_ff(director_t *d, int on)
{
    if (!grounded_state(d)) { return -1; }
    d->feedforward = (uint8_t)(on != 0);
    return 0;
}

/* ---- the score -------------------------------------------------------------- */
static void score(director_t *d, float t, int k, const stage_t *s, const v3_t *want, int following)
{
    const plan_t *p = &d->plan;
    v3_t at[FLEET_MAX];
    for (int i = 0; i < p->n; i++) { at[i] = fleet_where(i); }

    if (following && s->kind != ST_PADS && s->kind != ST_LAND) {
        float worst = 0.0f;
        for (int i = 0; i < p->n; i++) {
            float e = dist(at[i], want[i]);
            drone_score_t *c = &d->sc[i];
            c->sq += e * e;  c->n++;
            if (e > c->worst) { c->worst = e; }
            if (s->kind == ST_HOLD) {
                c->fsq += e * e;  c->fn++;
                if (e > c->fworst) { c->fworst = e; }
            }
            if (e > worst) { worst = e; }
        }
        if (s->kind == ST_HOLD) {
            d->hold_seen[k] = 1;
            if (worst > p->show->settle) { d->last_bad[k] = t; }
        }
    }
    float lim = beyond(d->closest);
    for (int i = 0; i < p->n; i++) {
        for (int j = i + 1; j < p->n; j++) {
            if (fleet_grounded(i) && fleet_grounded(j)) { continue; }
            float q = dist2(at[i], at[j]);
            if (q > lim) { continue; }
            float dd = t_sqrt(q);
            if (dd < d->closest) {
                d->closest = dd;  d->closest_t = t;  d->ci = (uint8_t)i;  d->cj = (uint8_t)j;
                lim = beyond(dd);
            }
        }
    }
}

static float rms_of(float sq, uint32_t n) { return n ? t_sqrt(sq / (float)n) : 0.0f; }

/* ---- frames ------------------------------------------------------------------
 * Ten ticks make one 100 ms report, and its lines are SPREAD over them:
 * tick 0 $SWM, ticks 1..n one $DRN each, tick n+1 $SEP.  All at once they
 * are ~390 B, more than uart.c's 256 B transmit ring holds - the later
 * drones' lines would be the ones that never fit.  One line a tick (~60 B)
 * always fits, and the UART sends it long before the next.
 * part < 0: every line now (the final picture, and the warm-up on the pads).
 * want: where the plan has each drone now, if the caller has it already
 * (the tick does) - or 0, and drn() works it out. */
static void swm(director_t *d, unsigned long t_ms, int k, const stage_t *s);
static void drn(director_t *d, float t, const stage_t *s, int i, const v3_t *want);
static void sep(director_t *d, unsigned long t_ms);
static void sco(director_t *d, int must);

/* Which line of the report this tick sends: 0 the $SWM, 1..n a $DRN, n + 1
 * the $SEP - or -1, none.  Played at real time (stride 1) a report is EVERY
 * ticks long and each tick has its line.  Played at N x real time the ticks
 * come N times as fast, and the UART - 11.5 KB a second, whatever the show
 * does - could not carry N reports' worth: lines would be dropped, the same
 * drones' lines every time.  So only every N-th report is sent, its lines N
 * ticks apart: in real time, exactly the picture rate of stride 1. */
static int report_part(const director_t *d, uint32_t tick)
{
    uint32_t n = d->stride ? d->stride : 1u, at = tick % (EVERY * n);
    return at % n ? -1 : (int)(at / n);
}

static void frames(director_t *d, float t, int k, const stage_t *s, int part, const v3_t *want)
{
    const plan_t *p = &d->plan;
    unsigned long t_ms = (unsigned long)(t * 1000.0f + 0.5f);
    if (part == 0 || part < 0) { swm(d, t_ms, k, s); }
    for (int i = 0; i < p->n; i++) {
        if (part == 1 + i || part < 0) { drn(d, t, s, i, want ? &want[i] : 0); }
    }
    if (part == 1 + p->n || part < 0) { sep(d, t_ms); }
}

static void swm(director_t *d, unsigned long t_ms, int k, const stage_t *s)
{
    const plan_t *p = &d->plan;
    char name[24];
    const char *kind = plan_kind_name(s->kind);
    if (d->aborted)              { fmt(name, sizeof name, "ABORTED");  kind = "land"; }
    else if (s->kind == ST_MOVE) { fmt(name, sizeof name, "to_%s", plan_stage_name(p, s)); }
    else                         { fmt(name, sizeof name, "%s", plan_stage_name(p, s)); }
    /* ...then the settings the station's menu shows: steady wind (cm/s, the
     * degrees it blows TOWARD), RTK GPS, feed-forward, and the state */
    frame(d, 0, "SWM,%lu,%d,%d,%d,%s,%s,%lu,%ld,%d,%d,%d,%s", t_ms, p->n, k, p->n_stages, name, kind,
          (unsigned long)director_show_ms(d), cm(d->wind), (int)d->wind_deg, d->rtk, d->feedforward,
          d->paused ? "PAUSED" : director_state_name(d->state));
}

static void drn(director_t *d, float t, const stage_t *s, int i, const v3_t *planned)
{
    const plan_t *p = &d->plan;
    fleet_drone_t v;
    v3_t want, vel, slot = plan_stage_end(p, s, i);
    fleet_view(i, &v);
    if (planned) { want = *planned; } else { plan_sample(p, s, i, t, &want, &vel); }
    frame(d, 0, "DRN,%d,%s,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld", i, pilot_mode_name(v.mode),
          cm(v.x), cm(v.y), cm(v.z), cm(want.x), cm(want.y), cm(want.z),
          cm(slot.x), cm(slot.y), cm(slot.z), (long)(v.charge + 0.5f),
          cm(v.air_x), cm(v.air_y));                 /* the air around THIS drone: wind + its gust */
}

static void sep(director_t *d, unsigned long t_ms)
{
    const plan_t *p = &d->plan;
    float near = 1e9f, lim = beyond(near);
    int ni = 0, nj = 0;
    for (int i = 0; i < p->n; i++) {
        for (int j = i + 1; j < p->n; j++) {
            if (fleet_grounded(i) && fleet_grounded(j)) { continue; }
            float q = dist2(fleet_where(i), fleet_where(j));
            if (q > lim) { continue; }
            float dd = t_sqrt(q);
            if (dd < near) { near = dd;  ni = i;  nj = j;  lim = beyond(dd); }
        }
    }
    if (near < 1e8f) {
        frame(d, 0, "SEP,%lu,%ld,%d,%d,%ld,%d,%d,%ld,%ld", t_ms, cm(near), ni, nj,
              cm(d->closest < 1e8f ? d->closest : near), d->ci, d->cj,
              cm(p->show->sep), cm(p->show->safe));
    }
}

static void finish(director_t *d, float t, int k, const stage_t *s)
{
    const plan_t *p = &d->plan;
    float sq = 0.0f;
    uint32_t n = 0;
    d->max_err = 0.0f;
    d->landed = 0;
    for (int i = 0; i < p->n; i++) {
        sq += d->sc[i].sq;  n += d->sc[i].n;
        if (d->sc[i].worst > d->max_err) { d->max_err = d->sc[i].worst; }
        if (fleet_mode(i) == PM_IDLE && fleet_grounded(i)) { d->landed++; }
    }
    d->rms = rms_of(sq, n);
    d->passed = (uint8_t)(!d->aborted && d->closest >= p->show->safe && d->landed == p->n);
    d->state = DR_DONE;
    frames(d, t, k, s, -1, 0);                          /* the final state: everyone down */
    sco(d, 1);
    director_report(d);
}

static void sco(director_t *d, int must)
{
    frame(d, must, "SCO,%ld,%ld,%ld,%d,%d,%d", cm(d->rms), cm(d->max_err),
          cm(d->closest < 1e8f ? d->closest : 0.0f), d->landed, d->plan.n, d->passed);
}

/* While nothing moves - READY on the pads, paused, DONE, REFUSED - the
 * director sends nothing of its own accord.  A station that connects THEN
 * (the usual order: start the simulator, then run.bat) would wait forever.
 * So Main.c calls this every 100 ms when idle, and the picture is sent
 * again, one line per call: a whole set every second.  A refusal is repeated
 * every 5 s, as text - that is all a refused show has to say. */
void director_heartbeat(director_t *d, uint32_t beat)
{
    const plan_t *p = &d->plan;
    if (d->state == DR_REFUSED) {
        if (beat % 50u == 0u) { say(d, "REFUSED before take-off: %s", p->why); }
        return;
    }
    if (d->quiet) { return; }
    float t = (d->state == DR_READY || d->state == DR_WARMUP) ? 0.0f : (float)d->ticks * TICK_S;
    int k;
    const stage_t *s = plan_stage(p, t, &k);
    int part = (int)(beat % EVERY);
    if (d->state == DR_DONE && part == 9) { sco(d, 0); }
    else { frames(d, t, k, s, part, 0); }
}

void director_report(director_t *d)
{
    const plan_t *p = &d->plan;
    const show_t *sh = p->show;
    char a[16], b[16], c[16], e[16], f[16];
    if (d->state != DR_DONE) { say(d, "no score yet: the show is %s", director_state_name(d->state)); return; }
    say(d, "  drone  track RMS  track max  form RMS  form max  battery  landing");
    for (int i = 0; i < p->n; i++) {
        fleet_drone_t v;
        fleet_view(i, &v);
        const drone_score_t *s = &d->sc[i];
        say(d, "  %5d  %7s m  %7s m  %6s m  %6s m  %5ld %%  %s m/s", i,
            fm(a, rms_of(s->sq, s->n)), fm(b, s->worst), fm(c, rms_of(s->fsq, s->fn)), fm(e, s->fworst),
            (long)(v.charge + 0.5f), fm(f, v.hardest_landing));
    }
    say(d, "    all  %7s m  %7s m", fm(a, d->rms), fm(b, d->max_err));
    say(d, "  settle (every drone within %s m of its slot, and staying there):", fm(a, sh->settle));
    for (int k = 0; k < p->n_stages; k++) {
        const stage_t *s = &p->st[k];
        if (!d->hold_seen[k]) { continue; }
        float last = d->last_bad[k];
        if (last < 0.0f)                  { say(d, "    %-10s 0.00 s", plan_stage_name(p, s)); }
        else if (last < s->t0 + s->T - 0.5f) { say(d, "    %-10s %s s", plan_stage_name(p, s), fm(a, last - s->t0 + 0.01f)); }
        else                              { say(d, "    %-10s NOT settled in its hold", plan_stage_name(p, s)); }
    }
    say(d, "  closest approach: drones %d and %d, %s m at t = %s s (safe %s m: %s; planned >= %s m)",
        d->ci, d->cj, fm(a, d->closest), fm(b, d->closest_t), fm(c, sh->safe),
        d->closest >= sh->safe ? "OK" : "TOO CLOSE", fm(e, sh->sep));
    say(d, "  landed: %d of %d", d->landed, p->n);
    if (d->aborted) { say(d, "  ABORTED at t = %s s: nothing after it is scored", fm(a, d->aborted_at)); }
    say(d, "  %s   (seed %lu, wind %s m/s, %s GPS%s)", d->passed ? "PASS" : "FAIL", (unsigned long)d->seed,
        fm(a, d->wind), d->rtk ? "RTK" : "plain", d->feedforward ? "" : ", no feed-forward");
}

void director_print_plan(director_t *d)
{
    const plan_t *p = &d->plan;
    char a[16], b[16], c[16];
    if (d->state == DR_REFUSED) { say(d, "REFUSED: %s", p->why); return; }
    say(d, "plan '%s': %d drones, %d stages, %s s; %d repair(s)", p->show->name, p->n, p->n_stages,
        fm(a, plan_duration(p)), p->repairs);
    for (int k = 0; k < p->n_stages; k++) {
        const stage_t *s = &p->st[k];
        char note[48] = "";
        if (s->kind == ST_MOVE) {
            for (int i = 0, used = 0; i < p->n && used < 40; i++) {
                float h = (float)p->lift[s->from][i] * 0.1f;
                if (h != 0.0f) {
                    used += fmt(note + used, sizeof note - (size_t)used, " %d %s %s m;", i,
                                     h > 0.0f ? "over" : "under", fm(c, f_abs(h)));
                }
            }
        }
        say(d, "  %2d %-7s %-8s %7s %6s%s", k, plan_kind_name(s->kind), plan_stage_name(p, s),
            fm(a, s->t0), fm(b, s->T), note);
    }
    say(d, "  closest planned pass: drones %d and %d, %s m (sep %s m)", p->ci, p->cj,
        fm(a, p->closest), fm(b, p->show->sep));
}

/* ---- the tick ------------------------------------------------------------------- */
int director_tick(director_t *d)
{
    const plan_t *p = &d->plan;
    if (d->state == DR_WARMUP) {
        fleet_step();
        d->warm++;
        int part = report_part(d, d->warm);                         /* one line a tick */
        if (!d->quiet && part >= 0) { frames(d, 0.0f, 0, &p->st[0], part, 0); }
        if (d->warm >= WARM_TICKS) {
            d->state = DR_READY;
            if (d->start_pending) { arm_all(d); }
            else { say(d, "ready: press A (key a) or type start"); }
        }
        return 1;
    }
    if ((d->state != DR_FLYING && d->state != DR_LANDING) || d->paused) { return 0; }

    float t = (float)d->ticks * TICK_S;
    int k;
    const stage_t *s = plan_stage(p, t, &k);
    v3_t want[FLEET_MAX], vel[FLEET_MAX];
    plan_sample_all(p, s, t, want, vel);

    int following = !(d->landing_sent || d->aborted);
    if (d->ticks % EVERY == 0u && following) {
        if (s->kind == ST_LAND) {
            land_all(d);
            d->landing_sent = 1;
            d->state = DR_LANDING;
        } else {
            for (int i = 0; i < p->n; i++) {
                fleet_target(i, want[i], d->feedforward ? vel[i] : (v3_t){ 0 });
            }
        }
    }
    fleet_step();
    d->ticks++;
    score(d, t, k, s, want, !(d->landing_sent || d->aborted));
    int part = report_part(d, d->ticks);
    if (!d->quiet && part >= 0) { frames(d, t, k, s, part, want); }

    int down = 1;
    for (int i = 0; i < p->n; i++) { if (fleet_mode(i) != PM_IDLE) { down = 0; } }
    if (((d->landing_sent || d->aborted) && down) || t > plan_duration(p) + GIVE_UP_S) {
        finish(d, t, k, s);
    }
    return 1;
}
