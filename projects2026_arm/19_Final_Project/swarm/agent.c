/*
 * agent.c - one drone of the swarm.  See agent.h.
 */
#include <stdarg.h>
#include <string.h>
#include "agent.h"
#include "../fmt.h"
#include "proto.h"
#include "../trig.h"

#define PHY_EVERY_MS    200u      /* telemetry: 5 Hz is plenty for the air and the screen */
#define NBT_EVERY_MS    1000u
#define EXP_EVERY_MS    100u
#define STEER_EVERY_MS  50u       /* the local rules, 20 times a second        */
#define THINK_EVERY_MS  250u      /* the job: what to do, where to go          */
#define GOSSIP_MS       1000u     /* $TSK and $MAP                             */
#define MSN_EVERY_MS    2000u     /* the mission, relayed again                */
#define HOP_EVERY_MS    500u
#define GND_FRESH_MS    3000u
#define ROUTE_FRESH_MS  4000u     /* a route whose ground number stopped rising is dead */
#define KEY_HOLD_MS     600u
#define SPOT_LEAD       1.0f      /* m: the setpoint never runs further ahead  */
#define CLIMB_Z         2.5f      /* m: below this, climb first                */
#define RECHARGE        1.0f      /* %/s on the pad (a battery swap, really)   */
#define SPOT_LIMIT      200.0f

static const char *const state_text[AG_COUNT] = { "NO-ID", "GROUND", "FLYING", "LANDING", "HOMING", "DEAD" };
static const char *const type_text[TY_COUNT]  = { "mapper", "sensor", "relay" };
static const char *const kind_text[MS_COUNT]  = { "HOLD", "GOTO", "SURVEY", "HOME" };
static const char WHY_DONE[] = "mission complete", WHY_ORDER[] = "ordered home",
                  WHY_BAT[]  = "battery low",      WHY_LOST[]  = "lost link",
                  WHY_ROUTE[] = "no route to the ground radio";

const char *agent_state_name(uint8_t s) { return s < AG_COUNT ? state_text[s] : "?"; }
const char *agent_type_name(uint8_t t)  { return t < TY_COUNT ? type_text[t] : "?"; }

/* ---- output --------------------------------------------------------------- */
static void say(agent_t *a, const char *f, ...) __attribute__((format(printf, 2, 3)));
static void say(agent_t *a, const char *f, ...)
{
    char buf[112];
    va_list ap;
    va_start(ap, f);
    fmt_v(buf, sizeof buf, f, ap);
    va_end(ap);
    a->emit(buf, 1);
}

static void frame(agent_t *a, int must, const char *f, ...) __attribute__((format(printf, 3, 4)));
static void frame(agent_t *a, int must, const char *f, ...)
{
    char line[RADIO_LINE + 8];                 /* "$" body "*CS\n", built in place */
    va_list ap;
    va_start(ap, f);
    int n = fmt_v(line + 1, RADIO_LINE - 5u, f, ap);
    va_end(ap);
    line[0] = '$';
    fmt(line + 1 + n, 6, "*%02X\n", (unsigned)proto_checksum(line + 1, (size_t)n));
    a->emit(line, must);
}

static int32_t cm(float m) { return (int32_t)(m * 100.0f + (m < 0.0f ? -0.5f : 0.5f)); }
static int32_t dm(float m) { return (int32_t)(m * 10.0f + (m < 0.0f ? -0.5f : 0.5f)); }

static const char *metres(char *buf, float m)
{
    int32_t c = cm(m);
    const char *sign = c < 0 ? "-" : "";
    if (c < 0) { c = -c; }
    fmt(buf, 16, "%s%ld.%02ld", sign, (long)(c / 100), (long)(c % 100));
    return buf;
}

static uint32_t roll(agent_t *a)
{
    uint32_t x = a->dice;
    x ^= x << 13;  x ^= x >> 17;  x ^= x << 5;
    return a->dice = x;
}

static void tasks_news(agent_t *a);

static float flat(v3_t p, v3_t q) { float dx = p.x - q.x, dy = p.y - q.y; return t_sqrt(dx * dx + dy * dy); }

static int due(uint32_t now, uint32_t *at, uint32_t every)
{
    if ((int32_t)(now - *at) < 0) { return 0; }
    *at += every;
    if ((int32_t)(now - *at) >= 0) { *at = now + every; }   /* far behind: no rush */
    return 1;
}

/* ---- life ------------------------------------------------------------------ */
void agent_init(agent_t *a, uint8_t id, uint8_t type, emit_fn emit)
{
    memset(a, 0, sizeof *a);
    a->emit = emit;
    a->id = id;
    a->type = type < TY_COUNT ? type : TY_MAPPER;
    a->hop = HOP_NONE;
    a->closest = 99.0f;
    nbr_init(&a->nb);
    tasks_reset(&a->tk, 0, 0);
    if (id < 1u || id > RADIO_MAX_ID) {
        a->state = AG_NOID;
        say(a, "no drone ID: strap PB0/PA7/PA15 to GND for bits 0/1/2 (diagram.json)\n");
        return;
    }
    a->state = AG_GROUND;
    a->dice  = 0x9E3779B9u * id + 0x7F4A7C15u;
    a->home  = (v3_t){ PAD_GAP * (float)(id - 1u), 0.0f, 0.0f };
    a->cruise = HOVER_Z + LANE_GAP * (float)((id - 1u) % 3u);
    craft_init(&a->body, a->dice ^ 0xA5A5A5A5u, a->home.x, a->home.y);
    a->body.gps = GPS_RTK;
    pilot_init(&a->brain);
    craft_sky(&a->sky, 0.0f, 0.0f);
    /* every schedule starts at this drone's own random moment: drones switched
     * on together must not all talk at once, for ever */
    a->next_bcn   = roll(a) % BCN_EVERY_MS;
    a->next_tsk   = roll(a) % GOSSIP_MS;
    a->next_map   = roll(a) % GOSSIP_MS;
    a->next_msn   = roll(a) % MSN_EVERY_MS;
    a->next_think = roll(a) % THINK_EVERY_MS;
    a->next_hop   = roll(a) % HOP_EVERY_MS;
    a->next_phy   = 5u;
    a->next_nbt   = NBT_EVERY_MS;
    char b[16];
    say(a, "drone %u (%s) up: pad at x = %s m\n", (unsigned)id, agent_type_name(a->type), metres(b, a->home.x));
}

static void go_home(agent_t *a, const char *why)
{
    if (a->state != AG_FLYING) { return; }
    tasks_release(&a->tk, a->id, a->now_ms);
    tasks_news(a);
    a->at_task_ms = 0;
    a->state = AG_HOMING;
    a->why = why;
    say(a, "going home: %s\n", why);
}

static void complete(agent_t *a)
{
    a->finished_mid = a->m.mid;
    a->finished_any = 1;
    a->done_m = 1;
    go_home(a, WHY_DONE);
}

void agent_takeoff(agent_t *a)
{
    if (a->state != AG_GROUND) { return; }
    int r = pilot_arm(&a->brain);
    if (r == PILOT_E_GPS) { say(a, "not yet: waiting for GPS\n"); return; }
    if (r != PILOT_OK)    { say(a, "cannot arm (%d)\n", r); return; }
    v3_t e = a->brain.est;
    a->spot = (v3_t){ e.x, e.y, a->cruise };
    a->hold = a->spot;
    a->vcmd = (v3_t){ 0 };
    a->has_goal = 0;
    pilot_goal(&a->brain, a->spot, (v3_t){ 0 });
    a->state = AG_FLYING;
    a->why = 0;
    a->route_ms = a->now_ms;
    a->relay_f = 0.2f;
    a->parked = 0;
    a->cut_ms = 0;
    a->falling_back = 0;
    say(a, "take-off\n");
}

/* Landing BY HAND (button B, `down`, the station's b) is an order: the drone
 * stays down - it does not rejoin the mission when charged - until it is
 * taken off by hand, or a NEW mission is posted (a follow goal that only
 * moves is not new). */
void agent_land(agent_t *a)
{
    if (a->state != AG_FLYING && a->state != AG_HOMING) { return; }
    tasks_release(&a->tk, a->id, a->now_ms);
    pilot_land(&a->brain);
    a->state = AG_LANDING;
    a->parked = 1;
    say(a, "landing where it is; staying down until told\n");
}

void agent_kill(agent_t *a)
{
    if (a->state == AG_NOID || a->state == AG_DEAD) { return; }
    a->state = AG_DEAD;
    a->asked = (cmd_t){ 0 };
    say(a, "KILLED: motors off, radio dead\n");
}

void agent_stick(agent_t *a, float vx, float vy) { a->stick_x = vx;  a->stick_y = vy; }

void agent_breeze(agent_t *a, float ms)
{
    a->breeze = ms < 0.0f ? 0.0f : ms;
    craft_sky(&a->sky, a->breeze, 0.0f);
}

/* ---- the mission -------------------------------------------------------------- */
int msn_encode(const mission_t *m, char *buf, int size)
{
    int n = fmt(buf, (size_t)size, "MSN,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u", (unsigned)m->mid, (unsigned)m->kind,
                (long)dm(m->gx), (long)dm(m->gy), (long)dm(m->x0), (long)dm(m->y0), (long)dm(m->x1),
                (long)dm(m->y1), (unsigned)m->np);
    for (unsigned i = 0; i < m->np && n < size - 16; i++) {
        n += fmt(buf + n, (size_t)(size - n), ",%ld,%ld", (long)dm(m->pt[2 * i]), (long)dm(m->pt[2 * i + 1]));
    }
    return n;
}

int msn_decode(const char *line, mission_t *m)
{
    int32_t f[9 + 2 * MSN_POINTS];
    int n = frame_ints(line, "MSN", f, 9 + 2 * MSN_POINTS);
    if (n < 9 || f[0] < 0 || f[0] > 65535 || f[1] < 0 || f[1] >= MS_COUNT ||
        f[8] < 0 || f[8] > (int32_t)MSN_POINTS || n != 9 + 2 * f[8]) {
        return -1;
    }
    memset(m, 0, sizeof *m);
    m->mid = (uint16_t)f[0];  m->kind = (uint8_t)f[1];  m->np = (uint8_t)f[8];
    m->gx = (float)f[2] * 0.1f;  m->gy = (float)f[3] * 0.1f;
    m->x0 = (float)f[4] * 0.1f;  m->y0 = (float)f[5] * 0.1f;
    m->x1 = (float)f[6] * 0.1f;  m->y1 = (float)f[7] * 0.1f;
    for (int i = 0; i < 2 * m->np; i++) { m->pt[i] = (float)f[9 + i] * 0.1f; }
    return 0;
}

static void adopt(agent_t *a, const mission_t *m)
{
    /* m_ms times the staggered take-off (id x 0.3 s after a mission appears).
     * A GOTO that only MOVES the goal - the station's follow mode sends one
     * every 0.4 s - is the same job: restarting the wait each time would keep
     * every drone but the first on its pad for ever. */
    int moved_goal = a->has_m && a->m.kind == MS_GOTO && m->kind == MS_GOTO;
    a->m = *m;
    a->has_m = 1;
    a->done_m = 0;
    if (!moved_goal) {
        a->m_ms = a->now_ms;
        a->parked = 0;                                   /* a new order */
        a->edge = 0.0f;                                  /* a new job: learn its edge anew */
        a->probe_ms = 4000u;
    }
    a->at_task_ms = 0;
    a->route_trips = 0;
    if (m->kind == MS_SURVEY) {
        cover_init(&a->cv, m->x0, m->y0, m->x1, m->y1);
        tasks_reset(&a->tk, m->np, m->pt);
    } else {
        cover_init(&a->cv, 0, 0, 0, 0);
        tasks_reset(&a->tk, 0, 0);
    }
    /* relay it soon - but not at the same moment as every other drone that
     * heard the same copy */
    a->next_msn = a->now_ms + 20u + roll(a) % 120u;
    say(a, "mission %u: %s\n", (unsigned)m->mid, kind_text[m->kind]);
    if (a->state == AG_HOMING && (a->why == WHY_DONE || a->why == WHY_ORDER || a->why == WHY_ROUTE) &&
        m->kind != MS_HOME) {
        a->state = AG_FLYING;                            /* new work: turn round */
        a->why = 0;
        a->route_ms = a->now_ms;
    }
    if (m->kind == MS_HOLD && a->state == AG_FLYING) { a->hold = a->brain.est;  a->hold.z = a->cruise; }
    if (m->kind == MS_HOME) { go_home(a, WHY_ORDER); }
}

/* RELAY: stand on the line from the ground radio to the work, as far out as
 * the link behind me allows.  No drone knows the radio range, so a relay
 * finds the edge by trying, from the quality of ONE link: the one to the
 * radio just behind it in the chain.  The first relay (lowest id among the
 * relays it hears) leans on the ground radio; the k-th on the relay ranked
 * k-1, and keeps beyond it.  Link strong (>= 92 %): move out; fair (>= 75 %):
 * stay; weak or gone for 1.5 s: move back.  The bar is high on purpose: a link
 * kept out of the fading edge of its range does not flicker, and a chain of
 * flickering links drops the route home of everyone beyond it.  The chain
 * sorts itself out. */
static v3_t relay_spot(agent_t *a)
{
    float cx = a->m.x0, cy = a->m.y0;
    if (a->m.kind == MS_SURVEY) { cx = 0.5f * (a->m.x0 + a->m.x1);  cy = 0.5f * (a->m.y0 + a->m.y1); }
    v3_t g = { a->m.gx, a->m.gy, 0.0f };
    const nbr_t *anchor = 0;                       /* the relay just before me */
    for (unsigned i = 0; i < a->nb.n; i++) {
        const nbr_t *o = &a->nb.e[i];
        if (o->type == TY_RELAY && o->mode == AG_FLYING && o->id < a->id && (!anchor || o->id > anchor->id)) {
            anchor = o;
        }
    }
    unsigned up;
    int behind = 0;
    if (!anchor) {
        up = a->gnd_ms && a->now_ms - a->gnd_ms < GND_FRESH_MS ? (a->gnd_qf * 100u + 32767u) / 65535u : 0u;
    } else {
        up = anchor->q;
        behind = flat(a->brain.est, g) < flat(anchor->pos, g) + 3.0f;   /* not yet past it */
    }
    if (behind || up >= 92u) {                                 /* strong: further out */
        a->relay_f += 0.004f;
        a->relay_bad_ms = 0;
    } else if (up >= 75u) {                                    /* fair: stay          */
        a->relay_bad_ms = 0;
    } else if (!a->relay_bad_ms) {
        a->relay_bad_ms = a->now_ms | 1u;
    } else if (a->now_ms - a->relay_bad_ms > 1500u) {           /* weak for 1.5 s: back */
        a->relay_f -= 0.006f;
    }
    if (a->relay_f > 0.9f)  { a->relay_f = 0.9f; }
    if (a->relay_f < 0.05f) { a->relay_f = 0.05f; }
    float f = a->relay_f;
    return (v3_t){ g.x + (cx - g.x) * f, g.y + (cy - g.y) * f, HOVER_Z };
}

/* A fingerprint of my copy of the auction: winners, bids, done flags. */
static uint32_t tasks_print(const tasks_t *k)
{
    uint32_t h = 2166136261u;
    for (unsigned i = 0; i < k->n; i++) {
        h = (h ^ k->t[i].winner) * 16777619u;
        h = (h ^ k->t[i].bid) * 16777619u;
        h = (h ^ k->t[i].done) * 16777619u;
    }
    return h;
}

/* My copy changed: tell the neighbours now, not at my next turn - news of a
 * claim or a finished task then crosses the swarm in tenths of a second per
 * hop, not a second per hop, and two drones rarely do the same job. */
static void tasks_news(agent_t *a)
{
    uint32_t soon = a->now_ms + 40u + roll(a) % 120u;
    if ((int32_t)(a->next_tsk - soon) > 0) { a->next_tsk = soon; }
}

/* Is the column above my pad clear?  No airborne neighbour within CLEAR_R
 * sideways - a drone going up or down cannot dodge (the pilot flies a
 * landing straight down), so it waits until the others have passed. */
static int clear(const agent_t *a)
{
    for (unsigned i = 0; i < a->nb.n; i++) {
        const nbr_t *o = &a->nb.e[i];
        if (o->pos.z > 0.8f && flat(o->pos, a->brain.est) < CLEAR_R) { return 0; }
    }
    return 1;
}

/* GOTO: go to the goal - but stay where the ground radio's word still
 * reaches.  A drone out of touch cannot hear a newer goal; a swarm that
 * followed a goal out of reach would stop there and never hear the goal come
 * back.  No drone knows the radio range, so each one LEARNS the edge:
 *   - when the newest ground beacon number it knows of has not risen for
 *     CUT_MS, it is out of touch: it flies back toward the ground radio,
 *     and KEEPS going in until it is in touch again;
 *   - where it gets back in touch is the edge (less 1 m) - wherever that is,
 *     nearer or further than before: the network may have grown or shrunk.
 *     From then on a goal beyond the edge is held AT the edge, on the line;
 *   - every probe_ms in touch it tries 1 m further: relays, or a better
 *     radio, may have made room.  probe_ms shrinks by a quarter after a
 *     probe that kept it in touch (down to 2 s - following relays
 *     out as they extend the network) and DOUBLES when a probe fails - touch
 *     lost within 1.5 m of the edge it was probing (up to 32 s - a goal that
 *     is simply out of reach is tried rarely).  A loss further out, where
 *     the network was a moment ago, says nothing about probing.
 *     The rule TCP and the station's pacing use, for the same reason.
 * Judged by the ground beacon number alone: the hop count is only worked out
 * every 0.5 s, and just after take-off it may not be yet.  A drone that has
 * never heard a number cannot know where the edge is. */
static v3_t goto_spot(agent_t *a)
{
    uint32_t now = a->now_ms;
    v3_t e = a->brain.est, base = { a->m.gx, a->m.gy, 0.0f }, g = { a->m.x0, a->m.y0, HOVER_Z };
    float d = flat(e, base);
    char m1[16], m2[16];
    if (a->gs_ms && now - a->gs_ms > CUT_MS) {            /* out of touch */
        if (!a->cut_ms) {
            a->cut_ms = now | 1u;
            a->cut_d = d;
            if (a->edge > 0.0f && d <= a->edge + 1.5f) {          /* a probe failed: slower */
                a->probe_ms = a->probe_ms * 2u > PROBE_MAX_MS ? PROBE_MAX_MS : a->probe_ms * 2u;
            }
            say(a, "out of the ground radio's reach %s m out: falling back until I hear it\n", metres(m1, d));
        }
        a->falling_back = 1;
        a->edge_ms = now;
        if (d < 0.5f) { return (v3_t){ base.x, base.y, HOVER_Z }; }
        float to = d - 5.0f < 2.0f ? 2.0f : d - 5.0f;     /* always 5 m further in than I am */
        return (v3_t){ base.x + (e.x - base.x) * to / d, base.y + (e.y - base.y) * to / d, HOVER_Z };
    }
    a->falling_back = 0;
    if (a->cut_ms) {                                      /* back in touch: here is the edge */
        a->edge = d - 1.0f < 3.0f ? 3.0f : d - 1.0f;
        say(a, "in reach again %s m out: holding goals within %s m\n", metres(m1, d), metres(m2, a->edge));
        a->cut_ms = 0;
        a->edge_ms = now;
    }
    if (a->edge > 0.0f) {
        if (now - a->edge_ms > a->probe_ms) {
            a->edge += 1.0f;
            a->edge_ms = now;
            a->probe_ms = a->probe_ms * 3u / 4u < PROBE_MIN_MS ? PROBE_MIN_MS : a->probe_ms * 3u / 4u;
        }
        float dg = flat(g, base);
        if (dg > a->edge) {                               /* hold it at the edge */
            float k = a->edge / dg;
            g.x = base.x + (g.x - base.x) * k;  g.y = base.y + (g.y - base.y) * k;
        }
    }
    return g;
}

/* Is my part of the mission done, as far as I know?  A mapper's part is the
 * map; a sensor's the tasks; a relay stays until both are, so the others
 * keep their link home until the end. */
static int my_part_done(const agent_t *a)
{
    if (a->m.kind != MS_SURVEY) { return 0; }
    if (a->type == TY_MAPPER) { return cover_done(&a->cv); }
    if (a->type == TY_SENSOR) { return tasks_all_done(&a->tk); }
    return cover_done(&a->cv) && tasks_all_done(&a->tk);
}

/* ---- deciding: 4 times a second -------------------------------------------- */
static void think(agent_t *a)
{
    uint32_t now = a->now_ms;
    float bat = a->body.charge;
    v3_t e = a->brain.est;
    int working = a->has_m && (a->m.kind == MS_GOTO || a->m.kind == MS_SURVEY);

    if (a->has_m && a->tk.n) { tasks_expire(&a->tk, a->id, now); }

    if (a->state == AG_GROUND) {
        if (working && my_part_done(a) && !a->done_m) {   /* the gossip says it is over */
            a->finished_mid = a->m.mid;  a->finished_any = 1;  a->done_m = 1;
        }
        int finished = a->finished_any && a->finished_mid == a->m.mid;
        /* back into the mission once charged; drones leave their pads one
         * after another (by id), not all in the same instant */
        if (working && !finished && !a->parked && bat >= BAT_GO && a->body.grounded && now - a->heard_ms < 5000u &&
            now - a->m_ms > (uint32_t)a->id * 300u && a->brain.fixes >= 10u && clear(a)) {
            say(a, "joining mission %u\n", (unsigned)a->m.mid);
            agent_takeoff(a);
        }
        return;
    }
    if (a->state == AG_HOMING) {
        /* sent home by a lost link or route, and now back in touch: work again */
        if ((a->why == WHY_LOST && now - a->heard_ms < 1000u) ||
            (a->why == WHY_ROUTE && a->hop != HOP_NONE)) {
            if (working && bat >= BAT_HOME + 5.0f && !(a->finished_any && a->finished_mid == a->m.mid)) {
                a->state = AG_FLYING;
                a->why = 0;
                a->route_ms = now;
                say(a, "back in touch: resuming\n");
                return;
            }
        }
        a->goal = a->home;  a->has_goal = 1;
        if (flat(e, a->home) < 0.6f && flat(a->brain.est_v, (v3_t){ 0 }) < 0.6f && clear(a)) {
            /* the pilot lands straight down over its goal: make that the pad
             * itself, not the setpoint, which may lead it by a metre */
            pilot_goal(&a->brain, (v3_t){ a->home.x, a->home.y, e.z }, (v3_t){ 0 });
            pilot_land(&a->brain);
            a->state = AG_LANDING;
            say(a, "over my pad: landing\n");
        }
        return;
    }
    if (a->state != AG_FLYING) { return; }

    /* ---- my own reasons to stop ---- */
    if (bat < BAT_HOME)                                    { go_home(a, WHY_BAT);  return; }
    if (working && now - a->heard_ms > LOST_MS)            { go_home(a, WHY_LOST); return; }
    if (working && now - a->route_ms > NOROUTE_MS) {
        /* the second time in one mission: the goal is out of radio reach -
         * stop trying (until a new mission), rather than yo-yo for ever */
        if (++a->route_trips >= 2u) {
            a->finished_mid = a->m.mid;  a->finished_any = 1;
            say(a, "giving up mission %u: out of radio reach\n", (unsigned)a->m.mid);
        }
        go_home(a, WHY_ROUTE);
        return;
    }
    if (a->has_m && a->m.kind == MS_HOME)                  { go_home(a, WHY_ORDER); return; }

    a->has_goal = 1;
    if (!working) { a->goal = a->hold;  return; }          /* no mission, or HOLD */
    if (a->type == TY_RELAY) {
        a->goal = relay_spot(a);
        if (my_part_done(a)) { complete(a); }
        return;
    }
    if (a->m.kind == MS_GOTO) { a->goal = goto_spot(a);  return; }

    /* SURVEY */
    if (a->type == TY_MAPPER) {
        if (my_part_done(a)) { complete(a);  return; }
        int cell = cover_choose(&a->cv, a->id, e, &a->nb, TY_MAPPER);
        a->goal = cover_centre(&a->cv, cell, HOVER_Z);
        return;
    }
    /* SENSOR */
    uint32_t before = tasks_print(&a->tk);
    int k = tasks_bid(&a->tk, a->id, e, bat, now);
    if (tasks_print(&a->tk) != before) { tasks_news(a); }
    if (k < 0) {
        if (my_part_done(a)) { complete(a);  return; }
        /* nothing for me: wait WHERE I AM - not on the point I just lost,
         * where I would stand in the winner's way */
        if (a->tasking) { a->hold = (v3_t){ e.x, e.y, a->cruise };  a->tasking = 0; }
        a->goal = a->hold;
        a->at_task_ms = 0;
        return;
    }
    a->tasking = 1;
    const task_t *t = &a->tk.t[k];
    a->goal = (v3_t){ t->x, t->y, HOVER_Z };
    if (flat(e, a->goal) < TASK_AT_M) {
        if (!a->at_task_ms) { a->at_task_ms = now | 1u; }
        else if (now - a->at_task_ms >= TASK_HOLD_MS) {
            tasks_finish(&a->tk, k);
            tasks_news(a);
            a->sensed++;
            a->at_task_ms = 0;
            say(a, "task %d done: reading %u\n", k + 1, (unsigned)(180u + roll(a) % 60u));
        }
    } else {
        a->at_task_ms = 0;
    }
}

/* ---- moving: 20 times a second -------------------------------------------------- */
static void move(agent_t *a)
{
    const pilot_t *p = &a->brain;
    steer_in_t in = {
        .me = p->est, .vel = p->est_v, .goal = a->goal, .vmax = STEER_VMAX,
        .has_goal = a->has_goal && p->est.z > CLIMB_Z,
    };
    if (a->state == AG_FLYING && a->has_m && a->m.kind == MS_GOTO && a->type != TY_RELAY && !a->falling_back) {
        in.k_coh = 0.12f;  in.k_ali = 0.3f;                /* a flock, not a queue */
        in.slack = STEER_SLACK;
    }
    /* falling back into reach: straight to the point, no slack, no flock -
     * stopping 3 m short of it, with the others, would leave it out of reach */
    steer_out_t out;
    steer(&in, &a->nb, a->now_ms, &out);
    a->vcmd.x += 0.5f * (out.v.x - a->vcmd.x);
    a->vcmd.y += 0.5f * (out.v.y - a->vcmd.y);
    a->avoiding = out.avoiding;
    if (out.avoiding) { a->avoids++; }
    if (p->est.z > CLIMB_Z && out.closest < a->closest) { a->closest = out.closest; }
    if (a->type == TY_MAPPER && a->has_m && a->m.kind == MS_SURVEY && p->est.z > CLIMB_Z) {
        if (cover_see(&a->cv, p->est)) { a->covered++; }
    }
}

static void hops(agent_t *a)
{
    uint32_t now = a->now_ms;
    if (a->gnd_ms && now - a->gnd_ms < GND_FRESH_MS) { a->hop = 1;  a->via = 0;  a->route_ms = now;  return; }
    /* through a neighbour: the one with the NEWEST ground number, then the
     * fewest hops.  A newer number than mine is news: take it. */
    const nbr_t *via = 0;
    for (unsigned i = 0; i < a->nb.n; i++) {
        const nbr_t *o = &a->nb.e[i];
        if (now - o->heard_ms >= 1000u || o->hop >= 15u) { continue; }
        if (!via || (int16_t)(uint16_t)(o->gs - via->gs) > 0 || (o->gs == via->gs && o->hop < via->hop)) { via = o; }
    }
    if (via && (int16_t)(uint16_t)(via->gs - a->gs) > 0) { a->gs = via->gs;  a->gs_ms = now; }
    if (via && via->gs == a->gs && a->gs_ms && now - a->gs_ms < ROUTE_FRESH_MS) {
        a->hop = (uint8_t)(via->hop + 1u);
        a->via = via->id;
        a->route_ms = now;
    } else {
        a->hop = HOP_NONE;                                 /* stale, or nobody: no route */
        a->via = 0;
    }
}

/* ---- the radio, out ----------------------------------------------------------------- */
static void beacon(agent_t *a)
{
    const pilot_t *p = &a->brain;
    if (p->fixes == 0u) { return; }
    float bat = a->body.charge;
    v3_t g = a->has_goal && (a->state == AG_FLYING || a->state == AG_HOMING) ? a->goal : p->est;
    bcn_t b = {
        .id = a->id, .seq = a->seq++, .mode = a->state, .nn = a->nb.n, .type = a->type, .hop = a->hop,
        .bat = (uint8_t)(bat < 0.0f ? 0.0f : bat > 100.0f ? 100.0f : bat),
        .x = cm(p->est.x), .y = cm(p->est.y), .z = cm(p->est.z),
        .vx = cm(p->est_v.x), .vy = cm(p->est_v.y), .vz = cm(p->est_v.z),
        .gx = dm(g.x), .gy = dm(g.y), .gs = a->gs,
    };
    char line[RADIO_LINE];
    bcn_encode(&b, line, sizeof line);
    a->emit(line, 1);                                     /* radio: never skipped */
    a->tx++;
}

static void gossip_tasks(agent_t *a)
{
    tasks_refresh(&a->tk, a->id, a->brain.est, a->body.charge, a->now_ms);
    char body[RADIO_LINE];
    int n = fmt(body, sizeof body, "TSK,%u,%u,%u", (unsigned)a->id, (unsigned)a->m.mid, (unsigned)a->tk.n);
    for (unsigned i = 0; i < a->tk.n; i++) {
        const task_t *t = &a->tk.t[i];
        n += fmt(body + n, sizeof body - (size_t)n, ",%u,%u,%u,%u,%u", (unsigned)t->winner, (unsigned)t->bid,
                 (unsigned)t->stamp, (unsigned)tasks_age_ds(t, a->now_ms), (unsigned)t->done);
    }
    frame(a, 1, "%s", body);
}

static void gossip_map(agent_t *a)
{
    frame(a, 1, "MAP,%u,%u,%lu,%lu,%lu,%lu", (unsigned)a->id, (unsigned)a->m.mid,
          (unsigned long)(a->cv.hi >> 16), (unsigned long)(a->cv.hi & 0xFFFFu),
          (unsigned long)(a->cv.lo >> 16), (unsigned long)(a->cv.lo & 0xFFFFu));
}

static void table(agent_t *a)
{
    char body[RADIO_LINE];
    int n = fmt(body, sizeof body, "NBT,%u,%u", (unsigned)a->id, (unsigned)a->nb.n);
    for (unsigned i = 0; i < a->nb.n && n < (int)sizeof body - 10; i++) {
        n += fmt(body + n, sizeof body - (size_t)n, ",%u,%u", (unsigned)a->nb.e[i].id, (unsigned)a->nb.e[i].q);
    }
    frame(a, 0, "%s", body);
}

static void physics(agent_t *a)
{
    const craft_t *c = &a->body;
    frame(a, 0, "PHY,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u", (unsigned)a->id,
          (long)cm(c->pos.x), (long)cm(c->pos.y), (long)cm(c->pos.z),
          (long)cm(c->vel.x), (long)cm(c->vel.y), (long)cm(c->vel.z), (unsigned)a->state, (unsigned)a->type);
}

/* ---- the tick ------------------------------------------------------------------------- */
void agent_tick(agent_t *a)
{
    a->ticks++;
    a->now_ms = a->ticks * AGENT_TICK_MS;
    uint32_t now = a->now_ms;
    if (a->state == AG_NOID) {
        if (now % 5000u == 0u) { say(a, "no drone ID - not flying, not talking\n"); }
        return;
    }
    if (a->state == AG_DEAD) {                           /* it falls; it says nothing */
        craft_step(&a->body, &a->sky, &a->asked);
        if (due(now, &a->next_phy, PHY_EVERY_MS)) { physics(a); }
        return;
    }

    /* ---- fly: the pilot every second tick, the airframe every tick ---- */
    if (a->ticks % (AIR_HZ / PILOT_HZ) == 0u) {
        meas_t m;
        craft_sense(&a->body, &m);
        pilot_step(&a->brain, &m, &a->asked);
    }
    craft_step(&a->body, &a->sky, &a->asked);

    if ((a->key_x != 0.0f || a->key_y != 0.0f) && now - a->key_ms > KEY_HOLD_MS) { a->key_x = a->key_y = 0.0f; }
    if (a->state == AG_GROUND && a->body.grounded && a->body.charge < 100.0f) {
        a->body.charge += RECHARGE * (AGENT_TICK_MS * 0.001f);
    }

    if (a->state == AG_FLYING || a->state == AG_HOMING) {
        if (a->brain.mode == PM_LAND) {                    /* the pilot's own guard: battery empty */
            tasks_release(&a->tk, a->id, now);
            a->state = AG_LANDING;
            say(a, "landing NOW: battery empty\n");
        } else {
            float dt = AGENT_TICK_MS * 0.001f;
            if (a->state == AG_FLYING && !(a->has_m && (a->m.kind == MS_GOTO || a->m.kind == MS_SURVEY))) {
                a->hold.x += (a->stick_x + a->key_x) * dt;     /* the hand on the joystick */
                a->hold.y += (a->stick_y + a->key_y) * dt;
                if (a->hold.x >  SPOT_LIMIT) { a->hold.x =  SPOT_LIMIT; }
                if (a->hold.x < -SPOT_LIMIT) { a->hold.x = -SPOT_LIMIT; }
                if (a->hold.y >  SPOT_LIMIT) { a->hold.y =  SPOT_LIMIT; }
                if (a->hold.y < -SPOT_LIMIT) { a->hold.y = -SPOT_LIMIT; }
            }
            v3_t e = a->brain.est;
            a->spot.x += a->vcmd.x * dt;
            a->spot.y += a->vcmd.y * dt;
            float lx = a->spot.x - e.x, ly = a->spot.y - e.y, l = t_sqrt(lx * lx + ly * ly);
            if (l > SPOT_LEAD) { a->spot.x = e.x + lx * SPOT_LEAD / l;  a->spot.y = e.y + ly * SPOT_LEAD / l; }
            a->spot.z = a->cruise;
            pilot_goal(&a->brain, a->spot, a->vcmd);
        }
    }
    if (a->state == AG_LANDING && a->brain.mode == PM_IDLE) {
        a->state = AG_GROUND;
        a->vcmd = (v3_t){ 0 };
        say(a, "landed\n");
    }

    if (due(now, &a->next_think, THINK_EVERY_MS)) { think(a); }
    if ((a->state == AG_FLYING || a->state == AG_HOMING) && due(now, &a->next_steer, STEER_EVERY_MS)) { move(a); }
    if (due(now, &a->next_hop, HOP_EVERY_MS)) { hops(a); }

    /* ---- talk ---- */
    if (due(now, &a->next_bcn, BCN_EVERY_MS)) {
        beacon(a);
        a->next_bcn = a->next_bcn - 15u + roll(a) % 31u;   /* +-15 ms of jitter */
    }
    if (a->has_m && (int32_t)(now - a->next_msn) >= 0) {
        char body[RADIO_LINE];
        msn_encode(&a->m, body, sizeof body);
        frame(a, 1, "%s", body);
        a->relayed++;
        a->next_msn = now + MSN_EVERY_MS - 100u + roll(a) % 200u;
    }
    if (a->has_m && a->tk.n && due(now, &a->next_tsk, GOSSIP_MS)) { gossip_tasks(a); }
    if (a->has_m && a->cv.nx && due(now, &a->next_map, GOSSIP_MS)) { gossip_map(a); }
    if (due(now, &a->next_exp, EXP_EVERY_MS)) {
        uint8_t gone[NBR_MAX];
        int k = nbr_expire(&a->nb, now, gone, (int)NBR_MAX);
        for (int i = 0; i < k; i++) {
            say(a, "-nbr %u: silent for %u ms (%u left)\n", (unsigned)gone[i], (unsigned)NBR_TIMEOUT_MS, (unsigned)a->nb.n);
        }
    }
    if (due(now, &a->next_phy, PHY_EVERY_MS)) { physics(a); }
    if (due(now, &a->next_nbt, NBT_EVERY_MS)) { table(a); }
}

/* ---- the radio, in -------------------------------------------------------------------- */
static void heard(agent_t *a, const char *line)
{
    bcn_t b;
    if (bcn_decode(line, &b) != BCN_OK) { a->rx_bad++; return; }
    uint8_t out;
    int r = nbr_heard(&a->nb, a->id, a->brain.est, &b, a->now_ms, &out);
    if (r == NBR_SELF) {
        if (a->rx_clash++ % 25u == 0u) { say(a, "ID clash: another drone says it is %u too\n", (unsigned)a->id); }
        return;
    }
    if (r == NBR_OLD) { a->rx_old++; return; }
    if (r == NBR_FAR) { a->rx_far++; return; }
    a->rx++;
    a->heard_ms = a->now_ms;
    if (out) { say(a, "-nbr %u: table full, made room\n", (unsigned)out); }
    if (r == NBR_NEW) {
        const nbr_t *n = nbr_find(&a->nb, b.id);
        float dx = n->pos.x - a->brain.est.x, dy = n->pos.y - a->brain.est.y, dz = n->pos.z - a->brain.est.z;
        char m[16];
        say(a, "+nbr %u (%s): %s m away (%u in table)\n", (unsigned)b.id, agent_type_name(b.type),
            metres(m, t_sqrt(dx * dx + dy * dy + dz * dz)), (unsigned)a->nb.n);
    }
}

static void heard_tasks(agent_t *a, const char *line)
{
    int32_t f[3 + 5 * TASK_MAX];
    int n = frame_ints(line, "TSK", f, 3 + 5 * (int)TASK_MAX);
    if (n < 3) { a->rx_bad++; return; }
    a->heard_ms = a->now_ms;
    if (!a->has_m || f[1] != a->m.mid || f[2] != a->tk.n || n != 3 + 5 * f[2]) { return; }
    uint32_t before = tasks_print(&a->tk);
    for (int i = 0; i < f[2]; i++) {
        const int32_t *e = &f[3 + 5 * i];
        if (e[0] < 0 || e[0] > (int32_t)RADIO_MAX_ID || e[1] < 0 || e[1] > 65535 || e[2] < 0 || e[2] > 65535 ||
            e[3] < 0 || e[3] > 65535) { continue; }
        tasks_merge(&a->tk, a->id, i, (uint8_t)e[0], (uint16_t)e[1], (uint16_t)e[2], (uint16_t)e[3],
                    e[4] != 0, a->now_ms);
    }
    if (tasks_print(&a->tk) != before) { tasks_news(a); }
}

static void heard_map(agent_t *a, const char *line)
{
    int32_t f[6];
    if (frame_ints(line, "MAP", f, 6) != 6) { a->rx_bad++; return; }
    a->heard_ms = a->now_ms;
    if (!a->has_m || f[1] != a->m.mid || !a->cv.nx) { return; }
    for (int i = 2; i < 6; i++) { if (f[i] < 0 || f[i] > 65535) { return; } }
    cover_merge(&a->cv, ((uint32_t)f[2] << 16) | (uint32_t)f[3], ((uint32_t)f[4] << 16) | (uint32_t)f[5]);
}

static void heard_mission(agent_t *a, const char *line)
{
    mission_t m;
    if (msn_decode(line, &m) != 0) { a->rx_bad++; return; }
    a->heard_ms = a->now_ms;
    if (!a->has_m || (int16_t)(uint16_t)(m.mid - a->m.mid) > 0) { adopt(a, &m); }
}

/* $KEY,<key>,<1|0>: the station's keyboard, while this drone is selected there */
static void key(agent_t *a, const char *line)
{
    const char *body;
    size_t len;
    if (proto_check(line, &body, &len) != PROTO_OK || len < 7) { return; }
    const char *k = body + 4, *comma = memchr(k, ',', len - 4);
    if (!comma || comma + 1 >= body + len) { return; }
    size_t kl = (size_t)(comma - k);
    int down = comma[1] == '1';
    if (kl == 1 && (*k == 'a' || *k == 'b')) {
        uint8_t bit = *k == 'a' ? 1u : 2u;
        if (!down) { a->keys_down &= (uint8_t)~bit; return; }
        if (a->keys_down & bit) { return; }
        a->keys_down |= bit;
        if (*k == 'a') { agent_takeoff(a); } else { agent_land(a); }
        return;
    }
    float v = down ? NUDGE_MS : 0.0f;
    a->key_ms = a->now_ms;
    if      (kl == 2 && memcmp(k, "UP", 2) == 0)    { a->key_y =  v; }
    else if (kl == 4 && memcmp(k, "DOWN", 4) == 0)  { a->key_y = -v; }
    else if (kl == 4 && memcmp(k, "LEFT", 4) == 0)  { a->key_x = -v; }
    else if (kl == 5 && memcmp(k, "RIGHT", 5) == 0) { a->key_x =  v; }
}

int agent_line(agent_t *a, const char *line)
{
    if (line[0] != '$') { return 0; }
    if (a->state == AG_NOID || a->state == AG_DEAD) { return 1; }   /* a dead radio hears nothing */
    if      (strncmp(line, "$BCN,", 5) == 0) { heard(a, line); }
    else if (strncmp(line, "$MSN,", 5) == 0) { heard_mission(a, line); }
    else if (strncmp(line, "$TSK,", 5) == 0) { heard_tasks(a, line); }
    else if (strncmp(line, "$MAP,", 5) == 0) { heard_map(a, line); }
    else if (strncmp(line, "$GND,", 5) == 0) {
        int32_t f[3];
        if (frame_ints(line, "GND", f, 3) == 3 && f[0] >= 0 && f[0] <= 65535) {
            a->gnd_ms = a->now_ms | 1u;  a->heard_ms = a->now_ms;
            int16_t ahead = (int16_t)(uint16_t)((uint16_t)f[0] - a->gnd_seq);
            if (!a->gnd_seen) { a->gnd_qf = 65535u;  a->gnd_seen = 1;  ahead = 1; }   /* the first one */
            if (ahead > 0 && ahead <= 50) {
                for (int k = 1; k < ahead; k++) { a->gnd_qf = (uint16_t)(a->gnd_qf - (a->gnd_qf >> 3)); }
                a->gnd_qf = (uint16_t)(a->gnd_qf + ((65535u - a->gnd_qf) >> 3));
            } else if (ahead > 50 || ahead < -20) { a->gnd_qf = 65535u; }
            a->gnd_seq = (uint16_t)f[0];
            if (!a->gs_ms || (int16_t)(uint16_t)((uint16_t)f[0] - a->gs) > 0) { a->gs = (uint16_t)f[0];  a->gs_ms = a->now_ms | 1u; }
        }
        else { a->rx_bad++; }
    }
    else if (strncmp(line, "$KEY,", 5) == 0) { key(a, line); }
    return 1;                                             /* any other $: ignored */
}

/* ---- the shell ------------------------------------------------------------------------- */
void agent_print_nbrs(agent_t *a)
{
    char m[16];
    say(a, "drone %u %s %s, t %lu ms, hop %u: %u neighbours (joined %lu, lost %lu, evicted %lu)\n",
        (unsigned)a->id, agent_type_name(a->type), agent_state_name(a->state), (unsigned long)a->now_ms,
        (unsigned)a->hop, (unsigned)a->nb.n, (unsigned long)a->nb.joins, (unsigned long)a->nb.losses,
        (unsigned long)a->nb.evictions);
    for (unsigned i = 0; i < a->nb.n; i++) {
        const nbr_t *n = &a->nb.e[i];
        float dx = n->pos.x - a->brain.est.x, dy = n->pos.y - a->brain.est.y, dz = n->pos.z - a->brain.est.z;
        say(a, "  %2u %-6s q %3u %%  %6s m  %4lu ms ago  hop %3u  bat %3u %%  %s\n",
            (unsigned)n->id, agent_type_name(n->type), (unsigned)n->q, metres(m, t_sqrt(dx * dx + dy * dy + dz * dz)),
            (unsigned long)(a->now_ms - n->heard_ms), (unsigned)n->hop, (unsigned)n->bat, agent_state_name(n->mode));
    }
    say(a, "  radio: tx %lu rx %lu bad %lu repeats %lu far %lu relayed %lu\n", (unsigned long)a->tx,
        (unsigned long)a->rx, (unsigned long)a->rx_bad, (unsigned long)a->rx_old, (unsigned long)a->rx_far,
        (unsigned long)a->relayed);
}

static void print_mission(agent_t *a)
{
    char b1[16], b2[16];
    if (!a->has_m) { say(a, "no mission heard\n"); return; }
    say(a, "mission %u %s%s; battery %u %%; closest neighbour seen %s m; avoids %lu\n", (unsigned)a->m.mid,
        kind_text[a->m.kind], a->done_m ? " (done here)" : "", (unsigned)a->body.charge,
        metres(b1, a->closest), (unsigned long)a->avoids);
    if (a->m.kind == MS_GOTO) {
        say(a, "  goto: %s%s\n", a->edge > 0.0f ? "edge learnt at " : "no edge known",
            a->edge > 0.0f ? metres(b1, a->edge) : "");
    }
    if (a->parked) { say(a, "  parked: landed by hand, staying down until told\n"); }
    if (a->cv.nx) {
        say(a, "  map: %d of %d cells covered (%u by me)\n", cover_count(&a->cv), cover_cells(&a->cv), (unsigned)a->covered);
    }
    for (unsigned i = 0; i < a->tk.n; i++) {
        const task_t *t = &a->tk.t[i];
        say(a, "  task %u at %s,%s: %s winner %u bid %u\n", i + 1, metres(b1, t->x), metres(b2, t->y),
            t->done ? "DONE" : "open", (unsigned)t->winner, (unsigned)t->bid);
    }
}

static int is(const char *a, const char *b) { return strcmp(a, b) == 0; }

static long number(const char *s)
{
    long v = 0;
    while (*s == ' ') { s++; }
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s++ - '0'); }
    return v;
}

int agent_command(agent_t *a, char *line)
{
    while (*line == ' ') { line++; }
    /* a frame that lost its '$' on the way ("KEY,RIGHT,1*26"): radio damage,
     * not a command - count it, say nothing */
    if (strchr(line, '*') && strchr(line, ',')) { a->rx_bad++;  return 1; }
    char *e = line + strlen(line);
    while (e > line && (e[-1] == ' ' || e[-1] == '\r')) { *--e = '\0'; }
    for (char *c = line; *c; c++) { if (*c >= 'A' && *c <= 'Z') { *c = (char)(*c + 32); } }
    if (is(line, "nbr") || is(line, "n"))       { agent_print_nbrs(a); return 1; }
    if (is(line, "msn") || is(line, "m"))       { print_mission(a); return 1; }
    if (is(line, "up") || is(line, "takeoff"))  { agent_takeoff(a); return 1; }
    if (is(line, "down") || is(line, "land"))   { agent_land(a); return 1; }
    if (is(line, "kill"))                       { agent_kill(a); return 1; }
    if (strncmp(line, "bat ", 4) == 0) {
        long v = number(line + 4);
        if (v > 100) { v = 100; }
        a->body.charge = (float)v;
        say(a, "battery set to %ld %%\n", v);
        return 1;
    }
    if (is(line, "help") || is(line, "?")) {
        say(a, "drone %u (%s): a/up take off  b/down land  arrows/joystick move  SEL/kill crash\n",
            (unsigned)a->id, agent_type_name(a->type));
        say(a, "cmds: nbr  msn  up  down  kill  bat N  stats  help\n");
        return 1;
    }
    return 0;
}
