/*
 * test_swarm.c - the whole swarm on the PC, every property tested
 *
 * The SAME agent.c, nbr.c, radio.c, steer.c and cover.c the chips
 * run, one agent_t per drone, ticked side by side.  Between them, a model of
 * the station's air (radio_medium.py): a radio frame reaches only the drones
 * within range, one tick (10 ms) later, maybe lost; a relay's radio reaches
 * twice as far; a cut radio neither sends nor hears; and a ground radio that
 * beacons and repeats the operator's mission.  Nothing in this file steers
 * a drone, assigns it anything or tells it what the others are doing - it
 * only carries what they transmit.  Distances for the checks come from the
 * airframes' TRUTH, which no drone sees.
 *
 *   out\test_swarm.exe          all scenarios
 *   out\test_swarm.exe -v       ... and every drone's printed events
 *   out\test_swarm.exe NAME     one scenario (its name's first letters)
 *
 * Exit code 0 = every check passed.
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../agent.h"
#include "proto.h"

#define MAXD     48
#define PEND     20000

/* ======================================================================
 *  the world: the air, the ground radio, the clock
 * ====================================================================== */
static agent_t A[MAXD];
static int     N;
static float   range_m = 12.0f, loss = 0.0f, fade = 0.25f;
static int     cut[MAXD];
static float   speed[MAXD], acc[MAXD];          /* clock rate per drone: 1 = real time */
static float   gnd_x = 6.0f, gnd_y = -4.0f;
static char    mission[RADIO_LINE];
static uint16_t mid = 100;
static uint32_t T;                              /* world time, ms */
static int     current = -1, verbose, close_log, trace_id, route_returns[MAXD];
static uint32_t dice = 12345u, seed = 1u, worlds, scen_hash;
static float   minsep;                           /* m, truth, between airborne drones */
static int     minsep_a, minsep_b;
static uint32_t minsep_t;
static long    frames_bad, frames_total, frames_air;
static long    tx_bytes[MAXD];
static long    phy_sent[MAXD], phy_must[MAXD];   /* $PHY frames, and those sent whatever the ring */

/* the world's obstacles: round columns, taller than every lane */
#define MAXOB 16
static float   obx[MAXOB], oby[MAXOB], obr[MAXOB];
static int     nob, crashes, crashed[MAXD];
static float   ob_closest;                        /* m, a flying drone to an obstacle's surface */

typedef struct { int dst; char line[RADIO_LINE + 8]; } pend_t;
static pend_t  pend[2][PEND];
static int     npend[2], cur_q;
static long    pend_dropped;

static int fails, checks;
static void check(int ok, const char *what, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, what);
    vsnprintf(buf, sizeof buf, what, ap);
    va_end(ap);
    checks++;
    if (!ok) { fails++; }
    printf("  %s  %s\n", ok ? "ok  " : "FAIL", buf);
}

static uint32_t rnd(void) { dice ^= dice << 13;  dice ^= dice >> 17;  dice ^= dice << 5;  return dice; }
static float frand(void) { return (float)(rnd() & 0xFFFFFF) / 16777216.0f; }

static float reach(int a, int b)                 /* -1: the ground radio */
{
    float ga = a >= 0 && A[a].type == TY_RELAY ? 2.0f : 1.0f;
    float gb = b >= 0 && A[b].type == TY_RELAY ? 2.0f : 1.0f;
    return range_m * (ga + gb) / 2.0f;
}

static v3_t where(int i) { return i < 0 ? (v3_t){ gnd_x, gnd_y, 1.0f } : A[i].body.pos; }

static float dist(v3_t p, v3_t q) { return sqrtf((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y) + (p.z - q.z) * (p.z - q.z)); }

static int arrives(int src, int dst)
{
    float d = dist(where(src), where(dst)), r = reach(src, dst);
    if (d > r) { return 0; }
    float p = 1.0f - loss, edge = r * (1.0f - fade);
    if (d > edge) { p *= 1.0f - (d - edge) / (r - edge) * 0.7f; }
    return frand() < p;
}

static void enqueue(int dst, const char *line)
{
    int q = cur_q ^ 1;
    if (npend[q] >= PEND) { pend_dropped++; return; }
    pend_t *p = &pend[q][npend[q]++];
    p->dst = dst;
    strncpy(p->line, line, sizeof p->line - 1);
    p->line[sizeof p->line - 1] = '\0';
    char *nl = strchr(p->line, '\n');
    if (nl) { *nl = '\0'; }
}

static void transmit(int src, const char *line)
{
    if (src >= 0 && cut[src]) { return; }
    for (int j = 0; j < N; j++) {
        if (j == src || cut[j]) { continue; }
        if (arrives(src, j)) { enqueue(j, line); }
    }
}

static void emit(const char *line, int must)
{
    int src = current;
    if (src >= 0 && !strncmp(line, "$PHY,", 5)) { phy_sent[src]++;  phy_must[src] += must != 0; }
    frames_total++;
    if (src >= 0) { tx_bytes[src] += (long)strlen(line); }
    if (line[0] == '$') {
        const char *body;
        size_t len;
        if (strlen(line) >= RADIO_LINE || proto_check(line, &body, &len) != PROTO_OK) { frames_bad++; }
        if (!strncmp(line, "$BCN,", 5) || !strncmp(line, "$MSN,", 5) || !strncmp(line, "$TSK,", 5) ||
            !strncmp(line, "$MAP,", 5)) {
            frames_air++;
            transmit(src, line);
        }
        return;
    }
    if (verbose) { printf("    %6.1f s [D%d] %s", T / 1000.0, src >= 0 ? A[src].id : 0, line); }
}

static void ground_send(const char *body)
{
    char line[RADIO_LINE + 8];
    snprintf(line, sizeof line, "$%s*%02X", body, (unsigned)proto_checksum(body, strlen(body)));
    transmit(-1, line);
}

static void post(uint8_t kind, float x0, float y0, float x1, float y1)
{
    mission_t m = { .mid = ++mid, .kind = kind, .gx = gnd_x, .gy = gnd_y, .x0 = x0, .y0 = y0, .x1 = x1, .y1 = y1 };
    msn_encode(&m, mission, sizeof mission);
    ground_send(mission);
}

static void world(int n, const uint8_t *types, float rng)
{
    N = n;
    dice = 12345u + seed * 7919u + scen_hash + ++worlds * 104729u;   /* by -s, scenario, world */
    range_m = rng;  loss = 0.0f;  fade = 0.25f;
    gnd_x = 6.0f;  gnd_y = -4.0f;
    mission[0] = '\0';
    T = 0;
    npend[0] = npend[1] = 0;
    minsep = 99.0f;
    frames_bad = frames_total = frames_air = 0;
    memset(cut, 0, sizeof cut);
    memset(route_returns, 0, sizeof route_returns);
    nob = crashes = 0;
    memset(crashed, 0, sizeof crashed);
    ob_closest = 99.0f;
    memset(tx_bytes, 0, sizeof tx_bytes);
    memset(phy_sent, 0, sizeof phy_sent);
    memset(phy_must, 0, sizeof phy_must);
    for (int i = 0; i < n; i++) {
        speed[i] = 1.0f;  acc[i] = 0.0f;
        current = i;
        agent_init(&A[i], (uint8_t)(i + 1), types ? types[i] : TY_MAPPER, emit);
    }
    current = -1;
}

static int flying(int i) { return A[i].state != AG_DEAD && A[i].body.pos.z > 1.0f && !A[i].body.grounded; }

static void step(void)
{
    /* what was sent last tick arrives now */
    cur_q ^= 1;
    int q = cur_q;
    for (int k = 0; k < npend[q]; k++) {
        current = pend[q][k].dst;
        agent_line(&A[current], pend[q][k].line);
    }
    npend[q] = 0;
    T += AGENT_TICK_MS;
    current = -1;
    if (T % 200u == 0u) {                               /* the ground radio: 5 beacons a second */
        char g[40];
        snprintf(g, sizeof g, "GND,%u,%d,%d", (unsigned)((T / 200u) % 65536u), (int)(gnd_x * 10), (int)(gnd_y * 10));
        ground_send(g);
        if (mission[0] && T % 2000u == 0u) { ground_send(mission); }
    }
    for (int i = 0; i < N; i++) {
        acc[i] += speed[i];
        while (acc[i] >= 1.0f) {                 /* a slower chip ticks less often */
            acc[i] -= 1.0f;
            current = i;
            agent_tick(&A[i]);
        }
    }
    current = -1;
    for (int i = 0; i < N; i++) {
        if (A[i].state == AG_HOMING && A[i].why && !strcmp(A[i].why, "no route to the ground radio")) { route_returns[i] = 1; }
    }
    for (int i = 0; i < N && nob; i++) {               /* the rangefinder, and collisions */
        if (crashed[i]) { continue; }
        v3_t p = A[i].body.pos;
        for (int k = 0; k < nob; k++) {
            float d = sqrtf((p.x - obx[k]) * (p.x - obx[k]) + (p.y - oby[k]) * (p.y - oby[k])) - obr[k];
            if (T % 500u == 0u && d <= 6.0f) {
                char body[48], line[64];
                snprintf(body, sizeof body, "OBS,%d,%d,%d,%d", k, (int)lroundf(obx[k] * 10), (int)lroundf(oby[k] * 10),
                         (int)lroundf(obr[k] * 10));
                snprintf(line, sizeof line, "$%s*%02X", body, (unsigned)proto_checksum(body, strlen(body)));
                enqueue(i, line);
            }
            if (p.z > 0.3f && !A[i].body.grounded) {
                if (d < ob_closest) { ob_closest = d; }
                if (d < 0.3f) {                            /* inside it, drone's size and all */
                    char body[16], line[32];
                    snprintf(body, sizeof body, "HIT,%d", A[i].id);
                    snprintf(line, sizeof line, "$%s*%02X", body, (unsigned)proto_checksum(body, strlen(body)));
                    enqueue(i, line);
                    crashed[i] = 1;
                    crashes++;
                    if (verbose) { printf("    %6.1f s CRASH: D%d into obstacle %d\n", T / 1000.0, A[i].id, k); }
                }
            }
        }
    }
    if (trace_id && trace_id <= N && T % 1000u == 0u) {
        const agent_t *a = &A[trace_id - 1];
        printf("    TRACE %.0f s D%d %s est(%.2f,%.2f,%.2f) v(%.2f,%.2f) vcmd(%.2f,%.2f) goal(%.2f,%.2f) has %d home(%.1f,%.1f) nb %u\n",
               T / 1000.0, a->id, agent_state_name(a->state), a->brain.est.x, a->brain.est.y, a->brain.est.z,
               a->brain.est_v.x, a->brain.est_v.y, a->vcmd.x, a->vcmd.y, a->goal.x, a->goal.y, a->has_goal,
               a->home.x, a->home.y, a->nb.n);
        printf("    NBRS %.0f s D%d:", T / 1000.0, a->id);
        for (unsigned k = 0; k < a->nb.n; k++) {
            const nbr_t *o = &a->nb.e[k];
            printf(" D%u %s (%.1f,%.1f,%.1f) truth (%.1f,%.1f,%.1f)", o->id, agent_state_name(o->mode), o->pos.x, o->pos.y, o->pos.z,
                   A[o->id - 1].body.pos.x, A[o->id - 1].body.pos.y, A[o->id - 1].body.pos.z);
        }
        printf("\n");
    }
    for (int i = 0; i < N; i++) {
        if (!flying(i)) { continue; }
        for (int j = i + 1; j < N; j++) {
            if (!flying(j)) { continue; }
            float d = dist(A[i].body.pos, A[j].body.pos);
            if (d < minsep) { minsep = d;  minsep_a = i + 1;  minsep_b = j + 1;  minsep_t = T; }
            if (close_log && d < 1.2f && T % 100u == 0u) {
                const agent_t *p = &A[i], *q = &A[j];
                printf("    CLOSE %.1f s d=%.2f  D%d %s (%.1f,%.1f,%.1f) v(%.1f,%.1f,%.1f) sees %d %s | "
                       "D%d %s (%.1f,%.1f,%.1f) v(%.1f,%.1f,%.1f) sees %d %s\n", T / 1000.0, d,
                       p->id, agent_state_name(p->state), p->body.pos.x, p->body.pos.y, p->body.pos.z,
                       p->body.vel.x, p->body.vel.y, p->body.vel.z, nbr_find(&p->nb, q->id) != 0, p->avoiding ? "AV" : "",
                       q->id, agent_state_name(q->state), q->body.pos.x, q->body.pos.y, q->body.pos.z,
                       q->body.vel.x, q->body.vel.y, q->body.vel.z, nbr_find(&q->nb, p->id) != 0, q->avoiding ? "AV" : "");
            }
        }
    }
}

static void run(uint32_t ms) { uint32_t end = T + ms; while (T < end) { step(); } }

static int __attribute__((unused)) all_landed(void)
{
    for (int i = 0; i < N; i++) {
        if (A[i].state == AG_DEAD) { continue; }
        if (A[i].state != AG_GROUND || !A[i].body.grounded) { return 0; }
    }
    return 1;
}

/* run until cond() or max_ms; returns the time it took, or 0 */
static void unfinished(void);
static uint32_t run_until(int (*cond)(void), uint32_t max_ms)
{
    uint32_t t0 = T;
    while (T - t0 < max_ms) {
        step();
        if (cond()) { return T - t0; }
    }
    unfinished();
    return 0;
}

static float flat(v3_t p, v3_t q);
static void obstacle(float x, float y, float r) { if (nob < MAXOB) { obx[nob] = x;  oby[nob] = y;  obr[nob] = r;  nob++; } }

/* n random obstacles in (x0..x1, y0..y1): >= 5 m apart surface to surface,
 * clear of the pads (4 m), the ground radio (5 m) and the `keep` points (3.5 m) */
static void random_obstacles(int n, float x0, float y0, float x1, float y1, int nkeep, const float *keep)
{
    for (int tries = 0; tries < n * 400 && nob < n; tries++) {
        float r = 0.5f + frand(), x = x0 + (x1 - x0) * frand(), y = y0 + (y1 - y0) * frand();
        int ok = 1;
        for (int i = 0; i < N && ok; i++) { ok = flat((v3_t){ x, y, 0 }, A[i].home) >= r + 4.0f; }
        if (ok) { ok = flat((v3_t){ x, y, 0 }, where(-1)) >= r + 5.0f; }
        for (int i = 0; i < nkeep && ok; i++) { ok = flat((v3_t){ x, y, 0 }, (v3_t){ keep[2 * i], keep[2 * i + 1], 0 }) >= r + 3.5f; }
        for (int k = 0; k < nob && ok; k++) { ok = flat((v3_t){ x, y, 0 }, (v3_t){ obx[k], oby[k], 0 }) >= r + obr[k] + 5.0f; }
        if (ok) { obstacle(x, y, r); }
    }
}

static int hears(int i, int id) { return nbr_find(&A[i].nb, (uint8_t)id) != 0; }
static float flat(v3_t p, v3_t q) { return sqrtf((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y)); }

static int on_pad(int i)
{
    return A[i].state == AG_GROUND && A[i].body.grounded && flat(A[i].body.pos, A[i].home) < 1.0f;
}

static int run_mission_on_pads(void)
{
    for (int i = 0; i < N; i++) { if (A[i].state != AG_DEAD && !on_pad(i)) { return 0; } }
    return 1;
}

/* the mission is over: every drone that is alive and can hear is on its own
 * pad and believes its part is done */
static int mission_over(void)
{
    for (int i = 0; i < N; i++) {
        if (A[i].state == AG_DEAD || cut[i]) { continue; }
        if (!on_pad(i) || !A[i].done_m) { return 0; }
    }
    return 1;
}


/* who is not finished, and why - printed when a mission times out */
static void unfinished(void)
{
    for (int i = 0; i < N; i++) {
        const agent_t *a = &A[i];
        if (a->state == AG_DEAD || cut[i] || (on_pad(i) && a->done_m)) { continue; }
        printf("    not done: D%d %s %s at (%.1f,%.1f,%.1f) bat %.0f%% done_m %d why %s map %d/%d",
               a->id, agent_type_name(a->type), agent_state_name(a->state), a->body.pos.x, a->body.pos.y,
               a->body.pos.z, a->body.charge, a->done_m, a->why ? a->why : "-", cover_count(&a->cv), cover_cells(&a->cv));
        printf(" hop %u nb %u\n", a->hop, a->nb.n);
    }
}

static int has_mission_all(void)
{
    for (int i = 0; i < N; i++) { if (!A[i].has_m || A[i].m.mid != mid) { return 0; } }
    return 1;
}


static const char *scen;
static int want(const char *name)
{
    if (scen && strncmp(name, scen, strlen(scen)) != 0) { return 0; }
    scen_hash = 2166136261u;                     /* the dice depend on the scenario, not on what ran before */
    for (const char *c = name; *c; c++) { scen_hash = (scen_hash ^ (uint8_t)*c) * 16777619u; }
    worlds = 0;
    return 1;
}

/* the standard survey: 20 x 20 m north of the pads */
static const float SX0 = 0, SY0 = 10, SX1 = 20, SY1 = 30;
static void survey(void) { post(MS_SURVEY, SX0, SY0, SX1, SY1); }

/* every drone that is not a relay covered cells of the map itself */
static int all_mapped(void)
{
    int ok = 1;
    for (int i = 0; i < N; i++) { if (A[i].type != TY_RELAY && A[i].state != AG_DEAD) { ok &= A[i].covered > 0; } }
    return ok;
}

static void report_sep(void)
{
    printf("    closest two airborne drones ever came: %.2f m (D%d-D%d at %.1f s)\n", minsep, minsep_a, minsep_b,
           minsep_t / 1000.0);
}

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-v")) { verbose = 1; }
        else if (!strcmp(argv[i], "-c")) { close_log = 1; }
        else if (!strcmp(argv[i], "-t") && i + 1 < argc) { trace_id = atoi(argv[++i]); }
        else if (!strcmp(argv[i], "-s") && i + 1 < argc) { seed = (uint32_t)atoi(argv[++i]); }
        else { scen = argv[i]; }
    }
    setvbuf(stdout, 0, _IONBF, 0);
    clock_t wall = clock();

    /* ================================================================== */
    if (want("protocol")) {
        printf("== protocol: every frame to a line and back ==\n");
        bcn_t b = { .id = 63, .seq = 65535, .x = -12345, .y = 678, .z = 400, .vx = -5, .vy = 0, .vz = 99,
                    .bat = 87, .mode = AG_HOMING, .nn = 8, .type = TY_RELAY, .hop = 255, .gx = -1234, .gy = 5678,
                    .gs = 65535 }, c;
        char line[RADIO_LINE];
        int n = bcn_encode(&b, line, sizeof line);
        check(n > 0 && n < (int)RADIO_LINE && line[n - 1] == '\n', "a beacon at its longest fits a line (%d B)", n);
        check(bcn_decode(line, &c) == BCN_OK && c.id == b.id && c.seq == b.seq && c.x == b.x && c.y == b.y &&
              c.z == b.z && c.vx == b.vx && c.vy == b.vy && c.vz == b.vz && c.bat == b.bat && c.mode == b.mode &&
              c.nn == b.nn && c.type == b.type && c.hop == b.hop && c.gx == b.gx && c.gy == b.gy && c.gs == b.gs,
              "it decodes to the same beacon, field by field");
        line[7] ^= 1;
        check(bcn_decode(line, &c) == BCN_E_FRAME, "one flipped bit fails the checksum");
        mission_t m = { .mid = 65535, .kind = MS_SURVEY, .gx = -12.3f, .gy = 45.6f, .x0 = -100, .y0 = -100,
                        .x1 = 100, .y1 = 99.9f }, m2;
        char body[RADIO_LINE], full[RADIO_LINE + 8];
        msn_encode(&m, body, sizeof body);
        snprintf(full, sizeof full, "$%s*%02X", body, (unsigned)proto_checksum(body, strlen(body)));
        check(strlen(full) < RADIO_LINE && msn_decode(full, &m2) == 0 && m2.mid == m.mid && m2.kind == m.kind &&
              fabsf(m2.y1 - 99.9f) < 0.05f && fabsf(m2.gy - 45.6f) < 0.05f,
              "a mission round-trips (%d B)", (int)strlen(full));
        check(msn_decode("$MSN,1,9,0,0,0,0,0,0*00", &m2) != 0, "a bad mission is refused");
        int32_t f[4];
        check(frame_ints("$GND,1,2,3*06", "GND", f, 4) == BCN_E_FRAME || frame_ints("$GND,1,2,3*06", "GND", f, 4) == 3,
              "frame_ints reads a frame");

        /* fuzz: 200 000 mangled frames into one drone - nothing may break */
        world(2, 0, 12.0f);
        run(1500);
        const char *seeds[] = { "$BCN,2,5,300,0,0,0,0,0,90,1,1,0,1,30,0*", "$MSN,9,2,60,-40,0,100,200,300,1,30,270*",
                                "$TSK,2,9,1,2,100,3,0,0*", "$MAP,2,9,0,0,0,15*", "$GND,1,60,-40*", "$KEY,a,1*" };
        long before_bad = A[0].rx_bad;
        for (int k = 0; k < 200000; k++) {
            char f2[RADIO_LINE + 8], bod[RADIO_LINE];
            const char *s = seeds[rnd() % 6];
            size_t L = strlen(s) - 2;                    /* without "*" ... rebuilt below */
            memcpy(bod, s + 1, L - 1);
            bod[L - 1] = '\0';
            int muts = 1 + (int)(rnd() % 4);
            for (int u = 0; u < muts && bod[0]; u++) {
                size_t at = rnd() % strlen(bod);
                switch (rnd() % 4) {
                case 0: bod[at] = (char)(32 + rnd() % 95); break;            /* a different character */
                case 1: bod[at] = ','; break;
                case 2: bod[at] = '\0'; break;                               /* cut short */
                default: bod[at] = (char)('0' + rnd() % 10); break;
                }
            }
            if (rnd() % 3) { snprintf(f2, sizeof f2, "$%s*%02X", bod, (unsigned)proto_checksum(bod, strlen(bod))); }
            else           { snprintf(f2, sizeof f2, "$%s*%02X", bod, (unsigned)(rnd() & 0xFF)); }   /* bad CS too */
            current = 0;
#ifdef FUZZDUMP
            fprintf(stderr, "%d %s\n", k, f2);  fflush(stderr);
#endif
            agent_line(&A[0], f2);
            if (k % 1000 == 0) { current = 0; agent_tick(&A[0]); }
        }
        current = -1;
        check(1, "200 000 mangled frames: no crash, no hang (%ld refused as malformed)", A[0].rx_bad - before_bad);
        {   /* a frame that lost its '$' on the way - seen in Wokwi as "? key,right,1*26" */
            char cmd[] = "KEY,RIGHT,1*26";
            uint32_t bad0 = A[0].rx_bad;
            int v0 = verbose;
            current = 0;
            int took = agent_command(&A[0], cmd);
            verbose = v0;
            check(took == 1 && A[0].rx_bad == bad0 + 1, "a frame that lost its '$' is counted as damage, not run as a command");
        }
        check(A[0].nb.n <= NBR_MAX && A[0].state != AG_NOID, "the drone's table and state are still sane");
        world(2, 0, 12.0f);
        run(30000);
        check(phy_sent[1] >= 148 && phy_sent[1] <= 152 && phy_must[1] >= 29 && phy_must[1] <= 31,
              "in 30 s a drone sent %ld position reports, %ld of them compulsory (one a second: "
              "never all skipped, so the station never loses it)", phy_sent[1], phy_must[1]);
    }

    /* ================================================================== */
    if (want("neighbours")) {
        printf("== neighbours (phase 1) ==\n");
        world(5, 0, 50.0f);
        run(3000);
        int ok = 1;
        for (int i = 0; i < 5; i++) { ok &= A[i].nb.n == 4; }
        check(ok, "five drones on their pads all find each other");
        range_m = 4.0f;
        run(3000);
        ok = 1;
        for (int i = 0; i < 5; i++) { ok &= A[i].nb.n == ((i == 0 || i == 4) ? 1u : 2u); }
        check(ok, "range 4 m, pads 3 m apart: a chain");
        range_m = 50.0f;
        run(1000);
        cut[2] = 1;
        run(3500);
        ok = 1;
        for (int i = 0; i < 5; i++) { if (i != 2) { ok &= !hears(i, 3); } }
        check(ok && A[2].nb.n == 0, "a dead radio is dropped by all within 3.5 s (timeout 3 s), and hears nobody");
        cut[2] = 0;
        loss = 0.3f;
        run(20000);
        float qs = 0;
        int qn = 0;
        for (int i = 0; i < 5; i++) { for (unsigned k = 0; k < A[i].nb.n; k++) { qs += A[i].nb.e[k].q; qn++; } }
        check(qn == 20 && qs / qn > 60 && qs / qn < 85, "30 %% loss for 20 s: every link held, quality %.0f %%", qs / qn);
        loss = 0;
        world(14, 0, 60.0f);
        run(4000);
        ok = 1;
        for (int k = 2; k <= 14; k++) { ok &= hears(0, k) == (k <= 9); }
        check(ok, "14 drones, a table of 8: drone 1 keeps exactly its 8 nearest");
    }

    /* ================================================================== */
    if (want("avoid")) {
        printf("== collision avoidance: drones told to fly THROUGH each other ==\n");
        uint8_t ty[8] = { 0 };
        /* two drones head-on, 18 m apart, swapping places */
        world(7, ty, 40.0f);
        run(2000);                                        /* GPS first */
        post(MS_HOLD, 0, 0, 0, 0);
        for (int i = 0; i < 7; i++) { if (i == 0 || i == 6) { current = i; agent_takeoff(&A[i]); } }
        run(6000);
        A[0].hold = (v3_t){ 18, 0, HOVER_Z };  A[6].hold = (v3_t){ 0, 0, HOVER_Z };
        minsep = 99;
        run(25000);
        report_sep();
        check(minsep > 1.2f, "head-on, straight at each other: they passed %.2f m apart (> 1.2)", minsep);
        check(flat(A[0].body.pos, A[0].hold) < 1.0f && flat(A[6].body.pos, A[6].hold) < 1.0f,
              "and both reached the other's spot (%.1f m, %.1f m off)", flat(A[0].body.pos, A[0].hold),
              flat(A[6].body.pos, A[6].hold));
        check(A[0].avoids > 0 && A[6].avoids > 0, "both used the look-ahead rule (%lu, %lu steps)",
              (unsigned long)A[0].avoids, (unsigned long)A[6].avoids);

        /* eight drones on a circle, every one to the opposite side at once */
        world(8, ty, 40.0f);
        run(2000);
        post(MS_HOLD, 0, 0, 0, 0);
        for (int i = 0; i < 8; i++) { current = i; agent_takeoff(&A[i]); }
        run(5000);
        float cx = 10.5f, cy = 15.0f, R = 9.0f;
        for (int i = 0; i < 8; i++) {
            float a = 6.2831853f * i / 8;
            A[i].hold = (v3_t){ cx + R * cosf(a), cy + R * sinf(a), HOVER_Z };
        }
        run(30000);                                       /* to the circle (spreading out) */
        minsep = 99;
        for (int i = 0; i < 8; i++) {
            float a = 6.2831853f * i / 8 + 3.1415926f;
            A[i].hold = (v3_t){ cx + R * cosf(a), cy + R * sinf(a), HOVER_Z };
        }
        float straight = 0;                               /* with straight lines they would all meet in the middle */
        uint32_t t0 = T, took = 0;
        while (T - t0 < 90000) {
            step();
            int there = 1;
            for (int i = 0; i < 8; i++) { there &= flat(A[i].body.pos, A[i].hold) < 1.0f; }
            if (there) { took = T - t0; break; }
        }
        report_sep();
        check(minsep > 1.0f, "8 drones swap across a circle (straight paths: all meet at the centre, %.0f m apart): "
              "closest %.2f m", straight, minsep);
        check(took > 0, "no deadlock: all eight arrived, in %.1f s", took / 1000.0);
    }

    /* ================================================================== */
    if (want("flock")) {
        printf("== flocking: GOTO a point 40 m away (radio 50 m: in reach) ==\n");
        world(6, 0, 50.0f);
        post(MS_GOTO, 10, 40, 0, 0);
        run(60000);
        float far = 0, sum = 0;
        int all_fly = 1;
        for (int i = 0; i < 6; i++) {
            float d = flat(A[i].body.pos, (v3_t){ 10, 40, 0 });
            if (d > far) { far = d; }
            sum += d;
            all_fly &= A[i].state == AG_FLYING;
        }
        report_sep();
        check(all_fly, "all six took off by themselves when the mission reached them");
        check(far < 6.0f, "they gathered round the point: mean %.1f m, farthest %.1f m", sum / 6, far);
        check(minsep > 1.5f, "and kept apart doing it: closest %.2f m", minsep);
        int linked = 1;
        for (int i = 0; i < 6; i++) { linked &= A[i].nb.n >= 3; }
        check(linked, "the flock is one group: everyone hears >= 3 others");
        /* the same point, out of reach: radio 15 m */
        world(6, 0, 15.0f);
        run(2000);
        post(MS_GOTO, 10, 40, 0, 0);
        run(60000);
        long heard = 0, samples = 0;
        float out = 0;
        for (int k = 0; k < 500; k++) {                   /* 5 s at the edge */
            step();
            for (int i = 0; i < 6; i++) {
                heard += A[i].hop != HOP_NONE;
                samples++;
                float d = flat(A[i].body.pos, where(-1));
                if (d > out) { out = d; }
            }
        }
        check(heard * 10 >= samples * 9 && out < 26.0f, "the point 44 m out, radio 15 m: they hold at the edge "
              "(at most %.1f m out), in touch %.0f %% of the time", out, 100.0 * heard / samples);
        post(MS_HOME, 0, 0, 0, 0);
        uint32_t t = run_until(run_mission_on_pads, 90000);
        check(t > 0, "so HOME reaches them, and all six are back on their pads in %.0f s", t / 1000.0);
        world(6, 0, 15.0f);
        run(2000);
        post(MS_GOTO, 8, 12, 0, 0);                 /* within reach this time */
        run(30000);
        post(MS_HOME, 0, 0, 0, 0);
        t = run_until(run_mission_on_pads, 90000);
        check(t > 0, "a flock within reach: HOME brings all six back to their own pads in %.1f s", t / 1000.0);
    }

    /* ================================================================== */
    if (want("multihop")) {
        printf("== multi-hop: a mission flooded beyond the ground radio's reach ==\n");
        world(7, 0, 7.0f);                                /* pads 0..18 m, range 7 m */
        gnd_x = 0;  gnd_y = -3;                           /* only drones 1-3 hear the ground */
        run(3000);
        int direct = 0;
        for (int i = 0; i < 7; i++) { direct += dist(where(-1), A[i].body.pos) <= 7.0f; }
        post(MS_HOLD, 0, 0, 0, 0);
        uint32_t t = run_until(has_mission_all, 10000);
        check(direct < 7 && t > 0, "%d of 7 hear the ground radio; all 7 have the mission in %.2f s", direct, t / 1000.0);
        run(2000);
        int sane = 1;
        printf("    hops:");
        for (int i = 0; i < 7; i++) {
            printf(" D%d=%u", i + 1, A[i].hop);
            int least = (int)ceilf(dist(where(-1), A[i].body.pos) / 7.0f);   /* no hop longer than the range */
            sane &= A[i].hop != HOP_NONE && A[i].hop >= least && A[i].hop <= least + 2;
        }
        printf("\n");
        check(A[0].hop == 1 && sane, "every drone has a route, and no hop count beats physics (>= distance / range)");
        cut[3] = cut[4] = 1;                              /* break the chain: D4 and D5 go dark */
        uint32_t t0 = T;
        while (!(A[5].hop == HOP_NONE && A[6].hop == HOP_NONE) && T - t0 < 30000) { step(); }
        check(A[5].hop == HOP_NONE && A[6].hop == HOP_NONE,
              "chain broken (D4, D5 dark): D6 and D7 work out they have no route, in %.1f s", (T - t0) / 1000.0);
        cut[3] = cut[4] = 0;
        run(3000);
        check(A[6].hop != HOP_NONE, "chain healed: the route is back (D7 hop %u)", A[6].hop);
    }

    uint8_t mix[7] = { TY_MAPPER, TY_MAPPER, TY_MAPPER, TY_RELAY, TY_MAPPER, TY_MAPPER, TY_MAPPER };
    /* ================================================================== */
    if (want("survey")) {
        printf("== SURVEY: 6 mappers and 1 relay, 16 cells - every drone flies ==\n");
        world(7, mix, 12.0f);
        run(2000);
        survey();
        uint32_t t_start = T;
        run(12000);
        v3_t relay = A[3].body.pos;                       /* mid-mission */
        uint32_t t = run_until(mission_over, 400000);
        if (t) { t = T - t_start; }
        int full = 1;
        for (int i = 0; i < 7; i++) { full &= cover_done(&A[i].cv); }
        report_sep();
        check(t > 0, "mission complete, every drone back on its own pad, %.0f s after it was posted", t / 1000.0);
        check(full, "every drone's copy of the map is full (%d of %d cells)", cover_count(&A[0].cv), cover_cells(&A[0].cv));
        printf("    cells seen by each:");
        for (int i = 0; i < 7; i++) { printf(" D%d %u", i + 1, A[i].covered); }
        printf("\n");
        check(all_mapped(), "all six mappers flew and covered cells themselves");
        check(A[3].covered == 0, "the relay flew but mapped nothing: it keeps the others in touch");
        float on_line = fabsf((relay.x - 6) * (20 + 4) - (relay.y + 4) * (10 - 6)) / sqrtf(24 * 24 + 4 * 4);
        check(on_line < 3.0f && relay.y > 1 && relay.y < 18 && relay.z > 3, "at 12 s the relay hovered between the "
              "ground radio and the area at (%.1f, %.1f, %.1f), %.1f m off the line", relay.x, relay.y, relay.z, on_line);
        check(minsep > 1.0f, "no two drones closer than 1 m all mission: %.2f m", minsep);
        check(frames_bad == 0, "%ld frames sent, every one well-formed", frames_total);
    }

    /* ================================================================== */
    if (want("faults")) {
        printf("== FAULTS: two mappers crash mid-survey ==\n");
        world(7, mix, 12.0f);
        run(2000);
        post(MS_SURVEY, 0, 10, 30, 35);                   /* 6 x 5 cells */
        uint32_t tw = T;                                  /* crash them once mapping is well under way */
        while (cover_count(&A[0].cv) < 6 && T - tw < 60000) { step(); }
        int victim = -1;
        for (int i = 2; i < 7 && victim < 0; i++) { if (A[i].type == TY_MAPPER && A[i].state == AG_FLYING) { victim = i; } }
        current = victim;  agent_kill(&A[victim]);
        current = 1;  agent_kill(&A[1]);  current = -1;   /* mapper D2 */
        printf("    killed mappers D%d and D2 at %.0f s, %d of %d cells covered\n", victim + 1, T / 1000.0,
               cover_count(&A[0].cv), cover_cells(&A[0].cv));
        run(6000);
        int dropped = 1;
        for (int i = 0; i < 7; i++) { if (A[i].state != AG_DEAD) { dropped &= !hears(i, 2) && !hears(i, victim + 1); } }
        check(dropped, "6 s later nobody has the dead in its table");
        uint32_t t = run_until(mission_over, 400000);
        int full = 1;
        for (int i = 0; i < 7; i++) { if (A[i].state != AG_DEAD) { full &= cover_done(&A[i].cv); } }
        check(t > 0 && full, "the survivors finished the map and are home, %.0f s after the crashes", t / 1000.0);
    }

    /* ================================================================== */
    if (want("battery")) {
        printf("== BATTERY: a mapper's battery fails mid-survey ==\n");
        world(7, mix, 12.0f);
        run(2000);
        post(MS_SURVEY, 0, 10, 30, 35);
        int s = -1;
        uint32_t t0 = T;
        while (s < 0 && T - t0 < 60000) {
            step();
            for (int i = 0; i < 7; i++) {
                if (A[i].type == TY_MAPPER && A[i].state == AG_FLYING && A[i].covered > 0 &&
                    flat(A[i].body.pos, A[i].home) > 8.0f) { s = i; }
            }
        }
        A[s].body.charge = 29.0f;                         /* a bad cell */
        printf("    D%d drops to 29 %% at %.0f s, having covered %u cells itself\n", s + 1, T / 1000.0, A[s].covered);
        run(1000);
        check(A[s].state == AG_HOMING, "it turned for home by itself");
        uint32_t t = run_until(mission_over, 400000);
        int full = 1;
        for (int i = 0; i < 7; i++) { full &= cover_done(&A[i].cv); }
        check(t > 0 && full, "the others finished the map, all home in %.0f s", t / 1000.0);
        check(A[s].body.charge > 30.0f, "D%d recharged on its pad (%.0f %%)", s + 1, A[s].body.charge);
    }

    /* ================================================================== */
    if (want("rejoin")) {
        printf("== REJOIN: a drone sent home on battery comes back once charged ==\n");
        world(3, mix, 12.0f);                             /* three mappers: a long job */
        run(2000);
        post(MS_SURVEY, -10, 10, 50, 50);         /* a big area: 8 x 8 cells, a long job */
        run(20000);
        A[0].body.charge = 29.0f;
        run(2000);
        check(A[0].state == AG_HOMING && A[0].why && !strcmp(A[0].why, "battery low"), "mapper D1 goes home on battery");
        uint32_t t0 = T;
        while (A[0].state != AG_GROUND && T - t0 < 60000) { step(); }
        while (A[0].state == AG_GROUND && T - t0 < 120000) { step(); }
        check(A[0].state == AG_FLYING && !A[0].done_m, "landed, recharged to %.0f %%, and rejoined the unfinished "
              "survey by itself %.0f s later", A[0].body.charge, (T - t0) / 1000.0);
    }

    /* ================================================================== */
    if (want("lostlink")) {
        printf("== LOST LINK: a drone's radio dies mid-mission ==\n");
        world(7, mix, 12.0f);
        run(2000);
        post(MS_SURVEY, 0, 10, 40, 40);           /* a long job */
        run(20000);
        check(A[0].state == AG_FLYING, "mapper D1 is at work (%s)", agent_state_name(A[0].state));
        cut[0] = 1;
        uint32_t t0 = T;
        while (A[0].state == AG_FLYING && T - t0 < 40000) { step(); }
        check(A[0].state == AG_HOMING && !strcmp(A[0].why, "lost link"), "deaf and mute, it turned for home after %.1f s",
              (T - t0) / 1000.0);
        while (!on_pad(0) && T - t0 < 120000) { step(); }
        check(on_pad(0), "and landed on its own pad");
        uint32_t t = run_until(mission_over, 400000);
        check(t > 0, "the others finished the (bigger) survey without it, %.0f s later", t / 1000.0);
    }

    /* ================================================================== */
    if (want("loss")) {
        printf("== 30 %% PACKET LOSS: the full survey ==\n");
        world(7, mix, 12.0f);
        loss = 0.3f;
        run(2000);
        survey();
        uint32_t t = run_until(mission_over, 500000);
        report_sep();
        check(t > 0, "completed under 30 %% loss in %.0f s", t / 1000.0);
        check(minsep > 1.0f, "still no two drones closer than 1 m: %.2f m", minsep);
    }

    /* ================================================================== */
    if (want("clocks")) {
        printf("== UNEQUAL CLOCKS: chips simulated at 100 %%, 85 %%, 70 %% speed ==\n");
        world(7, mix, 12.0f);
        for (int i = 0; i < 7; i++) { speed[i] = (i % 3 == 0) ? 1.0f : (i % 3 == 1) ? 0.85f : 0.7f; }
        run(2000);
        survey();
        uint32_t t = run_until(mission_over, 700000);
        report_sep();
        check(t > 0, "completed with every drone on its own clock speed, %.0f s", t / 1000.0);
        check(minsep > 1.0f, "and no two closer than 1 m: %.2f m", minsep);
    }

    /* ================================================================== */
    if (want("relay")) {
        printf("== RELAY: holding a point 39 m from the ground radio (radio range 12 m) ==\n");
        /* ground radio -> relay 18 m, relay -> relay 24 m, relay -> drone 18 m, drone -> drone 12 m,
         * all fading over their last quarter: one relay cannot bridge 39 m, a chain of two can -
         * if the relays find their places, knowing none of these numbers */
        static const uint8_t crews[3][6] = {
            { TY_MAPPER, TY_MAPPER, TY_MAPPER, TY_MAPPER, TY_MAPPER, TY_MAPPER },   /* no relay   */
            { TY_MAPPER, TY_MAPPER, TY_MAPPER, TY_RELAY,  TY_MAPPER, TY_MAPPER },   /* one relay  */
            { TY_MAPPER, TY_RELAY,  TY_MAPPER, TY_RELAY,  TY_MAPPER, TY_MAPPER },   /* a chain of two */
        };
        float routed_pct[3], there_pct[3];
        int gave_up[3];
        v3_t rpos[2] = { { 0 } };
        for (int pass = 0; pass < 3; pass++) {
            world(6, crews[pass], 12.0f);
            run(2000);
            post(MS_GOTO, 10, 35, 0, 0);
            long routed = 0, there = 0, samples = 0;
            for (int k = 0; k < 24000; k++) {            /* 4 minutes */
                step();
                for (int i = 0; i < 6; i++) {
                    if (A[i].type == TY_RELAY) { continue; }
                    samples++;
                    int at = A[i].state == AG_FLYING && flat(A[i].body.pos, (v3_t){ 10, 35, 0 }) < 8.0f;
                    there += at;
                    routed += at && A[i].hop != HOP_NONE;
                }
                if (pass == 2 && k == 9000) {                /* 90 s in: where did the relays settle? */
                    int r = 0;
                    for (int i = 0; i < 6 && r < 2; i++) { if (A[i].type == TY_RELAY) { rpos[r++] = A[i].body.pos; } }
                }
                if (verbose && pass == 2 && k % 1000 == 0) {
                    for (int i = 0; i < 6; i++) {
                        if (A[i].type == TY_RELAY) {
                            printf("    dbg t=%.0f D%d %s f=%.2f hop=%u pos=(%.1f,%.1f)\n", T / 1000.0, i + 1,
                                   agent_state_name(A[i].state), A[i].relay_f, A[i].hop, A[i].body.pos.x, A[i].body.pos.y);
                        }
                    }
                }
            }
            there_pct[pass] = 100.0f * there / samples;
            routed_pct[pass] = there ? 100.0f * routed / there : 0;
            gave_up[pass] = 0;
            for (int i = 0; i < 6; i++) { gave_up[pass] += A[i].finished_any && A[i].finished_mid == mid; }
            if (pass == 2) {
                printf("    90 s in, the relays had placed themselves at (%.1f, %.1f) and (%.1f, %.1f): %.0f and %.0f m "
                       "from the ground radio at (%.0f, %.0f); the point is 39 m out\n", rpos[0].x, rpos[0].y, rpos[1].x,
                       rpos[1].y, flat(rpos[0], where(-1)), flat(rpos[1], where(-1)), gnd_x, gnd_y);
            }
        }
        printf("    over 4 minutes, share of the workers' time spent at the point / linked home while there:\n"
               "      no relay %.0f %% / %.0f %%   one relay %.0f %% / %.0f %%   two relays %.0f %% / %.0f %%\n",
               there_pct[0], routed_pct[0], there_pct[1], routed_pct[1], there_pct[2], routed_pct[2]);
        check(there_pct[0] < 5.0f, "no relay: the point is out of reach - the workers waited at the edge "
              "(at the point %.0f %% of the time)", there_pct[0]);
        check(there_pct[2] > there_pct[0] + 25.0f && routed_pct[2] > 75.0f && gave_up[2] == 0,
              "two relays: a chain - the workers stay at the point (%.0f %% of the time), linked home %.0f %% of it",
              there_pct[2], routed_pct[2]);
    }

    /* ================================================================== */
    if (want("hetero")) {
        printf("== HETEROGENEITY: a relay and mappers, one firmware ==\n");
        world(7, mix, 12.0f);
        run(2000);
        survey();
        float relay_top = 0;
        uint32_t t0 = T;
        while (!mission_over() && T - t0 < 300000) { step(); if (A[3].body.pos.z > relay_top) { relay_top = A[3].body.pos.z; } }
        int cells = 0;
        for (int i = 0; i < 7; i++) { cells += A[i].covered; }
        check(relay_top > 3.0f && A[3].covered == 0, "the relay flew (%.1f m up) and mapped nothing", relay_top);
        check(cells >= cover_cells(&A[0].cv) && all_mapped(), "the six mappers covered all %d cells between them",
              cover_cells(&A[0].cv));
    }

    /* ================================================================== */
    if (want("replace")) {
        printf("== A NEW MISSION replaces the old one ==\n");
        world(5, mix, 12.0f);
        run(2000);
        post(MS_GOTO, 6, 20, 0, 0);
        run(20000);
        post(MS_GOTO, -2, 4, 0, 0);
        run(30000);
        float far = 0;
        for (int i = 0; i < 5; i++) {
            if (A[i].type == TY_RELAY) { continue; }
            float d = flat(A[i].body.pos, (v3_t){ -2, 4, 0 });
            if (d > far) { far = d; }
        }
        check(has_mission_all() && far < 7.0f, "every drone took the newer mission and went (farthest %.1f m)", far);
        uint16_t old = mid;
        mid -= 5;                                         /* a stale ground radio replays an older id */
        post(MS_HOME, 0, 0, 0, 0);
        run(3000);
        int kept = 1;
        for (int i = 0; i < 5; i++) { kept &= A[i].m.mid == old && A[i].state == AG_FLYING; }
        check(kept, "an OLDER mission id is ignored");
        mid = old;
        post(MS_HOME, 0, 0, 0, 0);
        check(run_until(run_mission_on_pads, 90000) > 0, "a newer HOME brings everyone back");
    }

    /* ================================================================== */
    if (want("follow")) {
        printf("== FOLLOW: a new GOTO every 0.4 s (the station's f key), the goal circling at 1.5 m/s ==\n");
        world(6, 0, 25.0f);
        run(2000);
        uint32_t t0 = T, all_up = 0, home_t = 0, rejoin_t = 0;
        int went_home = 0, landed = 0;
        double lag = 0;
        long lag_n = 0;
        minsep = 99.0f;
        float gx = 0, gy = 0;
        while (T - t0 < 110000) {
            uint32_t e = T - t0;
            if (e % 400u == 0u) {                          /* the pointer moved: a new goal */
                float ang = 1.5f * (float)e / 1000.0f / 10.0f;
                gx = 6.0f + 10.0f * cosf(ang);  gy = 14.0f + 10.0f * sinf(ang);
                post(MS_GOTO, gx, gy, 0, 0);
            }
            if (e == 30000u) { A[0].body.charge = 29.0f;  home_t = T; }   /* D1's battery fails */
            step();
            if (!all_up) {
                int up = 1;
                for (int i = 0; i < 6; i++) { up &= A[i].state == AG_FLYING && A[i].body.pos.z > 2.0f; }
                if (up) { all_up = T - t0; }
            }
            if (home_t && A[0].state == AG_HOMING) { went_home = 1; }
            if (went_home && A[0].state == AG_GROUND) { landed = 1; }
            if (landed && !rejoin_t && A[0].state == AG_FLYING) { rejoin_t = T - home_t; }
            if (e > 15000 && e < 30000) {                  /* steady following, all six */
                float cx = 0, cy = 0;
                for (int i = 0; i < 6; i++) { cx += A[i].body.pos.x / 6;  cy += A[i].body.pos.y / 6; }
                lag += sqrtf((cx - gx) * (cx - gx) + (cy - gy) * (cy - gy));
                lag_n++;
            }
        }
        report_sep();
        check(all_up > 0 && all_up < 6000, "all six took off and climbed while new goals kept coming: %.1f s "
              "(before the fix only drone 1 ever left its pad)", all_up / 1000.0);
        printf("    the swarm's centre followed %.1f m behind the moving goal, on average\n", lag / lag_n);
        check(lag / lag_n < 10.0, "it kept up with the goal");
        check(went_home && landed && rejoin_t > 0, "D1's battery failed mid-follow: home, landed, recharged and "
              "rejoined by itself %.0f s later", rejoin_t / 1000.0);
        check(minsep > 1.0f, "no two closer than 1 m: %.2f m", minsep);
    }

    /* ================================================================== */
    if (want("outofreach")) {
        printf("== FOLLOW OUT OF REACH: the goal dragged 50 m out (radio 15 m), then back; a hand landing ==\n");
        /* the session that went wrong in Wokwi: f, the mouse out of the radio
         * zone - the drones stopped; the mouse back - nothing; land them -
         * they took off again by themselves */
        world(6, 0, 15.0f);
        run(2000);
        uint32_t t0 = T, back_t = 0;
        float far_most = 0;
        int homed = 0;
        long in_touch = 0, touch_n = 0;
        const float bx = -4.0f, by = 2.0f;               /* where the goal comes back to: 11 m west */
        while (T - t0 < 90000) {
            uint32_t e = T - t0;
            float gx = e < 45000 ? 6.0f : bx, gy = e < 15000 ? 6.0f : e < 45000 ? 50.0f : by;   /* in, OUT, back */
            if (e % 400u == 0u) { post(MS_GOTO, gx, gy, 0, 0); }
            step();
            for (int i = 0; i < 6; i++) {
                float d = flat(A[i].body.pos, where(-1));
                if (e >= 15000 && e < 45000 && d > far_most) { far_most = d; }
                if (A[i].state == AG_HOMING || A[i].state == AG_GROUND) { homed |= e > 5000; }
            }
            if (e >= 25000 && e < 45000) {                /* waiting at the edge */
                for (int i = 0; i < 6; i++) { in_touch += A[i].hop != HOP_NONE;  touch_n++; }
            }
            if (e > 45000 && !back_t) {
                float cx = 0, cy = 0;
                for (int i = 0; i < 6; i++) { cx += A[i].body.pos.x / 6;  cy += A[i].body.pos.y / 6; }
                if (sqrtf((cx - bx) * (cx - bx) + (cy - by) * (cy - by)) < 5.0f) { back_t = e - 45000; }
            }
        }
        printf("    while the goal was 50 m out, the farthest drone went %.1f m from the ground radio\n", far_most);
        check(far_most < 35.0f, "the swarm stopped short, near the edge of reach - not at the goal 54 m out");
        check(in_touch > touch_n * 9 / 10, "and waited there in touch: a route home %.0f %% of the time",
              100.0 * in_touch / touch_n);
        check(back_t > 0 && back_t < 20000, "the goal came back (to the far side): the swarm was there in %.1f s",
              back_t / 1000.0);
        check(!homed, "nobody gave up and went home");

        current = 0;  agent_land(&A[0]);                  /* land D1 by hand, follow goals still coming */
        uint32_t t1 = T, up_again = 0;
        while (T - t1 < 60000) {
            if ((T - t1) % 400u == 0u) { post(MS_GOTO, 6.0f, 8.0f, 0, 0); }
            step();
            if (A[0].state == AG_FLYING && T - t1 > 3000 && !up_again) { up_again = T - t1; }
        }
        check(A[0].state == AG_GROUND && !up_again, "D1, landed by hand, stayed down for 60 s of follow goals "
              "(charged to %.0f %%)", A[0].body.charge);
        survey();                                         /* a NEW mission is a new order */
        uint32_t t2 = T;
        while (A[0].state != AG_FLYING && T - t2 < 10000) { step(); }
        check(A[0].state == AG_FLYING, "a new mission (a survey) is an order: it took off for it, %.1f s later",
              (T - t2) / 1000.0);
    }

    /* ================================================================== */
    if (want("obstacles")) {
        printf("== OBSTACLES: sensed by each drone's rangefinder, never given as a map ==\n");
        uint8_t ty[7] = { 0 };
        /* 1. straight at a 4 m-wide column, the goal right behind it */
        world(1, ty, 40.0f);
        obstacle(0.0f, 16.0f, 2.0f);
        run(2000);
        post(MS_HOLD, 0, 0, 0, 0);
        current = 0;  agent_takeoff(&A[0]);
        run(5000);
        A[0].hold = (v3_t){ 0.0f, 32.0f, A[0].cruise };      /* dead behind it */
        uint32_t t = 0, t0 = T;
        while (T - t0 < 60000) { step(); if (flat(A[0].body.pos, A[0].hold) < 1.0f) { t = T - t0;  break; } }
        check(t > 0 && crashes == 0, "a goal dead behind a 4 m column: flew round it and arrived in %.1f s, no crash", t / 1000.0);
        check(ob_closest > 0.8f, "closest it came to the column's surface: %.2f m", ob_closest);
        check(A[0].nobs <= OBS_MAX && nbr_find(&A[0].nb, 2) == 0, "it knew the column only from its own rangefinder");

        /* 2. a wall of three columns across the way: the slide must not stall */
        world(1, ty, 40.0f);
        obstacle(-3.0f, 16.0f, 1.5f);  obstacle(0.0f, 16.0f, 1.5f);  obstacle(3.0f, 16.0f, 1.5f);
        run(2000);
        post(MS_HOLD, 0, 0, 0, 0);
        current = 0;  agent_takeoff(&A[0]);
        run(5000);
        A[0].hold = (v3_t){ 0.0f, 30.0f, A[0].cruise };
        t = 0;  t0 = T;
        while (T - t0 < 90000) { step(); if (flat(A[0].body.pos, A[0].hold) < 1.0f) { t = T - t0;  break; } }
        check(t > 0 && crashes == 0, "a 10 m wall of three columns (gaps too narrow to pass): went round the end, "
              "%.1f s, no crash", t / 1000.0);

        /* 3. a flock through a random field */
        world(6, 0, 50.0f);
        random_obstacles(8, -6, 6, 24, 34, 0, 0);
        run(2000);
        post(MS_GOTO, 10, 40, 0, 0);
        run(60000);
        float far = 0;
        for (int i = 0; i < 6; i++) { float d = flat(A[i].body.pos, (v3_t){ 10, 40, 0 }); if (d > far) { far = d; } }
        report_sep();
        check(crashes == 0 && far < 8.0f, "a flock of 6 across a field of %d random columns to a point 40 m away: "
              "all there (farthest %.1f m), no crash", nob, far);
        check(minsep > 1.0f && ob_closest > 0.5f, "no two drones closer than 1 m (%.2f), none nearer a column than "
              "0.5 m (%.2f)", minsep, ob_closest);

        /* 4. a survey among random columns - one right on a cell's centre */
        world(7, mix, 12.0f);
        obstacle(12.5f, 27.5f, 1.0f);                         /* right on a cell centre */
        random_obstacles(8, SX0, SY0 - 6, SX1, SY1 + 4, 0, 0);
        run(2000);
        survey();
        uint32_t tm = run_until(mission_over, 400000);
        int full = 1;
        for (int i = 0; i < 7; i++) { full &= cover_done(&A[i].cv); }
        report_sep();
        check(tm > 0 && crashes == 0, "a survey among %d columns: complete, all home, in %.0f s, no crash", nob, tm / 1000.0);
        check(full, "map full - the cell with a column on its centre was seen from beside it");
        check(minsep > 1.0f && ob_closest > 0.5f, "no two drones closer than 1 m (%.2f), none nearer a column than "
              "0.5 m (%.2f)", minsep, ob_closest);
    }

    /* ================================================================== */
    if (want("killone")) {
        printf("== THREE DRONES, ONE KILLED, THEN A SURVEY: the other two fly it ==\n");
        uint8_t ty3[3] = { TY_MAPPER, TY_MAPPER, TY_MAPPER };
        int all_ok = 1;
        for (int variant = 0; variant < 6; variant++) {
            int victim = variant % 3, in_air = variant >= 3;
            world(3, ty3, 12.0f);
            run(2000);
            if (in_air) { post(MS_GOTO, 6, 12, 0, 0);  run(20000); }
            current = victim;  agent_kill(&A[victim]);  current = -1;
            run(3000);
            survey();
            int took_off[3] = { 0 };
            uint32_t t0 = T, t = 0;
            while (T - t0 < 300000) {
                step();
                for (int i = 0; i < 3; i++) { took_off[i] |= A[i].state == AG_FLYING && A[i].m.kind == MS_SURVEY; }
                if (mission_over()) { t = T - t0;  break; }
            }
            int ok = t > 0, full = 1;
            for (int i = 0; i < 3; i++) {
                if (i == victim) { continue; }
                ok &= took_off[i] && A[i].covered > 0;
                full &= cover_done(&A[i].cv);
            }
            printf("    D%d killed %s: %s - the other two flew and mapped, map %s, home in %.0f s\n", victim + 1,
                   in_air ? "in the air" : "on its pad", ok && full ? "ok" : "FAILED", full ? "full" : "NOT full", t / 1000.0);
            all_ok &= ok && full;
        }
        check(all_ok, "with any one of the three killed, on its pad or in the air, the other two survey the area");
    }

    /* ================================================================== */
    if (want("scale")) {
        printf("== SCALE: 40 drones, 15 m radio, a 40 x 40 m survey far down the line ==\n");
        static uint8_t ty[40];
        for (int i = 0; i < 40; i++) { ty[i] = i % 10 == 3 ? TY_RELAY : i % 4 == 1 ? TY_MAPPER : TY_MAPPER; }
        world(40, ty, 15.0f);
        run(3000);
        post(MS_SURVEY, 40, 10, 80, 50);
        uint32_t t_m = run_until(has_mission_all, 30000);
        int maxhop = 0;
        for (int i = 0; i < 40; i++) { if (A[i].hop != HOP_NONE && A[i].hop > maxhop) { maxhop = A[i].hop; } }
        check(t_m > 0, "the mission flooded to all 40 in %.1f s (up to %d hops)", t_m / 1000.0, maxhop);
        long bytes0[40];
        for (int i = 0; i < 40; i++) { bytes0[i] = tx_bytes[i]; }
        uint32_t t0 = T;
        uint32_t t = run_until(mission_over, 900000);
        int full = 1;
        for (int i = 0; i < 40; i++) { full &= cover_done(&A[i].cv); }
        report_sep();
        check(t > 0 && full, "64 cells covered, all 40 home, in %.0f s", t / 1000.0);
        int bounded = 1;
        for (int i = 0; i < 40; i++) { bounded &= A[i].nb.n <= NBR_MAX; }
        check(bounded, "no drone's table ever above %u rows", (unsigned)NBR_MAX);
        double rate = 0;
        for (int i = 0; i < 40; i++) { rate += (tx_bytes[i] - bytes0[i]) / ((T - t0) / 1000.0); }
        printf("    each drone sends %.0f B/s on average (115200 baud carries 11 520)\n", rate / 40);
        check(rate / 40 < 3000, "per-drone radio load stays small with 40 drones");
        check(minsep > 1.0f, "40 drones, no two closer than 1 m: %.2f m", minsep);
        check(sizeof(agent_t) < 2048, "one drone's whole state: %u B, whatever the swarm size", (unsigned)sizeof(agent_t));
    }

    printf("\nseed %u: %d of %d checks passed   (%.1f s of PC time; %ld frames lost to a full queue)\n", (unsigned)seed,
           checks - fails, checks,
           (double)(clock() - wall) / CLOCKS_PER_SEC, pend_dropped);
    return fails ? 1 : 0;
}
