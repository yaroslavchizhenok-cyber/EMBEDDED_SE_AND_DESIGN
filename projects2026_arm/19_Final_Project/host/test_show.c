/*
 * test_show.c - the firmware's show, flown on the PC.  Exit code 0 = all passed.
 *
 * The SAME plan.c, director.c, fleet.c, pilot.c, craft.c, show.c and fmt.c
 * the chip runs - only Main.c (the RTOS tasks, the UART) is left out, and the
 * director's output lands in emit() below instead of the serial port.  So a
 * show that passes here passes on the chip, only faster: the chip in Wokwi
 * flies it in slow motion, this flies it in well under a second.
 *
 *   test_show            every check
 *   test_show --print    also print what the chip would print (plan, score)
 *   test_show --frames   ONLY fly the show and print every line the chip
 *                        would send, frames included - a capture to play
 *                        into the station without Wokwi:
 *       out/test_show --frames > out/show.txt
 *       uv run tools/station/fakeboard.py --replay <that file> --rate 100
 *       uv run tools/station/station.py --port socket://localhost:4001 --view swarm
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "director.h"
#include "fmt.h"
#include "proto.h"
#include "pilot.h"

static int failures, checks, verbose, raw;

#define CHECK(ok, ...) do { checks++; printf("  %s  ", (ok) ? "ok  " : "FAIL"); printf(__VA_ARGS__); \
                            printf("\n"); if (!(ok)) { failures++; } } while (0)

/* ---- the director's output, captured ------------------------------------- */
static struct { int lines, frames, bad, sco, drn[FLEET_MAX], swm, sep; char sco_line[96]; } out;

static void emit(const char *line, int must)
{
    (void)must;
    out.lines++;
    if (raw) { fputs(line, stdout);  return; }
    if (line[0] != '$') {
        if (verbose) { fputs(line, stdout); }
        return;
    }
    char buf[160];
    size_t n = strlen(line);
    if (n < 3 || n >= sizeof buf || line[n - 2] != '\r' || line[n - 1] != '\n') { out.bad++; return; }
    memcpy(buf, line, n - 2);
    buf[n - 2] = '\0';
    const char *body;
    size_t len;
    if (proto_check(buf, &body, &len) != PROTO_OK) { out.bad++; return; }
    out.frames++;
    if (!strncmp(body, "SCO,", 4)) { out.sco++; snprintf(out.sco_line, sizeof out.sco_line, "%s", buf); }
    if (!strncmp(body, "SWM,", 4)) { out.swm++; }
    if (!strncmp(body, "SEP,", 4)) { out.sep++; }
    if (!strncmp(body, "DRN,", 4)) {
        int i = atoi(body + 4);
        if (i >= 0 && i < FLEET_MAX) { out.drn[i]++; }
    }
}

/* Fly a director from the pads to DONE (or give up after `limit` ticks). */
static long fly(director_t *d, long limit)
{
    long ticks = 0;
    director_start(d);
    while (d->state != DR_DONE && ticks < limit) {
        if (director_tick(d)) { ticks++; }
        else if (d->state == DR_READY) { director_start(d); }
        else { break; }
    }
    return ticks;
}

static float dist(v3_t a, v3_t b) { return sqrtf((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z)); }

/* the closest any two drones are ever planned, sampled 5x finer than plan.c checks */
static float finest_pass(const plan_t *p)
{
    float best = 1e9f;
    for (int k = 0; k < p->n_stages; k++) {
        const stage_t *s = &p->st[k];
        if (s->kind == ST_PADS || s->kind == ST_LAND) { continue; }
        for (float t = s->t0; t <= s->t0 + s->T; t += 0.02f) {
            v3_t at[FLEET_MAX], v;
            for (int i = 0; i < p->n; i++) { plan_sample(p, s, i, t, &at[i], &v); }
            for (int i = 0; i < p->n; i++) {
                for (int j = i + 1; j < p->n; j++) {
                    float dd = dist(at[i], at[j]);
                    if (dd < best) { best = dd; }
                }
            }
        }
    }
    return best;
}

/* ---- 1. fmt.c against the C library's snprintf --------------------------- */
static void test_fmt(void)
{
    printf("1. fmt.c, the small snprintf\n");
    char a[64], b[64];
    int bad = 0, n = 0;
#define SAME(...) do { fmt(a, sizeof a, __VA_ARGS__); snprintf(b, sizeof b, __VA_ARGS__); n++; \
                       if (strcmp(a, b)) { bad++; printf("    fmt \"%s\" vs libc \"%s\"\n", a, b); } } while (0)
    SAME("%d|%i|%u", -42, 7, 3000000000u);
    SAME("%ld|%lu|%5d|%-5d|%05d|%-8s|%8s", -123456789L, 4000000000UL, 42, 42, -42, "ab", "cd");
    SAME("%02X|%x|%X|%c|%%|%s", 0x0A, 255u, 0xBEEFu, 'q', "end");
    SAME("$%s*%02X", "SWM,1,2", 0x5D);
    SAME("%3lu/%lus|%7s m|%-10s|", 57ul, 126ul, "0.19", "heart");
    SAME("%d %d", 0, -2147483647 - 1);
#undef SAME
    fmt(a, 6, "%s", "truncated");
    CHECK(bad == 0, "%d formats agree with the C library's snprintf", n);
    CHECK(!strcmp(a, "trunc"), "truncates to the buffer and ends it: \"%s\"", a);
}

/* ---- 2. the Hungarian algorithm against brute force ------------------------ */
static float total(const v3_t *f, const v3_t *t, const uint8_t *who, int n)
{
    float s = 0.0f;
    for (int i = 0; i < n; i++) { float d = dist(f[i], t[who[i]]);  s += d * d; }
    return s;
}

static void permute(uint8_t *p, int k, int n, const v3_t *f, const v3_t *t, float *best)
{
    if (k == n) { float s = total(f, t, p, n);  if (s < *best) { *best = s; }  return; }
    for (int i = k; i < n; i++) {
        uint8_t x = p[k];  p[k] = p[i];  p[i] = x;
        permute(p, k + 1, n, f, t, best);
        x = p[k];  p[k] = p[i];  p[i] = x;
    }
}

static void test_assign(void)
{
    printf("2. who goes where (plan_assign)\n");
    srand(3050);
    int bad = 0;
    for (int c = 0; c < 300; c++) {
        int n = 1 + rand() % FLEET_MAX;
        v3_t f[FLEET_MAX], t[FLEET_MAX];
        for (int i = 0; i < n; i++) {
            f[i] = (v3_t){ (float)(rand() % 400) / 10.0f, (float)(rand() % 400) / 10.0f, (float)(rand() % 200) / 10.0f };
            t[i] = (v3_t){ (float)(rand() % 400) / 10.0f, (float)(rand() % 400) / 10.0f, (float)(rand() % 200) / 10.0f };
        }
        uint8_t who[FLEET_MAX], p[FLEET_MAX], seen[FLEET_MAX] = { 0 };
        plan_assign(f, t, n, who);
        for (int i = 0; i < n; i++) { p[i] = (uint8_t)i;  if (who[i] < n) { seen[who[i]]++; } }
        float best = 1e30f;
        permute(p, 0, n, f, t, &best);
        int perm = 1;
        for (int i = 0; i < n; i++) { if (seen[i] != 1) { perm = 0; } }
        if (!perm || total(f, t, who, n) > best * 1.0001f + 1e-3f) { bad++; }
    }
    CHECK(bad == 0, "300 random cases up to %d drones: %d optimal", FLEET_MAX, 300 - bad);
}

/* ---- 3. shapes, the plan, refusal, repair ---------------------------------- */
static void test_plan(void)
{
    printf("3. the planner\n");
    static plan_t p;
    for (int sh = 0; sh < SH_POINTS; sh++) {
        int ok = 1;
        for (int n = 2; n <= FLEET_MAX; n++) {
            form_t f = { (uint8_t)sh, WALL, 5.0f, 0.0f, 10.0f, 12.0f, 14.0f, 0.0f, "t", 0 };
            v3_t s[FLEET_MAX];
            plan_shape(&f, n, s);
            for (int i = 0; i < n; i++) { for (int j = i + 1; j < n; j++) { if (dist(s[i], s[j]) < 0.5f) { ok = 0; } } }
        }
        CHECK(ok, "shape %d: 2..%d drones, every slot distinct", sh, FLEET_MAX);
    }

    CHECK(plan_build(&p, &SHOW, 0) == 0, "show.c's show '%s' plans (%s)", SHOW.name, p.why[0] ? p.why : "no refusal");
    float fine = finest_pass(&p);
    CHECK(fine >= SHOW.sep * 0.97f, "its closest planned pass, sampled every 20 ms: %.2f m (sep %.1f)", fine, SHOW.sep);
    printf("       %d stages, %.1f s, %d repair(s)\n", p.n_stages, plan_duration(&p), p.repairs);

    show_t tight = SHOW;
    tight.n_forms = 1;
    tight.forms[0] = (form_t){ SH_CIRCLE, WALL, 5.0f, 0.0f, 10.0f, 10.0f, 4.0f, 0.0f, "small", 0 };
    CHECK(plan_build(&p, &tight, 0) != 0 && strstr(p.why, "apart"), "a 4 m circle of 5 is refused: \"%s\"", p.why);

    static const v3_t row[5] = { {-6,10,8}, {-3,10,8}, {0,10,8}, {3,10,8}, {6,10,8} };
    static const v3_t col[5] = { {0,4,8}, {0,7,8}, {0,10,8}, {0,13,8}, {0,16,8} };
    show_t cross = SHOW;
    cross.n_forms = 2;
    cross.forms[0] = (form_t){ SH_POINTS, FLAT, 2.0f, 0, 0, 0, 0, 0, "row", row };
    cross.forms[1] = (form_t){ SH_POINTS, FLAT, 2.0f, 0, 0, 0, 0, 0, "column", col };
    int r = plan_build(&p, &cross, 0);
    CHECK(r == 0 && p.repairs > 0, "a row turning into a column across it: planned with %d repair(s)", p.repairs);
    fine = finest_pass(&p);
    CHECK(fine >= cross.sep * 0.97f, "  ...and the repaired plan's closest pass is %.2f m", fine);
}

/* ---- 4. the director flies the show ------------------------------------------- */
static void test_fly(void)
{
    printf("4. the show, flown (director.c, fleet of %d)\n", SHOW.drones);
    static director_t d;
    memset(&out, 0, sizeof out);
    CHECK(director_init(&d, &SHOW, emit) == 0, "director_init: planned, on the pads");
    if (verbose) { director_print_plan(&d); }
    long ticks = fly(&d, 400000);
    CHECK(d.state == DR_DONE, "it ends: %s after %.1f s of show", director_state_name(d.state), ticks * 0.01);
    CHECK(d.passed, "PASS: closest %.2f m (safe %.1f), landed %d of %d", d.closest, SHOW.safe, d.landed, SHOW.drones);
    CHECK(d.rms < 0.5f, "tracking RMS %.2f m, worst %.2f m (RTK GPS, %.0f m/s wind)", d.rms, d.max_err, SHOW.wind);
    int every = 1;
    for (int i = 0; i < SHOW.drones; i++) { if (out.drn[i] < out.swm - 2) { every = 0; } }
    CHECK(out.bad == 0 && every, "%d frames, every checksum good; %d $SWM, a $DRN for every drone with each", out.frames, out.swm);
    CHECK(out.sco == 1, "one $SCO: %s", out.sco_line);
    if (verbose) { director_report(&d); }

    float rms1 = d.rms, close1 = d.closest;
    director_reset(&d);
    fly(&d, 400000);
    CHECK(d.rms == rms1 && d.closest == close1, "flown again, seed 1: identical (%.4f m, %.4f m)", d.rms, d.closest);
    director_set_seed(&d, 2);
    fly(&d, 400000);
    CHECK(d.passed && d.rms != rms1, "seed 2: different gusts, different numbers (%.4f m), PASS", d.rms);
    director_set_seed(&d, 1);

    director_set_ff(&d, 0);
    fly(&d, 400000);
    CHECK(d.max_err > 2.0f * 0.58f, "without feed-forward the worst error grows: %.2f m", d.max_err);
    director_set_ff(&d, 1);

    director_set_gps(&d, 0);
    fly(&d, 400000);
    CHECK(d.rms > 2.0f * rms1, "with plain GPS (lesson 14's) the RMS grows: %.2f m", d.rms);
    director_set_gps(&d, 1);
}

/* ---- 5. pause and abort ---------------------------------------------------------- */
static void test_commands(void)
{
    printf("5. pause and abort\n");
    static director_t d;
    director_init(&d, &SHOW, emit);
    director_start(&d);
    while (d.state != DR_FLYING) { director_tick(&d); }
    for (int k = 0; k < 2000; k++) { director_tick(&d); }      /* 20 s in */
    uint32_t t = d.ticks;
    director_pause(&d);
    int moved = 0;
    for (int k = 0; k < 100; k++) { moved += director_tick(&d); }
    CHECK(moved == 0 && d.ticks == t, "paused: the show clock stands still");
    director_start(&d);                                         /* A carries on */
    director_tick(&d);
    CHECK(d.ticks == t + 1, "start (button A) carries on");
    director_abort(&d);
    long n = 0;
    while (d.state != DR_DONE && n < 100000) { director_tick(&d);  n++; }
    CHECK(d.state == DR_DONE && d.landed == SHOW.drones && !d.passed,
          "abort: all %d landed in %.1f s, and it does not PASS", d.landed, n * 0.01);
}

/* ---- 6. a station that connects late ------------------------------------------
 * The usual order is: start the simulator, THEN run.bat.  By the time the
 * station connects, the board is READY on the pads, where nothing moves - so
 * it must keep sending its picture (director_heartbeat), or the station
 * waits forever.  Main.c calls the heartbeat every 100 ms while idle. */
static int full_picture(director_t *d)
{
    memset(&out, 0, sizeof out);
    for (uint32_t beat = 0; beat < 10; beat++) { director_heartbeat(d, beat); }   /* one second */
    int all = out.swm == 1 && out.bad == 0;      /* no $SEP on the pads: nobody is airborne */
    for (int i = 0; i < d->plan.n; i++) { if (out.drn[i] != 1) { all = 0; } }
    return all;
}

static void test_late_station(void)
{
    printf("6. a station that connects late\n");
    static director_t d;
    director_init(&d, &SHOW, emit);
    while (d.state == DR_WARMUP) { director_tick(&d); }
    CHECK(d.state == DR_READY && director_tick(&d) == 0, "READY on the pads: the show itself sends nothing");
    int ok = full_picture(&d);
    CHECK(ok, "the heartbeat: within 1 s, the $SWM and a $DRN for each of %d drones", d.plan.n);
    fly(&d, 400000);
    memset(&out, 0, sizeof out);
    for (uint32_t beat = 0; beat < 10; beat++) { director_heartbeat(&d, beat); }
    CHECK(out.sco == 1 && out.swm == 1 && out.bad == 0, "DONE: the heartbeat repeats the picture and the verdict (%s)", out.sco_line);

    show_t tight = SHOW;
    tight.n_forms = 1;
    tight.forms[0] = (form_t){ SH_CIRCLE, WALL, 5.0f, 0.0f, 10.0f, 10.0f, 4.0f, 0.0f, "small", 0 };
    director_init(&d, &tight, emit);
    memset(&out, 0, sizeof out);
    for (uint32_t beat = 0; beat < 100; beat++) { director_heartbeat(&d, beat); }
    CHECK(out.lines == 2 && out.frames == 0, "REFUSED: the reason, as text, every 5 s (%d lines in 10 s)", out.lines);
}

int main(int argc, char **argv)
{
    verbose = argc > 1 && !strcmp(argv[1], "--print");
    raw = argc > 1 && !strcmp(argv[1], "--frames");
    if (raw) {                                   /* a capture, nothing else */
        static director_t d;
        if (director_init(&d, &SHOW, emit) != 0) { return 1; }
        fly(&d, 400000);
        return d.passed ? 0 : 1;
    }
    test_fmt();
    test_assign();
    test_plan();
    test_fly();
    test_commands();
    test_late_station();
    printf("\n%d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
