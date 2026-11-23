/*
 * pilot.c - one drone's flight computer.  See pilot.h.
 *
 * Each pilot step (20 ms) runs four stages, as lesson 14's control_step()
 * does, but on the pilot it was handed:
 *
 *   believe   accelerometer dead-reckoning, pulled toward 200 ms-old GPS
 *   guard     a flat battery outranks the ground station
 *   aim       mode -> wanted velocity (feed-forward + feedback)
 *   steer     velocity PI -> acceleration -> thrust, roll, pitch
 */
#include <string.h>
#include "pilot.h"
#include "craft.h"            /* FIX_LAG, FIX_EVERY: the GPS's timing        */
#include "fmath.h"
#include "trig.h"

static const char *const mode_text[PM_COUNT] = { "IDLE", "FOLLOW", "HOVER", "LAND" };

const char *pilot_mode_name(uint8_t mode) { return mode < PM_COUNT ? mode_text[mode] : "?"; }

static float limit(float x, float lim) { return f_clamp(x, -lim, lim); }

static void switch_to(pilot_t *p, uint8_t mode, uint8_t why)
{
    p->mode = mode;  p->why = why;  p->mode_ms = p->now_ms;  p->quiet_ms = p->now_ms;
    if (mode == PM_IDLE) { p->sum = (v3_t){ 0 }; }
    if (mode == PM_HOVER || mode == PM_LAND) {
        p->goal_v = (v3_t){ 0 };                 /* stop where the goal is now */
    }
}

void pilot_init(pilot_t *p)
{
    memset(p, 0, sizeof *p);
    p->k = (pilot_tune_t){
        .k_pos = 1.2f, .k_vel = 2.2f, .k_int = 0.6f,
        .k_alt = 1.2f, .k_climb = 3.0f, .k_climb_int = 1.0f,
        .fix_speed = 2.5f, .tilt = 0.44f,           /* ~25 degrees            */
        .gps_a = 0.25f, .gps_b = 0.04f, .low_charge = 10.0f,
    };
    p->mode = PM_IDLE;  p->why = PW_START;
    p->charge = 100.0f;
    pilot_retune(p);
}

void pilot_retune(pilot_t *p)
{
    float sn, cs;
    t_sincos(p->k.tilt, &sn, &cs);
    p->reach = G0 * sn / cs;                                /* g tan(tilt)     */
    p->cap   = 0.8f * p->reach / p->k.k_int;                /* anti-windup     */
    p->cap_z = 3.0f / p->k.k_climb_int;
}

int pilot_arm(pilot_t *p)
{
    if (p->mode != PM_IDLE) { return PILOT_E_MODE; }
    if (p->fixes < 10u)     { return PILOT_E_GPS; }   /* a second of GPS first */
    p->goal   = (v3_t){ p->est.x, p->est.y, 0.0f };   /* "stay on the pad"     */
    p->goal_v = (v3_t){ 0 };
    p->goal_ms = p->now_ms;
    switch_to(p, PM_FOLLOW, PW_CMD);
    return PILOT_OK;
}

int pilot_land(pilot_t *p)
{
    if (p->mode == PM_IDLE) { return PILOT_E_MODE; }
    if (p->mode != PM_LAND) { switch_to(p, PM_LAND, PW_CMD); }
    return PILOT_OK;
}

void pilot_goal(pilot_t *p, v3_t at, v3_t vel)
{
    if (p->mode == PM_LAND || p->mode == PM_IDLE) { return; }   /* too late    */
    p->goal = at;  p->goal_v = vel;  p->goal_ms = p->now_ms;
    if (p->mode == PM_HOVER) { switch_to(p, PM_FOLLOW, PW_FRESH); }
}

/* ---- believe ------------------------------------------------------------ */
static void believe(pilot_t *p, const meas_t *m)
{
    const float dt = PILOT_DT;

    /* predict from the accelerometer, and remember the prediction */
    p->est_v.x += m->acc.x * dt;      p->est_v.y += m->acc.y * dt;
    p->est.x   += p->est_v.x * dt;    p->est.y   += p->est_v.y * dt;
    uint8_t here = p->past_at;
    p->past_x[here] = p->est.x;       p->past_y[here] = p->est.y;
    p->past_at = (uint8_t)((here + 1u) % PILOT_PAST);

    if (m->fix) {
        if (p->fixes == 0u) {
            /* the first fix: nothing to compare it with, so take it */
            p->est.x = m->fix_x;  p->est.y = m->fix_y;  p->est_v.x = p->est_v.y = 0.0f;
            for (unsigned k = 0; k < PILOT_PAST; k++) { p->past_x[k] = p->est.x; p->past_y[k] = p->est.y; }
        } else {
            /* The fix describes the drone FIX_LAG steps ago: measure the
             * surprise against what was believed THEN, correct now by it,
             * and shift the remembered past too so the next fix is judged
             * against a history that already knows this one. */
            uint8_t then = (uint8_t)((here + PILOT_PAST - FIX_LAG) % PILOT_PAST);
            float sx = m->fix_x - p->past_x[then];
            float sy = m->fix_y - p->past_y[then];
            float a = p->k.gps_a;
            float b = p->k.gps_b / ((float)FIX_EVERY * PILOT_DT);
            p->est.x   += a * sx;    p->est.y   += a * sy;
            p->est_v.x += b * sx;    p->est_v.y += b * sy;
            for (unsigned k = 0; k < PILOT_PAST; k++) { p->past_x[k] += a * sx; p->past_y[k] += a * sy; }
        }
        if (p->fixes < 255u) { p->fixes++; }
    }

    /* height: a complementary filter, baro for the average, IMU for the fast */
    float r = m->baro - p->est.z;
    p->est_v.z += (m->acc.z + 4.0f * r) * dt;
    p->est.z   += (p->est_v.z + 2.8f * r) * dt;
    p->heading  = m->heading;
}

/* ---- aim: the mode becomes a wanted velocity ----------------------------- */
static v3_t aim(pilot_t *p)
{
    v3_t want = { 0 };
    float gx = p->goal.x - p->est.x, gy = p->goal.y - p->est.y;

    if (p->mode == PM_LAND) {
        /* straight down over the goal; slow for the last 2.5 m */
        float cx = p->k.k_pos * gx, cy = p->k.k_pos * gy;
        float c  = t_sqrt(cx * cx + cy * cy);
        if (c > p->k.fix_speed) { cx *= p->k.fix_speed / c;  cy *= p->k.fix_speed / c; }
        want = (v3_t){ cx, cy, p->est.z > 2.5f ? -1.0f : -0.4f };
        return want;
    }

    /* FOLLOW and HOVER: feed-forward plus a LIMITED feedback correction.
     * The limit is on the correction only - the goal's own speed always
     * passes - so a drone that fell behind catches up at goal_v + fix_speed. */
    float cx = p->k.k_pos * gx, cy = p->k.k_pos * gy;
    float c  = t_sqrt(cx * cx + cy * cy);
    if (c > p->k.fix_speed) { cx *= p->k.fix_speed / c;  cy *= p->k.fix_speed / c; }
    want.x = p->goal_v.x + cx;
    want.y = p->goal_v.y + cy;
    /* vertically the same: the goal's climb rate plus at most 1 m/s of
     * correction, then the airframe's own limits */
    float cz = limit(p->k.k_alt * (p->goal.z - p->est.z), 1.0f);
    want.z = f_clamp(p->goal_v.z + cz, -2.5f, 3.0f);
    return want;
}

/* ---- the step ------------------------------------------------------------ */
void pilot_step(pilot_t *p, const meas_t *m, cmd_t *out)
{
    const float dt = PILOT_DT;
    p->now_ms = m->t_ms;
    p->charge = m->charge;
    believe(p, m);

    if (p->mode == PM_IDLE) {
        *out = (cmd_t){ 0 };
        p->last = *out;
        return;
    }

    /* guard */
    if (p->charge < p->k.low_charge && p->mode != PM_LAND) { switch_to(p, PM_LAND, PW_BATTERY); }

    /* the goal moves on by itself between updates; a silent ground station
     * freezes it, and the drone hovers where the goal stopped */
    if (p->mode == PM_FOLLOW && p->now_ms - p->goal_ms > PILOT_STALE_MS) {
        switch_to(p, PM_HOVER, PW_STALE);
    }
    p->goal.x += p->goal_v.x * dt;
    p->goal.y += p->goal_v.y * dt;
    p->goal.z += p->goal_v.z * dt;

    /* On the pad with the goal on the pad: motors idle, nothing integrates.
     * Without this the height integrator creeps and the drone hops. */
    if (p->mode == PM_FOLLOW && p->goal.z < 0.3f && p->est.z < 0.3f) {
        p->sum = (v3_t){ 0 };
        *out = (cmd_t){ .live = 1 };
        p->last = *out;
        return;
    }

    /* landed = low and not moving vertically for 1.5 s */
    if (p->mode == PM_LAND) {
        float vz = p->est_v.z < 0.0f ? -p->est_v.z : p->est_v.z;
        if (!(p->est.z < 0.5f && vz < 0.3f)) { p->quiet_ms = p->now_ms; }
        if (p->now_ms - p->quiet_ms > 1500u) {
            switch_to(p, PM_IDLE, PW_LANDED);
            *out = (cmd_t){ 0 };
            p->last = *out;
            return;
        }
    }

    /* steer: velocity PI.  In a steady wind the integral ends up holding the
     * tilt INTO the wind - read k_int * sum and you read the wind. */
    v3_t want = aim(p);
    float reach = p->reach;                                    /* g tan(tilt)  */
    v3_t gap = { want.x - p->est_v.x, want.y - p->est_v.y, want.z - p->est_v.z };
    if (p->est.z > 0.5f) {
        p->sum.x = limit(p->sum.x + gap.x * dt, p->cap);       /* anti-windup  */
        p->sum.y = limit(p->sum.y + gap.y * dt, p->cap);
    }
    p->sum.z = limit(p->sum.z + gap.z * dt, p->cap_z);

    float ax = p->k.k_vel * gap.x + p->k.k_int * p->sum.x;
    float ay = p->k.k_vel * gap.y + p->k.k_int * p->sum.y;
    float az = f_clamp(p->k.k_climb * gap.z + p->k.k_climb_int * p->sum.z, -0.5f * G0, 0.6f * G0);
    float ah = t_sqrt(ax * ax + ay * ay);
    if (ah > reach) { ax *= reach / ah;  ay *= reach / ah; }

    /* the thrust vector must point along (ax, ay, g + az): split it into the
     * body's forward and right, then into angles */
    float ch, sh;
    t_sincos(p->heading, &sh, &ch);
    float fwd  = ax * ch + ay * sh;
    float side = ax * sh - ay * ch;
    float up   = G0 + az;
    out->live = 1;
    out->lift = t_sqrt(fwd * fwd + side * side + up * up);
    out->nose = t_atan2(fwd, up);
    out->bank = t_atan2(side, t_sqrt(fwd * fwd + up * up));
    out->turn = limit(-2.0f * t_wrap(p->heading), 1.2f);    /* keep facing east */
    p->last = *out;
}
