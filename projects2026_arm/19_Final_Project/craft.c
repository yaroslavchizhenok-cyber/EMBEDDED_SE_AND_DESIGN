/*
 * craft.c - one drone's airframe, air and sensors.  See craft.h.
 *
 * Constants are those of a 1-2 kg quadcopter, the same choices lesson 14's
 * world.c makes, so a drone of the swarm flies like the drone of lesson 14.
 */
#include "craft.h"
#include "fmath.h"
#include "trig.h"

#define LAG_TILT    0.08f      /* s  the attitude loop, seen from outside       */
#define LAG_LIFT    0.05f      /* s  motors and propellers spinning up          */
#define LAG_SPIN    0.10f      /* s                                             */
#define TILT_LIM    0.70f      /* rad, the airframe's own limit                 */
#define LIFT_LIM    (2.0f * G0)/* thrust-to-weight 2                            */
#define DRAG_SIDE   0.35f      /* 1/s  per m/s of airspeed                      */
#define DRAG_UP     0.50f      /* 1/s                                           */
#define GUST_LIFE   2.0f       /* s  how long a gust lasts                      */
#define GUST_SHARE  0.30f      /* gust size as a fraction of the steady wind    */
#define WANDER_LIFE 5.0f       /* s  how slowly the GPS error wanders           */
#define BARO_JITTER 0.20f      /* m                                             */
#define ACC_JITTER  0.20f      /* m/s^2                                         */
#define ACC_BIAS    0.03f      /* m/s^2  typical size of an IMU's bias          */
#define HEAD_JITTER 0.01f      /* rad                                           */
#define DRAIN       0.35f      /* %/s at hover: about 4.8 minutes               */

/* ---- dice ---------------------------------------------------------------
 * A 32-bit linear congruential generator (Knuth's MMIX-style constants from
 * Numerical Recipes).  Its LOW bits are poor, so only the top 24 are used.
 * Deterministic on purpose: one seed is one exact show, run after run. */
static uint32_t roll_dice(craft_t *c)
{
    c->dice = c->dice * 1664525u + 1013904223u;
    return c->dice >> 8;                       /* 24 good bits                  */
}

static float flat(craft_t *c)                  /* uniform, -1 .. +1             */
{
    return (float)roll_dice(c) * (2.0f / 16777216.0f) - 1.0f;
}

static float bell(craft_t *c)                  /* about normal, sigma 1         */
{
    /* Four uniforms (variance 1/3 each) sum to variance 4/3; scaling by
     * sqrt(3)/2 makes it 1.  The central limit theorem does the rest. */
    return (flat(c) + flat(c) + flat(c) + flat(c)) * 0.8660254f;
}

/* first-order lag: `now` moves toward `want` with time constant tau */
static float chase(float now, float want, float dt, float tau)
{
    return now + (want - now) * (dt / tau);
}

void craft_init(craft_t *c, uint32_t seed, float x, float y)
{
    *c = (craft_t){ 0 };
    c->dice     = seed ? seed : 0x9E3779B9u;
    c->pos      = (v3_t){ x, y, 0.0f };
    c->charge   = 100.0f;
    c->grounded = 1;
    c->gps      = GPS_PLAIN;
    c->bias_x   = ACC_BIAS * bell(c);           /* every IMU is a bit off,    */
    c->bias_y   = ACC_BIAS * bell(c);           /* each in its own way        */
    for (unsigned k = 0; k <= FIX_LAG; k++) { c->trail_x[k] = x;  c->trail_y[k] = y; }
}

void craft_step(craft_t *c, const sky_t *sky, const cmd_t *cmd, float dt)
{
    /* 1. the rotors and the attitude loop answer the command, late */
    int   on   = cmd->live && c->charge > 0.0f;
    float want_lift = on ? f_clamp(cmd->lift, 0.0f, LIFT_LIM) : 0.0f;
    float want_bank = on ? f_clamp(cmd->bank, -TILT_LIM, TILT_LIM) : 0.0f;
    float want_nose = on ? f_clamp(cmd->nose, -TILT_LIM, TILT_LIM) : 0.0f;
    float want_spin = on ? cmd->turn : 0.0f;
    c->lift    = chase(c->lift, want_lift, dt, LAG_LIFT);
    c->bank    = chase(c->bank, want_bank, dt, LAG_TILT);
    c->nose    = chase(c->nose, want_nose, dt, LAG_TILT);
    c->spin    = chase(c->spin, want_spin, dt, LAG_SPIN);
    c->heading = t_wrap(c->heading + c->spin * dt);

    /* 2. the air: the shared steady wind plus this drone's own gust.  Each
     * gust axis relaxes toward zero over GUST_LIFE and is kicked by noise
     * sized so it settles at GUST_SHARE of the steady wind. */
    float spread = GUST_SHARE * sky->speed * t_sqrt(2.0f * dt / GUST_LIFE);
    c->gust_x += -c->gust_x * (dt / GUST_LIFE) + spread * bell(c);
    c->gust_y += -c->gust_y * (dt / GUST_LIFE) + spread * bell(c);
    c->air_x   = sky->speed * t_cos(sky->toward) + c->gust_x;
    c->air_y   = sky->speed * t_sin(sky->toward) + c->gust_y;

    /* 3. forces.  In the body: forward = along the nose, side = to the right. */
    float cb = t_cos(c->bank), sb = t_sin(c->bank);
    float cn = t_cos(c->nose), sn = t_sin(c->nose);
    float ch = t_cos(c->heading), sh = t_sin(c->heading);
    float fwd  = c->lift * sn * cb;
    float side = c->lift * sb;
    float up   = c->lift * cn * cb;
    v3_t a = {
        fwd * ch + side * sh - DRAG_SIDE * (c->vel.x - c->air_x),
        fwd * sh - side * ch - DRAG_SIDE * (c->vel.y - c->air_y),
        up - G0 - DRAG_UP * c->vel.z,
    };

    /* 4. resting on the ground until the rotors can lift it */
    if (c->grounded) {
        if (a.z > 0.0f) {
            c->grounded = 0;
        } else {
            a = (v3_t){ 0 };
            c->vel = (v3_t){ 0 };
        }
    }

    /* 5. semi-implicit Euler: velocity first, then position with it */
    c->vel.x += a.x * dt;  c->vel.y += a.y * dt;  c->vel.z += a.z * dt;
    c->pos.x += c->vel.x * dt;  c->pos.y += c->vel.y * dt;  c->pos.z += c->vel.z * dt;
    if (!c->grounded && c->pos.z <= 0.0f) {          /* touchdown              */
        if (-c->vel.z > c->hardest_landing) { c->hardest_landing = -c->vel.z; }
        c->pos.z = 0.0f;
        c->vel = (v3_t){ 0 };
        c->grounded = 1;
    }
    c->acc = a;

    /* 6. the battery: momentum theory says power ~ thrust^1.5 */
    float load = c->lift / G0;
    c->charge -= DRAIN * load * t_sqrt(load) * dt;
    if (c->charge < 0.0f) { c->charge = 0.0f; }

    c->t_ms += (uint32_t)(dt * 1000.0f + 0.5f);
}

void craft_sense(craft_t *c, meas_t *m)
{
    const float dt = PILOT_DT;
    m->t_ms    = c->t_ms;
    m->acc.x   = c->acc.x + c->bias_x + ACC_JITTER * bell(c);
    m->acc.y   = c->acc.y + c->bias_y + ACC_JITTER * bell(c);
    m->acc.z   = c->acc.z + ACC_JITTER * bell(c);
    m->heading = t_wrap(c->heading + HEAD_JITTER * bell(c));
    m->baro    = c->pos.z + BARO_JITTER * bell(c);
    m->charge  = c->charge;

    /* GPS: write where the drone is now into the ring; the slot after it is
     * then where it was FIX_LAG steps ago - that is what the receiver reports. */
    c->trail_x[c->trail_at] = c->pos.x;
    c->trail_y[c->trail_at] = c->pos.y;
    c->trail_at = (uint8_t)((c->trail_at + 1u) % (FIX_LAG + 1u));
    float kick = c->gps.wander * t_sqrt(2.0f * dt / WANDER_LIFE);
    c->drift_x += -c->drift_x * (dt / WANDER_LIFE) + kick * bell(c);
    c->drift_y += -c->drift_y * (dt / WANDER_LIFE) + kick * bell(c);

    m->fix = (uint8_t)(c->samples >= FIX_LAG && c->samples % FIX_EVERY == 0u);
    if (m->fix) {
        m->fix_x = c->trail_x[c->trail_at] + c->drift_x + c->gps.jitter * bell(c);
        m->fix_y = c->trail_y[c->trail_at] + c->drift_y + c->gps.jitter * bell(c);
    }
    c->samples++;
}
