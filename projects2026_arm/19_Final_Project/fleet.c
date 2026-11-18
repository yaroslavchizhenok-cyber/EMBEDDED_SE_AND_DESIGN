/*
 * fleet.c - an array of airframes and an array of pilots.  See fleet.h.
 */
#include "fleet.h"
#include "craft.h"
#include "pilot.h"

typedef struct {
    craft_t body;
    pilot_t brain;
    cmd_t   asked;               /* the pilot's last command: the airframe
                                    obeys it until the next pilot step       */
} drone_t;

static drone_t     fleet[FLEET_MAX];
static int         count;
static sky_t       sky;
static gps_grade_t grade;
static uint32_t    ticks, seed_used;

/* Each drone's dice must differ, or every drone meets the same gust at the
 * same moment and the swarm sways as one.  A drone's seed is the show's seed
 * mixed with its number by MurmurHash3's finaliser (multiply, shift, xor),
 * so neighbouring numbers give unrelated streams. */
static uint32_t mix(uint32_t seed, uint32_t i)
{
    uint32_t h = seed ^ (i * 0x9E3779B9u);
    h ^= h >> 16;  h *= 0x85EBCA6Bu;
    h ^= h >> 13;  h *= 0xC2B2AE35u;
    h ^= h >> 16;
    return h ? h : 1u;
}

static int ok(int i) { return i >= 0 && i < count; }

int fleet_init(int n, uint32_t seed)
{
    if (n < 1 || n > FLEET_MAX) { return -1; }
    count = n;
    ticks = 0;
    seed_used = seed;
    sky = (sky_t){ 0.0f, 0.0f };
    grade = GPS_PLAIN;
    for (int i = 0; i < n; i++) {
        craft_init(&fleet[i].body, mix(seed, (uint32_t)i), 0.0f, 0.0f);
        pilot_init(&fleet[i].brain);
        fleet[i].asked = (cmd_t){ 0 };
    }
    return n;
}

int fleet_count(void) { return count; }

void fleet_place(int i, float x, float y)
{
    if (!ok(i) || !fleet[i].body.grounded) { return; }
    craft_init(&fleet[i].body, mix(seed_used, (uint32_t)i), x, y);   /* same dice */
    fleet[i].body.gps  = grade;
    fleet[i].body.t_ms = ticks * (1000u / AIR_HZ);
}

void fleet_wind(float speed, float toward)
{
    sky.speed  = speed < 0.0f ? 0.0f : speed;
    sky.toward = toward;
}

void fleet_gps(float wander, float jitter)
{
    grade = (gps_grade_t){ wander < 0.0f ? 0.0f : wander, jitter < 0.0f ? 0.0f : jitter };
    for (int i = 0; i < count; i++) { fleet[i].body.gps = grade; }
}

int  fleet_arm(int i)  { return ok(i) ? pilot_arm(&fleet[i].brain)  : -1; }
int  fleet_land(int i) { return ok(i) ? pilot_land(&fleet[i].brain) : -1; }

void fleet_target(int i, v3_t at, v3_t vel)
{
    if (ok(i)) { pilot_goal(&fleet[i].brain, at, vel); }
}

void fleet_step(void)
{
    /* pilots first, on every second tick: they decide from what the sensors
     * said, then every airframe moves under that decision */
    if (ticks % (AIR_HZ / PILOT_HZ) == 0u) {
        for (int i = 0; i < count; i++) {
            drone_t *d = &fleet[i];
            meas_t seen;                               /* used once: not kept */
            craft_sense(&d->body, &seen);
            pilot_step(&d->brain, &seen, &d->asked);
        }
    }
    for (int i = 0; i < count; i++) {
        craft_step(&fleet[i].body, &sky, &fleet[i].asked, AIR_DT);
    }
    ticks++;
}

uint32_t fleet_time_ms(void) { return ticks * (1000u / AIR_HZ); }

v3_t    fleet_where(int i)    { return ok(i) ? fleet[i].body.pos : (v3_t){ 0 }; }
uint8_t fleet_mode(int i)     { return ok(i) ? fleet[i].brain.mode : 0u; }
uint8_t fleet_grounded(int i) { return ok(i) ? fleet[i].body.grounded : 1u; }

int fleet_view(int i, fleet_drone_t *o)
{
    if (!ok(i) || !o) { return -1; }
    const craft_t *c = &fleet[i].body;
    const pilot_t *p = &fleet[i].brain;
    *o = (fleet_drone_t){
        .t_ms = c->t_ms, .mode = p->mode, .why = p->why, .grounded = c->grounded, .fixes = p->fixes,
        .x = c->pos.x, .y = c->pos.y, .z = c->pos.z,
        .vx = c->vel.x, .vy = c->vel.y, .vz = c->vel.z,
        .ex = p->est.x, .ey = p->est.y, .ez = p->est.z,
        .gx = p->goal.x, .gy = p->goal.y, .gz = p->goal.z,
        .roll = c->bank, .pitch = c->nose, .yaw = c->heading,
        .lift = c->lift, .charge = c->charge,
        .air_x = c->air_x, .air_y = c->air_y,
        .hardest_landing = c->hardest_landing,
    };
    return 0;
}
