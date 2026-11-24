/*
 * Main.c - SOC3050 lesson 19, the swarm: ONE drone on ONE chip
 * Target: ST Nucleo-C031C6 (STM32C031C6, Cortex-M0+ at 48 MHz) on the app board.
 *
 * Every board in the swarm runs this same Main.hex.  What makes boards
 * different is the board itself: three ID straps (PB0, PA7, PA15, each tied
 * to GND or left open) say which drone it is, as address pins on a real
 * part do.  Each drone's diagram.json ties its own straps.
 *
 * The UART is this drone's RADIO: what it prints is what it transmits, what
 * it receives is what it hears.  The station's radio medium carries a beacon
 * to the drones in range of it, and only to them (agent.h says what goes out).
 *
 *   who     prio  period   does
 *   USART2  IRQ   a byte   assembles whole received lines in link.c's ring
 *   ui       2    20 ms    buttons, joystick, knob -> posted to fly; LD4
 *   fly      1    10 ms    the drone: every line heard, then agent_tick()
 *
 * Only fly ever touches the agent, so the agent needs no lock:
 *   - the interrupt hands over WHOLE lines (link.c): no task wakes per byte,
 *     and a line that cannot be kept is dropped whole, never cut in half.
 *     (Until 2026-11-23 a `link` task did this through the shared uart.c's
 *     64-byte queue; in Wokwi it lost bytes mid-frame.  Its stack and queue
 *     were 700 bytes of RAM, too.)
 *   - ui POSTS what the buttons did (posted_*), and fly applies it.
 * One lock is left: print_lock, so lines from two tasks never interleave.
 *
 * What goes out, in order of importance (emit):
 *   radio frames and events   always - the fly task waits for room if it must.
 *                             A beacon skipped would look to every neighbour
 *                             like a beacon lost, and weaken the link for nothing.
 *   telemetry ($PHY $NBT $HLT) only when the transmit ring has room to spare,
 *                             so it never delays the radio.
 *
 * Ten times a second fly reports the link, $HLT, for the station: lines
 * heard, lines refused (bad checksum or form), lines lost (no room, too long,
 * UART overrun), telemetry frames skipped, and BYTES TAKEN IN (low 24 bits).
 * The last is a credit: the station never has more than 384 bytes on their
 * way to this drone that it has not taken in yet - less than link.c's
 * 512-byte ring - so however slowly this chip runs (a slow Wokwi window),
 * its ring cannot overflow (swarm_station.py, "pacing").
 *
 * LD4 (PA5) blinks the number of neighbours this drone hears, then pauses:
 * three blinks = three neighbours.  Solid on: no ID straps.
 */
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "stm32c031xx.h"
#include "gpio.h"
#include "os.h"
#include "link.h"
#include "proto.h"
#include "pad.h"
#include "agent.h"
#include "../fmt.h"

#define BAUD 115200u
#define STACK(name, words) static uint32_t name[words] __attribute__((aligned(8)))
STACK(stk_fly, 256);              /* worst case ~650 B in use: -fstack-usage */
STACK(stk_ui, 96);

static agent_t me;
static os_mutex_t print_lock;
static volatile uint32_t skipped;

/* ============================================================================
 *  Output
 * ============================================================================ */
int _write(int fd, const char *buf, int len);           /* link.c */

static void say_raw(const char *buf, int n)
{
    int tasks = os_current_task() != 0;
    if (tasks) { os_mutex_lock(&print_lock); }
    (void)_write(1, buf, n);
    if (tasks) { os_mutex_unlock(&print_lock); }
}

static void say(const char *f, ...) __attribute__((format(printf, 1, 2)));
static void say(const char *f, ...)
{
    char buf[112];
    va_list ap;
    va_start(ap, f);
    int n = fmt_v(buf, sizeof buf, f, ap);
    va_end(ap);
    if (n > 0) { say_raw(buf, n); }
}

/* The agent's output.  must: radio frames and events - always sent.  0:
 * telemetry - sent only if the ring keeps TX_SPARE bytes free for the radio
 * after it ('\n' becomes "\r\n": +1). */
#define TX_SPARE 100u
static void emit(const char *line, int must)
{
    int n = (int)strlen(line);
    if (!must && os_current_task() != 0 && link_tx_room() < (uint32_t)n + 1u + TX_SPARE) { skipped++; return; }
    say_raw(line, n);
}

/* ============================================================================
 *  What ui posts for fly
 * ============================================================================ */
static volatile uint8_t posted_up, posted_down, posted_kill, posted_breeze_new;
static volatile float   posted_vx, posted_vy, posted_breeze;

/* ============================================================================
 *  fly - the drone, every 10 ms
 * ============================================================================ */
static volatile uint32_t tick_max_us, fly_late;

static void stats(void)
{
    const volatile link_stats_t *u = link_stats();
    uint32_t now = os_ticks() ? os_ticks() : 1u;
    say("drone %u: own clock %lu ms, chip clock %lu ms; tick max %lu us of 10000, late %lu\n",
        (unsigned)me.id, (unsigned long)me.now_ms, (unsigned long)now, (unsigned long)tick_max_us,
        (unsigned long)fly_late);
    for (uint32_t i = 0; i < os_task_count(); i++) {
        os_task_t *t = os_task(i);
        say("  %-5s CPU %3lu %%  stack %3lu/%lu words\n", t->name, (unsigned long)(t->ticks * 100u / now),
            (unsigned long)os_stack_used(t), (unsigned long)t->stack_words);
    }
    say("link rx %lu B %lu lines, %lu lost (%lu overruns), %lu refused; tx %lu B, %lu waits; telemetry skipped %lu\n",
        (unsigned long)u->rx_bytes, (unsigned long)u->rx_lines, (unsigned long)u->rx_lost,
        (unsigned long)u->rx_overruns, (unsigned long)me.rx_bad, (unsigned long)u->tx_bytes,
        (unsigned long)u->tx_waits, (unsigned long)skipped);
}

static void handle(char *line)
{
    if (agent_line(&me, line)) { return; }               /* a $ frame             */
    if (agent_command(&me, line)) { return; }
    char *w = line;
    while (*w == ' ') { w++; }
    if (strcmp(w, "stats") == 0) { stats(); return; }
    if (*w) { say("? %s - type help\n", w); }
}

/* $HLT,id,lines,refused,lost,skipped,bytes - running totals; the station
 * takes the differences */
static void health(void)
{
    const volatile link_stats_t *u = link_stats();
    char line[72];                                       /* built in place: "$" body "*CS\n" */
    int n = fmt(line + 1, sizeof line - 6u, "HLT,%u,%lu,%lu,%lu,%lu,%lu", (unsigned)me.id, (unsigned long)u->rx_lines,
                (unsigned long)me.rx_bad, (unsigned long)u->rx_lost, (unsigned long)skipped,
                (unsigned long)(u->rx_bytes & 0xFFFFFFu));
    line[0] = '$';
    fmt(line + 1 + n, 6, "*%02X\n", (unsigned)proto_checksum(line + 1, (size_t)n));
    emit(line, 0);
}

static void task_fly(void *arg)
{
    (void)arg;
    static char line[LINK_LINE];
    uint32_t last = os_ticks(), beat = 0;
    for (;;) {
        os_delay_until(&last, AGENT_TICK_MS);
        uint32_t t0 = os_ticks(), v0 = SysTick->VAL;
        while (link_line(line, sizeof line)) { handle(line); }   /* everything heard */
        if (++beat % 10u == 0u && me.state != AG_NOID) { health(); }    /* 10 Hz: the credit */
        if (posted_up)   { posted_up = 0u;   agent_takeoff(&me); }
        if (posted_down) { posted_down = 0u; agent_land(&me); }
        if (posted_kill) { posted_kill = 0u; agent_kill(&me); }
        if (posted_breeze_new) { posted_breeze_new = 0u; agent_breeze(&me, posted_breeze); }
        agent_stick(&me, posted_vx, posted_vy);
        agent_tick(&me);
        uint32_t t1 = os_ticks(), v1 = SysTick->VAL;
        uint32_t cy = (t1 - t0) * (SysTick->LOAD + 1u) + v0 - v1;
        if (cy / 48u > tick_max_us) { tick_max_us = cy / 48u; }
        if (os_ticks() - last > AGENT_TICK_MS) { fly_late++;  last = os_ticks(); }   /* no catch-up rush */
    }
}

/* ============================================================================
 *  ui - the buttons, the stick and the knob in; LD4 out
 * ============================================================================ */
static void task_ui(void *arg)
{
    (void)arg;
    uint32_t last = os_ticks(), cycle = 0;
    int16_t knob_ref = -1;
    for (;;) {
        os_delay_until(&last, 20);
        pad_t p;
        pad_read(&p);
        if (p.pressed & PAD_A) { posted_up = 1u; }
        if (p.pressed & PAD_B) { posted_down = 1u; }
        if (p.pressed & PAD_SEL) { posted_kill = 1u; }   /* the joystick's press: a crash */
        posted_vx = (float)p.x * (NUDGE_MS / 100.0f);
        posted_vy = (float)p.y * (NUDGE_MS / 100.0f);
        if (p.adc_ok) {                              /* the knob: only once MOVED */
            if (knob_ref < 0) { knob_ref = p.knob; }
            else if (abs(p.knob - knob_ref) > 12) {
                knob_ref = p.knob;
                posted_breeze = (float)p.knob * 0.01f;   /* 0..10 m/s */
                posted_breeze_new = 1u;
            }
        }
        /* LD4: one 200 ms slot (10 calls of 20 ms) per neighbour - on 80 ms,
         * off 120 - then five dark slots, a 1 s pause, and again. */
        cycle++;
        uint32_t slot = cycle / 10u, phase = cycle % 10u, n = me.nb.n;
        int on;
        if (me.state == AG_NOID) { on = 1; }
        else if (slot >= n + 5u)  { cycle = 0; on = 0; }
        else                      { on = slot < n && phase < 4u; }
        if (on) { pin_high(GPIOA, 5u); } else { pin_low(GPIOA, 5u); }
    }
}

/* ============================================================================
 *  The straps: which drone this board is, and what it carries
 *    ID    PB0, PA7, PA15  bits 0, 1, 2   (1..7)
 *    TYPE  PB1, PB2        bits 0, 1      0 or 1 mapper (camera)  2 relay
 *  A strap tied to GND reads 1.
 * ============================================================================ */
static uint8_t straps(GPIO_TypeDef *const *port, const uint8_t *pin, int n)
{
    for (int i = 0; i < n; i++) {
        pin_mode(port[i], pin[i], MODE_INPUT);
        pin_pull(port[i], pin[i], PULL_UP);
    }
    for (volatile int k = 0; k < 2000; k++) { }          /* let the pull-ups win  */
    uint8_t v = 0;
    for (int i = 0; i < n; i++) {
        if (!pin_read(port[i], pin[i])) { v |= (uint8_t)(1u << i); }
    }
    return v;
}

int main(void)
{
    link_init(SystemCoreClock, BAUD);
    int a = pad_init();                                  /* clocks GPIOA and GPIOB */
    pin_mode(GPIOA, 5u, MODE_OUTPUT);                    /* LD4                    */
    static GPIO_TypeDef *const id_port[3] = { GPIOB, GPIOA, GPIOA }, *const ty_port[2] = { GPIOB, GPIOB };
    static const uint8_t id_pin[3] = { 0u, 7u, 15u }, ty_pin[2] = { 1u, 2u };
    uint8_t id = straps(id_port, id_pin, 3), type = straps(ty_port, ty_pin, 2);
    say("\n=== SOC3050 lesson 19 - the swarm: one drone, one chip ===\n");
    say("  straps: drone %u, %s;  pad: %s\n", (unsigned)id, agent_type_name(type),
        a == 0 ? "joystick, knob, A/B" : "buttons only");
    agent_init(&me, id, type, emit);

    os_task_create("ui",   task_ui,   0, stk_ui,   96, 2);
    os_task_create("fly",  task_fly,  0, stk_fly,  256, 1);
    os_start();
}
