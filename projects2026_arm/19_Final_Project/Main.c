/*
 * Main.c - SOC3050 lesson 19: Swarm parade
 * Target: ST Nucleo-C031C6 (STM32C031C6, Cortex-M0+ at 48 MHz) on the app board.
 *
 * Five simulated drones fly a light show inside this chip: take off, a line,
 * a V, a circle, a heart, a cross, an arrow, a wide circle, home, land.  Each
 * drone has its own flight computer (pilot.c), its own gusts and its own GPS
 * error (craft.c).  At power-up the chip plans the whole show (plan.c) and
 * refuses it if two drones would ever be planned closer than 3 m.  Then it
 * flies it on one clock and scores it (director.c).
 *
 * This file is only the hardware half - everything that flies, plans and
 * scores is pure C, and host/test_show.c runs the same files on a PC.
 *
 *   task   prio  period   does
 *   link    3    a byte   the shell and the station's keys ($KEY)
 *   ui      2    20 ms    buttons and knob -> commands; LD4 blinks
 *   fly     1    10 ms    director_tick(): the fleet, the score, the frames
 *
 * fly is the LOWEST priority on purpose.  In Wokwi a tick of five drones
 * can take longer than 10 ms, and then fly never sleeps - and a higher-priority
 * task that never sleeps starves everything below it (lesson 13 froze exactly so).
 * At the bottom it can only use the time the others leave: the show goes into
 * slow motion, and the buttons, LD4 and the shell keep working.  On
 * silicon fly sleeps until each 10 ms is due, and the show runs in real time.
 *
 * Shared state and its lock:
 *   dir   dir_lock    the director (fleet, plan, score): fly ticks it, ui and
 *                     link give commands and read its state
 *   the UART          print_lock   no line is ever cut in two
 * Lock order: dir_lock before print_lock (the director prints while ticking).
 *
 * FLASH IS THE LIMIT here, not speed: the first build was 43 KB for a 32 KB
 * chip.  What it took to fit:
 *   - no strtok(), strtoul(), atoi(): newlib's strtok() calls assert(), and
 *     assert() prints through the whole stdio machinery - 3.5 KB to split a
 *     line into words;
 *   - the station's keys read here in a few lines, not by _lib's kbd.c and
 *     keys.c (1.4 KB) - this lesson only needs key PRESSES;
 *   - fmt.c instead of vsnprintf() (2 KB less), and sin/cos/sqrt compiled
 *     once (trig.c);
 *   - softfloat.c instead of libgcc's float + - * / (2.8 KB less, and each
 *     operation in half the cycles - README.md, "Made fast");
 *   - NO OLED.  The display, its I2C driver, the picture and the station's
 *     copy of it were 4 KB, and the station draws the show far better.  The
 *     app board's OLED stays dark; LD4 (PA5) blinks to show the chip is alive:
 *     slowly on the ground, fast in flight.
 */
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "stm32c031xx.h"
#include "os.h"
#include "uart.h"
#include "pad.h"
#include "proto.h"
#include "director.h"
#include "fmt.h"

#define BAUD 115200u
#define STACK(name, words) static uint32_t name[words] __attribute__((aligned(8)))
STACK(stk_fly, 352);
STACK(stk_ui, 256);
STACK(stk_link, 288);

static director_t dir;
static os_mutex_t dir_lock, print_lock;
/* The show plays at TWICE real time unless told otherwise (`speed 1`): two
 * minutes of show in one.  x2 is the fastest a real board keeps up with - a
 * tick costs it 2.4 ms, 3.7 at worst, of the 5 ms it then has.  If the chip
 * (or Wokwi) cannot keep the pace asked, the show simply runs as fast as it
 * can: fly never skips work, the clock falls behind instead. */
static volatile uint8_t  speed = 2;             /* x real time; 0 = flat out */
static volatile uint32_t dropped, pace_pct;      /* frames skipped (link busy); show speed % */

/* ============================================================================
 *  Output.  No printf: a stdio stream costs ~1.7 KB of flash (lesson 14,
 *  slide 21).  fmt.c formats, uart.c's _write() sends.
 * ============================================================================ */
int _write(int fd, const char *buf, int len);           /* uart.c */

static void say_raw(const char *buf, int n)
{
    int tasks = os_current_task() != 0;                  /* before os_start: no lock */
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
    if (n <= 0) { return; }
    if (n > (int)sizeof buf - 1) { n = (int)sizeof buf - 1; }
    say_raw(buf, n);
}

/* The director's output.  A telemetry frame may be skipped when the TX ring
 * is too full to take it - the next one is 100 ms away - so fly never waits
 * on the UART.  The score and replies (must) always go out. */
static void emit(const char *line, int must)
{
    int n = (int)strlen(line);
    if (!must && os_current_task() != 0 && uart_tx_room() < (uint32_t)n) { dropped++; return; }
    say_raw(line, n);
}

/* ============================================================================
 *  Measuring: what does one tick of the whole fleet cost?
 * ============================================================================
 * SysTick counts down from LOAD once per millisecond; a stamp is (ms, count)
 * and the difference of two stamps is CPU cycles (lesson 14). */
typedef struct { uint32_t ms, val; } stamp_t;

static stamp_t stamp(void)
{
    stamp_t s;
    do { s.ms = os_ticks();  s.val = SysTick->VAL; } while (s.ms != os_ticks());
    return s;
}

static uint32_t cycles(stamp_t a, stamp_t b)
{
    return (b.ms - a.ms) * (SysTick->LOAD + 1u) + a.val - b.val;
}

static volatile uint32_t tick_last, tick_max, tick_sum, tick_n, fly_ticks;

/* Wind from the keys and the knob is POSTED here, not applied under dir_lock:
 * that lock is held for a whole fleet tick, and a press must not wait for it.
 * fly applies it at the top of its next tick (and while resting: <= 20 ms). */
static volatile float   wind_post;
static volatile uint8_t wind_new;
static void post_wind(float ms) { wind_post = ms; wind_new = 1u; }

/* ============================================================================
 *  fly - the show, one 10 ms tick at a time
 * ============================================================================ */
static void task_fly(void *arg)
{
    (void)arg;
    uint32_t due = os_ticks(), idle = 0, beat = 0;
    for (;;) {
        stamp_t a = stamp();
        os_mutex_lock(&dir_lock);
        if (wind_new) { wind_new = 0u; director_set_wind(&dir, wind_post, dir.wind_deg); }
        /* Telemetry follows REAL time: with the show measured at N x real time
         * (pace_pct, by ui, every second) every N-th report is sent, so the
         * station gets its ten pictures a second at any speed and the UART is
         * never asked for more than it carries. */
        uint32_t pace = pace_pct;
        dir.stride = (uint8_t)(pace < 150u ? 1u : pace >= 950u ? 10u : (pace + 50u) / 100u);
        int moved = director_tick(&dir);
        os_mutex_unlock(&dir_lock);
        if (!moved) {                                    /* nothing to fly: rest */
            os_delay(20);
            due = os_ticks();
            if (++idle % 5u == 0u) {                     /* every 100 ms: the picture
                                                            again, for a station that
                                                            connected after it was sent */
                os_mutex_lock(&dir_lock);
                director_heartbeat(&dir, beat++);
                os_mutex_unlock(&dir_lock);
            }
            continue;
        }
        uint32_t cy = cycles(a, stamp());
        tick_last = cy;
        if (cy > tick_max) { tick_max = cy; }
        tick_sum += cy;  tick_n++;  fly_ticks++;

        uint8_t sp = speed;
        if (sp == 0u) { continue; }                      /* flat out             */
        due += 10u / sp;
        uint32_t now = os_ticks();
        if ((int32_t)(due - now) > 0) {
            os_delay(due - now);                         /* ahead: wait for it   */
        } else if ((int32_t)(now - due) > 200) {
            due = now;                                   /* behind: slow motion,
                                                            never a catch-up rush */
        }
    }
}

/* ============================================================================
 *  The shell - typed in Wokwi's serial monitor or the station's command box
 * ============================================================================ */
/* (uart.c sends every \n as \r\n, so no string here carries a \r.) */
static void help(void)
{
    say("keys: a start/go  b abort  s pause/go  1-5 wind 1-9 m/s (knob 0-10)\n"
        "cmds: start pause go abort reset plan score stats quiet loud help\n"
        "      seed N  wind MS [DEG]  gps rtk|plain  ff on|off  speed 0|1|2|5|10\n");
}

static void stats(void)
{
    const volatile uart_stats_t *u = uart_stats();
    uint32_t n = tick_n ? tick_n : 1u, now = os_ticks() ? os_ticks() : 1u;
    say("%s, speed x%u: show clock at %lu %% of real time\n",
        director_state_name(dir.state), (unsigned)speed, (unsigned long)pace_pct);
    say("tick (%d drones): mean %lu us, max %lu us (budget 10000)\n", dir.plan.n,
        (unsigned long)(tick_sum / n / 48u), (unsigned long)(tick_max / 48u));
    for (uint32_t i = 0; i < os_task_count(); i++) {
        os_task_t *t = os_task(i);
        say("  %-5s CPU %3lu %%  stack %3lu/%lu words\n", t->name, (unsigned long)(t->ticks * 100u / now),
            (unsigned long)os_stack_used(t), (unsigned long)t->stack_words);
    }
    say("uart tx %lu waits %lu rx %lu dropped %lu; frames skipped %lu\n", (unsigned long)u->tx_bytes,
        (unsigned long)u->tx_waits, (unsigned long)u->rx_bytes, (unsigned long)u->rx_dropped,
        (unsigned long)dropped);
}

static char *word(char **s)
{
    char *p = *s;
    while (*p == ' ' || *p == '\t') { p++; }
    if (!*p) { return 0; }
    char *w = p;
    while (*p && *p != ' ' && *p != '\t') { p++; }
    if (*p) { *p++ = '\0'; }
    *s = p;
    return w;
}

static long number(const char *s)
{
    long v = 0;
    int neg = *s == '-';
    if (*s == '-' || *s == '+') { s++; }
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s++ - '0'); }
    return neg ? -v : v;
}

static int is(const char *a, const char *b) { return strcmp(a, b) == 0; }

static void command(char *line)
{
    char *rest = line, *w = word(&rest), *a1 = word(&rest), *a2 = word(&rest);
    if (!w) { return; }
    for (char *c = w; *c; c++) { if (*c >= 'A' && *c <= 'Z') { *c = (char)(*c + 32); } }
    if (is(w, "stats")) { stats(); return; }
    if (is(w, "help") || is(w, "?")) { help(); return; }
    int bad = 0;
    os_mutex_lock(&dir_lock);
    if      (is(w, "start"))  { director_start(&dir); }
    else if (is(w, "go"))     { director_go(&dir); }
    else if (is(w, "pause"))  { director_pause(&dir); }
    else if (is(w, "abort"))  { director_abort(&dir); }
    else if (is(w, "plan"))   { director_print_plan(&dir); }
    else if (is(w, "score"))  { director_report(&dir); }
    else if (is(w, "quiet"))  { dir.quiet = 1; }
    else if (is(w, "loud"))   { dir.quiet = 0; }
    else if (is(w, "reset"))  {
        if (dir.state == DR_FLYING || dir.state == DR_LANDING) { bad = 2; } else { director_reset(&dir); }
    }
    else if (is(w, "seed") && a1) { bad = director_set_seed(&dir, (uint32_t)number(a1)) ? 2 : 0; }
    else if (is(w, "gps") && a1)  { bad = director_set_gps(&dir, is(a1, "rtk")) ? 2 : 0; }
    else if (is(w, "ff") && a1)   { bad = director_set_ff(&dir, is(a1, "on") || is(a1, "1")) ? 2 : 0; }
    else if (is(w, "wind") && a1) {
        director_set_wind(&dir, (float)number(a1), a2 ? (float)number(a2) : dir.wind_deg);
    }
    else if (is(w, "speed") && a1) {
        long v = number(a1);
        if (v == 0 || v == 1 || v == 2 || v == 5 || v == 10) { speed = (uint8_t)v; } else { bad = 1; }
    }
    else { bad = 1; }
    uint8_t st = dir.state;
    os_mutex_unlock(&dir_lock);
    if (bad == 1) { say("? %s - type help\n", w); }
    if (bad == 2) { say("%s: only on the ground (the show is %s)\n", w, director_state_name(st)); }
}

/* Is this key-down a new PRESS?  The station sends $KEY,<key>,1 when a key
 * goes down, repeats it every 200 ms while the key is held (so a lost "up"
 * cannot leave it stuck), and sends $KEY,<key>,0 when it comes up.  Holding s
 * must pause once, not toggle five times a second - so a key counts again
 * only after its "up", or after 600 ms without a repeat (the "up" was lost). */
static int fresh_press(char c, int down, uint32_t now)
{
    static struct { char key; uint32_t last; } held[4];
    int slot = -1;
    for (int i = 0; i < 4; i++) { if (held[i].key == c) { slot = i; } }
    if (!down) {
        if (slot >= 0) { held[slot].key = 0; }
        return 0;
    }
    if (slot >= 0) {
        int repeat = now - held[slot].last < 600u;
        held[slot].last = now;
        return !repeat;
    }
    for (int i = 0; i < 4; i++) {
        if (!held[i].key) { held[i].key = c;  held[i].last = now;  break; }
    }
    return 1;
}

/* A key from the station: $KEY,<key>,<1|0>*CS.  Only presses of one-letter
 * keys matter here; the station keeps the arrows for its camera. */
static void station_key(const char *line)
{
    const char *body;
    size_t len;
    if (proto_check(line, &body, &len) != PROTO_OK || len != 7 || body[5] != ',') { return; }
    char c = body[4];
    if (!fresh_press(c, body[6] == '1', os_ticks())) { return; }
    if (c == '?') { help(); return; }
    if (c >= '1' && c <= '5') { post_wind((float)((c - '1') * 2 + 1)); return; }
    os_mutex_lock(&dir_lock);
    if (c == 'a')                  { if (dir.paused) { director_go(&dir); } else { director_start(&dir); } }
    else if (c == 'b' || c == 'p') { director_abort(&dir); }
    else if (c == 's')             { if (dir.paused) { director_go(&dir); } else { director_pause(&dir); } }
    os_mutex_unlock(&dir_lock);
}

/* A line from the station that is not a command: a key ($KEY).  Any other
 * $ line - the station's $MIRROR heartbeat, asking for an OLED this firmware
 * does not drive - is ignored. */
static int station_line(const char *line)
{
    if (strncmp(line, "$KEY,", 5) == 0) { station_key(line); return 1; }
    return line[0] == '$';
}

static void task_link(void *arg)
{
    (void)arg;
    static char line[80];
    uint32_t n = 0;
    for (;;) {
        char c = (char)uart_getc();
        if (c == '\b' || c == 0x7F) { if (n) { n--; } continue; }
        if (c != '\r' && c != '\n') { if (n < sizeof line - 1u) { line[n++] = c; } continue; }
        if (n == 0) { continue; }
        line[n] = '\0';
        n = 0;
        if (!station_line(line)) { command(line); }
    }
}

/* ============================================================================
 *  ui - the buttons and the knob in, LD4 out
 * ============================================================================ */
static void task_ui(void *arg)
{
    (void)arg;
    uint32_t last = os_ticks(), cycle = 0, pace_ms = os_ticks(), pace_ticks = 0;
    int16_t knob_ref = -1;
    GPIOA->MODER = (GPIOA->MODER & ~(3u << (5 * 2))) | (1u << (5 * 2));   /* PA5: LD4 out */

    for (;;) {
        os_delay_until(&last, 20);
        uint32_t now = os_ticks();
        pad_t p;
        pad_read(&p);

        if (p.adc_ok) {                                     /* knob: no lock needed */
            if (knob_ref < 0) { knob_ref = p.knob; }
            else if (abs(p.knob - knob_ref) > 12) { knob_ref = p.knob; post_wind((float)p.knob * 0.01f); }
        }
        os_mutex_lock(&dir_lock);
        if (p.pressed & PAD_A) { if (dir.paused) { director_go(&dir); } else { director_start(&dir); } }
        if (p.pressed & PAD_B) { director_abort(&dir); }
        if (p.pressed & PAD_SEL) { if (dir.paused) { director_go(&dir); } else { director_pause(&dir); } }
        /* (the knob, above, is the wind, 0..10 m/s - but only once it is MOVED,
         * so the show's own wind holds until someone touches it) */
        int flying = dir.state == DR_FLYING || dir.state == DR_LANDING;
        os_mutex_unlock(&dir_lock);

        if (++cycle % (flying ? 5u : 25u) == 0u) { GPIOA->ODR ^= 1u << 5; }   /* LD4 */
        if (now - pace_ms >= 1000u) {                       /* show time vs real */
            uint32_t t = fly_ticks;
            pace_pct = (t - pace_ticks) * 10u * 100u / (now - pace_ms);
            pace_ticks = t;  pace_ms = now;
        }
    }
}

int main(void)
{
    uart_init(SystemCoreClock, BAUD);
    say("\n=== SOC3050 lesson 19 - Swarm parade ===\n");
    int a = pad_init();
    say("  pad: %s\n", a == 0 ? "joystick, knob, A/B/SEL" : "ADC failed - buttons only");

    say("  planning the show - in Wokwi this takes a while ...\n");
    if (director_init(&dir, &SHOW, emit) == 0) { director_print_plan(&dir); }
    help();

    os_task_create("link", task_link, 0, stk_link, 288, 3);
    os_task_create("ui",   task_ui,   0, stk_ui,   256, 2);
    os_task_create("fly",  task_fly,  0, stk_fly,  352, 1);
    os_start();
}
