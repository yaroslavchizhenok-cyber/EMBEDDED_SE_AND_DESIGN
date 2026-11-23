/*
 * link.h - the drone's radio link: USART2, whole LINES in, bytes out
 *
 * The course's shared uart.c (lesson 08) hands every received byte to a task
 * through a 64-byte kernel queue: an interrupt, a queue put and a task switch
 * per byte.  That is the right lesson for a shell.  It is the wrong shape for
 * a radio that hears five neighbours: a burst of frames fills 64 bytes in
 * 5.6 ms, and in an emulator that runs slower than real time the task does
 * not keep up - bytes are lost mid-frame, and a frame with one byte missing
 * is a frame lost (its checksum fails, or its '$' is gone).
 *
 * Here the INTERRUPT assembles the lines.  Each byte goes straight into a
 * 512-byte ring, and only when its '\n' arrives does the line become visible
 * to the reader (the head moves).  No task wakes per byte, no queue, no copy;
 * a burst of 7 frames fits.  A line that does not fit is dropped WHOLE and
 * counted - never half a line.  The fly task takes the lines out between
 * ticks (link_line).
 *
 * Out: the same lock-free ring as lesson 08 - tasks fill it, the TXE
 * interrupt empties it.  _write() lives here, so printf-style output and the
 * agent's frames all go this way.
 *
 * Replaces LIBS=uart for this firmware (build.bat).
 */
#ifndef LINK_H
#define LINK_H

#include <stdint.h>

#define LINK_RX_RING  512u        /* a power of two                          */
#define LINK_TX_RING  256u        /* a power of two                          */
#define LINK_LINE     120u        /* longest line kept ($ to the last byte)  */

typedef struct {
    uint32_t rx_bytes, rx_lines;
    uint32_t rx_overruns;         /* the UART's 1-byte buffer overflowed     */
    uint32_t rx_lost;             /* whole lines dropped: ring full or too long */
    uint32_t tx_bytes, tx_waits;
} link_stats_t;

void     link_init(uint32_t pclk, uint32_t baud);
int      link_line(char *out, uint32_t size);     /* 1: a whole line copied out  */
uint32_t link_tx_room(void);                      /* bytes _write can take now   */
const volatile link_stats_t *link_stats(void);

#endif
