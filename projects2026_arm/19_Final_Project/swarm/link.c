/*
 * link.c - the drone's radio link.  See link.h.
 */
#include "stm32c031xx.h"
#include "gpio.h"
#include "os.h"
#include "link.h"

#define TX_PIN      2u            /* PA2 - USART2 TX, AF1 */
#define RX_PIN      3u            /* PA3 - USART2 RX, AF1 */
#define AF_USART2   1u
#define RX_MASK     (LINK_RX_RING - 1u)
#define TX_MASK     (LINK_TX_RING - 1u)

static volatile link_stats_t stats;

/* ---- RX: the interrupt writes lines, the fly task reads them ---------------
 * rx_head: written ONLY by the ISR, and only when a whole line is in;
 * rx_tail: written ONLY by the reader.  rx_wr is the ISR's private cursor in
 * the line it is still receiving - bytes past rx_head that nobody may read. */
static char              rx_ring[LINK_RX_RING];
static volatile uint32_t rx_head, rx_tail;
static uint32_t          rx_wr, rx_len;
static uint8_t           rx_junk;             /* skipping the rest of a lost line */

/* ---- TX: tasks write head, the ISR writes tail (lesson 08) ---------------- */
static uint8_t           tx_ring[LINK_TX_RING];
static volatile uint32_t tx_head, tx_tail;

void link_init(uint32_t pclk, uint32_t baud)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN;
    RCC->APBENR1 |= RCC_APBENR1_USART2EN;
    pin_af(GPIOA, TX_PIN, AF_USART2);
    pin_af(GPIOA, RX_PIN, AF_USART2);
    USART2->BRR = (pclk + baud / 2u) / baud;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;
    NVIC_SetPriority(USART2_IRQn, 1);         /* above the kernel: a byte every 87 us */
    NVIC_EnableIRQ(USART2_IRQn);
}

const volatile link_stats_t *link_stats(void) { return &stats; }

static void rx_byte(char c)
{
    if (c == '\r' || c == '\n') {
        if (rx_junk) { rx_junk = 0;  rx_wr = rx_head;  rx_len = 0;  return; }
        if (rx_len == 0u) { return; }
        if (rx_wr - rx_tail >= LINK_RX_RING) {          /* no room for the '\0' */
            rx_wr = rx_head;  rx_len = 0;  stats.rx_lost++;
            return;
        }
        rx_ring[rx_wr++ & RX_MASK] = '\0';
        rx_head = rx_wr;                                /* the line is now readable */
        rx_len = 0;
        stats.rx_lines++;
        return;
    }
    if (rx_junk) { return; }
    if (c == '\b' || c == 0x7F) {                       /* a typed backspace */
        if (rx_len) { rx_wr--;  rx_len--; }
        return;
    }
    if (rx_len >= LINK_LINE - 1u || rx_wr - rx_tail >= LINK_RX_RING - 1u) {
        rx_junk = 1;                                    /* too long, or no room: */
        rx_wr = rx_head;  rx_len = 0;                   /* drop the WHOLE line   */
        stats.rx_lost++;
        return;
    }
    rx_ring[rx_wr++ & RX_MASK] = c;
    rx_len++;
}

int link_line(char *out, uint32_t size)
{
    uint32_t t = rx_tail, h = rx_head;
    if (t == h) { return 0; }
    uint32_t n = 0;
    char c;
    while ((c = rx_ring[t++ & RX_MASK]) != '\0') {
        if (n < size - 1u) { out[n++] = c; }
    }
    out[n] = '\0';
    rx_tail = t;                                        /* the room is free again */
    return 1;
}

uint32_t link_tx_room(void) { return (tx_tail + LINK_TX_RING - tx_head - 1u) & TX_MASK; }

void USART2_IRQHandler(void)
{
    uint32_t isr = USART2->ISR;
    if (isr & USART_ISR_ORE) {                          /* a byte was lost: so is its line */
        USART2->ICR = USART_ICR_ORECF;
        stats.rx_overruns++;
        if (rx_len) { rx_junk = 1;  rx_wr = rx_head;  rx_len = 0;  stats.rx_lost++; }
    }
    if (isr & USART_ISR_RXNE_RXFNE) {
        stats.rx_bytes++;
        rx_byte((char)USART2->RDR);
    }
    if ((USART2->CR1 & USART_CR1_TXEIE_TXFNFIE) && (isr & USART_ISR_TXE_TXFNF)) {
        if (tx_tail != tx_head) {
            USART2->TDR = tx_ring[tx_tail & TX_MASK];
            tx_tail = (tx_tail + 1u) & TX_MASK;
            stats.tx_bytes++;
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;    /* idle: stop asking */
        }
    }
}

static void tx_put(uint8_t c)
{
    uint32_t next = (tx_head + 1u) & TX_MASK;
    while (next == tx_tail) {
        stats.tx_waits++;
        if (os_current_task()) { os_delay(1); }
    }
    tx_ring[tx_head] = c;
    tx_head = next;
}

int _write(int fd, const char *buf, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        if (buf[i] == '\n') { tx_put('\r'); }
        tx_put((uint8_t)buf[i]);
    }
    USART2->CR1 |= USART_CR1_TXEIE_TXFNFIE;             /* wake the sender */
    return len;
}
