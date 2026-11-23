/*
 * drone_pc.c - a TEST STAND-IN for one Wokwi drone, so the station can be
 * tried without simulator windows.  NOT part of the swarm: the swarm is the
 * chips.
 *
 *     out\drone_pc.exe ID PORT [TYPE [RX_RATE [RX_BUF]]]
 *         e.g.  out\drone_pc.exe 3 4003 1               a sensor, a perfect link
 *               out\drone_pc.exe 3 4003 1 1200 64       ... behind a SLOW chip
 *
 * The same agent.c the chip runs, ticked every 10 ms of wall-clock time,
 * behind a TCP server on localhost:PORT that plays the part of Wokwi's
 * rfc2217 port (swarm_station.py --scheme socket).  One client at a time.
 *
 * RX_RATE and RX_BUF make it behave like a chip that cannot keep up - a
 * Wokwi window running slower than real time.  Bytes from the socket wait
 * in an RX_BUF-byte buffer that empties at RX_RATE bytes a second; a byte
 * that finds it full is lost, and so is the line it was part of (as link.c
 * drops a line whole).  That is the failure the station's pacing is there
 * to prevent (test_air.py).
 *
 * Like the chip it reports $HLT ten times a second: lines, refused, lost,
 * skipped, and the bytes it has taken in (the station's credit).
 */
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../agent.h"
#include "proto.h"

static SOCKET client = INVALID_SOCKET;
static agent_t me;
static unsigned long lines, lost, taken;

static void emit(const char *line, int must)
{
    (void)must;
    if (client == INVALID_SOCKET) { return; }
    char buf[256];                                /* link.c turns \n into \r\n; so does this */
    size_t n = 0;
    for (const char *p = line; *p && n < sizeof buf - 2; p++) {
        if (*p == '\n') { buf[n++] = '\r'; }
        buf[n++] = *p;
    }
    if (send(client, buf, (int)n, 0) == SOCKET_ERROR) { closesocket(client);  client = INVALID_SOCKET; }
}

static void handle(char *line)
{
    if (agent_line(&me, line) || agent_command(&me, line)) { return; }
    if (strcmp(line, "stats") == 0) {
        char b[120];
        snprintf(b, sizeof b, "link: %lu lines, %lu lost, %lu refused\n", lines, lost, (unsigned long)me.rx_bad);
        emit(b, 1);
        return;
    }
    if (*line) { char b[120]; snprintf(b, sizeof b, "? %.80s - type help\n", line); emit(b, 1); }
}

/* ---- the slow chip's receive side ------------------------------------------ */
static unsigned short fifo[4096];                 /* byte | 0x100: bytes were lost after it */
static unsigned fifo_n, fifo_cap = 4096;
static char line[RADIO_LINE];
static size_t n;
static int junk;

static void assemble(unsigned short e)
{
    char ch = (char)(e & 0xFF);
    taken++;
    if (ch == '\r' || ch == '\n') {
        if (junk) { lost++; } else if (n) { line[n] = '\0';  lines++;  handle(line); }
        n = 0;  junk = 0;
    } else if (!junk) {
        if (n < sizeof line - 1) { line[n++] = ch; } else { junk = 1; }
    }
    if (e & 0x100) { junk = 1; }                  /* the bytes after this one never came */
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: drone_pc ID PORT [TYPE [RX_RATE [RX_BUF]]]\n"); return 2; }
    int id = atoi(argv[1]), port = atoi(argv[2]), type = argc > 3 ? atoi(argv[3]) : 0;
    double rate = argc > 4 ? atof(argv[4]) : 0.0;   /* 0: as fast as it comes */
    if (argc > 5) { fifo_cap = (unsigned)atoi(argv[5]); }
    if (fifo_cap < 1 || fifo_cap > 4096) { fifo_cap = 4096; }
    WSADATA w;
    WSAStartup(MAKEWORD(2, 2), &w);
    SOCKET srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    BOOL yes = TRUE;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof yes);
    struct sockaddr_in a = { 0 };
    a.sin_family = AF_INET;
    a.sin_port = htons((u_short)port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(srv, (struct sockaddr *)&a, sizeof a) != 0 || listen(srv, 1) != 0) {
        fprintf(stderr, "drone_pc: cannot listen on %d\n", port);
        return 1;
    }
    printf("drone_pc: drone %d on localhost:%d\n", id, port);
    fflush(stdout);
    agent_init(&me, (uint8_t)id, (uint8_t)type, emit);

    ULONGLONG next = GetTickCount64(), last_drain = next, next_hlt = next + 100;
    double tokens = 0;
    for (;;) {
        ULONGLONG now = GetTickCount64();
        /* the chip takes bytes out of its buffer at its own pace */
        if (rate > 0) { tokens += rate * (double)(now - last_drain) / 1000.0; } else { tokens = 1e9; }
        last_drain = now;
        unsigned k = 0;
        while (k < fifo_n && tokens >= 1.0) { assemble(fifo[k++]);  tokens -= 1.0; }
        if (k) { memmove(fifo, fifo + k, (fifo_n - k) * sizeof fifo[0]);  fifo_n -= k; }
        if (tokens > 64) { tokens = 64; }                 /* no saving up */
        while ((LONGLONG)(now - next) >= 0) {             /* the 10 ms ticks due */
            agent_tick(&me);
            next += AGENT_TICK_MS;
            if ((LONGLONG)(now - next) > 500) { next = now; }
        }
        if ((LONGLONG)(now - next_hlt) >= 0 && me.state != AG_NOID) {
            next_hlt += 100;                              /* 10 Hz, like the chip */
            char body[80], fr[96];
            snprintf(body, sizeof body, "HLT,%d,%lu,%lu,%lu,0,%lu", id, lines, (unsigned long)me.rx_bad, lost,
                     taken & 0xFFFFFFul);
            snprintf(fr, sizeof fr, "$%s*%02X\n", body, (unsigned)proto_checksum(body, strlen(body)));
            emit(fr, 0);
        }
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(srv, &rd);
        if (client != INVALID_SOCKET) { FD_SET(client, &rd); }
        LONGLONG wait = (LONGLONG)(next - GetTickCount64());
        if (fifo_n) { wait = 1; }
        struct timeval tv = { 0, (long)(wait > 0 ? wait * 1000 : 0) };
        if (select(0, &rd, 0, 0, &tv) <= 0) { continue; }
        if (FD_ISSET(srv, &rd)) {
            SOCKET c = accept(srv, 0, 0);
            if (client != INVALID_SOCKET) { closesocket(client); }
            client = c;
            n = 0;  junk = 0;  fifo_n = 0;
        }
        if (client != INVALID_SOCKET && FD_ISSET(client, &rd)) {
            char buf[512];
            int got = recv(client, buf, sizeof buf, 0);
            if (got <= 0) { closesocket(client);  client = INVALID_SOCKET;  continue; }
            for (int i = 0; i < got; i++) {
                if (fifo_n < fifo_cap) { fifo[fifo_n++] = (unsigned char)buf[i]; }
                else if (fifo_n) { fifo[fifo_n - 1] |= 0x100; }   /* lost: mark the gap */
            }
        }
    }
}
