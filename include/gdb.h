/*
 * GDB Remote Serial Protocol Stub
 *
 * TCP server implementing GDB RSP for debugging firmware in the emulator.
 * Supports: register read/write, memory read/write, breakpoints,
 * single-step, continue.
 *
 * Usage: ./bramble firmware.uf2 -gdb [port]
 * Then: arm-none-eabi-gdb -ex "target remote :3333"
 */

#ifndef GDB_H
#define GDB_H

#include <stdint.h>

/* Default GDB server port */
#define GDB_DEFAULT_PORT 3333

/* Maximum number of hardware breakpoints */
#define GDB_MAX_BREAKPOINTS 16

/* GDB server state */
typedef struct {
    int server_fd;              /* Listening socket */
    int client_fd;              /* Connected client socket */
    int port;                   /* TCP port */
    int active;                 /* Server is running */
    int single_step;            /* Single-step mode */

    /* Breakpoints */
    uint32_t breakpoints[GDB_MAX_BREAKPOINTS];
    int bp_active[GDB_MAX_BREAKPOINTS];
    int bp_count;

    /* Packet buffer */
    char pkt_buf[4096];
    int pkt_len;
} gdb_state_t;

extern gdb_state_t gdb;

/* ========================================================================
 * API
 * ======================================================================== */

/* Initialize and start GDB server (blocking until client connects) */
int gdb_init(int port);

/* Cleanup and close sockets */
void gdb_cleanup(void);

/* Check if execution should stop (breakpoint hit or single-step) */
int gdb_should_stop(uint32_t pc);

/* Handle GDB commands (called when execution stops) */
/* Returns: 0 = continue execution, 1 = single-step, -1 = detach/quit */
int gdb_handle(void);

#endif /* GDB_H */
