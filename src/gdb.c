/*
 * GDB Remote Serial Protocol Stub
 *
 * Implements the GDB RSP over TCP for debugging firmware in the emulator.
 * Connects to GDB via: target remote :<port>
 *
 * Supported commands:
 *   ?        - Halt reason (SIGTRAP)
 *   g        - Read all registers (R0-R15 + xPSR, 17 regs)
 *   G        - Write all registers
 *   p n      - Read register n
 *   P n=val  - Write register n
 *   m addr,len - Read memory
 *   M addr,len:data - Write memory
 *   c        - Continue execution
 *   s        - Single step
 *   Z0,addr,kind - Set software breakpoint
 *   z0,addr,kind - Remove software breakpoint
 *   Z1,addr,kind - Set hardware breakpoint
 *   z1,addr,kind - Remove hardware breakpoint
 *   D        - Detach
 *   k        - Kill
 *   qSupported - Feature query
 *   qAttached  - Attached query
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#include "gdb.h"
#include "emulator.h"

gdb_state_t gdb;

/* ========================================================================
 * Helpers
 * ======================================================================== */

static uint8_t hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

static uint8_t hex_byte(const char *s) {
    return (hex_digit(s[0]) << 4) | hex_digit(s[1]);
}

static uint32_t hex_to_u32(const char *s, int *len_out) {
    uint32_t val = 0;
    int i = 0;
    while (s[i]) {
        char c = s[i];
        if (c >= '0' && c <= '9') { val = (val << 4) | (c - '0'); }
        else if (c >= 'a' && c <= 'f') { val = (val << 4) | (c - 'a' + 10); }
        else if (c >= 'A' && c <= 'F') { val = (val << 4) | (c - 'A' + 10); }
        else break;
        i++;
    }
    if (len_out) *len_out = i;
    return val;
}

static const char hex_chars[] = "0123456789abcdef";

static void u8_to_hex(uint8_t val, char *out) {
    out[0] = hex_chars[(val >> 4) & 0xF];
    out[1] = hex_chars[val & 0xF];
}

/* Convert 32-bit value to 8 hex chars (little-endian byte order for GDB) */
static void u32_to_hex_le(uint32_t val, char *out) {
    u8_to_hex((uint8_t)(val & 0xFF), out);
    u8_to_hex((uint8_t)((val >> 8) & 0xFF), out + 2);
    u8_to_hex((uint8_t)((val >> 16) & 0xFF), out + 4);
    u8_to_hex((uint8_t)((val >> 24) & 0xFF), out + 6);
}

/* Parse hex from little-endian byte order */
static uint32_t hex_le_to_u32(const char *s) {
    uint32_t val = 0;
    val |= (uint32_t)hex_byte(s);
    val |= (uint32_t)hex_byte(s + 2) << 8;
    val |= (uint32_t)hex_byte(s + 4) << 16;
    val |= (uint32_t)hex_byte(s + 6) << 24;
    return val;
}

/* ========================================================================
 * Packet I/O
 * ======================================================================== */

static int gdb_send_raw(const char *data, int len) {
    int sent = 0;
    while (sent < len) {
        int n = write(gdb.client_fd, data + sent, len - sent);
        if (n <= 0) return -1;
        sent += n;
    }
    return 0;
}

static int gdb_send_packet(const char *data) {
    int len = strlen(data);
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum += (uint8_t)data[i];
    }

    char buf[8192];
    int pos = 0;
    buf[pos++] = '$';
    memcpy(buf + pos, data, len);
    pos += len;
    buf[pos++] = '#';
    u8_to_hex(checksum, buf + pos);
    pos += 2;

    return gdb_send_raw(buf, pos);
}

static int gdb_recv_packet(char *out, int max_len) {
    char buf[8192];
    int total = 0;

    /* Read until we get a complete packet: $data#xx */
    while (1) {
        int n = read(gdb.client_fd, buf + total, sizeof(buf) - total - 1);
        if (n <= 0) return -1;
        total += n;
        buf[total] = '\0';

        /* Look for packet start */
        char *start = strchr(buf, '$');
        if (!start) {
            /* Handle interrupt (Ctrl-C = 0x03) */
            for (int i = 0; i < total; i++) {
                if (buf[i] == 0x03) {
                    out[0] = 0x03;
                    out[1] = '\0';
                    /* Send ACK */
                    gdb_send_raw("+", 1);
                    return 1;
                }
            }
            total = 0;
            continue;
        }

        /* Look for packet end */
        char *end = strchr(start, '#');
        if (end && (end - buf + 2) < total) {
            /* Extract data between $ and # */
            int data_len = end - start - 1;
            if (data_len >= max_len) data_len = max_len - 1;
            memcpy(out, start + 1, data_len);
            out[data_len] = '\0';

            /* Send ACK */
            gdb_send_raw("+", 1);
            return data_len;
        }
    }
}

/* ========================================================================
 * Register Access (ARM Cortex-M0+ register layout for GDB)
 *
 * GDB expects: R0-R12, SP(R13), LR(R14), PC(R15), xPSR
 * Total: 17 registers, 32 bits each
 * ======================================================================== */

static uint32_t gdb_read_reg(int reg) {
    if (reg >= 0 && reg <= 15) return cpu.r[reg];
    if (reg == 16) return cpu.xpsr;  /* CPSR/xPSR */
    return 0;
}

static void gdb_write_reg(int reg, uint32_t val) {
    if (reg >= 0 && reg <= 15) cpu.r[reg] = val;
    if (reg == 16) cpu.xpsr = val;
}

#define GDB_NUM_REGS 17

/* ========================================================================
 * Command Handlers
 * ======================================================================== */

static void handle_query(const char *pkt) {
    if (strncmp(pkt, "qSupported", 10) == 0) {
        gdb_send_packet("PacketSize=4096;swbreak+;hwbreak+");
    } else if (strncmp(pkt, "qAttached", 9) == 0) {
        gdb_send_packet("1");  /* Attached to existing process */
    } else if (strncmp(pkt, "qC", 2) == 0) {
        gdb_send_packet("QC1");  /* Current thread = 1 */
    } else if (strncmp(pkt, "qfThreadInfo", 12) == 0) {
        gdb_send_packet("m1");   /* Thread 1 */
    } else if (strncmp(pkt, "qsThreadInfo", 12) == 0) {
        gdb_send_packet("l");    /* End of thread list */
    } else if (strncmp(pkt, "qTStatus", 8) == 0) {
        gdb_send_packet("");     /* No trace running */
    } else {
        gdb_send_packet("");     /* Unsupported query */
    }
}

static void handle_read_registers(void) {
    char resp[GDB_NUM_REGS * 8 + 1];
    for (int i = 0; i < GDB_NUM_REGS; i++) {
        u32_to_hex_le(gdb_read_reg(i), resp + i * 8);
    }
    resp[GDB_NUM_REGS * 8] = '\0';
    gdb_send_packet(resp);
}

static void handle_write_registers(const char *data) {
    for (int i = 0; i < GDB_NUM_REGS && data[i * 8]; i++) {
        gdb_write_reg(i, hex_le_to_u32(data + i * 8));
    }
    gdb_send_packet("OK");
}

static void handle_read_register(const char *data) {
    int reg_num = 0;
    hex_to_u32(data, &reg_num);
    reg_num = (int)hex_to_u32(data, NULL);

    char resp[9];
    if (reg_num < GDB_NUM_REGS) {
        u32_to_hex_le(gdb_read_reg(reg_num), resp);
        resp[8] = '\0';
    } else {
        strcpy(resp, "xxxxxxxx");
    }
    gdb_send_packet(resp);
}

static void handle_write_register(const char *data) {
    int len;
    uint32_t reg_num = hex_to_u32(data, &len);
    if (data[len] == '=') {
        uint32_t val = hex_le_to_u32(data + len + 1);
        gdb_write_reg(reg_num, val);
    }
    gdb_send_packet("OK");
}

static void handle_read_memory(const char *data) {
    int len1, len2;
    uint32_t addr = hex_to_u32(data, &len1);
    uint32_t length = hex_to_u32(data + len1 + 1, &len2);  /* Skip comma */

    if (length > 2048) length = 2048;  /* Limit response size */

    char resp[4097];
    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = mem_read8(addr + i);
        u8_to_hex(byte, resp + i * 2);
    }
    resp[length * 2] = '\0';
    gdb_send_packet(resp);
}

static void handle_write_memory(const char *data) {
    int len1, len2;
    uint32_t addr = hex_to_u32(data, &len1);
    uint32_t length = hex_to_u32(data + len1 + 1, &len2);  /* Skip comma */
    const char *hex_data = data + len1 + 1 + len2 + 1;     /* Skip colon */

    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = hex_byte(hex_data + i * 2);
        mem_write8(addr + i, byte);
    }
    gdb_send_packet("OK");
}

static void handle_set_breakpoint(const char *data) {
    /* Z0,addr,kind or Z1,addr,kind */
    int len;
    uint32_t addr = hex_to_u32(data + 2, &len);  /* Skip "0," or "1," */

    /* Find free slot */
    for (int i = 0; i < GDB_MAX_BREAKPOINTS; i++) {
        if (!gdb.bp_active[i]) {
            gdb.breakpoints[i] = addr;
            gdb.bp_active[i] = 1;
            gdb.bp_count++;
            gdb_send_packet("OK");
            return;
        }
    }
    gdb_send_packet("E01");  /* No room */
}

static void handle_remove_breakpoint(const char *data) {
    int len;
    uint32_t addr = hex_to_u32(data + 2, &len);

    for (int i = 0; i < GDB_MAX_BREAKPOINTS; i++) {
        if (gdb.bp_active[i] && gdb.breakpoints[i] == addr) {
            gdb.bp_active[i] = 0;
            gdb.bp_count--;
            gdb_send_packet("OK");
            return;
        }
    }
    gdb_send_packet("OK");  /* Not found is OK */
}

/* ========================================================================
 * Public API
 * ======================================================================== */

int gdb_init(int port) {
    memset(&gdb, 0, sizeof(gdb));
    gdb.port = port;
    gdb.server_fd = -1;
    gdb.client_fd = -1;

    /* Create TCP server socket */
    gdb.server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (gdb.server_fd < 0) {
        perror("[GDB] socket");
        return -1;
    }

    int opt = 1;
    setsockopt(gdb.server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);

    if (bind(gdb.server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[GDB] bind");
        close(gdb.server_fd);
        return -1;
    }

    if (listen(gdb.server_fd, 1) < 0) {
        perror("[GDB] listen");
        close(gdb.server_fd);
        return -1;
    }

    printf("[GDB] Listening on port %d... (connect with: target remote :%d)\n",
           port, port);
    fflush(stdout);

    /* Wait for client connection */
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    gdb.client_fd = accept(gdb.server_fd, (struct sockaddr *)&client_addr, &client_len);
    if (gdb.client_fd < 0) {
        perror("[GDB] accept");
        close(gdb.server_fd);
        return -1;
    }

    printf("[GDB] Client connected\n");
    gdb.active = 1;
    return 0;
}

void gdb_cleanup(void) {
    if (gdb.client_fd >= 0) close(gdb.client_fd);
    if (gdb.server_fd >= 0) close(gdb.server_fd);
    gdb.client_fd = -1;
    gdb.server_fd = -1;
    gdb.active = 0;
}

int gdb_should_stop(uint32_t pc) {
    if (gdb.single_step) return 1;

    for (int i = 0; i < GDB_MAX_BREAKPOINTS; i++) {
        if (gdb.bp_active[i] && gdb.breakpoints[i] == pc) {
            return 1;
        }
    }
    return 0;
}

int gdb_handle(void) {
    if (!gdb.active) return -1;

    /* Send stop reason */
    gdb_send_packet("S05");  /* SIGTRAP */
    gdb.single_step = 0;

    while (1) {
        char pkt[4096];
        int len = gdb_recv_packet(pkt, sizeof(pkt));
        if (len < 0) {
            printf("[GDB] Client disconnected\n");
            gdb.active = 0;
            return -1;
        }

        if (len == 0) continue;

        /* Handle Ctrl-C interrupt */
        if (pkt[0] == 0x03) {
            gdb_send_packet("S02");  /* SIGINT */
            continue;
        }

        switch (pkt[0]) {
        case '?':
            gdb_send_packet("S05");  /* SIGTRAP */
            break;

        case 'g':
            handle_read_registers();
            break;

        case 'G':
            handle_write_registers(pkt + 1);
            break;

        case 'p':
            handle_read_register(pkt + 1);
            break;

        case 'P':
            handle_write_register(pkt + 1);
            break;

        case 'm':
            handle_read_memory(pkt + 1);
            break;

        case 'M':
            handle_write_memory(pkt + 1);
            break;

        case 'c':
            /* Continue execution */
            gdb.single_step = 0;
            return 0;

        case 's':
            /* Single step */
            gdb.single_step = 1;
            return 1;

        case 'Z':
            if (pkt[1] == '0' || pkt[1] == '1') {
                handle_set_breakpoint(pkt + 1);
            } else {
                gdb_send_packet("");  /* Unsupported type */
            }
            break;

        case 'z':
            if (pkt[1] == '0' || pkt[1] == '1') {
                handle_remove_breakpoint(pkt + 1);
            } else {
                gdb_send_packet("");
            }
            break;

        case 'D':
            gdb_send_packet("OK");
            printf("[GDB] Client detached\n");
            gdb.active = 0;
            return -1;

        case 'k':
            printf("[GDB] Kill request\n");
            gdb.active = 0;
            return -1;

        case 'H':
            /* Set thread - just acknowledge */
            gdb_send_packet("OK");
            break;

        case 'v':
            if (strncmp(pkt, "vMustReplyEmpty", 15) == 0) {
                gdb_send_packet("");
            } else if (strncmp(pkt, "vCont?", 6) == 0) {
                gdb_send_packet("vCont;c;s");
            } else if (strncmp(pkt, "vCont;c", 7) == 0) {
                gdb.single_step = 0;
                return 0;
            } else if (strncmp(pkt, "vCont;s", 7) == 0) {
                gdb.single_step = 1;
                return 1;
            } else {
                gdb_send_packet("");
            }
            break;

        case 'q':
            handle_query(pkt);
            break;

        default:
            gdb_send_packet("");  /* Unsupported command */
            break;
        }
    }
}
