/*
 * GDB Remote Serial Protocol Stub
 *
 * Implements the GDB RSP over TCP for debugging firmware in the emulator.
 * Connects to GDB via: target remote :<port>
 *
 * Features:
 *   - Dual-core debugging (Thread 1 = Core 0, Thread 2 = Core 1)
 *   - Software/hardware breakpoints with optional conditions
 *   - Write/read/access watchpoints (Z2/Z3/Z4)
 *   - Monitor commands for conditional breakpoints
 *
 * Supported commands:
 *   ?           - Halt reason
 *   g / G       - Read/write all registers (selected thread)
 *   p n / P n=v - Read/write single register
 *   m addr,len  - Read memory
 *   M addr,len:data - Write memory
 *   c / s       - Continue / single step
 *   Z0-Z4       - Set breakpoint/watchpoint
 *   z0-z4       - Remove breakpoint/watchpoint
 *   Hg/Hc       - Set thread for register/continue ops
 *   vCont       - Extended continue/step
 *   qSupported  - Feature query
 *   qfThreadInfo / qsThreadInfo - Thread enumeration
 *   qRcmd       - Monitor commands
 *   D / k       - Detach / kill
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
#include "rp2350_rv/rv_cpu.h"

/* RISC-V GDB support: external hart state pointers (set by main.c when -arch rv32) */
rv_cpu_state_t *gdb_rv_harts[2] = { NULL, NULL };
int gdb_is_riscv = 0;

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

/* Decode hex-encoded string in-place: "48656C6C6F" -> "Hello" */
static int hex_decode(const char *hex, char *out, int max_len) {
    int i = 0;
    while (hex[i * 2] && hex[i * 2 + 1] && i < max_len - 1) {
        out[i] = (char)hex_byte(hex + i * 2);
        i++;
    }
    out[i] = '\0';
    return i;
}

/* Encode string as hex: "Hello" -> "48656C6C6F" */
/* Encode exactly n bytes. hex_encode() is the NUL-terminated convenience
 * wrapper; callers sizing a fixed stack buffer must use the bounded form so the
 * encode cannot run past the end. */
static void hex_encode_n(const char *str, int n, char *out) {
    for (int i = 0; i < n; i++) {
        u8_to_hex((uint8_t)str[i], out + i * 2);
    }
    out[n * 2] = '\0';
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

/* A packet is '$' + payload + '#' + 2 hex checksum digits + NUL, so the
 * payload can be at most GDB_PACKET_BUF - 5 bytes. Clamping rather than
 * trusting the caller is deliberate: the payloads come from register dumps and
 * memory reads, whose length depends on guest-controlled state. */
#define GDB_PACKET_BUF 8192

static int gdb_send_packet(const char *data) {
    int len = (int)strlen(data);
    if (len > GDB_PACKET_BUF - 5)
        len = GDB_PACKET_BUF - 5;   /* cannot fit; truncate rather than smash */

    uint8_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum += (uint8_t)data[i];
    }

    char buf[GDB_PACKET_BUF];
    int pos = 0;
    buf[pos++] = '$';
    memcpy(buf + pos, data, (size_t)len);
    pos += len;
    buf[pos++] = '#';
    u8_to_hex(checksum, buf + pos);
    pos += 2;

    return gdb_send_raw(buf, pos);
}

/* Send an 'O' packet (console output to GDB, hex-encoded) */
static void gdb_send_output(const char *msg) {
    /* 'O' + two hex digits per input byte + NUL must fit in 4096. */
    size_t msglen = strlen(msg);
    char buf[4096];
    if (msglen > (sizeof(buf) - 2) / 2)
        msglen = (sizeof(buf) - 2) / 2;
    buf[0] = 'O';
    hex_encode_n(msg, (int)msglen, buf + 1);
}

static int is_hex(char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
}

static int gdb_recv_packet(char *out, int max_len) {
    char buf[8192];
    int total = 0;

    while (1) {
        int n = read(gdb.client_fd, buf + total, sizeof(buf) - total - 1);
        if (n <= 0) return -1;
        total += n;
        buf[total] = '\0';

        char *start = strchr(buf, '$');
        if (!start) {
            for (int i = 0; i < total; i++) {
                if (buf[i] == 0x03) {
                    out[0] = 0x03;
                    out[1] = '\0';
                    gdb_send_raw("+", 1);
                    return 1;
                }
            }
            total = 0;
            continue;
        }

        char *end = strchr(start, '#');
        if (end && (end - buf + 2) < total) {
            /* The two characters after '#' are the checksum. */
            const char *checksum_digits = end + 1;
            int data_len = end - start - 1;

            /* Verify the two hex checksum digits. The RSP framing is
             * "$<payload>#<sum of payload bytes, two hex digits>"; accepting
             * the packet without checking it means a corrupted write is ACKed
             * as though it arrived intact, and the debugger and the emulator
             * then disagree about what was sent with no indication of which is
             * wrong. */
            unsigned got = 0;
            int ok = is_hex(checksum_digits[0]) && is_hex(checksum_digits[1]);
            if (ok) {
                char d[3] = { checksum_digits[0], checksum_digits[1], '\0' };
                got = (unsigned)strtoul(d, NULL, 16);
                unsigned want = 0;
                for (int i = 0; i < data_len; i++)
                    want += (uint8_t)start[1 + i];
                ok = (got == (want & 0xFFu));
            }
            if (!ok) {
                /* NAK and keep looking for a good packet in the stream. */
                gdb_send_raw("-", 1);
                /* Drop everything up to and including the bad packet so the
                 * next read starts on a packet boundary. */
                total -= (int)(end + 2 - buf);
                if (total > 0)
                    memmove(buf, end + 2, (size_t)total);
                continue;
            }

            if (data_len >= max_len) data_len = max_len - 1;
            memcpy(out, start + 1, data_len);
            out[data_len] = '\0';
            gdb_send_raw("+", 1);
            return data_len;
        }
    }
}

/* ========================================================================
 * Register Access - Dual-Core Aware
 *
 * Reads/writes from cores[gdb.g_thread] instead of global cpu state.
 * ======================================================================== */

static uint32_t gdb_read_reg(int reg) {
    int core = gdb.g_thread;
    if (core < 0 || core >= NUM_CORES) core = 0;

    if (gdb_is_riscv && gdb_rv_harts[core]) {
        /* RISC-V: 33 registers (x0-x31 + PC) */
        rv_cpu_state_t *h = gdb_rv_harts[core];
        if (reg >= 0 && reg <= 31) return h->x[reg];
        if (reg == 32) return h->pc;
        return 0;
    }

    /* ARM: 17 registers (R0-R15 + xPSR) */
    if (reg >= 0 && reg <= 15) return cores[core].r[reg];
    if (reg == 16) return cores[core].xpsr;
    return 0;
}

static void gdb_write_reg(int reg, uint32_t val) {
    int core = gdb.g_thread;
    if (core < 0 || core >= NUM_CORES) core = 0;

    if (gdb_is_riscv && gdb_rv_harts[core]) {
        rv_cpu_state_t *h = gdb_rv_harts[core];
        if (reg >= 1 && reg <= 31) h->x[reg] = val;  /* x0 is hardwired to 0 */
        if (reg == 32) h->pc = val;
        return;
    }

    if (reg >= 0 && reg <= 15) cores[core].r[reg] = val;
    if (reg == 16) cores[core].xpsr = val;
}

#define GDB_NUM_REGS_ARM  17
#define GDB_NUM_REGS_RV   33
#define GDB_NUM_REGS      (gdb_is_riscv ? GDB_NUM_REGS_RV : GDB_NUM_REGS_ARM)

/* ========================================================================
 * Condition Evaluation
 * ======================================================================== */

static int gdb_eval_condition(const gdb_condition_t *cond, int core_id) {
    if (cond->type == GDB_COND_NONE) return 1; /* No condition = always stop */

    uint32_t lhs;
    switch (cond->type) {
    case GDB_COND_REG_EQ:
    case GDB_COND_REG_NE:
    case GDB_COND_REG_LT:
    case GDB_COND_REG_GT:
        if (cond->reg_num <= 15)
            lhs = cores[core_id].r[cond->reg_num];
        else if (cond->reg_num == 16)
            lhs = cores[core_id].xpsr;
        else
            return 1;
        break;
    case GDB_COND_MEM_EQ:
    case GDB_COND_MEM_NE:
        lhs = mem_read32(cond->addr);
        break;
    default:
        return 1;
    }

    switch (cond->type) {
    case GDB_COND_REG_EQ:
    case GDB_COND_MEM_EQ:
        return lhs == cond->value;
    case GDB_COND_REG_NE:
    case GDB_COND_MEM_NE:
        return lhs != cond->value;
    case GDB_COND_REG_LT:
        return lhs < cond->value;
    case GDB_COND_REG_GT:
        return lhs > cond->value;
    default:
        return 1;
    }
}

/* ========================================================================
 * Command Handlers
 * ======================================================================== */

static void handle_query(const char *pkt) {
    if (strncmp(pkt, "qSupported", 10) == 0) {
        gdb_send_packet("PacketSize=4096;swbreak+;hwbreak+;multiprocess-");
    } else if (strncmp(pkt, "qAttached", 9) == 0) {
        gdb_send_packet("1");
    } else if (strncmp(pkt, "qC", 2) == 0) {
        /* Current thread = core that stopped + 1 (1-based) */
        char resp[16];
        snprintf(resp, sizeof(resp), "QC%d", gdb.stop_core + 1);
        gdb_send_packet(resp);
    } else if (strncmp(pkt, "qfThreadInfo", 12) == 0) {
        /* Report both cores as threads */
        if (num_active_cores > 1)
            gdb_send_packet("m1,2");
        else
            gdb_send_packet("m1");
    } else if (strncmp(pkt, "qsThreadInfo", 12) == 0) {
        gdb_send_packet("l");  /* End of thread list */
    } else if (strncmp(pkt, "qTStatus", 8) == 0) {
        gdb_send_packet("");
    } else if (strncmp(pkt, "qRcmd,", 6) == 0) {
        /* Monitor command: hex-encoded string */
        char cmd[256];
        hex_decode(pkt + 6, cmd, sizeof(cmd));

        /* Parse monitor commands */
        if (strncmp(cmd, "help", 4) == 0) {
            gdb_send_output("Monitor commands:\n");
            gdb_send_output("  monitor info break      - List breakpoints\n");
            gdb_send_output("  monitor info watch      - List watchpoints\n");
            gdb_send_output("  monitor cond <N> r<R>==<V>  - Set condition on BP #N\n");
            gdb_send_output("  monitor cond <N> r<R>!=<V>  - Set condition on BP #N\n");
            gdb_send_output("  monitor cond <N> r<R><<V>   - Set condition on BP #N\n");
            gdb_send_output("  monitor cond <N> r<R>><V>   - Set condition on BP #N\n");
            gdb_send_output("  monitor cond <N> mem[<A>]==<V> - Memory condition\n");
            gdb_send_output("  monitor uncond <N>      - Remove condition from BP #N\n");
            gdb_send_packet("OK");
        } else if (strncmp(cmd, "info break", 10) == 0) {
            char line[128];
            int found = 0;
            for (int i = 0; i < GDB_MAX_BREAKPOINTS; i++) {
                if (gdb.bp_active[i]) {
                    snprintf(line, sizeof(line), "  #%d: 0x%08X", i, gdb.breakpoints[i]);
                    if (gdb.bp_cond[i].type != GDB_COND_NONE) {
                        char cond_desc[64];
                        snprintf(cond_desc, sizeof(cond_desc), " [cond: r%d op 0x%X]",
                                 gdb.bp_cond[i].reg_num, gdb.bp_cond[i].value);
                        strcat(line, cond_desc);
                    }
                    strcat(line, "\n");
                    gdb_send_output(line);
                    found = 1;
                }
            }
            if (!found) gdb_send_output("No breakpoints set\n");
            gdb_send_packet("OK");
        } else if (strncmp(cmd, "info watch", 10) == 0) {
            char line[128];
            int found = 0;
            for (int i = 0; i < GDB_MAX_WATCHPOINTS; i++) {
                if (gdb.watchpoints[i].active) {
                    const char *type_str = "access";
                    if (gdb.watchpoints[i].type == GDB_WP_WRITE) type_str = "write";
                    else if (gdb.watchpoints[i].type == GDB_WP_READ) type_str = "read";
                    snprintf(line, sizeof(line), "  #%d: %s 0x%08X len=%u\n",
                             i, type_str, gdb.watchpoints[i].addr,
                             gdb.watchpoints[i].length);
                    gdb_send_output(line);
                    found = 1;
                }
            }
            if (!found) gdb_send_output("No watchpoints set\n");
            gdb_send_packet("OK");
        } else if (strncmp(cmd, "cond ", 5) == 0) {
            /* monitor cond <N> r<R>==<V> */
            int bp_num = atoi(cmd + 5);
            if (bp_num < 0 || bp_num >= GDB_MAX_BREAKPOINTS || !gdb.bp_active[bp_num]) {
                gdb_send_output("Invalid breakpoint number\n");
                gdb_send_packet("OK");
            } else {
                /* Find the condition expression after the space */
                const char *expr = cmd + 5;
                while (*expr && *expr != ' ') expr++;
                while (*expr == ' ') expr++;

                gdb_condition_t *c = &gdb.bp_cond[bp_num];
                if (expr[0] == 'r' || expr[0] == 'R') {
                    /* Register condition: r0==5, r3!=0, r1<100, r2>0xff */
                    c->reg_num = (uint8_t)atoi(expr + 1);
                    const char *op = expr + 1;
                    while (*op >= '0' && *op <= '9') op++;
                    if (strncmp(op, "==", 2) == 0) {
                        c->type = GDB_COND_REG_EQ;
                        c->value = (uint32_t)strtoul(op + 2, NULL, 0);
                    } else if (strncmp(op, "!=", 2) == 0) {
                        c->type = GDB_COND_REG_NE;
                        c->value = (uint32_t)strtoul(op + 2, NULL, 0);
                    } else if (op[0] == '<') {
                        c->type = GDB_COND_REG_LT;
                        c->value = (uint32_t)strtoul(op + 1, NULL, 0);
                    } else if (op[0] == '>') {
                        c->type = GDB_COND_REG_GT;
                        c->value = (uint32_t)strtoul(op + 1, NULL, 0);
                    } else {
                        gdb_send_output("Unknown operator (use ==, !=, <, >)\n");
                        gdb_send_packet("OK");
                        return;
                    }
                    gdb_send_output("Condition set\n");
                } else if (strncmp(expr, "mem[", 4) == 0) {
                    /* Memory condition: mem[0x20000000]==0x1234 */
                    c->addr = (uint32_t)strtoul(expr + 4, NULL, 0);
                    const char *op = strchr(expr, ']');
                    if (op) op++;
                    if (op && strncmp(op, "==", 2) == 0) {
                        c->type = GDB_COND_MEM_EQ;
                        c->value = (uint32_t)strtoul(op + 2, NULL, 0);
                    } else if (op && strncmp(op, "!=", 2) == 0) {
                        c->type = GDB_COND_MEM_NE;
                        c->value = (uint32_t)strtoul(op + 2, NULL, 0);
                    } else {
                        gdb_send_output("Use mem[addr]==val or mem[addr]!=val\n");
                        gdb_send_packet("OK");
                        return;
                    }
                    gdb_send_output("Condition set\n");
                } else {
                    gdb_send_output("Use: cond <N> r<R>==<V> or mem[<A>]==<V>\n");
                }
                gdb_send_packet("OK");
            }
        } else if (strncmp(cmd, "uncond ", 7) == 0) {
            int bp_num = atoi(cmd + 7);
            if (bp_num >= 0 && bp_num < GDB_MAX_BREAKPOINTS) {
                gdb.bp_cond[bp_num].type = GDB_COND_NONE;
                gdb_send_output("Condition removed\n");
            }
            gdb_send_packet("OK");
        } else {
            gdb_send_output("Unknown command. Type 'monitor help'\n");
            gdb_send_packet("OK");
        }
    } else {
        gdb_send_packet("");
    }
}

static void handle_read_registers(void) {
    int nregs = gdb_is_riscv ? GDB_NUM_REGS_RV : GDB_NUM_REGS_ARM;
    char resp[GDB_NUM_REGS_RV * 8 + 1];  /* Max size (RV: 33*8 = 264) */
    for (int i = 0; i < nregs; i++) {
        u32_to_hex_le(gdb_read_reg(i), resp + i * 8);
    }
    resp[nregs * 8] = '\0';
    gdb_send_packet(resp);
}

static void handle_write_registers(const char *data) {
    int nregs = gdb_is_riscv ? GDB_NUM_REGS_RV : GDB_NUM_REGS_ARM;
    for (int i = 0; i < nregs && data[i * 8]; i++) {
        gdb_write_reg(i, hex_le_to_u32(data + i * 8));
    }
    gdb_send_packet("OK");
}

static void handle_read_register(const char *data) {
    int reg_num = (int)hex_to_u32(data, NULL);
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

static void handle_read_memory(const char *data, size_t avail) {
    int len1, len2;
    uint32_t addr = hex_to_u32(data, &len1);
    if (avail == 0 || (size_t)len1 + 1 >= avail) { gdb_send_packet("E01"); return; }
    uint32_t length = hex_to_u32(data + len1 + 1, &len2);

    if (length > 2048) length = 2048;

    char resp[4097];
    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = mem_read8(addr + i);
        u8_to_hex(byte, resp + i * 2);
    }
    resp[length * 2] = '\0';
    gdb_send_packet(resp);
}

static void handle_write_memory(const char *data, size_t avail) {
    int len1, len2;
    uint32_t addr = hex_to_u32(data, &len1);
    if ((size_t)len1 + 1 >= avail) { gdb_send_packet("E01"); return; }
    uint32_t length = hex_to_u32(data + len1 + 1, &len2);
    if ((size_t)len1 + 1 + (size_t)len2 + 1 >= avail) { gdb_send_packet("E01"); return; }
    const char *hex_data = data + len1 + 1 + len2 + 1;
    size_t payload = avail - (size_t)(hex_data - data);

    /* 'length' is peer-supplied and unbounded, but only `payload` bytes of hex
     * actually exist in the packet buffer. Without this clamp the loop reads
     * past the end of the caller's pkt[] array -- a host stack over-read whose
     * contents are then written into emulated RAM and readable by the same
     * peer. handle_read_memory() already clamped to 2048. */
    if (length > payload / 2) length = (uint32_t)(payload / 2);
    if (length > 2048) length = 2048;

    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = hex_byte(hex_data + i * 2);
        mem_write8(addr + i, byte);
    }
    gdb_send_packet("OK");
}

static void handle_set_breakpoint(const char *data) {
    /* Z0,addr,kind or Z1,addr,kind */
    int len;
    uint32_t addr = hex_to_u32(data + 2, &len);

    for (int i = 0; i < GDB_MAX_BREAKPOINTS; i++) {
        if (!gdb.bp_active[i]) {
            gdb.breakpoints[i] = addr;
            gdb.bp_active[i] = 1;
            gdb.bp_cond[i].type = GDB_COND_NONE;
            gdb.bp_count++;
            gdb_send_packet("OK");
            return;
        }
    }
    gdb_send_packet("E01");
}

static void handle_remove_breakpoint(const char *data) {
    int len;
    uint32_t addr = hex_to_u32(data + 2, &len);

    for (int i = 0; i < GDB_MAX_BREAKPOINTS; i++) {
        if (gdb.bp_active[i] && gdb.breakpoints[i] == addr) {
            gdb.bp_active[i] = 0;
            gdb.bp_cond[i].type = GDB_COND_NONE;
            gdb.bp_count--;
            gdb_send_packet("OK");
            return;
        }
    }
    gdb_send_packet("OK");
}

static void handle_set_watchpoint(const char *data) {
    /* Z2,addr,kind or Z3,addr,kind or Z4,addr,kind */
    int type = data[0] - '0'; /* 2, 3, or 4 */
    int len;
    uint32_t addr = hex_to_u32(data + 2, &len);
    uint32_t length = 4; /* Default watch 4 bytes */
    if (data[2 + len] == ',') {
        length = hex_to_u32(data + 2 + len + 1, NULL);
        if (length == 0) length = 4;
    }

    for (int i = 0; i < GDB_MAX_WATCHPOINTS; i++) {
        if (!gdb.watchpoints[i].active) {
            gdb.watchpoints[i].addr = addr;
            gdb.watchpoints[i].length = length;
            gdb.watchpoints[i].type = type;
            gdb.watchpoints[i].active = 1;
            gdb.wp_count++;
            gdb_send_packet("OK");
            return;
        }
    }
    gdb_send_packet("E01");
}

static void handle_remove_watchpoint(const char *data) {
    int type = data[0] - '0';
    int len;
    uint32_t addr = hex_to_u32(data + 2, &len);

    for (int i = 0; i < GDB_MAX_WATCHPOINTS; i++) {
        if (gdb.watchpoints[i].active &&
            gdb.watchpoints[i].addr == addr &&
            gdb.watchpoints[i].type == type) {
            gdb.watchpoints[i].active = 0;
            gdb.wp_count--;
            gdb_send_packet("OK");
            return;
        }
    }
    gdb_send_packet("OK");
}

/* Parse thread ID from H command: Hg<id> or Hc<id>
 * Thread IDs: -1 or 0 = all, 1 = core 0, 2 = core 1 */
static int parse_thread_id(const char *s) {
    if (s[0] == '-' && s[1] == '1') return -1;
    int tid = (int)strtol(s, NULL, 16);
    if (tid <= 0) return -1;  /* 0 or negative = all */
    return tid - 1;  /* Convert 1-based thread to 0-based core */
}

/* ========================================================================
 * Public API
 * ======================================================================== */

int gdb_init(int port) {
    memset(&gdb, 0, sizeof(gdb));
    gdb.port = port;
    gdb.server_fd = -1;
    gdb.client_fd = -1;
    gdb.g_thread = 0;  /* Default: core 0 */
    gdb.c_thread = -1;  /* Default: all cores */
    gdb.stop_core = 0;

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

    fprintf(stderr, "[GDB] Listening on port %d... (connect with: target remote :%d)\n",
           port, port);
    fflush(stdout);

    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    gdb.client_fd = accept(gdb.server_fd, (struct sockaddr *)&client_addr, &client_len);
    if (gdb.client_fd < 0) {
        perror("[GDB] accept");
        close(gdb.server_fd);
        return -1;
    }

    fprintf(stderr, "[GDB] Client connected\n");
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

int gdb_should_stop(uint32_t pc, int core_id) {
    /* Watchpoint hit (set by membus hooks) */
    if (gdb.wp_hit) return 1;

    if (gdb.single_step) return 1;

    for (int i = 0; i < GDB_MAX_BREAKPOINTS; i++) {
        if (gdb.bp_active[i] && gdb.breakpoints[i] == pc) {
            if (gdb_eval_condition(&gdb.bp_cond[i], core_id)) {
                gdb.stop_core = core_id;
                return 1;
            }
        }
    }
    return 0;
}

/* Check watchpoints on memory write */
int gdb_check_watchpoint_write(uint32_t addr, uint32_t size) {
    if (!gdb.active || gdb.wp_count == 0) return 0;

    for (int i = 0; i < GDB_MAX_WATCHPOINTS; i++) {
        if (!gdb.watchpoints[i].active) continue;
        if (gdb.watchpoints[i].type != GDB_WP_WRITE &&
            gdb.watchpoints[i].type != GDB_WP_ACCESS) continue;

        uint32_t wp_start = gdb.watchpoints[i].addr;
        uint32_t wp_end = wp_start + gdb.watchpoints[i].length;
        uint32_t acc_end = addr + size;

        /* Check overlap */
        if (addr < wp_end && acc_end > wp_start) {
            gdb.wp_hit = 1;
            gdb.wp_hit_addr = addr;
            gdb.wp_hit_type = gdb.watchpoints[i].type;
            return 1;
        }
    }
    return 0;
}

/* Check watchpoints on memory read */
int gdb_check_watchpoint_read(uint32_t addr, uint32_t size) {
    if (!gdb.active || gdb.wp_count == 0) return 0;

    for (int i = 0; i < GDB_MAX_WATCHPOINTS; i++) {
        if (!gdb.watchpoints[i].active) continue;
        if (gdb.watchpoints[i].type != GDB_WP_READ &&
            gdb.watchpoints[i].type != GDB_WP_ACCESS) continue;

        uint32_t wp_start = gdb.watchpoints[i].addr;
        uint32_t wp_end = wp_start + gdb.watchpoints[i].length;
        uint32_t acc_end = addr + size;

        if (addr < wp_end && acc_end > wp_start) {
            gdb.wp_hit = 1;
            gdb.wp_hit_addr = addr;
            gdb.wp_hit_type = gdb.watchpoints[i].type;
            return 1;
        }
    }
    return 0;
}

int gdb_handle(void) {
    if (!gdb.active) return -1;

    /* Send stop reason with thread info */
    if (gdb.wp_hit) {
        /* Watchpoint stop: T05watch:addr; or T05rwatch:addr; or T05awatch:addr; */
        char resp[64];
        const char *wp_type = "awatch";
        if (gdb.wp_hit_type == GDB_WP_WRITE) wp_type = "watch";
        else if (gdb.wp_hit_type == GDB_WP_READ) wp_type = "rwatch";
        snprintf(resp, sizeof(resp), "T05%s:%x;thread:%d;",
                 wp_type, gdb.wp_hit_addr, gdb.stop_core + 1);
        gdb_send_packet(resp);
        gdb.wp_hit = 0;
    } else {
        /* Normal stop (breakpoint/step): T05thread:N; */
        char resp[32];
        snprintf(resp, sizeof(resp), "T05thread:%d;", gdb.stop_core + 1);
        gdb_send_packet(resp);
    }
    gdb.single_step = 0;

    while (1) {
        char pkt[4096];
        /* Handlers receive pkt+1 (skipping the packet-type char); the number of
         * bytes actually present after that point is what bounds any payload. */
        int len = gdb_recv_packet(pkt, sizeof(pkt));
        size_t pkt_avail = (len > 1) ? (size_t)(len - 1) : 0;
        if (len < 0) {
            fprintf(stderr, "[GDB] Client disconnected\n");
            gdb.active = 0;
            return -1;
        }

        if (len == 0) continue;

        if (pkt[0] == 0x03) {
            char resp[32];
            snprintf(resp, sizeof(resp), "T02thread:%d;", gdb.stop_core + 1);
            gdb_send_packet(resp);
            continue;
        }

        switch (pkt[0]) {
        case '?':
            {
                char resp[32];
                snprintf(resp, sizeof(resp), "T05thread:%d;", gdb.stop_core + 1);
                gdb_send_packet(resp);
            }
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
            handle_read_memory(pkt + 1, pkt_avail);
            break;

        case 'M':
            handle_write_memory(pkt + 1, pkt_avail);
            break;

        case 'c':
            gdb.single_step = 0;
            return 0;

        case 's':
            gdb.single_step = 1;
            return 1;

        case 'Z':
            if (pkt[1] == '0' || pkt[1] == '1') {
                handle_set_breakpoint(pkt + 1);
            } else if (pkt[1] == '2' || pkt[1] == '3' || pkt[1] == '4') {
                handle_set_watchpoint(pkt + 1);
            } else {
                gdb_send_packet("");
            }
            break;

        case 'z':
            if (pkt[1] == '0' || pkt[1] == '1') {
                handle_remove_breakpoint(pkt + 1);
            } else if (pkt[1] == '2' || pkt[1] == '3' || pkt[1] == '4') {
                handle_remove_watchpoint(pkt + 1);
            } else {
                gdb_send_packet("");
            }
            break;

        case 'D':
            gdb_send_packet("OK");
            fprintf(stderr, "[GDB] Client detached\n");
            gdb.active = 0;
            return -1;

        case 'k':
            fprintf(stderr, "[GDB] Kill request\n");
            gdb.active = 0;
            return -1;

        case 'H':
            /* Hg<thread> or Hc<thread> */
            if (pkt[1] == 'g') {
                gdb.g_thread = parse_thread_id(pkt + 2);
                if (gdb.g_thread < 0) gdb.g_thread = 0;
            } else if (pkt[1] == 'c') {
                gdb.c_thread = parse_thread_id(pkt + 2);
            }
            gdb_send_packet("OK");
            break;

        case 'T':
            /* Thread alive query: T<thread-id> */
            {
                int tid = (int)hex_to_u32(pkt + 1, NULL);
                if (tid >= 1 && tid <= num_active_cores)
                    gdb_send_packet("OK");
                else
                    gdb_send_packet("E01");
            }
            break;

        case 'v':
            if (strncmp(pkt, "vMustReplyEmpty", 15) == 0) {
                gdb_send_packet("");
            } else if (strncmp(pkt, "vCont?", 6) == 0) {
                gdb_send_packet("vCont;c;s;C;S");
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
            gdb_send_packet("");
            break;
        }
    }
}
