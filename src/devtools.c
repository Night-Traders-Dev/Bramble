/*
 * Developer Tools for Bramble RP2040 Emulator
 *
 * Implementation of semihosting, coverage, hotspots, trace,
 * SYSCFG, and TBMAN peripherals.
 */

#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "emulator.h"
#include "devtools.h"

/* ========================================================================
 * ARM Semihosting
 * ======================================================================== */

int semihosting_enabled = 0;
int semihost_exit_requested = 0;
int semihost_exit_code = 0;

void semihosting_init(void) {
    semihosting_enabled = 1;
    semihost_exit_requested = 0;
    semihost_exit_code = 0;
}

/* Read a uint32 from guest memory at addr */
static uint32_t sh_read32(uint32_t addr) {
    return mem_read32(addr);
}

/* Read a byte from guest memory */
static uint8_t sh_read8(uint32_t addr) {
    return mem_read8(addr);
}

int semihosting_handle(void) {
    if (!semihosting_enabled) return 0;

    uint32_t op = cpu.r[0];
    uint32_t param = cpu.r[1];

    switch (op) {
    case SEMIHOST_SYS_WRITEC: {
        /* R1 points to a single character */
        char c = (char)sh_read8(param);
        fputc(c, stdout);
        fflush(stdout);
        cpu.r[0] = 0;
        break;
    }
    case SEMIHOST_SYS_WRITE0: {
        /* R1 points to a null-terminated string */
        for (int i = 0; i < 4096; i++) {
            char c = (char)sh_read8(param + (uint32_t)i);
            if (c == '\0') break;
            fputc(c, stdout);
        }
        fflush(stdout);
        cpu.r[0] = 0;
        break;
    }
    case SEMIHOST_SYS_WRITE: {
        /* param[0]=fd, param[1]=data_ptr, param[2]=len */
        uint32_t fd   = sh_read32(param);
        uint32_t data = sh_read32(param + 4);
        uint32_t len  = sh_read32(param + 8);
        FILE *out = (fd == 1) ? stdout : (fd == 2) ? stderr : stdout;
        for (uint32_t i = 0; i < len && i < 65536; i++) {
            fputc(sh_read8(data + i), out);
        }
        fflush(out);
        cpu.r[0] = 0;  /* 0 = all bytes written */
        break;
    }
    case SEMIHOST_SYS_READC: {
        int c = fgetc(stdin);
        cpu.r[0] = (c == EOF) ? (uint32_t)-1 : (uint32_t)c;
        break;
    }
    case SEMIHOST_SYS_EXIT: {
        /* ADP_Stopped_ApplicationExit = 0x20026 */
        semihost_exit_code = (param == 0x20026) ? 0 : (int)param;
        semihost_exit_requested = 1;
        cpu.r[15] = 0xFFFFFFFF;  /* Halt the core */
        break;
    }
    case SEMIHOST_SYS_EXIT_EXTENDED: {
        /* param[0] = reason, param[1] = exit code */
        semihost_exit_code = (int)sh_read32(param + 4);
        semihost_exit_requested = 1;
        cpu.r[15] = 0xFFFFFFFF;
        break;
    }
    case SEMIHOST_SYS_ERRNO:
        cpu.r[0] = 0;
        break;
    case SEMIHOST_SYS_ELAPSED:
        /* Return emulated time in centiseconds */
        cpu.r[0] = 0;
        break;
    case SEMIHOST_SYS_TICKFREQ:
        cpu.r[0] = 100;  /* 100 Hz */
        break;
    case SEMIHOST_SYS_OPEN:
    case SEMIHOST_SYS_CLOSE:
    case SEMIHOST_SYS_READ:
    case SEMIHOST_SYS_SEEK:
    case SEMIHOST_SYS_FLEN:
        /* Unimplemented file ops: return -1 */
        cpu.r[0] = (uint32_t)-1;
        break;
    default:
        /* Unknown operation */
        cpu.r[0] = (uint32_t)-1;
        break;
    }

    /* Advance PC past the BKPT instruction */
    cpu.r[15] += 2;
    return 1;
}

/* ========================================================================
 * Code Coverage
 * ======================================================================== */

int coverage_enabled = 0;
uint8_t *coverage_bitmap = NULL;
uint8_t *coverage_ram_bitmap = NULL;

void coverage_init(void) {
    coverage_bitmap = calloc(1, COVERAGE_BITMAP_BYTES);
    coverage_ram_bitmap = calloc(1, (RAM_SIZE / 2 + 7) / 8);
    if (coverage_bitmap && coverage_ram_bitmap) {
        coverage_enabled = 1;
    }
}

void coverage_cleanup(void) {
    free(coverage_bitmap);
    free(coverage_ram_bitmap);
    coverage_bitmap = NULL;
    coverage_ram_bitmap = NULL;
    coverage_enabled = 0;
}

void coverage_dump(const char *path) {
    if (!coverage_bitmap) return;
    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "[Coverage] Failed to open %s for writing\n", path);
        return;
    }
    /* Header: magic, flash_base, flash_size, ram_base, ram_size */
    uint32_t hdr[5] = { 0x434F5642, FLASH_BASE, FLASH_SIZE, RAM_BASE, RAM_SIZE }; /* "BVOC" */
    fwrite(hdr, 4, 5, f);
    fwrite(coverage_bitmap, 1, COVERAGE_BITMAP_BYTES, f);
    fwrite(coverage_ram_bitmap, 1, (RAM_SIZE / 2 + 7) / 8, f);
    fclose(f);
    fprintf(stderr, "[Coverage] Written to %s\n", path);
}

void coverage_report(void) {
    if (!coverage_bitmap) return;
    uint32_t flash_covered = 0;
    for (uint32_t i = 0; i < COVERAGE_BITMAP_BYTES; i++) {
        flash_covered += (uint32_t)__builtin_popcount(coverage_bitmap[i]);
    }
    uint32_t flash_total = FLASH_SIZE / 2;
    uint32_t ram_covered = 0;
    uint32_t ram_bytes = (RAM_SIZE / 2 + 7) / 8;
    for (uint32_t i = 0; i < ram_bytes; i++) {
        ram_covered += (uint32_t)__builtin_popcount(coverage_ram_bitmap[i]);
    }
    fprintf(stderr, " Coverage: %u/%u flash halfwords (%.1f%%), %u RAM halfwords executed\n",
            flash_covered, flash_total,
            flash_total > 0 ? (double)flash_covered / flash_total * 100.0 : 0.0,
            ram_covered);
}

/* ========================================================================
 * Hotspot Profiling
 * ======================================================================== */

int hotspots_enabled = 0;
int hotspots_top_n = 20;
hotspot_entry_t *hotspot_map = NULL;

void hotspots_init(void) {
    hotspot_map = calloc(HOTSPOT_MAP_SIZE, sizeof(hotspot_entry_t));
    if (hotspot_map) {
        hotspots_enabled = 1;
    }
}

void hotspots_cleanup(void) {
    free(hotspot_map);
    hotspot_map = NULL;
    hotspots_enabled = 0;
}

/* Comparison for qsort (descending by count) */
static int hotspot_cmp(const void *a, const void *b) {
    const hotspot_entry_t *ha = a, *hb = b;
    if (hb->count > ha->count) return 1;
    if (hb->count < ha->count) return -1;
    return 0;
}

void hotspots_report(void) {
    if (!hotspot_map) return;

    /* Collect non-zero entries */
    int count = 0;
    for (int i = 0; i < HOTSPOT_MAP_SIZE; i++) {
        if (hotspot_map[i].count > 0) count++;
    }
    if (count == 0) return;

    /* Copy to sortable array */
    int n = count < hotspots_top_n ? count : hotspots_top_n;
    hotspot_entry_t *sorted = malloc((size_t)count * sizeof(hotspot_entry_t));
    if (!sorted) return;

    int si = 0;
    for (int i = 0; i < HOTSPOT_MAP_SIZE; i++) {
        if (hotspot_map[i].count > 0) {
            sorted[si++] = hotspot_map[i];
        }
    }
    qsort(sorted, (size_t)count, sizeof(hotspot_entry_t), hotspot_cmp);

    fprintf(stderr, " Hotspots (top %d of %d unique PCs):\n", n, count);
    fprintf(stderr, "   %-12s  %-12s  %s\n", "Address", "Count", "Region");
    for (int i = 0; i < n; i++) {
        const char *region = "???";
        uint32_t pc = sorted[i].pc;
        if (pc >= FLASH_BASE && pc < FLASH_BASE + FLASH_SIZE)
            region = "flash";
        else if (pc >= RAM_BASE && pc < RAM_BASE + RAM_SIZE)
            region = "ram";
        else if (pc < 0x4000)
            region = "rom";
        fprintf(stderr, "   0x%08X  %-12u  %s\n", pc, sorted[i].count, region);
    }
    free(sorted);
}

/* ========================================================================
 * Instruction Trace
 * ======================================================================== */

int trace_enabled = 0;
FILE *trace_file = NULL;

void trace_init(const char *path) {
    trace_file = fopen(path, "wb");
    if (trace_file) {
        trace_enabled = 1;
        /* Write header: magic + version */
        uint32_t hdr[2] = { 0x54524342, 1 }; /* "BCRT" v1 */
        fwrite(hdr, 4, 2, trace_file);
        fprintf(stderr, "[Trace] Writing instruction trace to %s\n", path);
    } else {
        fprintf(stderr, "[Trace] Failed to open %s\n", path);
    }
}

void trace_cleanup(void) {
    if (trace_file) {
        fclose(trace_file);
        trace_file = NULL;
    }
    trace_enabled = 0;
}

/* ========================================================================
 * Exit Code Hook
 * ======================================================================== */

int exit_code_enabled = 0;
uint32_t exit_code_addr = 0;

/* ========================================================================
 * Timeout Enforcement
 * ======================================================================== */

int timeout_seconds = 0;
volatile int timeout_expired = 0;

static void timeout_handler(int sig) {
    (void)sig;
    timeout_expired = 1;
}

void timeout_start(int seconds) {
    timeout_seconds = seconds;
    timeout_expired = 0;
    signal(SIGALRM, timeout_handler);
    alarm((unsigned)seconds);
    fprintf(stderr, "[Init] Timeout: %d seconds\n", seconds);
}

void timeout_cancel(void) {
    alarm(0);
    signal(SIGALRM, SIG_DFL);
}

/* ========================================================================
 * SYSCFG Peripheral (0x40004000)
 *
 * RP2040 datasheet section 2.22. Registers:
 *   0x00 PROC0_NMI_MASK      R/W  NMI mask for proc0
 *   0x04 PROC1_NMI_MASK      R/W  NMI mask for proc1
 *   0x08 PROC_CONFIG          RO   Processor configuration
 *   0x0C PROC_IN_SYNC_BYPASS  R/W  Input sync bypass
 *   0x10 PROC_IN_SYNC_BYPASS_HI R/W
 *   0x14 DBGFORCE             R/W  Debug force
 *   0x18 MEMPOWERDOWN         R/W  Memory power-down control
 * ======================================================================== */

static uint32_t syscfg_regs[SYSCFG_SIZE / 4];

int syscfg_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000u;
    return (base >= SYSCFG_BASE && base < SYSCFG_BASE + SYSCFG_SIZE);
}

uint32_t syscfg_read(uint32_t offset) {
    offset &= 0xFFF;
    if (offset >= SYSCFG_SIZE) return 0;
    if (offset == 0x08) {
        /* PROC_CONFIG: both processors out of reset */
        return 0x00000000;
    }
    return syscfg_regs[offset / 4];
}

void syscfg_write(uint32_t offset, uint32_t val) {
    offset &= 0xFFF;
    if (offset >= SYSCFG_SIZE) return;
    if (offset == 0x08) return; /* PROC_CONFIG is read-only */
    syscfg_regs[offset / 4] = val;
}

/* ========================================================================
 * TBMAN Peripheral (0x4006C000)
 *
 * RP2040 datasheet section 2.21. Registers:
 *   0x00 PLATFORM   RO  Bit 0 = ASIC, Bit 1 = FPGA
 *
 * In a real chip, PLATFORM reads 0x01 (ASIC). In simulation/emulation
 * it should read 0x02 (FPGA/sim) or a custom value.  We return 0x01
 * to match real hardware behavior (firmware expects ASIC).
 * ======================================================================== */

int tbman_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000u;
    return (base >= TBMAN_BASE && base < TBMAN_BASE + TBMAN_SIZE);
}

uint32_t tbman_read(uint32_t offset) {
    offset &= 0xFFF;
    if (offset == 0x00) {
        return 0x01;  /* ASIC platform */
    }
    return 0;
}

void tbman_write(uint32_t offset, uint32_t val) {
    (void)offset;
    (void)val;
    /* All TBMAN registers are read-only */
}

/* ========================================================================
 * ELF Symbol Table Loading
 * ======================================================================== */

int symbols_loaded = 0;

/* ELF32 section and symbol structures (local to this file) */
typedef struct { uint32_t sh_name, sh_type, sh_flags, sh_addr, sh_offset, sh_size;
                 uint32_t sh_link, sh_info, sh_addralign, sh_entsize; } elf32_shdr_t;
typedef struct { uint32_t st_name, st_value, st_size; uint8_t st_info, st_other;
                 uint16_t st_shndx; } elf32_sym_t;

#define SHT_SYMTAB 2
#define STT_FUNC   2

typedef struct {
    uint32_t addr;
    uint32_t size;
    char name[64];
} sym_entry_t;

static sym_entry_t *sym_table = NULL;
static int sym_count = 0;
static int sym_cap = 0;

static int sym_cmp(const void *a, const void *b) {
    const sym_entry_t *sa = a, *sb = b;
    if (sa->addr < sb->addr) return -1;
    if (sa->addr > sb->addr) return 1;
    return 0;
}

int symbols_load(const char *elf_path) {
    FILE *f = fopen(elf_path, "rb");
    if (!f) return 0;

    uint8_t ident[16];
    if (fread(ident, 1, 16, f) != 16 || ident[0] != 0x7f || ident[1] != 'E') {
        fclose(f); return 0;
    }

    /* Read ELF header fields we need */
    fseek(f, 32, SEEK_SET); /* e_shoff */
    uint32_t e_shoff; if (fread(&e_shoff, 4, 1, f) != 1) { fclose(f); return 0; }
    fseek(f, 46, SEEK_SET);
    uint16_t e_shentsize, e_shnum, e_shstrndx;
    if (fread(&e_shentsize, 2, 1, f) != 1 ||
        fread(&e_shnum, 2, 1, f) != 1 ||
        fread(&e_shstrndx, 2, 1, f) != 1) { fclose(f); return 0; }

    if (e_shnum == 0 || e_shentsize < 40) { fclose(f); return 0; }

    /* Read all section headers */
    elf32_shdr_t *shdrs = calloc(e_shnum, sizeof(elf32_shdr_t));
    if (!shdrs) { fclose(f); return 0; }
    fseek(f, (long)e_shoff, SEEK_SET);
    for (int i = 0; i < e_shnum; i++) {
        if (fread(&shdrs[i], sizeof(elf32_shdr_t), 1, f) != 1) break;
        if (e_shentsize > sizeof(elf32_shdr_t))
            fseek(f, e_shentsize - sizeof(elf32_shdr_t), SEEK_CUR);
    }

    /* Find .symtab */
    for (int i = 0; i < e_shnum; i++) {
        if (shdrs[i].sh_type != SHT_SYMTAB) continue;

        uint32_t strtab_idx = shdrs[i].sh_link;
        if (strtab_idx >= e_shnum) continue;

        /* Load string table */
        uint32_t str_size = shdrs[strtab_idx].sh_size;
        char *strtab = malloc(str_size);
        if (!strtab) continue;
        fseek(f, (long)shdrs[strtab_idx].sh_offset, SEEK_SET);
        if (fread(strtab, 1, str_size, f) != str_size) { free(strtab); continue; }

        /* Load symbols */
        uint32_t nsyms = shdrs[i].sh_size / sizeof(elf32_sym_t);
        fseek(f, (long)shdrs[i].sh_offset, SEEK_SET);

        sym_cap = 1024;
        sym_table = malloc((size_t)sym_cap * sizeof(sym_entry_t));
        sym_count = 0;

        for (uint32_t s = 0; s < nsyms; s++) {
            elf32_sym_t sym;
            if (fread(&sym, sizeof(sym), 1, f) != 1) break;
            if ((sym.st_info & 0xF) != STT_FUNC) continue;
            if (sym.st_value == 0) continue;
            if (sym.st_name >= str_size) continue;

            if (sym_count >= sym_cap) {
                sym_cap *= 2;
                sym_table = realloc(sym_table, (size_t)sym_cap * sizeof(sym_entry_t));
            }
            sym_entry_t *e = &sym_table[sym_count++];
            e->addr = sym.st_value & ~1u; /* Clear Thumb bit */
            e->size = sym.st_size;
            strncpy(e->name, &strtab[sym.st_name], sizeof(e->name) - 1);
            e->name[sizeof(e->name) - 1] = '\0';
        }

        free(strtab);
        break;
    }

    free(shdrs);
    fclose(f);

    if (sym_count > 0) {
        qsort(sym_table, (size_t)sym_count, sizeof(sym_entry_t), sym_cmp);
        symbols_loaded = 1;
        fprintf(stderr, "[Symbols] Loaded %d function symbols from %s\n", sym_count, elf_path);
    }
    return sym_count;
}

void symbols_cleanup(void) {
    free(sym_table);
    sym_table = NULL;
    sym_count = 0;
    symbols_loaded = 0;
}

const char *symbols_lookup(uint32_t addr, uint32_t *offset_out) {
    if (!symbols_loaded || sym_count == 0) return NULL;
    /* Binary search for largest addr <= target */
    int lo = 0, hi = sym_count - 1, best = -1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (sym_table[mid].addr <= addr) {
            best = mid;
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    if (best < 0) return NULL;
    uint32_t off = addr - sym_table[best].addr;
    if (sym_table[best].size > 0 && off >= sym_table[best].size) return NULL;
    if (offset_out) *offset_out = off;
    return sym_table[best].name;
}

/* ========================================================================
 * Scripted I/O
 * ======================================================================== */

int script_enabled = 0;

typedef struct {
    uint32_t time_us;
    int type;     /* 0=uart, 1=gpio */
    int channel;  /* uart num or gpio pin */
    uint8_t data[256];
    int data_len;
    int gpio_val;
    int fired;
} script_event_t;

static script_event_t *script_events = NULL;
static int script_event_count = 0;
static int script_event_cap = 0;

int script_init(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "[Script] Failed to open %s\n", path); return -1; }

    script_event_cap = 64;
    script_events = calloc((size_t)script_event_cap, sizeof(script_event_t));
    script_event_count = 0;

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n') continue;

        uint32_t ms;
        char cmd[32], arg[256];
        if (sscanf(line, "%ums: %31s %255[^\n]", &ms, cmd, arg) < 2) continue;

        if (script_event_count >= script_event_cap) {
            script_event_cap *= 2;
            script_events = realloc(script_events, (size_t)script_event_cap * sizeof(script_event_t));
        }
        script_event_t *ev = &script_events[script_event_count++];
        memset(ev, 0, sizeof(*ev));
        ev->time_us = ms * 1000;

        if (strncmp(cmd, "uart", 4) == 0) {
            ev->type = 0;
            ev->channel = cmd[4] - '0';
            /* Parse string: strip quotes, handle \n */
            int di = 0;
            int in_str = 0;
            for (int i = 0; arg[i] && di < 255; i++) {
                if (arg[i] == '"') { in_str = !in_str; continue; }
                if (arg[i] == '\\' && arg[i+1] == 'n') { ev->data[di++] = '\r'; i++; continue; }
                if (arg[i] == '\\' && arg[i+1] == 'r') { ev->data[di++] = '\r'; i++; continue; }
                ev->data[di++] = (uint8_t)arg[i];
            }
            ev->data_len = di;
        } else if (strncmp(cmd, "gpio", 4) == 0) {
            ev->type = 1;
            ev->channel = atoi(cmd + 4);
            ev->gpio_val = atoi(arg);
        }
    }
    fclose(f);
    script_enabled = 1;
    fprintf(stderr, "[Script] Loaded %d events from %s\n", script_event_count, path);
    return 0;
}

void script_poll(uint32_t elapsed_us) {
    if (!script_enabled) return;
    extern void uart_rx_push(int uart_num, uint8_t byte);
    extern void gpio_set_input_pin(uint8_t pin, uint8_t value);

    for (int i = 0; i < script_event_count; i++) {
        script_event_t *ev = &script_events[i];
        if (ev->fired || elapsed_us < ev->time_us) continue;
        ev->fired = 1;

        if (ev->type == 0) {
            for (int d = 0; d < ev->data_len; d++) {
                uart_rx_push(ev->channel, ev->data[d]);
            }
        } else if (ev->type == 1) {
            gpio_set_input_pin((uint8_t)ev->channel, (uint8_t)ev->gpio_val);
        }
    }
}

void script_cleanup(void) {
    free(script_events);
    script_events = NULL;
    script_event_count = 0;
    script_enabled = 0;
}

/* ========================================================================
 * Expected Output Matching
 * ======================================================================== */

int expect_enabled = 0;
char *expect_path = NULL;
char *expect_capture_buf = NULL;
size_t expect_capture_len = 0;
size_t expect_capture_cap = 0;

void expect_init(const char *path) {
    expect_path = strdup(path);
    expect_capture_cap = 4096;
    expect_capture_buf = malloc(expect_capture_cap);
    expect_capture_len = 0;
    expect_enabled = 1;
}

void expect_append(const char *data, size_t len) {
    if (!expect_enabled || !expect_capture_buf) return;
    while (expect_capture_len + len > expect_capture_cap) {
        expect_capture_cap *= 2;
        expect_capture_buf = realloc(expect_capture_buf, expect_capture_cap);
    }
    memcpy(expect_capture_buf + expect_capture_len, data, len);
    expect_capture_len += len;
}

int expect_check(void) {
    if (!expect_enabled || !expect_path) return 0;

    FILE *f = fopen(expect_path, "rb");
    if (!f) {
        fprintf(stderr, "[Expect] Cannot open golden file: %s\n", expect_path);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *golden = malloc((size_t)fsize);
    size_t nread = fread(golden, 1, (size_t)fsize, f);
    fclose(f);
    if ((long)nread != fsize) { free(golden); return 1; }

    int match = ((size_t)fsize == expect_capture_len &&
                 memcmp(golden, expect_capture_buf, expect_capture_len) == 0);

    if (match) {
        fprintf(stderr, "[Expect] Output matches golden file (%zu bytes)\n", expect_capture_len);
    } else {
        fprintf(stderr, "[Expect] Output DIFFERS from golden file\n");
        fprintf(stderr, "[Expect]   Expected: %ld bytes, Got: %zu bytes\n",
                fsize, expect_capture_len);
        /* Show first difference */
        size_t min_len = (size_t)fsize < expect_capture_len ? (size_t)fsize : expect_capture_len;
        for (size_t i = 0; i < min_len; i++) {
            if (golden[i] != expect_capture_buf[i]) {
                fprintf(stderr, "[Expect]   First diff at byte %zu: expected 0x%02X got 0x%02X\n",
                        i, (uint8_t)golden[i], (uint8_t)expect_capture_buf[i]);
                break;
            }
        }
    }
    free(golden);
    return match ? 0 : 1;
}

void expect_cleanup(void) {
    free(expect_path);
    free(expect_capture_buf);
    expect_path = NULL;
    expect_capture_buf = NULL;
    expect_enabled = 0;
}

/* ========================================================================
 * Memory Watch Log
 * ======================================================================== */

watch_region_t watch_regions[MAX_WATCH_REGIONS];
int watch_count = 0;

int watch_add(uint32_t addr, uint32_t len) {
    if (watch_count >= MAX_WATCH_REGIONS) return -1;
    watch_regions[watch_count].addr = addr;
    watch_regions[watch_count].len = len;
    watch_regions[watch_count].active = 1;
    watch_count++;
    fprintf(stderr, "[Watch] Monitoring 0x%08X..0x%08X\n", addr, addr + len);
    return 0;
}

void watch_check_write(uint32_t addr, uint32_t val, int width) {
    for (int i = 0; i < watch_count; i++) {
        if (!watch_regions[i].active) continue;
        if (addr >= watch_regions[i].addr &&
            addr < watch_regions[i].addr + watch_regions[i].len) {
            const char *sym = symbols_lookup(cpu.r[15], NULL);
            fprintf(stderr, "[Watch] WRITE%d 0x%08X = 0x%08X (PC=0x%08X%s%s)\n",
                    width * 8, addr, val, cpu.r[15],
                    sym ? " " : "", sym ? sym : "");
        }
    }
}

void watch_check_read(uint32_t addr, uint32_t val, int width) {
    for (int i = 0; i < watch_count; i++) {
        if (!watch_regions[i].active) continue;
        if (addr >= watch_regions[i].addr &&
            addr < watch_regions[i].addr + watch_regions[i].len) {
            const char *sym = symbols_lookup(cpu.r[15], NULL);
            fprintf(stderr, "[Watch] READ%d  0x%08X → 0x%08X (PC=0x%08X%s%s)\n",
                    width * 8, addr, val, cpu.r[15],
                    sym ? " " : "", sym ? sym : "");
        }
    }
}

/* ========================================================================
 * Call Graph
 * ======================================================================== */

int callgraph_enabled = 0;

typedef struct {
    uint32_t caller;
    uint32_t callee;
    uint32_t count;
} callgraph_edge_t;

#define CALLGRAPH_MAP_SIZE (1 << 16)
#define CALLGRAPH_MAP_MASK (CALLGRAPH_MAP_SIZE - 1)

static callgraph_edge_t *cg_map = NULL;

void callgraph_init(void) {
    cg_map = calloc(CALLGRAPH_MAP_SIZE, sizeof(callgraph_edge_t));
    if (cg_map) callgraph_enabled = 1;
}

void callgraph_record_call(uint32_t caller_pc, uint32_t target_pc) {
    if (!callgraph_enabled) return;
    uint32_t hash = ((caller_pc >> 1) ^ (target_pc >> 1) ^ (caller_pc >> 17)) & CALLGRAPH_MAP_MASK;
    callgraph_edge_t *e = &cg_map[hash];
    if (e->caller == caller_pc && e->callee == target_pc) {
        e->count++;
    } else if (e->count == 0) {
        e->caller = caller_pc;
        e->callee = target_pc;
        e->count = 1;
    } else {
        /* Collision — overwrite (frequency bias) */
        e->caller = caller_pc;
        e->callee = target_pc;
        e->count++;
    }
}

void callgraph_dump(const char *path) {
    if (!cg_map) return;
    FILE *f = fopen(path, "w");
    if (!f) { fprintf(stderr, "[CallGraph] Failed to open %s\n", path); return; }

    fprintf(f, "digraph callgraph {\n");
    fprintf(f, "  rankdir=LR;\n");
    fprintf(f, "  node [shape=box, fontsize=10];\n");

    for (int i = 0; i < CALLGRAPH_MAP_SIZE; i++) {
        if (cg_map[i].count == 0) continue;
        const char *caller_name = symbols_lookup(cg_map[i].caller, NULL);
        const char *callee_name = symbols_lookup(cg_map[i].callee, NULL);

        if (caller_name && callee_name) {
            fprintf(f, "  \"%s\" -> \"%s\" [label=\"%u\"];\n",
                    caller_name, callee_name, cg_map[i].count);
        } else {
            fprintf(f, "  \"0x%08X\" -> \"0x%08X\" [label=\"%u\"];\n",
                    cg_map[i].caller, cg_map[i].callee, cg_map[i].count);
        }
    }
    fprintf(f, "}\n");
    fclose(f);
    fprintf(stderr, "[CallGraph] Written to %s\n", path);
}

void callgraph_cleanup(void) {
    free(cg_map);
    cg_map = NULL;
    callgraph_enabled = 0;
}

/* ========================================================================
 * Stack Watermark
 * ======================================================================== */

int stack_check_enabled = 0;
uint32_t stack_watermark[2] = { 0xFFFFFFFF, 0xFFFFFFFF };

void stack_check_report(void) {
    if (!stack_check_enabled) return;
    for (int c = 0; c < 2; c++) {
        if (stack_watermark[c] == 0xFFFFFFFF) continue;
        uint32_t stack_top = RAM_BASE + RAM_SIZE;
        uint32_t used = stack_top - stack_watermark[c];
        fprintf(stderr, " Stack Core %d: low watermark 0x%08X (peak %u bytes used)\n",
                c, stack_watermark[c], used);
        if (stack_watermark[c] < RAM_BASE + 1024) {
            fprintf(stderr, " WARNING: Core %d stack within 1KB of RAM base!\n", c);
        }
    }
}

/* ========================================================================
 * IRQ Latency Profiling
 * ======================================================================== */

int irq_latency_enabled = 0;
irq_latency_entry_t irq_latency[IRQ_LATENCY_MAX_IRQS];
uint64_t global_cycle_count = 0;

void irq_latency_pend(uint32_t irq) {
    if (!irq_latency_enabled || irq >= IRQ_LATENCY_MAX_IRQS) return;
    irq_latency[irq].pend_cycle = global_cycle_count;
}

void irq_latency_enter(uint32_t irq) {
    if (!irq_latency_enabled || irq >= IRQ_LATENCY_MAX_IRQS) return;
    if (irq_latency[irq].pend_cycle == 0) return;

    uint32_t lat = (uint32_t)(global_cycle_count - irq_latency[irq].pend_cycle);
    irq_latency_entry_t *e = &irq_latency[irq];
    e->total_cycles += lat;
    e->count++;
    if (lat < e->min_cycles || e->min_cycles == 0) e->min_cycles = lat;
    if (lat > e->max_cycles) e->max_cycles = lat;
    e->pend_cycle = 0;
}

void irq_latency_report(void) {
    if (!irq_latency_enabled) return;
    fprintf(stderr, " IRQ Latency (cycles):\n");
    fprintf(stderr, "   %-6s  %-8s  %-8s  %-8s  %-8s\n",
            "IRQ", "Count", "Min", "Avg", "Max");
    for (int i = 0; i < IRQ_LATENCY_MAX_IRQS; i++) {
        if (irq_latency[i].count == 0) continue;
        uint32_t avg = (uint32_t)(irq_latency[i].total_cycles / irq_latency[i].count);
        fprintf(stderr, "   %-6d  %-8u  %-8u  %-8u  %-8u\n",
                i, irq_latency[i].count, irq_latency[i].min_cycles,
                avg, irq_latency[i].max_cycles);
    }
}

/* ========================================================================
 * Bus Transaction Logging
 * ======================================================================== */

int log_uart_enabled = 0;
int log_spi_enabled = 0;
int log_i2c_enabled = 0;

void bus_log_uart(int num, int is_tx, uint8_t byte) {
    if (!log_uart_enabled) return;
    char printable = (byte >= 0x20 && byte < 0x7F) ? (char)byte : '.';
    fprintf(stderr, "[UART%d] %s 0x%02X '%c'\n", num, is_tx ? "TX" : "RX", byte, printable);
}

void bus_log_spi(int num, int is_tx, uint8_t byte) {
    if (!log_spi_enabled) return;
    fprintf(stderr, "[SPI%d] %s 0x%02X\n", num, is_tx ? "MOSI" : "MISO", byte);
}

void bus_log_i2c(int num, int is_write, uint8_t addr7, uint8_t byte) {
    if (!log_i2c_enabled) return;
    fprintf(stderr, "[I2C%d] addr=0x%02X %s 0x%02X\n",
            num, addr7, is_write ? "W" : "R", byte);
}

/* ========================================================================
 * GPIO VCD Trace
 * ======================================================================== */

int gpio_trace_enabled = 0;
static FILE *vcd_file = NULL;
static uint32_t vcd_prev_pins = 0;

void gpio_trace_init(const char *path) {
    vcd_file = fopen(path, "w");
    if (!vcd_file) { fprintf(stderr, "[VCD] Failed to open %s\n", path); return; }

    /* VCD header */
    fprintf(vcd_file, "$timescale 1us $end\n");
    fprintf(vcd_file, "$scope module gpio $end\n");
    for (int i = 0; i < 30; i++) {
        fprintf(vcd_file, "$var wire 1 %c gpio%d $end\n", (char)('!' + i), i);
    }
    fprintf(vcd_file, "$upscope $end\n");
    fprintf(vcd_file, "$enddefinitions $end\n");
    fprintf(vcd_file, "#0\n");
    /* Initial state: all low */
    for (int i = 0; i < 30; i++) {
        fprintf(vcd_file, "0%c\n", (char)('!' + i));
    }

    vcd_prev_pins = 0;
    gpio_trace_enabled = 1;
    fprintf(stderr, "[VCD] GPIO trace → %s\n", path);
}

void gpio_trace_record(uint8_t pin, uint8_t value) {
    if (!gpio_trace_enabled || !vcd_file || pin >= 30) return;

    uint32_t mask = 1u << pin;
    uint32_t new_pins = (vcd_prev_pins & ~mask) | ((value ? 1u : 0u) << pin);
    if (new_pins == vcd_prev_pins) return;

    /* Timestamp in microseconds from global cycle count */
    uint32_t us = (timing_config.cycles_per_us > 0)
        ? (uint32_t)(global_cycle_count / timing_config.cycles_per_us)
        : (uint32_t)global_cycle_count;

    fprintf(vcd_file, "#%u\n%d%c\n", us, value ? 1 : 0, (char)('!' + pin));
    vcd_prev_pins = new_pins;
}

void gpio_trace_cleanup(void) {
    if (vcd_file) fclose(vcd_file);
    vcd_file = NULL;
    gpio_trace_enabled = 0;
}

/* ========================================================================
 * Fault Injection
 * ======================================================================== */

fault_injection_t fault_injections[MAX_FAULT_INJECTIONS];
int fault_count = 0;

int fault_add(const char *spec) {
    if (fault_count >= MAX_FAULT_INJECTIONS) return -1;

    fault_injection_t *fi = &fault_injections[fault_count];
    memset(fi, 0, sizeof(*fi));

    char type[32];
    uint64_t cycle;
    uint32_t addr = 0;

    if (sscanf(spec, "%31[^:]:%lu:%x", type, &cycle, &addr) < 2) {
        fprintf(stderr, "[Fault] Invalid spec: %s\n", spec);
        return -1;
    }

    if (strcmp(type, "flash_bitflip") == 0) {
        fi->type = FAULT_FLASH_BITFLIP;
        fi->addr = addr;
    } else if (strcmp(type, "ram_corrupt") == 0) {
        fi->type = FAULT_RAM_CORRUPT;
        fi->addr = addr;
    } else if (strcmp(type, "brownout") == 0) {
        fi->type = FAULT_BROWNOUT;
    } else {
        fprintf(stderr, "[Fault] Unknown type: %s\n", type);
        return -1;
    }

    fi->trigger_cycle = cycle;
    fi->fired = 0;
    fault_count++;
    fprintf(stderr, "[Fault] Scheduled %s at cycle %lu\n", type, (unsigned long)cycle);
    return 0;
}

void fault_check(uint64_t cycle) {
    extern int watchdog_reboot_pending;
    for (int i = 0; i < fault_count; i++) {
        fault_injection_t *fi = &fault_injections[i];
        if (fi->fired || cycle < fi->trigger_cycle) continue;
        fi->fired = 1;

        switch (fi->type) {
        case FAULT_FLASH_BITFLIP:
            if (fi->addr < FLASH_SIZE) {
                cpu.flash[fi->addr] ^= 0x01;
                fprintf(stderr, "[Fault] Flash bit flip at 0x%08X (cycle %lu)\n",
                        FLASH_BASE + fi->addr, (unsigned long)cycle);
            }
            break;
        case FAULT_RAM_CORRUPT:
            if (fi->addr >= RAM_BASE && fi->addr < RAM_BASE + RAM_SIZE) {
                uint32_t off = fi->addr - RAM_BASE;
                cpu.ram[off] = 0xDE;
                fprintf(stderr, "[Fault] RAM corruption at 0x%08X (cycle %lu)\n",
                        fi->addr, (unsigned long)cycle);
            }
            break;
        case FAULT_BROWNOUT:
            watchdog_reboot_pending = 1;
            fprintf(stderr, "[Fault] Brownout reset (cycle %lu)\n", (unsigned long)cycle);
            break;
        }
    }
}

/* ========================================================================
 * Cycle Profiling
 * ======================================================================== */

int profile_enabled = 0;
profile_entry_t *profile_map = NULL;

void profile_init(void) {
    profile_map = calloc(PROFILE_MAP_SIZE, sizeof(profile_entry_t));
    if (profile_map) profile_enabled = 1;
}

void profile_cleanup(void) {
    free(profile_map);
    profile_map = NULL;
    profile_enabled = 0;
}

static int profile_cmp(const void *a, const void *b) {
    const profile_entry_t *pa = a, *pb = b;
    if (pb->total_cycles > pa->total_cycles) return 1;
    if (pb->total_cycles < pa->total_cycles) return -1;
    return 0;
}

void profile_dump(const char *path) {
    if (!profile_map) return;
    FILE *f = fopen(path, "w");
    if (!f) { fprintf(stderr, "[Profile] Failed to open %s\n", path); return; }

    fprintf(f, "address,cycles,count,avg_cycles,function\n");
    for (int i = 0; i < PROFILE_MAP_SIZE; i++) {
        if (profile_map[i].count == 0) continue;
        uint32_t off = 0;
        const char *name = symbols_lookup(profile_map[i].pc, &off);
        fprintf(f, "0x%08X,%u,%u,%u,%s+0x%X\n",
                profile_map[i].pc, profile_map[i].total_cycles,
                profile_map[i].count,
                profile_map[i].total_cycles / profile_map[i].count,
                name ? name : "???", off);
    }
    fclose(f);
    fprintf(stderr, "[Profile] Written to %s\n", path);
}

void profile_report(void) {
    if (!profile_map) return;

    /* Collect and sort top 20 */
    int count = 0;
    for (int i = 0; i < PROFILE_MAP_SIZE; i++) {
        if (profile_map[i].count > 0) count++;
    }
    if (count == 0) return;

    int n = count < 20 ? count : 20;
    profile_entry_t *sorted = malloc((size_t)count * sizeof(profile_entry_t));
    if (!sorted) return;

    int si = 0;
    for (int i = 0; i < PROFILE_MAP_SIZE; i++) {
        if (profile_map[i].count > 0) sorted[si++] = profile_map[i];
    }
    qsort(sorted, (size_t)count, sizeof(profile_entry_t), profile_cmp);

    fprintf(stderr, " Cycle Profile (top %d of %d):\n", n, count);
    fprintf(stderr, "   %-12s  %-12s  %-8s  %s\n", "Address", "Cycles", "Count", "Function");
    for (int i = 0; i < n; i++) {
        uint32_t off = 0;
        const char *name = symbols_lookup(sorted[i].pc, &off);
        if (name) {
            fprintf(stderr, "   0x%08X  %-12u  %-8u  %s+0x%X\n",
                    sorted[i].pc, sorted[i].total_cycles, sorted[i].count, name, off);
        } else {
            fprintf(stderr, "   0x%08X  %-12u  %-8u\n",
                    sorted[i].pc, sorted[i].total_cycles, sorted[i].count);
        }
    }
    free(sorted);
}

/* ========================================================================
 * Memory Access Heatmap
 * ======================================================================== */

int mem_heatmap_enabled = 0;
heatmap_entry_t *heatmap_ram = NULL;

void mem_heatmap_init(void) {
    heatmap_ram = calloc(HEATMAP_RAM_BLOCKS, sizeof(heatmap_entry_t));
    if (heatmap_ram) mem_heatmap_enabled = 1;
}

void mem_heatmap_cleanup(void) {
    free(heatmap_ram);
    heatmap_ram = NULL;
    mem_heatmap_enabled = 0;
}

void mem_heatmap_dump(const char *path) {
    if (!heatmap_ram) return;
    FILE *f = fopen(path, "w");
    if (!f) { fprintf(stderr, "[Heatmap] Failed to open %s\n", path); return; }

    fprintf(f, "block_addr,reads,writes,total\n");
    for (uint32_t i = 0; i < HEATMAP_RAM_BLOCKS; i++) {
        if (heatmap_ram[i].reads == 0 && heatmap_ram[i].writes == 0) continue;
        fprintf(f, "0x%08X,%u,%u,%u\n",
                RAM_BASE + (i << HEATMAP_BLOCK_SHIFT),
                heatmap_ram[i].reads, heatmap_ram[i].writes,
                heatmap_ram[i].reads + heatmap_ram[i].writes);
    }
    fclose(f);
    fprintf(stderr, "[Heatmap] Written to %s\n", path);
}
