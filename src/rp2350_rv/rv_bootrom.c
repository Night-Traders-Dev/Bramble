/*
 * RP2350 RISC-V Bootrom
 *
 * Generates RISC-V bootrom with:
 *   - Reset vector → boot code (SP init, mtvec, flash jump)
 *   - ROM function table compatible with Pico SDK rom_func_lookup()
 *   - Function stubs at well-known addresses (intercepted by rv_rom_intercept)
 *   - Trap handler at 0x0004
 */

#include <string.h>
#include <stdio.h>
#include "rp2350_rv/rv_bootrom.h"
#include "rp2350_rv/rv_membus.h"
#include "rp2350_rv/rp2350_memmap.h"
#include "rp2350_rv/rv_icache.h"
#include "emulator.h"

#define RV_ROM_DATA_FLASH_DEVINFO16_PTR_LITERAL 0x0200
#define RV_ROM_DATA_FLASH_DEVINFO16_WORD        0x0204
#define RV_ROM_DATA_GIT_REVISION_STRING         0x0210
#define RV_ROM_DATA_TABLE_BASE                  0x0280
#define RV_ROM_SYS_INFO_SUPPORTED_FLAGS \
    (RV_SYS_INFO_CHIP_INFO | RV_SYS_INFO_CRITICAL | RV_SYS_INFO_CPU_INFO | \
     RV_SYS_INFO_FLASH_DEV_INFO | RV_SYS_INFO_BOOT_RANDOM | RV_SYS_INFO_NONCE | \
     RV_SYS_INFO_BOOT_INFO)

/* Helper: write 32-bit LE word to ROM buffer */
static void rom_w32(uint8_t *rom, uint32_t off, uint32_t val) {
    rom[off+0] = (uint8_t)(val);
    rom[off+1] = (uint8_t)(val >> 8);
    rom[off+2] = (uint8_t)(val >> 16);
    rom[off+3] = (uint8_t)(val >> 24);
}

static void rom_w16(uint8_t *rom, uint32_t off, uint16_t val) {
    rom[off+0] = (uint8_t)(val);
    rom[off+1] = (uint8_t)(val >> 8);
}

/* RISC-V instruction encoders */
static uint32_t rv_jal(uint32_t rd, int32_t off) {
    uint32_t imm = (uint32_t)off;
    uint32_t enc = (rd << 7) | 0x6F;
    enc |= ((imm >> 12) & 0xFF) << 12;
    enc |= ((imm >> 11) & 1) << 20;
    enc |= ((imm >> 1) & 0x3FF) << 21;
    enc |= ((imm >> 20) & 1) << 31;
    return enc;
}

static uint32_t rv_lui(uint32_t rd, uint32_t imm_upper) {
    return (imm_upper & 0xFFFFF000) | (rd << 7) | 0x37;
}

static uint32_t rv_addi(uint32_t rd, uint32_t rs1, int32_t imm) {
    return ((uint32_t)imm << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x13;
}

static uint32_t rv_jalr(uint32_t rd, uint32_t rs1, int32_t imm) {
    return ((uint32_t)imm << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x67;
}

/* CSRRW rd, csr, rs1 */
static uint32_t rv_csrrw(uint32_t rd, uint16_t csr, uint32_t rs1) {
    return ((uint32_t)csr << 20) | (rs1 << 15) | (1 << 12) | (rd << 7) | 0x73;
}

/* RET = JALR x0, x1, 0 */
static uint32_t rv_ret(void) {
    return rv_jalr(0, 1, 0);
}

static uint16_t rv_rom_flash_size_code(uint32_t flash_size) {
    uint32_t size = 8 * 1024;
    uint16_t code = 1;

    while (size < flash_size && code < 0x0C) {
        size <<= 1;
        code++;
    }
    return code;
}

static uint16_t rv_rom_flash_devinfo_word(uint32_t flash_size) {
    /* CS0 size in bits 8..11, D8h block erase support in bit 7. */
    return (uint16_t)(0x0080u | (rv_rom_flash_size_code(flash_size) << 8));
}

static int rv_rom_is_stub_pc(uint32_t pc) {
    return pc == RV_ROM_FN_TABLE_LOOKUP ||
           pc == RV_ROM_FN_TABLE_LOOKUP_ENTRY ||
           (pc >= RV_ROM_FN_MEMCPY && pc < RV_ROM_FN_LAST);
}

static uint32_t rv_rom_lookup_address(uint32_t code, uint32_t mask) {
    if (mask == RV_ROM_RT_FLAG_DATA) {
        switch (code) {
        case RV_ROM_DATA_SOFTWARE_GIT_REVISION:
            return RV_ROM_DATA_GIT_REVISION_STRING;
        case RV_ROM_DATA_FLASH_DEVINFO16_PTR:
            return RV_ROM_DATA_FLASH_DEVINFO16_PTR_LITERAL;
        default:
            return 0;
        }
    }

    if (mask != RV_ROM_RT_FLAG_FUNC_RISCV && mask != RV_ROM_RT_FLAG_FUNC_RISCV_FAR) {
        return 0;
    }

    switch (code) {
    case RV_ROM_CODE_FLASH_ENTER_CMD_XIP:
        return RV_ROM_FN_FLASH_ENTER;
    case RV_ROM_CODE_FLASH_EXIT_XIP:
        return RV_ROM_FN_FLASH_EXIT;
    case RV_ROM_CODE_FLASH_FLUSH_CACHE:
        return RV_ROM_FN_FLASH_FLUSH_CACHE;
    case RV_ROM_CODE_CONNECT_INTERNAL_FLASH:
        return RV_ROM_FN_CONNECT_INTERNAL_FLASH;
    case RV_ROM_CODE_FLASH_ERASE:
        return RV_ROM_FN_FLASH_ERASE;
    case RV_ROM_CODE_FLASH_PROG:
        return RV_ROM_FN_FLASH_PROGRAM;
    case RV_ROM_CODE_BOOTROM_STATE_RESET:
        return RV_ROM_FN_BOOTROM_STATE_RESET;
    case RV_ROM_CODE_GET_SYS_INFO:
        return RV_ROM_FN_GET_SYS_INFO;
    case RV_ROM_CODE_REBOOT:
        return RV_ROM_FN_REBOOT;
    case RV_ROM_CODE_SET_BOOTROM_STACK:
        return RV_ROM_FN_SET_STACK;
    default:
        return 0;
    }
}

static int rv_rom_get_sys_info(rv_membus_state_t *bus, uint32_t out_addr,
                               uint32_t out_words, uint32_t flags) {
    uint32_t included = flags & RV_ROM_SYS_INFO_SUPPORTED_FLAGS;
    uint32_t words[16];
    uint32_t count = 0;

    if (!included) {
        return RV_BOOTROM_ERROR_INVALID_ARG;
    }

    words[count++] = included;

    if (included & RV_SYS_INFO_CHIP_INFO) {
        words[count++] = 0x00000000; /* package_id */
        words[count++] = 0xB2350001; /* device_id_lo */
        words[count++] = 0x00000001; /* device_id_hi */
    }
    if (included & RV_SYS_INFO_CRITICAL) {
        words[count++] = 0x00000008; /* default arch = RISC-V, debug enabled */
    }
    if (included & RV_SYS_INFO_CPU_INFO) {
        words[count++] = RV_PICOBIN_CPU_RISCV | (0x02u << 8);
    }
    if (included & RV_SYS_INFO_FLASH_DEV_INFO) {
        words[count++] = rv_rom_flash_devinfo_word(bus->flash_size);
    }
    if (included & RV_SYS_INFO_BOOT_RANDOM) {
        words[count++] = 0x2350C0DE;
        words[count++] = 0x12345678;
        words[count++] = 0x89ABCDEF;
        words[count++] = 0x0BADF00D;
    }
    if (included & RV_SYS_INFO_NONCE) {
        words[count++] = 0x10203040;
        words[count++] = 0x50607080;
    }
    if (included & RV_SYS_INFO_BOOT_INFO) {
        words[count++] = (uint32_t)RV_BOOT_PARTITION_NONE |
                         ((uint32_t)RV_BOOT_TYPE_NORMAL << 8) |
                         ((uint32_t)RV_BOOT_PARTITION_NONE << 16);
        words[count++] = 0;
        words[count++] = 0;
        words[count++] = 0;
    }

    if (out_words < count) {
        return RV_BOOTROM_ERROR_BUFFER_TOO_SMALL;
    }

    for (uint32_t i = 0; i < count; i++) {
        rv_mem_write32(bus, out_addr + i * 4, words[i]);
    }

    return (int)count;
}

/* ========================================================================
 * ROM Initialization
 * ======================================================================== */

uint32_t rv_bootrom_init(uint8_t *rom, uint32_t rom_size,
                         uint32_t flash_base, uint32_t sram_end) {
    static const struct { uint32_t code; uint32_t addr; } fn_table[] = {
        { RV_ROM_CODE_FLASH_ENTER_CMD_XIP,   RV_ROM_FN_FLASH_ENTER },
        { RV_ROM_CODE_FLASH_EXIT_XIP,        RV_ROM_FN_FLASH_EXIT },
        { RV_ROM_CODE_FLASH_FLUSH_CACHE,     RV_ROM_FN_FLASH_FLUSH_CACHE },
        { RV_ROM_CODE_CONNECT_INTERNAL_FLASH, RV_ROM_FN_CONNECT_INTERNAL_FLASH },
        { RV_ROM_CODE_FLASH_ERASE,           RV_ROM_FN_FLASH_ERASE },
        { RV_ROM_CODE_FLASH_PROG,            RV_ROM_FN_FLASH_PROGRAM },
        { RV_ROM_CODE_BOOTROM_STATE_RESET,   RV_ROM_FN_BOOTROM_STATE_RESET },
        { RV_ROM_CODE_GET_SYS_INFO,          RV_ROM_FN_GET_SYS_INFO },
        { RV_ROM_CODE_REBOOT,                RV_ROM_FN_REBOOT },
        { RV_ROM_CODE_SET_BOOTROM_STACK,     RV_ROM_FN_SET_STACK },
        { 0, 0 }
    };
    static const struct { uint32_t code; uint32_t addr; } data_table[] = {
        { RV_ROM_DATA_SOFTWARE_GIT_REVISION, RV_ROM_DATA_GIT_REVISION_STRING },
        { RV_ROM_DATA_FLASH_DEVINFO16_PTR,   RV_ROM_DATA_FLASH_DEVINFO16_PTR_LITERAL },
        { 0, 0 }
    };

    memset(rom, 0, rom_size);

    /* 0x0000: JAL x0, 0x20 (jump to boot code at 0x20) */
    rom_w32(rom, 0x0000, rv_jal(0, 0x20));

    /* 0x0004: Trap handler — C.J self (infinite loop) */
    rom_w16(rom, 0x0004, 0xA001);

    /* 0x0010: Magic and table pointers (RP2350 ROM header) */
    rom[0x10] = 'R'; rom[0x11] = 'P'; rom[0x12] = 0x02; rom[0x13] = 0x00;
    rom_w32(rom, 0x14, 0x0100);  /* Function table at 0x0100 */
    rom_w32(rom, 0x18, RV_ROM_DATA_TABLE_BASE);  /* Data table */
    rom_w32(rom, 0x1C, RV_ROM_FN_TABLE_LOOKUP); /* Lookup function */
    rom_w16(rom, RV_ROM_WELL_KNOWN_LOOKUP_PTR, RV_ROM_FN_TABLE_LOOKUP);
    rom_w16(rom, RV_ROM_WELL_KNOWN_LOOKUP_ENTRY, RV_ROM_FN_TABLE_LOOKUP_ENTRY);
    rom_w16(rom, RV_ROM_WELL_KNOWN_ENTRY, 0x0000);

    /* 0x0020: Boot code */
    uint32_t sp_upper = sram_end & 0xFFFFF000;
    int32_t sp_lower = (int32_t)(sram_end & 0xFFF);
    if (sp_lower & 0x800) { sp_upper += 0x1000; sp_lower -= 0x1000; }

    uint32_t pc = 0x20;
    /* Set SP (x2) */
    rom_w32(rom, pc, rv_lui(2, sp_upper)); pc += 4;
    if (sp_lower != 0) {
        rom_w32(rom, pc, rv_addi(2, 2, sp_lower)); pc += 4;
    }
    /* Set GP (x3) to SRAM base */
    rom_w32(rom, pc, rv_lui(3, RP2350_SRAM_BASE)); pc += 4;
    /* Set mtvec to 0x0004 (trap handler) — CSRRW x0, mtvec, t0 */
    rom_w32(rom, pc, rv_addi(5, 0, 4)); pc += 4;       /* t0 = 4 */
    rom_w32(rom, pc, rv_csrrw(0, 0x305, 5)); pc += 4;  /* mtvec = t0 */
    /* Jump to flash */
    rom_w32(rom, pc, rv_lui(5, flash_base)); pc += 4;
    rom_w32(rom, pc, rv_jalr(0, 5, 0)); pc += 4;

    /* 0x0100: Function table entries [32-bit code, 32-bit address] */
    uint32_t tbl = 0x0100;
    for (int i = 0; fn_table[i].code != 0; i++) {
        rom_w32(rom, tbl, fn_table[i].code);
        rom_w32(rom, tbl + 4, fn_table[i].addr);
        tbl += 8;
    }
    rom_w32(rom, tbl, 0);  /* End sentinel */

    /* 0x0200: Data blobs returned by rom_data_lookup() */
    rom_w32(rom, RV_ROM_DATA_FLASH_DEVINFO16_PTR_LITERAL, RV_ROM_DATA_FLASH_DEVINFO16_WORD);
    rom_w16(rom, RV_ROM_DATA_FLASH_DEVINFO16_WORD, rv_rom_flash_devinfo_word(RP2350_FLASH_DEFAULT));
    memcpy(&rom[RV_ROM_DATA_GIT_REVISION_STRING], "bramble-rv", sizeof("bramble-rv"));

    /* 0x0280: Data table entries [32-bit code, 32-bit address] */
    tbl = RV_ROM_DATA_TABLE_BASE;
    for (int i = 0; data_table[i].code != 0; i++) {
        rom_w32(rom, tbl, data_table[i].code);
        rom_w32(rom, tbl + 4, data_table[i].addr);
        tbl += 8;
    }
    rom_w32(rom, tbl, 0);

    /* 0x0300: RP2350 well-known lookup entrypoints. The actual dispatch happens
     * in rv_rom_intercept(), so the ROM bodies are simple RET stubs.
     */
    rom_w32(rom, RV_ROM_FN_TABLE_LOOKUP, rv_ret());
    rom_w32(rom, RV_ROM_FN_TABLE_LOOKUP_ENTRY, rv_ret());

    /* 0x0400+: Function stubs — all are simple RET (intercepted by rv_rom_intercept) */
    for (uint32_t addr = RV_ROM_FN_MEMCPY; addr < RV_ROM_FN_LAST && addr + 4 <= rom_size; addr += 4) {
        rom_w32(rom, addr, rv_ret());
    }

    fprintf(stderr, "[RV-BOOT] ROM initialized: SP=0x%08X, entry=0x%08X, %d ROM functions, %d data entries\n",
            sram_end, flash_base,
            (int)(sizeof(fn_table) / sizeof(fn_table[0]) - 1),
            (int)(sizeof(data_table) / sizeof(data_table[0]) - 1));

    return 0x00000000;
}

/* ========================================================================
 * ROM Function Interception
 *
 * When PC hits a ROM function stub, perform the operation natively in C
 * and return to caller. Arguments in a0-a2 (x10-x12), result in a0 (x10).
 * ======================================================================== */

int rv_rom_intercept(rv_cpu_state_t *cpu) {
    uint32_t pc = cpu->pc;
    if (!rv_rom_is_stub_pc(pc)) return 0;

    rv_membus_state_t *bus = (rv_membus_state_t *)cpu->bus;
    if (!bus) return 0;

    uint32_t a0 = cpu->x[10];
    uint32_t a1 = cpu->x[11];
    uint32_t a2 = cpu->x[12];

    switch (pc) {
    case RV_ROM_FN_TABLE_LOOKUP:
    case RV_ROM_FN_TABLE_LOOKUP_ENTRY:
        cpu->x[10] = rv_rom_lookup_address(a0, a1);
        if (cpu->debug_enabled && cpu->x[10] == 0) {
            fprintf(stderr, "[RV-ROM] unresolved lookup code=0x%04X mask=0x%X\n",
                    a0 & 0xFFFF, a1);
        }
        break;

    case RV_ROM_FN_MEMCPY:
    case RV_ROM_FN_MEMCPY4: {
        /* memcpy(dst, src, len) — a0=dst, a1=src, a2=len */
        for (uint32_t i = 0; i < a2; i++) {
            uint8_t b = rv_mem_read8(bus, a1 + i);
            rv_mem_write8(bus, a0 + i, b);
        }
        /* Return dst in a0 (already there) */
        break;
    }

    case RV_ROM_FN_MEMSET:
    case RV_ROM_FN_MEMSET4: {
        /* memset(dst, val, len) — a0=dst, a1=val, a2=len */
        uint8_t val = (uint8_t)(a1 & 0xFF);
        for (uint32_t i = 0; i < a2; i++) {
            rv_mem_write8(bus, a0 + i, val);
        }
        break;
    }

    case RV_ROM_FN_POPCOUNT32:
        cpu->x[10] = (uint32_t)__builtin_popcount(a0);
        break;

    case RV_ROM_FN_CLZ32:
        cpu->x[10] = a0 ? (uint32_t)__builtin_clz(a0) : 32;
        break;

    case RV_ROM_FN_CTZ32:
        cpu->x[10] = a0 ? (uint32_t)__builtin_ctz(a0) : 32;
        break;

    case RV_ROM_FN_REVERSE32: {
        uint32_t v = a0;
        v = ((v >> 1) & 0x55555555) | ((v & 0x55555555) << 1);
        v = ((v >> 2) & 0x33333333) | ((v & 0x33333333) << 2);
        v = ((v >> 4) & 0x0F0F0F0F) | ((v & 0x0F0F0F0F) << 4);
        v = ((v >> 8) & 0x00FF00FF) | ((v & 0x00FF00FF) << 8);
        v = (v >> 16) | (v << 16);
        cpu->x[10] = v;
        break;
    }

    case RV_ROM_FN_FLASH_ENTER:
    case RV_ROM_FN_FLASH_EXIT:
    case RV_ROM_FN_FLASH_FLUSH_CACHE:
    case RV_ROM_FN_CONNECT_INTERNAL_FLASH:
    case RV_ROM_FN_BOOTROM_STATE_RESET:
        /* Flash connect/exit XIP — no-op in emulator */
        break;

    case RV_ROM_FN_FLASH_ERASE: {
        /* flash_range_erase(offset, count) — a0=offset, a1=count */
        uint32_t offset = a0;
        uint32_t count = a1;
        if (offset + count <= bus->flash_size) {
            memset(&bus->flash[offset], 0xFF, count);
            /* The instruction cache holds decoded words fetched from flash, so
             * it only stays valid while flash is immutable. Erasing without
             * invalidating left the hart executing the erased instructions, so
             * an update-to-flash wrote the bytes and then jumped into the old
             * code. */
            if (cpu->icache)
                rv_icache_invalidate_range((rv_icache_t *)cpu->icache,
                                           RP2350_FLASH_BASE + offset, count);
            if (cpu->debug_enabled)
                fprintf(stderr, "[RV-ROM] flash_range_erase(0x%X, %u)\n", offset, count);
        }
        break;
    }

    case RV_ROM_FN_FLASH_PROGRAM: {
        /* flash_range_program(offset, data, count) — a0=offset, a1=data_ptr, a2=count */
        uint32_t offset = a0;
        uint32_t count = a2;
        if (offset + count <= bus->flash_size) {
            for (uint32_t i = 0; i < count; i++) {
                bus->flash[offset + i] = rv_mem_read8(bus, a1 + i);
            }
            /* See flash_range_erase: stale decoded instructions must go. */
            if (cpu->icache)
                rv_icache_invalidate_range((rv_icache_t *)cpu->icache,
                                           RP2350_FLASH_BASE + offset, count);
            if (cpu->debug_enabled)
                fprintf(stderr, "[RV-ROM] flash_range_program(0x%X, %u bytes)\n", offset, count);
        }
        break;
    }

    case RV_ROM_FN_GET_SYS_INFO:
        cpu->x[10] = (uint32_t)rv_rom_get_sys_info(bus, a0, a1, a2);
        break;

    case RV_ROM_FN_REBOOT:
        fprintf(stderr, "[RV-ROM] Reboot requested\n");
        cpu->x[10] = RV_BOOTROM_OK;
        cpu->is_halted = 1;
        break;

    case RV_ROM_FN_SET_STACK:
        cpu->x[10] = RV_BOOTROM_OK;
        break;

    default:
        return 0;  /* Not a recognized ROM function */
    }

    /* Return to caller: PC = ra (x1) */
    cpu->pc = cpu->x[1];
    cpu->x[0] = 0;
    cpu->step_count++;
    cpu->cycle_count++;
    cpu->instret_count++;
    return 1;
}
