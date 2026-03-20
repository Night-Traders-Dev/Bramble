/*
 * RP2350 RISC-V Bootrom
 *
 * Generates a minimal RISC-V bootrom image in the ROM buffer.
 * The bootrom:
 *   1. Sets SP to top of SRAM (x2 = SRAM_END)
 *   2. Sets GP to a conventional value (x3 = SRAM_BASE)
 *   3. Jumps to flash entry point (0x10000000)
 *
 * The ROM also contains a trap handler that halts the core on
 * unexpected traps, and stub return addresses for ROM function calls.
 *
 * Layout:
 *   0x0000: Reset vector (jump to boot code)
 *   0x0004: Trap vector (loops forever / halts)
 *   0x0010: Boot code (set SP, jump to flash)
 *   0x0100: ROM function stubs (JAL x0,0 = halt)
 */

#include <string.h>
#include <stdio.h>
#include "rp2350_rv/rv_bootrom.h"
#include "rp2350_rv/rp2350_memmap.h"

/* Helper: write a 32-bit little-endian word to ROM buffer */
static void rom_write32(uint8_t *rom, uint32_t offset, uint32_t val) {
    rom[offset + 0] = (uint8_t)(val);
    rom[offset + 1] = (uint8_t)(val >> 8);
    rom[offset + 2] = (uint8_t)(val >> 16);
    rom[offset + 3] = (uint8_t)(val >> 24);
}

/* Encode RISC-V instructions */

/* JAL rd, offset — J-type encoding */
static uint32_t rv_jal(uint32_t rd, int32_t offset) {
    uint32_t imm = (uint32_t)offset;
    uint32_t enc = (rd << 7) | 0x6F;  /* opcode = JAL */
    enc |= ((imm >> 12) & 0xFF) << 12;   /* imm[19:12] */
    enc |= ((imm >> 11) & 1) << 20;       /* imm[11] */
    enc |= ((imm >> 1) & 0x3FF) << 21;    /* imm[10:1] */
    enc |= ((imm >> 20) & 1) << 31;       /* imm[20] */
    return enc;
}

/* LUI rd, imm — U-type encoding */
static uint32_t rv_lui(uint32_t rd, uint32_t imm_upper) {
    return (imm_upper & 0xFFFFF000) | (rd << 7) | 0x37;
}

/* ADDI rd, rs1, imm — I-type encoding */
static uint32_t rv_addi(uint32_t rd, uint32_t rs1, int32_t imm) {
    return ((uint32_t)imm << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x13;
}

/* JALR rd, rs1, 0 — jump to address in rs1 */
static uint32_t rv_jalr(uint32_t rd, uint32_t rs1, int32_t imm) {
    return ((uint32_t)imm << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x67;
}

/* EBREAK — halts execution */
static uint32_t rv_ebreak(void) {
    return 0x00100073;
}

/* C.J offset=0  (infinite loop, encoded as 16-bit compressed) */
static uint16_t rv_c_j_self(void) {
    /* C.J with offset 0: 101 00000000 01 = 0xA001 */
    return 0xA001;
}

uint32_t rv_bootrom_init(uint8_t *rom, uint32_t rom_size,
                         uint32_t flash_base, uint32_t sram_end) {
    memset(rom, 0, rom_size);

    /*
     * 0x0000: JAL x0, 0x10 (jump to boot code at offset 0x10)
     *         offset from 0x0000 to 0x0010 = +16
     */
    rom_write32(rom, 0x0000, rv_jal(0, 0x10));

    /*
     * 0x0004: Trap handler — C.J self (infinite loop)
     * This is where mtvec points by default. If firmware doesn't set
     * mtvec, unhandled traps loop here.
     */
    rom[0x0004] = (uint8_t)(rv_c_j_self());
    rom[0x0005] = (uint8_t)(rv_c_j_self() >> 8);

    /*
     * 0x0010: Boot code
     *   LUI x2, upper(sram_end)     ; SP = top of SRAM
     *   ADDI x2, x2, lower(sram_end)
     *   LUI x5, upper(flash_base)   ; t0 = flash entry
     *   JALR x0, x5, 0              ; jump to flash
     */
    uint32_t sp_upper = sram_end & 0xFFFFF000;
    int32_t sp_lower = (int32_t)(sram_end & 0xFFF);
    /* If lower bits have sign bit set, adjust upper */
    if (sp_lower & 0x800) {
        sp_upper += 0x1000;
        sp_lower = sp_lower - 0x1000;
    }

    uint32_t pc = 0x10;
    rom_write32(rom, pc, rv_lui(2, sp_upper));        pc += 4;  /* LUI x2 (SP) */
    if (sp_lower != 0) {
        rom_write32(rom, pc, rv_addi(2, 2, sp_lower)); pc += 4; /* ADDI x2, x2, lower */
    }

    /* Set GP (x3) to SRAM base (conventional for RV toolchains) */
    rom_write32(rom, pc, rv_lui(3, RP2350_SRAM_BASE)); pc += 4;

    /* Load flash base into t0 and jump */
    rom_write32(rom, pc, rv_lui(5, flash_base));        pc += 4;  /* LUI t0, flash_base */
    rom_write32(rom, pc, rv_jalr(0, 5, 0));             pc += 4;  /* JALR x0, t0, 0 */

    /*
     * 0x0100+: ROM function stubs
     * Simple EBREAK for any ROM function call — firmware shouldn't be
     * calling bootrom functions directly in most Pico 2 use cases,
     * but we provide stubs to avoid crashes.
     */
    for (uint32_t i = 0x100; i < 0x200 && i + 4 <= rom_size; i += 4) {
        rom_write32(rom, i, rv_ebreak());
    }

    fprintf(stderr, "[RV-BOOT] ROM initialized: SP=0x%08X, entry=0x%08X\n",
            sram_end, flash_base);

    return 0x00000000;  /* CPU starts at ROM address 0 */
}
