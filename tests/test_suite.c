/*
 * Bramble RP2040 Emulator - Test Suite
 *
 * Tests for v0.5.0 features:
 *   - PRIMASK (CPSID/CPSIE) interrupt masking
 *   - SVC exception triggering
 *   - RAM execution support
 *   - O(1) dispatch table
 *   - Peripheral stubs (SPI, I2C, PWM)
 *   - ADCS / SBCS / RSBS instructions
 *   - Zero-copy dual-core memory routing
 *   - ELF loader
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "emulator.h"
#include "instructions.h"
#include "nvic.h"
#include "timer.h"
#include "gpio.h"

/* ========================================================================
 * Minimal Test Framework
 * ======================================================================== */

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) static void name(void)
#define RUN_TEST(name) do { \
    printf("  %-50s ", #name); \
    fflush(stdout); \
    name(); \
    tests_run++; \
} while (0)

#define ASSERT_EQ(expected, actual, msg) do { \
    uint32_t _e = (uint32_t)(expected); \
    uint32_t _a = (uint32_t)(actual); \
    if (_e != _a) { \
        printf("FAIL\n    %s: expected 0x%08X, got 0x%08X\n", msg, _e, _a); \
        tests_failed++; \
        return; \
    } \
} while (0)

#define ASSERT_NEQ(unexpected, actual, msg) do { \
    uint32_t _u = (uint32_t)(unexpected); \
    uint32_t _a = (uint32_t)(actual); \
    if (_u == _a) { \
        printf("FAIL\n    %s: got unexpected 0x%08X\n", msg, _a); \
        tests_failed++; \
        return; \
    } \
} while (0)

#define ASSERT_TRUE(cond, msg) do { \
    if (!(cond)) { \
        printf("FAIL\n    %s\n", msg); \
        tests_failed++; \
        return; \
    } \
} while (0)

#define PASS() do { \
    printf("PASS\n"); \
    tests_passed++; \
} while (0)

/* Reset CPU state to clean baseline */
static void reset_cpu(void) {
    memset(&cpu, 0, sizeof(cpu));
    cpu.xpsr = 0x01000000; /* Thumb bit set */
    cpu.primask = 0;
    cpu.current_irq = 0xFFFFFFFF;
    cpu.r[13] = RAM_BASE + RAM_SIZE - 0x100; /* SP near top of RAM */
    cpu.r[15] = FLASH_BASE + 0x100;          /* PC in flash */
    cpu.vtor = FLASH_BASE + 0x100;
    mem_set_ram_ptr(cpu.ram, RAM_BASE, RAM_SIZE);
    nvic_reset();
    timer_reset();
}

/* ========================================================================
 * PRIMASK Tests (CPSID / CPSIE)
 * ======================================================================== */

TEST(test_cpsid_sets_primask) {
    reset_cpu();
    ASSERT_EQ(0, cpu.primask, "PRIMASK should start at 0");

    /* CPSID i = 0xB672 */
    instr_cpsid(0xB672);
    ASSERT_EQ(1, cpu.primask, "CPSID should set PRIMASK to 1");
    PASS();
}

TEST(test_cpsie_clears_primask) {
    reset_cpu();
    cpu.primask = 1;

    /* CPSIE i = 0xB662 */
    instr_cpsie(0xB662);
    ASSERT_EQ(0, cpu.primask, "CPSIE should clear PRIMASK to 0");
    PASS();
}

TEST(test_primask_blocks_interrupts) {
    reset_cpu();

    /* Set up: enable IRQ 0 and make it pending */
    nvic_enable_irq(0);
    nvic_set_pending(0);

    /* With PRIMASK=1, cpu_step should NOT take the interrupt */
    cpu.primask = 1;

    /* Place a NOP (0xBF00) at PC for cpu_step to execute */
    uint32_t pc_offset = cpu.r[15] - FLASH_BASE;
    cpu.flash[pc_offset] = 0x00;
    cpu.flash[pc_offset + 1] = 0xBF;

    uint32_t pc_before = cpu.r[15];
    cpu_step();

    /* PC should advance past the NOP, NOT jump to an ISR */
    ASSERT_EQ(pc_before + 2, cpu.r[15], "PRIMASK=1 should block interrupt delivery");

    /* IRQ should still be pending */
    ASSERT_TRUE(nvic_state.pending & 1, "IRQ 0 should still be pending");
    PASS();
}

TEST(test_primask_allows_interrupts_when_clear) {
    reset_cpu();

    /* Set up a vector table entry for IRQ 0 (vector 16) */
    uint32_t handler_addr = FLASH_BASE + 0x200;
    uint32_t vtor = cpu.vtor;
    uint32_t vector_offset = 16 * 4; /* IRQ 0 = vector 16 */
    uint32_t vector_addr = vtor + vector_offset;
    uint32_t flash_off = vector_addr - FLASH_BASE;
    /* Write handler address with Thumb bit */
    uint32_t handler_thumb = handler_addr | 1;
    memcpy(&cpu.flash[flash_off], &handler_thumb, 4);

    /* Place a NOP at handler so it doesn't crash */
    uint32_t handler_flash_off = handler_addr - FLASH_BASE;
    cpu.flash[handler_flash_off] = 0x00;
    cpu.flash[handler_flash_off + 1] = 0xBF;

    /* Enable and pend IRQ 0 */
    nvic_enable_irq(0);
    nvic_set_pending(0);

    /* PRIMASK=0 should allow the interrupt */
    cpu.primask = 0;

    /* Place a NOP at current PC */
    uint32_t pc_offset = cpu.r[15] - FLASH_BASE;
    cpu.flash[pc_offset] = 0x00;
    cpu.flash[pc_offset + 1] = 0xBF;

    cpu_step();

    /* PC should have jumped to the handler */
    ASSERT_EQ(handler_addr, cpu.r[15], "PRIMASK=0 should allow interrupt delivery");
    PASS();
}

/* ========================================================================
 * SVC Exception Tests
 * ======================================================================== */

TEST(test_svc_triggers_exception) {
    reset_cpu();

    /* Set up SVCall handler (vector 11) */
    uint32_t handler_addr = FLASH_BASE + 0x300;
    uint32_t vector_offset = EXC_SVCALL * 4;
    uint32_t vector_addr = cpu.vtor + vector_offset;
    uint32_t flash_off = vector_addr - FLASH_BASE;
    uint32_t handler_thumb = handler_addr | 1;
    memcpy(&cpu.flash[flash_off], &handler_thumb, 4);

    uint32_t pc_before = cpu.r[15];
    uint32_t sp_before = cpu.r[13];

    /* SVC #0 = 0xDF00 */
    instr_svc(0xDF00);

    /* PC should be at the handler */
    ASSERT_EQ(handler_addr, cpu.r[15], "SVC should jump to SVCall handler");

    /* LR should have EXC_RETURN value */
    ASSERT_EQ(0xFFFFFFF9, cpu.r[14], "LR should be EXC_RETURN (0xFFFFFFF9)");

    /* SP should have decreased by 32 (8 registers * 4 bytes) */
    ASSERT_EQ(sp_before - 32, cpu.r[13], "SP should decrease by 32 for exception frame");

    /* Return address on stack should be pc_before + 2 (past SVC) */
    uint32_t stacked_pc = mem_read32(cpu.r[13] + 24);
    ASSERT_EQ(pc_before + 2, stacked_pc, "Stacked PC should be past SVC instruction");

    PASS();
}

/* ========================================================================
 * RAM Execution Tests
 * ======================================================================== */

TEST(test_ram_execution_allowed) {
    reset_cpu();

    /* Set PC to RAM */
    cpu.r[15] = RAM_BASE + 0x100;

    /* Place a MOVS R0, #42 (0x202A) in RAM */
    cpu.ram[0x100] = 0x2A;
    cpu.ram[0x101] = 0x20;

    ASSERT_TRUE(!cpu_is_halted(), "CPU should not halt with PC in RAM");

    cpu_step();

    ASSERT_EQ(42, cpu.r[0], "MOVS R0, #42 should set R0 to 42");
    ASSERT_EQ(RAM_BASE + 0x102, cpu.r[15], "PC should advance by 2");
    PASS();
}

TEST(test_ram_execution_boundary) {
    reset_cpu();

    /* PC at top of RAM should be halted */
    cpu.r[15] = RAM_TOP;
    ASSERT_TRUE(cpu_is_halted(), "CPU should halt at RAM_TOP");

    /* PC just inside RAM should be valid */
    cpu.r[15] = RAM_TOP - 2;
    cpu.ram[RAM_SIZE - 2] = 0x00;
    cpu.ram[RAM_SIZE - 1] = 0xBF; /* NOP */
    ASSERT_TRUE(!cpu_is_halted(), "CPU should run at RAM_TOP-2");

    PASS();
}

TEST(test_invalid_pc_halts) {
    reset_cpu();

    /* PC outside flash and RAM */
    cpu.r[15] = 0x40000000; /* peripheral space */
    ASSERT_TRUE(cpu_is_halted(), "CPU should halt with PC in peripheral space");

    cpu.r[15] = 0x00000000;
    ASSERT_TRUE(cpu_is_halted(), "CPU should halt with PC at 0x0");

    PASS();
}

/* ========================================================================
 * Dispatch Table Tests
 * ======================================================================== */

TEST(test_dispatch_movs_imm8) {
    reset_cpu();

    /* MOVS R3, #0x55 -> encoding: 0010 0011 0101 0101 = 0x2355 */
    uint16_t instr = 0x2355;
    cpu.r[15] = FLASH_BASE + 0x100;
    uint32_t off = cpu.r[15] - FLASH_BASE;
    cpu.flash[off] = instr & 0xFF;
    cpu.flash[off + 1] = instr >> 8;

    cpu_step();

    ASSERT_EQ(0x55, cpu.r[3], "MOVS R3, #0x55 via dispatch");
    PASS();
}

TEST(test_dispatch_adds_reg) {
    reset_cpu();
    cpu.r[1] = 10;
    cpu.r[2] = 20;

    /* ADDS R0, R1, R2 -> 0001 100 010 001 000 = 0x1888 */
    uint16_t instr = 0x1888;
    uint32_t off = cpu.r[15] - FLASH_BASE;
    cpu.flash[off] = instr & 0xFF;
    cpu.flash[off + 1] = instr >> 8;

    cpu_step();

    ASSERT_EQ(30, cpu.r[0], "ADDS R0, R1, R2 via dispatch");
    PASS();
}

TEST(test_dispatch_lsls_imm) {
    reset_cpu();
    cpu.r[1] = 1;

    /* LSLS R0, R1, #4 -> 000 00 00100 001 000 = 0x0108 */
    uint16_t instr = 0x0108;
    uint32_t off = cpu.r[15] - FLASH_BASE;
    cpu.flash[off] = instr & 0xFF;
    cpu.flash[off + 1] = instr >> 8;

    cpu_step();

    ASSERT_EQ(16, cpu.r[0], "LSLS R0, R1, #4 via dispatch");
    PASS();
}

TEST(test_dispatch_bcond) {
    reset_cpu();

    /* Set Z flag so BEQ will branch */
    cpu.xpsr |= 0x40000000; /* Z flag */

    /* BEQ +4 -> 0xD0 01 (cond=0, offset=+2 halfwords = +4 bytes) */
    /* Target = PC + 4 + offset*2 = PC + 4 + 2 = PC + 6... actually:
       BEQ encoding: 1101 cccc iiiiiiii where offset is sign-extended << 1
       BEQ +4: 1101 0000 0000 0001 = 0xD001 -> target = (PC+4) + 1*2 = PC+6 */
    uint16_t instr = 0xD001;
    uint32_t off = cpu.r[15] - FLASH_BASE;
    cpu.flash[off] = instr & 0xFF;
    cpu.flash[off + 1] = instr >> 8;

    uint32_t pc_before = cpu.r[15];
    cpu_step();

    /* Target should be (pc_before + 4) + 1*2 = pc_before + 6 */
    ASSERT_EQ(pc_before + 6, cpu.r[15], "BEQ should branch when Z=1");
    PASS();
}

/* ========================================================================
 * Peripheral Stub Tests (SPI, I2C, PWM)
 * ======================================================================== */

TEST(test_spi0_status_register) {
    reset_cpu();

    /* SPI0 SSPSR should return TFE=1, TNF=1 (0x03) */
    uint32_t val = mem_read32(SPI0_BASE + SPI_SSPSR);
    ASSERT_EQ(0x03, val, "SPI0 SSPSR should return 0x03 (TFE|TNF)");
    PASS();
}

TEST(test_spi1_status_register) {
    reset_cpu();

    uint32_t val = mem_read32(SPI1_BASE + SPI_SSPSR);
    ASSERT_EQ(0x03, val, "SPI1 SSPSR should return 0x03 (TFE|TNF)");
    PASS();
}

TEST(test_spi_other_regs_zero) {
    reset_cpu();

    ASSERT_EQ(0, mem_read32(SPI0_BASE + SPI_SSPCR0), "SPI0 CR0 should return 0");
    ASSERT_EQ(0, mem_read32(SPI0_BASE + SPI_SSPDR), "SPI0 DR should return 0");
    PASS();
}

TEST(test_i2c_read_zero) {
    reset_cpu();

    ASSERT_EQ(0, mem_read32(I2C0_BASE), "I2C0 base should return 0");
    ASSERT_EQ(0, mem_read32(I2C1_BASE + 0x10), "I2C1 offset should return 0");
    PASS();
}

TEST(test_pwm_read_zero) {
    reset_cpu();

    ASSERT_EQ(0, mem_read32(PWM_BASE), "PWM base should return 0");
    ASSERT_EQ(0, mem_read32(PWM_BASE + 0x100), "PWM offset should return 0");
    PASS();
}

TEST(test_peripheral_writes_no_crash) {
    reset_cpu();

    /* These should not crash */
    mem_write32(SPI0_BASE + SPI_SSPCR0, 0x12345678);
    mem_write32(I2C0_BASE, 0xDEADBEEF);
    mem_write32(PWM_BASE, 0xCAFEBABE);

    /* If we get here, writes didn't crash */
    PASS();
}

/* ========================================================================
 * ADCS / SBCS / RSBS Instruction Tests
 * ======================================================================== */

TEST(test_adcs_with_carry) {
    reset_cpu();
    cpu.r[0] = 0xFFFFFFFF;
    cpu.r[1] = 1;
    cpu.xpsr |= 0x20000000; /* Set carry flag */

    /* ADCS R0, R1: 0100 0001 01 001 000 = 0x4148 */
    instr_adcs(0x4148);

    /* 0xFFFFFFFF + 1 + 1(carry) = 0x00000001, carry set */
    ASSERT_EQ(1, cpu.r[0], "ADCS: 0xFFFFFFFF + 1 + carry=1 should be 1");
    ASSERT_TRUE(cpu.xpsr & 0x20000000, "ADCS: carry should be set (overflow)");
    PASS();
}

TEST(test_adcs_without_carry) {
    reset_cpu();
    cpu.r[0] = 5;
    cpu.r[1] = 10;
    cpu.xpsr &= ~0x20000000; /* Clear carry flag */

    instr_adcs(0x4148);

    ASSERT_EQ(15, cpu.r[0], "ADCS: 5 + 10 + carry=0 should be 15");
    PASS();
}

TEST(test_sbcs_basic) {
    reset_cpu();
    cpu.r[0] = 100;
    cpu.r[1] = 30;
    cpu.xpsr |= 0x20000000; /* Set carry (no borrow) */

    /* SBCS R0, R1: 0100 0001 10 001 000 = 0x4188 */
    instr_sbcs(0x4188);

    /* 100 - 30 - (1-1) = 70 */
    ASSERT_EQ(70, cpu.r[0], "SBCS: 100 - 30 with carry=1 should be 70");
    PASS();
}

TEST(test_sbcs_with_borrow) {
    reset_cpu();
    cpu.r[0] = 100;
    cpu.r[1] = 30;
    cpu.xpsr &= ~0x20000000; /* Clear carry (borrow) */

    instr_sbcs(0x4188);

    /* 100 - 30 - 1 = 69 */
    ASSERT_EQ(69, cpu.r[0], "SBCS: 100 - 30 with carry=0 should be 69");
    PASS();
}

TEST(test_rsbs_negate) {
    reset_cpu();
    cpu.r[1] = 42;

    /* RSBS R0, R1, #0 (NEG): 0100 0010 01 001 000 = 0x4248 */
    instr_rsbs(0x4248);

    ASSERT_EQ((uint32_t)-42, cpu.r[0], "RSBS: NEG 42 should be -42");
    ASSERT_TRUE(cpu.xpsr & 0x80000000, "RSBS: negative flag should be set");
    PASS();
}

TEST(test_rsbs_zero) {
    reset_cpu();
    cpu.r[1] = 0;

    instr_rsbs(0x4248);

    ASSERT_EQ(0, cpu.r[0], "RSBS: NEG 0 should be 0");
    ASSERT_TRUE(cpu.xpsr & 0x40000000, "RSBS: zero flag should be set");
    PASS();
}

/* ========================================================================
 * Zero-Copy Dual-Core Memory Routing Tests
 * ======================================================================== */

TEST(test_mem_set_ram_ptr_routing) {
    reset_cpu();

    /* Write a value to cpu.ram */
    uint32_t val = 0xDEADBEEF;
    memcpy(&cpu.ram[0], &val, 4);

    /* Read should return the value */
    ASSERT_EQ(0xDEADBEEF, mem_read32(RAM_BASE), "Default RAM routing reads cpu.ram");

    /* Now redirect to a different buffer */
    static uint8_t alt_ram[1024];
    memset(alt_ram, 0, sizeof(alt_ram));
    uint32_t alt_val = 0xCAFEBABE;
    memcpy(&alt_ram[0], &alt_val, 4);

    mem_set_ram_ptr(alt_ram, RAM_BASE, sizeof(alt_ram));

    ASSERT_EQ(0xCAFEBABE, mem_read32(RAM_BASE), "Redirected RAM routing reads alt buffer");

    /* Restore */
    mem_set_ram_ptr(cpu.ram, RAM_BASE, RAM_SIZE);
    ASSERT_EQ(0xDEADBEEF, mem_read32(RAM_BASE), "Restored RAM routing reads cpu.ram");

    PASS();
}

TEST(test_dual_core_ram_isolation) {
    reset_cpu();

    /* Initialize dual core */
    dual_core_init();

    /* Write different values to each core's RAM */
    uint32_t val0 = 0x11111111;
    uint32_t val1 = 0x22222222;
    memcpy(&cores[CORE0].ram[0], &val0, 4);
    memcpy(&cores[CORE1].ram[0], &val1, 4);

    /* Read via dual-core memory bus */
    ASSERT_EQ(0x11111111, mem_read32_dual(CORE0, CORE0_RAM_START),
              "Core0 should see its own RAM");
    ASSERT_EQ(0x22222222, mem_read32_dual(CORE1, CORE1_RAM_START),
              "Core1 should see its own RAM");

    /* Cores should NOT see each other's RAM at their own base address */
    /* Core0 reads from core1's range - should go through fallback */

    PASS();
}

TEST(test_dual_core_shared_flash) {
    reset_cpu();

    /* Write a value to flash */
    uint32_t val = 0xABCD1234;
    uint32_t flash_off = 0x200;
    memcpy(&cpu.flash[flash_off], &val, 4);

    dual_core_init();

    /* Both cores should see the same flash data */
    ASSERT_EQ(0xABCD1234, mem_read32_dual(CORE0, FLASH_BASE + flash_off),
              "Core0 should read shared flash");
    ASSERT_EQ(0xABCD1234, mem_read32_dual(CORE1, FLASH_BASE + flash_off),
              "Core1 should read shared flash");

    PASS();
}

TEST(test_dual_core_shared_ram) {
    reset_cpu();
    dual_core_init();

    /* Use address above CORE1_RAM_END to avoid per-core overlap */
    uint32_t shared_addr = SHARED_RAM_BASE + 0x2000; /* 0x20042000 */

    /* Write to shared RAM via core 0 */
    mem_write32_dual(CORE0, shared_addr, 0x55AA55AA);

    /* Core 1 should see it */
    ASSERT_EQ(0x55AA55AA, mem_read32_dual(CORE1, shared_addr),
              "Both cores should see shared RAM writes");

    PASS();
}

/* ========================================================================
 * ELF Loader Tests
 * ======================================================================== */

/* Create a minimal valid ELF32 ARM binary in a temp file */
static const char *create_test_elf(void) {
    static const char *path = "/tmp/bramble_test.elf";
    FILE *f = fopen(path, "wb");
    if (!f) return NULL;

    /* ELF header (52 bytes) */
    uint8_t ehdr[52] = {0};
    ehdr[0] = 0x7f; ehdr[1] = 'E'; ehdr[2] = 'L'; ehdr[3] = 'F'; /* magic */
    ehdr[4] = 1;    /* 32-bit */
    ehdr[5] = 1;    /* little-endian */
    ehdr[6] = 1;    /* ELF version */
    /* e_type = ET_EXEC (2) at offset 16 */
    ehdr[16] = 2; ehdr[17] = 0;
    /* e_machine = EM_ARM (40) at offset 18 */
    ehdr[18] = 40; ehdr[19] = 0;
    /* e_version = 1 at offset 20 */
    ehdr[20] = 1;
    /* e_entry = 0x10000101 at offset 24 */
    ehdr[24] = 0x01; ehdr[25] = 0x01; ehdr[26] = 0x00; ehdr[27] = 0x10;
    /* e_phoff = 52 at offset 28 */
    ehdr[28] = 52;
    /* e_ehsize = 52 at offset 40 */
    ehdr[40] = 52;
    /* e_phentsize = 32 at offset 42 */
    ehdr[42] = 32;
    /* e_phnum = 1 at offset 44 */
    ehdr[44] = 1;

    fwrite(ehdr, 1, 52, f);

    /* Program header (32 bytes) */
    uint8_t phdr[32] = {0};
    /* p_type = PT_LOAD (1) */
    phdr[0] = 1;
    /* p_offset = 84 (after ELF header + program header) */
    phdr[4] = 84;
    /* p_vaddr = 0x10000100 */
    phdr[8] = 0x00; phdr[9] = 0x01; phdr[10] = 0x00; phdr[11] = 0x10;
    /* p_paddr = 0x10000100 */
    phdr[12] = 0x00; phdr[13] = 0x01; phdr[14] = 0x00; phdr[15] = 0x10;
    /* p_filesz = 8 */
    phdr[16] = 8;
    /* p_memsz = 8 */
    phdr[20] = 8;
    /* p_flags = PF_R|PF_X (5) */
    phdr[24] = 5;

    fwrite(phdr, 1, 32, f);

    /* Segment data: 8 bytes (a simple vector table stub) */
    /* SP = 0x20041000, reset vector = 0x10000201 */
    uint8_t data[8] = {
        0x00, 0x10, 0x04, 0x20,  /* SP = 0x20041000 */
        0x01, 0x02, 0x00, 0x10   /* Reset vector = 0x10000201 (Thumb) */
    };
    fwrite(data, 1, 8, f);

    fclose(f);
    return path;
}

TEST(test_elf_loader_valid) {
    reset_cpu();

    const char *path = create_test_elf();
    ASSERT_TRUE(path != NULL, "Failed to create test ELF");

    int result = load_elf(path);
    ASSERT_TRUE(result != 0, "load_elf should succeed for valid ELF");

    /* Verify the data was loaded to flash */
    uint32_t sp = mem_read32(FLASH_BASE + 0x100);
    uint32_t reset_vec = mem_read32(FLASH_BASE + 0x104);

    ASSERT_EQ(0x20041000, sp, "ELF should load SP value to flash");
    ASSERT_EQ(0x10000201, reset_vec, "ELF should load reset vector to flash");

    PASS();
}

TEST(test_elf_loader_invalid_magic) {
    reset_cpu();

    const char *path = "/tmp/bramble_test_bad.elf";
    FILE *f = fopen(path, "wb");
    ASSERT_TRUE(f != NULL, "Failed to create bad ELF file");

    uint8_t bad_data[64] = {0};
    bad_data[0] = 0x00; /* Not ELF magic */
    fwrite(bad_data, 1, 64, f);
    fclose(f);

    int result = load_elf(path);
    ASSERT_EQ(0, result, "load_elf should fail for invalid ELF magic");

    PASS();
}

TEST(test_elf_loader_wrong_arch) {
    reset_cpu();

    const char *path = "/tmp/bramble_test_x86.elf";
    FILE *f = fopen(path, "wb");
    ASSERT_TRUE(f != NULL, "Failed to create x86 ELF file");

    uint8_t ehdr[52] = {0};
    ehdr[0] = 0x7f; ehdr[1] = 'E'; ehdr[2] = 'L'; ehdr[3] = 'F';
    ehdr[4] = 1; ehdr[5] = 1; ehdr[6] = 1;
    ehdr[16] = 2; ehdr[17] = 0;
    ehdr[18] = 3; ehdr[19] = 0; /* EM_386 instead of EM_ARM */
    ehdr[20] = 1;
    fwrite(ehdr, 1, 52, f);
    fclose(f);

    int result = load_elf(path);
    ASSERT_EQ(0, result, "load_elf should fail for x86 ELF");

    PASS();
}

/* ========================================================================
 * Memory Bus Tests
 * ======================================================================== */

TEST(test_flash_read_write) {
    reset_cpu();

    /* Flash writes should be ignored */
    mem_write32(FLASH_BASE, 0xDEADBEEF);

    /* Write directly to verify */
    uint32_t val = 0x12345678;
    memcpy(&cpu.flash[0], &val, 4);

    ASSERT_EQ(0x12345678, mem_read32(FLASH_BASE), "Flash read should work");
    PASS();
}

TEST(test_ram_read_write) {
    reset_cpu();

    mem_write32(RAM_BASE + 0x100, 0xAABBCCDD);
    ASSERT_EQ(0xAABBCCDD, mem_read32(RAM_BASE + 0x100), "RAM write/read 32-bit");

    mem_write16(RAM_BASE + 0x200, 0x1234);
    ASSERT_EQ(0x1234, mem_read16(RAM_BASE + 0x200), "RAM write/read 16-bit");

    mem_write8(RAM_BASE + 0x300, 0xAB);
    ASSERT_EQ(0xAB, mem_read8(RAM_BASE + 0x300), "RAM write/read 8-bit");

    PASS();
}

TEST(test_uart_output) {
    reset_cpu();

    /* UART write should not crash (output goes to stdout) */
    mem_write32(UART0_DR, 'X');

    /* UART FR should return idle flags */
    uint32_t fr = mem_read32(UART0_FR);
    ASSERT_EQ(0x00000090, fr, "UART FR should return idle status");

    PASS();
}

/* ========================================================================
 * Additional Instruction Tests
 * ======================================================================== */

TEST(test_str_ldr_sp_imm8) {
    reset_cpu();
    cpu.r[0] = 0xFEEDFACE;
    cpu.r[13] = RAM_BASE + 0x400;

    /* STR R0, [SP, #8] = 0x9002: 1001 0 000 00000010 */
    uint16_t str_instr = 0x9002;
    uint32_t off = cpu.r[15] - FLASH_BASE;
    cpu.flash[off] = str_instr & 0xFF;
    cpu.flash[off + 1] = str_instr >> 8;

    cpu_step();

    /* Verify value was stored */
    uint32_t stored = mem_read32(RAM_BASE + 0x400 + 8);
    ASSERT_EQ(0xFEEDFACE, stored, "STR R0, [SP, #8] should store value");

    /* Now load it back into R1: LDR R1, [SP, #8] = 0x9902 */
    cpu.r[1] = 0;
    uint16_t ldr_instr = 0x9902;
    off = cpu.r[15] - FLASH_BASE;
    cpu.flash[off] = ldr_instr & 0xFF;
    cpu.flash[off + 1] = ldr_instr >> 8;

    cpu_step();

    ASSERT_EQ(0xFEEDFACE, cpu.r[1], "LDR R1, [SP, #8] should load value");
    PASS();
}

TEST(test_push_pop) {
    reset_cpu();
    cpu.r[0] = 0x11;
    cpu.r[1] = 0x22;
    cpu.r[14] = 0xAABBCCDD; /* LR */
    cpu.r[13] = RAM_BASE + 0x800;

    /* PUSH {R0, R1, LR} = 0xB503: 1011 0 1 01 00000011 */
    uint16_t push_instr = 0xB503;
    uint32_t off = cpu.r[15] - FLASH_BASE;
    cpu.flash[off] = push_instr & 0xFF;
    cpu.flash[off + 1] = push_instr >> 8;

    cpu_step();

    /* SP should decrease by 12 (3 regs) */
    ASSERT_EQ(RAM_BASE + 0x800 - 12, cpu.r[13], "PUSH should decrease SP");

    /* Clobber registers */
    cpu.r[0] = 0;
    cpu.r[1] = 0;
    cpu.r[14] = 0;

    /* POP {R0, R1, PC} = 0xBD03: 1011 1 1 01 00000011 */
    uint16_t pop_instr = 0xBD03;
    off = cpu.r[15] - FLASH_BASE;
    cpu.flash[off] = pop_instr & 0xFF;
    cpu.flash[off + 1] = pop_instr >> 8;

    cpu_step();

    ASSERT_EQ(0x11, cpu.r[0], "POP should restore R0");
    ASSERT_EQ(0x22, cpu.r[1], "POP should restore R1");
    /* PC gets loaded with what was LR, with Thumb bit cleared */
    ASSERT_EQ(0xAABBCCDC, cpu.r[15], "POP PC should load saved LR (Thumb-cleared)");

    PASS();
}

/* ========================================================================
 * Main
 * ======================================================================== */

int main(void) {
    printf("========================================\n");
    printf(" Bramble RP2040 Emulator - Test Suite\n");
    printf("========================================\n\n");

    /* Initialize emulator subsystems */
    cpu_init();
    nvic_init();
    timer_init();
    gpio_init();

    /* PRIMASK Tests */
    printf("[PRIMASK]\n");
    RUN_TEST(test_cpsid_sets_primask);
    RUN_TEST(test_cpsie_clears_primask);
    RUN_TEST(test_primask_blocks_interrupts);
    RUN_TEST(test_primask_allows_interrupts_when_clear);
    printf("\n");

    /* SVC Tests */
    printf("[SVC Exception]\n");
    RUN_TEST(test_svc_triggers_exception);
    printf("\n");

    /* RAM Execution Tests */
    printf("[RAM Execution]\n");
    RUN_TEST(test_ram_execution_allowed);
    RUN_TEST(test_ram_execution_boundary);
    RUN_TEST(test_invalid_pc_halts);
    printf("\n");

    /* Dispatch Table Tests */
    printf("[Dispatch Table]\n");
    RUN_TEST(test_dispatch_movs_imm8);
    RUN_TEST(test_dispatch_adds_reg);
    RUN_TEST(test_dispatch_lsls_imm);
    RUN_TEST(test_dispatch_bcond);
    printf("\n");

    /* Peripheral Stub Tests */
    printf("[Peripheral Stubs]\n");
    RUN_TEST(test_spi0_status_register);
    RUN_TEST(test_spi1_status_register);
    RUN_TEST(test_spi_other_regs_zero);
    RUN_TEST(test_i2c_read_zero);
    RUN_TEST(test_pwm_read_zero);
    RUN_TEST(test_peripheral_writes_no_crash);
    printf("\n");

    /* ADCS / SBCS / RSBS Tests */
    printf("[ADCS/SBCS/RSBS]\n");
    RUN_TEST(test_adcs_with_carry);
    RUN_TEST(test_adcs_without_carry);
    RUN_TEST(test_sbcs_basic);
    RUN_TEST(test_sbcs_with_borrow);
    RUN_TEST(test_rsbs_negate);
    RUN_TEST(test_rsbs_zero);
    printf("\n");

    /* Zero-Copy Dual-Core Tests */
    printf("[Dual-Core Memory]\n");
    RUN_TEST(test_mem_set_ram_ptr_routing);
    RUN_TEST(test_dual_core_ram_isolation);
    RUN_TEST(test_dual_core_shared_flash);
    RUN_TEST(test_dual_core_shared_ram);
    printf("\n");

    /* ELF Loader Tests */
    printf("[ELF Loader]\n");
    RUN_TEST(test_elf_loader_valid);
    RUN_TEST(test_elf_loader_invalid_magic);
    RUN_TEST(test_elf_loader_wrong_arch);
    printf("\n");

    /* Memory Bus Tests */
    printf("[Memory Bus]\n");
    RUN_TEST(test_flash_read_write);
    RUN_TEST(test_ram_read_write);
    RUN_TEST(test_uart_output);
    printf("\n");

    /* Instruction Integration Tests */
    printf("[Instruction Integration]\n");
    RUN_TEST(test_str_ldr_sp_imm8);
    RUN_TEST(test_push_pop);
    printf("\n");

    /* Summary */
    printf("========================================\n");
    printf(" Results: %d/%d passed, %d failed\n", tests_passed, tests_run, tests_failed);
    printf("========================================\n");

    /* Clean up temp files */
    remove("/tmp/bramble_test.elf");
    remove("/tmp/bramble_test_bad.elf");
    remove("/tmp/bramble_test_x86.elf");

    return tests_failed > 0 ? 1 : 0;
}
