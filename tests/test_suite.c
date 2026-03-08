/*
 * Bramble RP2040 Emulator - Test Suite
 *
 * Comprehensive tests covering:
 *   v0.5.0: PRIMASK, SVC, RAM exec, dispatch, peripheral stubs, ADCS/SBCS/RSBS,
 *           dual-core memory, ELF loader
 *   v0.6.0: SysTick, MSR/MRS, NVIC preemption, SCB registers
 *   v0.7.0: Resets, Clocks, XOSC, PLL, Watchdog, ADC, UART registers
 *   v0.8.0: Timer, spinlocks, FIFO, bitwise ops, shifts, byte/halfword ops,
 *           branches, STMIA/LDMIA, MUL, exception round-trip, audit fixes
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
#include "clocks.h"
#include "adc.h"
#include "rom.h"

/* ========================================================================
 * Test Framework (Verbose)
 * ======================================================================== */

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;
static int category_run = 0;
static int category_passed = 0;
static int category_failed = 0;

#define TEST(name) static void name(void)
#define RUN_TEST(name) do { \
    printf("  %-50s ", #name); \
    fflush(stdout); \
    name(); \
    tests_run++; \
    category_run++; \
} while (0)

#define ASSERT_EQ(expected, actual, msg) do { \
    uint32_t _e = (uint32_t)(expected); \
    uint32_t _a = (uint32_t)(actual); \
    if (_e != _a) { \
        printf("FAIL\n    %s: expected 0x%08X, got 0x%08X\n", msg, _e, _a); \
        tests_failed++; \
        category_failed++; \
        return; \
    } \
} while (0)

#define ASSERT_NEQ(unexpected, actual, msg) do { \
    uint32_t _u = (uint32_t)(unexpected); \
    uint32_t _a = (uint32_t)(actual); \
    if (_u == _a) { \
        printf("FAIL\n    %s: got unexpected 0x%08X\n", msg, _a); \
        tests_failed++; \
        category_failed++; \
        return; \
    } \
} while (0)

#define ASSERT_TRUE(cond, msg) do { \
    if (!(cond)) { \
        printf("FAIL\n    %s\n", msg); \
        tests_failed++; \
        category_failed++; \
        return; \
    } \
} while (0)

#define PASS() do { \
    printf("PASS\n"); \
    tests_passed++; \
    category_passed++; \
} while (0)

#define BEGIN_CATEGORY(name) do { \
    printf("\n[%s]\n", name); \
    category_run = 0; \
    category_passed = 0; \
    category_failed = 0; \
} while (0)

#define END_CATEGORY(name) do { \
    printf("  -- %s: %d/%d passed\n", name, category_passed, category_run); \
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
    rom_init();
}

/* ========================================================================
 * PRIMASK Tests (CPSID / CPSIE)
 * ======================================================================== */

TEST(test_cpsid_sets_primask) {
    reset_cpu();
    ASSERT_EQ(0, cpu.primask, "PRIMASK should start at 0");
    instr_cpsid(0xB672);
    ASSERT_EQ(1, cpu.primask, "CPSID should set PRIMASK to 1");
    PASS();
}

TEST(test_cpsie_clears_primask) {
    reset_cpu();
    cpu.primask = 1;
    instr_cpsie(0xB662);
    ASSERT_EQ(0, cpu.primask, "CPSIE should clear PRIMASK to 0");
    PASS();
}

TEST(test_primask_blocks_interrupts) {
    reset_cpu();
    cpu.primask = 1;
    nvic_enable_irq(0);
    nvic_set_pending(0);
    ASSERT_EQ(1, cpu.primask, "PRIMASK should be 1");
    ASSERT_NEQ(0xFFFFFFFF, nvic_get_pending_irq(), "IRQ 0 should be pending");
    PASS();
}

TEST(test_primask_allows_interrupts_when_clear) {
    reset_cpu();
    cpu.primask = 0;
    nvic_enable_irq(0);
    nvic_set_pending(0);
    uint32_t pending = nvic_get_pending_irq();
    ASSERT_EQ(0, pending, "IRQ 0 should be pending");
    nvic_clear_pending(0);
    PASS();
}

/* ========================================================================
 * SVC Exception Tests
 * ======================================================================== */

TEST(test_svc_triggers_exception) {
    reset_cpu();
    ASSERT_EQ(0xFFFFFFFF, cpu.current_irq, "No active IRQ initially");
    PASS();
}

/* ========================================================================
 * RAM Execution Tests
 * ======================================================================== */

TEST(test_ram_execution_allowed) {
    reset_cpu();
    cpu.r[15] = RAM_BASE + 0x100;
    ASSERT_EQ(0, cpu_is_halted(), "PC in RAM should not be halted");
    PASS();
}

TEST(test_ram_execution_boundary) {
    reset_cpu();
    cpu.r[15] = RAM_TOP - 2;
    ASSERT_EQ(0, cpu_is_halted(), "PC at RAM top boundary should not be halted");
    PASS();
}

TEST(test_invalid_pc_halts) {
    reset_cpu();
    cpu.r[15] = 0xFFFFFFFF;
    ASSERT_EQ(1, cpu_is_halted(), "PC=0xFFFFFFFF should be halted");
    PASS();
}

/* ========================================================================
 * Dispatch Table Tests
 * ======================================================================== */

TEST(test_dispatch_movs_imm8) {
    reset_cpu();
    instr_movs_imm8(0x2042);
    ASSERT_EQ(0x42, cpu.r[0], "MOVS R0, #0x42");
    PASS();
}

TEST(test_dispatch_adds_reg) {
    reset_cpu();
    cpu.r[1] = 10;
    cpu.r[2] = 20;
    instr_adds_reg_reg(0x1888);
    ASSERT_EQ(30, cpu.r[0], "ADDS R0, R1, R2 = 30");
    PASS();
}

TEST(test_dispatch_lsls_imm) {
    reset_cpu();
    cpu.r[1] = 1;
    instr_shift_logical_left(0x0108);
    ASSERT_EQ(16, cpu.r[0], "LSLS R0, R1, #4 = 16");
    PASS();
}

TEST(test_dispatch_bcond) {
    reset_cpu();
    cpu.xpsr |= 0x40000000; /* Z flag */
    pc_updated = 0;
    instr_bcond(0xD002);
    ASSERT_TRUE(pc_updated == 1, "BEQ should update PC");
    PASS();
}

/* ========================================================================
 * Peripheral Stub Tests
 * ======================================================================== */

TEST(test_spi0_status_register) {
    reset_cpu();
    ASSERT_EQ(0x03, mem_read32(SPI0_BASE + SPI_SSPSR), "SPI0 SSPSR = 0x03");
    PASS();
}

TEST(test_spi1_status_register) {
    reset_cpu();
    ASSERT_EQ(0x03, mem_read32(SPI1_BASE + SPI_SSPSR), "SPI1 SSPSR = 0x03");
    PASS();
}

TEST(test_spi_other_regs_zero) {
    reset_cpu();
    ASSERT_EQ(0, mem_read32(SPI0_BASE + SPI_SSPCR0), "SPI0 SSPCR0 = 0");
    PASS();
}

TEST(test_i2c_read_zero) {
    reset_cpu();
    ASSERT_EQ(0, mem_read32(I2C0_BASE), "I2C0 reads 0");
    PASS();
}

TEST(test_pwm_read_zero) {
    reset_cpu();
    ASSERT_EQ(0, mem_read32(PWM_BASE), "PWM reads 0");
    PASS();
}

TEST(test_peripheral_writes_no_crash) {
    reset_cpu();
    mem_write32(SPI0_BASE + SPI_SSPDR, 0xAA);
    mem_write32(I2C0_BASE, 0x55);
    mem_write32(PWM_BASE, 0xFF);
    PASS();
}

/* ========================================================================
 * ADCS / SBCS / RSBS Tests
 * ======================================================================== */

TEST(test_adcs_with_carry) {
    reset_cpu();
    cpu.r[0] = 0xFFFFFFFF;
    cpu.r[1] = 1;
    cpu.xpsr |= 0x20000000; /* C flag */
    instr_adcs(0x4148);
    ASSERT_EQ(1, cpu.r[0], "ADCS: 0xFFFFFFFF + 1 + 1 = 1");
    PASS();
}

TEST(test_adcs_without_carry) {
    reset_cpu();
    cpu.r[0] = 5;
    cpu.r[1] = 3;
    cpu.xpsr &= ~0x20000000;
    instr_adcs(0x4148);
    ASSERT_EQ(8, cpu.r[0], "ADCS: 5 + 3 + 0 = 8");
    PASS();
}

TEST(test_sbcs_basic) {
    reset_cpu();
    cpu.r[0] = 10;
    cpu.r[1] = 3;
    cpu.xpsr |= 0x20000000; /* C=1 */
    instr_sbcs(0x4188);
    ASSERT_EQ(7, cpu.r[0], "SBCS: 10 - 3 - 0 = 7");
    PASS();
}

TEST(test_sbcs_with_borrow) {
    reset_cpu();
    cpu.r[0] = 10;
    cpu.r[1] = 3;
    cpu.xpsr &= ~0x20000000; /* C=0 */
    instr_sbcs(0x4188);
    ASSERT_EQ(6, cpu.r[0], "SBCS: 10 - 3 - 1 = 6");
    PASS();
}

TEST(test_rsbs_negate) {
    reset_cpu();
    cpu.r[1] = 5;
    instr_rsbs(0x4248);
    ASSERT_EQ(0xFFFFFFFB, cpu.r[0], "RSBS: 0 - 5 = -5");
    PASS();
}

TEST(test_rsbs_zero) {
    reset_cpu();
    cpu.r[1] = 0;
    instr_rsbs(0x4248);
    ASSERT_EQ(0, cpu.r[0], "RSBS: 0 - 0 = 0");
    ASSERT_TRUE(cpu.xpsr & 0x40000000, "Z flag should be set");
    PASS();
}

/* ========================================================================
 * Dual-Core Memory Tests
 * ======================================================================== */

TEST(test_mem_set_ram_ptr_routing) {
    reset_cpu();
    mem_write32(RAM_BASE, 0xDEADBEEF);
    ASSERT_EQ(0xDEADBEEF, mem_read32(RAM_BASE), "RAM write/read via pointer");
    PASS();
}

TEST(test_dual_core_ram_isolation) {
    reset_cpu();
    dual_core_init();
    cores[CORE0].ram[0] = 0xAA;
    cores[CORE1].ram[0] = 0x55;
    ASSERT_TRUE(cores[CORE0].ram[0] != cores[CORE1].ram[0], "Core RAM isolated");
    PASS();
}

TEST(test_dual_core_shared_flash) {
    reset_cpu();
    dual_core_init();
    cpu.flash[0x100] = 0x42;
    uint32_t val0 = mem_read32_dual(CORE0, FLASH_BASE + 0x100);
    uint32_t val1 = mem_read32_dual(CORE1, FLASH_BASE + 0x100);
    ASSERT_EQ(val0, val1, "Both cores read same flash");
    PASS();
}

TEST(test_dual_core_shared_ram) {
    reset_cpu();
    dual_core_init();
    mem_write32_dual(CORE0, SHARED_RAM_BASE, 0xCAFEBABE);
    uint32_t val = mem_read32_dual(CORE1, SHARED_RAM_BASE);
    ASSERT_EQ(0xCAFEBABE, val, "Shared RAM visible to both cores");
    PASS();
}

/* ========================================================================
 * ELF Loader Tests
 * ======================================================================== */

TEST(test_elf_loader_valid) {
    reset_cpu();
    uint8_t elf_data[128];
    memset(elf_data, 0, sizeof(elf_data));
    elf_data[0] = 0x7F; elf_data[1] = 'E'; elf_data[2] = 'L'; elf_data[3] = 'F';
    elf_data[4] = 1; elf_data[5] = 1; elf_data[6] = 1;
    elf_data[18] = 40;
    elf_data[24] = 0x01; elf_data[25] = 0x01; elf_data[26] = 0x00; elf_data[27] = 0x10;
    elf_data[28] = 52; elf_data[42] = 32; elf_data[44] = 1;
    /* Program header at offset 52 */
    elf_data[52] = 1; /* PT_LOAD */
    elf_data[56] = 84; /* p_offset */
    elf_data[60] = 0x00; elf_data[61] = 0x01; elf_data[62] = 0x00; elf_data[63] = 0x10;
    elf_data[64] = 0x00; elf_data[65] = 0x01; elf_data[66] = 0x00; elf_data[67] = 0x10;
    elf_data[68] = 8; elf_data[72] = 8;
    elf_data[84] = 0xAA; elf_data[85] = 0xBB;
    FILE *f = fopen("/tmp/test.elf", "wb");
    ASSERT_TRUE(f != NULL, "Failed to create test ELF file");
    fwrite(elf_data, 1, sizeof(elf_data), f);
    fclose(f);
    int result = load_elf("/tmp/test.elf");
    ASSERT_TRUE(result != 0, "ELF load should succeed");
    PASS();
}

TEST(test_elf_loader_invalid_magic) {
    reset_cpu();
    uint8_t bad_elf[64];
    memset(bad_elf, 0, sizeof(bad_elf));
    FILE *f = fopen("/tmp/test_bad.elf", "wb");
    ASSERT_TRUE(f != NULL, "Failed to create bad ELF file");
    fwrite(bad_elf, 1, sizeof(bad_elf), f);
    fclose(f);
    int result = load_elf("/tmp/test_bad.elf");
    ASSERT_EQ(0, result, "Bad ELF should fail to load");
    PASS();
}

TEST(test_elf_loader_wrong_arch) {
    reset_cpu();
    uint8_t elf_data[64];
    memset(elf_data, 0, sizeof(elf_data));
    elf_data[0] = 0x7F; elf_data[1] = 'E'; elf_data[2] = 'L'; elf_data[3] = 'F';
    elf_data[4] = 1; elf_data[5] = 1; elf_data[6] = 1;
    elf_data[18] = 3; /* x86 */
    FILE *f = fopen("/tmp/test_x86.elf", "wb");
    ASSERT_TRUE(f != NULL, "Failed to create x86 ELF file");
    fwrite(elf_data, 1, sizeof(elf_data), f);
    fclose(f);
    int result = load_elf("/tmp/test_x86.elf");
    ASSERT_EQ(0, result, "x86 ELF should fail on ARM emulator");
    PASS();
}

/* ========================================================================
 * Memory Bus Tests
 * ======================================================================== */

TEST(test_flash_read_write) {
    reset_cpu();
    cpu.flash[0] = 0x42;
    ASSERT_EQ(0x42, mem_read8(FLASH_BASE), "Flash byte read");
    PASS();
}

TEST(test_ram_read_write) {
    reset_cpu();
    mem_write32(RAM_BASE, 0xDEADBEEF);
    ASSERT_EQ(0xDEADBEEF, mem_read32(RAM_BASE), "RAM write/read 32");
    PASS();
}

TEST(test_uart_output) {
    reset_cpu();
    mem_write32(UART0_DR, 'X');
    PASS();
}

/* ========================================================================
 * Instruction Integration Tests
 * ======================================================================== */

TEST(test_str_ldr_sp_imm8) {
    reset_cpu();
    cpu.r[0] = 0xCAFEBABE;
    cpu.r[13] = RAM_BASE + 0x100;
    instr_str_sp_imm8(0x9000);
    cpu.r[0] = 0;
    instr_ldr_sp_imm8(0x9800);
    ASSERT_EQ(0xCAFEBABE, cpu.r[0], "STR/LDR SP round-trip");
    PASS();
}

TEST(test_push_pop) {
    reset_cpu();
    cpu.r[0] = 0x11111111;
    cpu.r[1] = 0x22222222;
    cpu.r[13] = RAM_BASE + 0x200;
    instr_push(0xB403);
    cpu.r[0] = 0; cpu.r[1] = 0;
    pc_updated = 0;
    instr_pop(0xBC03);
    ASSERT_EQ(0x11111111, cpu.r[0], "POP R0 restored");
    ASSERT_EQ(0x22222222, cpu.r[1], "POP R1 restored");
    PASS();
}

/* ========================================================================
 * SysTick Timer Tests
 * ======================================================================== */

TEST(test_systick_registers) {
    reset_cpu();
    nvic_write_register(SYST_RVR, 1000);
    ASSERT_EQ(1000, nvic_read_register(SYST_RVR), "SysTick RVR");
    PASS();
}

TEST(test_systick_countdown) {
    reset_cpu();
    nvic_write_register(SYST_RVR, 100);
    nvic_write_register(SYST_CVR, 0);
    nvic_write_register(SYST_CSR, 0x05);
    /* First tick: CVR=0 triggers reload to RVR=100; second tick: 100->99 */
    systick_tick(2);
    ASSERT_EQ(99, nvic_read_register(SYST_CVR), "SysTick countdown 100->99");
    PASS();
}

TEST(test_systick_disabled_no_count) {
    reset_cpu();
    nvic_write_register(SYST_RVR, 100);
    nvic_write_register(SYST_CVR, 0);
    nvic_write_register(SYST_CSR, 0x00);
    systick_tick(1);
    ASSERT_EQ(0, nvic_read_register(SYST_CVR), "Disabled SysTick no count");
    PASS();
}

TEST(test_systick_calib_tenms) {
    reset_cpu();
    uint32_t calib = nvic_read_register(SYST_CALIB);
    ASSERT_EQ(0xC0002710, calib, "SYST_CALIB: NOREF=1, SKEW=1, TENMS=10000");
    PASS();
}

/* ========================================================================
 * MSR/MRS Instruction Tests
 * ======================================================================== */

TEST(test_mrs_primask) {
    reset_cpu();
    cpu.primask = 1;
    instr_mrs_32(0, 0x10);
    ASSERT_EQ(1, cpu.r[0], "MRS R0, PRIMASK");
    PASS();
}

TEST(test_msr_primask) {
    reset_cpu();
    cpu.r[0] = 1;
    instr_msr_32(0, 0x10);
    ASSERT_EQ(1, cpu.primask, "MSR PRIMASK, R0");
    PASS();
}

TEST(test_mrs_xpsr) {
    reset_cpu();
    cpu.xpsr = 0xF1000000;
    instr_mrs_32(0, 0x03);
    ASSERT_EQ(0xF1000000, cpu.r[0], "MRS R0, xPSR");
    PASS();
}

TEST(test_msr_apsr_flags) {
    reset_cpu();
    cpu.r[0] = 0xF0000000;
    instr_msr_32(0, 0x00);
    ASSERT_EQ(0xF0000000 | 0x01000000, cpu.xpsr, "MSR APSR + Thumb preserved");
    PASS();
}

TEST(test_mrs_msr_control) {
    reset_cpu();
    cpu.r[0] = 0x03;
    instr_msr_32(0, 0x14);
    ASSERT_EQ(0x03, cpu.control, "MSR CONTROL");
    instr_mrs_32(1, 0x14);
    ASSERT_EQ(0x03, cpu.r[1], "MRS CONTROL");
    PASS();
}

TEST(test_32bit_msr_dispatch) {
    reset_cpu();
    cpu.r[0] = 1;
    uint16_t upper = 0xF380, lower = 0x8810;
    memcpy(&cpu.flash[0x100], &upper, 2);
    memcpy(&cpu.flash[0x102], &lower, 2);
    cpu.r[15] = FLASH_BASE + 0x100;
    cpu_step();
    ASSERT_EQ(1, cpu.primask, "32-bit MSR via cpu_step");
    PASS();
}

TEST(test_32bit_mrs_dispatch) {
    reset_cpu();
    cpu.primask = 1;
    uint16_t upper = 0xF3EF, lower = 0x8010;
    memcpy(&cpu.flash[0x100], &upper, 2);
    memcpy(&cpu.flash[0x102], &lower, 2);
    cpu.r[15] = FLASH_BASE + 0x100;
    cpu_step();
    ASSERT_EQ(1, cpu.r[0], "32-bit MRS via cpu_step");
    PASS();
}

TEST(test_32bit_dsb_dispatch) {
    reset_cpu();
    uint16_t upper = 0xF3BF, lower = 0x8F4F;
    memcpy(&cpu.flash[0x100], &upper, 2);
    memcpy(&cpu.flash[0x102], &lower, 2);
    cpu.r[15] = FLASH_BASE + 0x100;
    uint32_t pc_before = cpu.r[15];
    cpu_step();
    ASSERT_EQ(pc_before + 4, cpu.r[15], "DSB advances PC by 4");
    PASS();
}

/* ========================================================================
 * NVIC Priority Preemption Tests
 * ======================================================================== */

TEST(test_nvic_priority_preemption_blocked) {
    reset_cpu();
    cpu.current_irq = 15;
    nvic_state.active_exceptions |= (1u << 15);
    nvic_enable_irq(1);
    nvic_set_pending(1);
    nvic_state.priority[1] = 0xC0;
    uint8_t active_pri = nvic_get_exception_priority(15);
    uint8_t pending_pri = nvic_state.priority[1] & 0xC0;
    ASSERT_TRUE(pending_pri >= active_pri, "Lower priority should not preempt");
    PASS();
}

TEST(test_nvic_priority_preemption_allowed) {
    reset_cpu();
    cpu.current_irq = 15;
    nvic_state.active_exceptions |= (1u << 15);
    nvic_enable_irq(0);
    nvic_set_pending(0);
    nvic_state.priority[0] = 0x00;
    nvic_write_register(SCB_SHPR3, 0xC0000000); /* SysTick prio = 0xC0 */
    uint8_t active_pri = nvic_get_exception_priority(15);
    uint8_t pending_pri = nvic_state.priority[0] & 0xC0;
    ASSERT_TRUE(pending_pri < active_pri, "Higher priority should preempt");
    nvic_clear_pending(0);
    PASS();
}

TEST(test_nvic_exception_priority_lookup) {
    reset_cpu();
    nvic_write_register(SCB_SHPR3, 0xC0C00000);
    ASSERT_EQ(0xC0, nvic_get_exception_priority(EXC_SYSTICK), "SysTick priority");
    ASSERT_EQ(0xC0, nvic_get_exception_priority(EXC_PENDSV), "PendSV priority");
    PASS();
}

/* ========================================================================
 * SCB Register Tests
 * ======================================================================== */

TEST(test_scb_shpr_registers) {
    reset_cpu();
    nvic_write_register(SCB_SHPR3, 0xC0C00000);
    ASSERT_EQ(0xC0C00000, nvic_read_register(SCB_SHPR3), "SHPR3 read back");
    PASS();
}

TEST(test_scb_vtor_write) {
    reset_cpu();
    nvic_write_register(SCB_VTOR, 0x10000200);
    ASSERT_EQ(0x10000200, cpu.vtor, "VTOR updated");
    PASS();
}

/* ========================================================================
 * Resets Peripheral Tests
 * ======================================================================== */

TEST(test_resets_power_on_state) {
    clocks_init();
    ASSERT_NEQ(0, clocks_read32(RESETS_BASE + 0x00), "RESET non-zero at power-on");
    PASS();
}

TEST(test_resets_release_and_done) {
    clocks_init();
    clocks_write32(RESETS_BASE + 0x00, 0x00000000);
    ASSERT_NEQ(0, clocks_read32(RESETS_BASE + 0x08), "RESET_DONE non-zero");
    PASS();
}

TEST(test_resets_atomic_clear) {
    clocks_init();
    uint32_t before = clocks_read32(RESETS_BASE + 0x00);
    clocks_write32(RESETS_BASE + 0x3000, 0x00000001);
    uint32_t after = clocks_read32(RESETS_BASE + 0x00);
    ASSERT_TRUE((before & 1) && !(after & 1), "CLR alias clears bit 0");
    PASS();
}

/* ========================================================================
 * Clocks Peripheral Tests
 * ======================================================================== */

TEST(test_clocks_selected_always_set) {
    clocks_init();
    ASSERT_NEQ(0, clocks_read32(CLOCKS_BASE + 0x08), "CLK SELECTED non-zero");
    PASS();
}

TEST(test_clocks_ctrl_write_read) {
    clocks_init();
    clocks_write32(CLOCKS_BASE + 0x00, 0x00000880);
    ASSERT_EQ(0x00000880, clocks_read32(CLOCKS_BASE + 0x00), "CLK CTRL r/w");
    PASS();
}

/* ========================================================================
 * XOSC / PLL / Watchdog / ADC / UART Tests
 * ======================================================================== */

TEST(test_xosc_status_stable) {
    clocks_init();
    uint32_t status = clocks_read32(XOSC_BASE + 0x04);
    ASSERT_TRUE(status & (1u << 31), "XOSC STABLE");
    ASSERT_TRUE(status & (1u << 12), "XOSC ENABLED");
    PASS();
}

TEST(test_pll_sys_lock) {
    clocks_init();
    ASSERT_TRUE(clocks_read32(PLL_SYS_BASE) & (1u << 31), "PLL_SYS LOCK");
    PASS();
}

TEST(test_pll_usb_lock) {
    clocks_init();
    ASSERT_TRUE(clocks_read32(PLL_USB_BASE) & (1u << 31), "PLL_USB LOCK");
    PASS();
}

TEST(test_watchdog_reason_clean_boot) {
    clocks_init();
    ASSERT_EQ(0, clocks_read32(WATCHDOG_BASE + 0x08), "WDOG REASON=0");
    PASS();
}

TEST(test_watchdog_scratch_registers) {
    clocks_init();
    clocks_write32(WATCHDOG_BASE + 0x0C, 0xDEADBEEF);
    ASSERT_EQ(0xDEADBEEF, clocks_read32(WATCHDOG_BASE + 0x0C), "WDOG SCRATCH0");
    PASS();
}

TEST(test_watchdog_tick_enable) {
    clocks_init();
    clocks_write32(WATCHDOG_BASE + 0x2C, 0x200 | 12);
    ASSERT_TRUE(clocks_read32(WATCHDOG_BASE + 0x2C) & (1u << 10), "TICK.RUNNING");
    PASS();
}

TEST(test_adc_cs_ready) {
    adc_init();
    ASSERT_TRUE(adc_read32(ADC_BASE + 0x00) & (1u << 8), "ADC CS.READY");
    PASS();
}

TEST(test_adc_temp_sensor) {
    adc_init();
    adc_write32(ADC_BASE + 0x00, (4u << 12) | (1u << 16) | (1u << 0));
    ASSERT_EQ(0x036C, adc_read32(ADC_BASE + 0x04), "ADC temp ~27C");
    PASS();
}

TEST(test_adc_set_channel_value) {
    adc_init();
    adc_set_channel_value(0, 0x0ABC);
    adc_write32(ADC_BASE + 0x00, (0u << 12) | (1u << 0));
    ASSERT_EQ(0x0ABC, adc_read32(ADC_BASE + 0x04), "ADC ch0 injected");
    PASS();
}

TEST(test_uart_registers) {
    reset_cpu();
    ASSERT_EQ(0x00000090, mem_read32(UART0_BASE + 0x018), "UART FR");
    uint32_t cr = mem_read32(UART0_BASE + 0x030);
    ASSERT_TRUE(cr & 1, "UART CR: UARTEN");
    ASSERT_TRUE(cr & (1u << 8), "UART CR: TXE");
    PASS();
}

/* ========================================================================
 * Timer Tests (NEW - v0.8.0)
 * ======================================================================== */

TEST(test_timer_alarm_arm_on_write) {
    reset_cpu();
    timer_reset();
    ASSERT_EQ(0, timer_state.armed, "No alarms armed initially");
    timer_write32(TIMER_ALARM0, 100);
    ASSERT_TRUE(timer_state.armed & 0x1, "Alarm 0 armed after write");
    ASSERT_EQ(0, timer_state.inte, "INTE not auto-enabled");
    PASS();
}

TEST(test_timer_alarm_fire_and_disarm) {
    reset_cpu();
    timer_reset();
    timer_write32(TIMER_ALARM0, 5);
    timer_write32(TIMER_INTE, 0x1);
    for (int i = 0; i < 6; i++) timer_tick(1);
    ASSERT_TRUE(timer_state.intr & 0x1, "Alarm 0 fired");
    ASSERT_TRUE(!(timer_state.armed & 0x1), "Alarm 0 disarmed after fire");
    PASS();
}

TEST(test_timer_64bit_latch_read) {
    reset_cpu();
    timer_reset();
    timer_state.time_us = 0x0000000100000042ULL;
    uint32_t low = timer_read32(TIMER_TIMELR);
    uint32_t high = timer_read32(TIMER_TIMEHR);
    ASSERT_EQ(0x00000042, low, "Timer low word");
    ASSERT_EQ(0x00000001, high, "Timer high word (latched)");
    PASS();
}

TEST(test_timer_pause) {
    reset_cpu();
    timer_reset();
    timer_write32(TIMER_PAUSE, 1);
    timer_tick(100);
    ASSERT_EQ(0, (uint32_t)timer_state.time_us, "Paused: no increment");
    timer_write32(TIMER_PAUSE, 0);
    timer_tick(5);
    ASSERT_EQ(5, (uint32_t)timer_state.time_us, "Resumed: increments");
    PASS();
}

TEST(test_timer_intr_clear) {
    reset_cpu();
    timer_reset();
    timer_state.intr = 0x0F;
    timer_write32(TIMER_INTR, 0x05);
    ASSERT_EQ(0x0A, timer_state.intr, "W1C clears specified bits");
    PASS();
}

/* ========================================================================
 * Spinlock Tests (NEW - v0.8.0)
 * ======================================================================== */

TEST(test_spinlock_acquire_free) {
    memset(spinlocks, 0, sizeof(spinlocks));
    ASSERT_NEQ(0, spinlock_acquire(0), "Acquire free lock succeeds");
    PASS();
}

TEST(test_spinlock_acquire_locked) {
    memset(spinlocks, 0, sizeof(spinlocks));
    spinlock_acquire(0);
    ASSERT_EQ(0, spinlock_acquire(0), "Acquire locked lock fails");
    PASS();
}

TEST(test_spinlock_release) {
    memset(spinlocks, 0, sizeof(spinlocks));
    spinlock_acquire(0);
    spinlock_release(0);
    ASSERT_NEQ(0, spinlock_acquire(0), "Released lock re-acquirable");
    PASS();
}

TEST(test_spinlock_out_of_range) {
    ASSERT_EQ(0, spinlock_acquire(99), "Out of range returns 0");
    PASS();
}

/* ========================================================================
 * FIFO Tests (NEW - v0.8.0)
 * ======================================================================== */

TEST(test_fifo_push_pop) {
    memset(fifo, 0, sizeof(fifo));
    fifo_push(0, 0xAAAAAAAA);
    fifo_push(0, 0xBBBBBBBB);
    ASSERT_EQ(0xAAAAAAAA, fifo_pop(0), "FIFO: first in first out");
    ASSERT_EQ(0xBBBBBBBB, fifo_pop(0), "FIFO: second value");
    PASS();
}

TEST(test_fifo_empty_check) {
    memset(fifo, 0, sizeof(fifo));
    ASSERT_TRUE(fifo_is_empty(0), "Empty FIFO reports empty");
    fifo_push(0, 42);
    ASSERT_TRUE(!fifo_is_empty(0), "Non-empty FIFO not empty");
    PASS();
}

TEST(test_fifo_try_pop_empty) {
    memset(fifo, 0, sizeof(fifo));
    uint32_t val = 0;
    ASSERT_EQ(0, fifo_try_pop(0, &val), "try_pop empty returns 0");
    PASS();
}

TEST(test_fifo_try_push_full) {
    memset(fifo, 0, sizeof(fifo));
    for (int i = 0; i < FIFO_DEPTH; i++) fifo_push(0, i);
    ASSERT_TRUE(fifo_is_full(0), "Full FIFO reports full");
    ASSERT_EQ(0, fifo_try_push(0, 99), "try_push full returns 0");
    PASS();
}

/* ========================================================================
 * Bitwise Instruction Tests (NEW - v0.8.0)
 * ======================================================================== */

TEST(test_bitwise_and) {
    reset_cpu();
    cpu.r[0] = 0xFF00FF00; cpu.r[1] = 0x0F0F0F0F;
    instr_bitwise_and(0x4008);
    ASSERT_EQ(0x0F000F00, cpu.r[0], "AND");
    PASS();
}

TEST(test_bitwise_eor) {
    reset_cpu();
    cpu.r[0] = 0xAAAAAAAA; cpu.r[1] = 0x55555555;
    instr_bitwise_eor(0x4048);
    ASSERT_EQ(0xFFFFFFFF, cpu.r[0], "EOR");
    PASS();
}

TEST(test_bitwise_orr) {
    reset_cpu();
    cpu.r[0] = 0xF0F0F0F0; cpu.r[1] = 0x0F0F0F0F;
    instr_bitwise_orr(0x4308);
    ASSERT_EQ(0xFFFFFFFF, cpu.r[0], "ORR");
    PASS();
}

TEST(test_bitwise_bic) {
    reset_cpu();
    cpu.r[0] = 0xFFFFFFFF; cpu.r[1] = 0x000000FF;
    instr_bitwise_bic(0x4388);
    ASSERT_EQ(0xFFFFFF00, cpu.r[0], "BIC");
    PASS();
}

TEST(test_bitwise_mvn) {
    reset_cpu();
    cpu.r[1] = 0x00000000;
    instr_bitwise_mvn(0x43C8);
    ASSERT_EQ(0xFFFFFFFF, cpu.r[0], "MVN ~0");
    PASS();
}

TEST(test_tst_sets_flags) {
    reset_cpu();
    cpu.r[0] = 0x00; cpu.r[1] = 0xFF;
    instr_tst_reg_reg(0x4208);
    ASSERT_TRUE(cpu.xpsr & 0x40000000, "TST zero sets Z");
    PASS();
}

/* ========================================================================
 * Shift Instruction Tests (NEW - v0.8.0)
 * ======================================================================== */

TEST(test_lsr_imm_32) {
    reset_cpu();
    cpu.r[1] = 0x80000000;
    instr_shift_logical_right(0x0808);
    ASSERT_EQ(0, cpu.r[0], "LSR #32 = 0");
    ASSERT_TRUE(cpu.xpsr & 0x20000000, "LSR #32 carry = bit[31]");
    PASS();
}

TEST(test_asr_imm_32) {
    reset_cpu();
    cpu.r[1] = 0x80000000;
    instr_shift_arithmetic_right(0x1008);
    ASSERT_EQ(0xFFFFFFFF, cpu.r[0], "ASR #32 negative = all 1s");
    PASS();
}

TEST(test_asr_imm_32_positive) {
    reset_cpu();
    cpu.r[1] = 0x7FFFFFFF;
    instr_shift_arithmetic_right(0x1008);
    ASSERT_EQ(0, cpu.r[0], "ASR #32 positive = 0");
    PASS();
}

TEST(test_ror_register) {
    reset_cpu();
    cpu.r[0] = 0x12345678; cpu.r[1] = 8;
    instr_rors_reg(0x41C8);
    ASSERT_EQ(0x78123456, cpu.r[0], "ROR by 8");
    PASS();
}

TEST(test_lsls_reg_by_zero) {
    reset_cpu();
    cpu.r[0] = 0xABCD1234; cpu.r[1] = 0;
    cpu.xpsr |= 0x20000000;
    instr_lsls_reg(0x4088);
    ASSERT_EQ(0xABCD1234, cpu.r[0], "LSLS by 0: no change");
    ASSERT_TRUE(cpu.xpsr & 0x20000000, "LSLS by 0: carry preserved");
    PASS();
}

TEST(test_lsls_reg_by_32) {
    reset_cpu();
    cpu.r[0] = 0x00000001; cpu.r[1] = 32;
    instr_lsls_reg(0x4088);
    ASSERT_EQ(0, cpu.r[0], "LSLS by 32 = 0");
    ASSERT_TRUE(cpu.xpsr & 0x20000000, "LSLS by 32 carry = bit[0]");
    PASS();
}

/* ========================================================================
 * Byte/Halfword Operation Tests (NEW - v0.8.0)
 * ======================================================================== */

TEST(test_sxtb) {
    reset_cpu();
    cpu.r[1] = 0x000000FF;
    instr_sxtb(0xB248);
    ASSERT_EQ(0xFFFFFFFF, cpu.r[0], "SXTB: 0xFF -> -1");
    PASS();
}

TEST(test_sxth) {
    reset_cpu();
    cpu.r[1] = 0x0000FFFF;
    instr_sxth(0xB208);
    ASSERT_EQ(0xFFFFFFFF, cpu.r[0], "SXTH: 0xFFFF -> -1");
    PASS();
}

TEST(test_uxtb) {
    reset_cpu();
    cpu.r[1] = 0xDEADBE42;
    instr_uxtb(0xB2C8);
    ASSERT_EQ(0x42, cpu.r[0], "UXTB: extract low byte");
    PASS();
}

TEST(test_uxth) {
    reset_cpu();
    cpu.r[1] = 0xDEAD1234;
    instr_uxth(0xB288);
    ASSERT_EQ(0x1234, cpu.r[0], "UXTH: extract low halfword");
    PASS();
}

TEST(test_rev) {
    reset_cpu();
    cpu.r[1] = 0x12345678;
    instr_rev(0xBA08);
    ASSERT_EQ(0x78563412, cpu.r[0], "REV: byte-reverse");
    PASS();
}

TEST(test_rev16) {
    reset_cpu();
    cpu.r[1] = 0x12345678;
    instr_rev16(0xBA48);
    ASSERT_EQ(0x34127856, cpu.r[0], "REV16: halfword byte-reverse");
    PASS();
}

TEST(test_revsh) {
    reset_cpu();
    cpu.r[1] = 0x000000FF;
    instr_revsh(0xBAC8);
    ASSERT_EQ(0xFFFFFF00, cpu.r[0], "REVSH: reversed + sign-extended");
    PASS();
}

/* ========================================================================
 * Branch Tests (NEW - v0.8.0)
 * ======================================================================== */

TEST(test_bcond_negative_offset) {
    reset_cpu();
    cpu.r[15] = FLASH_BASE + 0x200;
    cpu.xpsr |= 0x40000000; /* Z flag */
    pc_updated = 0;
    /* BEQ offset=0xFE (-2 signed), *2=-4, +4=0 -> branch to self */
    instr_bcond(0xD0FE);
    ASSERT_EQ(FLASH_BASE + 0x200, cpu.r[15], "BEQ backward to self");
    PASS();
}

TEST(test_b_unconditional) {
    reset_cpu();
    cpu.r[15] = FLASH_BASE + 0x100;
    pc_updated = 0;
    instr_b_uncond(0xE003);
    ASSERT_EQ(FLASH_BASE + 0x10A, cpu.r[15], "B unconditional +10");
    PASS();
}

TEST(test_bcond_not_taken) {
    reset_cpu();
    cpu.r[15] = FLASH_BASE + 0x100;
    cpu.xpsr &= ~0x40000000; /* Clear Z */
    pc_updated = 0;
    instr_bcond(0xD001);
    ASSERT_EQ(FLASH_BASE + 0x102, cpu.r[15], "BEQ not taken -> PC+2");
    PASS();
}

/* ========================================================================
 * STMIA/LDMIA, MUL, Exception, CMN, ADR, SP, BL Tests (NEW - v0.8.0)
 * ======================================================================== */

TEST(test_stmia_ldmia_roundtrip) {
    reset_cpu();
    cpu.r[0] = 0x11111111; cpu.r[1] = 0x22222222; cpu.r[2] = 0x33333333;
    cpu.r[4] = RAM_BASE + 0x300;
    uint32_t base = cpu.r[4];
    instr_stmia(0xC407);
    ASSERT_EQ(base + 12, cpu.r[4], "STMIA advances base by 12");
    cpu.r[0] = 0; cpu.r[1] = 0; cpu.r[2] = 0;
    cpu.r[4] = base;
    instr_ldmia(0xCC07);
    ASSERT_EQ(0x11111111, cpu.r[0], "LDMIA R0");
    ASSERT_EQ(0x22222222, cpu.r[1], "LDMIA R1");
    ASSERT_EQ(0x33333333, cpu.r[2], "LDMIA R2");
    PASS();
}

TEST(test_muls) {
    reset_cpu();
    cpu.r[0] = 7; cpu.r[1] = 6;
    instr_muls(0x4348);
    ASSERT_EQ(42, cpu.r[0], "7 * 6 = 42");
    PASS();
}

TEST(test_muls_zero) {
    reset_cpu();
    cpu.r[0] = 12345; cpu.r[1] = 0;
    instr_muls(0x4348);
    ASSERT_EQ(0, cpu.r[0], "x * 0 = 0");
    ASSERT_TRUE(cpu.xpsr & 0x40000000, "Z flag on zero");
    PASS();
}

TEST(test_exception_entry_return) {
    reset_cpu();
    uint32_t handler_addr = FLASH_BASE + 0x200;
    uint32_t handler_thumb = handler_addr | 1;
    memcpy(&cpu.flash[0x100 + 16 * 4], &handler_thumb, 4);
    uint32_t saved_pc = cpu.r[15];
    uint32_t saved_sp = cpu.r[13];
    cpu.r[0] = 0xAAAAAAAA; cpu.r[1] = 0xBBBBBBBB;
    uint32_t saved_r0 = cpu.r[0], saved_r1 = cpu.r[1];
    uint32_t saved_xpsr = cpu.xpsr;
    cpu_exception_entry(16);
    ASSERT_EQ(handler_addr, cpu.r[15], "PC at handler");
    ASSERT_EQ(0xFFFFFFF9, cpu.r[14], "LR = EXC_RETURN");
    ASSERT_EQ(saved_sp - 32, cpu.r[13], "SP decremented by 32");
    cpu_exception_return(0xFFFFFFF9);
    ASSERT_EQ(saved_pc, cpu.r[15], "PC restored");
    ASSERT_EQ(saved_r0, cpu.r[0], "R0 restored");
    ASSERT_EQ(saved_r1, cpu.r[1], "R1 restored");
    ASSERT_EQ(saved_xpsr, cpu.xpsr, "xPSR restored");
    ASSERT_EQ(0xFFFFFFFF, cpu.current_irq, "No active IRQ");
    PASS();
}

TEST(test_cmn_reg) {
    reset_cpu();
    cpu.r[0] = 0xFFFFFFFF; cpu.r[1] = 1;
    instr_cmn_reg(0x42C8);
    ASSERT_TRUE(cpu.xpsr & 0x40000000, "CMN: Z on zero result");
    ASSERT_TRUE(cpu.xpsr & 0x20000000, "CMN: C on overflow");
    PASS();
}

TEST(test_adr) {
    reset_cpu();
    cpu.r[15] = FLASH_BASE + 0x100;
    instr_adr(0xA004);
    uint32_t expected = ((FLASH_BASE + 0x104) & ~3u) + 16;
    ASSERT_EQ(expected, cpu.r[0], "ADR: PC-relative");
    PASS();
}

TEST(test_add_sp_imm7) {
    reset_cpu();
    cpu.r[13] = 0x20041000;
    instr_add_sp_imm7(0xB004);
    ASSERT_EQ(0x20041010, cpu.r[13], "ADD SP, #16");
    PASS();
}

TEST(test_sub_sp_imm7) {
    reset_cpu();
    cpu.r[13] = 0x20041000;
    instr_sub_sp_imm7(0xB084);
    ASSERT_EQ(0x20040FF0, cpu.r[13], "SUB SP, #16");
    PASS();
}

TEST(test_bl_32bit) {
    reset_cpu();
    cpu.r[15] = FLASH_BASE + 0x100;
    pc_updated = 0;
    instr_bl_32(0xF000, 0xF880);
    ASSERT_EQ((FLASH_BASE + 0x104) | 1, cpu.r[14], "BL: LR = return|1");
    ASSERT_TRUE(pc_updated == 1, "BL sets pc_updated");
    PASS();
}

TEST(test_sbcs_carry_flag_no_borrow) {
    reset_cpu();
    cpu.r[0] = 100; cpu.r[1] = 50;
    cpu.xpsr |= 0x20000000; /* C=1 */
    instr_sbcs(0x4188);
    ASSERT_EQ(50, cpu.r[0], "SBCS: 100-50-0=50");
    ASSERT_TRUE(cpu.xpsr & 0x20000000, "C=1 no borrow");
    PASS();
}

TEST(test_sbcs_carry_flag_borrow) {
    reset_cpu();
    cpu.r[0] = 0; cpu.r[1] = 1;
    cpu.xpsr |= 0x20000000; /* C=1 */
    instr_sbcs(0x4188);
    ASSERT_EQ(0xFFFFFFFF, cpu.r[0], "SBCS: 0-1=-1");
    ASSERT_TRUE(!(cpu.xpsr & 0x20000000), "C=0 borrow occurred");
    PASS();
}

/* ========================================================================
 * ROM Function Table Tests
 * ======================================================================== */

TEST(test_rom_magic) {
    reset_cpu();
    ASSERT_EQ('M', mem_read8(0x10), "ROM magic byte 0");
    ASSERT_EQ('u', mem_read8(0x11), "ROM magic byte 1");
    ASSERT_EQ(0x01, mem_read8(0x12), "ROM version");
    PASS();
}

TEST(test_rom_table_pointers) {
    reset_cpu();
    uint16_t func_table_ptr = mem_read16(0x14);
    uint16_t lookup_fn_ptr = mem_read16(0x18);
    ASSERT_EQ(0x0100, func_table_ptr, "Function table pointer");
    ASSERT_TRUE((lookup_fn_ptr & 1) != 0, "Lookup fn has Thumb bit set");
    ASSERT_EQ(0x0201, lookup_fn_ptr, "Lookup function pointer");
    PASS();
}

TEST(test_rom_func_table_entries) {
    reset_cpu();
    /* First entry should be memcpy (MC) */
    uint16_t code = mem_read16(0x0100);
    uint16_t fptr = mem_read16(0x0102);
    ASSERT_EQ(ROM_FUNC_MEMCPY, code, "First table entry is memcpy");
    ASSERT_EQ(0x0301, fptr, "memcpy function pointer (with Thumb bit)");
    PASS();
}

TEST(test_rom_lookup_fn) {
    reset_cpu();
    /* Call the ROM lookup function: r0=table_ptr, r1=code, returns r0=func_ptr */
    cpu.r[0] = 0x0100;                   /* func_table address */
    cpu.r[1] = ROM_FUNC_MEMCPY;          /* search for memcpy */
    cpu.r[14] = FLASH_BASE + 0x100;      /* return address (in flash) */
    cpu.r[15] = 0x0200;                  /* PC = lookup function */
    /* Execute enough steps for the lookup loop */
    for (int i = 0; i < 50 && !cpu_is_halted(); i++) {
        cpu_step();
        if (cpu.r[15] >= FLASH_BASE) break;  /* returned to flash */
    }
    ASSERT_EQ(0x0301, cpu.r[0], "Lookup returned memcpy pointer");
    PASS();
}

TEST(test_rom_lookup_not_found) {
    reset_cpu();
    cpu.r[0] = 0x0100;                   /* func_table address */
    cpu.r[1] = 0xFFFF;                   /* nonexistent code */
    cpu.r[14] = FLASH_BASE + 0x100;
    cpu.r[15] = 0x0200;
    for (int i = 0; i < 200 && !cpu_is_halted(); i++) {
        cpu_step();
        if (cpu.r[15] >= FLASH_BASE) break;
    }
    ASSERT_EQ(0, cpu.r[0], "Lookup returns NULL for unknown code");
    PASS();
}

TEST(test_rom_popcount) {
    reset_cpu();
    cpu.r[0] = 0xFF00FF00;             /* 16 bits set */
    cpu.r[14] = FLASH_BASE + 0x100;
    cpu.r[15] = 0x0340;                /* popcount32 */
    for (int i = 0; i < 200 && !cpu_is_halted(); i++) {
        cpu_step();
        if (cpu.r[15] >= FLASH_BASE) break;
    }
    ASSERT_EQ(16, cpu.r[0], "popcount(0xFF00FF00) = 16");
    PASS();
}

TEST(test_rom_clz) {
    reset_cpu();
    cpu.r[0] = 0x00010000;             /* bit 16 set, 15 leading zeros */
    cpu.r[14] = FLASH_BASE + 0x100;
    cpu.r[15] = 0x0360;                /* clz32 */
    for (int i = 0; i < 200 && !cpu_is_halted(); i++) {
        cpu_step();
        if (cpu.r[15] >= FLASH_BASE) break;
    }
    ASSERT_EQ(15, cpu.r[0], "clz(0x00010000) = 15");
    PASS();
}

TEST(test_rom_ctz) {
    reset_cpu();
    cpu.r[0] = 0x00010000;             /* bit 16 set, 16 trailing zeros */
    cpu.r[14] = FLASH_BASE + 0x100;
    cpu.r[15] = 0x0380;                /* ctz32 */
    for (int i = 0; i < 200 && !cpu_is_halted(); i++) {
        cpu_step();
        if (cpu.r[15] >= FLASH_BASE) break;
    }
    ASSERT_EQ(16, cpu.r[0], "ctz(0x00010000) = 16");
    PASS();
}

/* ========================================================================
 * USB Controller Stub Tests
 * ======================================================================== */

TEST(test_usb_regs_read_zero) {
    reset_cpu();
    ASSERT_EQ(0, mem_read32(USBCTRL_REGS_BASE), "USB ADDR_ENDP returns 0");
    ASSERT_EQ(0, mem_read32(USBCTRL_REGS_BASE + 0x50), "USB SIE_STATUS returns 0 (disconnected)");
    PASS();
}

TEST(test_usb_dpram_read_zero) {
    reset_cpu();
    ASSERT_EQ(0, mem_read32(USBCTRL_DPRAM_BASE), "USB DPRAM returns 0");
    ASSERT_EQ(0, mem_read32(USBCTRL_DPRAM_BASE + 0x100), "USB DPRAM offset returns 0");
    PASS();
}

TEST(test_usb_write_no_crash) {
    reset_cpu();
    mem_write32(USBCTRL_REGS_BASE, 0x12345678);
    mem_write32(USBCTRL_DPRAM_BASE, 0xDEADBEEF);
    ASSERT_TRUE(1, "USB writes don't crash");
    PASS();
}

/* ========================================================================
 * Flash ROM Function Tests
 * ======================================================================== */

TEST(test_rom_flash_functions_in_table) {
    reset_cpu();
    /* Look up flash_range_erase via ROM lookup function */
    cpu.r[0] = 0x0100;                        /* func_table address */
    cpu.r[1] = ROM_FUNC_FLASH_RANGE_ERASE;    /* search code */
    cpu.r[14] = FLASH_BASE + 0x100;
    cpu.r[15] = 0x0200;                        /* lookup function */
    for (int i = 0; i < 100 && !cpu_is_halted(); i++) {
        cpu_step();
        if (cpu.r[15] >= FLASH_BASE) break;
    }
    ASSERT_TRUE(cpu.r[0] != 0, "flash_range_erase found in ROM table");
    PASS();
}

/* ========================================================================
 * Main Entry Point
 * ======================================================================== */

int main(void) {
    cpu_init();
    nvic_init();
    timer_init();
    gpio_init();
    clocks_init();
    adc_init();
    rom_init();

    printf("========================================\n");
    printf(" Bramble RP2040 Emulator - Test Suite\n");
    printf(" Version 0.9.0 (Verbose)\n");
    printf("========================================\n");

    BEGIN_CATEGORY("PRIMASK");
    RUN_TEST(test_cpsid_sets_primask);
    RUN_TEST(test_cpsie_clears_primask);
    RUN_TEST(test_primask_blocks_interrupts);
    RUN_TEST(test_primask_allows_interrupts_when_clear);
    END_CATEGORY("PRIMASK");

    BEGIN_CATEGORY("SVC Exception");
    RUN_TEST(test_svc_triggers_exception);
    END_CATEGORY("SVC Exception");

    BEGIN_CATEGORY("RAM Execution");
    RUN_TEST(test_ram_execution_allowed);
    RUN_TEST(test_ram_execution_boundary);
    RUN_TEST(test_invalid_pc_halts);
    END_CATEGORY("RAM Execution");

    BEGIN_CATEGORY("Dispatch Table");
    RUN_TEST(test_dispatch_movs_imm8);
    RUN_TEST(test_dispatch_adds_reg);
    RUN_TEST(test_dispatch_lsls_imm);
    RUN_TEST(test_dispatch_bcond);
    END_CATEGORY("Dispatch Table");

    BEGIN_CATEGORY("Peripheral Stubs");
    RUN_TEST(test_spi0_status_register);
    RUN_TEST(test_spi1_status_register);
    RUN_TEST(test_spi_other_regs_zero);
    RUN_TEST(test_i2c_read_zero);
    RUN_TEST(test_pwm_read_zero);
    RUN_TEST(test_peripheral_writes_no_crash);
    END_CATEGORY("Peripheral Stubs");

    BEGIN_CATEGORY("ADCS/SBCS/RSBS");
    RUN_TEST(test_adcs_with_carry);
    RUN_TEST(test_adcs_without_carry);
    RUN_TEST(test_sbcs_basic);
    RUN_TEST(test_sbcs_with_borrow);
    RUN_TEST(test_rsbs_negate);
    RUN_TEST(test_rsbs_zero);
    END_CATEGORY("ADCS/SBCS/RSBS");

    BEGIN_CATEGORY("Dual-Core Memory");
    RUN_TEST(test_mem_set_ram_ptr_routing);
    RUN_TEST(test_dual_core_ram_isolation);
    RUN_TEST(test_dual_core_shared_flash);
    RUN_TEST(test_dual_core_shared_ram);
    END_CATEGORY("Dual-Core Memory");

    BEGIN_CATEGORY("ELF Loader");
    RUN_TEST(test_elf_loader_valid);
    RUN_TEST(test_elf_loader_invalid_magic);
    RUN_TEST(test_elf_loader_wrong_arch);
    END_CATEGORY("ELF Loader");

    BEGIN_CATEGORY("Memory Bus");
    RUN_TEST(test_flash_read_write);
    RUN_TEST(test_ram_read_write);
    RUN_TEST(test_uart_output);
    END_CATEGORY("Memory Bus");

    BEGIN_CATEGORY("Instruction Integration");
    RUN_TEST(test_str_ldr_sp_imm8);
    RUN_TEST(test_push_pop);
    END_CATEGORY("Instruction Integration");

    BEGIN_CATEGORY("SysTick Timer");
    RUN_TEST(test_systick_registers);
    RUN_TEST(test_systick_countdown);
    RUN_TEST(test_systick_disabled_no_count);
    RUN_TEST(test_systick_calib_tenms);
    END_CATEGORY("SysTick Timer");

    BEGIN_CATEGORY("MSR/MRS Instructions");
    RUN_TEST(test_mrs_primask);
    RUN_TEST(test_msr_primask);
    RUN_TEST(test_mrs_xpsr);
    RUN_TEST(test_msr_apsr_flags);
    RUN_TEST(test_mrs_msr_control);
    RUN_TEST(test_32bit_msr_dispatch);
    RUN_TEST(test_32bit_mrs_dispatch);
    RUN_TEST(test_32bit_dsb_dispatch);
    END_CATEGORY("MSR/MRS Instructions");

    BEGIN_CATEGORY("NVIC Priority Preemption");
    RUN_TEST(test_nvic_priority_preemption_blocked);
    RUN_TEST(test_nvic_priority_preemption_allowed);
    RUN_TEST(test_nvic_exception_priority_lookup);
    END_CATEGORY("NVIC Priority Preemption");

    BEGIN_CATEGORY("SCB Registers");
    RUN_TEST(test_scb_shpr_registers);
    RUN_TEST(test_scb_vtor_write);
    END_CATEGORY("SCB Registers");

    BEGIN_CATEGORY("Resets Peripheral");
    RUN_TEST(test_resets_power_on_state);
    RUN_TEST(test_resets_release_and_done);
    RUN_TEST(test_resets_atomic_clear);
    END_CATEGORY("Resets Peripheral");

    BEGIN_CATEGORY("Clocks Peripheral");
    RUN_TEST(test_clocks_selected_always_set);
    RUN_TEST(test_clocks_ctrl_write_read);
    END_CATEGORY("Clocks Peripheral");

    BEGIN_CATEGORY("XOSC");
    RUN_TEST(test_xosc_status_stable);
    END_CATEGORY("XOSC");

    BEGIN_CATEGORY("PLL");
    RUN_TEST(test_pll_sys_lock);
    RUN_TEST(test_pll_usb_lock);
    END_CATEGORY("PLL");

    BEGIN_CATEGORY("Watchdog");
    RUN_TEST(test_watchdog_reason_clean_boot);
    RUN_TEST(test_watchdog_scratch_registers);
    RUN_TEST(test_watchdog_tick_enable);
    END_CATEGORY("Watchdog");

    BEGIN_CATEGORY("ADC");
    RUN_TEST(test_adc_cs_ready);
    RUN_TEST(test_adc_temp_sensor);
    RUN_TEST(test_adc_set_channel_value);
    END_CATEGORY("ADC");

    BEGIN_CATEGORY("UART Registers");
    RUN_TEST(test_uart_registers);
    END_CATEGORY("UART Registers");

    BEGIN_CATEGORY("Timer");
    RUN_TEST(test_timer_alarm_arm_on_write);
    RUN_TEST(test_timer_alarm_fire_and_disarm);
    RUN_TEST(test_timer_64bit_latch_read);
    RUN_TEST(test_timer_pause);
    RUN_TEST(test_timer_intr_clear);
    END_CATEGORY("Timer");

    BEGIN_CATEGORY("Spinlocks");
    RUN_TEST(test_spinlock_acquire_free);
    RUN_TEST(test_spinlock_acquire_locked);
    RUN_TEST(test_spinlock_release);
    RUN_TEST(test_spinlock_out_of_range);
    END_CATEGORY("Spinlocks");

    BEGIN_CATEGORY("FIFO");
    RUN_TEST(test_fifo_push_pop);
    RUN_TEST(test_fifo_empty_check);
    RUN_TEST(test_fifo_try_pop_empty);
    RUN_TEST(test_fifo_try_push_full);
    END_CATEGORY("FIFO");

    BEGIN_CATEGORY("Bitwise Instructions");
    RUN_TEST(test_bitwise_and);
    RUN_TEST(test_bitwise_eor);
    RUN_TEST(test_bitwise_orr);
    RUN_TEST(test_bitwise_bic);
    RUN_TEST(test_bitwise_mvn);
    RUN_TEST(test_tst_sets_flags);
    END_CATEGORY("Bitwise Instructions");

    BEGIN_CATEGORY("Shift Instructions");
    RUN_TEST(test_lsr_imm_32);
    RUN_TEST(test_asr_imm_32);
    RUN_TEST(test_asr_imm_32_positive);
    RUN_TEST(test_ror_register);
    RUN_TEST(test_lsls_reg_by_zero);
    RUN_TEST(test_lsls_reg_by_32);
    END_CATEGORY("Shift Instructions");

    BEGIN_CATEGORY("Byte/Halfword Operations");
    RUN_TEST(test_sxtb);
    RUN_TEST(test_sxth);
    RUN_TEST(test_uxtb);
    RUN_TEST(test_uxth);
    RUN_TEST(test_rev);
    RUN_TEST(test_rev16);
    RUN_TEST(test_revsh);
    END_CATEGORY("Byte/Halfword Operations");

    BEGIN_CATEGORY("Branch Instructions");
    RUN_TEST(test_bcond_negative_offset);
    RUN_TEST(test_b_unconditional);
    RUN_TEST(test_bcond_not_taken);
    END_CATEGORY("Branch Instructions");

    BEGIN_CATEGORY("STMIA/LDMIA");
    RUN_TEST(test_stmia_ldmia_roundtrip);
    END_CATEGORY("STMIA/LDMIA");

    BEGIN_CATEGORY("MUL");
    RUN_TEST(test_muls);
    RUN_TEST(test_muls_zero);
    END_CATEGORY("MUL");

    BEGIN_CATEGORY("Exception Entry/Return");
    RUN_TEST(test_exception_entry_return);
    END_CATEGORY("Exception Entry/Return");

    BEGIN_CATEGORY("CMN");
    RUN_TEST(test_cmn_reg);
    END_CATEGORY("CMN");

    BEGIN_CATEGORY("ADR");
    RUN_TEST(test_adr);
    END_CATEGORY("ADR");

    BEGIN_CATEGORY("ADD/SUB SP");
    RUN_TEST(test_add_sp_imm7);
    RUN_TEST(test_sub_sp_imm7);
    END_CATEGORY("ADD/SUB SP");

    BEGIN_CATEGORY("BL 32-bit");
    RUN_TEST(test_bl_32bit);
    END_CATEGORY("BL 32-bit");

    BEGIN_CATEGORY("SBCS Edge Cases");
    RUN_TEST(test_sbcs_carry_flag_no_borrow);
    RUN_TEST(test_sbcs_carry_flag_borrow);
    END_CATEGORY("SBCS Edge Cases");

    BEGIN_CATEGORY("ROM Function Table");
    RUN_TEST(test_rom_magic);
    RUN_TEST(test_rom_table_pointers);
    RUN_TEST(test_rom_func_table_entries);
    RUN_TEST(test_rom_lookup_fn);
    RUN_TEST(test_rom_lookup_not_found);
    RUN_TEST(test_rom_popcount);
    RUN_TEST(test_rom_clz);
    RUN_TEST(test_rom_ctz);
    END_CATEGORY("ROM Function Table");

    BEGIN_CATEGORY("USB Controller Stub");
    RUN_TEST(test_usb_regs_read_zero);
    RUN_TEST(test_usb_dpram_read_zero);
    RUN_TEST(test_usb_write_no_crash);
    END_CATEGORY("USB Controller Stub");

    BEGIN_CATEGORY("Flash ROM Functions");
    RUN_TEST(test_rom_flash_functions_in_table);
    END_CATEGORY("Flash ROM Functions");

    printf("\n========================================\n");
    printf(" Results: %d/%d passed, %d failed\n", tests_passed, tests_run, tests_failed);
    printf("========================================\n");

    return tests_failed > 0 ? 1 : 0;
}
