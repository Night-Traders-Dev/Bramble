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
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "pwm.h"
#include "dma.h"
#include "pio.h"
#include "usb.h"

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
    uart_init();
    spi_init();
    i2c_init();
    pwm_init();
    dma_init();
    pio_init();
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

TEST(test_i2c_con_default) {
    reset_cpu();
    ASSERT_EQ(0x7F, mem_read32(I2C0_BASE + I2C_CON), "I2C0 CON default");
    PASS();
}

TEST(test_pwm_csr_default) {
    reset_cpu();
    ASSERT_EQ(0, mem_read32(PWM_BASE + PWM_CH_CSR), "PWM CSR default 0");
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
    mem_write32(UART0_BASE + UART_DR, 'X');
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

/* ========================================================================
 * ADC FIFO Tests
 * ======================================================================== */

TEST(test_adc_fifo_push_pop) {
    adc_init();
    adc_set_channel_value(0, 0x0ABC);
    /* Enable ADC, select channel 0 */
    adc_state.cs = ADC_CS_EN | (0u << ADC_CS_AINSEL_SHIFT);
    /* Enable FIFO */
    adc_state.fcs = ADC_FCS_EN;

    /* Trigger conversion */
    adc_do_conversion();

    /* FIFO level should be 1 */
    uint32_t fcs = adc_read32(ADC_BASE + 0x08);
    uint32_t level = (fcs >> ADC_FCS_LEVEL_SHIFT) & 0xF;
    ASSERT_EQ(1, level, "FIFO level should be 1 after one conversion");

    /* Pop from FIFO */
    uint32_t val = adc_read32(ADC_BASE + 0x0C);
    ASSERT_EQ(0x0ABC, val, "FIFO pop should return channel value");

    /* FIFO should be empty now */
    fcs = adc_read32(ADC_BASE + 0x08);
    ASSERT_TRUE(fcs & ADC_FCS_EMPTY, "FIFO should be empty after pop");
    PASS();
}

TEST(test_adc_fifo_overflow) {
    adc_init();
    adc_set_channel_value(0, 100);
    adc_state.cs = ADC_CS_EN | (0u << ADC_CS_AINSEL_SHIFT);
    adc_state.fcs = ADC_FCS_EN;

    /* Push 4 entries (full) */
    for (int i = 0; i < 4; i++) {
        adc_do_conversion();
    }

    uint32_t fcs = adc_read32(ADC_BASE + 0x08);
    ASSERT_TRUE(fcs & ADC_FCS_FULL, "FIFO should be full at depth 4");

    /* 5th conversion should overflow */
    adc_do_conversion();
    fcs = adc_read32(ADC_BASE + 0x08);
    ASSERT_TRUE(fcs & ADC_FCS_OVER, "Overflow flag should be set");
    PASS();
}

TEST(test_adc_fifo_underflow) {
    adc_init();
    adc_state.fcs = ADC_FCS_EN;

    /* Read from empty FIFO */
    adc_read32(ADC_BASE + 0x0C);

    uint32_t fcs = adc_read32(ADC_BASE + 0x08);
    ASSERT_TRUE(fcs & ADC_FCS_UNDER, "Underflow flag should be set");
    PASS();
}

TEST(test_adc_fifo_shift) {
    adc_init();
    adc_set_channel_value(0, 0x0FFF);  /* 12-bit max */
    adc_state.cs = ADC_CS_EN | (0u << ADC_CS_AINSEL_SHIFT);
    adc_state.fcs = ADC_FCS_EN | ADC_FCS_SHIFT;  /* Enable shift (12-bit -> 8-bit) */

    adc_do_conversion();
    uint32_t val = adc_read32(ADC_BASE + 0x0C);
    /* 0x0FFF >> 4 = 0xFF */
    ASSERT_EQ(0xFF, val, "Shifted result should be 8-bit (0xFF)");
    PASS();
}

TEST(test_adc_fifo_w1c_flags) {
    adc_init();
    adc_state.fcs = ADC_FCS_EN;
    adc_state.fifo_over = 1;
    adc_state.fifo_under = 1;

    /* Verify flags are set */
    uint32_t fcs = adc_read32(ADC_BASE + 0x08);
    ASSERT_TRUE(fcs & ADC_FCS_OVER, "OVER flag set");
    ASSERT_TRUE(fcs & ADC_FCS_UNDER, "UNDER flag set");

    /* W1C: clear OVER */
    adc_write32(ADC_BASE + 0x08, ADC_FCS_OVER);
    fcs = adc_read32(ADC_BASE + 0x08);
    ASSERT_TRUE(!(fcs & ADC_FCS_OVER), "OVER flag cleared by W1C");
    ASSERT_TRUE(fcs & ADC_FCS_UNDER, "UNDER flag still set");
    PASS();
}

TEST(test_adc_rrobin) {
    adc_init();
    adc_set_channel_value(0, 100);
    adc_set_channel_value(2, 200);
    /* Enable channels 0 and 2 in round-robin, start on channel 0 */
    adc_state.cs = ADC_CS_EN |
                   (0u << ADC_CS_AINSEL_SHIFT) |
                   ((1u << 0 | 1u << 2) << ADC_CS_RROBIN_SHIFT);
    adc_state.fcs = ADC_FCS_EN;

    /* First conversion: channel 0 */
    adc_do_conversion();
    uint32_t val0 = adc_read32(ADC_BASE + 0x0C);
    ASSERT_EQ(100, val0, "First conversion should be channel 0");

    /* AINSEL should now be 2 */
    uint32_t ainsel = (adc_state.cs & ADC_CS_AINSEL_MASK) >> ADC_CS_AINSEL_SHIFT;
    ASSERT_EQ(2, ainsel, "AINSEL should advance to channel 2");

    /* Second conversion: channel 2 */
    adc_do_conversion();
    uint32_t val2 = adc_read32(ADC_BASE + 0x0C);
    ASSERT_EQ(200, val2, "Second conversion should be channel 2");

    /* AINSEL should wrap back to 0 */
    ainsel = (adc_state.cs & ADC_CS_AINSEL_MASK) >> ADC_CS_AINSEL_SHIFT;
    ASSERT_EQ(0, ainsel, "AINSEL should wrap back to channel 0");
    PASS();
}

TEST(test_adc_start_once_triggers_conversion) {
    adc_init();
    adc_set_channel_value(1, 0x0555);
    adc_state.cs = ADC_CS_EN | (1u << ADC_CS_AINSEL_SHIFT);
    adc_state.fcs = ADC_FCS_EN;

    /* Write START_ONCE via CS register */
    adc_write32(ADC_BASE + 0x00, adc_state.cs | ADC_CS_START_ONCE);

    /* FIFO should have one entry */
    uint32_t fcs = adc_read32(ADC_BASE + 0x08);
    uint32_t level = (fcs >> ADC_FCS_LEVEL_SHIFT) & 0xF;
    ASSERT_EQ(1, level, "START_ONCE should trigger one conversion");

    uint32_t val = adc_read32(ADC_BASE + 0x0C);
    ASSERT_EQ(0x0555, val, "FIFO should contain channel 1 value");

    /* START_ONCE should be auto-cleared */
    ASSERT_TRUE(!(adc_state.cs & ADC_CS_START_ONCE), "START_ONCE should auto-clear");
    PASS();
}

TEST(test_uart_registers) {
    reset_cpu();
    ASSERT_EQ(0x00000090, mem_read32(UART0_BASE + 0x018), "UART FR");
    /* UART starts disabled per PL011 hardware reset */
    ASSERT_EQ(0, mem_read32(UART0_BASE + 0x030), "UART CR: disabled at reset");
    /* Enable UART with TXE + RXE */
    mem_write32(UART0_BASE + UART_CR, UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE);
    uint32_t cr = mem_read32(UART0_BASE + 0x030);
    ASSERT_TRUE(cr & 1, "UART CR: UARTEN");
    ASSERT_TRUE(cr & (1u << 8), "UART CR: TXE");
    PASS();
}

TEST(test_uart_baud_readback) {
    reset_cpu();
    mem_write32(UART0_BASE + UART_IBRD, 67);
    mem_write32(UART0_BASE + UART_FBRD, 52);
    ASSERT_EQ(67, mem_read32(UART0_BASE + UART_IBRD), "UART0 IBRD readback");
    ASSERT_EQ(52, mem_read32(UART0_BASE + UART_FBRD), "UART0 FBRD readback");
    PASS();
}

TEST(test_uart1_independent) {
    reset_cpu();
    /* Write to UART1 baud, check UART0 is unaffected */
    mem_write32(UART0_BASE + UART_IBRD, 100);
    mem_write32(UART1_BASE + UART_IBRD, 200);
    ASSERT_EQ(100, mem_read32(UART0_BASE + UART_IBRD), "UART0 IBRD unchanged");
    ASSERT_EQ(200, mem_read32(UART1_BASE + UART_IBRD), "UART1 IBRD independent");
    /* UART1 FR also works */
    ASSERT_EQ(0x00000090, mem_read32(UART1_BASE + UART_FR), "UART1 FR");
    PASS();
}

TEST(test_uart_cr_readback) {
    reset_cpu();
    /* Disable UART, verify readback */
    mem_write32(UART0_BASE + UART_CR, 0);
    ASSERT_EQ(0, mem_read32(UART0_BASE + UART_CR), "UART CR cleared");
    /* Re-enable */
    mem_write32(UART0_BASE + UART_CR, UART_CR_UARTEN | UART_CR_TXE);
    uint32_t cr = mem_read32(UART0_BASE + UART_CR);
    ASSERT_TRUE(cr & UART_CR_UARTEN, "UART CR re-enabled");
    ASSERT_TRUE(cr & UART_CR_TXE, "UART CR TXE set");
    PASS();
}

TEST(test_uart_imsc_icr) {
    reset_cpu();
    /* Enable UART so TX empty interrupt is asserted */
    mem_write32(UART0_BASE + UART_CR, UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE);
    /* Set TX interrupt mask */
    mem_write32(UART0_BASE + UART_IMSC, UART_INT_TX);
    ASSERT_EQ(UART_INT_TX, mem_read32(UART0_BASE + UART_IMSC), "IMSC TX set");
    /* RIS has TX set (FIFO empty), so MIS should show it */
    uint32_t mis = mem_read32(UART0_BASE + UART_MIS);
    ASSERT_TRUE(mis & UART_INT_TX, "MIS TX active");
    /* Clear TX interrupt */
    mem_write32(UART0_BASE + UART_ICR, UART_INT_TX);
    ASSERT_EQ(0, mem_read32(UART0_BASE + UART_RIS) & UART_INT_TX, "RIS TX cleared");
    ASSERT_EQ(0, mem_read32(UART0_BASE + UART_MIS), "MIS zero after clear");
    PASS();
}

TEST(test_uart_atomic_set_clr) {
    reset_cpu();
    /* Enable UART with TXE + RXE first */
    mem_write32(UART0_BASE + UART_CR, UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE);
    /* CLR alias: clear RXE (bit 9) */
    mem_write32(UART0_BASE + 0x3000 + UART_CR, UART_CR_RXE);
    uint32_t cr = mem_read32(UART0_BASE + UART_CR);
    ASSERT_TRUE(!(cr & UART_CR_RXE), "CLR alias cleared RXE");
    ASSERT_TRUE(cr & UART_CR_TXE, "CLR alias preserved TXE");
    /* SET alias: set RXE back */
    mem_write32(UART0_BASE + 0x2000 + UART_CR, UART_CR_RXE);
    cr = mem_read32(UART0_BASE + UART_CR);
    ASSERT_TRUE(cr & UART_CR_RXE, "SET alias restored RXE");
    PASS();
}

TEST(test_uart_periph_id) {
    reset_cpu();
    /* PL011 peripheral identification */
    ASSERT_EQ(0x11, mem_read32(UART0_BASE + 0xFE0), "PERIPHID0");
    ASSERT_EQ(0x10, mem_read32(UART0_BASE + 0xFE4), "PERIPHID1");
    ASSERT_EQ(0x34, mem_read32(UART0_BASE + 0xFE8), "PERIPHID2");
    ASSERT_EQ(0x00, mem_read32(UART0_BASE + 0xFEC), "PERIPHID3");
    PASS();
}

/* ========================================================================
 * UART Rx Tests (NEW - v0.11.0)
 * ======================================================================== */

TEST(test_uart_rx_push_pop) {
    reset_cpu();
    /* Push a byte and read it back via DR */
    int ok = uart_rx_push(0, 'H');
    ASSERT_EQ(1, ok, "uart_rx_push returns 1");
    uint32_t dr = mem_read32(UART0_BASE + UART_DR);
    ASSERT_EQ('H', dr & 0xFF, "DR read returns pushed byte");
    PASS();
}

TEST(test_uart_rx_fifo_empty_flag) {
    reset_cpu();
    /* Initially RX FIFO should be empty */
    uint32_t fr = mem_read32(UART0_BASE + UART_FR);
    ASSERT_TRUE(fr & UART_FR_RXFE, "RXFE set when empty");
    ASSERT_TRUE(!(fr & UART_FR_RXFF), "RXFF clear when empty");
    /* Push a byte - no longer empty */
    uart_rx_push(0, 'A');
    fr = mem_read32(UART0_BASE + UART_FR);
    ASSERT_TRUE(!(fr & UART_FR_RXFE), "RXFE clear after push");
    /* Pop it - empty again */
    mem_read32(UART0_BASE + UART_DR);
    fr = mem_read32(UART0_BASE + UART_FR);
    ASSERT_TRUE(fr & UART_FR_RXFE, "RXFE set after pop");
    PASS();
}

TEST(test_uart_rx_fifo_full_flag) {
    reset_cpu();
    /* Fill the FIFO */
    for (int i = 0; i < UART_RX_FIFO_SIZE; i++) {
        ASSERT_EQ(1, uart_rx_push(0, (uint8_t)i), "push succeeds");
    }
    /* Next push should fail */
    ASSERT_EQ(0, uart_rx_push(0, 0xFF), "push fails when full");
    /* RXFF should be set */
    uint32_t fr = mem_read32(UART0_BASE + UART_FR);
    ASSERT_TRUE(fr & UART_FR_RXFF, "RXFF set when full");
    PASS();
}

TEST(test_uart_rx_fifo_order) {
    reset_cpu();
    /* Push 3 bytes, read them back in FIFO order */
    uart_rx_push(0, 'X');
    uart_rx_push(0, 'Y');
    uart_rx_push(0, 'Z');
    ASSERT_EQ('X', mem_read32(UART0_BASE + UART_DR) & 0xFF, "FIFO order [0]");
    ASSERT_EQ('Y', mem_read32(UART0_BASE + UART_DR) & 0xFF, "FIFO order [1]");
    ASSERT_EQ('Z', mem_read32(UART0_BASE + UART_DR) & 0xFF, "FIFO order [2]");
    PASS();
}

TEST(test_uart_rx_interrupt) {
    reset_cpu();
    /* Default IFLS RX trigger = 1/2 full = 8 bytes */
    /* Push 7 bytes - below trigger, RX interrupt should not be set */
    for (int i = 0; i < 7; i++) {
        uart_rx_push(0, (uint8_t)('A' + i));
    }
    ASSERT_EQ(0, uart_state[0].ris & UART_INT_RX, "RX IRQ not set below trigger");
    /* Push 8th byte - at trigger level, RX interrupt should be set */
    uart_rx_push(0, 'H');
    ASSERT_TRUE(uart_state[0].ris & UART_INT_RX, "RX IRQ set at trigger level");
    PASS();
}

TEST(test_uart_rx_interrupt_clear) {
    reset_cpu();
    /* Fill to trigger level and verify interrupt */
    for (int i = 0; i < 8; i++) {
        uart_rx_push(0, (uint8_t)i);
    }
    ASSERT_TRUE(uart_state[0].ris & UART_INT_RX, "RX IRQ set");
    /* Read enough to go below trigger */
    for (int i = 0; i < 2; i++) {
        mem_read32(UART0_BASE + UART_DR);
    }
    ASSERT_EQ(0, uart_state[0].ris & UART_INT_RX, "RX IRQ cleared after reads");
    PASS();
}

TEST(test_uart1_rx_independent) {
    reset_cpu();
    /* Push to UART1, read from UART1 */
    uart_rx_push(1, 'Q');
    uint32_t dr = mem_read32(UART1_BASE + UART_DR);
    ASSERT_EQ('Q', dr & 0xFF, "UART1 RX returns pushed byte");
    /* UART0 should still be empty */
    uint32_t fr0 = mem_read32(UART0_BASE + UART_FR);
    ASSERT_TRUE(fr0 & UART_FR_RXFE, "UART0 RX FIFO still empty");
    PASS();
}

TEST(test_uart_rx_masked_interrupt) {
    reset_cpu();
    /* Enable RX interrupt mask */
    mem_write32(UART0_BASE + UART_IMSC, UART_INT_RX);
    /* Fill to trigger level */
    for (int i = 0; i < 8; i++) {
        uart_rx_push(0, (uint8_t)i);
    }
    /* MIS should show RX interrupt */
    uint32_t mis = mem_read32(UART0_BASE + UART_MIS);
    ASSERT_TRUE(mis & UART_INT_RX, "MIS shows RX when IMSC enabled");
    /* Disable mask, MIS should be clear */
    mem_write32(UART0_BASE + UART_IMSC, 0);
    mis = mem_read32(UART0_BASE + UART_MIS);
    ASSERT_EQ(0, mis & UART_INT_RX, "MIS clear when IMSC disabled");
    PASS();
}

/* ========================================================================
 * SPI Tests
 * ======================================================================== */

TEST(test_spi0_status) {
    reset_cpu();
    uint32_t sr = mem_read32(SPI0_BASE + SPI_SSPSR);
    ASSERT_TRUE(sr & SPI_SSPSR_TFE, "SPI0 TX FIFO empty");
    ASSERT_TRUE(sr & SPI_SSPSR_TNF, "SPI0 TX not full");
    ASSERT_TRUE(!(sr & SPI_SSPSR_BSY), "SPI0 not busy");
    PASS();
}

TEST(test_spi_cr0_readback) {
    reset_cpu();
    mem_write32(SPI0_BASE + SPI_SSPCR0, 0x0007);  /* 8-bit, SPI mode */
    ASSERT_EQ(0x0007, mem_read32(SPI0_BASE + SPI_SSPCR0), "SPI0 CR0 readback");
    PASS();
}

TEST(test_spi1_independent) {
    reset_cpu();
    mem_write32(SPI0_BASE + SPI_SSPCPSR, 2);
    mem_write32(SPI1_BASE + SPI_SSPCPSR, 64);
    ASSERT_EQ(2,  mem_read32(SPI0_BASE + SPI_SSPCPSR), "SPI0 CPSR");
    ASSERT_EQ(64, mem_read32(SPI1_BASE + SPI_SSPCPSR), "SPI1 CPSR independent");
    PASS();
}

TEST(test_spi_periph_id) {
    reset_cpu();
    ASSERT_EQ(0x22, mem_read32(SPI0_BASE + SPI_PERIPHID0), "SPI PERIPHID0 (PL022)");
    PASS();
}

/* ========================================================================
 * I2C Tests
 * ======================================================================== */

TEST(test_i2c0_status) {
    reset_cpu();
    uint32_t st = mem_read32(I2C0_BASE + I2C_STATUS);
    ASSERT_TRUE(st & I2C_STATUS_TFE, "I2C0 TX FIFO empty");
    ASSERT_TRUE(st & I2C_STATUS_TFNF, "I2C0 TX not full");
    ASSERT_TRUE(!(st & I2C_STATUS_ACTIVITY), "I2C0 not active");
    PASS();
}

TEST(test_i2c_tar_readback) {
    reset_cpu();
    mem_write32(I2C0_BASE + I2C_TAR, 0x50);
    ASSERT_EQ(0x50, mem_read32(I2C0_BASE + I2C_TAR), "I2C0 TAR readback");
    PASS();
}

TEST(test_i2c1_independent) {
    reset_cpu();
    mem_write32(I2C0_BASE + I2C_TAR, 0x10);
    mem_write32(I2C1_BASE + I2C_TAR, 0x20);
    ASSERT_EQ(0x10, mem_read32(I2C0_BASE + I2C_TAR), "I2C0 TAR");
    ASSERT_EQ(0x20, mem_read32(I2C1_BASE + I2C_TAR), "I2C1 TAR independent");
    PASS();
}

TEST(test_i2c_enable_disable) {
    reset_cpu();
    ASSERT_EQ(0, mem_read32(I2C0_BASE + I2C_ENABLE), "I2C0 disabled at init");
    mem_write32(I2C0_BASE + I2C_ENABLE, 1);
    ASSERT_EQ(1, mem_read32(I2C0_BASE + I2C_ENABLE), "I2C0 enabled");
    PASS();
}

TEST(test_i2c_comp_type) {
    reset_cpu();
    ASSERT_EQ(0x44570140, mem_read32(I2C0_BASE + I2C_COMP_TYPE), "I2C COMP_TYPE");
    PASS();
}

/* ========================================================================
 * PWM Tests
 * ======================================================================== */

TEST(test_pwm_slice_defaults) {
    reset_cpu();
    ASSERT_EQ(0xFFFF, mem_read32(PWM_BASE + PWM_CH_TOP), "PWM slice 0 TOP default");
    ASSERT_EQ(0x10, mem_read32(PWM_BASE + PWM_CH_DIV), "PWM slice 0 DIV default (1.0)");
    ASSERT_EQ(0, mem_read32(PWM_BASE + PWM_CH_CSR), "PWM slice 0 CSR default");
    PASS();
}

TEST(test_pwm_slice_readback) {
    reset_cpu();
    /* Write slice 0 registers */
    mem_write32(PWM_BASE + PWM_CH_TOP, 1000);
    mem_write32(PWM_BASE + PWM_CH_CC, 0x01F40064);  /* A=500, B=100 */
    ASSERT_EQ(1000, mem_read32(PWM_BASE + PWM_CH_TOP), "PWM TOP readback");
    ASSERT_EQ(0x01F40064, mem_read32(PWM_BASE + PWM_CH_CC), "PWM CC readback");
    PASS();
}

TEST(test_pwm_multiple_slices) {
    reset_cpu();
    /* Slice 0 at offset 0x00, slice 3 at offset 0x3C (3 * 0x14) */
    mem_write32(PWM_BASE + 0x00 + PWM_CH_TOP, 999);
    mem_write32(PWM_BASE + 0x3C + PWM_CH_TOP, 1999);
    ASSERT_EQ(999,  mem_read32(PWM_BASE + 0x00 + PWM_CH_TOP), "PWM slice 0 TOP");
    ASSERT_EQ(1999, mem_read32(PWM_BASE + 0x3C + PWM_CH_TOP), "PWM slice 3 TOP");
    PASS();
}

TEST(test_pwm_global_enable) {
    reset_cpu();
    mem_write32(PWM_BASE + PWM_EN, 0x05);  /* Enable slices 0 and 2 */
    ASSERT_EQ(0x05, mem_read32(PWM_BASE + PWM_EN), "PWM EN readback");
    PASS();
}

/* ========================================================================
 * DMA Controller Tests (NEW - v0.10.0)
 * ======================================================================== */

TEST(test_dma_n_channels) {
    reset_cpu();
    ASSERT_EQ(DMA_NUM_CHANNELS, mem_read32(DMA_BASE + DMA_N_CHANNELS), "DMA N_CHANNELS");
    PASS();
}

TEST(test_dma_channel_defaults) {
    reset_cpu();
    /* All channel registers should be zero except CHAIN_TO=self */
    ASSERT_EQ(0, dma_state.ch[0].read_addr, "DMA ch0 read_addr default");
    ASSERT_EQ(0, dma_state.ch[0].write_addr, "DMA ch0 write_addr default");
    ASSERT_EQ(0, dma_state.ch[0].trans_count, "DMA ch0 trans_count default");
    /* CHAIN_TO field (bits 14:11) should be channel number (0 for ch0) */
    uint32_t chain_to = (dma_state.ch[0].ctrl & DMA_CTRL_CHAIN_TO_MASK) >> DMA_CTRL_CHAIN_TO_SHIFT;
    ASSERT_EQ(0, chain_to, "DMA ch0 CHAIN_TO defaults to self");
    chain_to = (dma_state.ch[5].ctrl & DMA_CTRL_CHAIN_TO_MASK) >> DMA_CTRL_CHAIN_TO_SHIFT;
    ASSERT_EQ(5, chain_to, "DMA ch5 CHAIN_TO defaults to self");
    PASS();
}

TEST(test_dma_register_readback) {
    reset_cpu();
    /* Write channel 0 registers via direct addresses (alias 0) */
    mem_write32(DMA_BASE + 0 * DMA_CH_STRIDE + DMA_CH_READ_ADDR, 0x20000100);
    mem_write32(DMA_BASE + 0 * DMA_CH_STRIDE + DMA_CH_WRITE_ADDR, 0x20000200);
    mem_write32(DMA_BASE + 0 * DMA_CH_STRIDE + DMA_CH_TRANS_COUNT, 16);
    ASSERT_EQ(0x20000100, mem_read32(DMA_BASE + DMA_CH_READ_ADDR), "DMA ch0 READ_ADDR readback");
    ASSERT_EQ(0x20000200, mem_read32(DMA_BASE + DMA_CH_WRITE_ADDR), "DMA ch0 WRITE_ADDR readback");
    ASSERT_EQ(16, mem_read32(DMA_BASE + DMA_CH_TRANS_COUNT), "DMA ch0 TRANS_COUNT readback");
    PASS();
}

TEST(test_dma_word_transfer) {
    reset_cpu();
    /* Write 4 words to RAM source region */
    uint32_t src = RAM_BASE + 0x1000;
    uint32_t dst = RAM_BASE + 0x2000;
    mem_write32(src + 0, 0xDEADBEEF);
    mem_write32(src + 4, 0xCAFEBABE);
    mem_write32(src + 8, 0x12345678);
    mem_write32(src + 12, 0xAAAABBBB);

    /* Configure DMA channel 0: word transfer, incr both, 4 words */
    mem_write32(DMA_BASE + DMA_CH_READ_ADDR, src);
    mem_write32(DMA_BASE + DMA_CH_WRITE_ADDR, dst);
    mem_write32(DMA_BASE + DMA_CH_TRANS_COUNT, 4);
    /* CTRL_TRIG: EN=1, DATA_SIZE=2 (word), INCR_READ=1, INCR_WRITE=1
     * Writing CTRL_TRIG triggers the transfer */
    uint32_t ctrl = DMA_CTRL_EN | (DMA_SIZE_WORD << DMA_CTRL_DATA_SIZE_SHIFT)
                  | DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE;
    mem_write32(DMA_BASE + DMA_CH_CTRL_TRIG, ctrl);

    /* Verify destination */
    ASSERT_EQ(0xDEADBEEF, mem_read32(dst + 0),  "DMA word copy [0]");
    ASSERT_EQ(0xCAFEBABE, mem_read32(dst + 4),  "DMA word copy [1]");
    ASSERT_EQ(0x12345678, mem_read32(dst + 8),  "DMA word copy [2]");
    ASSERT_EQ(0xAAAABBBB, mem_read32(dst + 12), "DMA word copy [3]");
    /* trans_count should be 0 after completion */
    ASSERT_EQ(0, dma_state.ch[0].trans_count, "DMA ch0 trans_count after transfer");
    PASS();
}

TEST(test_dma_byte_transfer) {
    reset_cpu();
    uint32_t src = RAM_BASE + 0x3000;
    uint32_t dst = RAM_BASE + 0x4000;
    mem_write8(src + 0, 0x41);  /* 'A' */
    mem_write8(src + 1, 0x42);  /* 'B' */
    mem_write8(src + 2, 0x43);  /* 'C' */

    mem_write32(DMA_BASE + DMA_CH_READ_ADDR, src);
    mem_write32(DMA_BASE + DMA_CH_WRITE_ADDR, dst);
    mem_write32(DMA_BASE + DMA_CH_TRANS_COUNT, 3);
    uint32_t ctrl = DMA_CTRL_EN | (DMA_SIZE_BYTE << DMA_CTRL_DATA_SIZE_SHIFT)
                  | DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE;
    mem_write32(DMA_BASE + DMA_CH_CTRL_TRIG, ctrl);

    ASSERT_EQ(0x41, mem_read8(dst + 0), "DMA byte copy [0]");
    ASSERT_EQ(0x42, mem_read8(dst + 1), "DMA byte copy [1]");
    ASSERT_EQ(0x43, mem_read8(dst + 2), "DMA byte copy [2]");
    PASS();
}

TEST(test_dma_no_incr_write) {
    reset_cpu();
    /* Transfer 3 words to same destination (no INCR_WRITE) - last value wins */
    uint32_t src = RAM_BASE + 0x5000;
    uint32_t dst = RAM_BASE + 0x6000;
    mem_write32(src + 0, 0x11111111);
    mem_write32(src + 4, 0x22222222);
    mem_write32(src + 8, 0x33333333);

    mem_write32(DMA_BASE + DMA_CH_READ_ADDR, src);
    mem_write32(DMA_BASE + DMA_CH_WRITE_ADDR, dst);
    mem_write32(DMA_BASE + DMA_CH_TRANS_COUNT, 3);
    uint32_t ctrl = DMA_CTRL_EN | (DMA_SIZE_WORD << DMA_CTRL_DATA_SIZE_SHIFT)
                  | DMA_CTRL_INCR_READ;  /* no INCR_WRITE */
    mem_write32(DMA_BASE + DMA_CH_CTRL_TRIG, ctrl);

    /* Only last word written to dst */
    ASSERT_EQ(0x33333333, mem_read32(dst), "DMA no-incr-write: last value at dst");
    PASS();
}

TEST(test_dma_interrupt_on_completion) {
    reset_cpu();
    uint32_t src = RAM_BASE + 0x7000;
    uint32_t dst = RAM_BASE + 0x8000;
    mem_write32(src, 0xFEEDFACE);

    /* Channel 2, 1-word transfer */
    uint32_t ch2 = 2 * DMA_CH_STRIDE;
    mem_write32(DMA_BASE + ch2 + DMA_CH_READ_ADDR, src);
    mem_write32(DMA_BASE + ch2 + DMA_CH_WRITE_ADDR, dst);
    mem_write32(DMA_BASE + ch2 + DMA_CH_TRANS_COUNT, 1);
    uint32_t ctrl = DMA_CTRL_EN | (DMA_SIZE_WORD << DMA_CTRL_DATA_SIZE_SHIFT)
                  | DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE;
    mem_write32(DMA_BASE + ch2 + DMA_CH_CTRL_TRIG, ctrl);

    /* INTR bit 2 should be set */
    ASSERT_TRUE(dma_state.intr & (1 << 2), "DMA INTR bit set for ch2");
    /* Clear it via W1C */
    mem_write32(DMA_BASE + DMA_INTR, (1 << 2));
    ASSERT_EQ(0, dma_state.intr & (1 << 2), "DMA INTR bit cleared via W1C");
    PASS();
}

TEST(test_dma_irq_quiet) {
    reset_cpu();
    uint32_t src = RAM_BASE + 0x9000;
    uint32_t dst = RAM_BASE + 0xA000;
    mem_write32(src, 0x12345678);

    /* Channel 3 with IRQ_QUIET */
    uint32_t ch3 = 3 * DMA_CH_STRIDE;
    mem_write32(DMA_BASE + ch3 + DMA_CH_READ_ADDR, src);
    mem_write32(DMA_BASE + ch3 + DMA_CH_WRITE_ADDR, dst);
    mem_write32(DMA_BASE + ch3 + DMA_CH_TRANS_COUNT, 1);
    uint32_t ctrl = DMA_CTRL_EN | (DMA_SIZE_WORD << DMA_CTRL_DATA_SIZE_SHIFT)
                  | DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE | DMA_CTRL_IRQ_QUIET;
    mem_write32(DMA_BASE + ch3 + DMA_CH_CTRL_TRIG, ctrl);

    /* INTR bit 3 should NOT be set */
    ASSERT_EQ(0, dma_state.intr & (1 << 3), "DMA IRQ_QUIET suppresses INTR");
    /* But transfer should still complete */
    ASSERT_EQ(0x12345678, mem_read32(dst), "DMA IRQ_QUIET transfer completes");
    PASS();
}

TEST(test_dma_interrupt_status) {
    reset_cpu();
    /* Set up INTE0 and force INTF0 to test INTS0 computation */
    mem_write32(DMA_BASE + DMA_INTE0, 0x00F);  /* Enable ch0-3 */
    mem_write32(DMA_BASE + DMA_INTF0, 0x004);  /* Force ch2 */
    uint32_t ints0 = mem_read32(DMA_BASE + DMA_INTS0);
    /* INTS0 = (INTR | INTF0) & INTE0 = (0 | 0x004) & 0x00F = 0x004 */
    ASSERT_EQ(0x004, ints0, "DMA INTS0 = (INTR|INTF0)&INTE0");
    PASS();
}

TEST(test_dma_chain_transfer) {
    reset_cpu();
    uint32_t src0 = RAM_BASE + 0xB000;
    uint32_t dst0 = RAM_BASE + 0xC000;
    uint32_t src1 = RAM_BASE + 0xD000;
    uint32_t dst1 = RAM_BASE + 0xE000;

    mem_write32(src0, 0xAAAAAAAA);
    mem_write32(src1, 0xBBBBBBBB);

    /* Set up channel 1 first (target of chain) */
    uint32_t ch1 = 1 * DMA_CH_STRIDE;
    mem_write32(DMA_BASE + ch1 + DMA_CH_READ_ADDR, src1);
    mem_write32(DMA_BASE + ch1 + DMA_CH_WRITE_ADDR, dst1);
    mem_write32(DMA_BASE + ch1 + DMA_CH_TRANS_COUNT, 1);
    /* Ch1: EN, word, incr both, CHAIN_TO=self (1) */
    uint32_t ctrl1 = DMA_CTRL_EN | (DMA_SIZE_WORD << DMA_CTRL_DATA_SIZE_SHIFT)
                   | DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE
                   | (1 << DMA_CTRL_CHAIN_TO_SHIFT);
    /* Write CTRL without trigger (use alias 1: CTRL is first reg, not trigger) */
    mem_write32(DMA_BASE + ch1 + DMA_CH_AL1_CTRL, ctrl1);

    /* Set up channel 0 with CHAIN_TO=1 */
    uint32_t ch0 = 0 * DMA_CH_STRIDE;
    mem_write32(DMA_BASE + ch0 + DMA_CH_READ_ADDR, src0);
    mem_write32(DMA_BASE + ch0 + DMA_CH_WRITE_ADDR, dst0);
    mem_write32(DMA_BASE + ch0 + DMA_CH_TRANS_COUNT, 1);
    uint32_t ctrl0 = DMA_CTRL_EN | (DMA_SIZE_WORD << DMA_CTRL_DATA_SIZE_SHIFT)
                   | DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE
                   | (1 << DMA_CTRL_CHAIN_TO_SHIFT);  /* CHAIN_TO = ch1 */
    mem_write32(DMA_BASE + ch0 + DMA_CH_CTRL_TRIG, ctrl0);  /* Trigger ch0 */

    /* Both transfers should have completed */
    ASSERT_EQ(0xAAAAAAAA, mem_read32(dst0), "DMA chain: ch0 dst");
    ASSERT_EQ(0xBBBBBBBB, mem_read32(dst1), "DMA chain: ch1 dst");
    PASS();
}

TEST(test_dma_multi_chan_trigger) {
    reset_cpu();
    uint32_t src4 = RAM_BASE + 0xF000;
    uint32_t dst4 = RAM_BASE + 0x10000;
    mem_write32(src4, 0x44444444);

    /* Configure ch4 but don't trigger */
    uint32_t ch4 = 4 * DMA_CH_STRIDE;
    mem_write32(DMA_BASE + ch4 + DMA_CH_READ_ADDR, src4);
    mem_write32(DMA_BASE + ch4 + DMA_CH_WRITE_ADDR, dst4);
    mem_write32(DMA_BASE + ch4 + DMA_CH_TRANS_COUNT, 1);
    uint32_t ctrl = DMA_CTRL_EN | (DMA_SIZE_WORD << DMA_CTRL_DATA_SIZE_SHIFT)
                  | DMA_CTRL_INCR_READ | DMA_CTRL_INCR_WRITE
                  | (4 << DMA_CTRL_CHAIN_TO_SHIFT);
    /* Write via AL1_CTRL (not a trigger register) */
    mem_write32(DMA_BASE + ch4 + DMA_CH_AL1_CTRL, ctrl);

    /* Trigger via MULTI_CHAN_TRIGGER */
    mem_write32(DMA_BASE + DMA_MULTI_CHAN_TRIGGER, (1 << 4));
    ASSERT_EQ(0x44444444, mem_read32(dst4), "DMA MULTI_CHAN_TRIGGER ch4");
    PASS();
}

TEST(test_dma_atomic_set_clr) {
    reset_cpu();
    /* Write INTE0 normally, then use SET alias to add bits */
    mem_write32(DMA_BASE + DMA_INTE0, 0x003);
    ASSERT_EQ(0x003, mem_read32(DMA_BASE + DMA_INTE0), "DMA INTE0 initial");
    /* SET alias: DMA_BASE + 0x2000 + offset */
    mem_write32(DMA_BASE + 0x2000 + DMA_INTE0, 0x00C);
    ASSERT_EQ(0x00F, mem_read32(DMA_BASE + DMA_INTE0), "DMA INTE0 after SET");
    /* CLR alias */
    mem_write32(DMA_BASE + 0x3000 + DMA_INTE0, 0x005);
    ASSERT_EQ(0x00A, mem_read32(DMA_BASE + DMA_INTE0), "DMA INTE0 after CLR");
    PASS();
}

/* ========================================================================
 * PIO Tests (v0.12.0)
 * ======================================================================== */

TEST(test_pio_fstat_fifos_empty) {
    reset_cpu();
    uint32_t fstat = mem_read32(PIO0_BASE + PIO_FSTAT);
    /* All TX empty (bits 27:24), all RX empty (bits 11:8) */
    ASSERT_TRUE(fstat & (0x0F << PIO_FSTAT_TXEMPTY_SHIFT), "TX FIFOs empty");
    ASSERT_TRUE(fstat & (0x0F << PIO_FSTAT_RXEMPTY_SHIFT), "RX FIFOs empty");
    PASS();
}

TEST(test_pio_instr_mem_readback) {
    reset_cpu();
    /* Write a PIO instruction to memory slot 0 */
    mem_write32(PIO0_BASE + PIO_INSTR_MEM0, 0xE040);  /* SET PINS, 0 */
    ASSERT_EQ(0xE040, mem_read32(PIO0_BASE + PIO_INSTR_MEM0), "PIO instr_mem[0] readback");
    /* Write to slot 5 */
    mem_write32(PIO0_BASE + PIO_INSTR_MEM0 + 5 * 4, 0x8020);
    ASSERT_EQ(0x8020, mem_read32(PIO0_BASE + PIO_INSTR_MEM0 + 5 * 4), "PIO instr_mem[5] readback");
    /* 16-bit mask: upper bits should be stripped */
    mem_write32(PIO0_BASE + PIO_INSTR_MEM0 + 1 * 4, 0xFFFFE001);
    ASSERT_EQ(0xE001, mem_read32(PIO0_BASE + PIO_INSTR_MEM0 + 1 * 4), "PIO instr 16-bit mask");
    PASS();
}

TEST(test_pio_sm_register_readback) {
    reset_cpu();
    /* Write SM0 CLKDIV */
    mem_write32(PIO0_BASE + PIO_SM0_CLKDIV, 0x01000000);
    ASSERT_EQ(0x01000000, mem_read32(PIO0_BASE + PIO_SM0_CLKDIV), "SM0 CLKDIV readback");
    /* Write SM0 PINCTRL */
    mem_write32(PIO0_BASE + PIO_SM0_PINCTRL, 0x04000500);
    ASSERT_EQ(0x04000500, mem_read32(PIO0_BASE + PIO_SM0_PINCTRL), "SM0 PINCTRL readback");
    /* Write SM2 CLKDIV (stride 0x18, SM2 offset = 0x0C8 + 2*0x18 = 0x0F8) */
    mem_write32(PIO0_BASE + PIO_SM0_CLKDIV + 2 * PIO_SM_STRIDE, 0x02000000);
    ASSERT_EQ(0x02000000, mem_read32(PIO0_BASE + PIO_SM0_CLKDIV + 2 * PIO_SM_STRIDE), "SM2 CLKDIV readback");
    PASS();
}

TEST(test_pio_ctrl_enable) {
    reset_cpu();
    ASSERT_EQ(0, mem_read32(PIO0_BASE + PIO_CTRL), "PIO CTRL starts at 0");
    /* Enable SM0 and SM2 */
    mem_write32(PIO0_BASE + PIO_CTRL, 0x05);
    ASSERT_EQ(0x05, mem_read32(PIO0_BASE + PIO_CTRL), "PIO CTRL SM0+SM2 enabled");
    PASS();
}

TEST(test_pio_dbg_cfginfo) {
    reset_cpu();
    uint32_t cfginfo = mem_read32(PIO0_BASE + PIO_DBG_CFGINFO);
    /* RP2040: 4 FIFO depth (bits 21:16), 32 instr mem (bits 13:8), 4 SMs (bits 3:0) */
    uint32_t fifo_depth = (cfginfo >> 16) & 0x3F;
    uint32_t imem_size = (cfginfo >> 8) & 0x3F;
    uint32_t n_sm = cfginfo & 0x0F;
    ASSERT_EQ(4, fifo_depth, "DBG_CFGINFO FIFO depth");
    ASSERT_EQ(PIO_INSTR_MEM_SIZE, imem_size, "DBG_CFGINFO instr mem size");
    ASSERT_EQ(PIO_NUM_SM, n_sm, "DBG_CFGINFO num SMs");
    PASS();
}

TEST(test_pio1_independent) {
    reset_cpu();
    /* Write to PIO0 instr_mem[0] */
    mem_write32(PIO0_BASE + PIO_INSTR_MEM0, 0x1234);
    /* Write to PIO1 instr_mem[0] */
    mem_write32(PIO1_BASE + PIO_INSTR_MEM0, 0x5678);
    ASSERT_EQ(0x1234, mem_read32(PIO0_BASE + PIO_INSTR_MEM0), "PIO0 instr independent");
    ASSERT_EQ(0x5678, mem_read32(PIO1_BASE + PIO_INSTR_MEM0), "PIO1 instr independent");
    /* PIO1 CTRL independent */
    mem_write32(PIO1_BASE + PIO_CTRL, 0x0F);
    ASSERT_EQ(0, mem_read32(PIO0_BASE + PIO_CTRL), "PIO0 CTRL unaffected");
    ASSERT_EQ(0x0F, mem_read32(PIO1_BASE + PIO_CTRL), "PIO1 CTRL set");
    PASS();
}

TEST(test_pio_irq_write_clear) {
    reset_cpu();
    /* Force IRQ bits */
    mem_write32(PIO0_BASE + PIO_IRQ_FORCE, 0x05);
    ASSERT_EQ(0x05, mem_read32(PIO0_BASE + PIO_IRQ), "IRQ set by force");
    /* Write-1-to-clear IRQ */
    mem_write32(PIO0_BASE + PIO_IRQ, 0x01);
    ASSERT_EQ(0x04, mem_read32(PIO0_BASE + PIO_IRQ), "IRQ after W1C");
    PASS();
}

TEST(test_pio_tx_rx_fifo_stubs) {
    reset_cpu();
    /* TX writes accepted silently, RX reads return 0 */
    mem_write32(PIO0_BASE + PIO_TXF0, 0xDEADBEEF);
    mem_write32(PIO0_BASE + PIO_TXF3, 0xCAFEBABE);
    ASSERT_EQ(0, mem_read32(PIO0_BASE + PIO_RXF0), "RXF0 returns 0");
    ASSERT_EQ(0, mem_read32(PIO0_BASE + PIO_RXF3), "RXF3 returns 0");
    PASS();
}

TEST(test_pio_atomic_set_clr) {
    reset_cpu();
    /* Write IRQ0_INTE normally */
    mem_write32(PIO0_BASE + PIO_IRQ0_INTE, 0x003);
    ASSERT_EQ(0x003, mem_read32(PIO0_BASE + PIO_IRQ0_INTE), "PIO IRQ0_INTE initial");
    /* SET alias */
    mem_write32(PIO0_BASE + 0x2000 + PIO_IRQ0_INTE, 0x00C);
    ASSERT_EQ(0x00F, mem_read32(PIO0_BASE + PIO_IRQ0_INTE), "PIO IRQ0_INTE after SET");
    /* CLR alias */
    mem_write32(PIO0_BASE + 0x3000 + PIO_IRQ0_INTE, 0x005);
    ASSERT_EQ(0x00A, mem_read32(PIO0_BASE + PIO_IRQ0_INTE), "PIO IRQ0_INTE after CLR");
    PASS();
}

/* ========================================================================
 * SRAM Alias Tests (v0.13.0)
 * ======================================================================== */

TEST(test_sram_alias_write_read) {
    reset_cpu();
    /* Write via normal SRAM address, read via alias */
    mem_write32(RAM_BASE + 0x100, 0xDEADBEEF);
    ASSERT_EQ(0xDEADBEEF, mem_read32(SRAM_ALIAS_BASE + 0x100), "SRAM alias read mirrors normal");
    PASS();
}

TEST(test_sram_alias_write_through) {
    reset_cpu();
    /* Write via alias, read via normal */
    mem_write32(SRAM_ALIAS_BASE + 0x200, 0xCAFEBABE);
    ASSERT_EQ(0xCAFEBABE, mem_read32(RAM_BASE + 0x200), "SRAM alias write mirrors to normal");
    PASS();
}

TEST(test_sram_alias_byte_halfword) {
    reset_cpu();
    /* Byte access via alias */
    mem_write8(SRAM_ALIAS_BASE + 0x300, 0x42);
    ASSERT_EQ(0x42, mem_read8(RAM_BASE + 0x300), "SRAM alias byte access");
    /* Halfword access via alias */
    mem_write16(SRAM_ALIAS_BASE + 0x304, 0x1234);
    ASSERT_EQ(0x1234, mem_read16(RAM_BASE + 0x304), "SRAM alias halfword access");
    PASS();
}

/* ========================================================================
 * XIP Cache Control Tests (v0.13.0)
 * ======================================================================== */

TEST(test_xip_ctrl_defaults) {
    reset_cpu();
    /* Default: EN=1, ERR_BADWRITE=1 */
    ASSERT_EQ(0x03, mem_read32(XIP_CTRL_BASE), "XIP CTRL default");
    PASS();
}

TEST(test_xip_stat_ready) {
    reset_cpu();
    /* STAT: FIFO_EMPTY=1 (bit 2), FLUSH_READY=1 (bit 1) */
    uint32_t stat = mem_read32(XIP_CTRL_BASE + 0x08);
    ASSERT_TRUE(stat & (1u << 2), "XIP STAT FIFO_EMPTY");
    ASSERT_TRUE(stat & (1u << 1), "XIP STAT FLUSH_READY");
    PASS();
}

TEST(test_xip_flush_strobe) {
    reset_cpu();
    /* Flush is strobe: write 1, reads back 0 */
    mem_write32(XIP_CTRL_BASE + 0x04, 1);
    ASSERT_EQ(0, mem_read32(XIP_CTRL_BASE + 0x04), "XIP FLUSH reads 0 (strobe)");
    PASS();
}

TEST(test_xip_counter_readback) {
    reset_cpu();
    mem_write32(XIP_CTRL_BASE + 0x0C, 100);  /* CTR_HIT */
    mem_write32(XIP_CTRL_BASE + 0x10, 200);  /* CTR_ACC */
    ASSERT_EQ(100, mem_read32(XIP_CTRL_BASE + 0x0C), "XIP CTR_HIT readback");
    ASSERT_EQ(200, mem_read32(XIP_CTRL_BASE + 0x10), "XIP CTR_ACC readback");
    PASS();
}

TEST(test_xip_sram_readwrite) {
    reset_cpu();
    /* XIP SRAM (cache as SRAM) at 0x15000000 */
    mem_write32(XIP_SRAM_BASE, 0x12345678);
    ASSERT_EQ(0x12345678, mem_read32(XIP_SRAM_BASE), "XIP SRAM word readback");
    mem_write8(XIP_SRAM_BASE + 4, 0xAB);
    ASSERT_EQ(0xAB, mem_read8(XIP_SRAM_BASE + 4), "XIP SRAM byte readback");
    PASS();
}

TEST(test_xip_flash_aliases) {
    reset_cpu();
    /* Write test data to flash via cpu.flash directly */
    uint32_t test_val = 0xBEEFCAFE;
    memcpy(&cpu.flash[0], &test_val, 4);
    /* All XIP aliases should return the same flash data */
    ASSERT_EQ(test_val, mem_read32(FLASH_BASE), "XIP normal read");
    ASSERT_EQ(test_val, mem_read32(XIP_NOALLOC_BASE), "XIP NOALLOC read");
    ASSERT_EQ(test_val, mem_read32(XIP_NOCACHE_BASE), "XIP NOCACHE read");
    ASSERT_EQ(test_val, mem_read32(XIP_NOCACHE_NOALLOC), "XIP NOCACHE_NOALLOC read");
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

TEST(test_ldmia_base_in_reglist) {
    /* ARMv6-M: when base register is in the register list, writeback is
     * NOT applied — the loaded value wins.  This is the pattern used by
     * Pico SDK boot2: LDM R0!, {R0, R1} to load SP and entry point. */
    reset_cpu();
    uint32_t addr = RAM_BASE + 0x400;
    mem_write32(addr + 0, 0xDEADBEEF);  /* value for R0 */
    mem_write32(addr + 4, 0x10001234);  /* value for R1 */
    cpu.r[0] = addr;
    /* 0xC803 = LDMIA R0!, {R0, R1} */
    instr_ldmia(0xC803);
    ASSERT_EQ(0xDEADBEEF, cpu.r[0], "LDMIA base-in-list: R0 gets loaded value, not writeback");
    ASSERT_EQ(0x10001234, cpu.r[1], "LDMIA base-in-list: R1 loaded correctly");
    PASS();
}

TEST(test_ldmia_base_not_in_reglist_writeback) {
    /* When base register is NOT in the list, writeback IS applied */
    reset_cpu();
    uint32_t addr = RAM_BASE + 0x400;
    mem_write32(addr + 0, 0xAAAAAAAA);
    mem_write32(addr + 4, 0xBBBBBBBB);
    cpu.r[2] = addr;
    /* 0xCA03 = LDMIA R2!, {R0, R1} */
    instr_ldmia(0xCA03);
    ASSERT_EQ(0xAAAAAAAA, cpu.r[0], "LDMIA writeback: R0 loaded");
    ASSERT_EQ(0xBBBBBBBB, cpu.r[1], "LDMIA writeback: R1 loaded");
    ASSERT_EQ(addr + 8, cpu.r[2], "LDMIA writeback: base advanced by 8");
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

TEST(test_usb_dpram_readback) {
    reset_cpu();
    /* DPRAM is real memory — writes should be readable */
    mem_write32(USBCTRL_DPRAM_BASE + 0x80, 0xCAFEBABE);
    ASSERT_EQ(0xCAFEBABE, mem_read32(USBCTRL_DPRAM_BASE + 0x80),
              "USB DPRAM should retain written data");
    PASS();
}

TEST(test_usb_sie_status_disconnected) {
    reset_cpu();
    /* SIE_STATUS should always return 0 (no VBUS, not connected) */
    ASSERT_EQ(0, mem_read32(USBCTRL_REGS_BASE + USB_SIE_STATUS),
              "SIE_STATUS should be 0 (disconnected)");
    /* Even after writing MAIN_CTRL to enable */
    mem_write32(USBCTRL_REGS_BASE + USB_MAIN_CTRL, 1);
    ASSERT_EQ(0, mem_read32(USBCTRL_REGS_BASE + USB_SIE_STATUS),
              "SIE_STATUS still 0 after enable");
    PASS();
}

TEST(test_usb_main_ctrl_readback) {
    reset_cpu();
    mem_write32(USBCTRL_REGS_BASE + USB_MAIN_CTRL, 0x01);
    ASSERT_EQ(0x01, mem_read32(USBCTRL_REGS_BASE + USB_MAIN_CTRL),
              "MAIN_CTRL should retain written value");
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
 * PIO Execution Tests
 * ======================================================================== */

/* Helper: encode PIO instruction */
static uint16_t pio_enc(uint8_t opcode, uint8_t delay_sideset, uint8_t arg) {
    return (uint16_t)((opcode << 13) | (delay_sideset << 8) | arg);
}

TEST(test_pio_set_x) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    /* SET X, 15 (opcode=7, dest=5 for X, data=15) */
    uint16_t instr = pio_enc(PIO_OP_SET, 0, (5 << 5) | 15);
    pio_sm_exec(0, 0, instr);
    ASSERT_EQ(15, s->x, "SET X, 15");
    PASS();
}

TEST(test_pio_set_y) {
    reset_cpu();
    pio_sm_t *s = &pio_state[0].sm[0];

    uint16_t instr = pio_enc(PIO_OP_SET, 0, (6 << 5) | 23);
    pio_sm_exec(0, 0, instr);
    ASSERT_EQ(23, s->y, "SET Y, 23");
    PASS();
}

TEST(test_pio_mov_x_to_y) {
    reset_cpu();
    pio_sm_t *s = &pio_state[0].sm[0];

    s->x = 0xDEADBEEF;
    /* MOV Y, X: opcode=5, dest=2(Y), op=0(none), source=1(X) */
    uint16_t instr = pio_enc(PIO_OP_MOV, 0, (2 << 5) | (0 << 3) | 1);
    pio_sm_exec(0, 0, instr);
    ASSERT_EQ(0xDEADBEEF, s->y, "MOV Y, X");
    PASS();
}

TEST(test_pio_mov_invert) {
    reset_cpu();
    pio_sm_t *s = &pio_state[0].sm[0];

    s->x = 0x00000000;
    /* MOV Y, ~X: opcode=5, dest=2(Y), op=1(invert), source=1(X) */
    uint16_t instr = pio_enc(PIO_OP_MOV, 0, (2 << 5) | (1 << 3) | 1);
    pio_sm_exec(0, 0, instr);
    ASSERT_EQ(0xFFFFFFFF, s->y, "MOV Y, ~X (invert)");
    PASS();
}

TEST(test_pio_jmp_always) {
    reset_cpu();
    pio_sm_t *s = &pio_state[0].sm[0];
    s->pc = 0;

    /* JMP 10: opcode=0, cond=0(always), addr=10 */
    uint16_t instr = pio_enc(PIO_OP_JMP, 0, (0 << 5) | 10);
    pio_sm_exec(0, 0, instr);
    ASSERT_EQ(10, s->pc, "JMP always to addr 10");
    PASS();
}

TEST(test_pio_jmp_x_zero) {
    reset_cpu();
    pio_sm_t *s = &pio_state[0].sm[0];
    s->pc = 0;
    s->x = 0;

    /* JMP !X, 5: opcode=0, cond=1(!X), addr=5 */
    uint16_t instr = pio_enc(PIO_OP_JMP, 0, (1 << 5) | 5);
    pio_sm_exec(0, 0, instr);
    ASSERT_EQ(5, s->pc, "JMP !X (X=0) should jump to 5");

    /* Now X != 0, should NOT jump */
    s->pc = 0;
    s->x = 42;
    s->execctrl = (31u << 12);  /* wrap_top=31 */
    pio_sm_exec(0, 0, instr);
    ASSERT_EQ(1, s->pc, "JMP !X (X!=0) should advance PC to 1");
    PASS();
}

TEST(test_pio_jmp_x_dec) {
    reset_cpu();
    pio_sm_t *s = &pio_state[0].sm[0];
    s->pc = 0;
    s->x = 3;
    s->execctrl = (31u << 12);

    /* JMP X--, 10: opcode=0, cond=2(X--), addr=10 */
    uint16_t instr = pio_enc(PIO_OP_JMP, 0, (2 << 5) | 10);
    pio_sm_exec(0, 0, instr);
    ASSERT_EQ(10, s->pc, "JMP X-- (X=3) should jump");
    ASSERT_EQ(2, s->x, "X should be decremented to 2");

    /* X=0: should not jump, X wraps to 0xFFFFFFFF */
    s->pc = 0;
    s->x = 0;
    pio_sm_exec(0, 0, instr);
    ASSERT_EQ(1, s->pc, "JMP X-- (X=0) should not jump");
    PASS();
}

TEST(test_pio_tx_fifo_push_pull) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    /* CPU pushes value into TX FIFO */
    pio_write32(0, PIO_TXF0, 0x12345678);
    ASSERT_EQ(1, s->tx_fifo.count, "TX FIFO should have 1 entry");

    /* PULL (non-blocking): opcode=4, arg = (1<<7) | (0<<6) | (0<<5) = PULL, no-ife, no-block */
    uint16_t pull = pio_enc(PIO_OP_PUSH_PULL, 0, (1 << 7) | (0 << 6) | (0 << 5));
    pio_sm_exec(0, 0, pull);
    ASSERT_EQ(0x12345678, s->osr, "PULL should load TX FIFO value into OSR");
    ASSERT_EQ(0, s->tx_fifo.count, "TX FIFO should be empty after PULL");
    PASS();
}

TEST(test_pio_rx_fifo_push_read) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    s->isr = 0xCAFEBABE;
    s->isr_count = 32;

    /* PUSH: opcode=4, arg = (0<<7) | (0<<6) | (0<<5) = PUSH, no-iff, no-block */
    uint16_t push = pio_enc(PIO_OP_PUSH_PULL, 0, (0 << 7) | (0 << 6) | (0 << 5));
    pio_sm_exec(0, 0, push);
    ASSERT_EQ(1, s->rx_fifo.count, "RX FIFO should have 1 entry after PUSH");
    ASSERT_EQ(0, s->isr, "ISR should be cleared after PUSH");

    /* CPU reads from RX FIFO */
    uint32_t val = pio_read32(0, PIO_RXF0);
    ASSERT_EQ(0xCAFEBABE, val, "RX FIFO read should return pushed value");
    ASSERT_EQ(0, s->rx_fifo.count, "RX FIFO should be empty after read");
    PASS();
}

TEST(test_pio_pull_blocking_stalls) {
    reset_cpu();
    pio_sm_t *s = &pio_state[0].sm[0];

    /* PULL blocking with empty TX FIFO */
    uint16_t pull_block = pio_enc(PIO_OP_PUSH_PULL, 0, (1 << 7) | (0 << 6) | (1 << 5));
    pio_sm_exec(0, 0, pull_block);
    ASSERT_EQ(1, s->stalled, "PULL blocking on empty FIFO should stall");
    PASS();
}

TEST(test_pio_fstat_reflects_fifo) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];

    /* Initially all FIFOs empty */
    uint32_t fstat = pio_read32(0, PIO_FSTAT);
    ASSERT_EQ(0x0F, (fstat >> PIO_FSTAT_TXEMPTY_SHIFT) & 0x0F, "All TX empty initially");
    ASSERT_EQ(0x0F, (fstat >> PIO_FSTAT_RXEMPTY_SHIFT) & 0x0F, "All RX empty initially");

    /* Push into SM0 TX FIFO */
    pio_write32(0, PIO_TXF0, 0x1234);
    fstat = pio_read32(0, PIO_FSTAT);
    ASSERT_EQ(0x0E, (fstat >> PIO_FSTAT_TXEMPTY_SHIFT) & 0x0F, "SM0 TX no longer empty");

    /* Push ISR into SM0 RX FIFO */
    p->sm[0].isr = 0xABCD;
    p->sm[0].isr_count = 32;
    uint16_t push = pio_enc(PIO_OP_PUSH_PULL, 0, 0);
    pio_sm_exec(0, 0, push);
    fstat = pio_read32(0, PIO_FSTAT);
    ASSERT_EQ(0x0E, (fstat >> PIO_FSTAT_RXEMPTY_SHIFT) & 0x0F, "SM0 RX no longer empty");
    PASS();
}

TEST(test_pio_wrap) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    /* Set wrap: bottom=2, top=5 */
    s->execctrl = (5u << 12) | (2u << 7);
    s->pc = 5;

    /* Execute NOP (SET X, 0) to trigger PC advance with wrap */
    uint16_t nop = pio_enc(PIO_OP_SET, 0, (5 << 5) | 0);
    pio_sm_exec(0, 0, nop);
    ASSERT_EQ(2, s->pc, "PC should wrap from 5 back to 2");
    PASS();
}

TEST(test_pio_out_x) {
    reset_cpu();
    pio_sm_t *s = &pio_state[0].sm[0];

    s->osr = 0xFF;
    s->osr_count = 0;
    /* Default SHIFTCTRL: shift right */
    s->shiftctrl = (1u << 19);  /* OUT shift right */

    /* OUT X, 8: opcode=3, dest=1(X), bit_count=8 */
    uint16_t instr = pio_enc(PIO_OP_OUT, 0, (1 << 5) | 8);
    pio_sm_exec(0, 0, instr);
    ASSERT_EQ(0xFF, s->x, "OUT X, 8 should put lower 8 bits of OSR into X");
    PASS();
}

TEST(test_pio_in_x_push) {
    reset_cpu();
    pio_sm_t *s = &pio_state[0].sm[0];

    s->x = 0xAB;
    s->isr = 0;
    s->isr_count = 0;
    s->shiftctrl = (1u << 18);  /* IN shift right */

    /* IN X, 8: opcode=2, source=1(X), bit_count=8 */
    uint16_t instr = pio_enc(PIO_OP_IN, 0, (1 << 5) | 8);
    pio_sm_exec(0, 0, instr);
    ASSERT_EQ(8, s->isr_count, "ISR count should be 8 after IN X,8");
    ASSERT_TRUE(s->isr != 0, "ISR should contain data after IN");
    PASS();
}

TEST(test_pio_irq_set_clear) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];

    /* IRQ SET 3: opcode=6, clr=0, wait=0, index=3 */
    uint16_t set_irq = pio_enc(PIO_OP_IRQ, 0, (0 << 6) | (0 << 5) | 3);
    pio_sm_exec(0, 0, set_irq);
    ASSERT_EQ(0x08, p->irq & 0x08, "IRQ 3 should be set");

    /* IRQ CLEAR 3: opcode=6, clr=1, wait=0, index=3 */
    uint16_t clr_irq = pio_enc(PIO_OP_IRQ, 0, (1 << 6) | (0 << 5) | 3);
    pio_sm_exec(0, 0, clr_irq);
    ASSERT_EQ(0, p->irq & 0x08, "IRQ 3 should be cleared");
    PASS();
}

TEST(test_pio_sm_enable_step) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    /* Load a simple program: SET X, 7 at addr 0 */
    p->instr_mem[0] = pio_enc(PIO_OP_SET, 0, (5 << 5) | 7);
    /* SET Y, 3 at addr 1 */
    p->instr_mem[1] = pio_enc(PIO_OP_SET, 0, (6 << 5) | 3);

    s->pc = 0;
    s->execctrl = (31u << 12);  /* wrap_top=31 */

    /* Enable SM0 */
    pio_write32(0, PIO_CTRL, 0x01);

    /* Step PIO */
    pio_step();
    ASSERT_EQ(7, s->x, "After step, X should be 7 (from SET X, 7)");
    ASSERT_EQ(1, s->pc, "PC should advance to 1");

    pio_step();
    ASSERT_EQ(3, s->y, "After second step, Y should be 3 (from SET Y, 3)");
    PASS();
}

TEST(test_pio_sm_restart_clears_state) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    s->x = 42;
    s->y = 99;
    s->pc = 15;
    s->isr = 0xFFFF;

    /* Write CTRL with SM_RESTART bit for SM0 (bit 4) */
    pio_write32(0, PIO_CTRL, (1u << 4));

    ASSERT_EQ(0, s->x, "X should be 0 after restart");
    ASSERT_EQ(0, s->y, "Y should be 0 after restart");
    ASSERT_EQ(0, s->pc, "PC should be 0 after restart");
    ASSERT_EQ(0, s->isr, "ISR should be 0 after restart");
    PASS();
}

TEST(test_pio_flevel_reflects_fifo) {
    reset_cpu();

    /* Push 2 items into SM0 TX FIFO */
    pio_write32(0, PIO_TXF0, 0x1111);
    pio_write32(0, PIO_TXF0, 0x2222);

    uint32_t flevel = pio_read32(0, PIO_FLEVEL);
    /* SM0: TX bits [3:0], RX bits [7:4]; TX count should be 2 */
    uint32_t sm0_tx = flevel & 0x0F;
    ASSERT_EQ(2, sm0_tx, "FLEVEL SM0 TX should be 2");
    PASS();
}

/* ========================================================================
 * PIO Clock Division Tests
 * ======================================================================== */

TEST(test_pio_clkdiv_default_runs_every_cycle) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    /* Default clkdiv should be 1.0 (INT=1, FRAC=0) — execute every cycle */
    ASSERT_EQ(1u << 16, s->clkdiv, "Default CLKDIV should be 1.0");

    /* SET X, N: opcode=7, dest=5(X), data=N => pio_enc(7, 0, (5<<5)|N) */
    uint16_t set_x_5 = pio_enc(PIO_OP_SET, 0, (5 << 5) | 5);
    uint16_t set_x_6 = pio_enc(PIO_OP_SET, 0, (5 << 5) | 6);
    p->instr_mem[0] = set_x_5;
    p->instr_mem[1] = set_x_6;
    s->pc = 0;
    s->execctrl = (1u << 12);  /* wrap_top=1 */

    /* Enable SM0 */
    p->ctrl = 1;

    /* One step should execute SET X, 5 */
    pio_step();
    ASSERT_EQ(5, s->x, "X should be 5 after one step at clkdiv=1");

    /* Next step should execute SET X, 6 */
    pio_step();
    ASSERT_EQ(6, s->x, "X should be 6 after second step at clkdiv=1");
    PASS();
}

TEST(test_pio_clkdiv_divide_by_2) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    /* Set CLKDIV to 2.0 (INT=2, FRAC=0) — execute every other cycle */
    s->clkdiv = 2u << 16;
    s->clk_frac_acc = 0;

    uint16_t set_x_5 = pio_enc(PIO_OP_SET, 0, (5 << 5) | 5);
    uint16_t set_x_6 = pio_enc(PIO_OP_SET, 0, (5 << 5) | 6);
    p->instr_mem[0] = set_x_5;
    p->instr_mem[1] = set_x_6;
    s->pc = 0;
    s->execctrl = (1u << 12);  /* wrap_top=1 */

    /* Enable SM0 */
    p->ctrl = 1;

    /* First step: accumulator=256, divisor=512, not time yet */
    pio_step();
    ASSERT_EQ(0, s->x, "X should still be 0 after 1st step at clkdiv=2");

    /* Second step: accumulator=512 >= 512, executes SET X, 5 */
    pio_step();
    ASSERT_EQ(5, s->x, "X should be 5 after 2nd step at clkdiv=2");

    /* Third step: accumulator=256, not time yet */
    pio_step();
    ASSERT_EQ(5, s->x, "X should still be 5 after 3rd step");

    /* Fourth step: executes SET X, 6 */
    pio_step();
    ASSERT_EQ(6, s->x, "X should be 6 after 4th step at clkdiv=2");
    PASS();
}

TEST(test_pio_clkdiv_fractional) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    /* Set CLKDIV to 1.5 (INT=1, FRAC=128 which is 128/256=0.5) */
    /* Fixed-point divisor = (1 << 8) | 128 = 384 */
    s->clkdiv = (1u << 16) | (128u << 8);
    s->clk_frac_acc = 0;

    uint16_t set_x_5 = pio_enc(PIO_OP_SET, 0, (5 << 5) | 5);
    uint16_t set_x_6 = pio_enc(PIO_OP_SET, 0, (5 << 5) | 6);
    uint16_t set_x_7 = pio_enc(PIO_OP_SET, 0, (5 << 5) | 7);
    p->instr_mem[0] = set_x_5;
    p->instr_mem[1] = set_x_6;
    p->instr_mem[2] = set_x_7;
    s->pc = 0;
    s->execctrl = (2u << 12);  /* wrap_top=2 */

    p->ctrl = 1;

    /* Step 1: acc=256 >= 384? No */
    pio_step();
    ASSERT_EQ(0, s->x, "Step 1: X should be 0 (1.5 divider)");

    /* Step 2: acc=512 >= 384? Yes -> execute, acc=512-384=128 */
    pio_step();
    ASSERT_EQ(5, s->x, "Step 2: X should be 5 (first exec at 1.5 divider)");

    /* Step 3: acc=128+256=384 >= 384? Yes -> execute, acc=0 */
    pio_step();
    ASSERT_EQ(6, s->x, "Step 3: X should be 6 (second exec at 1.5 divider)");

    /* Step 4: acc=256 >= 384? No */
    pio_step();
    ASSERT_EQ(6, s->x, "Step 4: X should still be 6");

    /* Step 5: acc=512 >= 384? Yes -> execute */
    pio_step();
    ASSERT_EQ(7, s->x, "Step 5: X should be 7 (third exec at 1.5 divider)");
    PASS();
}

TEST(test_pio_clkdiv_restart_resets_accumulator) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    /* Set clkdiv=2 and accumulate some */
    s->clkdiv = 2u << 16;
    s->clk_frac_acc = 200;

    /* CLKDIV_RESTART for SM0 (bit 8 of CTRL) */
    pio_write32(0, PIO_CTRL, (1u << 8));

    ASSERT_EQ(0, s->clk_frac_acc, "clk_frac_acc should be 0 after CLKDIV_RESTART");
    PASS();
}

TEST(test_pio_sm_restart_clears_clkdiv_acc) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    s->clk_frac_acc = 500;

    /* SM_RESTART for SM0 (bit 4 of CTRL) */
    pio_write32(0, PIO_CTRL, (1u << 4));

    ASSERT_EQ(0, s->clk_frac_acc, "clk_frac_acc should be 0 after SM_RESTART");
    PASS();
}

TEST(test_pio_force_exec_bypasses_clkdiv) {
    reset_cpu();
    pio_block_t *p = &pio_state[0];
    pio_sm_t *s = &p->sm[0];

    /* Set very slow clock: INT=100 */
    s->clkdiv = 100u << 16;
    s->clk_frac_acc = 0;

    /* Enable SM0 */
    p->ctrl = 1;

    /* Force-exec SET X, 9 via SM_INSTR write */
    uint16_t set_x_9 = pio_enc(PIO_OP_SET, 0, (5 << 5) | 9);
    pio_write32(0, PIO_SM0_CLKDIV + 0x10, set_x_9);

    /* One step should execute the forced instruction despite slow clock */
    pio_step();
    ASSERT_EQ(9, s->x, "Force-exec should bypass clock divider");
    PASS();
}

/* ========================================================================
 * Cycle Timing Tests
 * ======================================================================== */

TEST(test_timing_default_cycles_per_us) {
    reset_cpu();
    /* Default should be 1 cycle per µs (fast-forward mode) */
    ASSERT_EQ(1, timing_config.cycles_per_us, "Default cycles_per_us should be 1");
    PASS();
}

TEST(test_timing_set_clock_mhz) {
    reset_cpu();
    timing_set_clock_mhz(125);
    ASSERT_EQ(125, timing_config.cycles_per_us, "cycles_per_us should be 125 after set");
    ASSERT_EQ(0, timing_config.cycle_accumulator, "accumulator should reset to 0");
    /* Restore default */
    timing_set_clock_mhz(1);
    PASS();
}

TEST(test_timing_alu_1_cycle) {
    /* ALU instructions (MOVS, ADDS, etc.) should be 1 cycle */
    ASSERT_EQ(1, timing_instruction_cycles(0x2000, 0), "MOVS Rd, #imm8 = 1 cycle");
    ASSERT_EQ(1, timing_instruction_cycles(0x1800, 0), "ADDS Rd, Rn, Rm = 1 cycle");
    ASSERT_EQ(1, timing_instruction_cycles(0x4000, 0), "ANDS = 1 cycle");
    PASS();
}

TEST(test_timing_load_store_2_cycles) {
    /* LDR/STR should be 2 cycles */
    ASSERT_EQ(2, timing_instruction_cycles(0x6800, 0), "LDR Rd, [Rn, #imm5] = 2 cycles");
    ASSERT_EQ(2, timing_instruction_cycles(0x6000, 0), "STR Rd, [Rn, #imm5] = 2 cycles");
    ASSERT_EQ(2, timing_instruction_cycles(0x4800, 0), "LDR Rd, [PC, #] = 2 cycles");
    ASSERT_EQ(2, timing_instruction_cycles(0x5800, 0), "LDR reg-offset = 2 cycles");
    PASS();
}

TEST(test_timing_branch_taken) {
    /* Conditional branch taken = 2 cycles, not taken = 1 cycle */
    ASSERT_EQ(2, timing_instruction_cycles(0xD000, 1), "BEQ taken = 2 cycles");
    ASSERT_EQ(1, timing_instruction_cycles(0xD000, 0), "BEQ not taken = 1 cycle");
    /* Unconditional branch always 2 cycles */
    ASSERT_EQ(2, timing_instruction_cycles(0xE000, 0), "B uncond = 2 cycles");
    PASS();
}

TEST(test_timing_bx_blx_3_cycles) {
    /* BX/BLX register = 3 cycles */
    ASSERT_EQ(3, timing_instruction_cycles(0x4700, 0), "BX Rm = 3 cycles");
    ASSERT_EQ(3, timing_instruction_cycles(0x4780, 0), "BLX Rm = 3 cycles");
    PASS();
}

TEST(test_timing_push_pop_1_plus_n) {
    /* PUSH {R0, R1} = 1 + 2 = 3 cycles */
    ASSERT_EQ(3, timing_instruction_cycles(0xB403, 0), "PUSH {R0,R1} = 3 cycles");
    /* PUSH {R0, LR} = 1 + 2 = 3 cycles (bit 8 = LR) */
    ASSERT_EQ(3, timing_instruction_cycles(0xB501, 0), "PUSH {R0,LR} = 3 cycles");
    /* POP {R0} = 1 + 1 = 2 cycles */
    ASSERT_EQ(2, timing_instruction_cycles(0xBC01, 0), "POP {R0} = 2 cycles");
    /* POP {R0, PC} = 1 + 1 + 1(PC refill) + 1(PC bit) = 4 cycles */
    ASSERT_EQ(4, timing_instruction_cycles(0xBD01, 0), "POP {R0,PC} = 4 cycles");
    PASS();
}

TEST(test_timing_bl_32bit_4_cycles) {
    /* BL (32-bit) = 4 cycles */
    ASSERT_EQ(4, timing_instruction_cycles_32(0xF000, 0xF800), "BL = 4 cycles");
    /* MSR = 4 cycles */
    ASSERT_EQ(4, timing_instruction_cycles_32(0xF380, 0x8800), "MSR = 4 cycles");
    /* DSB = 3 cycles */
    ASSERT_EQ(3, timing_instruction_cycles_32(0xF3BF, 0x8F40), "DSB = 3 cycles");
    PASS();
}

TEST(test_timing_accumulator_125mhz) {
    reset_cpu();
    timing_set_clock_mhz(125);
    timer_reset();

    /* Timer should not advance until 125 cycles accumulated */
    uint64_t t0 = timer_state.time_us;

    /* Manually tick 124 cycles — should not advance timer */
    timing_config.cycle_accumulator = 0;
    timing_config.cycle_accumulator += 124;
    uint32_t us = timing_config.cycle_accumulator / timing_config.cycles_per_us;
    ASSERT_EQ(0, us, "124 cycles at 125MHz = 0 µs");

    /* 125 cycles should give exactly 1 µs */
    timing_config.cycle_accumulator = 125;
    us = timing_config.cycle_accumulator / timing_config.cycles_per_us;
    ASSERT_EQ(1, us, "125 cycles at 125MHz = 1 µs");

    /* 250 cycles = 2 µs */
    timing_config.cycle_accumulator = 250;
    us = timing_config.cycle_accumulator / timing_config.cycles_per_us;
    ASSERT_EQ(2, us, "250 cycles at 125MHz = 2 µs");

    (void)t0;
    /* Restore default */
    timing_set_clock_mhz(1);
    PASS();
}

TEST(test_timing_backward_compat) {
    /* At 1 cycle/µs (default), timer should advance 1 µs per instruction step */
    reset_cpu();
    timing_set_clock_mhz(1);
    timer_reset();
    timing_config.cycle_accumulator = 0;

    uint64_t t0 = timer_state.time_us;

    /* Place a MOVS R0, #0 instruction at PC */
    uint32_t pc = cpu.r[15];
    mem_write16(pc, 0x2000);  /* MOVS R0, #0 */
    mem_write16(pc + 2, 0xBE00);  /* BKPT (to stop) */

    cpu_step();  /* Execute MOVS R0, #0 */

    uint64_t t1 = timer_state.time_us;
    ASSERT_EQ(1, (uint32_t)(t1 - t0), "1 cycle/µs: MOVS should advance timer by 1 µs");
    timing_set_clock_mhz(1);
    PASS();
}

TEST(test_timing_stmia_ldmia_1_plus_n) {
    /* STMIA R0!, {R1,R2,R3} = 1 + 3 = 4 cycles */
    ASSERT_EQ(4, timing_instruction_cycles(0xC00E, 0), "STMIA {R1,R2,R3} = 4 cycles");
    /* LDMIA R0!, {R1} = 1 + 1 = 2 cycles */
    ASSERT_EQ(2, timing_instruction_cycles(0xC802, 0), "LDMIA {R1} = 2 cycles");
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
    RUN_TEST(test_i2c_con_default);
    RUN_TEST(test_pwm_csr_default);
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

    BEGIN_CATEGORY("ADC FIFO");
    RUN_TEST(test_adc_fifo_push_pop);
    RUN_TEST(test_adc_fifo_overflow);
    RUN_TEST(test_adc_fifo_underflow);
    RUN_TEST(test_adc_fifo_shift);
    RUN_TEST(test_adc_fifo_w1c_flags);
    RUN_TEST(test_adc_rrobin);
    RUN_TEST(test_adc_start_once_triggers_conversion);
    END_CATEGORY("ADC FIFO");

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
    RUN_TEST(test_ldmia_base_in_reglist);
    RUN_TEST(test_ldmia_base_not_in_reglist_writeback);
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
    RUN_TEST(test_usb_dpram_readback);
    RUN_TEST(test_usb_sie_status_disconnected);
    RUN_TEST(test_usb_main_ctrl_readback);
    END_CATEGORY("USB Controller");

    BEGIN_CATEGORY("Flash ROM Functions");
    RUN_TEST(test_rom_flash_functions_in_table);
    END_CATEGORY("Flash ROM Functions");

    BEGIN_CATEGORY("UART Peripheral");
    RUN_TEST(test_uart_output);
    RUN_TEST(test_uart_registers);
    RUN_TEST(test_uart_baud_readback);
    RUN_TEST(test_uart1_independent);
    RUN_TEST(test_uart_cr_readback);
    RUN_TEST(test_uart_imsc_icr);
    RUN_TEST(test_uart_atomic_set_clr);
    RUN_TEST(test_uart_periph_id);
    END_CATEGORY("UART Peripheral");

    BEGIN_CATEGORY("UART Rx");
    RUN_TEST(test_uart_rx_push_pop);
    RUN_TEST(test_uart_rx_fifo_empty_flag);
    RUN_TEST(test_uart_rx_fifo_full_flag);
    RUN_TEST(test_uart_rx_fifo_order);
    RUN_TEST(test_uart_rx_interrupt);
    RUN_TEST(test_uart_rx_interrupt_clear);
    RUN_TEST(test_uart1_rx_independent);
    RUN_TEST(test_uart_rx_masked_interrupt);
    END_CATEGORY("UART Rx");

    BEGIN_CATEGORY("SPI Peripheral");
    RUN_TEST(test_spi0_status);
    RUN_TEST(test_spi_cr0_readback);
    RUN_TEST(test_spi1_independent);
    RUN_TEST(test_spi_periph_id);
    END_CATEGORY("SPI Peripheral");

    BEGIN_CATEGORY("I2C Peripheral");
    RUN_TEST(test_i2c0_status);
    RUN_TEST(test_i2c_tar_readback);
    RUN_TEST(test_i2c1_independent);
    RUN_TEST(test_i2c_enable_disable);
    RUN_TEST(test_i2c_comp_type);
    END_CATEGORY("I2C Peripheral");

    BEGIN_CATEGORY("PWM Peripheral");
    RUN_TEST(test_pwm_slice_defaults);
    RUN_TEST(test_pwm_slice_readback);
    RUN_TEST(test_pwm_multiple_slices);
    RUN_TEST(test_pwm_global_enable);
    END_CATEGORY("PWM Peripheral");

    BEGIN_CATEGORY("DMA Controller");
    RUN_TEST(test_dma_n_channels);
    RUN_TEST(test_dma_channel_defaults);
    RUN_TEST(test_dma_register_readback);
    RUN_TEST(test_dma_word_transfer);
    RUN_TEST(test_dma_byte_transfer);
    RUN_TEST(test_dma_no_incr_write);
    RUN_TEST(test_dma_interrupt_on_completion);
    RUN_TEST(test_dma_irq_quiet);
    RUN_TEST(test_dma_interrupt_status);
    RUN_TEST(test_dma_chain_transfer);
    RUN_TEST(test_dma_multi_chan_trigger);
    RUN_TEST(test_dma_atomic_set_clr);
    END_CATEGORY("DMA Controller");

    BEGIN_CATEGORY("PIO Peripheral");
    RUN_TEST(test_pio_fstat_fifos_empty);
    RUN_TEST(test_pio_instr_mem_readback);
    RUN_TEST(test_pio_sm_register_readback);
    RUN_TEST(test_pio_ctrl_enable);
    RUN_TEST(test_pio_dbg_cfginfo);
    RUN_TEST(test_pio1_independent);
    RUN_TEST(test_pio_irq_write_clear);
    RUN_TEST(test_pio_tx_rx_fifo_stubs);
    RUN_TEST(test_pio_atomic_set_clr);
    END_CATEGORY("PIO Peripheral");

    BEGIN_CATEGORY("PIO Execution");
    RUN_TEST(test_pio_set_x);
    RUN_TEST(test_pio_set_y);
    RUN_TEST(test_pio_mov_x_to_y);
    RUN_TEST(test_pio_mov_invert);
    RUN_TEST(test_pio_jmp_always);
    RUN_TEST(test_pio_jmp_x_zero);
    RUN_TEST(test_pio_jmp_x_dec);
    RUN_TEST(test_pio_tx_fifo_push_pull);
    RUN_TEST(test_pio_rx_fifo_push_read);
    RUN_TEST(test_pio_pull_blocking_stalls);
    RUN_TEST(test_pio_fstat_reflects_fifo);
    RUN_TEST(test_pio_wrap);
    RUN_TEST(test_pio_out_x);
    RUN_TEST(test_pio_in_x_push);
    RUN_TEST(test_pio_irq_set_clear);
    RUN_TEST(test_pio_sm_enable_step);
    RUN_TEST(test_pio_sm_restart_clears_state);
    RUN_TEST(test_pio_flevel_reflects_fifo);
    END_CATEGORY("PIO Execution");

    BEGIN_CATEGORY("PIO Clock Division");
    RUN_TEST(test_pio_clkdiv_default_runs_every_cycle);
    RUN_TEST(test_pio_clkdiv_divide_by_2);
    RUN_TEST(test_pio_clkdiv_fractional);
    RUN_TEST(test_pio_clkdiv_restart_resets_accumulator);
    RUN_TEST(test_pio_sm_restart_clears_clkdiv_acc);
    RUN_TEST(test_pio_force_exec_bypasses_clkdiv);
    END_CATEGORY("PIO Clock Division");

    BEGIN_CATEGORY("SRAM Aliasing");
    RUN_TEST(test_sram_alias_write_read);
    RUN_TEST(test_sram_alias_write_through);
    RUN_TEST(test_sram_alias_byte_halfword);
    END_CATEGORY("SRAM Aliasing");

    BEGIN_CATEGORY("XIP Cache Control");
    RUN_TEST(test_xip_ctrl_defaults);
    RUN_TEST(test_xip_stat_ready);
    RUN_TEST(test_xip_flush_strobe);
    RUN_TEST(test_xip_counter_readback);
    RUN_TEST(test_xip_sram_readwrite);
    RUN_TEST(test_xip_flash_aliases);
    END_CATEGORY("XIP Cache Control");

    BEGIN_CATEGORY("Cycle Timing");
    RUN_TEST(test_timing_default_cycles_per_us);
    RUN_TEST(test_timing_set_clock_mhz);
    RUN_TEST(test_timing_alu_1_cycle);
    RUN_TEST(test_timing_load_store_2_cycles);
    RUN_TEST(test_timing_branch_taken);
    RUN_TEST(test_timing_bx_blx_3_cycles);
    RUN_TEST(test_timing_push_pop_1_plus_n);
    RUN_TEST(test_timing_bl_32bit_4_cycles);
    RUN_TEST(test_timing_accumulator_125mhz);
    RUN_TEST(test_timing_backward_compat);
    RUN_TEST(test_timing_stmia_ldmia_1_plus_n);
    END_CATEGORY("Cycle Timing");

    printf("\n========================================\n");
    printf(" Results: %d/%d passed, %d failed\n", tests_passed, tests_run, tests_failed);
    printf("========================================\n");

    return tests_failed > 0 ? 1 : 0;
}
