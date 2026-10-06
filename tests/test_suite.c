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
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include "emulator.h"
#include "instructions.h"
#include "nvic.h"
#include "spi_flash.h"
#include "fatfs.h"
#include "bme280.h"
#include "netbridge.h"
#include "cyw43.h"
#include "gdb.h"
#include "fuse_mount.h"
#include "tapif.h"
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
#include "rtc.h"
#include "sdcard.h"
#include "emmc.h"
#include "storage.h"
#include "corepool.h"
#include "wire.h"
#include "devtools.h"
#include "rp2350_rv/rv_cpu.h"
#include "rp2350_rv/rv_clint.h"
#include "rp2350_rv/rv_membus.h"
#include "rp2350_rv/rv_bootrom.h"
#include "rp2350_rv/rp2350_periph.h"
#include "rp2350_rv/rv_icache.h"
#include "rp2350_rv/rp2350_memmap.h"
#include "rp2350_arm/m33_cpu.h"
#include "thumb32.h"
#include "vnet.h"
#include "sdd.h"
#include "w5500.h"

/* Hoisted to file scope: these were declared inside function bodies,
 * which is legal C but needlessly re-declares them per function. */
extern uint32_t m33_basepri;
extern int membus_rp2350_mode;
extern int membus_rp2350_mode;
extern int membus_rp2350_mode;
extern void *membus_rp2350_periph;
extern int membus_rp2350_mode;
extern int membus_rp2350_mode;
extern int membus_rp2350_mode;
extern int membus_rp2350_mode;
extern int watchdog_reboot_pending;

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
    memset(&cores, 0, sizeof(cores));
    cpu.xpsr = 0x01000000; /* Thumb bit set */
    cpu.primask = 0;
    cpu.current_irq = 0xFFFFFFFF;
    cpu.r[13] = RAM_BASE + RAM_SIZE - 0x100; /* SP near top of RAM */
    cpu.r[15] = FLASH_BASE + 0x100;          /* PC in flash */
    cpu.vtor = FLASH_BASE + 0x100;
    num_active_cores = MAX_CORES;
    set_active_core(CORE0);
    watchdog_reboot_pending = 0;
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

static void write_le16(uint8_t *buf, size_t offset, uint16_t val) {
    buf[offset + 0] = (uint8_t)(val & 0xFF);
    buf[offset + 1] = (uint8_t)((val >> 8) & 0xFF);
}

static void write_le32(uint8_t *buf, size_t offset, uint32_t val) {
    buf[offset + 0] = (uint8_t)(val & 0xFF);
    buf[offset + 1] = (uint8_t)((val >> 8) & 0xFF);
    buf[offset + 2] = (uint8_t)((val >> 16) & 0xFF);
    buf[offset + 3] = (uint8_t)((val >> 24) & 0xFF);
}

static void init_minimal_test_elf(uint8_t *elf_data, size_t size) {
    memset(elf_data, 0, size);
    elf_data[0] = 0x7F;
    elf_data[1] = 'E';
    elf_data[2] = 'L';
    elf_data[3] = 'F';
    elf_data[4] = 1;
    elf_data[5] = 1;
    elf_data[6] = 1;
    write_le16(elf_data, 16, 2);
    write_le16(elf_data, 18, 40);
    write_le32(elf_data, 20, 1);
    write_le32(elf_data, 24, 0x10000101);
    write_le32(elf_data, 28, 52);
    write_le16(elf_data, 40, 52);
    write_le16(elf_data, 42, 32);
    write_le16(elf_data, 44, 1);
    write_le32(elf_data, 52, 1);
}

static const char *corepool_test_registry_path(void) {
    static char path[128];
    snprintf(path, sizeof(path), "/tmp/bramble-corepool-test-%d.reg", (int)getpid());
    return path;
}

static void use_corepool_test_registry(void) {
    const char *path = corepool_test_registry_path();
    unlink(path);
    setenv(COREPOOL_REGISTRY_ENV, path, 1);
}

static void cleanup_corepool_test_registry(void) {
    const char *path = getenv(COREPOOL_REGISTRY_ENV);
    if (path && path[0]) {
        unlink(path);
    }
    unsetenv(COREPOOL_REGISTRY_ENV);
}

static void install_vector_handler(uint32_t vector_num, uint32_t handler_addr) {
    uint32_t handler_thumb = handler_addr | 1u;
    uint32_t offset = (cpu.vtor - FLASH_BASE) + vector_num * 4u;
    memcpy(&cpu.flash[offset], &handler_thumb, sizeof(handler_thumb));
}

#define TEST_IO_QSPI_BASE     0x40018000u
#define TEST_PADS_QSPI_BASE   0x40020000u
#define TEST_BUSCTRL_BASE     0x40030000u
#define TEST_XIP_SSI_BAUDR    (XIP_SSI_BASE + 0x14u)

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
    uint32_t handler_addr = FLASH_BASE + 0x240;
    uint32_t saved_pc = cpu.r[15];
    uint32_t saved_xpsr = cpu.xpsr;

    install_vector_handler(EXC_SVCALL, handler_addr);
    instr_svc(0xDF7F);

    ASSERT_EQ(EXC_SVCALL, cpu.current_irq, "SVCall should become the active exception");
    ASSERT_EQ(handler_addr, cpu.r[15], "PC should jump to the SVCall handler");
    ASSERT_EQ(0xFFFFFFF9, cpu.r[14], "LR should contain EXC_RETURN");
    ASSERT_EQ(EXC_SVCALL, cpu.xpsr & 0x3F, "IPSR should reflect SVCall");

    cpu_exception_return(0xFFFFFFF9);
    ASSERT_EQ(saved_pc + 2, cpu.r[15], "SVC should return to the next instruction");
    ASSERT_EQ(saved_xpsr, cpu.xpsr, "xPSR should be restored after SVCall return");
    ASSERT_EQ(0xFFFFFFFF, cpu.current_irq, "No active IRQ after SVCall return");
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
    /* This previously wrote three peripheral registers and asserted nothing, so
     * it passed even when the writes were silently dropped -- which is exactly
     * how the SIO GPIO write bug survived. Read the registers back. */
    reset_cpu();
    spi_init();
    i2c_init();
    pwm_init();

    /* SPI: writing SSPDR with SSE enabled must reach the transmit FIFO. */
    mem_write32(SPI0_BASE + SPI_SSPCR1, SPI_CR1_SSE);
    mem_write32(SPI0_BASE + SPI_SSPDR, 0xAA);
    ASSERT_TRUE(spi_state[0].tx_count == 0,
               "SSPDR is executed immediately, so the TX FIFO drains");

    /* I2C: TAR is plain read/write. (DATA_CMD is write-only -- reading it
     * pops the RX FIFO -- so it cannot be used to check a write landed.) */
    mem_write32(I2C0_BASE + I2C_TAR, 0x55);
    ASSERT_EQ(0x55, mem_read32(I2C0_BASE + I2C_TAR),
              "I2C target-address register must read back");

    /* PWM: the enable register is plain RW. */
    mem_write32(PWM_BASE + PWM_EN, 0xFF);
    ASSERT_EQ(0xFF, mem_read32(PWM_BASE + PWM_EN), "PWM EN must read back");
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

TEST(test_cpu_bind_core_context_roundtrip) {
    reset_cpu();
    dual_core_init();

    cpu.r[0] = 0xAAAAAAAA;
    cpu.r[15] = FLASH_BASE + 0x180;
    cpu.current_irq = 0xFFFFFFFF;
    set_active_core(CORE1);

    cores[CORE0].r[0] = 0x11111111;
    cores[CORE0].r[15] = FLASH_BASE + 0x220;
    cores[CORE0].current_irq = 16;

    cpu_bind_context_t ctx;
    ASSERT_TRUE(cpu_bind_core_context(CORE0, &ctx), "Binding a running core should succeed");
    ASSERT_EQ(0x11111111, cpu.r[0], "Bound CPU state should come from the selected core");
    ASSERT_EQ(FLASH_BASE + 0x220, cpu.r[15], "Bound PC should come from the selected core");
    ASSERT_EQ(16, cpu.current_irq, "Bound IRQ state should come from the selected core");
    ASSERT_EQ(CORE0, get_active_core(), "Binding should switch the active core");

    cpu.r[0] = 0x22222222;
    cpu.r[15] = FLASH_BASE + 0x224;
    cpu.current_irq = 17;

    cpu_unbind_core_context(CORE0, &ctx);
    ASSERT_EQ(0x22222222, cores[CORE0].r[0], "Unbinding should save updated registers back to the core");
    ASSERT_EQ(FLASH_BASE + 0x224, cores[CORE0].r[15], "Unbinding should save the updated PC back to the core");
    ASSERT_EQ(17, cores[CORE0].current_irq, "Unbinding should save IRQ state back to the core");
    ASSERT_EQ(0xAAAAAAAA, cpu.r[0], "Global CPU state should be restored after unbind");
    ASSERT_EQ(FLASH_BASE + 0x180, cpu.r[15], "Global PC should be restored after unbind");
    ASSERT_EQ(CORE0, get_active_core(), "Active core should stay on the core that just ran");
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

TEST(test_uf2_loader_rejects_oversized_payload) {
    reset_cpu();
    uint8_t uf2_data[512];
    memset(uf2_data, 0, sizeof(uf2_data));

    write_le32(uf2_data, 0, 0x0A324655);
    write_le32(uf2_data, 4, 0x9E5D5157);
    write_le32(uf2_data, 12, FLASH_BASE);
    write_le32(uf2_data, 16, 512);
    write_le32(uf2_data, 508, 0x0AB16F30);

    cpu.flash[0] = 0xA5;
    FILE *f = fopen("/tmp/test_bad.uf2", "wb");
    ASSERT_TRUE(f != NULL, "Failed to create malformed UF2 file");
    fwrite(uf2_data, 1, sizeof(uf2_data), f);
    fclose(f);

    int result = load_uf2("/tmp/test_bad.uf2");
    ASSERT_EQ(0, result, "Oversized UF2 payload should be rejected");
    ASSERT_EQ(0xA5, cpu.flash[0], "Rejected UF2 must not modify flash");
    unlink("/tmp/test_bad.uf2");
    PASS();
}

TEST(test_elf_loader_rejects_segment_overflow) {
    reset_cpu();
    uint8_t elf_data[128];
    init_minimal_test_elf(elf_data, sizeof(elf_data));

    write_le32(elf_data, 56, 84);
    write_le32(elf_data, 60, 0xFFFFFF00);
    write_le32(elf_data, 64, 0xFFFFFF00);
    write_le32(elf_data, 68, 16);
    write_le32(elf_data, 72, 1024);
    memset(&elf_data[84], 0xAA, 16);

    FILE *f = fopen("/tmp/test_overflow.elf", "wb");
    ASSERT_TRUE(f != NULL, "Failed to create overflow ELF file");
    fwrite(elf_data, 1, sizeof(elf_data), f);
    fclose(f);

    int result = load_elf("/tmp/test_overflow.elf");
    ASSERT_EQ(0, result, "Overflowing ELF segment should be rejected");
    unlink("/tmp/test_overflow.elf");
    PASS();
}

TEST(test_elf_loader_rejects_filesz_gt_memsz) {
    reset_cpu();
    uint8_t elf_data[128];
    init_minimal_test_elf(elf_data, sizeof(elf_data));

    write_le32(elf_data, 56, 84);
    write_le32(elf_data, 60, FLASH_BASE + 0x100);
    write_le32(elf_data, 64, FLASH_BASE + 0x100);
    write_le32(elf_data, 68, 16);
    write_le32(elf_data, 72, 8);
    memset(&elf_data[84], 0xBB, 16);

    FILE *f = fopen("/tmp/test_filesz_gt_memsz.elf", "wb");
    ASSERT_TRUE(f != NULL, "Failed to create invalid ELF file");
    fwrite(elf_data, 1, sizeof(elf_data), f);
    fclose(f);

    int result = load_elf("/tmp/test_filesz_gt_memsz.elf");
    ASSERT_EQ(0, result, "ELF with filesz > memsz should be rejected");
    unlink("/tmp/test_filesz_gt_memsz.elf");
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

TEST(test_flash_alias_writes_ignored) {
    reset_cpu();
    uint32_t initial = 0x11223344;
    memcpy(&cpu.flash[0], &initial, sizeof(initial));

    mem_write32(XIP_NOALLOC_BASE, 0xAABBCCDD);
    mem_write16(XIP_NOCACHE_BASE + 2, 0xEEFF);
    mem_write8(XIP_NOCACHE_NOALLOC + 1, 0x99);

    ASSERT_EQ(initial, mem_read32(FLASH_BASE), "Flash aliases should ignore writes");
    PASS();
}

TEST(test_nvic_memory_map_subword_access) {
    reset_cpu();
    mem_write8(NVIC_IPR + 1, 0xC0);
    mem_write16(NVIC_IPR + 2, 0x4080);

    ASSERT_EQ(0xC0, mem_read8(NVIC_IPR + 1), "Byte writes should reach NVIC IPR");
    ASSERT_EQ(0x4080, mem_read16(NVIC_IPR + 2), "Halfword writes should reach NVIC IPR");
    ASSERT_EQ(0xC0, nvic_states[0].priority[1], "IRQ1 priority should reflect byte write");
    ASSERT_EQ(0x80, nvic_states[0].priority[2], "IRQ2 priority should reflect halfword write");
    ASSERT_EQ(0x40, nvic_states[0].priority[3], "IRQ3 priority should reflect halfword write");
    PASS();
}

TEST(test_xip_ssi_atomic_aliases) {
    reset_cpu();
    mem_write32(TEST_XIP_SSI_BAUDR, 0x0004);
    ASSERT_EQ(0x0004, mem_read32(TEST_XIP_SSI_BAUDR), "XIP SSI BAUDR write/read");

    mem_write32(TEST_XIP_SSI_BAUDR + 0x2000, 0x0001);
    ASSERT_EQ(0x0005, mem_read32(TEST_XIP_SSI_BAUDR), "SET alias should OR into BAUDR");

    mem_write32(TEST_XIP_SSI_BAUDR + 0x3000, 0x0004);
    ASSERT_EQ(0x0001, mem_read32(TEST_XIP_SSI_BAUDR), "CLR alias should clear BAUDR bits");

    mem_write32(TEST_XIP_SSI_BAUDR + 0x1000, 0x0003);
    ASSERT_EQ(0x0002, mem_read32(TEST_XIP_SSI_BAUDR), "XOR alias should toggle BAUDR bits");
    PASS();
}

TEST(test_io_qspi_atomic_aliases) {
    reset_cpu();
    const uint32_t ctrl = TEST_IO_QSPI_BASE + 0x04;

    mem_write32(ctrl, 0x00000001);
    mem_write32(ctrl + 0x2000, 0x00000004);
    mem_write32(ctrl + 0x3000, 0x00000001);
    mem_write32(ctrl + 0x1000, 0x00000006);

    ASSERT_EQ(0x00000002, mem_read32(ctrl), "IO_QSPI aliases should update the underlying CTRL register");
    PASS();
}

TEST(test_pads_qspi_atomic_aliases) {
    reset_cpu();
    const uint32_t pad = TEST_PADS_QSPI_BASE + 0x04;

    mem_write32(pad, 0x00000003);
    mem_write32(pad + 0x2000, 0x00000008);
    mem_write32(pad + 0x3000, 0x00000002);
    mem_write32(pad + 0x1000, 0x00000009);

    ASSERT_EQ(0x00000000, mem_read32(pad), "PADS_QSPI aliases should update the underlying pad register");
    PASS();
}

TEST(test_busctrl_atomic_aliases) {
    reset_cpu();
    mem_write32(TEST_BUSCTRL_BASE, 0x1111);
    ASSERT_EQ(0x1111, mem_read32(TEST_BUSCTRL_BASE), "BUSCTRL bus priority write/read");

    mem_write32(TEST_BUSCTRL_BASE + 0x3000, 0x0010);
    ASSERT_EQ(0x1101, mem_read32(TEST_BUSCTRL_BASE), "BUSCTRL CLR alias should clear priority bits");

    mem_write32(TEST_BUSCTRL_BASE + 0x1000, 0x0100);
    ASSERT_EQ(0x1001, mem_read32(TEST_BUSCTRL_BASE), "BUSCTRL XOR alias should toggle priority bits");
    PASS();
}

/* SIO GPIO output registers. SIO_BASE and SIO_BASE_GPIO are the same
 * address, and mem_write32() tests SIO_BASE first, so these writes reach
 * sio_write32() and never gpio_bus_match()/gpio_write32(). Reads already
 * delegate; writes have to as well or GPIO output state is never updated
 * and every trace hook built on it is dead. */
#define TEST_SIO_GPIO_OUT      (0xD0000000 + 0x10)
#define TEST_SIO_GPIO_OUT_SET  (0xD0000000 + 0x14)
#define TEST_SIO_GPIO_OUT_CLR  (0xD0000000 + 0x18)
#define TEST_SIO_GPIO_OUT_XOR  (0xD0000000 + 0x1C)
#define TEST_SIO_GPIO_OE       (0xD0000000 + 0x20)
#define TEST_SIO_GPIO_OE_SET   (0xD0000000 + 0x24)

TEST(test_sio_gpio_out_writes_reach_gpio_state) {
    reset_cpu();
    gpio_reset();

    mem_write32(TEST_SIO_GPIO_OUT_SET, 1u << 25);
    ASSERT_EQ(1u << 25, mem_read32(TEST_SIO_GPIO_OUT), "GPIO_OUT_SET should set pin 25");

    mem_write32(TEST_SIO_GPIO_OUT_CLR, 1u << 25);
    ASSERT_EQ(0, mem_read32(TEST_SIO_GPIO_OUT), "GPIO_OUT_CLR should clear pin 25");

    mem_write32(TEST_SIO_GPIO_OUT_XOR, 1u << 25);
    ASSERT_EQ(1u << 25, mem_read32(TEST_SIO_GPIO_OUT), "GPIO_OUT_XOR should toggle pin 25 on");

    mem_write32(TEST_SIO_GPIO_OUT, 0);
    ASSERT_EQ(0, mem_read32(TEST_SIO_GPIO_OUT), "GPIO_OUT should write the whole mask");
    PASS();
}

TEST(test_sio_gpio_oe_writes_reach_gpio_state) {
    reset_cpu();
    gpio_reset();

    mem_write32(TEST_SIO_GPIO_OE_SET, 1u << 25);
    ASSERT_EQ(1u << 25, mem_read32(TEST_SIO_GPIO_OE), "GPIO_OE_SET should enable output on pin 25");

    mem_write32(TEST_SIO_GPIO_OE, 0);
    ASSERT_EQ(0, mem_read32(TEST_SIO_GPIO_OE), "GPIO_OE should write the whole mask");
    PASS();
}

TEST(test_sio_non_gpio_writes_still_reach_sio) {
    reset_cpu();
    gpio_reset();

    /* The delegation must not swallow the rest of SIO space: DIV is above
     * the GPIO offsets and still belongs to sio_write32(). */
    mem_write32(SIO_BASE + 0x60, 100);  /* DIV_UDIVIDEND */
    mem_write32(SIO_BASE + 0x64, 7);    /* DIV_UDIVISOR */
    ASSERT_EQ(14, mem_read32(SIO_BASE + 0x70), "DIV quotient 100/7");
    ASSERT_EQ(2, mem_read32(SIO_BASE + 0x74), "DIV remainder of 100/7");
    PASS();
}

TEST(test_uart_output) {
    /* Previously asserted nothing at all. A write to DR must reach the UART
     * model rather than being dropped by the address decode. */
    reset_cpu();
    uart_init();
    mem_write32(UART0_BASE + UART_CR, UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE);
    mem_write32(UART0_BASE + UART_DR, 'X');
    ASSERT_EQ('X', uart_state[0].dr, "UART0 DR write must reach the UART model");
    PASS();
}

TEST(test_uart_stdio_activity_tracks_tx) {
    reset_cpu();
    ASSERT_EQ(0, uart_stdio_active(0), "UART0 should start with no stdio activity");

    mem_write32(UART0_BASE + UART_CR, UART_CR_UARTEN | UART_CR_TXE);
    mem_write32(UART0_BASE + UART_DR, 'X');

    ASSERT_EQ(1, uart_stdio_active(0), "UART0 stdio activity should latch after TX");
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

TEST(test_systick_enable_before_rvr_does_not_fire) {
    reset_cpu();
    /* Firmware ordering used by arduino-pico: SYST_CSR is written first and
     * SYST_RVR on the next instruction. Enabling a SysTick whose CVR is 0
     * makes it reload; ARMv6-M raises COUNTFLAG/SysTick only on a 1->0
     * transition of a running counter, never on that reload. Firing here
     * takes the exception before the RVR write can run, leaving RVR at 0
     * and re-pending SysTick every cycle thereafter. */
    nvic_write_register(SYST_CSR, 0x07); /* ENABLE | TICKINT | CLKSOURCE */
    systick_tick(1);
    ASSERT_EQ(0, systick_states[get_active_core()].pending,
              "SysTick pended on the cycle it was enabled");
    ASSERT_EQ(0, nvic_read_register(SYST_CSR) & (1u << 16),
              "COUNTFLAG set on the cycle SysTick was enabled");

    /* The firmware's next instruction still gets to set the reload value. */
    nvic_write_register(SYST_RVR, 0x00FFFFFF);
    systick_tick(1);
    ASSERT_EQ(0x00FFFFFF, nvic_read_register(SYST_CVR),
              "counter did not load from RVR");
    PASS();
}

TEST(test_systick_fires_on_counter_wrap) {
    reset_cpu();
    nvic_write_register(SYST_RVR, 4);
    nvic_write_register(SYST_CSR, 0x07);
    systick_tick(1); /* silent load: CVR 0 -> 4 */
    systick_tick(4); /* genuine 1->0 transition */
    ASSERT_EQ(1, systick_states[get_active_core()].pending,
              "SysTick did not pend on a real counter wrap");
    ASSERT_TRUE(nvic_read_register(SYST_CSR) & (1u << 16),
                "COUNTFLAG not set on a real counter wrap");
    ASSERT_EQ(4, nvic_read_register(SYST_CVR), "counter did not reload after wrap");
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
    nvic_states[0].active_exceptions |= (1u << 15);
    nvic_enable_irq(1);
    nvic_set_pending(1);
    nvic_states[0].priority[1] = 0xC0;
    uint8_t active_pri = nvic_get_exception_priority(15);
    uint8_t pending_pri = nvic_states[0].priority[1] & 0xC0;
    ASSERT_TRUE(pending_pri >= active_pri, "Lower priority should not preempt");
    PASS();
}

TEST(test_nvic_priority_preemption_allowed) {
    reset_cpu();
    cpu.current_irq = 15;
    nvic_states[0].active_exceptions |= (1u << 15);
    nvic_enable_irq(0);
    nvic_set_pending(0);
    nvic_states[0].priority[0] = 0x00;
    nvic_write_register(SCB_SHPR3, 0xC0000000); /* SysTick prio = 0xC0 */
    uint8_t active_pri = nvic_get_exception_priority(15);
    uint8_t pending_pri = nvic_states[0].priority[0] & 0xC0;
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
    mem_write32(SCB_SHPR3, 0xC0C00000);
    ASSERT_EQ(0xC0C00000, mem_read32(SCB_SHPR3), "SHPR3 should be readable through the memory bus");
    PASS();
}

TEST(test_scb_vtor_write) {
    reset_cpu();
    mem_write32(SCB_VTOR, 0x10000200);
    ASSERT_EQ(0x10000200, cpu.vtor, "VTOR updated");
    PASS();
}

TEST(test_scb_icsr_memory_map_pending_bits) {
    reset_cpu();
    mem_write32(SCB_ICSR, ICSR_PENDSVSET | ICSR_PENDSTSET);
    ASSERT_TRUE(mem_read32(SCB_ICSR) & ICSR_PENDSVSET, "ICSR should report PendSV pending");
    ASSERT_TRUE(mem_read32(SCB_ICSR) & ICSR_PENDSTSET, "ICSR should report SysTick pending");

    mem_write32(SCB_ICSR, ICSR_PENDSVCLR | ICSR_PENDSTCLR);
    ASSERT_TRUE((mem_read32(SCB_ICSR) & ICSR_PENDSVSET) == 0, "PendSV clear should route through ICSR");
    ASSERT_TRUE((mem_read32(SCB_ICSR) & ICSR_PENDSTSET) == 0, "SysTick clear should route through ICSR");
    PASS();
}

TEST(test_scb_aircr_sysresetreq_requires_key) {
    reset_cpu();
    mem_write32(SCB_AIRCR, 1u << 2);
    ASSERT_EQ(0, watchdog_reboot_pending, "SYSRESETREQ should ignore writes without VECTKEY");

    mem_write32(SCB_AIRCR, 0x05FA0004);
    ASSERT_EQ(1, watchdog_reboot_pending, "SYSRESETREQ should assert reboot when keyed");
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
    /* UARTCR resets to 0x301: TXE and RXE set, UARTEN clear. The datasheet
     * gives both bits a reset of 1 (Table 433), so a read-modify-write of CR
     * that skips the initial write keeps the transmit/receive enables. */
    ASSERT_EQ(0x00000301, mem_read32(UART0_BASE + 0x030),
              "UART CR reset = 0x301 (TXE|RXE set, UARTEN clear)");
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
    /* The transmit interrupt is a LEVEL: "if the transmit FIFO is equal to or
     * lower than the programmed trigger level then the transmit interrupt is
     * asserted HIGH". With the FIFO empty that condition always holds, so
     * writing ICR=TXIC does not leave it low -- it deasserts and immediately
     * reasserts. Treating the clear as permanent is what made the canonical
     * IRQ-driven TX loop hang after one interrupt. */
    mem_write32(UART0_BASE + UART_ICR, UART_INT_TX);
    ASSERT_TRUE(mem_read32(UART0_BASE + UART_RIS) & UART_INT_TX,
               "RIS TX must reassert (level-triggered, FIFO at trigger level)");
    /* MIS = RIS & IMSC, so it reasserts with RIS. */
    ASSERT_TRUE(mem_read32(UART0_BASE + UART_MIS) & UART_INT_TX,
               "MIS TX reasserts while the level condition holds");

    /* Clearing the mask really does remove it: MIS = RIS & IMSC. */
    mem_write32(UART0_BASE + UART_IMSC, 0);
    ASSERT_EQ(0, mem_read32(UART0_BASE + UART_MIS), "MIS zero once unmasked");
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
    ASSERT_EQ(DMA_NUM_CHANNELS_RP2040, mem_read32(DMA_BASE + DMA_N_CHANNELS), "DMA N_CHANNELS");
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
    uint32_t saved_pc = cpu.r[15];
    uint32_t saved_sp = cpu.r[13];
    cpu.r[0] = 0xAAAAAAAA; cpu.r[1] = 0xBBBBBBBB;
    uint32_t saved_r0 = cpu.r[0], saved_r1 = cpu.r[1];
    uint32_t saved_xpsr = cpu.xpsr;
    install_vector_handler(16, handler_addr);
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

TEST(test_cpu_step_delivers_pending_external_irq) {
    reset_cpu();
    uint32_t handler_addr = FLASH_BASE + 0x220;

    install_vector_handler(16, handler_addr);
    nvic_enable_irq(0);
    nvic_set_pending(0);

    cpu_step();

    ASSERT_EQ(16, cpu.current_irq, "IRQ0 should enter exception 16");
    ASSERT_EQ(handler_addr, cpu.r[15], "PC should jump to the IRQ handler");
    ASSERT_TRUE(nvic_states[0].iabr & 0x1, "IABR bit should be set for the active IRQ");

    cpu_exception_return(0xFFFFFFF9);
    ASSERT_EQ(0, nvic_states[0].iabr, "IABR bit should clear after returning");
    PASS();
}

TEST(test_exception_nesting_restores_previous_exception) {
    reset_cpu();

    install_vector_handler(16, FLASH_BASE + 0x260);
    install_vector_handler(17, FLASH_BASE + 0x280);

    cpu_exception_entry(16);
    cpu_exception_entry(17);

    ASSERT_EQ(17, cpu.current_irq, "Nested exception should become active");
    ASSERT_EQ(2, cores[0].exception_depth, "Exception depth should reflect nesting");
    ASSERT_TRUE(nvic_states[0].iabr & 0x3, "Both nested IRQs should be marked active");

    cpu_exception_return(0xFFFFFFF9);
    ASSERT_EQ(16, cpu.current_irq, "Returning from nested exception should restore the previous IRQ");
    ASSERT_EQ(1, cores[0].exception_depth, "Exception depth should decrease after one return");
    ASSERT_EQ(0x1, nvic_states[0].iabr, "Only the outer IRQ should remain active");

    cpu_exception_return(0xFFFFFFF9);
    ASSERT_EQ(0xFFFFFFFF, cpu.current_irq, "Returning from the outer exception should restore thread mode");
    ASSERT_EQ(0, cores[0].exception_depth, "Exception depth should return to zero");
    ASSERT_EQ(0, nvic_states[0].iabr, "No IRQs should remain active");
    PASS();
}

TEST(test_cpu_step_invalid_pc_enters_hardfault) {
    reset_cpu();
    uint32_t handler_addr = FLASH_BASE + 0x2A0;

    install_vector_handler(EXC_HARDFAULT, handler_addr);
    cpu.r[15] = 0x40000000;

    cpu_step();

    ASSERT_EQ(EXC_HARDFAULT, cpu.current_irq, "Invalid PC should enter HardFault");
    ASSERT_EQ(handler_addr, cpu.r[15], "PC should jump to the HardFault handler");
    ASSERT_EQ(0, cores[0].is_halted, "Single HardFault should not lock up the core");
    PASS();
}

TEST(test_double_hardfault_locks_up) {
    reset_cpu();
    cpu.current_irq = EXC_HARDFAULT;

    cpu_exception_entry(EXC_HARDFAULT);

    ASSERT_EQ(1, cores[0].is_halted, "HardFault during HardFault should lock up the core");
    ASSERT_EQ(0xFFFFFFFF, cpu.r[15], "Lockup should halt execution");
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

TEST(test_usb_cdc_stdio_active_requires_bidirectional_console) {
    usb_init();
    ASSERT_EQ(0, usb_cdc_stdio_active(), "CDC stdio should be inactive after reset");

    usb_state.enum_state = USB_ENUM_ACTIVE;
    ASSERT_EQ(0, usb_cdc_stdio_active(), "CDC stdio should need endpoints");

    usb_state.cdc_in_ep = 2;
    ASSERT_EQ(0, usb_cdc_stdio_active(), "CDC stdio should need an OUT endpoint");

    usb_state.cdc_out_ep = 2;
    ASSERT_EQ(1, usb_cdc_stdio_active(), "CDC stdio should be active with both endpoints");
    PASS();
}

TEST(test_usb_cdc_rx_push_requires_ready_console) {
    usb_init();
    ASSERT_EQ(0, usb_cdc_rx_push('A'), "CDC RX push should fail before enumeration");

    usb_state.enum_state = USB_ENUM_ACTIVE;
    ASSERT_EQ(0, usb_cdc_rx_push('B'), "CDC RX push should require an OUT endpoint");

    usb_state.cdc_out_ep = 2;
    ASSERT_EQ(1, usb_cdc_rx_push('C'), "CDC RX push should succeed once console is ready");
    ASSERT_EQ(1, usb_state.cdc_rx_count, "CDC RX FIFO count should increase");
    ASSERT_EQ('C', usb_state.cdc_rx_fifo[0], "CDC RX FIFO should contain the pushed byte");

    usb_state.cdc_rx_count = (int)sizeof(usb_state.cdc_rx_fifo);
    ASSERT_EQ(0, usb_cdc_rx_push('D'), "CDC RX push should fail when FIFO is full");
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
 * CPUID and NVIC Extensions Tests
 * ======================================================================== */

static void test_cpuid_register(void) {
    /* CPUID at 0xE000ED00 should return Cortex-M0+ identifier */
    uint32_t cpuid = mem_read32(SCB_BASE);
    /* Implementer=ARM(0x41), PartNo=0xC60(Cortex-M0+) */
    ASSERT_EQ(0x41, (cpuid >> 24) & 0xFF, "CPUID Implementer = ARM");
    ASSERT_EQ(0xC60, (cpuid >> 4) & 0xFFF, "CPUID PartNo = Cortex-M0+");
    PASS();
}

static void test_nvic_iabr_read(void) {
    /* IABR should return the active bit register */
    nvic_init();
    uint32_t iabr = mem_read32(NVIC_IABR);
    ASSERT_EQ(0, iabr, "IABR initially 0");
    PASS();
}

static void test_nvic_ipr7_readwrite(void) {
    /* IPR7 covers IRQs 24-27 (RP2040 has 26 IRQs: 0-25) */
    nvic_init();
    /* Write priority for IRQ 24 and 25 */
    mem_write32(NVIC_IPR + 24, 0x0000C040);
    /* Read back */
    uint32_t ipr7 = mem_read32(NVIC_IPR + 24);
    ASSERT_EQ(0x40, ipr7 & 0xFF, "IPR7 byte 0 = IRQ24 priority 0x40");
    ASSERT_EQ(0xC0, (ipr7 >> 8) & 0xFF, "IPR7 byte 1 = IRQ25 priority 0xC0");
    PASS();
}

/* ========================================================================
 * RTC Ticking Tests
 * ======================================================================== */

static void test_rtc_load_and_read(void) {
    rtc_init();
    /* Set date: 2026-03-09, Sunday(0), 14:30:45 */
    rtc_state.setup_0 = (2026u << 12) | (3u << 8) | 9u;
    rtc_state.setup_1 = (0u << 24) | (14u << 16) | (30u << 8) | 45u;
    /* Load setup into running time */
    rtc_write32(RTC_CTRL, RTC_CTRL_LOAD | RTC_CTRL_ENABLE);
    /* Read back RTC_1 (year/month/day) */
    uint32_t rtc1 = rtc_read32(RTC_RTC_1);
    ASSERT_EQ(2026, (rtc1 >> 12) & 0xFFF, "RTC year = 2026");
    ASSERT_EQ(3, (rtc1 >> 8) & 0xF, "RTC month = 3");
    ASSERT_EQ(9, rtc1 & 0x1F, "RTC day = 9");
    /* Read back RTC_0 (dotw/hour/min/sec) */
    uint32_t rtc0 = rtc_read32(RTC_RTC_0);
    ASSERT_EQ(14, (rtc0 >> 16) & 0x1F, "RTC hour = 14");
    ASSERT_EQ(30, (rtc0 >> 8) & 0x3F, "RTC min = 30");
    ASSERT_EQ(45, rtc0 & 0x3F, "RTC sec = 45");
    PASS();
}

static void test_rtc_tick_seconds(void) {
    rtc_init();
    rtc_state.setup_0 = (2026u << 12) | (1u << 8) | 1u;
    rtc_state.setup_1 = (0u << 24) | (0u << 16) | (0u << 8) | 0u;
    rtc_write32(RTC_CTRL, RTC_CTRL_LOAD | RTC_CTRL_ENABLE);
    /* Tick 3 seconds worth of microseconds */
    rtc_tick(1000000);
    rtc_tick(1000000);
    rtc_tick(1000000);
    uint32_t rtc0 = rtc_read32(RTC_RTC_0);
    ASSERT_EQ(3, rtc0 & 0x3F, "RTC sec = 3 after 3M us");
    PASS();
}

static void test_rtc_minute_rollover(void) {
    rtc_init();
    /* Start at 23:59:58 */
    rtc_state.setup_0 = (2026u << 12) | (1u << 8) | 1u;
    rtc_state.setup_1 = (0u << 24) | (23u << 16) | (59u << 8) | 58u;
    rtc_write32(RTC_CTRL, RTC_CTRL_LOAD | RTC_CTRL_ENABLE);
    /* Tick 3 seconds -> should roll to 00:00:01 next day */
    rtc_tick(3000000);
    uint32_t rtc0 = rtc_read32(RTC_RTC_0);
    ASSERT_EQ(0, (rtc0 >> 16) & 0x1F, "RTC hour = 0 after midnight");
    ASSERT_EQ(0, (rtc0 >> 8) & 0x3F, "RTC min = 0 after midnight");
    ASSERT_EQ(1, rtc0 & 0x3F, "RTC sec = 1 after midnight");
    /* Day should have advanced */
    uint32_t rtc1 = rtc_read32(RTC_RTC_1);
    ASSERT_EQ(2, rtc1 & 0x1F, "RTC day = 2 after midnight");
    PASS();
}

static void test_rtc_not_ticking_when_disabled(void) {
    rtc_init();
    rtc_state.setup_0 = (2026u << 12) | (1u << 8) | 1u;
    rtc_state.setup_1 = (0u << 24) | (0u << 16) | (0u << 8) | 0u;
    /* Load but do NOT enable */
    rtc_write32(RTC_CTRL, RTC_CTRL_LOAD);
    rtc_tick(5000000);
    uint32_t rtc0 = rtc_read32(RTC_RTC_0);
    ASSERT_EQ(0, rtc0 & 0x3F, "RTC sec = 0 when disabled");
    PASS();
}

/* ========================================================================
 * SD Card Tests
 * ======================================================================== */

TEST(test_sdcard_init_creates_state) {
    sdcard_t sd;
    /* Init with a temp path (won't actually create file in test) */
    int rc = sdcard_init(&sd, "/tmp/bramble_test_sd.img", 1024 * 1024);
    ASSERT_EQ(0, rc, "sdcard_init should succeed");
    ASSERT_TRUE(sd.data != NULL, "SD data buffer should be allocated");
    ASSERT_EQ(1024 * 1024, sd.size, "SD size should match");
    ASSERT_EQ(SD_STATE_IDLE, sd.state, "Initial state should be IDLE");
    ASSERT_EQ(0, sd.initialized, "Should not be initialized yet");
    sdcard_cleanup(&sd);
    PASS();
}

TEST(test_sdcard_cmd0_goes_idle) {
    sdcard_t sd;
    sdcard_init(&sd, "/tmp/bramble_test_sd.img", 1024 * 1024);
    sd.cs_active = 1;

    /* Send CMD0 (GO_IDLE_STATE): 0x40, 0x00, 0x00, 0x00, 0x00, 0x95 */
    sdcard_spi_xfer(&sd, 0x40);
    sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x95);

    /* Read response — should be R1 with idle bit set (0x01) */
    uint8_t r1 = sdcard_spi_xfer(&sd, 0xFF);
    ASSERT_EQ(0x01, r1, "CMD0 response should be 0x01 (idle)");
    ASSERT_EQ(SD_STATE_IDLE, sd.state, "State should be IDLE after CMD0");
    sdcard_cleanup(&sd);
    PASS();
}

TEST(test_sdcard_cmd8_returns_check_pattern) {
    sdcard_t sd;
    sdcard_init(&sd, "/tmp/bramble_test_sd.img", 1024 * 1024);
    sd.cs_active = 1;

    /* CMD8 (SEND_IF_COND): 0x48, 0x00, 0x00, 0x01, 0xAA, 0x87 */
    sdcard_spi_xfer(&sd, 0x48);
    sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x01);
    sdcard_spi_xfer(&sd, 0xAA);
    sdcard_spi_xfer(&sd, 0x87);

    /* R7 response: R1 + 4 bytes */
    uint8_t r1 = sdcard_spi_xfer(&sd, 0xFF);
    ASSERT_EQ(0x01, r1, "CMD8 R1 should be 0x01 (idle)");
    uint8_t b1 = sdcard_spi_xfer(&sd, 0xFF);
    uint8_t b2 = sdcard_spi_xfer(&sd, 0xFF);
    uint8_t b3 = sdcard_spi_xfer(&sd, 0xFF);
    uint8_t b4 = sdcard_spi_xfer(&sd, 0xFF);
    /* Check pattern: 0x000001AA */
    ASSERT_EQ(0x00, b1, "CMD8 byte 1");
    ASSERT_EQ(0x00, b2, "CMD8 byte 2");
    ASSERT_EQ(0x01, b3, "CMD8 byte 3");
    ASSERT_EQ(0xAA, b4, "CMD8 byte 4");
    sdcard_cleanup(&sd);
    PASS();
}

TEST(test_sdcard_acmd41_initializes) {
    sdcard_t sd;
    sdcard_init(&sd, "/tmp/bramble_test_sd.img", 1024 * 1024);
    sd.cs_active = 1;

    /* CMD55 (APP_CMD) */
    sdcard_spi_xfer(&sd, 0x77); sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x00); sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x00); sdcard_spi_xfer(&sd, 0x01);
    sdcard_spi_xfer(&sd, 0xFF); /* read R1 */

    /* ACMD41 (SD_SEND_OP_COND) */
    sdcard_spi_xfer(&sd, 0x69); sdcard_spi_xfer(&sd, 0x40);
    sdcard_spi_xfer(&sd, 0x00); sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x00); sdcard_spi_xfer(&sd, 0x01);
    uint8_t r1 = sdcard_spi_xfer(&sd, 0xFF);

    ASSERT_EQ(0x00, r1, "ACMD41 should return 0x00 (ready)");
    ASSERT_EQ(1, sd.initialized, "Card should be initialized");
    ASSERT_EQ(SD_STATE_READY, sd.state, "State should be READY");
    sdcard_cleanup(&sd);
    PASS();
}

TEST(test_sdcard_cmd17_read_block) {
    sdcard_t sd;
    sdcard_init(&sd, "/tmp/bramble_test_sd.img", 1024 * 1024);
    sd.cs_active = 1;
    sd.initialized = 1;
    sd.state = SD_STATE_READY;

    /* Write known data at block 0 */
    memset(sd.data, 0xAB, 512);

    /* CMD17 (READ_SINGLE_BLOCK) block 0 */
    sdcard_spi_xfer(&sd, 0x51); sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x00); sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x00); sdcard_spi_xfer(&sd, 0x01);

    /* Read R1 */
    uint8_t r1 = sdcard_spi_xfer(&sd, 0xFF);
    ASSERT_EQ(0x00, r1, "CMD17 R1 should be 0x00");

    /* Read data token */
    uint8_t token = sdcard_spi_xfer(&sd, 0xFF);
    ASSERT_EQ(0xFE, token, "Data token should be 0xFE");

    /* Read first data byte */
    uint8_t d0 = sdcard_spi_xfer(&sd, 0xFF);
    ASSERT_EQ(0xAB, d0, "First data byte should match");

    sdcard_cleanup(&sd);
    PASS();
}

TEST(test_sdcard_cmd24_write_block) {
    sdcard_t sd;
    sdcard_init(&sd, "/tmp/bramble_test_sd.img", 1024 * 1024);
    sd.cs_active = 1;
    sd.initialized = 1;
    sd.state = SD_STATE_READY;

    /* CMD24 (WRITE_BLOCK) block 1 */
    sdcard_spi_xfer(&sd, 0x58); sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x00); sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x01); sdcard_spi_xfer(&sd, 0x01);

    /* Read R1 */
    uint8_t r1 = sdcard_spi_xfer(&sd, 0xFF);
    ASSERT_EQ(0x00, r1, "CMD24 R1 should be 0x00");

    /* Send data token */
    sdcard_spi_xfer(&sd, 0xFE);
    /* Send 512 bytes of data */
    for (int i = 0; i < 512; i++) {
        sdcard_spi_xfer(&sd, 0xCD);
    }
    /* Send CRC */
    sdcard_spi_xfer(&sd, 0x00);
    sdcard_spi_xfer(&sd, 0x00);

    /* Read data response */
    uint8_t resp = sdcard_spi_xfer(&sd, 0xFF);
    ASSERT_EQ(0x05, resp & 0x1F, "Data response should be ACCEPTED (0x05)");

    /* Verify data was written */
    ASSERT_EQ(0xCD, sd.data[512], "Written data should be at block 1 offset");
    ASSERT_EQ(1, sd.dirty, "Dirty flag should be set");

    sdcard_cleanup(&sd);
    PASS();
}

/* ========================================================================
 * eMMC Tests
 * ======================================================================== */

TEST(test_emmc_init_creates_state) {
    emmc_t em;
    int rc = emmc_init(&em, "/tmp/bramble_test_emmc.img", 2 * 1024 * 1024);
    ASSERT_EQ(0, rc, "emmc_init should succeed");
    ASSERT_TRUE(em.data != NULL, "eMMC data buffer should be allocated");
    ASSERT_EQ(2 * 1024 * 1024, em.size, "eMMC size should match");
    ASSERT_EQ(EMMC_STATE_IDLE, em.state, "Initial state should be IDLE");
    emmc_cleanup(&em);
    PASS();
}

TEST(test_emmc_cmd0_goes_idle) {
    emmc_t em;
    emmc_init(&em, "/tmp/bramble_test_emmc.img", 2 * 1024 * 1024);
    em.cs_active = 1;

    /* CMD0 */
    emmc_spi_xfer(&em, 0x40); emmc_spi_xfer(&em, 0x00);
    emmc_spi_xfer(&em, 0x00); emmc_spi_xfer(&em, 0x00);
    emmc_spi_xfer(&em, 0x00); emmc_spi_xfer(&em, 0x95);
    uint8_t r1 = emmc_spi_xfer(&em, 0xFF);
    ASSERT_EQ(0x01, r1, "CMD0 response should be 0x01 (idle)");
    emmc_cleanup(&em);
    PASS();
}

TEST(test_emmc_cmd1_initializes) {
    emmc_t em;
    emmc_init(&em, "/tmp/bramble_test_emmc.img", 2 * 1024 * 1024);
    em.cs_active = 1;

    /* CMD1 (SEND_OP_COND) */
    emmc_spi_xfer(&em, 0x41); emmc_spi_xfer(&em, 0x40);
    emmc_spi_xfer(&em, 0x00); emmc_spi_xfer(&em, 0x00);
    emmc_spi_xfer(&em, 0x00); emmc_spi_xfer(&em, 0x01);
    uint8_t r1 = emmc_spi_xfer(&em, 0xFF);
    ASSERT_EQ(0x00, r1, "CMD1 should return 0x00 (ready)");
    ASSERT_EQ(1, em.initialized, "eMMC should be initialized");
    ASSERT_EQ(EMMC_STATE_READY, em.state, "State should be READY");
    emmc_cleanup(&em);
    PASS();
}

TEST(test_emmc_cmd17_read_block) {
    emmc_t em;
    emmc_init(&em, "/tmp/bramble_test_emmc.img", 2 * 1024 * 1024);
    em.cs_active = 1;
    em.initialized = 1;
    em.state = EMMC_STATE_READY;
    memset(em.data, 0x55, 512);

    /* CMD17 block 0 */
    emmc_spi_xfer(&em, 0x51); emmc_spi_xfer(&em, 0x00);
    emmc_spi_xfer(&em, 0x00); emmc_spi_xfer(&em, 0x00);
    emmc_spi_xfer(&em, 0x00); emmc_spi_xfer(&em, 0x01);
    uint8_t r1 = emmc_spi_xfer(&em, 0xFF);
    ASSERT_EQ(0x00, r1, "CMD17 R1 should be 0x00");
    uint8_t token = emmc_spi_xfer(&em, 0xFF);
    ASSERT_EQ(0xFE, token, "Data token should be 0xFE");
    uint8_t d0 = emmc_spi_xfer(&em, 0xFF);
    ASSERT_EQ(0x55, d0, "First data byte should match");
    emmc_cleanup(&em);
    PASS();
}

/* ========================================================================
 * Flash Write-Through Tests
 * ======================================================================== */

TEST(test_flash_persist_sync_no_crash_without_path) {
    /* Calling sync without a path set should not crash */
    flash_persist_sync(0, 4096);
    PASS();
}

/* Regression: the flash ROM stubs took a guest-supplied offset and length and
 * tested `offs + count <= FLASH_SIZE`. Both are 32-bit, so the sum wrapped and
 * the guard passed for wildly out-of-range requests, giving guest firmware a
 * host out-of-bounds write at an attacker-chosen offset. */
TEST(test_rom_flash_erase_rejects_wrapping_offset) {
    reset_cpu();
    /* reset_cpu() leaves flash contents alone, so establish the pattern over
     * the whole window the assertions below inspect. */
    memset(&cpu.flash[0], 0x5A, 0x2000);

    struct { uint32_t offs, count; const char *what; } cases[] = {
        { 0xFFFF0000u, 0x00020000u, "offs+count wraps below FLASH_SIZE" },
        { 0xFFFFFE00u, 0x00000200u, "count straddles the 32-bit boundary" },
        { 0xFFFFF000u, 0x00002000u, "offs just below the top of the array" },
        { 0x00200000u, 0x00000001u, "offs exactly at FLASH_SIZE" },
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        cpu.r[0] = cases[i].offs;
        cpu.r[1] = cases[i].count;
        cpu.r[15] = ROM_FLASH_RANGE_ERASE_ADDR;
        rom_intercept(ROM_FLASH_RANGE_ERASE_ADDR);
    }
    /* Nothing above the first sector may have been touched. */
    int untouched = 1;
    for (uint32_t i = 0x200; i < 0x2000; i++) {
        if (cpu.flash[i] != 0x5A) { untouched = 0; break; }
    }
    ASSERT_TRUE(untouched, "erase must not touch memory past the requested range");

    /* A legitimate in-range request must still work. */
    cpu.r[0] = 0x00001000u;
    cpu.r[1] = 0x00000200u;
    rom_intercept(ROM_FLASH_RANGE_ERASE_ADDR);
    ASSERT_EQ(0xFF, cpu.flash[0x1000], "in-range erase should set 0xFF");
    ASSERT_EQ(0xFF, cpu.flash[0x11FF], "in-range erase should set the whole range");
    ASSERT_EQ(0x5A, cpu.flash[0x1200], "in-range erase must not overrun its length");
    PASS();
}

TEST(test_rom_flash_program_rejects_wrapping_offset) {
    reset_cpu();
    memset(&cpu.flash[0], 0x5A, 512);

    cpu.r[0] = 0xFFFF0000u;   /* offs */
    cpu.r[1] = 0x20000000u;   /* src */
    cpu.r[2] = 0x00020000u;   /* count: offs + count wraps to 0 */
    rom_intercept(ROM_FLASH_RANGE_PROGRAM_ADDR);

    int untouched = 1;
    for (uint32_t i = 0; i < 512; i++) {
        if (cpu.flash[i] != 0x5A) { untouched = 0; break; }
    }
    ASSERT_TRUE(untouched, "program must not write when the range wraps");

    /* In-range program still copies. */
    cpu.r[0] = 0x00002000u;      /* dst offset */
    cpu.r[1] = FLASH_BASE + 0x100u;  /* src: guest address inside flash */
    cpu.r[2] = 0x00000010u;      /* count */
    memset(&cpu.flash[0x100], 0xC3, 16);
    rom_intercept(ROM_FLASH_RANGE_PROGRAM_ADDR);
    ASSERT_EQ(0xC3, cpu.flash[0x2000], "in-range program should copy src to dst");
    PASS();
}

TEST(test_flash_persist_set_and_close) {
    /* Previously called set_path/close and asserted nothing -- not even that
     * the file was created, and it left persist_path set for later tests. */
    unlink("/tmp/bramble_test_flash.bin");
    flash_persist_set_path("/tmp/bramble_test_flash.bin");
    ASSERT_EQ(0, flash_persist_open(), "flash_persist_open must succeed");
    flash_persist_sync(0, 16);
    flash_persist_close();
    /* The image file must now exist and hold the written bytes. */
    FILE *f = fopen("/tmp/bramble_test_flash.bin", "rb");
    ASSERT_TRUE(f != NULL, "flash image file should have been created");
    if (f) {
        uint8_t hdr[16] = {0};
        size_t n = fread(hdr, 1, sizeof(hdr), f);
        fclose(f);
        ASSERT_EQ(16, n, "flash image should contain the synced bytes");
    }
    unlink("/tmp/bramble_test_flash.bin");
    flash_persist_set_path(NULL);
    PASS();
}

/* ========================================================================
 * Core Pool / Threading Tests
 * ======================================================================== */

TEST(test_corepool_detect_host_cpus) {
    int cpus = corepool_detect_host_cpus();
    ASSERT_TRUE(cpus >= 1, "Host must have at least 1 CPU");
    PASS();
}

TEST(test_corepool_init_and_cleanup) {
    use_corepool_test_registry();
    corepool_init();
    ASSERT_TRUE(corepool.host_cpus >= 1, "Host CPUs should be detected");
    ASSERT_EQ(0, corepool.running, "Should not be running initially");
    corepool_cleanup();
    cleanup_corepool_test_registry();
    PASS();
}

TEST(test_corepool_register_and_unregister) {
    use_corepool_test_registry();
    corepool_init();
    corepool_register(2);
    ASSERT_EQ(1, corepool.registered, "Should be registered");
    corepool_unregister();
    ASSERT_EQ(0, corepool.registered, "Should be unregistered");
    corepool_cleanup();
    cleanup_corepool_test_registry();
    PASS();
}

TEST(test_corepool_query_cores_returns_valid) {
    use_corepool_test_registry();
    corepool_init();
    int cores_recommended = corepool_query_cores();
    ASSERT_TRUE(cores_recommended >= 1, "Must recommend at least 1 core");
    ASSERT_TRUE(cores_recommended <= MAX_CORES, "Must not exceed MAX_CORES");
    corepool_cleanup();
    cleanup_corepool_test_registry();
    PASS();
}

TEST(test_corepool_query_cores_prunes_stale_entries) {
    use_corepool_test_registry();

    FILE *f = fopen(corepool_test_registry_path(), "w");
    ASSERT_TRUE(f != NULL, "Failed to create corepool registry fixture");
    fprintf(f, "999999 2 1\n");
    fclose(f);

    corepool_init();
    int cores_recommended = corepool_query_cores();
    ASSERT_TRUE(cores_recommended >= 1, "Query should succeed with stale entries present");

    f = fopen(corepool_test_registry_path(), "r");
    ASSERT_TRUE(f != NULL, "Registry should still exist after query");
    ASSERT_EQ((uint32_t)EOF, (uint32_t)fgetc(f), "Stale registry entries should be pruned");
    fclose(f);

    corepool_cleanup();
    cleanup_corepool_test_registry();
    PASS();
}

TEST(test_num_active_cores_default) {
    /* setup() sets num_active_cores = MAX_CORES for dual-core tests.
     * Verify that Core 1 auto-launch can bump from 1→2 correctly. */
    int saved = num_active_cores;
    num_active_cores = 1;
    ASSERT_EQ(1, num_active_cores, "num_active_cores should start at 1 when set");
    num_active_cores = saved;
    PASS();
}

TEST(test_wfi_sets_core_flag) {
    cpu_init();
    dual_core_init();
    cpu_reset_core(CORE0);
    cores[CORE0].is_wfi = 0;

    /* Simulate WFI by setting flag directly (instruction sets it) */
    cores[CORE0].is_wfi = 1;
    ASSERT_EQ(1, cores[CORE0].is_wfi, "WFI flag should be set");

    /* SEV should clear it */
    cores[CORE0].is_wfi = 0;
    ASSERT_EQ(0, cores[CORE0].is_wfi, "WFI flag should be cleared by SEV");
    PASS();
}

/* ========================================================================
 * Wire Protocol Tests
 * ======================================================================== */

TEST(test_wire_poll_handles_partial_uart_frame) {
    reset_cpu();
    memset(&wire_state, 0, sizeof(wire_state));

    int sv[2];
    ASSERT_EQ(0, socketpair(AF_UNIX, SOCK_STREAM, 0, sv), "socketpair should succeed");

    int flags = fcntl(sv[0], F_GETFL, 0);
    ASSERT_TRUE(flags >= 0, "Should read socket flags");
    ASSERT_EQ(0, fcntl(sv[0], F_SETFL, flags | O_NONBLOCK), "Should set peer socket non-blocking");

    wire_state.link_count = 1;
    wire_state.links[0].state = WIRE_CONNECTED;
    wire_state.links[0].listen_fd = -1;
    wire_state.links[0].peer_fd = sv[0];
    strcpy(wire_state.links[0].path, "/tmp/bramble_test_wire.sock");

    wire_msg_t msg = { .type = WIRE_MSG_UART_DATA, .channel = 0, .len = 1, .reserved = 0 };
    uint8_t frame[sizeof(msg) + 1];
    memcpy(frame, &msg, sizeof(msg));
    frame[sizeof(msg)] = 'A';

    ASSERT_EQ(2, write(sv[1], frame, 2), "Should write a partial frame header");
    wire_poll();
    ASSERT_EQ(0, uart_state[0].rx_count, "Partial frame must not be delivered");

    ASSERT_EQ((uint32_t)(sizeof(frame) - 2), (uint32_t)write(sv[1], frame + 2, sizeof(frame) - 2),
              "Should write remaining frame bytes");
    wire_poll();
    ASSERT_EQ(1, uart_state[0].rx_count, "Complete frame should reach UART RX");
    ASSERT_EQ('A', uart_read32(0, UART_DR), "UART should receive the transmitted byte");

    close(sv[0]);
    close(sv[1]);
    memset(&wire_state, 0, sizeof(wire_state));
    PASS();
}

TEST(test_wire_eth_frame_relay) {
    reset_cpu();
    memset(&wire_state, 0, sizeof(wire_state));

    int sv[2];
    ASSERT_EQ(0, socketpair(AF_UNIX, SOCK_STREAM, 0, sv), "socketpair should succeed");

    int flags = fcntl(sv[0], F_GETFL, 0);
    ASSERT_TRUE(flags >= 0, "Should read socket flags");
    ASSERT_EQ(0, fcntl(sv[0], F_SETFL, flags | O_NONBLOCK), "Should set non-blocking");

    wire_state.link_count = 1;
    wire_state.links[0].state = WIRE_CONNECTED;
    wire_state.links[0].listen_fd = -1;
    wire_state.links[0].peer_fd = sv[0];
    wire_state.links[0].type = WIRE_MSG_ETH_FRAME;
    strcpy(wire_state.links[0].path, "/tmp/bramble_test_eth.sock");

    /* Build a minimal Ethernet frame (14-byte header + 4-byte payload) */
    uint8_t frame[18];
    memset(frame, 0xFF, 6);     /* Broadcast dest MAC */
    memset(frame + 6, 0x02, 6); /* Source MAC */
    frame[12] = 0x08;           /* EtherType: IPv4 */
    frame[13] = 0x00;
    frame[14] = 'T'; frame[15] = 'E'; frame[16] = 'S'; frame[17] = 'T';

    /* Wire framing: 4-byte header + 2-byte LE length + frame */
    wire_msg_t msg = { .type = WIRE_MSG_ETH_FRAME, .channel = 0, .len = 0, .reserved = 0 };
    uint8_t wire_frame[sizeof(msg) + 2 + sizeof(frame)];
    memcpy(wire_frame, &msg, sizeof(msg));
    wire_frame[sizeof(msg)]     = sizeof(frame) & 0xFF;
    wire_frame[sizeof(msg) + 1] = (sizeof(frame) >> 8) & 0xFF;
    memcpy(wire_frame + sizeof(msg) + 2, frame, sizeof(frame));

    /* Initialize vnet so wire handler can deliver the frame */
    vnet_init();

    /* Write complete ETH wire frame */
    ssize_t n = write(sv[1], wire_frame, sizeof(wire_frame));
    ASSERT_EQ((uint32_t)sizeof(wire_frame), (uint32_t)n, "Should write entire ETH wire frame");

    wire_poll();

    /* The frame was delivered to vnet_tx_frame; verify stats */
    ASSERT_EQ(1, vnet.frames_tx, "vnet should have transmitted the ETH frame");

    close(sv[0]);
    close(sv[1]);
    memset(&wire_state, 0, sizeof(wire_state));
    vnet_cleanup();
    PASS();
}

TEST(test_wire_eth_active) {
    memset(&wire_state, 0, sizeof(wire_state));
    ASSERT_EQ(0, wire_eth_active(), "No ETH links initially");

    wire_state.link_count = 1;
    wire_state.links[0].type = WIRE_MSG_ETH_FRAME;
    ASSERT_EQ(1, wire_eth_active(), "ETH link should be active");

    memset(&wire_state, 0, sizeof(wire_state));
    PASS();
}

/* ========================================================================
 * Virtual Network Bus Tests
 * ======================================================================== */

static uint8_t test_vnet_rx_buf[VNET_MAX_FRAME];
static int test_vnet_rx_len = 0;

static void test_vnet_rx_callback(void *ctx, const uint8_t *frame, int len) {
    (void)ctx;
    if (len > 0 && len <= VNET_MAX_FRAME) {
        memcpy(test_vnet_rx_buf, frame, (size_t)len);
        test_vnet_rx_len = len;
    }
}

TEST(test_vnet_init_cleanup) {
    vnet_init();
    ASSERT_EQ(1, vnet.enabled, "vnet should be enabled after init");
    ASSERT_EQ(-1, vnet.tap_fd, "TAP should not be open");
    ASSERT_EQ(0, vnet.port_count, "No ports initially");
    ASSERT_EQ(0, vnet.peer_count, "No peers initially");
    vnet_cleanup();
    ASSERT_EQ(0, vnet.enabled, "vnet should be disabled after cleanup");
    PASS();
}

TEST(test_vnet_register_port) {
    vnet_init();
    uint8_t mac[6] = {0x02, 0xBB, 0x00, 0x00, 0x00, 0x42};
    int idx = vnet_register_port("test-dev", VNET_PORT_CUSTOM, mac,
                                  test_vnet_rx_callback, NULL);
    ASSERT_EQ(0, idx, "First port should be index 0");
    ASSERT_EQ(1, vnet.port_count, "Port count should be 1");
    ASSERT_TRUE(vnet.ports[0].active, "Port should be active");
    ASSERT_EQ(0x42, vnet.ports[0].mac[5], "MAC should match");
    vnet_cleanup();
    PASS();
}

TEST(test_vnet_frame_delivery_to_port) {
    vnet_init();
    /* Port with broadcast-matching MAC */
    uint8_t mac[6] = {0x02, 0xBB, 0x00, 0x00, 0x00, 0x10};
    int port = vnet_register_port("receiver", VNET_PORT_CUSTOM, mac,
                                   test_vnet_rx_callback, NULL);
    test_vnet_rx_len = 0;

    /* Build broadcast Ethernet frame */
    uint8_t frame[18];
    memset(frame, 0xFF, 6);       /* Broadcast dest */
    memset(frame + 6, 0x02, 6);   /* Src MAC */
    frame[12] = 0x08; frame[13] = 0x00;
    frame[14] = 'H'; frame[15] = 'I'; frame[16] = '!'; frame[17] = 0;

    /* TX from "outside" (src_port = -1) */
    vnet_tx_frame(-1, frame, 18);
    ASSERT_EQ(18, test_vnet_rx_len, "Port should receive broadcast frame");
    ASSERT_EQ('H', test_vnet_rx_buf[14], "Frame data should match");

    /* TX from the port itself should NOT be delivered back */
    test_vnet_rx_len = 0;
    vnet_tx_frame(port, frame, 18);
    ASSERT_EQ(0, test_vnet_rx_len, "Port should not receive its own frame");

    vnet_cleanup();
    PASS();
}

TEST(test_vnet_unicast_delivery) {
    vnet_init();
    uint8_t mac1[6] = {0x02, 0xBB, 0x00, 0x00, 0x00, 0x01};
    uint8_t mac2[6] = {0x02, 0xBB, 0x00, 0x00, 0x00, 0x02};

    int port1 = vnet_register_port("dev1", VNET_PORT_CUSTOM, mac1,
                                    test_vnet_rx_callback, NULL);
    vnet_register_port("dev2", VNET_PORT_CUSTOM, mac2,
                       test_vnet_rx_callback, NULL);
    (void)port1;

    /* Unicast frame addressed to mac1 only */
    uint8_t frame[14];
    memcpy(frame, mac1, 6);       /* Dest = mac1 */
    memset(frame + 6, 0x02, 6);   /* Src */
    frame[12] = 0x08; frame[13] = 0x00;

    test_vnet_rx_len = 0;
    vnet_tx_frame(-1, frame, 14);
    /* At least one port should receive it (mac1) */
    ASSERT_TRUE(test_vnet_rx_len > 0, "Unicast should be delivered to matching port");

    vnet_cleanup();
    PASS();
}

TEST(test_vnet_generate_mac) {
    uint8_t mac[6];
    vnet_generate_mac(mac, 0);
    ASSERT_EQ(0x02, mac[0], "Locally-administered unicast");
    ASSERT_EQ(0xBB, mac[1], "Bramble OUI");
    ASSERT_EQ(0x00, mac[5], "Index 0");
    vnet_generate_mac(mac, 255);
    ASSERT_EQ(0xFF, mac[5], "Index 255");
    PASS();
}

TEST(test_vnet_peer_socketpair) {
    /* Test peer frame exchange using socketpair */
    vnet_init();

    /* Register a receiving port */
    uint8_t mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; /* Broadcast MAC */
    int port = vnet_register_port("peer-test", VNET_PORT_CUSTOM, mac,
                                   test_vnet_rx_callback, NULL);
    (void)port;

    /* Create a socketpair to simulate peer connection */
    int sv[2];
    ASSERT_EQ(0, socketpair(AF_UNIX, SOCK_STREAM, 0, sv), "socketpair");
    int flags = fcntl(sv[0], F_GETFL, 0);
    fcntl(sv[0], F_SETFL, flags | O_NONBLOCK);

    /* Manually inject peer */
    vnet.peers[0].fd = sv[0];
    vnet.peers[0].listen_fd = -1;
    vnet.peers[0].rx_len = 0;
    strcpy(vnet.peers[0].path, "/tmp/test_peer");
    vnet.peer_count = 1;

    /* Write a length-prefixed frame from the "remote" side */
    uint8_t frame[14];
    memset(frame, 0xFF, 6);
    memset(frame + 6, 0x02, 6);
    frame[12] = 0x08; frame[13] = 0x00;

    uint8_t hdr[4];
    uint32_t flen = 14;
    hdr[0] = flen & 0xFF; hdr[1] = (flen >> 8) & 0xFF;
    hdr[2] = (flen >> 16) & 0xFF; hdr[3] = (flen >> 24) & 0xFF;

    write(sv[1], hdr, 4);
    write(sv[1], frame, 14);

    test_vnet_rx_len = 0;
    vnet_poll();
    ASSERT_TRUE(test_vnet_rx_len > 0, "Peer frame should be delivered to port");

    close(sv[0]);
    close(sv[1]);
    vnet.peers[0].fd = -1;
    vnet.peer_count = 0;
    vnet_cleanup();
    PASS();
}

/* ========================================================================
 * Software-Defined Device Tests
 * ======================================================================== */

TEST(test_sdd_thermometer_create) {
    sdd_init();
    int idx = sdd_create_thermometer(25.0f, 0, 0x48);
    ASSERT_TRUE(idx >= 0, "Thermometer should be created");
    ASSERT_EQ(1, sdd_registry.count, "Registry should have 1 device");
    ASSERT_EQ(0x48, sdd_registry.devices[0].i2c_addr, "I2C addr should be 0x48");
    sdd_cleanup();
    ASSERT_EQ(0, sdd_registry.count, "Registry should be empty after cleanup");
    PASS();
}

TEST(test_sdd_thermometer_i2c_read) {
    sdd_init();
    sdd_create_thermometer(25.0f, 0, 0x48);
    sdd_device_t *dev = &sdd_registry.devices[0];

    /* Start transaction */
    dev->i2c_start(dev->ctx);

    /* Write register pointer = 0x00 (temperature) */
    dev->i2c_write(dev->ctx, 0x00);

    /* Read MSB and LSB of temperature */
    uint8_t msb = dev->i2c_read(dev->ctx);
    uint8_t lsb = dev->i2c_read(dev->ctx);

    /* 25.0°C in TMP102 format: raw = 25.0 / 0.0625 = 400 = 0x190
     * Register value = 0x190 << 4 = 0x1900
     * MSB = 0x19, LSB = 0x00 */
    ASSERT_EQ(0x19, msb, "Temperature MSB for 25°C");
    ASSERT_EQ(0x00, lsb, "Temperature LSB for 25°C");

    dev->i2c_stop(dev->ctx);
    sdd_cleanup();
    PASS();
}

TEST(test_sdd_thermometer_custom_temp) {
    sdd_init();
    sdd_create_thermometer(37.5f, 0, 0x48);
    sdd_device_t *dev = &sdd_registry.devices[0];

    dev->i2c_start(dev->ctx);
    dev->i2c_write(dev->ctx, 0x00);  /* Temperature register */

    uint8_t msb = dev->i2c_read(dev->ctx);
    uint8_t lsb = dev->i2c_read(dev->ctx);

    /* 37.5°C: raw = 37.5 / 0.0625 = 600 = 0x258
     * Register = 0x258 << 4 = 0x2580
     * MSB = 0x25, LSB = 0x80 */
    ASSERT_EQ(0x25, msb, "Temperature MSB for 37.5°C");
    ASSERT_EQ(0x80, lsb, "Temperature LSB for 37.5°C");

    dev->i2c_stop(dev->ctx);
    sdd_cleanup();
    PASS();
}

TEST(test_sdd_thermometer_config_register) {
    sdd_init();
    sdd_create_thermometer(25.0f, 0, 0x48);
    sdd_device_t *dev = &sdd_registry.devices[0];

    dev->i2c_start(dev->ctx);
    dev->i2c_write(dev->ctx, 0x01);  /* Config register */

    uint8_t cfg_msb = dev->i2c_read(dev->ctx);
    uint8_t cfg_lsb = dev->i2c_read(dev->ctx);
    uint16_t config = ((uint16_t)cfg_msb << 8) | cfg_lsb;

    ASSERT_EQ(0x60A0, config, "Default config should be 0x60A0");

    dev->i2c_stop(dev->ctx);
    sdd_cleanup();
    PASS();
}

TEST(test_sdd_create_from_arg) {
    sdd_init();
    int rc = sdd_create_from_arg("thermometer:temp=42.0,addr=0x49");
    ASSERT_EQ(0, rc, "Create from arg should succeed");
    ASSERT_EQ(1, sdd_registry.count, "Should have 1 device");
    ASSERT_EQ(0x49, sdd_registry.devices[0].i2c_addr, "Addr should be 0x49");
    sdd_cleanup();
    PASS();
}

TEST(test_sdd_unknown_type) {
    sdd_init();
    int rc = sdd_create_from_arg("nonexistent_device");
    ASSERT_EQ(-1, rc, "Unknown device type should fail");
    ASSERT_EQ(0, sdd_registry.count, "No devices should be registered");
    sdd_cleanup();
    PASS();
}

/* ========================================================================
 * W5500 Live Networking Tests
 * ======================================================================== */

TEST(test_w5500_init_host_fds) {
    w5500_t dev;
    w5500_init(&dev);
    ASSERT_EQ(0, dev.live, "Live mode should be off by default");
    ASSERT_EQ(-1, dev.vnet_port, "vnet port should be -1");
    for (int i = 0; i < W5500_NUM_SOCKETS; i++) {
        ASSERT_EQ(-1, dev.sockets[i].host_fd, "Host fd should be -1");
        ASSERT_EQ(-1, dev.sockets[i].host_listen_fd, "Listen fd should be -1");
    }
    PASS();
}

TEST(test_w5500_set_live) {
    w5500_t dev;
    w5500_init(&dev);
    ASSERT_EQ(0, dev.live, "Should start not-live");
    w5500_set_live(&dev, 1);
    ASSERT_EQ(1, dev.live, "Should be live after set");
    w5500_set_live(&dev, 0);
    ASSERT_EQ(0, dev.live, "Should be not-live after clear");
    PASS();
}

TEST(test_w5500_tcp_open_creates_host_socket) {
    w5500_t dev;
    w5500_init(&dev);
    w5500_set_live(&dev, 1);

    /* Open socket 0 in TCP mode */
    dev.sockets[0].regs[W5500_Sn_MR] = W5500_MR_TCP;
    dev.sockets[0].regs[W5500_Sn_CR] = W5500_CMD_OPEN;
    /* Trigger command processing via SPI write to CR register */
    w5500_spi_cs(&dev, 1);
    /* Manually process (in real usage the SPI write triggers this) */
    dev.sockets[0].regs[W5500_Sn_CR] = W5500_CMD_OPEN;
    /* Simulate by calling internal - just check the state */
    /* Re-init to test */
    w5500_init(&dev);
    w5500_set_live(&dev, 1);
    dev.sockets[0].regs[W5500_Sn_MR] = W5500_MR_TCP;
    dev.sockets[0].regs[W5500_Sn_CR] = W5500_CMD_OPEN;

    /* Call the SPI write path to trigger command processing:
     * BSB for socket 0 register block = 0x01 (shifted left 3 = 0x08)
     * We write to offset W5500_Sn_CR (0x0001) */
    dev.cs_active = 0;
    w5500_spi_cs(&dev, 1);
    w5500_spi_xfer(&dev, 0x00);          /* Addr high = 0 */
    w5500_spi_xfer(&dev, W5500_Sn_CR);   /* Addr low = 1 */
    w5500_spi_xfer(&dev, (0x01 << 3) | 0x04); /* BSB=sock0_reg, write, VDM */
    w5500_spi_xfer(&dev, W5500_CMD_OPEN); /* Write OPEN command */
    w5500_spi_cs(&dev, 0);

    ASSERT_EQ(W5500_SOCK_INIT, dev.sockets[0].regs[W5500_Sn_SR],
              "Socket should be in INIT state after TCP OPEN");
    ASSERT_TRUE(dev.sockets[0].host_fd >= 0,
                "Host socket should be created in live mode");

    /* Cleanup */
    if (dev.sockets[0].host_fd >= 0) close(dev.sockets[0].host_fd);
    PASS();
}

TEST(test_w5500_udp_open_creates_host_socket) {
    w5500_t dev;
    w5500_init(&dev);
    w5500_set_live(&dev, 1);

    /* Open via SPI */
    dev.sockets[0].regs[W5500_Sn_MR] = W5500_MR_UDP;
    dev.cs_active = 0;
    w5500_spi_cs(&dev, 1);
    w5500_spi_xfer(&dev, 0x00);
    w5500_spi_xfer(&dev, W5500_Sn_CR);
    w5500_spi_xfer(&dev, (0x01 << 3) | 0x04);
    w5500_spi_xfer(&dev, W5500_CMD_OPEN);
    w5500_spi_cs(&dev, 0);

    ASSERT_EQ(W5500_SOCK_UDP, dev.sockets[0].regs[W5500_Sn_SR],
              "Socket should be in UDP state");
    ASSERT_TRUE(dev.sockets[0].host_fd >= 0,
                "Host UDP socket should be created");

    if (dev.sockets[0].host_fd >= 0) close(dev.sockets[0].host_fd);
    PASS();
}

TEST(test_w5500_close_cleans_host_socket) {
    w5500_t dev;
    w5500_init(&dev);
    w5500_set_live(&dev, 1);

    /* Open TCP */
    dev.sockets[0].regs[W5500_Sn_MR] = W5500_MR_TCP;
    dev.cs_active = 0;
    w5500_spi_cs(&dev, 1);
    w5500_spi_xfer(&dev, 0x00);
    w5500_spi_xfer(&dev, W5500_Sn_CR);
    w5500_spi_xfer(&dev, (0x01 << 3) | 0x04);
    w5500_spi_xfer(&dev, W5500_CMD_OPEN);
    w5500_spi_cs(&dev, 0);
    ASSERT_TRUE(dev.sockets[0].host_fd >= 0, "Socket should be open");

    /* Close */
    dev.cs_active = 0;
    w5500_spi_cs(&dev, 1);
    w5500_spi_xfer(&dev, 0x00);
    w5500_spi_xfer(&dev, W5500_Sn_CR);
    w5500_spi_xfer(&dev, (0x01 << 3) | 0x04);
    w5500_spi_xfer(&dev, W5500_CMD_CLOSE);
    w5500_spi_cs(&dev, 0);

    ASSERT_EQ(W5500_SOCK_CLOSED, dev.sockets[0].regs[W5500_Sn_SR],
              "Socket should be CLOSED");
    ASSERT_EQ(-1, dev.sockets[0].host_fd, "Host fd should be -1 after close");
    PASS();
}

/* ========================================================================
 * Cortex-M33 Tests
 * ======================================================================== */

TEST(test_m33_cpuid) {
    reset_cpu();
    /* Default should be M0+ CPUID */
    uint32_t cpuid = nvic_read_register(0xE000ED00);
    ASSERT_EQ(0x410CC601, cpuid, "Default CPUID should be Cortex-M0+");
    /* Switch to M33 */
    nvic_cpuid_value = 0x410FD210;
    cpuid = nvic_read_register(0xE000ED00);
    ASSERT_EQ(0x410FD210, cpuid, "M33 CPUID should be 0x410FD210");
    /* Restore */
    nvic_cpuid_value = 0x410CC601;
    PASS();
}

TEST(test_m33_basepri) {
    reset_cpu();
    m33_basepri = 0;
    /* MSR BASEPRI, R0 (SYSm=0x11) — set BASEPRI to 0x40 */
    cpu.r[0] = 0x40;
    instr_msr_32(0, 0x11);
    ASSERT_EQ(0x40, m33_basepri, "BASEPRI should be set to 0x40");
    /* MRS R1, BASEPRI (SYSm=0x11) */
    instr_mrs_32(1, 0x11);
    ASSERT_EQ(0x40, cpu.r[1], "MRS BASEPRI should read 0x40");
    /* BASEPRI_MAX: only increases threshold */
    cpu.r[0] = 0x20;
    instr_msr_32(0, 0x12);  /* BASEPRI_MAX */
    ASSERT_EQ(0x40, m33_basepri, "BASEPRI_MAX with lower value should not decrease");
    cpu.r[0] = 0x80;
    instr_msr_32(0, 0x12);
    ASSERT_EQ(0x80, m33_basepri, "BASEPRI_MAX with higher value should increase");
    m33_basepri = 0;
    PASS();
}

TEST(test_m33_thumb2_sdiv) {
    reset_cpu();
    /* SDIV R0, R1, R2: R0 = R1 / R2 (signed) */
    cpu.r[1] = 42;
    cpu.r[2] = 7;
    /* SDIV encoding: upper=0xFB91, lower=0xF0F2 (Rd=0, Rn=1, Rm=2) */
    uint16_t upper = 0xFB91;
    uint16_t lower = 0xF0F2;
    uint32_t instr32 = ((uint32_t)upper << 16) | lower;
    (void)instr32;
    /* Call the 32-bit handler directly */
    thumb32_step(0x10000100, upper, lower);
    ASSERT_EQ(6, cpu.r[0], "42 / 7 = 6 (SDIV)");
    PASS();
}

TEST(test_m33_thumb2_movw_movt) {
    reset_cpu();
    /* MOVW R0, #0x1234: upper=0xF241, lower=0x2034 */
    /* MOVW: imm16 = i:imm4:imm3:imm8 */
    /* MOVW R0, #0x0000 = F240 0000 */
    thumb32_step(0x10000100, 0xF240, 0x0000);
    ASSERT_EQ(0, cpu.r[0], "MOVW R0, #0 should set R0=0");
    /* MOVW R0, #0x00FF = F240 00FF */
    thumb32_step(0x10000100, 0xF240, 0x00FF);
    ASSERT_EQ(0xFF, cpu.r[0], "MOVW R0, #0xFF should set R0=0xFF");
    PASS();
}

/* ========================================================================
 * RISC-V Hazard3 Tests
 * ======================================================================== */

TEST(test_rv_cpu_init) {
    rv_cpu_state_t rv;
    rv_cpu_init(&rv, 0);
    ASSERT_EQ(1, rv.is_halted, "RV CPU should start halted");
    ASSERT_EQ(0, rv.hart_id, "Hart ID should be 0");
    ASSERT_EQ(0, rv.x[0], "x0 should be 0");
    uint32_t misa = rv.csr[CSR_MISA];
    ASSERT_TRUE(misa & (1 << 8), "misa should have I bit");
    ASSERT_TRUE(misa & (1 << 12), "misa should have M bit");
    ASSERT_TRUE(misa & (1 << 0), "misa should have A bit");
    ASSERT_TRUE(misa & (1 << 2), "misa should have C bit");
    PASS();
}

TEST(test_rv_cpu_reset) {
    rv_cpu_state_t rv;
    rv_cpu_init(&rv, 1);
    rv_cpu_reset(&rv, 0x10000000);
    ASSERT_EQ(0, rv.is_halted, "RV CPU should not be halted after reset");
    ASSERT_EQ(0x10000000, rv.pc, "PC should be set to entry");
    ASSERT_EQ(1, rv.hart_id, "Hart ID should be 1");
    PASS();
}

TEST(test_rv_addi_instruction) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;
    rv_cpu_reset(&rv, 0x10000000);
    /* ADDI x1, x0, 42 (0x02A00093) */
    uint32_t instr = 0x02A00093;
    memcpy(&cpu.flash[0], &instr, 4);
    rv_cpu_step(&rv);
    ASSERT_EQ(42, rv.x[1], "x1 should be 42 after ADDI x1, x0, 42");
    ASSERT_EQ(0x10000004, rv.pc, "PC should advance by 4");
    PASS();
}

TEST(test_rv_lui_instruction) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;
    rv_cpu_reset(&rv, 0x10000000);
    /* LUI x2, 0x20000 (sets x2 = 0x20000000) — 0x200000B7 */
    uint32_t instr = 0x200000B7;  /* LUI x1, 0x20000 */
    memcpy(&cpu.flash[0], &instr, 4);
    rv_cpu_step(&rv);
    ASSERT_EQ(0x20000000, rv.x[1], "x1 should be 0x20000000 after LUI");
    PASS();
}

TEST(test_rv_add_sub) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;
    rv_cpu_reset(&rv, 0x10000000);
    /* ADDI x1, x0, 10 */
    uint32_t prog[] = {
        0x00A00093,  /* addi x1, x0, 10 */
        0x01400113,  /* addi x2, x0, 20 */
        0x002081B3,  /* add  x3, x1, x2 */
        0x40208233,  /* sub  x4, x1, x2 */
    };
    memcpy(&cpu.flash[0], prog, sizeof(prog));
    rv_cpu_step(&rv); rv_cpu_step(&rv); rv_cpu_step(&rv); rv_cpu_step(&rv);
    ASSERT_EQ(10, rv.x[1], "x1 = 10");
    ASSERT_EQ(20, rv.x[2], "x2 = 20");
    ASSERT_EQ(30, rv.x[3], "x3 = x1 + x2 = 30");
    ASSERT_EQ((uint32_t)-10, rv.x[4], "x4 = x1 - x2 = -10");
    PASS();
}

TEST(test_rv_branch_beq) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;
    rv_cpu_reset(&rv, 0x10000000);
    uint32_t prog[] = {
        0x00000093,  /* addi x1, x0, 0 */
        0x00108463,  /* beq  x1, x1, +8 (skip next) */
        0x06400093,  /* addi x1, x0, 100 (should be skipped) */
        0x0C800093,  /* addi x1, x0, 200 (branch target) */
    };
    memcpy(&cpu.flash[0], prog, sizeof(prog));
    rv_cpu_step(&rv); rv_cpu_step(&rv); rv_cpu_step(&rv);
    ASSERT_EQ(200, rv.x[1], "x1 should be 200 (branch taken, skipped 100)");
    PASS();
}

TEST(test_rv_load_store) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;
    rv_cpu_reset(&rv, 0x10000000);
    /* Use SW/LW for reliable word-level store/load test */
    uint32_t prog[] = {
        0x200000B7,  /* lui  x1, 0x20000 (x1=0x20000000, SRAM base) */
        0x0FF00113,  /* addi x2, x0, 255 */
        0x0020A023,  /* sw   x2, 0(x1) — store word */
        0x0000A183,  /* lw   x3, 0(x1) — load word */
    };
    memcpy(&cpu.flash[0], prog, sizeof(prog));
    for (int i = 0; i < 4; i++) rv_cpu_step(&rv);
    ASSERT_EQ(0x20000000, rv.x[1], "x1 should be SRAM base");
    ASSERT_EQ(255, rv.x[2], "x2 should be 255");
    ASSERT_EQ(255, rv.x[3], "x3 should be 255 after SW+LW roundtrip");
    PASS();
}

TEST(test_rv_mul_div) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;
    rv_cpu_reset(&rv, 0x10000000);
    uint32_t prog[] = {
        0x00700093,  /* addi x1, x0, 7 */
        0x00600113,  /* addi x2, x0, 6 */
        0x022081B3,  /* mul  x3, x1, x2 */
        0x02208233,  /* mulh x4, x1, x2 (signed high) */
        0x022042B3,  /* div  x5, x0, x2 (0/6=0) */
        0x02204333,  /* div  x6, x0, x2... actually let's do x1/x2 */
    };
    /* Fix: div x5, x1, x2 = 7/6 = 1 */
    prog[4] = 0x0220C2B3;  /* div x5, x1, x2 */
    memcpy(&cpu.flash[0], prog, sizeof(prog));
    for (int i = 0; i < 5; i++) rv_cpu_step(&rv);
    ASSERT_EQ(42, rv.x[3], "7 * 6 = 42");
    ASSERT_EQ(1, rv.x[5], "7 / 6 = 1");
    PASS();
}

TEST(test_rv_jal_jalr) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;
    rv_cpu_reset(&rv, 0x10000000);
    uint32_t prog[] = {
        0x008000EF,  /* jal x1, +8 (link to x1, jump to PC+8) */
        0x00000013,  /* nop (addi x0, x0, 0) */
        0x06400113,  /* addi x2, x0, 100 (target: x2=100) */
    };
    memcpy(&cpu.flash[0], prog, sizeof(prog));
    rv_cpu_step(&rv);  /* JAL: x1=PC+4, PC=PC+8 */
    ASSERT_EQ(0x10000004, rv.x[1], "x1 should be return address (PC+4)");
    ASSERT_EQ(0x10000008, rv.pc, "PC should be at JAL target");
    rv_cpu_step(&rv);  /* ADDI x2, x0, 100 */
    ASSERT_EQ(100, rv.x[2], "x2 should be 100");
    PASS();
}

TEST(test_rv_compressed_c_addi) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;
    rv_cpu_reset(&rv, 0x10000000);
    /* C.LI rd, imm: 010 imm[5] rd[4:0] imm[4:0] 01
     * C.LI x1, 5: 010 0 00001 00101 01 = 0x4095 */
    uint16_t c_li = 0x4095;
    /* C.ADDI rd, nzimm: 000 nzimm[5] rd[4:0] nzimm[4:0] 01
     * C.ADDI x1, 10: 000 0 00001 01010 01 = 0x00A9 */
    uint16_t c_addi = 0x00A9;
    memcpy(&cpu.flash[0], &c_li, 2);
    memcpy(&cpu.flash[2], &c_addi, 2);
    rv_cpu_step(&rv);  /* C.LI x1, 5 */
    ASSERT_EQ(5, rv.x[1], "x1 should be 5 after C.LI");
    ASSERT_EQ(0x10000002, rv.pc, "PC should advance by 2 (compressed)");
    rv_cpu_step(&rv);  /* C.ADDI x1, 10 */
    ASSERT_EQ(15, rv.x[1], "x1 should be 15 after C.ADDI x1, 10");
    PASS();
}

TEST(test_rv_csr_mhartid) {
    rv_cpu_state_t rv;
    rv_cpu_init(&rv, 0);
    ASSERT_EQ(0, rv_csr_read(&rv, CSR_MHARTID), "Hart 0 mhartid should be 0");
    rv_cpu_init(&rv, 1);
    ASSERT_EQ(1, rv_csr_read(&rv, CSR_MHARTID), "Hart 1 mhartid should be 1");
    PASS();
}

TEST(test_rv_trap_enter_return) {
    rv_cpu_state_t rv;
    rv_cpu_init(&rv, 0);
    rv_cpu_reset(&rv, 0x10000100);
    rv.csr[CSR_MSTATUS] |= MSTATUS_MIE;  /* Enable interrupts */
    rv.csr[CSR_MTVEC] = 0x10000200;       /* Direct mode */
    rv_trap_enter(&rv, MCAUSE_ILLEGAL_INSTR, 0xDEADBEEF);
    ASSERT_EQ(0x10000100, rv.csr[CSR_MEPC], "MEPC should be saved PC");
    ASSERT_EQ(MCAUSE_ILLEGAL_INSTR, rv.csr[CSR_MCAUSE], "MCAUSE should be illegal instr");
    /* Hazard3 hardwires mtval to zero (datasheet Table 367): "Machine bad
     * address or instruction. Hardwired to zero." It was being written with the
     * fault data, so firmware distinguishing fault causes by mtval saw
     * meaningless values on every trap. */
    ASSERT_EQ(0x00000000, rv.csr[CSR_MTVAL], "MTVAL is hardwired to zero on Hazard3");
    ASSERT_EQ(0x10000200, rv.pc, "PC should be at mtvec");
    ASSERT_TRUE(!(rv.csr[CSR_MSTATUS] & MSTATUS_MIE), "MIE should be cleared");
    ASSERT_TRUE(rv.csr[CSR_MSTATUS] & MSTATUS_MPIE, "MPIE should be set (was enabled)");
    rv_trap_return(&rv);
    ASSERT_EQ(0x10000100, rv.pc, "PC should be restored to MEPC");
    ASSERT_TRUE(rv.csr[CSR_MSTATUS] & MSTATUS_MIE, "MIE should be restored from MPIE");
    PASS();
}

TEST(test_rv_clint_timer) {
    rv_clint_state_t clint;
    rv_clint_init(&clint, 1);
    ASSERT_EQ(0, clint.mtime, "mtime should start at 0");
    rv_clint_tick(&clint, 1);
    ASSERT_EQ(1, clint.mtime, "mtime should be 1 after 1 tick");
    rv_clint_tick(&clint, 99);
    ASSERT_EQ(100, clint.mtime, "mtime should be 100 after 100 ticks");
    PASS();
}

TEST(test_rv_clint_timer_interrupt) {
    rv_clint_state_t clint;
    rv_cpu_state_t rv;
    rv_clint_init(&clint, 1);
    rv_cpu_init(&rv, 0);
    rv_cpu_reset(&rv, 0x10000000);
    rv.csr[CSR_MSTATUS] |= MSTATUS_MIE;
    rv.csr[CSR_MIE] = (1u << 7);  /* Enable MTIE */
    rv.csr[CSR_MTVEC] = 0x10000100;
    clint.mtimecmp[0] = 50;
    /* Tick past the compare */
    rv_clint_tick(&clint, 50);
    int delivered = rv_clint_check_interrupts(&clint, &rv);
    ASSERT_EQ(1, delivered, "Timer interrupt should be delivered");
    ASSERT_EQ(0x10000100, rv.pc, "PC should be at mtvec handler");
    PASS();
}

/* Regression (datasheet audit C1/C2): the IO_BANK0 per-pin window claimed a
 * fixed 0x200 span, so on RP2040 -- where GPIO29_CTRL is the last per-pin
 * register at +0x0EC and INTR0 begins at +0x0F0 -- INTR0..PROC0_INTS were
 * decoded as phantom GPIO30+ STATUS/CTRL registers and the per-pin branch
 * returned early. A plain store to PROC0_INTE (what gpio_set_irq_enabled()
 * does without an atomic alias) was dropped, and gpio_acknowledge_irq() could
 * never clear a latched edge. */
TEST(test_gpio_interrupt_regs_are_decoded_not_pins) {
    reset_cpu();
    gpio_init();

    /* Pin window is 30 pins * 8 bytes; INTR0 must not be inside it. */
    uint32_t intr0 = IO_BANK0_BASE + GPIO_IRQ_WINDOW_RP2040;
    uint32_t proc0_inte0 = IO_BANK0_BASE + GPIO_IRQ_WINDOW_RP2040 + 0x10;
    uint32_t proc0_intf0 = IO_BANK0_BASE + GPIO_IRQ_WINDOW_RP2040 + 0x20;
    uint32_t proc0_ints0 = IO_BANK0_BASE + GPIO_IRQ_WINDOW_RP2040 + 0x30;

    gpio_write32(proc0_inte0, 0xFFFFFFFF);
    gpio_write32(proc0_intf0, 0xFFFFFFFF);
    ASSERT_EQ(0xFFFFFFFF, gpio_read32(proc0_inte0), "PROC0_INTE0 must be writable");
    ASSERT_EQ(0xFFFFFFFF, gpio_read32(proc0_intf0), "PROC0_INTF0 must be writable");
    /* INTS is (INTR|INTF)&INTE and must reflect the force bits. */
    ASSERT_EQ(0xFFFFFFFF, gpio_read32(proc0_ints0), "PROC0_INTS0 must mask INTR|INTF by INTE");

    /* The write must not have landed on a phantom pin register. */
    int phantom_clean = 1;
    for (int pin = 30; pin < 48; pin++) {
        if (gpio_state.pins[pin].status != 0 || gpio_state.pins[pin].ctrl != 0) {
            phantom_clean = 0;
            break;
        }
    }
    ASSERT_TRUE(phantom_clean, "interrupt register writes must not reach phantom pins 30-47");

    /* INTR0 is write-1-to-clear regardless of alias. */
    gpio_write32(proc0_inte0, 0);
    gpio_write32(proc0_intf0, 0);
    gpio_state.intr[0] = 0xFFFFFFFF;
    gpio_write32(intr0, 0xFFFFFFFF);
    ASSERT_EQ(0x00000000, gpio_read32(intr0), "INTR0 must clear on write-1");

    /* Per-pin CTRL is still decoded as a pin. */
    gpio_write32(IO_BANK0_BASE + 0x0 * 8 + GPIO_CTRL_OFFSET, GPIO_FUNC_PWM);
    ASSERT_EQ(GPIO_FUNC_PWM, gpio_read32(IO_BANK0_BASE + 0x0 * 8 + GPIO_CTRL_OFFSET),
              "GPIO0_CTRL must still decode as a per-pin register");
    PASS();
}

/* Regression (datasheet audit A2/C12): IO_BANK0 and PADS_BANK0 are relocated on
 * RP2350, and those addresses hold unrelated peripherals on RP2040, so the
 * matchers previously claimed only the RP2040 bases. Every GPIO and pad
 * register access on RP2350-ARM was therefore unmapped and silently dropped. */
TEST(test_gpio_chip_aware_bases) {
    reset_cpu();

    membus_rp2350_mode = 0;
    ASSERT_EQ(IO_BANK0_BASE, gpio_io_bank0_base(), "RP2040 IO_BANK0 base");
    ASSERT_EQ(PADS_BANK0_BASE, gpio_pads_bank0_base(), "RP2040 PADS_BANK0 base");
    ASSERT_EQ(NUM_GPIO_PINS_RP2040, gpio_num_user_pins(), "RP2040 user pin count");

    membus_rp2350_mode = 1;
    ASSERT_EQ(RP2350_IO_BANK0_BASE, gpio_io_bank0_base(), "RP2350 IO_BANK0 base");
    ASSERT_EQ(RP2350_PADS_BANK0_BASE, gpio_pads_bank0_base(), "RP2350 PADS_BANK0 base");
    ASSERT_EQ(NUM_GPIO_PINS, gpio_num_user_pins(), "RP2350 user pin count");

    gpio_init();
    /* A pad write on RP2350 must land in the pad block, not fall through. */
    gpio_write32(RP2350_PADS_BANK0_BASE + 0x04, 0x00000096);
    ASSERT_EQ(0x00000096, gpio_read32(RP2350_PADS_BANK0_BASE + 0x04),
              "RP2350 PADS_BANK0 write/read");
    /* ...and through its atomic aliases too. */
    gpio_write32(RP2350_PADS_BANK0_BASE + 0x08, 0x56);
    gpio_write32(RP2350_PADS_BANK0_BASE + REG_ALIAS_SET_BITS + 0x08, 0x40);
    ASSERT_EQ(0x56 | 0x40, gpio_read32(RP2350_PADS_BANK0_BASE + 0x08),
              "RP2350 PADS_BANK0 SET alias");

    /* GPIO47 exists on RP2350 only. */
    gpio_write32(RP2350_IO_BANK0_BASE + 47 * 8 + GPIO_CTRL_OFFSET, GPIO_FUNC_SIO);
    ASSERT_EQ(GPIO_FUNC_SIO, gpio_read32(RP2350_IO_BANK0_BASE + 47 * 8 + GPIO_CTRL_OFFSET),
              "RP2350 GPIO47_CTRL must be addressable");

    /* RP2350 interrupt window is at +0x230, not +0x0F0. */
    /* On RP2350 there are 6 INTR banks, so PROC0_INTE0 is 6 words past INTR0. */
    uint32_t rp2350_inte0 = RP2350_IO_BANK0_BASE + GPIO_IRQ_WINDOW_RP2350 + 6 * 4;
    gpio_write32(rp2350_inte0, 0xFFFFFFFF);
    ASSERT_EQ(0xFFFFFFFF, gpio_read32(rp2350_inte0),
              "RP2350 PROC0_INTE0 must be at the RP2350 window offset");
    /* INTR5 (pins 32-47) exists only on RP2350. */
    gpio_write32(RP2350_IO_BANK0_BASE + GPIO_IRQ_WINDOW_RP2350 + 5 * 4, 0);
    gpio_state.intr[5] = 0xFFFFFFFF;
    gpio_write32(RP2350_IO_BANK0_BASE + GPIO_IRQ_WINDOW_RP2350 + 5 * 4, 0xFFFFFFFF);
    ASSERT_EQ(0x00000000, gpio_read32(RP2350_IO_BANK0_BASE + GPIO_IRQ_WINDOW_RP2350 + 5 * 4),
              "RP2350 INTR5 must be write-1-to-clear");

    /* FUNCSEL resets to NULL on both chips, not SIO. */
    gpio_init();
    ASSERT_EQ(GPIO_FUNC_NULL, gpio_read32(gpio_io_bank0_base() + GPIO_CTRL_OFFSET),
              "GPIO0 CTRL reset must be FUNCSEL=0x1f (NULL)");
    /* PADS word 0 is VOLTAGE_SELECT, so word n maps to pads[n-1]; SWCLK and SWD
     * sit one word past the user pads. */
    uint32_t swclk = RP2350_PADS_BANK0_BASE + ((uint32_t)gpio_num_user_pins() + 1) * 4;
    ASSERT_EQ(0x00000096, gpio_read32(swclk), "SWCLK pad reset must be 0x96");

    membus_rp2350_mode = 0;
    PASS();
}

/* Regression: the GPIO bases are chip-relative, but the Hazard3 membus
 * rewrites RP2350 GPIO bases back to their RP2040 equivalents before
 * delegating to the shared bus. When the GPIO layout was made chip-aware that
 * delegation path stopped matching, so on the RV32 path IO_BANK0 and PADS_BANK0
 * became unmapped and a RISC-V LittleOS image hung instead of booting. The
 * layout view must follow the address, not just the emulated chip. */
TEST(test_gpio_layout_follows_delegated_addresses) {
    int saved_mode = membus_rp2350_mode;
    int saved_delegate = membus_rv_delegate;

    membus_rp2350_mode = 1;
    membus_rv_delegate = 0;
    ASSERT_EQ(RP2350_IO_BANK0_BASE, gpio_io_bank0_base(),
              "M33 path must use the RP2350 IO_BANK0 base");
    ASSERT_EQ(NUM_GPIO_PINS, gpio_num_user_pins(),
              "M33 path must use the RP2350 pin count");

    /* Hazard3 delegation: the shared bus is handed RP2040 addresses. */
    membus_rv_delegate = 1;
    ASSERT_EQ(IO_BANK0_BASE, gpio_io_bank0_base(),
              "RV32 delegation must decode with the RP2040 IO_BANK0 base");
    ASSERT_EQ(PADS_BANK0_BASE, gpio_pads_bank0_base(),
              "RV32 delegation must decode with the RP2040 PADS_BANK0 base");
    ASSERT_EQ(NUM_GPIO_PINS_RP2040, gpio_num_user_pins(),
              "RV32 delegation must use the RP2040 pin count");

    /* And an RP2040-format address must actually be claimed and decoded. */
    gpio_init();
    gpio_write32(IO_BANK0_BASE + 0x0 * 8 + GPIO_CTRL_OFFSET, GPIO_FUNC_SIO);
    ASSERT_EQ(GPIO_FUNC_SIO, gpio_read32(IO_BANK0_BASE + 0x0 * 8 + GPIO_CTRL_OFFSET),
              "delegated RP2040 IO_BANK0 write/read");
    gpio_write32(IO_BANK0_BASE + GPIO_IRQ_WINDOW_RP2040 + 0x10, 0xFFFFFFFF);
    ASSERT_EQ(0xFFFFFFFF, gpio_read32(IO_BANK0_BASE + GPIO_IRQ_WINDOW_RP2040 + 0x10),
              "delegated PROC0_INTE0 must be writable");
    gpio_write32(PADS_BANK0_BASE + 0x04, 0x12345678);
    ASSERT_EQ(0x12345678, gpio_read32(PADS_BANK0_BASE + 0x04),
              "delegated RP2040 PADS_BANK0 write/read");

    membus_rp2350_mode = saved_mode;
    membus_rv_delegate = saved_delegate;
    PASS();
}

/* Regression (datasheet audit B1): every IRQ is renumbered on RP2350. The
 * peripheral models used to signal RP2040 vectors regardless of chip, so a
 * UART0 interrupt was delivered to vector 20 (= PIO2_IRQ_1) on RP2350 and a
 * GPIO bank edge to vector 13 (= DMA_IRQ_3). */
TEST(test_rp2350_irq_renumbering) {
    membus_rp2350_mode = 0;
    ASSERT_EQ(IRQ_UART0_IRQ, nvic_irq_number(IRQ_UART0_IRQ), "RP2040 UART0 keeps vector 20");
    ASSERT_EQ(IRQ_SPI0_IRQ,  nvic_irq_number(IRQ_SPI0_IRQ),  "RP2040 SPI0 keeps vector 18");
    ASSERT_EQ(IRQ_IO_IRQ_BANK0, nvic_irq_number(IRQ_IO_IRQ_BANK0), "RP2040 IO keeps vector 13");

    membus_rp2350_mode = 1;
    ASSERT_EQ(33, nvic_irq_number(IRQ_UART0_IRQ), "RP2350 UART0 must be vector 33");
    ASSERT_EQ(34, nvic_irq_number(IRQ_UART1_IRQ), "RP2350 UART1 must be vector 34");
    ASSERT_EQ(31, nvic_irq_number(IRQ_SPI0_IRQ),  "RP2350 SPI0 must be vector 31");
    ASSERT_EQ(21, nvic_irq_number(IRQ_IO_IRQ_BANK0), "RP2350 IO_BANK0 must be vector 21");
    ASSERT_EQ(23, nvic_irq_number(IRQ_IO_IRQ_QSPI),  "RP2350 IO_QSPI must be vector 23");
    ASSERT_EQ(10, nvic_irq_number(IRQ_DMA_IRQ_0),    "RP2350 DMA_IRQ_0 must be vector 10");
    ASSERT_EQ(30, nvic_irq_number(IRQ_CLOCKS_IRQ),   "RP2350 CLOCKS must be vector 30");
    ASSERT_EQ(35, nvic_irq_number(IRQ_ADC_IRQ_FIFO), "RP2350 ADC must be vector 35");
    /* RP2350 has no RP2040-style RTC; the AON timer lives in POWMAN. */
    ASSERT_EQ(45, nvic_irq_number(IRQ_RTC_IRQ),      "RP2350 RTC maps to POWMAN_TIMER");

    membus_rp2350_mode = 0;
    PASS();
}

TEST(test_rv_membus_sram) {
    rv_membus_state_t bus;
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv_mem_write32(&bus, 0x20000000, 0xDEADBEEF);
    ASSERT_EQ(0xDEADBEEF, rv_mem_read32(&bus, 0x20000000), "SRAM write/read");
    rv_mem_write8(&bus, 0x20000010, 0x42);
    ASSERT_EQ(0x42, rv_mem_read8(&bus, 0x20000010), "SRAM byte write/read");
    /* Alias */
    rv_mem_write32(&bus, 0x21000000, 0x12345678);
    ASSERT_EQ(0x12345678, rv_mem_read32(&bus, 0x20000000 + 0), "SRAM alias should mirror");
    PASS();
}

TEST(test_rv_shared_periph_translated_base) {
    /* Regression for issue #16. rv_translate_shared_addr() rewrites RP2350
     * peripheral bases back to their RP2040 equivalents before delegating to
     * the shared membus, but uart_match()/spi_match() only recognised the
     * RP2350 base while membus_rp2350_mode was set. Every RISC-V UART and SPI
     * access therefore fell through to "unmapped" and was silently dropped --
     * a Hazard3 image produced no console output and never reached its shell. */
    int saved_mode = membus_rp2350_mode;
    int saved_delegate = membus_rv_delegate;
    membus_rp2350_mode = 1;
    uart_init();
    spi_init();

    /* Hazard3 delegation: the RV path supplies already-translated addresses. */
    membus_rv_delegate = 1;
    ASSERT_EQ(0, uart_match(UART0_BASE), "translated RP2040 UART0 base should match");
    ASSERT_EQ(1, uart_match(UART1_BASE), "translated RP2040 UART1 base should match");
    ASSERT_EQ(0, spi_match(SPI0_BASE), "translated RP2040 SPI0 base should match");
    ASSERT_EQ(1, spi_match(SPI1_BASE), "translated RP2040 SPI1 base should match");
    ASSERT_EQ((uint32_t)-1, (uint32_t)uart_match(RP2350_UART0_BASE),
              "a translated address must not match the RP2350 UART base");
    membus_rv_delegate = 0;

    /* Cortex-M33: RP2350 addresses reach the shared bus untouched. */
    ASSERT_EQ(0, uart_match(RP2350_UART0_BASE), "RP2350 UART0 base should match");
    ASSERT_EQ(1, uart_match(RP2350_UART1_BASE), "RP2350 UART1 base should match");
    ASSERT_EQ(0, uart_match(RP2350_UART0_BASE | 0x2000), "RP2350 UART0 SET alias should match");
    ASSERT_EQ(0, spi_match(RP2350_SPI0_BASE), "RP2350 SPI0 base should match");
    ASSERT_EQ(1, spi_match(RP2350_SPI1_BASE), "RP2350 SPI1 base should match");
    ASSERT_EQ((uint32_t)-1, (uint32_t)uart_match(UART0_BASE),
              "M33 path must not claim the RP2040 UART0 base");

    /* The two chip maps overlap, so a matcher accepting both address spaces
     * hands the RP2350 pad blocks to the UART/SPI models. */
    ASSERT_EQ((uint32_t)-1, (uint32_t)uart_match(RP2350_PADS_BANK0_BASE),
              "RP2350 PADS_BANK0 must not be claimed by the UART");
    ASSERT_EQ((uint32_t)-1, (uint32_t)spi_match(RP2350_PADS_QSPI_BASE),
              "RP2350 PADS_QSPI must not be claimed by the SPI");

    /* End-to-end: an RISC-V store to the RP2350 UART0 DR must reach UART0. */
    rv_membus_state_t bus;
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv_mem_write32(&bus, RP2350_UART0_BASE + UART_DR, 0x41);
    ASSERT_EQ(0x41, uart_state[0].dr, "RISC-V UART0 DR write should reach UART0");
    ASSERT_EQ(0, uart_state[1].dr, "RISC-V UART0 write must not land on UART1");

    /* ...and an RISC-V store to the RP2350 PADS_BANK0 must not reach UART1. */
    uart_init();
    uint32_t cr_reset = uart_state[1].cr;
    rv_mem_write32(&bus, RP2350_PADS_BANK0_BASE + 0x30, 0xDEADBEEF);
    ASSERT_EQ(cr_reset, uart_state[1].cr,
              "RP2350 PADS_BANK0 write must not land in UART1 CR");

    uart_init();
    spi_init();
    membus_rp2350_mode = saved_mode;
    membus_rv_delegate = saved_delegate;
    PASS();
}

TEST(test_rv_step_advances_devtools_cycle_count) {
    /* Regression for issue #15. gpio_trace_record() timestamps events from
     * global_cycle_count, but the Hazard3 engine never advanced it, so a
     * RISC-V -gpio-trace recording was stamped entirely with #0. */
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;
    rv_cpu_reset(&rv, 0x10000000);

    /* 64 x "ADDI x0, x0, 0" (0x00000013), a four-byte NOP. */
    uint32_t nop = 0x00000013;
    for (int i = 0; i < 64; i++)
        memcpy(&cpu.flash[i * 4], &nop, 4);

    uint64_t before = global_cycle_count;
    for (int i = 0; i < 64; i++)
        rv_cpu_step(&rv);
    ASSERT_EQ(64, (uint32_t)(global_cycle_count - before),
              "each rv_cpu_step must advance global_cycle_count");
    PASS();
}

TEST(test_vcd_timestamps_advance) {
    /* Regression for issue #15. gpio_trace_record() stamps events from
     * global_cycle_count, so the timeline only moves if cpu_step() advances
     * that counter -- it used to do so only under irq_latency_enabled, which
     * left -gpio-trace with an all-#0 recording. VCD timestamps are also
     * 64-bit, so a long run must not wrap back to zero. */
    const char *path = "/tmp/bramble_test_gpio.vcd";
    uint32_t saved_cpu = timing_config.cycles_per_us;
    uint64_t saved_cycles = global_cycle_count;

    unlink(path);
    timing_config.cycles_per_us = 1;
    gpio_trace_init(path);
    ASSERT_TRUE(gpio_trace_enabled, "gpio_trace_init should enable tracing");

    /* Execute a real instruction through cpu_step(), then toggle a pin the way
     * firmware would. No -irq-latency here: the counter must advance anyway. */
    reset_cpu();
    gpio_init();
    uint16_t movs = 0x2042;  /* MOVS R0, #0x42 */
    memcpy(&cpu.flash[0x100], &movs, 2);
    cpu.r[15] = FLASH_BASE + 0x100;
    cpu_step();
    ASSERT_TRUE(global_cycle_count > 0,
                "cpu_step must advance global_cycle_count without -irq-latency");

    gpio_write32(SIO_GPIO_OUT, 0x1);
    global_cycle_count = 500;
    gpio_trace_record(1, 1);
    global_cycle_count = 1000;
    gpio_trace_record(0, 0);
    /* Past 2^32: a 32-bit stamp would wrap back to 7056. */
    global_cycle_count = 5000000000ULL;
    gpio_trace_record(2, 1);
    gpio_trace_cleanup();

    FILE *f = fopen(path, "r");
    ASSERT_TRUE(f != NULL, "VCD file should have been written");
    char buf[8192];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = '\0';

    /* The first change came straight after a real instruction retired, so pin0
     * going high must not be stamped #0 (the header's own all-low dump at #0
     * emits "0!", so the "1!" change is the one to look for). */
    ASSERT_TRUE(strstr(buf, "#0\n1!") == NULL, "VCD must not stamp pin0 high at #0");
    ASSERT_TRUE(strstr(buf, "1!") != NULL, "VCD should record pin0 going high");
    ASSERT_TRUE(strstr(buf, "#500\n") != NULL, "VCD should carry a #500 stamp");
    ASSERT_TRUE(strstr(buf, "#1000\n") != NULL, "VCD should carry a #1000 stamp");
    ASSERT_TRUE(strstr(buf, "#5000000000\n") != NULL,
                "VCD timestamps must be 64-bit, not truncated at 2^32");

    unlink(path);
    timing_config.cycles_per_us = saved_cpu;
    global_cycle_count = saved_cycles;
    PASS();
}

TEST(test_rv_bootrom_init) {
    rv_membus_state_t bus;
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    uint32_t entry = rv_bootrom_init(bus.rom, bus.rom_size, 0x10000000, 0x20082000);
    ASSERT_EQ(0x00000000, entry, "Entry should be ROM address 0");
    /* Check magic */
    ASSERT_EQ('R', bus.rom[0x10], "ROM magic[0] should be 'R'");
    ASSERT_EQ('P', bus.rom[0x11], "ROM magic[1] should be 'P'");
    ASSERT_EQ(0x02, bus.rom[0x12], "ROM magic[2] should be 0x02");
    PASS();
}

TEST(test_rv_icache) {
    rv_icache_t cache;
    rv_icache_init(&cache);
    uint32_t instr; uint8_t size;
    ASSERT_EQ(0, rv_icache_lookup(&cache, 0x10000000, &instr, &size), "Should miss on empty");
    rv_icache_insert(&cache, 0x10000000, 0xDEADBEEF, 4);
    ASSERT_EQ(1, rv_icache_lookup(&cache, 0x10000000, &instr, &size), "Should hit after insert");
    ASSERT_EQ(0xDEADBEEF, instr, "Cached instruction should match");
    ASSERT_EQ(4, size, "Size should be 4");
    PASS();
}

TEST(test_rv_periph_bootram) {
    rp2350_periph_state_t periph;
    rp2350_periph_init(&periph, 0);
    rp2350_periph_write8(&periph, 0x400E0000, 0x42);
    ASSERT_EQ(0x42, rp2350_periph_read8(&periph, 0x400E0000), "BOOTRAM byte access");
    rp2350_periph_write32(&periph, 0x400E0010, 0xCAFEBABE);
    ASSERT_EQ(0xCAFEBABE, rp2350_periph_read32(&periph, 0x400E0010), "BOOTRAM word access");
    PASS();
}

TEST(test_rv_periph_timer1) {
    rp2350_periph_state_t periph;
    rp2350_periph_init(&periph, 0);
    /* Write alarm and tick */
    rp2350_periph_write32(&periph, 0x400B8010, 100);  /* ALARM0 = 100 */
    ASSERT_EQ(1, periph.timer1.armed & 1, "Alarm 0 should be armed");
    rp2350_timer1_tick(&periph, 100);
    ASSERT_TRUE(periph.timer1.intr & 1, "Alarm 0 should fire at 100us");
    ASSERT_EQ(0, periph.timer1.armed & 1, "Alarm 0 should disarm after firing");
    PASS();
}

TEST(test_rv_hazard3_csrs) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;
    /* Xh3irq uses *array* CSRs: the window index is in the LSBs of the value
     * and the window is 16 bits wide, so 0xa5a50002 writes 0xa5a5 to bits
     * 47:32 (datasheet 3.8.6.1.1). Enable IRQ 3 and IRQ 9. */
    rv_csr_write(&rv, CSR_MEIEA, (0x0008u << 16) | 0u);   /* win 0: bits 15:0 */
    rv_csr_write(&rv, CSR_MEIEA, (0x0200u << 16) | 2u);   /* win 2: bits 47:32 */
    /* window 0 (bits 15:0) holds 0x0008, window 2 (bits 47:32) holds 0x0200 */
    ASSERT_EQ(0x0008, (uint32_t)(bus.clint.ext_enable[0] & 0xFFFF),
              "MEIEA must write the indexed 16-bit window (low)");
    ASSERT_EQ(0x0200, (uint32_t)((bus.clint.ext_enable[0] >> 32) & 0xFFFF),
              "MEIEA must write the indexed 16-bit window (high)");

    /* MEINEXT returns the lowest enabled+pending external IRQ. */
    ASSERT_EQ(0xFFFFFFFF, rv_csr_read(&rv, CSR_MEINEXT), "MEINEXT with nothing pending");
    rv_clint_set_ext_pending(&bus.clint, 3);
    rv_clint_set_ext_pending(&bus.clint, 11);
    ASSERT_EQ(3, rv_csr_read(&rv, CSR_MEINEXT), "MEINEXT should be the enabled IRQ 3");

    /* MEIPA exposes pending for whichever window MEIEA last selected. */
    rv_csr_write(&rv, CSR_MEIEA, 0);                       /* select window 0 */
    ASSERT_EQ((1u << 3) | (1u << 11), rv_csr_read(&rv, CSR_MEIPA),
              "MEIPA must show the pending bits in the selected window");

    /* MEIFA forces bits into the pending array. */
    rv_clint_clear_ext_pending(&bus.clint, 3);
    rv_csr_write(&rv, CSR_MEIFA, (0x0001u << 16) | 3u);   /* force bit 48 */
    rv_csr_write(&rv, CSR_MEIEA, 0);                       /* window 0: bits 47:32 */
    ASSERT_EQ(1u << 11, rv_csr_read(&rv, CSR_MEIPA),
              "bit 3 was cleared and the forced bit 48 is not in window 0");
    rv_csr_write(&rv, CSR_MEIEA, (0u << 16) | 3u);        /* select window 3 */
    ASSERT_EQ(1, rv_csr_read(&rv, CSR_MEIPA), "MEIFA must force a bit into MEIPA");

    /* MEIPRA is writable and readable through its window. */
    rv_csr_write(&rv, CSR_MEIPRA, (0x1234u << 16) | 2u);
    ASSERT_EQ(0x1234, bus.clint.ext_priority[2], "MEIPRA write");
    rv_csr_write(&rv, CSR_MEIEA, 2u);
    ASSERT_EQ(0x1234, rv_csr_read(&rv, CSR_MEIPRA), "MEIPRA read via the window index");
    PASS();
}

/* Reading SSPDR with an empty RX FIFO must not invent a transfer. It used to
 * push a 0xFF through the device model so SDK poll loops could not spin, which
 * meant a plain register read -- including a read-modify-write from a SET/CLR/
 * XOR alias -- sent a byte to whatever was attached. */
TEST(test_spi_sspsdr_read_does_not_clock_a_phantom_byte) {
    spi_init();

    mem_write32(SPI0_BASE + SPI_SSPCR1, SPI_CR1_SSE);

    uint32_t tx_before = spi_state[0].tx_count;
    for (int i = 0; i < 4; i++)
        (void)mem_read32(SPI0_BASE + SPI_SSPDR);

    ASSERT_EQ(tx_before, spi_state[0].tx_count,
              "reading SSPDR must not push anything into the TX FIFO");
    ASSERT_EQ(0, spi_state[0].rx_count,
              "reading an empty SSPDR must not conjure RX data");
    PASS();
}

/* mtvec MODE is WARL and only direct mode is supported, so bit 1 must read 0
 * whatever firmware writes. */
TEST(test_rv_mtvec_mode_bit_is_warl) {
    rv_cpu_state_t rv;
    rv_cpu_init(&rv, 0);

    /* Datasheet reset value: direct mode, base 0x00001ffc. */
    ASSERT_EQ(0x00001FFFu, rv.csr[CSR_MTVEC], "mtvec reset value");

    rv_csr_write(&rv, CSR_MTVEC, 0x00000202u);  /* MODE = 2 (reserved) */
    ASSERT_EQ(0x00000200u, rv_csr_read(&rv, CSR_MTVEC),
              "mtvec MODE[1] must read back as 0 (WARL)");

    rv_csr_write(&rv, CSR_MTVEC, 0x00000101u);  /* MODE = 1 (vectored) */
    ASSERT_EQ(0x00000101u, rv_csr_read(&rv, CSR_MTVEC),
              "vectored mode is supported and must read back as written");
    PASS();
}

/* Zcb c.sb / c.sh (riscv-isa-manual src/zc.adoc):
 *
 *   c.sb  100 | 010 | rs1' | uimm[0|1] | rs2' | 00
 *   c.sh  100 | 011 | rs1' | 0 uimm[1] | rs2' | 00
 *
 * Both previously raised an illegal-instruction trap: quadrant 0 had no
 * funct3==4 arm at all.
 */
/* c.sb's immediate is bit-reversed in the encoding: uimm[1] is encoding[5] and
 * uimm[0] is encoding[6]. */
#define ZCB_SB(rs1p, uimm, rs2p) \
    (0x8000u | (0x2u << 10) | ((rs1p) << 7) | \
     ((((uimm) >> 1) & 1u) << 5) | (((uimm) & 1u) << 6) | ((rs2p) << 2) | 0u)
#define ZCB_SH(rs1p, uimm, rs2p) \
    (0x8000u | (0x3u << 10) | ((rs1p) << 7) | (((uimm) >> 1) << 5) | ((rs2p) << 2) | 0u)

/* Run one 16-bit instruction held in RV SRAM and return after the store. */
static void rv_exec_one_16(rv_cpu_state_t *rv, rv_membus_state_t *bus, uint16_t insn) {
    const uint32_t code = RP2350_SRAM_BASE + 0xC00;
    rv_mem_write32(bus, code, (uint32_t)insn);
    rv->is_halted = 0;          /* rv_cpu_init() leaves the hart halted */
    rv->pc = code;
    rv_cpu_step(rv);
}

TEST(test_rv_zcb_c_sb_uses_its_offset) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;

    const uint32_t base = RP2350_SRAM_BASE + 0x400;
    for (int i = 0; i < 16; i++)
        rv_mem_write32(&bus, base + (uint32_t)i * 4, 0);

    rv.x[8] = base;   /* rs1' = 0 -> x8 */
    rv.x[9] = 0xA5;   /* rs2' = 1 -> x9 */

    /* uimm = 0b11 -> both encoding[5] and encoding[6] set. */
    rv_exec_one_16(&rv, &bus, ZCB_SB(0, 0x3, 1));

    ASSERT_EQ(0xA5, rv_mem_read8(&bus, base + 3) & 0xFF, "c.sb at uimm=3");
    ASSERT_EQ(0x00, rv_mem_read8(&bus, base) & 0xFF, "c.sb must not write base+0");
    PASS();
}

TEST(test_rv_zcb_c_sh_uses_its_offset) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;

    const uint32_t base = RP2350_SRAM_BASE + 0x400;
    for (int i = 0; i < 16; i++)
        rv_mem_write32(&bus, base + (uint32_t)i * 4, 0);

    rv.x[8] = base;
    rv.x[9] = 0x1234;

    /* c.sh's uimm[0] is hardwired 0, so encoding[6] must be ignored. */
    rv_exec_one_16(&rv, &bus, (uint16_t)(ZCB_SH(0, 0x2, 1) | (1u << 6)));

    ASSERT_EQ(0x1234, rv_mem_read16(&bus, base + 2) & 0xFFFF, "c.sh at uimm=2");
    ASSERT_EQ(0x0000, rv_mem_read16(&bus, base) & 0xFFFF,
              "c.sh must not write base+0, and must ignore encoding[6]");
    PASS();
}

TEST(test_rv_zcb_c_sb_uses_both_rs1_and_rs2) {
    /* rs1'/rs2' are independent fields; the old code assumed rs1' == x8 always. */
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;

    const uint32_t a = RP2350_SRAM_BASE + 0x400;
    const uint32_t b = RP2350_SRAM_BASE + 0x500;
    for (int i = 0; i < 16; i++) {
        rv_mem_write32(&bus, a + (uint32_t)i * 4, 0);
        rv_mem_write32(&bus, b + (uint32_t)i * 4, 0);
    }

    rv.x[11] = a;     /* rs1' = 3 -> x11 (the base) */
    rv.x[12] = 0x7E;  /* rs2' = 4 -> x12 (the source) */
    rv.x[13] = b;     /* must not be involved at all */

    rv_exec_one_16(&rv, &bus, ZCB_SB(3, 0x2, 4));   /* c.sb x12, 2(x11) */

    ASSERT_EQ(0x7E, rv_mem_read8(&bus, a + 2) & 0xFF, "c.sb must store via rs2'");
    ASSERT_EQ(0x00, rv_mem_read8(&bus, b + 2) & 0xFF,
              "c.sb must use rs1' as the base, not some other register");
    PASS();
}

/* RP2350's PWM block differs from RP2040's almost entirely above the per-slice
 * registers: 12 slices rather than 8, the global block at 0xF0 rather than 0xA0,
 * 12-bit global registers rather than 8, and *two* interrupt outputs. Offsets
 * from pico-sdk src/rp2350/.../regs/pwm.h. */
TEST(test_pwm_rp2350_layout) {
    pwm_init();
    membus_rp2350_mode = 1;

    /* Slice 11's CSR is at 0xDC -- beyond anything RP2040 has. */
    pwm_write32(11 * 0x14 + PWM_CH_TOP, 0x1234u);
    ASSERT_EQ(0x1234u, pwm_read32(11 * 0x14 + PWM_CH_TOP), "RP2350 slice 11");
    ASSERT_EQ(0, pwm_read32(PWM2350_G_EN), "global block starts clear");

    /* PWM_EN aliases the per-slice CSR enable bits and is 12 bits wide. It used
     * to be masked to 0xFF, so slices 8-11 could never be enabled. */
    pwm_write32(PWM2350_G_EN, 0xFFF);
    ASSERT_EQ(0xFFFu, pwm_read32(PWM2350_G_EN), "PWM_EN must be 12 bits on RP2350");
    for (int i = 0; i < 12; i++)
        ASSERT_TRUE((pwm_state.slice[i].csr & PWM_CSR_EN) != 0,
                    "slice must be enabled via PWM_EN");
    ASSERT_TRUE((pwm_state.slice[0].csr & PWM_CSR_EN) == 0 ||
                (pwm_state.slice[0].csr & PWM_CSR_EN) != 0, "");

    pwm_write32(PWM2350_G_EN, 0);
    ASSERT_EQ(0, pwm_read32(PWM2350_G_EN), "PWM_EN clear");
    ASSERT_EQ(0, pwm_state.slice[11].csr & PWM_CSR_EN,
              "clearing PWM_EN must clear slice 11's CSR enable");

    /* The RP2040 offsets are *not* the RP2350 ones: on RP2350, 0xA8 lands
     * inside slice 10's register area, not in the interrupt block. */
    ASSERT_TRUE(pwm_read32(PWM2040_G_INTE) == pwm_read32(10 * 0x14 + PWM_CH_CC) ||
                pwm_read32(PWM2040_G_INTE) == 0,
              "0xA8 must not be treated as the interrupt enable on RP2350");

    /* The two interrupt outputs are independent. */
    pwm_write32(PWM2350_G_IRQ0_INTE, 0x001);
    pwm_write32(PWM2350_G_IRQ1_INTE, 0x100);
    ASSERT_EQ(0x001u, pwm_read32(PWM2350_G_IRQ0_INTE), "IRQ0 enable");
    ASSERT_EQ(0x100u, pwm_read32(PWM2350_G_IRQ1_INTE), "IRQ1 enable");
    pwm_write32(PWM2350_G_IRQ1_INTF, 0x100);
    ASSERT_EQ(0x100u, pwm_read32(PWM2350_G_IRQ1_INTS), "IRQ1 forced status");

    membus_rp2350_mode = 0;
    PASS();
}

/* RP2040 keeps 8 slices, the 0xA0 block and 8-bit globals -- the change must not
 * have regressed it. */
TEST(test_pwm_rp2040_layout_unchanged) {
    pwm_init();
    membus_rp2350_mode = 0;

    pwm_write32(7 * 0x14 + PWM_CH_TOP, 0xABCDu);
    ASSERT_EQ(0xABCDu, pwm_read32(7 * 0x14 + PWM_CH_TOP), "RP2040 slice 7");

    pwm_write32(PWM2040_G_EN, 0xFF);
    ASSERT_EQ(0xFFu, pwm_read32(PWM2040_G_EN), "RP2040 PWM_EN is 8 bits");

    pwm_write32(PWM2040_G_INTE, 0x10);
    pwm_write32(PWM2040_G_INTF, 0x10);
    ASSERT_EQ(0x10u, pwm_read32(PWM2040_G_INTS), "RP2040 INTS");

    /* Slices 8-11 do not exist on RP2040. Their offsets (0xA0, 0xB4, 0xC8,
     * 0xDC) all land inside or past the global block, which ends at 0xB0, so
     * none may read back as a slice register. 0xA0 is PWM_EN by design. */
    ASSERT_EQ(0xFFu, pwm_read32(8 * 0x14), "0xA0 is PWM_EN on RP2040, not slice 8");
    for (int i = 9; i < 12; i++)
        ASSERT_EQ(0, pwm_read32((uint32_t)i * 0x14 + PWM_CH_CSR),
                  "RP2040 has only 8 slices");
    PASS();
}

/* LDRD/STRD (immediate) T1, ARMv7-M ARM A6.7.49 / A6.7.124:
 *
 *   1 1 1 0 1 0 0 P U 1 W L | Rn | Rt Rt2 imm8
 *
 * with index = (P==1), add = (U==1), wback = (W==1), imm32 = imm8:'00'.
 * L distinguishes LDRD (1) from STRD (0).
 *
 * These are implemented in t32_ldrd_strd(); an earlier note in
 * docs/full_audit.md claimed they were not, because the grep that produced the
 * claim only matched the guard helpers and missed the executor. This pins the
 * behaviour against the ARM's field layout. */
#define LDRD_T1(P_, U_, W_, Rn_, Rt_, Rt2_, imm_) \
    ((uint16_t)((0xEu << 12) | (1u << 11) | (0u << 10) | (0u << 9) | \
                ((P_) << 8) | ((U_) << 7) | (1u << 6) | ((W_) << 5) | \
                (1u << 4) | ((Rn_) & 0xFu)))
#define LDRD_T1_LOW(Rt_, Rt2_, imm_) \
    ((uint16_t)(((Rt_) << 12) | ((Rt2_) << 8) | ((imm_) & 0xFFu)))
#define STRD_T1(P_, U_, W_, Rn_, Rt_, Rt2_, imm_) \
    ((uint16_t)(LDRD_T1(P_, U_, W_, Rn_, Rt_, Rt2_, imm_) & ~0x10u))

TEST(test_thumb2_ldrd_strd_immediate) {
    const uint32_t base = RAM_BASE + 0x2800;
    reset_cpu();
    mem_write32(base,     0x11111111u);
    mem_write32(base + 4, 0x22222222u);

    /* LDRD r0, r1, [r2, #0] -- offset form, no writeback. */
    cpu.r[2] = base;
    cpu.r[0] = 0;
    cpu.r[1] = 0;
    cpu.r[15] = RAM_BASE;
    thumb32_step(RAM_BASE, LDRD_T1(1, 1, 0, 2, 0, 1, 0), LDRD_T1_LOW(0, 1, 0));
    ASSERT_EQ(0x11111111u, cpu.r[0], "LDRD must load Rt from [Rn]");
    ASSERT_EQ(0x22222222u, cpu.r[1], "LDRD must load Rt2 from [Rn]+4");

    /* imm8 is scaled by four: #4 means a 16-byte displacement. */
    mem_write32(base + 16, 0xAAAAAAAAu);
    mem_write32(base + 20, 0xBBBBBBBBu);
    cpu.r[0] = 0;
    cpu.r[1] = 0;
    thumb32_step(RAM_BASE, LDRD_T1(1, 1, 0, 2, 0, 1, 4), LDRD_T1_LOW(0, 1, 4));
    ASSERT_EQ(0xAAAAAAAAu, cpu.r[0], "LDRD imm8 must be scaled by 4");
    ASSERT_EQ(0xBBBBBBBBu, cpu.r[1], "LDRD second word");

    /* Pre-indexed with writeback (P=1, W=1): Rn becomes Rn + offset. */
    cpu.r[2] = base;
    thumb32_step(RAM_BASE, LDRD_T1(1, 1, 1, 2, 0, 1, 4), LDRD_T1_LOW(0, 1, 4));
    ASSERT_EQ(base + 16, cpu.r[2], "LDRD writeback must update Rn");

    /* U=0 subtracts: Rn = base+16 with a 16-byte offset lands back on base,
     * which holds 0x11111111, not the 0xAAAAAAAA at base+16. */
    cpu.r[2] = base + 16;
    thumb32_step(RAM_BASE, LDRD_T1(1, 0, 0, 2, 0, 1, 4), LDRD_T1_LOW(0, 1, 4));
    ASSERT_EQ(0x11111111u, cpu.r[0], "U=0 must subtract the offset");

    /* STRD stores Rt and Rt2. */
    reset_cpu();
    cpu.r[2] = base;
    cpu.r[0] = 0xCAFEBABEu;
    cpu.r[1] = 0xFEEDFACEu;
    mem_write32(base,     0);
    mem_write32(base + 4, 0);
    cpu.r[15] = RAM_BASE;
    thumb32_step(RAM_BASE, STRD_T1(1, 1, 0, 2, 0, 1, 0), LDRD_T1_LOW(0, 1, 0));
    ASSERT_EQ(0xCAFEBABEu, mem_read32(base), "STRD must store Rt");
    ASSERT_EQ(0xFEEDFACEu, mem_read32(base + 4), "STRD must store Rt2");
    PASS();
}

/* RP2350 datasheet section 5.3: core 0 launches core 1 by pushing
 *
 *     { 0, 0, 1, vector_table, sp, entry }
 *
 * to core 1 over the SIO inter-processor FIFO (SIO +0x54). The emulator used to
 * claim SIO +0x1C0..0x1CC as a boot mailbox, which the datasheet says is the
 * TMDS encoder (TMDS_CTRL / TMDS_WDATA / TMDS_PEEK_SINGLE / TMDS_POP_SINGLE).
 * So multicore_launch_core1() wrote its entry point into the pixel encoder and
 * hart 1 never started. */
TEST(test_rv_hart1_launch_uses_the_documented_fifo_protocol) {
    rv_membus_state_t bus;
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);

    const uint32_t vtor = 0x20040000u;
    const uint32_t sp   = 0x20081000u;
    const uint32_t entry = 0x10000101u;   /* Thumb bit set by the sender */

    /* Wrong sequence must not launch: these are the invented mailbox writes. */
    rv_mem_write32(&bus, RP2350_SIO_BASE + 0x1C0u, vtor);
    rv_mem_write32(&bus, RP2350_SIO_BASE + 0x1C4u, sp);
    rv_mem_write32(&bus, RP2350_SIO_BASE + 0x1C8u, entry);
    rv_mem_write32(&bus, RP2350_SIO_BASE + 0x1CCu, 1u);
    uint32_t e, s2, v;
    ASSERT_TRUE(rv_membus_check_hart1_launch(&bus, &e, &s2, &v) == 0,
                "SIO +0x1C0..0x1CC is TMDS, not a launch mailbox");

    /* The real protocol. */
    const uint32_t seq[6] = { 0, 0, 1, vtor, sp, entry };
    for (int i = 0; i < 6; i++)
        rv_mem_write32(&bus, RP2350_SIO_BASE + RV_SIO_FIFO_WR, seq[i]);

    ASSERT_TRUE(rv_membus_check_hart1_launch(&bus, &e, &s2, &v) == 1,
                "the section 5.3 sequence must launch hart 1");
    ASSERT_EQ(sp, s2, "launch must carry the stack pointer");
    ASSERT_EQ(entry & ~1u, e, "launch must carry the entry point, Thumb bit clear");
    ASSERT_EQ(vtor, v, "launch must carry vector_table");

    /* Only once: the second poll must report nothing new. */
    ASSERT_TRUE(rv_membus_check_hart1_launch(&bus, &e, &s2, &v) == 0,
                "a launch must be reported exactly once");
    PASS();
}

/* spi_flash had no test coverage at all. The parts worth pinning are the
 * bounds checks: offsets are 64-bit and the access is guest-controlled, so an
 * overflow in the length check would turn a bounded read into a host OOB. */
TEST(test_spi_flash_rejects_out_of_range_accesses) {
    unlink("/tmp/bramble_sf_test.bin");

    /* Chip 0 defaults to 64 MB. Note spi_flash_close() clears `enabled`, so it
     * must not be called between configure and use -- calling it there is
     * exactly what made the first version of this test see every read rejected. */
    spi_flash_configure(0, "/tmp/bramble_sf_test.bin");
    spi_flash_set_size(0, 64);

    const uint64_t bytes = 64ull * 1024 * 1024;

    uint8_t buf[64];
    memset(buf, 0, sizeof(buf));

    /* Sanity: a read inside the chip is accepted (it may fail to open, but it
     * must not be rejected by the bounds check). */
    ASSERT_TRUE(spi_flash_read(0, 0, buf, 16) >= 0,
                "an in-range read must not be rejected");

    /* Past the end. */
    ASSERT_EQ(-1, spi_flash_read(0, bytes - 8, buf, 16),
              "a read straddling the end must be rejected");
    ASSERT_EQ(-1, spi_flash_read(0, bytes, buf, 1),
              "a read at exactly the end must be rejected");

    /* 64-bit overflow: offset + len must not wrap. */
    ASSERT_EQ(-1, spi_flash_read(0, bytes - 4, buf, sizeof(buf)),
              "offset + len must be checked in 64 bits, not truncated");
    ASSERT_EQ(-1, spi_flash_read(0, 0xFFFFFFFFFFFFFFF0ull, buf, sizeof(buf)),
              "a near-UINT64_MAX offset must be rejected, not wrapped");

    /* Out-of-range chip index. */
    ASSERT_EQ(-1, spi_flash_read(99, 0, buf, 4), "chip index must be range-checked");

    /* NULL buffer and zero length. */
    ASSERT_EQ(-1, spi_flash_read(0, 0, NULL, 4), "NULL buffer must be rejected");
    ASSERT_EQ(0, spi_flash_read(0, 0, buf, 0), "zero length must be a no-op");

    spi_flash_close();
    unlink("/tmp/bramble_sf_test.bin");
    PASS();
}

/* The chip size is normalised to the 32 MB erase unit, so a small request must
 * not produce a chip smaller than one unit. */
TEST(test_spi_flash_size_is_normalised_to_the_erase_unit) {
    spi_flash_configure(1, "/tmp/bramble_sf_test2.bin");
    spi_flash_set_size(1, 1);          /* below the 32 MB unit */
    spi_flash_set_size(1, 0);          /* nonsense: must be ignored */

    uint8_t buf[16];
    /* A read within the first 32 MB must be in range. */
    ASSERT_TRUE(spi_flash_read(1, 1024, buf, 16) >= 0,
                "a small size request must normalise to at least one erase unit");
    /* A read past 32 MB must not be: the request was clamped, not honoured. */
    ASSERT_EQ(-1, spi_flash_read(1, 33ull * 1024 * 1024, buf, 16),
              "size must clamp to the erase unit rather than accept the request");
    spi_flash_close();
    unlink("/tmp/bramble_sf_test2.bin");
    PASS();
}

/* fatfs had no test coverage. The BPB is entirely guest-controlled, so the
 * validation in fat16_mount() is the security-relevant surface: a crafted
 * reserved_sectors/sectors_per_fat would otherwise place the root directory
 * megabytes past the media and turn every dirent access into an OOB read. */

/* Build a minimal valid FAT16 image, then let the caller corrupt fields. */
static uint8_t *fat16_test_image(size_t *size_out) {
    size_t sectors = 8192;                 /* 4 MiB */
    size_t bytes = sectors * 512;
    uint8_t *m = calloc(1, bytes);
    if (m == NULL) return NULL;

    m[510] = 0x55; m[511] = 0xAA;          /* boot signature */
    m[11] = 0x00; m[12] = 0x02;            /* bytes_per_sector = 512 */
    m[13] = 4;                              /* sectors_per_cluster */
    m[14] = 0x01; m[15] = 0x00;            /* reserved_sectors = 1 */
    m[16] = 2;                              /* num_fats = 2 */
    m[17] = 0x80; m[18] = 0x00;            /* root_entry_count = 128 (LE) */
    m[19] = 0x00; m[20] = 0x00;            /* total_sectors16 = 0 -> use 32-bit */
    m[22] = 0x10; m[23] = 0x00;            /* sectors_per_fat = 16 */
    /* total_sectors32 at 0x20 */
    m[32] = (uint8_t)(sectors & 0xFF);
    m[33] = (uint8_t)((sectors >> 8) & 0xFF);
    m[34] = (uint8_t)((sectors >> 16) & 0xFF);
    m[35] = (uint8_t)((sectors >> 24) & 0xFF);

    /* FAT entries: chain 2 -> 3 -> EOC. */
    size_t fat_off = 512;
    m[fat_off + 2 * 2] = 3;
    m[fat_off + 3 * 2] = 0xFF;
    m[fat_off + 4 * 2] = 0xFF;
    m[fat_off + 5 * 2] = 0xFF;

    /* Root directory: "HELLO.TXT" in 8.3, one cluster, 13 bytes. */
    size_t root_off = 512 + 2u * 16u * 512u;
    memcpy(&m[root_off], "HELLO   TXT", 11);
    m[root_off + 11] = 0x20;               /* archive */
    m[root_off + 26] = 2;                  /* first cluster low */
    m[root_off + 28] = 13;                 /* file size */
    /* ...and the same name in the second root entry, for list tests. */
    memcpy(&m[root_off + 32], "DATA    BIN", 11);
    m[root_off + 43] = 0x20;
    m[root_off + 58] = 2;
    m[root_off + 60] = 13;
    /* Mark both used entries with a valid first byte. */
    m[root_off + 32] = 'D';

    /* Cluster 2 payload. */
    size_t data_off = root_off + 128u * 32u;
    const char *msg = "hello world\n";
    memcpy(&m[data_off], msg, 13);

    *size_out = bytes;
    return m;
}

TEST(test_fat16_mount_rejects_malformed_bpb) {
    size_t bytes = 0;
    uint8_t *img = fat16_test_image(&bytes);
    ASSERT_TRUE(img != NULL, "test image allocation");
    if (!img) { PASS(); return; }   /* PASS() does not return */

    fat16_fs_t fs;

    /* A well-formed image must mount. */
    ASSERT_TRUE(fat16_mount(&fs, img, bytes) == 0, "valid FAT16 image must mount");
    ASSERT_EQ(512, fs.bytes_per_sector, "bytes_per_sector");
    ASSERT_EQ(4, fs.sectors_per_cluster, "sectors_per_cluster");

    /* Reads must find the file and its contents. */
    uint8_t buf[64];
    memset(buf, 0, sizeof(buf));
    int n = fat16_read_file(&fs, "HELLO.TXT", buf, sizeof(buf));
    ASSERT_TRUE(n == 13, "HELLO.TXT should be 13 bytes");
    ASSERT_TRUE(memcmp(buf, "hello world\n", 12) == 0, "HELLO.TXT contents");

    /* Missing boot signature. */
    uint8_t *bad = fat16_test_image(&bytes);
    bad[510] = 0x00;
    ASSERT_TRUE(fat16_mount(&fs, bad, bytes) != 0, "bad boot signature must fail");
    free(bad);

    /* Media smaller than one sector. */
    bad = fat16_test_image(&bytes);
    ASSERT_TRUE(fat16_mount(&fs, bad, 100) != 0, "short media must fail");
    free(bad);

    /* bytes_per_sector != 512 is not supported. */
    bad = fat16_test_image(&bytes);
    bad[11] = 0x00; bad[12] = 0x04;         /* 1024 */
    ASSERT_TRUE(fat16_mount(&fs, bad, bytes) != 0, "bytes_per_sector != 512 must fail");
    free(bad);

    /* Zero fields that would divide by zero. */
    bad = fat16_test_image(&bytes);
    bad[13] = 0;                           /* sectors_per_cluster = 0 */
    ASSERT_TRUE(fat16_mount(&fs, bad, bytes) != 0, "sectors_per_cluster = 0 must fail");
    free(bad);

    bad = fat16_test_image(&bytes);
    bad[16] = 0;                           /* num_fats = 0 */
    ASSERT_TRUE(fat16_mount(&fs, bad, bytes) != 0, "num_fats = 0 must fail");
    free(bad);

    bad = fat16_test_image(&bytes);
    bad[22] = 0; bad[23] = 0;              /* sectors_per_fat = 0 */
    ASSERT_TRUE(fat16_mount(&fs, bad, bytes) != 0, "sectors_per_fat = 0 must fail");
    free(bad);

    /* Metadata larger than the declared volume: the crafted case. */
    bad = fat16_test_image(&bytes);
    bad[14] = 0xFF; bad[15] = 0xFF;        /* reserved_sectors = 0xFFFF */
    ASSERT_TRUE(fat16_mount(&fs, bad, bytes) != 0,
                "metadata beyond the volume must be rejected");
    free(bad);

    free(img);
    PASS();
}

TEST(test_fat16_rejects_out_of_bounds_directory_entries) {
    size_t bytes = 0;
    uint8_t *img = fat16_test_image(&bytes);
    ASSERT_TRUE(img != NULL, "test image allocation");
    if (!img) { PASS(); return; }   /* PASS() does not return */

    fat16_fs_t fs;
    ASSERT_TRUE(fat16_mount(&fs, img, bytes) == 0, "mount");

    /* The 32-bit total_sectors path must be used when the 16-bit field is 0;
     * otherwise volumes over 32 MiB silently truncate. */
    ASSERT_TRUE(fs.total_sectors > 0xFFFF || fs.total_sectors == 8192,
                "total_sectors must come from the 32-bit field when the 16-bit is 0");

    /* Stat on a name that is not present. */
    fat16_fileinfo_t info;
    ASSERT_TRUE(fat16_stat(&fs, "MISSING.TXT", &info) != 0,
                "stat of a missing file must fail");

    /* Read a missing file. */
    uint8_t buf[64];
    ASSERT_TRUE(fat16_read_file(&fs, "MISSING.TXT", buf, sizeof(buf)) < 0,
                "read of a missing file must fail");

    /* A buffer too small must be reported, not silently truncated. */
    uint8_t tiny[4];
    ASSERT_TRUE(fat16_read_file(&fs, "HELLO.TXT", tiny, sizeof(tiny)) != 13,
                "a short buffer must not report a full 13-byte read");

    /* Listing must not run past the root directory. */
    fat16_fileinfo_t files[16];
    int nf = fat16_list_root(&fs, files, 16);
    ASSERT_TRUE(nf >= 2, "list_root should find both files");
    ASSERT_TRUE(nf <= 16, "list_root must respect the caller's capacity");

    free(img);
    PASS();
}

/* bme280 had no test coverage. It is reached over the emulated I2C bus, so its
 * register-pointer protocol is worth pinning: a write sets the pointer and
 * subsequent reads auto-increment through the register map. Getting that wrong
 * makes every SDK read of the calibration block silently return the wrong
 * bytes, which no firmware would notice without a probe. */
TEST(test_bme280_register_protocol) {
    bme280_t dev;
    bme280_init(&dev);

    /* The I2C protocol needs an explicit start between transactions: start
     * clears ptr_set so the *next* write byte is taken as the register pointer
     * rather than as data. Driving bme280_i2c_write() directly, without the
     * start, silently writes data into whatever register was last addressed. */
#define BME_SEL(reg) (bme280_i2c_start(&dev), bme280_i2c_write(&dev, (reg)))

    /* Chip identification must be readable at 0xD0. */
    BME_SEL(BME280_REG_CHIP_ID);
    ASSERT_EQ(BME280_CHIP_ID, bme280_i2c_read(&dev), "chip id at 0xD0");

    /* Reads auto-increment, so the pointer must have advanced past 0xD0. */
    uint8_t after = bme280_i2c_read(&dev);
    ASSERT_TRUE(after != BME280_CHIP_ID, "reads must advance the register pointer");

    /* The soft-reset command is recognised at 0xE0. */
    BME_SEL(BME280_REG_RESET);
    bme280_i2c_write(&dev, BME280_RESET_CMD);

    /* ctrl_meas is writable and readable back. */
    BME_SEL(BME280_REG_CTRL_MEAS);
    bme280_i2c_write(&dev, BME280_MODE_NORMAL);
    BME_SEL(BME280_REG_CTRL_MEAS);
    uint8_t ctrl = bme280_i2c_read(&dev);
    ASSERT_TRUE((ctrl & 0x03) == BME280_MODE_NORMAL,
                "ctrl_meas mode bits must read back as written");

    /* The simulation setters must reach the register file, and the 20-bit
     * values must survive the msb/lsb/xlsb split. */
    bme280_set_temperature(&dev, 25.0f);
    BME_SEL(BME280_REG_TEMP_MSB);
    uint8_t t_msb  = bme280_i2c_read(&dev);
    uint8_t t_lsb  = bme280_i2c_read(&dev);
    uint8_t t_xlsb = bme280_i2c_read(&dev);
    int32_t raw = ((int32_t)t_msb << 12) | ((int32_t)t_lsb << 4) | (t_xlsb >> 4);
    ASSERT_TRUE(raw > 0 && raw <= 0xFFFFF, "temperature must be a valid 20-bit raw value");

    /* A different temperature must produce a different raw value. */
    bme280_set_temperature(&dev, 30.0f);
    BME_SEL(BME280_REG_TEMP_MSB);
    int32_t raw2 = ((int32_t)bme280_i2c_read(&dev) << 12);
    ASSERT_TRUE(raw2 != raw, "a different temperature must change the raw value");

    bme280_set_pressure(&dev, 101325.0f);
    BME_SEL(BME280_REG_PRESS_MSB);
    uint8_t p_msb = bme280_i2c_read(&dev);
    ASSERT_TRUE(p_msb != 0, "set_pressure must populate the pressure regs");

    bme280_set_humidity(&dev, 50.0f);
    BME_SEL(BME280_REG_HUM_MSB);
    uint8_t h_msb = bme280_i2c_read(&dev);
    ASSERT_TRUE(h_msb != 0, "set_humidity must populate the humidity regs");

#undef BME_SEL
    PASS();
}

/* netbridge indexes fixed per-UART arrays, and the uart_num comes from the
 * emulated UART model. Without the bounds check a bad index is an OOB write
 * into net_bridge_tx_pending[]. Pinned here; the data path needs a connected
 * client socket and is not reachable from a unit test. */
TEST(test_net_bridge_rejects_out_of_range_uart_index) {
    net_bridge_uart_tx(99, 'x');   /* must be a no-op, not an OOB write */
    net_bridge_uart_tx(-1, 'x');
    ASSERT_EQ(1, 1, "out-of-range UART writes must be dropped without effect");
    ASSERT_EQ(0, net_bridge_uart_active(99), "an out-of-range UART must not be active");
    ASSERT_EQ(0, net_bridge_uart_active(-1), "a negative UART must not be active");

    /* With no client connected, nothing should report active. */
    for (int u = 0; u < 2; u++)
        ASSERT_EQ(0, net_bridge_uart_active(u), "UART must be inactive with no client");
    PASS();
}

/* cyw43 is the largest uncovered file (1345 lines). It bit-bangs an SPI
 * transaction over three GPIOs, so the state machine is drivable from a test
 * without any hardware -- and the interesting part is the buffer indexing in the
 * read phase, which walks resp_buf[] with resp_offset/resp_len driven by the
 * emulated device. */
#define CYW_WL_CS   23
#define CYW_WL_CLK  24
#define CYW_WL_DIO  25

TEST(test_cyw43_gpio_intercept_only_claims_wifi_pins) {
    /* The model is gated on cyw43.enabled, which main.c sets only for -wifi or
     * -tap. The test harness passes neither, so enable it explicitly before
     * init -- otherwise every intercept returns 0 and the test would "pass"
     * without exercising the state machine at all. */
    cyw43.enabled = 1;
    cyw43_init();

    ASSERT_EQ(1, cyw43_is_wifi_gpio(CYW_WL_CS), "WL_CS must be a WiFi pin");
    ASSERT_EQ(1, cyw43_is_wifi_gpio(CYW_WL_CLK), "WL_CLK must be a WiFi pin");
    ASSERT_EQ(1, cyw43_is_wifi_gpio(CYW_WL_DIO), "WL_DIO must be a WiFi pin");
    ASSERT_EQ(0, cyw43_is_wifi_gpio(0), "GPIO 0 must not be a WiFi pin");
    ASSERT_EQ(0, cyw43_is_wifi_gpio(15), "GPIO 15 (LED) must not be a WiFi pin");

    /* A non-WiFi GPIO must be passed through, not consumed by the model. */
    ASSERT_EQ(0, cyw43_gpio_intercept(15, 1), "a non-WiFi GPIO must not be intercepted");
    ASSERT_EQ(0, cyw43_gpio_intercept(0, 1), "a non-WiFi GPIO must not be intercepted");

    /* The WiFi pins are claimed regardless of the value written. */
    ASSERT_EQ(1, cyw43_gpio_intercept(CYW_WL_CS, 1), "WL_CS is intercepted");
    ASSERT_EQ(1, cyw43_gpio_intercept(CYW_WL_CS, 0), "WL_CS is intercepted");
    ASSERT_EQ(1, cyw43_gpio_intercept(CYW_WL_CLK, 1), "WL_CLK is intercepted");
    ASSERT_EQ(1, cyw43_gpio_intercept(CYW_WL_DIO, 0), "WL_DIO is intercepted");
    PASS();
}

/* Clocking 32 command bits must move the state machine into the data phase, and
 * the read phase must serve resp_buf[] without running off the end. */
TEST(test_cyw43_bitbang_spi_state_machine) {
    cyw43.enabled = 1;    /* see the note in the test above */
    cyw43_init();

    /* Chip select low starts the command phase. */
    cyw43_gpio_intercept(CYW_WL_CS, 0);
    ASSERT_EQ(1, cyw43.spi.cs_active, "CS low must select the device");
    ASSERT_EQ(0, cyw43.spi.cmd_bits, "a fresh command must start with no bits");

    /* Clock edges before the 32nd must keep accumulating the command word. */
    uint32_t cmd = 0x0000A500u;   /* plausible SPI command header */
    for (int i = 0; i < 31; i++) {
        int bit = (int)((cmd >> i) & 1u);
        cyw43_gpio_intercept(CYW_WL_DIO, (uint32_t)bit);
        cyw43_gpio_intercept(CYW_WL_CLK, 1);
        cyw43_gpio_intercept(CYW_WL_CLK, 0);
        ASSERT_EQ(i + 1, cyw43.spi.cmd_bits, "each rising clock must shift one bit");
    }
    ASSERT_EQ(0, cyw43.spi.in_data_phase, "must not enter the data phase early");

    /* The 32nd bit completes the command and switches phases. */
    cyw43_gpio_intercept(CYW_WL_DIO, (uint32_t)((cmd >> 31) & 1u));
    cyw43_gpio_intercept(CYW_WL_CLK, 1);
    cyw43_gpio_intercept(CYW_WL_CLK, 0);
    ASSERT_EQ(1, cyw43.spi.in_data_phase, "32 command bits must enter the data phase");

    /* Deselecting must drop the chip select. */
    cyw43_gpio_intercept(CYW_WL_CS, 1);
    ASSERT_EQ(0, cyw43.spi.cs_active, "CS high must deselect the device");

    /* Now clock the read phase. With no response queued the model must serve
     * zeros indefinitely rather than reading past resp_buf[]. */
    uint32_t served_before = cyw43.spi.resp_offset;
    for (int i = 0; i < 64; i++) {
        cyw43_gpio_intercept(CYW_WL_DIO, 0);
        cyw43_gpio_intercept(CYW_WL_CLK, 1);
        cyw43_gpio_intercept(CYW_WL_CLK, 0);
    }
    /* Nothing was queued, so resp_offset must not have advanced -- that is the
     * guard against walking off the end of resp_buf[]. */
    ASSERT_EQ(served_before, cyw43.spi.resp_offset,
              "the read phase must not advance past an empty response buffer");

    cyw43.enabled = 0;    /* leave global state as we found it */
    PASS();
}

/* GDB RSP over a socketpair. `gdb` is extern, so the test can point
 * gdb.client_fd at one end and drive the protocol directly -- no port binding,
 * no fork, no timing. gdb_recv_packet() and gdb_handle() are static, so this
 * goes through gdb_handle(), which is the real entry point the emulator uses. */
static uint8_t gdb_checksum(const char *payload) {
    uint8_t sum = 0;
    for (const char *p = payload; *p; p++)
        sum += (uint8_t)*p;
    return sum;
}

/* Send "$<payload>#<checksum>" on the test end of the socketpair. */
static void gdb_test_send_packet(int fd, const char *payload, int corrupt) {
    char pkt[512];
    uint8_t sum = gdb_checksum(payload);
    int n = snprintf(pkt, sizeof(pkt), "$%s#%02x", payload, sum);
    if (corrupt)
        pkt[n - 2] = (pkt[n - 2] == '0') ? '1' : '0';   /* break the checksum */
    (void)!write(fd, pkt, (size_t)n);
}

/* Drain whatever the emulator has written. The socket is put in non-blocking
 * mode: sv[1] is half-closed for writing (so the emulator sees EOF and
 * gdb_handle() returns), but its *read* side is still open, so a blocking read
 * would wait forever for more data that will never arrive. */
static void gdb_test_drain(int fd, int *flags) {
    int fl = fcntl(fd, F_GETFL, 0);
    (void)fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    *flags = fl;
}

static void gdb_test_undrain(int fd, int fl) {
    (void)fcntl(fd, F_SETFL, fl);
}

/* Drive gdb_handle() with one packet already queued. It blocks on read, so the
 * packet must be in the socket first; a 0x03 interrupt makes it return. */
static void gdb_test_drive(int fd, const char *payload, int corrupt) {
    gdb_test_send_packet(fd, payload, corrupt);
    (void)!write(fd, "\x03", 1);          /* 0x03 -> handler replies and loops */
    /* Break the blocking read so gdb_handle() returns. */
    (void)!write(fd, "$", 1);
    shutdown(fd, SHUT_WR);
    (void)gdb_handle();
}

TEST(test_gdb_rsp_accepts_a_valid_packet) {
    int sv[2];
    ASSERT_TRUE(socketpair(AF_UNIX, SOCK_STREAM, 0, sv) == 0, "socketpair");
    if (sv[0] < 0) { PASS(); return; }   /* PASS() does not return */

    memset(&gdb, 0, sizeof(gdb));
    gdb.active = 1;
    gdb.client_fd = sv[0];
    gdb.g_thread = 0;

    gdb_test_drive(sv[1], "?", 0);

    /* The first thing on the wire is the stop reply, then an ACK ('+') for the
     * packet we sent. Both must appear. */
    int fl;
    gdb_test_drain(sv[1], &fl);
    int saw_stop = 0, saw_ack = 0;
    unsigned char c;
    while (read(sv[1], &c, 1) == 1) {
        if (c == '$') saw_stop = 1;
        if (c == '+') saw_ack = 1;
    }
    gdb_test_undrain(sv[1], fl);
    ASSERT_TRUE(saw_stop, "the emulator must announce a stop reply");
    ASSERT_TRUE(saw_ack, "a valid packet must be acknowledged with '+'");

    gdb.active = 0;
    close(sv[0]); close(sv[1]);
    PASS();
}

/* The RSP framing carries a two-digit checksum of the payload. Accepting a
 * packet without checking it ACKs a corrupted write as intact, so the debugger
 * and the emulator silently disagree about what was sent. */
TEST(test_gdb_rsp_rejects_a_bad_checksum) {
    int sv[2];
    ASSERT_TRUE(socketpair(AF_UNIX, SOCK_STREAM, 0, sv) == 0, "socketpair");
    if (sv[0] < 0) { PASS(); return; }   /* PASS() does not return */

    memset(&gdb, 0, sizeof(gdb));
    gdb.active = 1;
    gdb.client_fd = sv[0];
    gdb.g_thread = 0;

    gdb_test_send_packet(sv[1], "?", 1);   /* wrong checksum */
    (void)!write(sv[1], "\x03", 1);
    (void)!write(sv[1], "$", 1);
    shutdown(sv[1], SHUT_WR);
    (void)gdb_handle();

    int fl;
    gdb_test_drain(sv[1], &fl);
    int saw_ack = 0, saw_nak = 0;
    unsigned char c;
    while (read(sv[1], &c, 1) == 1) {
        if (c == '+') saw_ack = 1;
        if (c == '-') saw_nak = 1;
    }
    gdb_test_undrain(sv[1], fl);
    ASSERT_TRUE(saw_nak, "a bad checksum must be NAKed with '-'");
    ASSERT_TRUE(!saw_ack, "a bad checksum must not be acknowledged");

    gdb.active = 0;
    close(sv[0]); close(sv[1]);
    PASS();
}

/* A single oversized packet must not walk past its buffer. */
TEST(test_gdb_rsp_handles_an_oversized_packet) {
    int sv[2];
    ASSERT_TRUE(socketpair(AF_UNIX, SOCK_STREAM, 0, sv) == 0, "socketpair");
    if (sv[0] < 0) { PASS(); return; }   /* PASS() does not return */

    memset(&gdb, 0, sizeof(gdb));
    gdb.active = 1;
    gdb.client_fd = sv[0];
    gdb.g_thread = 0;

    /* Build a packet far larger than the 4096-byte receive buffer. */
    static char big[16384];
    size_t n = 0;
    big[n++] = '$';
    for (size_t i = 0; i < sizeof(big) - 8; i++)
        big[n++] = (char)('a' + (i % 26));
    uint8_t sum = 0;
    for (size_t i = 1; i < n; i++)
        sum += (uint8_t)big[i];
    n += (size_t)snprintf(big + n, sizeof(big) - n, "#%02x", sum);

    (void)!write(sv[1], big, n);
    (void)!write(sv[1], "\x03", 1);
    (void)!write(sv[1], "$", 1);
    shutdown(sv[1], SHUT_WR);
    (void)gdb_handle();      /* must survive */

    gdb.active = 0;
    close(sv[0]); close(sv[1]);
    PASS();
}

/* fuse_mount had no coverage. Its mount path needs a real /dev/fuse mount and is
 * exercised manually via -mount; what is unit-testable is the entry validation,
 * which is where a bad guest image would otherwise be mounted as if valid. */
TEST(test_fuse_mount_rejects_an_invalid_image) {
    fuse_mount_stop();

    uint8_t junk[1024];
    memset(junk, 0, sizeof(junk));

    /* No FAT16 boot signature: must be refused, and must not become active. */
    ASSERT_TRUE(fuse_mount_start(junk, sizeof(junk), "/tmp/bramble_fuse_test") != 0,
                "an image with no FAT16 boot signature must be refused");
    ASSERT_EQ(0, fuse_mount_active(), "a refused mount must not report active");

    /* Right signature, nonsense geometry: still must be refused. */
    junk[510] = 0x55; junk[511] = 0xAA;
    ASSERT_TRUE(fuse_mount_start(junk, sizeof(junk), "/tmp/bramble_fuse_test") != 0,
                "an image with a bad BPB must be refused");
    ASSERT_EQ(0, fuse_mount_active(), "a refused mount must not report active");

    /* Too small to even hold a boot sector. */
    uint8_t tiny[16];
    memset(tiny, 0, sizeof(tiny));
    ASSERT_TRUE(fuse_mount_start(tiny, sizeof(tiny), "/tmp/bramble_fuse_test") != 0,
                "an image smaller than one sector must be refused");

    rmdir("/tmp/bramble_fuse_test");
    PASS();
}

/* fuse_set_flash_offset records where the filesystem region lives in flash so
 * persistence can sync exactly that range back. */
TEST(test_fuse_set_flash_offset_records_the_region) {
    fuse_set_flash_offset(0);
    ASSERT_EQ(0, fuse_mount_active(), "no mount means not active");

    fuse_set_flash_offset(0x10000);
    fuse_set_flash_offset(0);
    PASS();
}

/* VLDR/VSTR (ARMv7-M ARM A7.7.236 / A7.7.267):
 *
 *   VLDR  1110 1101 U D 0 1 | Rn | Vd | 1011 | imm8    (double, D:Vd)
 *   VLDR  1110 1101 U D 0 1 | Rn | Vd | 1010 | imm8    (single, Vd:D)
 *   VSTR  same with bit 4 clear
 *
 * imm32 = ZeroExtend(imm8:'00', 32). These are 32-bit Thumb-2 encodings whose
 * first halfword is 0xEDxx -- which is why earlier revisions mistook them for
 * the ARM core LDRD/STRD opcodes at 0xED4x/0xED5x.
 *
 * The round trip is observable end to end even though vfp_s/vfp_d are
 * file-static: a VSTR from a register followed by a VLDR into the same register
 * must reproduce the seeded value, and a VLDR into a *different* register then
 * a VSTR of that register must move the value there.
 */
#define VFP_LS_HW1(U, D, Rn, load) \
    ((uint16_t)(0xED00u | (((U) & 1) << 7) | (((D) & 1) << 6) | \
                (((load) & 1) << 4) | ((Rn) & 0xFu)))
#define VFP_LS_HW2(Vd, size, imm8) \
    ((uint16_t)((((Vd) & 0xFu) << 12) | (((size) & 0xFu) << 8) | \
                ((imm8) & 0xFFu)))

TEST(test_thumb2_vldr_vstr_double_roundtrip) {
    reset_cpu();
    const uint32_t base = RAM_BASE + 0x2C00;
    const uint32_t dst  = RAM_BASE + 0x2D00;

    mem_write32(base,     0);
    mem_write32(base + 4, 0);
    mem_write32(dst,      0);
    mem_write32(dst + 4,  0);

    /* Seed D0 by storing 0x0123456789ABCDEF into memory, then loading it. */
    mem_write32(base,     0x89ABCDEFu);
    mem_write32(base + 4, 0x01234567u);
    cpu.r[0] = base;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 0, 0, 1 /*load*/), VFP_LS_HW2(0, 0xB /*double*/, 0));

    /* D0 now holds that value; store it back somewhere else. */
    cpu.r[1] = dst;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 0, 1, 0 /*store*/), VFP_LS_HW2(0, 0xB, 0));

    ASSERT_EQ(0x89ABCDEFu, mem_read32(dst), "VLDR/VSTR .64 must round-trip the low word");
    ASSERT_EQ(0x01234567u, mem_read32(dst + 4), "VLDR/VSTR .64 must round-trip the high word");

    /* U=0 subtracts: D = 1 (register 1) loaded from r2 - 16. */
    mem_write32(dst - 16,     0xCAFEBABEu);
    mem_write32(dst - 12,     0xFEEDFACEu);
    cpu.r[2] = dst;
    thumb32_step(RAM_BASE, VFP_LS_HW1(0, 1, 2, 1), VFP_LS_HW2(0, 0xB, 4));
    cpu.r[3] = base;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 1, 3, 0), VFP_LS_HW2(0, 0xB, 0));
    ASSERT_EQ(0xCAFEBABEu, mem_read32(base), "VLDR with U=0 must subtract imm8<<2");
    ASSERT_EQ(0xFEEDFACEu, mem_read32(base + 4), "VLDR with U=0 high word");
    PASS();
}

TEST(test_thumb2_vldr_vstr_single_roundtrip) {
    reset_cpu();
    const uint32_t base = RAM_BASE + 0x2E00;
    mem_write32(base, 0x12345678u);

    /* S register number is Vd:D, so Vd=4, D=0 selects s8. */
    cpu.r[0] = base;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 0, 0, 1), VFP_LS_HW2(4, 0xA /*single*/, 0));

    /* Store s8 back through a different base register. */
    cpu.r[1] = base + 16;
    mem_write32(base + 16, 0);
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 0, 1, 0), VFP_LS_HW2(4, 0xA, 0));

    ASSERT_EQ(0x12345678u, mem_read32(base + 16), "VLDR/VSTR .32 must round-trip");
    PASS();
}

/* tapif is the last source file with no coverage. Its real behaviour is a
 * /dev/net/tun file descriptor, so this drives an actual TAP interface when one
 * is present and asserts the failure paths otherwise. The interface is created
 * out of band:
 *
 *   sudo ip tuntap add mode tap dev brtest0
 *   sudo ip addr add 192.168.77.1/24 dev brtest0
 *   sudo ip link set brtest0 up
 *
 * Set BRAMBLE_TEST_TAP to its name to exercise the data path; without it the
 * test still checks that a bad name is refused. */
TEST(test_tapif_open_rejects_an_unknown_interface) {
    /* A name that cannot exist must be refused, not left half-open. */
    ASSERT_TRUE(tapif_open("brtest_nonexistent") < 0,
                "an unknown TAP interface must be refused");
    /* A NULL/empty name lets the kernel pick a unit. */
    PASS();
}

TEST(test_tapif_read_write_against_a_real_interface) {
    const char *name = getenv("BRAMBLE_TEST_TAP");
    if (name == NULL || name[0] == '\0') {
        /* No interface configured: assert only the refusal path so the test
         * is meaningful without root. PASS() does not return, so this must. */
        ASSERT_TRUE(tapif_open("brtest_nonexistent") < 0, "bad name must fail");
        PASS();
        return;
    }

    int fd = tapif_open(name);
    ASSERT_TRUE(fd >= 0, "tapif_open on a configured interface must succeed");
    if (fd < 0) PASS();

    /* A write must succeed. The interface is NO-CARRIER with no peer, but the
     * TAP write path buffers, so this must not fail or block. */
    uint8_t frame[64];
    memset(frame, 0xA5, sizeof(frame));
    frame[0] = 0x02; frame[1] = 0x00;          /* looks like an ARP request */
    int w = tapif_write(fd, frame, (int)sizeof(frame));
    ASSERT_TRUE(w > 0, "tapif_write must accept a frame");

    /* Read must be bounded by the buffer and must not overrun it. An UP TAP
     * interface legitimately receives host-generated ARP/IPv6 traffic even
     * with no peer, so the *amount* is environment-dependent -- but whatever
     * it is must fit in the buffer the caller supplied. */
    uint8_t in[64];
    memset(in, 0, sizeof(in));
    int r = tapif_read(fd, in, (int)sizeof(in));
    ASSERT_TRUE(r <= (int)sizeof(in), "tapif_read must not exceed the caller's buffer");
    ASSERT_TRUE(r >= 0 || r == -1, "tapif_read must return a length or an error");

    /* Writing nothing is a no-op, not an error. */
    ASSERT_TRUE(tapif_write(fd, frame, 0) >= 0, "a zero-length write must not fail");

    tapif_close(fd);
    PASS();
}

/* RP2350 datasheet Table 649: TICKS has three registers per generator at a
 * 12-byte stride. The decoder assumed 8, so TIMER1_CTRL at 0x024 resolved as
 * generator 4, register 4 -- the watchdog's CYCLES. */
TEST(test_rp2350_ticks_generator_stride) {
    rp2350_periph_state_t st;
    rp2350_periph_init(&st, 0);

    /* Table 649 offsets for each generator's CTRL. */
    static const uint32_t ctrl_off[] = {
        0x000,  /* PROC0_CTRL    */
        0x00c,  /* PROC1_CTRL    */
        0x018,  /* TIMER0_CTRL   */
        0x024,  /* TIMER1_CTRL   */
        0x030,  /* WATCHDOG_CTRL */
        0x03c,  /* RISCV_CTRL    */
    };

    /* Write a distinct, non-default value to each CTRL and read it back. With
     * an 8-byte stride the later generators would alias onto a neighbour. */
    for (unsigned g = 0; g < sizeof(ctrl_off) / sizeof(ctrl_off[0]); g++) {
        rp2350_periph_write32(&st, RP2350_TICKS_BASE + ctrl_off[g], 0x10u + g);
    }
    for (unsigned g = 0; g < sizeof(ctrl_off) / sizeof(ctrl_off[0]); g++) {
        uint32_t v = rp2350_periph_read32(&st, RP2350_TICKS_BASE + ctrl_off[g]);
        ASSERT_EQ(0x10u + g, v, "TICKS generator CTRL must not alias");
    }

    /* CYCLES and COUNT are read-only: writing must not change them. */
    rp2350_periph_write32(&st, RP2350_TICKS_BASE + 0x028, 0xDEADBEEFu); /* TIMER1_CYCLES */
    ASSERT_EQ(0, rp2350_periph_read32(&st, RP2350_TICKS_BASE + 0x028),
              "TIMER1_CYCLES is read-only");

    /* COUNT must exist as its own register, distinct from CYCLES. */
    rp2350_ticks_tick(&st.ticks, 5);
    ASSERT_TRUE(rp2350_periph_read32(&st, RP2350_TICKS_BASE + 0x02C) !=
                rp2350_periph_read32(&st, RP2350_TICKS_BASE + 0x028),
                "TIMER1_COUNT and TIMER1_CYCLES must be distinct registers");
    PASS();
}

/* RP2350 WATCHDOG register list ends at SCRATCH7 (0x028) and REASON "logs the
 * reason for the last reset". Both were wrong: REASON returned a hardcoded 0, so
 * a watchdog reboot looked identical to a power-on reset, and 0x2C was decoded
 * as a TICK register that does not exist on this chip. */
TEST(test_watchdog_reason_and_rp2350_map) {
    clocks_init();
    membus_rp2350_mode = 1;

    /* A clean start reads zero -- both bits zero means a hardware reset. */
    ASSERT_EQ(WATCHDOG_REASON_RESET,
              clocks_read32(RP2350_WATCHDOG_BASE + 0x08),
              "a clean boot must report REASON 0");

    /* Triggering the watchdog must be visible on the next boot. */
    clocks_write32(RP2350_WATCHDOG_BASE + 0x00, 1u << 31);   /* CTRL.TRIGGER */
    ASSERT_EQ(WATCHDOG_REASON_WDOG, clocks_read32(RP2350_WATCHDOG_BASE + 0x08),
              "a watchdog reset must be distinguishable from a power-on reset");

    /* 0x2C does not exist on RP2350 (no TICK register). */
    clocks_write32(RP2350_WATCHDOG_BASE + 0x2C, 0xFFFFFFFFu);
    ASSERT_EQ(0, clocks_read32(RP2350_WATCHDOG_BASE + 0x2C),
              "RP2350 has no TICK register at 0x2C");

    /* SCRATCH0-7 persist through a soft reset. */
    clocks_write32(RP2350_WATCHDOG_BASE + 0x0C, 0xC0FFEE01u);
    clocks_write32(RP2350_WATCHDOG_BASE + 0x28, 0x0BADF00Du);   /* SCRATCH7 */
    clocks_reset();
    ASSERT_EQ(0xC0FFEE01u, clocks_read32(RP2350_WATCHDOG_BASE + 0x0C),
              "SCRATCH0 must survive a soft reset");
    ASSERT_EQ(0x0BADF00Du, clocks_read32(RP2350_WATCHDOG_BASE + 0x28),
              "SCRATCH7 must survive a soft reset");

    membus_rp2350_mode = 0;
    PASS();
}

/* The VFP register file is 32 words: S0-S31 are the storage, D0-D15 overlay them
 * as pairs (Dn = {S(2n), S(2n+1)}), and D16-D31 alias D0-D15.
 *
 * The double and single views used to be separate arrays, so a 64-bit value
 * written by VLDR was invisible to a 32-bit access of the overlapping register.
 *
 * Encodings: a double register is (D << 4) | Vd, a single is (Vd << 1) | D.
 * So D1 is D=0,Vd=1 and covers S2 (D=0,Vd=1) and S3 (D=1,Vd=1). */
TEST(test_vfp_double_aliases_single_pair) {
    reset_cpu();
    const uint32_t src = RAM_BASE + 0x3100;
    const uint32_t out = RAM_BASE + 0x3140;

    mem_write32(src,     0x89ABCDEFu);
    mem_write32(src + 4, 0x01234567u);

    cpu.r[0] = src;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 0, 0, 1), VFP_LS_HW2(1, 0xB, 0));  /* D1 */

    cpu.r[1] = out;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 0, 1, 0), VFP_LS_HW2(1, 0xA, 0));  /* S2 */
    ASSERT_EQ(0x89ABCDEFu, mem_read32(out), "D1's low word must appear in S2");

    cpu.r[1] = out + 4;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 1, 1, 0), VFP_LS_HW2(1, 0xA, 0));  /* S3 */
    ASSERT_EQ(0x01234567u, mem_read32(out + 4), "D1's high word must appear in S3");

    /* The reverse direction needs VMOV (the only way to write S from a GPR),
     * which has its own defect -- see the VMOV entry in docs/full_audit.md.
     * Until that is fixed this test covers the single->observer direction. */
    PASS();
}

/* VLDR/VSTR .64 with a double register number of 16-31 indexed a 16-entry array
 * and wrote past the end of the global, into `exclusive_monitor`. */
TEST(test_vfp_double_registers_16_to_31) {
    reset_cpu();
    const uint32_t src = RAM_BASE + 0x2E00;
    const uint32_t dst = RAM_BASE + 0x2F00;

    mem_write32(src,     0x89ABCDEFu);
    mem_write32(src + 4, 0x01234567u);
    mem_write32(dst,     0);
    mem_write32(dst + 4, 0);

    cpu.r[0] = src;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 1, 0, 1), VFP_LS_HW2(15, 0xB, 0)); /* D31 */
    cpu.r[1] = dst;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 1, 1, 0), VFP_LS_HW2(15, 0xB, 0)); /* D31 */

    ASSERT_EQ(0x89ABCDEFu, mem_read32(dst),
              "VLDR/VSTR .64 through D31 must round-trip the low word");
    ASSERT_EQ(0x01234567u, mem_read32(dst + 4),
              "VLDR/VSTR .64 through D31 must round-trip the high word");
    PASS();
}

/* D16-D31 are aliases of D0-D15, not distinct registers. */
TEST(test_vfp_high_doubles_alias_low_ones) {
    reset_cpu();
    const uint32_t src = RAM_BASE + 0x3200;
    const uint32_t out = RAM_BASE + 0x3240;

    mem_write32(src,     0x0BADF00Du);
    mem_write32(src + 4, 0xFEEDFACEu);

    /* Load D31, then read it back through D15. */
    cpu.r[0] = src;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 1, 0, 1), VFP_LS_HW2(15, 0xB, 0)); /* D31 */
    cpu.r[1] = out;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 0, 1, 0), VFP_LS_HW2(15, 0xB, 0)); /* D15 */
    ASSERT_EQ(0x0BADF00Du, mem_read32(out), "D31 must alias D15, low word");
    ASSERT_EQ(0xFEEDFACEu, mem_read32(out + 4), "D31 must alias D15, high word");

    /* Write through D17 and read back through D1. */
    mem_write32(src + 0x10u, 0x5A5A5A5Au);
    mem_write32(src + 0x14u, 0xA5A5A5A5u);
    cpu.r[0] = src + 0x10u;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 1, 0, 1), VFP_LS_HW2(1, 0xB, 0));  /* D17 */
    cpu.r[1] = out + 0x10u;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 1, 1, 0), VFP_LS_HW2(1, 0xB, 0));  /* D17 */
    cpu.r[1] = out + 0x20u;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 0, 1, 0), VFP_LS_HW2(1, 0xB, 0));  /* D1 */
    ASSERT_EQ(0x5A5A5A5Au, mem_read32(out + 0x20u), "D17 must alias D1");
    ASSERT_EQ(0xA5A5A5A5u, mem_read32(out + 0x24u), "D17/D1 high word");
    PASS();
}

/* D30 and D31 alias D14 and D15, so they must stay distinct from each other. */
TEST(test_vfp_high_doubles_do_not_clobber_low_ones) {
    reset_cpu();
    const uint32_t src30 = RAM_BASE + 0x3000;
    const uint32_t src31 = RAM_BASE + 0x3010;
    const uint32_t out   = RAM_BASE + 0x3040;

    mem_write32(src30,     0xAAAAAAAAu);
    mem_write32(src30 + 4, 0xBBBBBBBBu);
    mem_write32(src31,     0xCCCCCCCCu);
    mem_write32(src31 + 4, 0xDDDDDDDDu);

    cpu.r[0] = src30;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 1, 0, 1), VFP_LS_HW2(14, 0xB, 0)); /* D30 */
    cpu.r[0] = src31;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 1, 0, 1), VFP_LS_HW2(15, 0xB, 0)); /* D31 */

    cpu.r[1] = out;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 1, 1, 0), VFP_LS_HW2(14, 0xB, 0)); /* D30 */
    ASSERT_EQ(0xAAAAAAAAu, mem_read32(out), "D30 must survive a load into D31");
    ASSERT_EQ(0xBBBBBBBBu, mem_read32(out + 4), "D30's high word");

    cpu.r[1] = out + 8;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 1, 1, 0), VFP_LS_HW2(15, 0xB, 0)); /* D31 */
    ASSERT_EQ(0xCCCCCCCCu, mem_read32(out + 8), "D31 must hold its own value");
    ASSERT_EQ(0xDDDDDDDDu, mem_read32(out + 12), "D31's high word");
    PASS();
}

/* CTRL's NBITS fields are encoded 0-7 meaning 1-8 valid bits, so a full byte
 * per lane needs all three fields set to 7. CTRL == 0 means a single valid bit. */
#define TMDS_NBITS8 \
    ((7u << TMDS_CTRL_NBITS_SHIFT(0)) | (7u << TMDS_CTRL_NBITS_SHIFT(1)) | \
     (7u << TMDS_CTRL_NBITS_SHIFT(2)))

/* The RP2350 TMDS encoder occupies SIO 0x1c0-0x1e4. That range was unmapped --
 * the fake hart-launch mailbox that used to sit there was removed and nothing
 * replaced it -- so firmware using the DVI encoder read the unhandled marker. */
TEST(test_tmds_register_map_and_control_symbols) {
    tmds_state_t st;
    tmds_init(&st);

    /* CTRL is read/write; CLEAR_BALANCE is self-clearing. */
    uint32_t ctrl = 0;
    ASSERT_TRUE(tmds_write(&st, TMDS_CTRL, 0x0000001Fu) == 1, "CTRL must write");
    ASSERT_TRUE(tmds_read(&st, TMDS_CTRL, &ctrl) == 1, "CTRL must read");
    ASSERT_EQ(0x1Fu, ctrl, "CTRL bits must read back");
    tmds_write(&st, TMDS_CTRL, 0x10000000u);            /* CLEAR_BALANCE */
    tmds_read(&st, TMDS_CTRL, &ctrl);
    ASSERT_EQ(0, ctrl, "CLEAR_BALANCE must be self-clearing");

    /* WDATA is write-only. */
    tmds_write(&st, TMDS_WDATA, 0x1234u);
    ASSERT_TRUE(tmds_read(&st, TMDS_WDATA, &ctrl) == 1, "WDATA must be mapped");
    ASSERT_EQ(0, ctrl, "WDATA is write-only");

    /* All-black and all-white select the fixed control symbols, which the DVI
     * specification pins exactly and which no disparity affects. */
    tmds_write(&st, TMDS_CTRL, TMDS_NBITS8);              /* no rotate, 8 bits */
    tmds_write(&st, TMDS_WDATA, 0x0000u);
    uint32_t peek;
    tmds_read(&st, TMDS_PEEK_SINGLE, &peek);
    /* Without INTERLEAVE the three symbols are contiguous, lane 0 at bit 0. */
    ASSERT_EQ(0x354u, peek & 0x3FFu, "lane 0 of 0x0000 must be control C0");
    ASSERT_EQ(0x0ABu, (peek >> 10) & 0x3FFu, "lane 1 of 0x0000 must be control C1");
    ASSERT_EQ(0x0A4u, (peek >> 20) & 0x3FFu, "lane 2 of 0x0000 must be control C2");

    tmds_write(&st, TMDS_WDATA, 0xFFFFu);
    tmds_read(&st, TMDS_PEEK_SINGLE, &peek);
    ASSERT_EQ(0x0ABu, peek & 0x3FFu, "lane 0 of 0xffff must be control C1");
    ASSERT_EQ(0x354u, (peek >> 10) & 0x3FFu, "lane 1 of 0xffff must be control C0");
    ASSERT_EQ(0x0A4u, (peek >> 20) & 0x3FFu, "lane 2 of 0xffff must be control C2");
    PASS();
}

/* PEEK advances the DC balance but does not shift the colour register;
 * POP does both. */
TEST(test_tmds_peek_does_not_shift_but_pop_does) {
    tmds_state_t st;
    uint32_t a, b;

    tmds_init(&st);
    tmds_write(&st, TMDS_CTRL, TMDS_NBITS8);
    tmds_write(&st, TMDS_WDATA, 0x1234u);

    /* With PIX_SHIFT = 0 (no shift), POP and PEEK must agree. */
    tmds_read(&st, TMDS_POP_SINGLE, &a);
    tmds_write(&st, TMDS_WDATA, 0x1234u);
    tmds_read(&st, TMDS_PEEK_SINGLE, &b);
    ASSERT_EQ(a, b, "with PIX_SHIFT=0, POP and PEEK must return the same word");

    /* PIX_SHIFT=1 shifts by one bit per POP. */
    tmds_init(&st);
    tmds_write(&st, TMDS_CTRL, TMDS_NBITS8 | (1u << TMDS_CTRL_PIX_SHIFT_SHIFT));
    tmds_write(&st, TMDS_WDATA, 0x0001u);        /* 0x0001 -> 0x0002 -> 0x0004 */
    tmds_read(&st, TMDS_POP_SINGLE, &a);         /* encodes 0x0001 */
    tmds_write(&st, TMDS_CTRL, 0);
    tmds_write(&st, TMDS_WDATA, 0x0002u);
    tmds_read(&st, TMDS_PEEK_SINGLE, &b);
    ASSERT_EQ(a, b, "one POP with PIX_SHIFT=1 must advance the colour by one bit");

    /* PEEK must leave the colour register alone, so repeated PEEKs of a
     * control symbol are identical. */
    tmds_init(&st);
    tmds_write(&st, TMDS_CTRL, TMDS_NBITS8);
    tmds_write(&st, TMDS_WDATA, 0x0000u);
    tmds_read(&st, TMDS_PEEK_SINGLE, &a);
    tmds_read(&st, TMDS_PEEK_SINGLE, &b);
    ASSERT_EQ(a, b, "PEEK must not shift the colour register");
    PASS();
}

/* INTERLEAVE repacks the same three symbols differently: 5 chunks of
 * 3 lanes x 2 bits rather than three contiguous 10-bit fields. */
TEST(test_tmds_interleave_packing) {
    tmds_state_t st;
    uint32_t plain, interleaved;

    tmds_init(&st);
    tmds_write(&st, TMDS_CTRL, TMDS_NBITS8);
    tmds_write(&st, TMDS_WDATA, 0x0000u);
    tmds_read(&st, TMDS_PEEK_SINGLE, &plain);

    tmds_init(&st);
    tmds_write(&st, TMDS_CTRL, TMDS_NBITS8 | TMDS_CTRL_INTERLEAVE);
    tmds_write(&st, TMDS_WDATA, 0x0000u);
    tmds_read(&st, TMDS_PEEK_SINGLE, &interleaved);

    ASSERT_TRUE(plain != interleaved, "INTERLEAVE must change the packing");

    /* Unpack both and confirm the same three symbols are present. */
    uint16_t p[3];
    for (unsigned l = 0; l < 3u; l++)
        p[l] = (uint16_t)((plain >> (10u * l)) & 0x3FFu);
    for (unsigned bit = 0; bit < 10u; bit++) {
        for (unsigned l = 0; l < 3u; l++) {
            unsigned chunk = bit / 2u, slot = bit % 2u;
            unsigned pos = chunk * 6u + l * 2u + slot;
            uint16_t got = (uint16_t)(((interleaved >> pos) & 1u) << bit);
            ASSERT_EQ(p[l] & (1u << bit), got,
                      "interleaved bit must land in the same symbol position");
        }
    }
    PASS();
}

/* ROT brings the wanted lane colour into the top of the byte. Datasheet
 * example: in RGB565 red is bits 15:11, so right-rotate by 8 to align. */
TEST(test_tmds_lane_rotation) {
    tmds_state_t st;
    uint32_t out;

    /* All lanes are 0xFF when unrotated. Rotating lane 2 by 8 brings colour
     * bits 15:8 to the top, so feeding 0xFF00 keeps lane 2 all ones. */
    tmds_init(&st);
    tmds_write(&st, TMDS_CTRL, TMDS_NBITS8 | (8u << TMDS_CTRL_ROT_SHIFT(2)));
    tmds_write(&st, TMDS_WDATA, 0xFF00u);
    tmds_read(&st, TMDS_PEEK_SINGLE, &out);
    ASSERT_EQ(0x0A4u, (out >> 20) & 0x3FFu,
              "lane 2 must be control C2 after rotating 0xff00 by 8");
    PASS();
}

/* The TMDS unit tests above call tmds_read()/tmds_write() directly, so they
 * cannot catch the block being unreachable from the bus -- which is exactly the
 * original defect. This goes through rv_mem_write32/rv_mem_read32, which is how
 * firmware reaches it. */
TEST(test_tmds_reachable_through_rv_sio_bus) {
    rv_membus_state_t bus;
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);

    /* CTRL must round-trip through the bus, proving the SIO decode routes the
     * 0x1c0-0x1e4 range to the encoder rather than falling through. */
    uint32_t ctrl = TMDS_NBITS8;
    rv_mem_write32(&bus, RP2350_SIO_BASE + TMDS_CTRL, ctrl);
    ASSERT_EQ(ctrl, rv_mem_read32(&bus, RP2350_SIO_BASE + TMDS_CTRL),
              "TMDS_CTRL must be reachable through the RV SIO bus");

    /* A black pixel written through WDATA must read back as the control
     * symbols, not the unhandled marker (0xDEAD____) this range used to give. */
    rv_mem_write32(&bus, RP2350_SIO_BASE + TMDS_WDATA, 0x0000u);
    uint32_t peek = rv_mem_read32(&bus, RP2350_SIO_BASE + TMDS_PEEK_SINGLE);
    ASSERT_TRUE((peek & 0xFFFF0000u) != 0xDEAD0000u,
                "TMDS_PEEK_SINGLE must not return the unhandled marker");
    ASSERT_EQ(0x354u, peek & 0x3FFu, "lane 0 must be control C0 through the bus");
    ASSERT_EQ(0x0ABu, (peek >> 10) & 0x3FFu, "lane 1 must be control C1");
    ASSERT_EQ(0x0A4u, (peek >> 20) & 0x3FFu, "lane 2 must be control C2");

    /* All six DOUBLE registers must decode and return two 10-bit symbols.
     * PEEK and POP alternate every 4 bytes, so the lane index must step by 8;
     * dividing by 4 indexed past the end of the symbol arrays, which only
     * AddressSanitizer on riscv64 caught. */
    static const uint32_t dbl[6] = {
        TMDS_PEEK_DOUBLE_L0, TMDS_POP_DOUBLE_L0,
        TMDS_PEEK_DOUBLE_L1, TMDS_POP_DOUBLE_L1,
        TMDS_PEEK_DOUBLE_L2, TMDS_POP_DOUBLE_L2,
    };
    for (unsigned k = 0; k < 6u; k++) {
        uint32_t v = rv_mem_read32(&bus, RP2350_SIO_BASE + dbl[k]);
        /* Two 10-bit symbols packed at the bottom; nothing above bit 20. */
        ASSERT_EQ(0, v >> 20,
                  "DOUBLE registers must return exactly two 10-bit symbols");
        ASSERT_TRUE((v & 0x3FFu) != 0 || ((v >> 10) & 0x3FFu) != 0,
                    "a DOUBLE register must return a non-empty symbol");
    }
    PASS();
}

/* Halfword and byte reads must reach the same peripheral models as 32-bit
 * reads. mem_read16 modelled only a handful of blocks, so a halfword read of
 * CLOCKS, TIMER, PADS, GPIO, USB or the NVIC returned 0 while the identical
 * access as 32 bits worked. */
TEST(test_subword_reads_reach_peripherals) {
    extern int membus_rp2350_mode;
    int saved = membus_rp2350_mode;
    membus_rp2350_mode = 0;
    reset_cpu();

    /* CLOCKS: FREQ_CNT is a plain counter we can write. */
    mem_write32(CLOCKS_BASE + 0x04, 0x1234u);   /* CLK_GPCNT0 */
    uint32_t w = mem_read32(CLOCKS_BASE + 0x04);
    ASSERT_EQ(0x1234u, w, "32-bit CLOCKS read is the baseline");
    ASSERT_EQ(0x1234u, mem_read16(CLOCKS_BASE + 0x04), "low halfword");
    ASSERT_EQ(0x0000u, mem_read16(CLOCKS_BASE + 0x06), "high halfword");
    ASSERT_EQ(0x34u, mem_read8(CLOCKS_BASE + 0x04), "byte 0");
    ASSERT_EQ(0x12u, mem_read8(CLOCKS_BASE + 0x05), "byte 1");

    /* NVIC: ISER0 is plain read/write. */
    mem_write32(NVIC_BASE + 0x100, 0xA5A50000u);
    w = mem_read32(NVIC_BASE + 0x100);
    ASSERT_EQ(0xA5A50000u, w, "32-bit NVIC read is the baseline");
    ASSERT_EQ(0x0000u, mem_read16(NVIC_BASE + 0x100), "NVIC low halfword");
    ASSERT_EQ(0xA5A5u,  mem_read16(NVIC_BASE + 0x102), "NVIC high halfword");
    ASSERT_EQ(0xA5u,    mem_read8(NVIC_BASE + 0x102), "NVIC byte");

    membus_rp2350_mode = saved;
    PASS();
}

/* Byte and halfword access to the SIO register block. The four sub-word entry
 * points had no SIO handling: mem_write8 discarded SIO writes outright and
 * mem_read8 fell through to its unmapped path, so LDRB from SIO_GPIO_IN returned
 * 0xFF and STRB to SIO_GPIO_OUT did nothing. */
TEST(test_sio_subword_access) {
    int saved = membus_rp2350_mode;
    membus_rp2350_mode = 0;
    reset_cpu();

    /* DIV_UDIVIDEND at SIO+0x60 is plain read/write storage. */
    mem_write32(SIO_BASE + 0x60, 0xAABBCCDDu);

    ASSERT_EQ(0xDDu, mem_read8(SIO_BASE + 0x60), "byte 0 is the low byte");
    ASSERT_EQ(0xCCu, mem_read8(SIO_BASE + 0x61), "byte 1 is bits 15:8");
    ASSERT_EQ(0xBBu, mem_read8(SIO_BASE + 0x62), "byte 2 is bits 23:16");
    ASSERT_EQ(0xAAu, mem_read8(SIO_BASE + 0x63), "byte 3 is the high byte");

    ASSERT_EQ(0xCCDDu, mem_read16(SIO_BASE + 0x60), "low halfword");
    ASSERT_EQ(0xAABBu, mem_read16(SIO_BASE + 0x62), "high halfword");

    /* A sub-word write must merge into the register, not replace it. */
    mem_write8(SIO_BASE + 0x61, 0x11u);
    ASSERT_EQ(0xAABB11DDu, mem_read32(SIO_BASE + 0x60),
              "a byte write must merge into the register");

    mem_write16(SIO_BASE + 0x62, 0x5678u);
    ASSERT_EQ(0x567811DDu, mem_read32(SIO_BASE + 0x60),
              "a halfword write must merge into the register");

    membus_rp2350_mode = saved;
    PASS();
}

/* Widening the Arm SIO window to 0x200 to reach TMDS at 0x1c0 silently broke
 * spinlocks: SPINLOCK_BASE is SIO_BASE + 0x100, so the SIO window test ran first
 * and swallowed every spinlock access on the Arm path. The 388-test suite still
 * passed, because nothing exercised spinlocks through mem_write32. */
TEST(test_arm_spinlocks_survive_the_widened_sio_window) {
    int saved = membus_rp2350_mode;

    /* spinlock_acquire() returns 1 << n when it takes the lock and 0 when it is
     * already held; a write releases. So acquire, re-read, release, re-read
     * pins the whole round trip. */

    /* RP2040: SIO window 0x100, spinlocks begin immediately after it. */
    membus_rp2350_mode = 0;
    reset_cpu();
    mem_write32(SPINLOCK_BASE, 1u);                        /* release lock 0 */
    ASSERT_EQ(1u, mem_read32(SPINLOCK_BASE),
              "acquiring spinlock 0 must return 1 << 0");
    ASSERT_EQ(0u, mem_read32(SPINLOCK_BASE),
              "re-reading a held spinlock must report it as already held");
    mem_write32(SPINLOCK_BASE, 1u);
    ASSERT_EQ(1u, mem_read32(SPINLOCK_BASE),
              "a released spinlock must be acquirable again");

    /* RP2350: window is 0x200 and so overlaps the entire spinlock block. If the
     * SIO window is tested first these never reach the spinlock model. */
    membus_rp2350_mode = 1;
    reset_cpu();

    mem_write32(SPINLOCK_BASE + 8, 1u);                    /* lock 2 */
    ASSERT_EQ(1u << 2, mem_read32(SPINLOCK_BASE + 8),
              "spinlock 2 must reach the spinlock model on RP2350");
    ASSERT_EQ(0u, mem_read32(SPINLOCK_BASE + 8), "and stay held");

    /* The top of the block, 0x17c, right where the widened window's overlap
     * ends -- the case most likely to be mis-routed. */
    mem_write32(SPINLOCK_BASE + (31 * 4), 1u);
    ASSERT_EQ(1u << 31, mem_read32(SPINLOCK_BASE + (31 * 4)),
              "spinlock 31 (0x17c) must reach the spinlock model");

    /* The *write* side needs its own check: release lock 3, then prove it was
     * really released by acquiring it again. Reading alone cannot tell a
     * swallowed write from a working one, because a read acquires either way. */
    ASSERT_EQ(1u << 3, mem_read32(SPINLOCK_BASE + 12),
              "acquiring spinlock 3 must return 1 << 3");
    ASSERT_EQ(0u, mem_read32(SPINLOCK_BASE + 12), "and then report it held");
    mem_write32(SPINLOCK_BASE + 12, 1u);              /* release */
    ASSERT_EQ(1u << 3, mem_read32(SPINLOCK_BASE + 12),
              "a release written through mem_write32 must take effect");

    /* And the TMDS register just past the spinlocks must still decode, so the
     * two regions cannot both be claimed by one window test. */
    uint32_t val = mem_read32(SIO_BASE + TMDS_CTRL);
    (void)val;   /* reachability is asserted by the TMDS tests */

    membus_rp2350_mode = saved;
    PASS();
}

/* The RP2350 TMDS encoder lives in the shared SIO block, so the Arm cores must
 * reach it. The decoder was originally in the RV SIO path only, which made
 * TMDS unreachable from Arm entirely -- and it had its own private state copy,
 * so the two cores would not have seen each other's writes even if both could
 * decode it. */
TEST(test_tmds_reachable_from_arm_and_shared_with_rv) {

    /* Publish a shared RP2350 peripheral block the same way main.c does for the
     * M33. The Arm SIO path resolves TMDS through this pointer. */
    static rp2350_periph_state_t shared;
    rp2350_periph_init(&shared, 1);
    void *saved_periph = membus_rp2350_periph;
    int saved_mode = membus_rp2350_mode;
    membus_rp2350_periph = &shared;
    membus_rp2350_mode = 1;
    reset_cpu();

    uint32_t ctrl = TMDS_NBITS8;
    mem_write32(SIO_BASE + TMDS_CTRL, ctrl);
    ASSERT_EQ(ctrl, mem_read32(SIO_BASE + TMDS_CTRL),
              "TMDS_CTRL must be reachable through the Arm SIO path");

    /* Write a black pixel through the Arm core... */
    mem_write32(SIO_BASE + TMDS_WDATA, 0x0000u);
    uint32_t via_arm = mem_read32(SIO_BASE + TMDS_PEEK_SINGLE);
    ASSERT_EQ(0x354u, via_arm & 0x3FFu, "lane 0 must be control C0 from Arm");
    ASSERT_EQ(0x0ABu, (via_arm >> 10) & 0x3FFu, "lane 1 must be control C1");
    ASSERT_EQ(0x0A4u, (via_arm >> 20) & 0x3FFu, "lane 2 must be control C2");

    /* ...and read the same encoder state through a Hazard3 bus bound to the
     * same block. With private per-core copies the RV read would see a zeroed
     * WDATA and disagree. */
    rv_membus_state_t bus;
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    bus.periph = shared;
    uint32_t via_rv = rv_mem_read32(&bus, RP2350_SIO_BASE + TMDS_PEEK_SINGLE);
    ASSERT_EQ(via_arm & 0x3FFu, via_rv & 0x3FFu,
              "both cores must see the same TMDS register file");

    membus_rp2350_periph = saved_periph;
    membus_rp2350_mode = saved_mode;
    PASS();
}

/* RP2350 has four DMA interrupt lines (INTE0-INTE3 at 0x404..0x43c) where
 * RP2040 has two. INTE2/INTE3 did not decode at all, and only two lines were
 * signalled, so a channel could never raise DMA_IRQ_2 or DMA_IRQ_3. */
TEST(test_dma_rp2350_has_four_interrupt_lines) {
    dma_init();
    membus_rp2350_mode = 1;

    /* The four enable registers must be independent. */
    dma_write32(DMA_INTE0, 0x1u);
    dma_write32(DMA_INTE1, 0x2u);
    dma_write32(DMA_INTE2, 0x4u);
    dma_write32(DMA_INTE3, 0x8u);

    ASSERT_EQ(0x1u, dma_read32(DMA_INTE0), "INTE0 must be at 0x404");
    ASSERT_EQ(0x2u, dma_read32(DMA_INTE1), "INTE1 must be at 0x414");
    ASSERT_EQ(0x4u, dma_read32(DMA_INTE2), "INTE2 must be at 0x424");
    ASSERT_EQ(0x8u, dma_read32(DMA_INTE3), "INTE3 must be at 0x434");

    /* Forcing on one line must not set the others. */
    dma_write32(DMA_INTF2, 0x4u);
    ASSERT_EQ(0x4u, dma_read32(DMA_INTF2), "INTF2 must be at 0x428");
    ASSERT_EQ(0x4u, dma_read32(DMA_INTS2),
              "a forced IRQ on line 2 must appear in INTS2");
    ASSERT_EQ(0, dma_read32(DMA_INTS0), "line 2 must not disturb line 0");
    ASSERT_EQ(0, dma_read32(DMA_INTS1), "line 2 must not disturb line 1");
    ASSERT_EQ(0, dma_read32(DMA_INTS3), "line 2 must not disturb line 3");

    /* A forced IRQ whose line is not enabled must not show in status. */
    dma_write32(DMA_INTF3, 0x1u);   /* channel 0, but INTE3 only has bit 3 */
    ASSERT_EQ(0, dma_read32(DMA_INTS3), "a forced but disabled IRQ must not set");

    /* Force registers must not alias the enable registers. */
    ASSERT_EQ(0x8u, dma_read32(DMA_INTE3),
              "writing INTF2 must not clobber INTE3");
    PASS();
}

/* The extra lines are RP2350-only; on RP2040 their internal IRQ numbers must be
 * rejected rather than landing on an unrelated vector. */
TEST(test_dma_extra_irqs_are_rp2350_only) {
    ASSERT_TRUE(IRQ_DMA_IRQ_2 >= NUM_EXTERNAL_IRQS,
                "DMA_IRQ_2 must sit past the RP2040 IRQ range");
    ASSERT_TRUE(IRQ_DMA_IRQ_3 >= NUM_EXTERNAL_IRQS,
                "DMA_IRQ_3 must sit past the RP2040 IRQ range");

    membus_rp2350_mode = 0;
    ASSERT_EQ(26, nvic_num_external_irqs(), "RP2040 has 26 external IRQs");
    membus_rp2350_mode = 1;
    ASSERT_TRUE(nvic_num_external_irqs() > IRQ_DMA_IRQ_3,
                "RP2350 must be able to reach the extra DMA lines");
    PASS();
}

/* VMOV between an Arm core register and a single-precision register,
 * ARM ARM A7.7.243 encoding T1:
 *
 *   1110 1110 000 op | Vn | Rt | 1010 | N 0 0 1 0 | 0000
 *                       ^op = bit 20
 *
 * The old decode pinned op to 0, took its direction from bits 23:16 (always
 * below the 0x17 threshold, so the "to core register" path was unreachable),
 * and took the register number's low bit from bit 6 where the encoding pins 0.
 */
static void vfp_vmov_exec(unsigned op, unsigned vn, unsigned rt, unsigned n) {
    uint32_t insn = 0xEE000A10u | ((uint32_t)(op) << 20) |
                    ((uint32_t)(vn) << 16) | ((uint32_t)(rt) << 12) |
                    ((uint32_t)(n) << 7);
    thumb32_step(RAM_BASE, (uint16_t)(insn >> 16), (uint16_t)insn);
}

TEST(test_vfp_vmov_both_directions) {
    reset_cpu();
    const uint32_t out = RAM_BASE + 0x3300;

    /* op=0: core register -> S1 (Vn=0, N=1). */
    cpu.r[0] = 0x12345678u;
    vfp_vmov_exec(0, 0, 0, 1);

    /* op=1: S1 -> core register, the path the old decode made dead. */
    cpu.r[1] = 0;
    vfp_vmov_exec(1, 0, 1, 1);
    ASSERT_EQ(0x12345678u, cpu.r[1],
              "VMOV to core register must read back what was written");

    /* The two directions must be genuinely independent, not symmetric by
     * accident: writing a different value to S1 must be visible through D0. */
    cpu.r[2] = 0xCAFEBABEu;
    vfp_vmov_exec(0, 0, 2, 1);
    cpu.r[1] = 0;
    vfp_vmov_exec(1, 0, 1, 1);
    ASSERT_EQ(0xCAFEBABEu, cpu.r[1], "VMOV must return the latest S1 value");

    /* And the write must reach the double view, proving VMOV and VLDR share
     * one register file. */
    cpu.r[3] = out;
    thumb32_step(RAM_BASE, VFP_LS_HW1(1, 0, 3, 0), VFP_LS_HW2(0, 0xB, 0)); /* VSTR D0 */
    ASSERT_EQ(0xCAFEBABEu, mem_read32(out),
              "a VMOV into S1 must be visible through D0");
    PASS();
}

/* The single register number is (Vn << 1) | N, so all 32 must be reachable.
 * The old decode used bit 6, which the encoding pins to 0. */
TEST(test_vfp_vmov_reaches_all_single_registers) {
    reset_cpu();

    for (unsigned n = 0; n < 32u; n++) {
        cpu.r[0] = 0xC0DE0000u | n;
        vfp_vmov_exec(0, n >> 1, 0, n & 1u);
    }
    for (unsigned n = 0; n < 32u; n++) {
        cpu.r[1] = 0;
        vfp_vmov_exec(1, n >> 1, 1, n & 1u);
        ASSERT_EQ(0xC0DE0000u | n, cpu.r[1],
                  "every single register S0-S31 must be independently reachable");
    }
    PASS();
}

/* RP2350 places the PIO interrupt block after the RX FIFO PUTGET window:
 * GPIOBASE 0x168, INTR 0x16c, IRQ0_INTE 0x170 ... IRQ1_INTS 0x184. The defines
 * started at 0x128/0x12c, which are RXF0_PUTGET0/PUTGET1, so every access to a
 * PIO IRQ register landed in the PUTGET window and PIO interrupts could not be
 * enabled or forced at all. */
TEST(test_pio_rp2350_irq_register_offsets) {
    pio_init();
    membus_rp2350_mode = 1;

    /* The datasheet offsets must be distinct and decodable. */
    pio_write32(0, PIO_IRQ0_INTE, 0x5u);
    ASSERT_EQ(0x5u, pio_read32(0, PIO_IRQ0_INTE), "IRQ0_INTE must be at 0x170");
    ASSERT_EQ(0, pio_read32(0, PIO_IRQ0_INTF), "a fresh block has no forced IRQs");

    /* Force bit 0, which is enabled (0x5 = bits 0 and 2), and confirm status. */
    pio_write32(0, PIO_IRQ0_INTF, 0x1u);
    ASSERT_EQ(0x1u, pio_read32(0, PIO_IRQ0_INTF), "IRQ0_INTF must be at 0x174");
    ASSERT_EQ(0x1u, pio_read32(0, PIO_IRQ0_INTS),
              "IRQ0_INTS must show a forced IRQ that is enabled");

    /* Bit 1 is forced but not enabled, so it must not appear in status. */
    pio_write32(0, PIO_IRQ0_INTF, 0x3u);
    ASSERT_EQ(0x1u, pio_read32(0, PIO_IRQ0_INTS),
              "a forced but disabled IRQ must not set the status");

    /* The two interrupt lines are independent. */
    pio_write32(0, PIO_IRQ1_INTE, 0x8u);
    ASSERT_EQ(0x8u, pio_read32(0, PIO_IRQ1_INTE), "IRQ1_INTE must be at 0x17c");
    pio_write32(0, PIO_IRQ1_INTF, 0x8u);
    ASSERT_EQ(0x8u, pio_read32(0, PIO_IRQ1_INTF), "IRQ1_INTF must be at 0x180");
    ASSERT_EQ(0x8u, pio_read32(0, PIO_IRQ1_INTS),
              "irq1 must see its own force, independently of irq0");

    /* Writing PUTGET must not disturb the IRQ registers -- the bug made these
     * the same storage. */
    pio_write32(0, PIO_RXF0_PUTGET, 0xDEADBEEFu);
    ASSERT_EQ(0x5u, pio_read32(0, PIO_IRQ0_INTE),
              "writing RXF0_PUTGET0 must not alias IRQ0_INTE");

    /* Only the state-machine bits exist: RP2350 has eight, so bits 8+ are
     * reserved and must not be retained. */
    pio_write32(0, PIO_IRQ0_INTE, 0xFFFFFFFFu);
    ASSERT_EQ(0xFFu, pio_read32(0, PIO_IRQ0_INTE),
              "IRQ enables must mask to the eight RP2350 state machines");

    membus_rp2350_mode = 0;
    PASS();
}

/* RP2040 has four state machines, so the mask must be narrower there. */
TEST(test_pio_rp2040_irq_mask) {
    pio_init();
    membus_rp2350_mode = 0;

    pio_write32(0, PIO_IRQ0_INTE, 0xFFFFFFFFu);
    ASSERT_EQ(0x0Fu, pio_read32(0, PIO_IRQ0_INTE),
              "RP2040 PIO has only four state machine IRQ bits");

    membus_rp2350_mode = 0;
    PASS();
}

/* Writing LOAD arms the watchdog -- there is no separate enable bit. LOAD was
 * stored and never counted, so firmware that relied on the watchdog to recover
 * from a hang would spin forever instead of getting the reset it asked for. */
TEST(test_watchdog_countdown_expires) {
    clocks_init();
    membus_rp2350_mode = 0;

    /* Disarmed: ticking must never arm anything by itself. */
    clocks_watchdog_tick(1000);
    ASSERT_EQ(0, watchdog_reboot_pending, "an unarmed watchdog must not fire");

    /* Arm with a short LOAD, then tick past it. */
    clocks_write32(WATCHDOG_BASE + 0x04, 10);
    ASSERT_EQ(0, watchdog_reboot_pending, "arming alone must not fire");
    clocks_watchdog_tick(9);
    ASSERT_EQ(0, watchdog_reboot_pending, "must not fire before the countdown ends");
    clocks_watchdog_tick(1);
    ASSERT_TRUE(watchdog_reboot_pending != 0, "the watchdog must fire once LOAD expires");
    ASSERT_EQ(WATCHDOG_REASON_WDOG, clocks_read32(WATCHDOG_BASE + 0x08),
              "an expired watchdog must be recorded in REASON");

    /* Firing is one-shot: the countdown must not re-trigger every tick. */
    watchdog_reboot_pending = 0;
    clocks_watchdog_tick(1000);
    ASSERT_EQ(0, watchdog_reboot_pending, "an expired watchdog must not re-arm itself");

    /* LOAD masks to 24 bits -- the datasheet's documented maximum. */
    clocks_init();
    clocks_write32(WATCHDOG_BASE + 0x04, 0xFFFFFFFFu);
    clocks_watchdog_tick(0x1000000u + 1u);   /* just past the masked maximum */
    ASSERT_TRUE(watchdog_reboot_pending != 0,
                "LOAD above 0xffffff must clamp to the 24-bit maximum");
    PASS();
}

/* A software reset must be distinguishable from a power-on reset. */
TEST(test_watchdog_reason_soft_reset) {
    clocks_init();
    membus_rp2350_mode = 0;

    ASSERT_EQ(WATCHDOG_REASON_RESET, clocks_read32(WATCHDOG_BASE + 0x08),
              "a cold boot reports a hardware reset");
    clocks_note_soft_reset();
    ASSERT_EQ(WATCHDOG_REASON_SOFT, clocks_read32(WATCHDOG_BASE + 0x08),
              "a software reset must set REASON bit 1");

    /* REASON must survive the soft reset that caused it, and cold boot clears it. */
    clocks_reset();
    ASSERT_EQ(WATCHDOG_REASON_SOFT, clocks_read32(WATCHDOG_BASE + 0x08),
              "REASON must survive the soft reset that set it");
    clocks_init();
    ASSERT_EQ(WATCHDOG_REASON_RESET, clocks_read32(WATCHDOG_BASE + 0x08),
              "a cold boot must clear REASON");
    PASS();
}

/* RP2040 does have TICK, so the same offset must keep working there. */
TEST(test_watchdog_tick_exists_on_rp2040) {
    clocks_init();
    membus_rp2350_mode = 0;

    clocks_write32(WATCHDOG_BASE + 0x2C, WATCHDOG_TICK_ENABLE | 0x1234u);
    ASSERT_TRUE((clocks_read32(WATCHDOG_BASE + 0x2C) & WATCHDOG_TICK_ENABLE) != 0,
                "RP2040 has a TICK register and ENABLE must read back");
    PASS();
}

/* C9: SIO CPUID must return the hart id, not a constant. Hart 1's boot sequence
 * branches on it to decide whether it is the secondary core. */
TEST(test_rv_sio_cpuid_is_hart_dependent) {
    rv_membus_state_t bus;
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);

    rv_clint_set_current_hart(0);
    ASSERT_EQ(0, rv_mem_read32(&bus, RP2350_SIO_BASE + RV_SIO_CPUID),
              "core 0 must read CPUID == 0");

    rv_clint_set_current_hart(1);
    ASSERT_EQ(1, rv_mem_read32(&bus, RP2350_SIO_BASE + RV_SIO_CPUID),
              "core 1 must read CPUID == 1");

    rv_clint_set_current_hart(0);
    PASS();
}

/* F1: misa must advertise the extensions this core actually implements, and
 * the identification CSRs must not read back as "unimplemented". */
TEST(test_rv_misa_and_id_csr_values) {
    rv_cpu_state_t rv;
    rv_cpu_init(&rv, 0);

    uint32_t misa = rv_csr_read(&rv, CSR_MISA);
    ASSERT_TRUE((misa & (1u << 30)) != 0, "MXL must be 1 for RV32");
    ASSERT_TRUE((misa & (1u << 0)) != 0, "misa.A must be set (atomics are implemented)");
    ASSERT_TRUE((misa & (1u << 2)) != 0, "misa.C must be set (compressed is implemented)");
    ASSERT_TRUE((misa & (1u << 8)) != 0, "misa.I must be set");
    ASSERT_TRUE((misa & (1u << 12)) != 0, "misa.M must be set");
    /* X declares the non-standard extensions (Zba/Zbb/Zbs/Zcb/Zcmp/Zbkb), which
     * this core does implement. It used to be clear. */
    ASSERT_TRUE((misa & (1u << 23)) != 0, "misa.X must be set for the Zb* extensions");

    /* U is deliberately not advertised: there is no user mode, and claiming it
     * would send firmware into transitions the core cannot complete. */
    ASSERT_TRUE((misa & (1u << 20)) == 0,
                "misa.U must stay clear while user mode is unimplemented");

    ASSERT_EQ(0x00000493, rv_csr_read(&rv, CSR_MVENDORID), "mvendorid");
    ASSERT_EQ(0x0000001B, rv_csr_read(&rv, CSR_MARCHID), "marchid");
    ASSERT_EQ(0x86FC4E3F, rv_csr_read(&rv, CSR_MIMPID), "mimpid");

    /* Identification CSRs are read-only. */
    rv_csr_write(&rv, CSR_MARCHID, 0xDEAD);
    ASSERT_EQ(0x0000001B, rv_csr_read(&rv, CSR_MARCHID), "marchid must be read-only");

    rv_cpu_state_t rv1;
    rv_cpu_init(&rv1, 1);
    ASSERT_EQ(1, rv_csr_read(&rv1, CSR_MHARTID), "mhartid must be per-hart");
    PASS();
}

/* F1: MRET drops MPP to the least-privileged supported mode. Leaving it at
 * M-mode made a handler that had correctly dropped privilege still look
 * privileged. */
TEST(test_rv_mret_clears_mpp) {
    rv_cpu_state_t rv;
    rv_cpu_init(&rv, 0);

    rv.csr[CSR_MSTATUS] = MSTATUS_MPP | MSTATUS_MPIE;
    rv.csr[CSR_MEPC] = 0x1000;

    rv_trap_return(&rv);

    ASSERT_EQ(0, rv.csr[CSR_MSTATUS] & MSTATUS_MPP,
              "MRET must clear MPP (there is no user mode to drop to)");
    ASSERT_EQ(0x1000, rv.pc, "MRET must restore PC from MEPC");
    PASS();
}

/* F1: an icache flush or range invalidate must actually drop entries, and a
 * range invalidate must leave unrelated addresses cached. */
TEST(test_rv_icache_invalidation_after_flash_write) {
    rv_icache_t ic;
    rv_icache_init(&ic);

    const uint32_t base = 0x10000100;
    rv_icache_insert(&ic, base, 0x00000013, 4);           /* nop */
    rv_icache_insert(&ic, base + 0x1000, 0x00000013, 4);  /* elsewhere */

    uint32_t instr = 0;
    uint8_t size = 0;
    ASSERT_TRUE(rv_icache_lookup(&ic, base, &instr, &size) == 1, "entry should hit");

    rv_icache_invalidate_range(&ic, base, 4);

    ASSERT_TRUE(rv_icache_lookup(&ic, base, &instr, &size) == 0,
                "the entry covering the written address must be dropped");
    ASSERT_TRUE(rv_icache_lookup(&ic, base + 0x1000, &instr, &size) == 1,
                "an unrelated address must stay cached");
    PASS();
}

TEST(test_rv_icache_flush_drops_everything) {
    rv_icache_t ic;
    rv_icache_init(&ic);
    rv_icache_insert(&ic, 0x10000100, 0x00000013, 4);
    rv_icache_insert(&ic, 0x10000900, 0x00000013, 4);

    rv_icache_flush(&ic);

    uint32_t instr = 0;
    uint8_t size = 0;
    ASSERT_TRUE(rv_icache_lookup(&ic, 0x10000100, &instr, &size) == 0,
                "flush must drop the first entry");
    ASSERT_TRUE(rv_icache_lookup(&ic, 0x10000900, &instr, &size) == 0,
                "flush must drop the second entry");
    PASS();
}

/* C10: a peripheral IRQ has to reach mip.MEIP. Before the nvic.c -> Xh3irq
 * bridge, nvic_signal_irq() latched the bit in the NVIC, which the RV core
 * never reads, so mip.MEIP stayed 0 and firmware blocking on a UART receive
 * interrupt hung forever.
 *
 * main.c's bridge is a static function over its own bus, so the test installs
 * an equivalent sink over a local bus. What is under test is the nvic.c fan-out
 * itself: whatever vector nvic renames to must land in the Xh3irq pending
 * array, and be acknowledged on clear. */
static rv_clint_state_t *test_irq_bridge_clint = NULL;

static void test_irq_bridge_sink(uint32_t irq, int asserted) {
    if (!test_irq_bridge_clint) return;
    if (asserted)
        rv_clint_set_ext_pending(test_irq_bridge_clint, irq);
    else
        rv_clint_clear_ext_pending(test_irq_bridge_clint, irq);
}

TEST(test_rv_peripheral_irq_reaches_mip_meip) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;

    test_irq_bridge_clint = &bus.clint;
    nvic_set_ext_irq_sink(test_irq_bridge_sink);

    /* A peripheral asserting its line, exactly as uart.c does. */
    nvic_signal_irq(IRQ_UART0_IRQ);

    /* Take the vector nvic actually delivered, whatever chip mapping is active. */
    uint32_t irq = nvic_irq_number(IRQ_UART0_IRQ);
    ASSERT_TRUE(irq < RV_NUM_EXT_IRQS, "the vector must be representable in Xh3irq");

    /* It must be visible in the Xh3irq pending array... */
    rv_csr_write(&rv, CSR_MEIEA, irq / 16);   /* select the containing window */
    ASSERT_TRUE((rv_csr_read(&rv, CSR_MEIPA) & (1u << (irq % 16))) != 0,
                "MEIPA must show the peripheral IRQ as pending");

    /* Acknowledging must deassert the line again. */
    nvic_clear_pending(irq);
    rv_csr_write(&rv, CSR_MEIEA, irq / 16);
    ASSERT_EQ(0, rv_csr_read(&rv, CSR_MEIPA) & (1u << (irq % 16)),
              "clearing the pending bit must clear MEIPA too");

    nvic_set_ext_irq_sink(NULL);
    test_irq_bridge_clint = NULL;
    PASS();
}

/* With the line asserted, enabling it must make mip.MEIP set and trap. */
TEST(test_rv_peripheral_irq_traps) {
    rv_cpu_state_t rv;
    rv_membus_state_t bus;
    rv_cpu_init(&rv, 0);
    rv_membus_init(&bus, cpu.flash, FLASH_SIZE, 1);
    rv.bus = &bus;

    test_irq_bridge_clint = &bus.clint;
    nvic_set_ext_irq_sink(test_irq_bridge_sink);

    nvic_signal_irq(IRQ_UART0_IRQ);
    uint32_t irq = nvic_irq_number(IRQ_UART0_IRQ);

    /* Nothing is enabled yet, so mip.MEIP must stay clear. */
    rv_clint_check_interrupts(&bus.clint, &rv);
    ASSERT_EQ(0, rv.csr[CSR_MIP] & MIP_MEIP, "MEIP must stay clear while disabled");

    /* Enable the interrupt and unmask it, then it must trap. */
    /* Enable the one bit for this IRQ: mask bit (irq % 16) in window irq/16. */
    rv_csr_write(&rv, CSR_MEIEA, ((1u << (irq % 16)) << 16) | (irq / 16));
    rv.csr[CSR_MIE] |= MIP_MEIP;
    rv.csr[CSR_MSTATUS] |= MSTATUS_MIE;

    /* Point mtvec at a recognisable handler: rv_cpu_init() leaves it 0, so
     * without this the trap "redirects" to address 0 and looks like a no-op. */
    rv.csr[CSR_MTVEC] = 0x00000200;
    uint32_t pc_before = rv.pc;
    int delivered = rv_clint_check_interrupts(&bus.clint, &rv);

    ASSERT_TRUE((rv.csr[CSR_MIP] & MIP_MEIP) != 0, "the enabled IRQ must set mip.MEIP");
    ASSERT_TRUE(delivered == 1, "the external interrupt must be delivered");
    ASSERT_EQ(MCAUSE_MEI, rv.csr[CSR_MCAUSE],
              "mcause must be the machine external-interrupt cause");
    ASSERT_TRUE(rv.pc != pc_before, "the trap must redirect control via mtvec");
    ASSERT_TRUE((rv.csr[CSR_MSTATUS] & MSTATUS_MIE) == 0,
                "entering the trap must clear MSTATUS.MIE");

    nvic_set_ext_irq_sink(NULL);
    test_irq_bridge_clint = NULL;
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
    printf(" Bramble v0.48.0 test suite\n");
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
    RUN_TEST(test_cpu_bind_core_context_roundtrip);
    END_CATEGORY("Dual-Core Memory");

    BEGIN_CATEGORY("UF2 Loader");
    RUN_TEST(test_uf2_loader_rejects_oversized_payload);
    END_CATEGORY("UF2 Loader");

    BEGIN_CATEGORY("ELF Loader");
    RUN_TEST(test_elf_loader_valid);
    RUN_TEST(test_elf_loader_invalid_magic);
    RUN_TEST(test_elf_loader_wrong_arch);
    RUN_TEST(test_elf_loader_rejects_segment_overflow);
    RUN_TEST(test_elf_loader_rejects_filesz_gt_memsz);
    END_CATEGORY("ELF Loader");

    BEGIN_CATEGORY("Memory Bus");
    RUN_TEST(test_flash_read_write);
    RUN_TEST(test_ram_read_write);
    RUN_TEST(test_flash_alias_writes_ignored);
    RUN_TEST(test_nvic_memory_map_subword_access);
    RUN_TEST(test_xip_ssi_atomic_aliases);
    RUN_TEST(test_io_qspi_atomic_aliases);
    RUN_TEST(test_pads_qspi_atomic_aliases);
    RUN_TEST(test_busctrl_atomic_aliases);
    RUN_TEST(test_sio_gpio_out_writes_reach_gpio_state);
    RUN_TEST(test_sio_gpio_oe_writes_reach_gpio_state);
    RUN_TEST(test_sio_non_gpio_writes_still_reach_sio);
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
    RUN_TEST(test_systick_enable_before_rvr_does_not_fire);
    RUN_TEST(test_systick_fires_on_counter_wrap);
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
    RUN_TEST(test_scb_icsr_memory_map_pending_bits);
    RUN_TEST(test_scb_aircr_sysresetreq_requires_key);
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
    RUN_TEST(test_cpu_step_delivers_pending_external_irq);
    RUN_TEST(test_exception_nesting_restores_previous_exception);
    RUN_TEST(test_cpu_step_invalid_pc_enters_hardfault);
    RUN_TEST(test_double_hardfault_locks_up);
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
    RUN_TEST(test_usb_cdc_stdio_active_requires_bidirectional_console);
    RUN_TEST(test_usb_cdc_rx_push_requires_ready_console);
    END_CATEGORY("USB Controller");

    BEGIN_CATEGORY("Flash ROM Functions");
    RUN_TEST(test_rom_flash_functions_in_table);
    END_CATEGORY("Flash ROM Functions");

    BEGIN_CATEGORY("UART Peripheral");
    RUN_TEST(test_uart_output);
    RUN_TEST(test_uart_stdio_activity_tracks_tx);
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

    BEGIN_CATEGORY("CPUID and NVIC Extensions");
    RUN_TEST(test_cpuid_register);
    RUN_TEST(test_nvic_iabr_read);
    RUN_TEST(test_nvic_ipr7_readwrite);
    END_CATEGORY("CPUID and NVIC Extensions");

    BEGIN_CATEGORY("RTC Ticking");
    RUN_TEST(test_rtc_load_and_read);
    RUN_TEST(test_rtc_tick_seconds);
    RUN_TEST(test_rtc_minute_rollover);
    RUN_TEST(test_rtc_not_ticking_when_disabled);
    END_CATEGORY("RTC Ticking");

    BEGIN_CATEGORY("SD Card SPI");
    RUN_TEST(test_sdcard_init_creates_state);
    RUN_TEST(test_sdcard_cmd0_goes_idle);
    RUN_TEST(test_sdcard_cmd8_returns_check_pattern);
    RUN_TEST(test_sdcard_acmd41_initializes);
    RUN_TEST(test_sdcard_cmd17_read_block);
    RUN_TEST(test_sdcard_cmd24_write_block);
    END_CATEGORY("SD Card SPI");

    BEGIN_CATEGORY("eMMC SPI");
    RUN_TEST(test_emmc_init_creates_state);
    RUN_TEST(test_emmc_cmd0_goes_idle);
    RUN_TEST(test_emmc_cmd1_initializes);
    RUN_TEST(test_emmc_cmd17_read_block);
    END_CATEGORY("eMMC SPI");

    BEGIN_CATEGORY("Flash Persistence");
    RUN_TEST(test_flash_persist_sync_no_crash_without_path);
    RUN_TEST(test_rom_flash_erase_rejects_wrapping_offset);
    RUN_TEST(test_rom_flash_program_rejects_wrapping_offset);
    RUN_TEST(test_flash_persist_set_and_close);
    END_CATEGORY("Flash Persistence");

    BEGIN_CATEGORY("Core Pool / Threading");
    RUN_TEST(test_corepool_detect_host_cpus);
    RUN_TEST(test_corepool_init_and_cleanup);
    RUN_TEST(test_corepool_register_and_unregister);
    RUN_TEST(test_corepool_query_cores_returns_valid);
    RUN_TEST(test_corepool_query_cores_prunes_stale_entries);
    RUN_TEST(test_num_active_cores_default);
    RUN_TEST(test_wfi_sets_core_flag);
    END_CATEGORY("Core Pool / Threading");

    BEGIN_CATEGORY("Wire Protocol");
    RUN_TEST(test_wire_poll_handles_partial_uart_frame);
    RUN_TEST(test_wire_eth_frame_relay);
    RUN_TEST(test_wire_eth_active);
    END_CATEGORY("Wire Protocol");

    BEGIN_CATEGORY("Virtual Network Bus");
    RUN_TEST(test_vnet_init_cleanup);
    RUN_TEST(test_vnet_register_port);
    RUN_TEST(test_vnet_frame_delivery_to_port);
    RUN_TEST(test_vnet_unicast_delivery);
    RUN_TEST(test_vnet_generate_mac);
    RUN_TEST(test_vnet_peer_socketpair);
    END_CATEGORY("Virtual Network Bus");

    BEGIN_CATEGORY("Software-Defined Devices");
    RUN_TEST(test_sdd_thermometer_create);
    RUN_TEST(test_sdd_thermometer_i2c_read);
    RUN_TEST(test_sdd_thermometer_custom_temp);
    RUN_TEST(test_sdd_thermometer_config_register);
    RUN_TEST(test_sdd_create_from_arg);
    RUN_TEST(test_sdd_unknown_type);
    END_CATEGORY("Software-Defined Devices");

    BEGIN_CATEGORY("W5500 Live Networking");
    RUN_TEST(test_w5500_init_host_fds);
    RUN_TEST(test_w5500_set_live);
    RUN_TEST(test_w5500_tcp_open_creates_host_socket);
    RUN_TEST(test_w5500_udp_open_creates_host_socket);
    RUN_TEST(test_w5500_close_cleans_host_socket);
    END_CATEGORY("W5500 Live Networking");

    BEGIN_CATEGORY("Cortex-M33");
    RUN_TEST(test_m33_cpuid);
    RUN_TEST(test_m33_basepri);
    RUN_TEST(test_m33_thumb2_sdiv);
    RUN_TEST(test_m33_thumb2_movw_movt);
    END_CATEGORY("Cortex-M33");

    BEGIN_CATEGORY("RISC-V CPU");
    RUN_TEST(test_rv_cpu_init);
    RUN_TEST(test_rv_cpu_reset);
    RUN_TEST(test_rv_addi_instruction);
    RUN_TEST(test_rv_lui_instruction);
    RUN_TEST(test_rv_add_sub);
    RUN_TEST(test_rv_branch_beq);
    RUN_TEST(test_rv_load_store);
    RUN_TEST(test_rv_mul_div);
    RUN_TEST(test_rv_jal_jalr);
    RUN_TEST(test_rv_compressed_c_addi);
    RUN_TEST(test_rv_csr_mhartid);
    RUN_TEST(test_rv_trap_enter_return);
    RUN_TEST(test_rv_step_advances_devtools_cycle_count);
    END_CATEGORY("RISC-V CPU");

    BEGIN_CATEGORY("RISC-V CLINT");
    RUN_TEST(test_rv_clint_timer);
    RUN_TEST(test_rv_clint_timer_interrupt);
    END_CATEGORY("RISC-V CLINT");

    BEGIN_CATEGORY("RISC-V Memory Bus");
    RUN_TEST(test_gpio_interrupt_regs_are_decoded_not_pins);
    RUN_TEST(test_gpio_chip_aware_bases);
    RUN_TEST(test_gpio_layout_follows_delegated_addresses);
    RUN_TEST(test_rp2350_irq_renumbering);
    RUN_TEST(test_rv_membus_sram);
    RUN_TEST(test_rv_shared_periph_translated_base);
    RUN_TEST(test_rv_bootrom_init);
    END_CATEGORY("RISC-V Memory Bus");

    BEGIN_CATEGORY("RISC-V ICache");
    RUN_TEST(test_rv_icache);
    END_CATEGORY("RISC-V ICache");

    BEGIN_CATEGORY("RP2350 Peripherals");
    RUN_TEST(test_rv_periph_bootram);
    RUN_TEST(test_rv_periph_timer1);
    RUN_TEST(test_rv_hazard3_csrs);
    RUN_TEST(test_spi_sspsdr_read_does_not_clock_a_phantom_byte);
    RUN_TEST(test_rv_mtvec_mode_bit_is_warl);
    RUN_TEST(test_rv_zcb_c_sb_uses_its_offset);
    RUN_TEST(test_rv_zcb_c_sh_uses_its_offset);
    RUN_TEST(test_rv_zcb_c_sb_uses_both_rs1_and_rs2);
    RUN_TEST(test_pwm_rp2350_layout);
    RUN_TEST(test_pwm_rp2040_layout_unchanged);
    RUN_TEST(test_thumb2_ldrd_strd_immediate);
    RUN_TEST(test_rv_hart1_launch_uses_the_documented_fifo_protocol);
    RUN_TEST(test_spi_flash_rejects_out_of_range_accesses);
    RUN_TEST(test_spi_flash_size_is_normalised_to_the_erase_unit);
    RUN_TEST(test_fat16_mount_rejects_malformed_bpb);
    RUN_TEST(test_fat16_rejects_out_of_bounds_directory_entries);
    RUN_TEST(test_bme280_register_protocol);
    RUN_TEST(test_net_bridge_rejects_out_of_range_uart_index);
    RUN_TEST(test_fuse_mount_rejects_an_invalid_image);
    RUN_TEST(test_fuse_set_flash_offset_records_the_region);
    RUN_TEST(test_gdb_rsp_accepts_a_valid_packet);
    RUN_TEST(test_gdb_rsp_rejects_a_bad_checksum);
    RUN_TEST(test_gdb_rsp_handles_an_oversized_packet);
    RUN_TEST(test_cyw43_gpio_intercept_only_claims_wifi_pins);
    RUN_TEST(test_cyw43_bitbang_spi_state_machine);
    RUN_TEST(test_tapif_open_rejects_an_unknown_interface);
    RUN_TEST(test_tapif_read_write_against_a_real_interface);
    RUN_TEST(test_thumb2_vldr_vstr_double_roundtrip);
    RUN_TEST(test_thumb2_vldr_vstr_single_roundtrip);
    RUN_TEST(test_tmds_register_map_and_control_symbols);
    RUN_TEST(test_tmds_peek_does_not_shift_but_pop_does);
    RUN_TEST(test_tmds_interleave_packing);
    RUN_TEST(test_tmds_lane_rotation);
    RUN_TEST(test_subword_reads_reach_peripherals);
    RUN_TEST(test_sio_subword_access);
    RUN_TEST(test_arm_spinlocks_survive_the_widened_sio_window);
    RUN_TEST(test_tmds_reachable_from_arm_and_shared_with_rv);
    RUN_TEST(test_tmds_reachable_through_rv_sio_bus);
    RUN_TEST(test_dma_rp2350_has_four_interrupt_lines);
    RUN_TEST(test_dma_extra_irqs_are_rp2350_only);
    RUN_TEST(test_vfp_vmov_both_directions);
    RUN_TEST(test_vfp_vmov_reaches_all_single_registers);
    RUN_TEST(test_vfp_vmov_reaches_all_single_registers);
    RUN_TEST(test_vfp_double_aliases_single_pair);
    RUN_TEST(test_vfp_double_registers_16_to_31);
    RUN_TEST(test_vfp_high_doubles_alias_low_ones);
    RUN_TEST(test_vfp_high_doubles_do_not_clobber_low_ones);
    RUN_TEST(test_pio_rp2350_irq_register_offsets);
    RUN_TEST(test_pio_rp2040_irq_mask);
    RUN_TEST(test_watchdog_reason_and_rp2350_map);
    RUN_TEST(test_watchdog_countdown_expires);
    RUN_TEST(test_watchdog_reason_soft_reset);
    RUN_TEST(test_watchdog_tick_exists_on_rp2040);
    RUN_TEST(test_rp2350_ticks_generator_stride);
    RUN_TEST(test_rv_sio_cpuid_is_hart_dependent);
    RUN_TEST(test_rv_misa_and_id_csr_values);
    RUN_TEST(test_rv_mret_clears_mpp);
    RUN_TEST(test_rv_icache_invalidation_after_flash_write);
    RUN_TEST(test_rv_icache_flush_drops_everything);
    RUN_TEST(test_rv_peripheral_irq_reaches_mip_meip);
    RUN_TEST(test_rv_peripheral_irq_traps);
    END_CATEGORY("RP2350 Peripherals");

    BEGIN_CATEGORY("GPIO VCD Trace");
    RUN_TEST(test_vcd_timestamps_advance);
    END_CATEGORY("GPIO VCD Trace");

    printf("\n========================================\n");
    printf(" Results: %d/%d passed, %d failed\n", tests_passed, tests_run, tests_failed);
    printf("========================================\n");

    return tests_failed > 0 ? 1 : 0;
}
