#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "emulator.h"
#include "gpio.h"
#include "timer.h"
#include "nvic.h"
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
#include "gdb.h"
#include "devtools.h"

/* ========================================================================
 * XIP Cache Control State (0x14000000)
 *
 * Stub implementation: cache is always "ready" and "empty".
 * Real RP2040 has 16KB XIP cache that caches flash reads.
 * ======================================================================== */

#define XIP_CTRL_OFFSET    0x00  /* Cache enable (bit 3=POWER_DOWN, bit 1=ERR_BADWRITE, bit 0=EN) */
#define XIP_FLUSH_OFFSET   0x04  /* Write 1 to flush (strobe, self-clearing) */
#define XIP_STAT_OFFSET    0x08  /* bit 2=FIFO_EMPTY, bit 1=FLUSH_READY */
#define XIP_CTR_HIT_OFFSET 0x0C  /* Cache hit counter */
#define XIP_CTR_ACC_OFFSET 0x10  /* Cache access counter */
#define XIP_STREAM_ADDR    0x14  /* Stream address */
#define XIP_STREAM_CTR     0x18  /* Stream counter */
#define XIP_STREAM_FIFO    0x1C  /* Stream FIFO (read-only) */

static uint32_t xip_ctrl_reg = 0x00000003;  /* EN=1, ERR_BADWRITE=1 (RP2040 default) */
static uint32_t xip_ctr_hit = 0;
static uint32_t xip_ctr_acc = 0;
static uint32_t xip_stream_addr = 0;
static uint32_t xip_stream_ctr = 0;

/* XIP SRAM: 16KB cache memory usable as SRAM */
static uint8_t xip_sram[XIP_SRAM_SIZE];

/* Debug logging for unmapped peripheral accesses */
int mem_debug_unmapped = 0;  /* Set via -debug-mem flag */

/* ========================================================================
 * SYSINFO Registers (0x40000000)
 *
 * Returns chip identification information.
 * ======================================================================== */

#define SYSINFO_BASE        0x40000000
#define SYSINFO_CHIP_ID     0x00
#define SYSINFO_PLATFORM    0x04
#define SYSINFO_GITREF      0x40  /* ROM version git hash */

/* RP2040-B2: REVISION=2, PART=0x0002, MANUFACTURER=Raspberry Pi (0x927) */
#define RP2040_CHIP_ID      ((2u << 28) | (0x0002u << 12) | (0x927u << 1) | 1u)
#define RP2040_PLATFORM     0x00000002  /* ASIC=1, FPGA=0 */

static uint32_t sysinfo_read(uint32_t offset) {
    switch (offset) {
    case SYSINFO_CHIP_ID:  return RP2040_CHIP_ID;
    case SYSINFO_PLATFORM: return RP2040_PLATFORM;
    case SYSINFO_GITREF:   return 0x00000001;  /* Bootrom version */
    default:               return 0;
    }
}

/* ========================================================================
 * BUSCTRL Registers (0x40030000)
 *
 * Bus fabric priority and performance counters.
 * Used by pico_rand for entropy seeding.
 * ======================================================================== */

#define BUSCTRL_BASE 0x40030000

static uint32_t busctrl_bus_priority = 0;
static uint32_t busctrl_perfsel[4] = {0x1F, 0x1F, 0x1F, 0x1F};  /* Reset value */
static uint32_t busctrl_perfctr[4] = {0, 0, 0, 0};

static int busctrl_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;
    return base >= BUSCTRL_BASE && base < BUSCTRL_BASE + 0x1000;
}

static uint32_t busctrl_read(uint32_t offset) {
    switch (offset) {
    case 0x00: return busctrl_bus_priority;
    case 0x04: return 0;  /* BUS_PRIORITY_ACK */
    case 0x08: return busctrl_perfctr[0];
    case 0x0C: return busctrl_perfsel[0];
    case 0x10: return busctrl_perfctr[1];
    case 0x14: return busctrl_perfsel[1];
    case 0x18: return busctrl_perfctr[2];
    case 0x1C: return busctrl_perfsel[2];
    case 0x20: return busctrl_perfctr[3];
    case 0x24: return busctrl_perfsel[3];
    default:   return 0;
    }
}

static void busctrl_write(uint32_t offset, uint32_t val) {
    switch (offset) {
    case 0x00: busctrl_bus_priority = val & 0x1111; break;
    case 0x08: busctrl_perfctr[0] = 0; break;  /* W1C: any write clears */
    case 0x0C: busctrl_perfsel[0] = val & 0x1F; break;
    case 0x10: busctrl_perfctr[1] = 0; break;
    case 0x14: busctrl_perfsel[1] = val & 0x1F; break;
    case 0x18: busctrl_perfctr[2] = 0; break;
    case 0x1C: busctrl_perfsel[2] = val & 0x1F; break;
    case 0x20: busctrl_perfctr[3] = 0; break;
    case 0x24: busctrl_perfsel[3] = val & 0x1F; break;
    default: break;
    }
}

/* ========================================================================
 * IO_QSPI Registers (0x40018000)
 *
 * Minimal stub for QSPI pad GPIO control (6 pins: SCLK, SS, SD0-SD3).
 * Each pin has STATUS (read-only) + CTRL (read/write) = 8 bytes per pin.
 * Total pin registers: 6 * 8 = 48 bytes (0x00-0x2F)
 * Plus INTR/INTE/INTF/INTS at 0x30-0x3C
 * ======================================================================== */

#define IO_QSPI_BASE        0x40018000
#define IO_QSPI_BLOCK_SIZE  0x60

/* Store CTRL registers for 6 QSPI GPIOs + interrupt registers */
static uint32_t io_qspi_ctrl[6];     /* CTRL for SCLK, SS, SD0-SD3 */
static uint32_t io_qspi_inte;
static uint32_t io_qspi_intf;

static int io_qspi_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;
    return (base >= IO_QSPI_BASE && base < IO_QSPI_BASE + IO_QSPI_BLOCK_SIZE);
}

static uint32_t io_qspi_read(uint32_t offset) {
    /* Pin registers: each pin has STATUS at +0, CTRL at +4, 8 bytes apart */
    if (offset < 0x30) {
        uint32_t pin = offset / 8;
        uint32_t reg = offset % 8;
        if (pin < 6) {
            if (reg == 0) return 0;  /* STATUS: always 0 */
            if (reg == 4) return io_qspi_ctrl[pin];
        }
        return 0;
    }
    switch (offset) {
    case 0x30: return 0;              /* INTR (raw) */
    case 0x34: return io_qspi_inte;   /* INTE */
    case 0x38: return io_qspi_intf;   /* INTF */
    case 0x3C: return 0;              /* INTS */
    default:   return 0;
    }
}

static void io_qspi_write(uint32_t offset, uint32_t val) {
    if (offset < 0x30) {
        uint32_t pin = offset / 8;
        uint32_t reg = offset % 8;
        if (pin < 6 && reg == 4) {
            io_qspi_ctrl[pin] = val;
        }
        return;
    }
    switch (offset) {
    case 0x34: io_qspi_inte = val; break;
    case 0x38: io_qspi_intf = val; break;
    default: break;
    }
}

/* ========================================================================
 * PADS_QSPI Registers (0x40020000)
 *
 * Minimal stub for QSPI pad electrical control.
 * ======================================================================== */

#define PADS_QSPI_BASE        0x40020000
#define PADS_QSPI_BLOCK_SIZE  0x20

static uint32_t pads_qspi_regs[8];  /* VOLTAGE_SELECT + 6 pads + spare */

static int pads_qspi_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;
    return (base >= PADS_QSPI_BASE && base < PADS_QSPI_BASE + PADS_QSPI_BLOCK_SIZE);
}

static int gpio_bus_match(uint32_t addr) {
    return ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
            (addr >= IO_BANK0_BASE + REG_ALIAS_XOR_BITS && addr < IO_BANK0_BASE + REG_ALIAS_XOR_BITS + 0x200) ||
            (addr >= IO_BANK0_BASE + REG_ALIAS_SET_BITS && addr < IO_BANK0_BASE + REG_ALIAS_SET_BITS + 0x200) ||
            (addr >= IO_BANK0_BASE + REG_ALIAS_CLR_BITS && addr < IO_BANK0_BASE + REG_ALIAS_CLR_BITS + 0x200) ||
            (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x80) ||
            (addr >= PADS_BANK0_BASE + REG_ALIAS_XOR_BITS && addr < PADS_BANK0_BASE + REG_ALIAS_XOR_BITS + 0x80) ||
            (addr >= PADS_BANK0_BASE + REG_ALIAS_SET_BITS && addr < PADS_BANK0_BASE + REG_ALIAS_SET_BITS + 0x80) ||
            (addr >= PADS_BANK0_BASE + REG_ALIAS_CLR_BITS && addr < PADS_BANK0_BASE + REG_ALIAS_CLR_BITS + 0x80) ||
            (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100));
}

static uint32_t pads_qspi_read(uint32_t offset) {
    if (offset < sizeof(pads_qspi_regs)) return pads_qspi_regs[offset / 4];
    return 0;
}

static void pads_qspi_write(uint32_t offset, uint32_t val) {
    if (offset < sizeof(pads_qspi_regs)) pads_qspi_regs[offset / 4] = val;
}

/* ========================================================================
 * XIP SSI State (0x18000000)
 *
 * This is a minimal emulation of the RP2040 flash serial interface. It only
 * models the register and FIFO behavior needed by Pico SDK flash_do_cmd()
 * and by boot2's XIP re-entry sequence.
 * ======================================================================== */

#define XIP_SSI_FIFO_DEPTH           16

#define XIP_SSI_CTRLR0_OFFSET        0x00
#define XIP_SSI_CTRLR1_OFFSET        0x04
#define XIP_SSI_SSIENR_OFFSET        0x08
#define XIP_SSI_SER_OFFSET           0x10
#define XIP_SSI_BAUDR_OFFSET         0x14
#define XIP_SSI_TXFLR_OFFSET         0x20
#define XIP_SSI_RXFLR_OFFSET         0x24
#define XIP_SSI_SR_OFFSET            0x28
#define XIP_SSI_DR0_OFFSET           0x60
#define XIP_SSI_RX_SAMPLE_DLY_OFFSET 0xF0
#define XIP_SSI_SPI_CTRLR0_OFFSET    0xF4

#define XIP_SSI_SR_RFF               (1u << 4)
#define XIP_SSI_SR_RFNE              (1u << 3)
#define XIP_SSI_SR_TFE               (1u << 2)
#define XIP_SSI_SR_TFNF              (1u << 1)
#define XIP_SSI_SR_BUSY              (1u << 0)

#define FLASH_CMD_READ_STATUS1       0x05
#define FLASH_CMD_WRITE_ENABLE       0x06
#define FLASH_CMD_WRITE_STATUS       0x01
#define FLASH_CMD_READ_STATUS2       0x35
#define FLASH_CMD_READ_UNIQUE_ID     0x4B
#define FLASH_CMD_FAST_READ_QUAD_IO  0xEB

typedef struct {
    uint32_t ctrlr0;
    uint32_t ctrlr1;
    uint32_t ssienr;
    uint32_t ser;
    uint32_t baudr;
    uint32_t rx_sample_dly;
    uint32_t spi_ctrlr0;
    uint32_t rx_fifo[XIP_SSI_FIFO_DEPTH];
    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t rx_count;
    uint8_t command;
    uint8_t phase;
    uint8_t status_reg1;
    uint8_t status_reg2;
    bool write_enable_latched;
    bool transaction_active;
} xip_ssi_state_t;

static xip_ssi_state_t xip_ssi = {
    .ssienr = 1,
    .ser = 1,
    .baudr = 2,
    .status_reg2 = 0x02, /* Report QE already set so boot2 skips SR programming */
};

static const uint8_t xip_ssi_unique_id[8] = {
    0x42, 0x52, 0x41, 0x4D, 0x42, 0x4C, 0x45, 0x01
};

static void xip_ssi_reset_transaction(void) {
    xip_ssi.transaction_active = false;
    xip_ssi.command = 0;
    xip_ssi.phase = 0;
}

static void xip_ssi_clear_rx_fifo(void) {
    xip_ssi.rx_head = 0;
    xip_ssi.rx_tail = 0;
    xip_ssi.rx_count = 0;
}

static void xip_ssi_enqueue_rx(uint32_t val) {
    if (xip_ssi.rx_count >= XIP_SSI_FIFO_DEPTH) {
        return;
    }
    xip_ssi.rx_fifo[xip_ssi.rx_tail] = val;
    xip_ssi.rx_tail = (uint8_t)((xip_ssi.rx_tail + 1) % XIP_SSI_FIFO_DEPTH);
    xip_ssi.rx_count++;
}

static uint32_t xip_ssi_dequeue_rx(void) {
    uint32_t val;

    if (!xip_ssi.rx_count) {
        return 0;
    }

    val = xip_ssi.rx_fifo[xip_ssi.rx_head];
    xip_ssi.rx_head = (uint8_t)((xip_ssi.rx_head + 1) % XIP_SSI_FIFO_DEPTH);
    xip_ssi.rx_count--;
    return val;
}

static uint8_t xip_ssi_expected_writes(uint8_t command) {
    switch (command) {
    case FLASH_CMD_READ_STATUS1:
    case FLASH_CMD_READ_STATUS2:
        return 2;
    case FLASH_CMD_WRITE_ENABLE:
        return 1;
    case FLASH_CMD_WRITE_STATUS:
        return 3;
    case FLASH_CMD_READ_UNIQUE_ID:
        return 13; /* cmd + 4 dummy + 8 data clocks */
    case FLASH_CMD_FAST_READ_QUAD_IO:
        return 2;  /* boot2 issues command + address/mode word */
    default:
        return 1;
    }
}

static uint32_t xip_ssi_exchange_byte(uint8_t tx) {
    uint8_t command;
    uint8_t phase;
    uint32_t rx = 0;

    if (!xip_ssi.transaction_active) {
        xip_ssi.transaction_active = true;
        xip_ssi.command = tx;
        xip_ssi.phase = 0;
    }

    command = xip_ssi.command;
    phase = xip_ssi.phase;

    switch (command) {
    case FLASH_CMD_READ_STATUS1:
        rx = (phase == 1) ? xip_ssi.status_reg1 : 0;
        break;
    case FLASH_CMD_READ_STATUS2:
        rx = (phase == 1) ? xip_ssi.status_reg2 : 0;
        break;
    case FLASH_CMD_WRITE_ENABLE:
        xip_ssi.write_enable_latched = true;
        rx = 0;
        break;
    case FLASH_CMD_WRITE_STATUS:
        if (phase == 1 && xip_ssi.write_enable_latched) {
            xip_ssi.status_reg1 = tx;
        } else if (phase == 2 && xip_ssi.write_enable_latched) {
            xip_ssi.status_reg2 = tx;
            xip_ssi.write_enable_latched = false;
        }
        rx = 0;
        break;
    case FLASH_CMD_READ_UNIQUE_ID:
        if (phase >= 5 && phase < 13) {
            rx = xip_ssi_unique_id[phase - 5];
        }
        break;
    case FLASH_CMD_FAST_READ_QUAD_IO:
        rx = 0;
        break;
    default:
        rx = 0;
        break;
    }

    xip_ssi.phase++;
    if (xip_ssi.phase >= xip_ssi_expected_writes(command)) {
        xip_ssi_reset_transaction();
    }

    return rx;
}

static uint32_t xip_ssi_read(uint32_t offset) {
    uint32_t status = XIP_SSI_SR_TFE | XIP_SSI_SR_TFNF;

    switch (offset) {
    case XIP_SSI_CTRLR0_OFFSET:
        return xip_ssi.ctrlr0;
    case XIP_SSI_CTRLR1_OFFSET:
        return xip_ssi.ctrlr1;
    case XIP_SSI_SSIENR_OFFSET:
        return xip_ssi.ssienr;
    case XIP_SSI_SER_OFFSET:
        return xip_ssi.ser;
    case XIP_SSI_BAUDR_OFFSET:
        return xip_ssi.baudr;
    case XIP_SSI_TXFLR_OFFSET:
        return 0;
    case XIP_SSI_RXFLR_OFFSET:
        return xip_ssi.rx_count;
    case XIP_SSI_SR_OFFSET:
        if (xip_ssi.rx_count) {
            status |= XIP_SSI_SR_RFNE;
        }
        if (xip_ssi.rx_count >= XIP_SSI_FIFO_DEPTH) {
            status |= XIP_SSI_SR_RFF;
        }
        return status;
    case XIP_SSI_DR0_OFFSET:
        return xip_ssi_dequeue_rx();
    case XIP_SSI_RX_SAMPLE_DLY_OFFSET:
        return xip_ssi.rx_sample_dly;
    case XIP_SSI_SPI_CTRLR0_OFFSET:
        return xip_ssi.spi_ctrlr0;
    default:
        return 0;
    }
}

static void xip_ssi_write(uint32_t offset, uint32_t val) {
    switch (offset) {
    case XIP_SSI_CTRLR0_OFFSET:
        xip_ssi.ctrlr0 = val;
        xip_ssi_reset_transaction();
        break;
    case XIP_SSI_CTRLR1_OFFSET:
        xip_ssi.ctrlr1 = val;
        break;
    case XIP_SSI_SSIENR_OFFSET:
        xip_ssi.ssienr = val & 1u;
        if (!xip_ssi.ssienr) {
            xip_ssi_clear_rx_fifo();
            xip_ssi_reset_transaction();
        }
        break;
    case XIP_SSI_SER_OFFSET:
        xip_ssi.ser = val & 1u;
        break;
    case XIP_SSI_BAUDR_OFFSET:
        xip_ssi.baudr = val & 0xFFFFu;
        break;
    case XIP_SSI_DR0_OFFSET:
        if (xip_ssi.ssienr) {
            xip_ssi_enqueue_rx(xip_ssi_exchange_byte((uint8_t)val));
        }
        break;
    case XIP_SSI_RX_SAMPLE_DLY_OFFSET:
        xip_ssi.rx_sample_dly = val;
        break;
    case XIP_SSI_SPI_CTRLR0_OFFSET:
        xip_ssi.spi_ctrlr0 = val;
        break;
    default:
        break;
    }
}

static uint32_t xip_ctrl_read(uint32_t offset) {
    switch (offset) {
    case XIP_CTRL_OFFSET:    return xip_ctrl_reg;
    case XIP_FLUSH_OFFSET:   return 0;  /* Always reads as 0 (strobe register) */
    case XIP_STAT_OFFSET:    return (1u << 2) | (1u << 1);  /* FIFO_EMPTY=1, FLUSH_READY=1 */
    case XIP_CTR_HIT_OFFSET: return xip_ctr_hit;
    case XIP_CTR_ACC_OFFSET: return xip_ctr_acc;
    case XIP_STREAM_ADDR:    return xip_stream_addr;
    case XIP_STREAM_CTR:     return xip_stream_ctr;
    case XIP_STREAM_FIFO:    return 0;  /* No stream data */
    default: return 0;
    }
}

static void xip_ctrl_write(uint32_t offset, uint32_t val) {
    switch (offset) {
    case XIP_CTRL_OFFSET:    xip_ctrl_reg = val & 0x0B; break;  /* EN, ERR_BADWRITE, POWER_DOWN */
    case XIP_FLUSH_OFFSET:   break;  /* Strobe: accept and ignore (no actual cache) */
    case XIP_CTR_HIT_OFFSET: xip_ctr_hit = val; break;
    case XIP_CTR_ACC_OFFSET: xip_ctr_acc = val; break;
    case XIP_STREAM_ADDR:    xip_stream_addr = val; break;
    case XIP_STREAM_CTR:     xip_stream_ctr = val; break;
    default: break;
    }
}

/* ========================================================================
 * SRAM Alias Translation
 *
 * RP2040 mirrors SRAM at 0x21000000 (same as 0x20000000).
 * Translate alias addresses to canonical SRAM addresses.
 * ======================================================================== */

static inline uint32_t sram_alias_translate(uint32_t addr) {
    if (addr >= SRAM_ALIAS_BASE && addr < SRAM_ALIAS_BASE + RAM_SIZE) {
        return addr - SRAM_ALIAS_BASE + RAM_BASE;
    }
    return addr;
}

/* ========================================================================
 * Active RAM pointer for zero-copy dual-core context switching
 *
 * In single-core mode: points to cpu.ram (full 264KB)
 * In dual-core mode: points to cores[active_core].ram (per-core 132KB)
 * ======================================================================== */

static uint8_t *active_ram = NULL;
static uint32_t active_ram_base = RAM_BASE;
static uint32_t active_ram_size = RAM_SIZE;

void mem_set_ram_ptr(uint8_t *ram, uint32_t base, uint32_t size) {
    active_ram = ram;
    active_ram_base = base;
    active_ram_size = size;
}

/* Lazy init: ensure active_ram is set before first use */
static inline uint8_t *get_ram(void) {
    if (!active_ram) active_ram = cpu.ram;
    return active_ram;
}

/* ========================================================================
 * Clock-Domain Peripheral Address Check
 *
 * RP2040 peripherals have atomic register aliases at +0x1000/+0x2000/+0x3000.
 * Each 4KB peripheral occupies a 16KB block. We check the 16KB-aligned base.
 * ======================================================================== */

static int is_clocks_addr(uint32_t addr) {
    uint32_t base = addr & ~0x3FFF; /* 16KB-aligned base */
    return (base == RESETS_BASE  || base == CLOCKS_BASE ||
            base == XOSC_BASE   || base == PLL_SYS_BASE ||
            base == PLL_USB_BASE || base == WATCHDOG_BASE ||
            base == PSM_BASE    || base == ROSC_BASE);
}

static int is_adc_addr(uint32_t addr) {
    return (addr & ~0x3FFF) == ADC_BASE;
}

/* ========================================================================
 * SIO Core-Local State
 * ======================================================================== */

#define SIO_CPUID_OFFSET               0x00
#define SIO_GPIO_IN_OFFSET             0x04
#define SIO_GPIO_HI_IN_OFFSET          0x08
#define SIO_GPIO_OUT_OFFSET            0x10
#define SIO_GPIO_OUT_SET_OFFSET        0x14
#define SIO_GPIO_OUT_CLR_OFFSET        0x18
#define SIO_GPIO_OUT_XOR_OFFSET        0x1C
#define SIO_GPIO_OE_OFFSET             0x20
#define SIO_GPIO_OE_SET_OFFSET         0x24
#define SIO_GPIO_OE_CLR_OFFSET         0x28
#define SIO_GPIO_OE_XOR_OFFSET         0x2C
#define SIO_FIFO_ST_OFFSET             0x50
#define SIO_FIFO_WR_OFFSET             0x54
#define SIO_FIFO_RD_OFFSET             0x58
#define SIO_SPINLOCK_ST_OFFSET         0x5C
#define SIO_DIV_UDIVIDEND_OFFSET       0x60
#define SIO_DIV_UDIVISOR_OFFSET        0x64
#define SIO_DIV_SDIVIDEND_OFFSET       0x68
#define SIO_DIV_SDIVISOR_OFFSET        0x6C
#define SIO_DIV_QUOTIENT_OFFSET        0x70
#define SIO_DIV_REMAINDER_OFFSET       0x74
#define SIO_DIV_CSR_OFFSET             0x78

/* SIO Interpolators (2 per core) */
#define SIO_INTERP0_ACCUM0             0x80
#define SIO_INTERP0_ACCUM1             0x84
#define SIO_INTERP0_BASE0              0x88
#define SIO_INTERP0_BASE1              0x8C
#define SIO_INTERP0_BASE2              0x90
#define SIO_INTERP0_POP_LANE0          0x94
#define SIO_INTERP0_POP_LANE1          0x98
#define SIO_INTERP0_POP_FULL           0x9C
#define SIO_INTERP0_PEEK_LANE0         0xA0
#define SIO_INTERP0_PEEK_LANE1         0xA4
#define SIO_INTERP0_PEEK_FULL          0xA8
#define SIO_INTERP0_CTRL_LANE0         0xAC
#define SIO_INTERP0_CTRL_LANE1         0xB0
#define SIO_INTERP0_ACCUM0_ADD         0xB4
#define SIO_INTERP0_ACCUM1_ADD         0xB8
#define SIO_INTERP0_BASE_1AND0         0xBC

#define SIO_INTERP1_ACCUM0             0xC0
#define SIO_INTERP1_ACCUM1             0xC4
#define SIO_INTERP1_BASE0              0xC8
#define SIO_INTERP1_BASE1              0xCC
#define SIO_INTERP1_BASE2              0xD0
#define SIO_INTERP1_POP_LANE0          0xD4
#define SIO_INTERP1_POP_LANE1          0xD8
#define SIO_INTERP1_POP_FULL           0xDC
#define SIO_INTERP1_PEEK_LANE0         0xE0
#define SIO_INTERP1_PEEK_LANE1         0xE4
#define SIO_INTERP1_PEEK_FULL          0xE8
#define SIO_INTERP1_CTRL_LANE0         0xEC
#define SIO_INTERP1_CTRL_LANE1         0xF0
#define SIO_INTERP1_ACCUM0_ADD         0xF4
#define SIO_INTERP1_ACCUM1_ADD         0xF8
#define SIO_INTERP1_BASE_1AND0         0xFC

#define SIO_FIFO_ST_ROE                (1u << 3)
#define SIO_FIFO_ST_WOF                (1u << 2)
#define SIO_FIFO_ST_RDY                (1u << 1)
#define SIO_FIFO_ST_VLD                (1u << 0)

#define SIO_DIV_CSR_DIRTY              (1u << 1)
#define SIO_DIV_CSR_READY              (1u << 0)

typedef struct {
    uint32_t udividend;
    uint32_t udivisor;
    uint32_t sdividend;
    uint32_t sdivisor;
    uint32_t quotient;
    uint32_t remainder;
    uint32_t csr;
} sio_divider_state_t;

static sio_divider_state_t sio_dividers[NUM_CORES] = {
    [0] = { .csr = SIO_DIV_CSR_READY },
    [1] = { .csr = SIO_DIV_CSR_READY },
};

/* ========================================================================
 * SIO Interpolator State
 *
 * Each core has 2 interpolators (INTERP0, INTERP1). Each interpolator has:
 *   - 2 accumulators (ACCUM0, ACCUM1)
 *   - 3 base values (BASE0, BASE1, BASE2)
 *   - 2 control registers (CTRL_LANE0, CTRL_LANE1)
 *
 * Lane result = (ACCUM >> shift) & mask
 * PEEK/POP operations read lane results; POP also adds BASE to ACCUM.
 * ======================================================================== */

/* Forward declaration (defined below in SIO section) */
static inline int sio_current_core(void);

typedef struct {
    uint32_t accum0;
    uint32_t accum1;
    uint32_t base0;
    uint32_t base1;
    uint32_t base2;
    uint32_t ctrl0;  /* CTRL_LANE0 */
    uint32_t ctrl1;  /* CTRL_LANE1 */
} interp_state_t;

static interp_state_t sio_interp[NUM_CORES][2];

/* Compute lane result: shift and mask the accumulator */
static uint32_t interp_lane_result(interp_state_t *interp, int lane) {
    uint32_t ctrl = lane ? interp->ctrl1 : interp->ctrl0;
    uint32_t accum = lane ? interp->accum1 : interp->accum0;

    uint32_t shift = ctrl & 0x1F;        /* bits [4:0] */
    uint32_t mask_lsb = (ctrl >> 5) & 0x1F;  /* bits [9:5] */
    uint32_t mask_msb = (ctrl >> 10) & 0x1F; /* bits [14:10] */
    int sign_ext = (ctrl >> 15) & 1;     /* bit 15: SIGNED */

    uint32_t val = accum >> shift;

    /* Apply mask: bits from mask_lsb to mask_msb are kept */
    if (mask_msb >= mask_lsb) {
        uint32_t width = mask_msb - mask_lsb + 1;
        uint32_t mask;
        if (width >= 32) {
            mask = 0xFFFFFFFF;
        } else {
            mask = ((1u << width) - 1) << mask_lsb;
        }
        val &= mask;

        /* Sign extension from mask_msb */
        if (sign_ext && (val & (1u << mask_msb))) {
            val |= ~((1u << (mask_msb + 1)) - 1);
        }
    }

    return val;
}

/* Compute FULL result: LANE0 + LANE1 + BASE2 (or LANE0 + BASE2 etc depending on CTRL) */
static uint32_t interp_full_result(interp_state_t *interp) {
    uint32_t lane0 = interp_lane_result(interp, 0);
    uint32_t lane1 = interp_lane_result(interp, 1);
    return interp->base2 + lane0 + lane1;
}

static uint32_t sio_interp_read(int interp_idx, uint32_t reg_offset) {
    int core = sio_current_core();
    interp_state_t *interp = &sio_interp[core][interp_idx];

    switch (reg_offset) {
    case 0x00: return interp->accum0;
    case 0x04: return interp->accum1;
    case 0x08: return interp->base0;
    case 0x0C: return interp->base1;
    case 0x10: return interp->base2;
    case 0x14: /* POP_LANE0: read result then add base0 to accum0 */
    {
        uint32_t r = interp_lane_result(interp, 0);
        interp->accum0 += interp->base0;
        return r;
    }
    case 0x18: /* POP_LANE1 */
    {
        uint32_t r = interp_lane_result(interp, 1);
        interp->accum1 += interp->base1;
        return r;
    }
    case 0x1C: /* POP_FULL */
    {
        uint32_t r = interp_full_result(interp);
        interp->accum0 += interp->base0;
        interp->accum1 += interp->base1;
        return r;
    }
    case 0x20: return interp_lane_result(interp, 0);  /* PEEK_LANE0 */
    case 0x24: return interp_lane_result(interp, 1);  /* PEEK_LANE1 */
    case 0x28: return interp_full_result(interp);      /* PEEK_FULL */
    case 0x2C: return interp->ctrl0;
    case 0x30: return interp->ctrl1;
    case 0x34: return interp->accum0;  /* ACCUM0_ADD read = raw accum0 */
    case 0x38: return interp->accum1;  /* ACCUM1_ADD read = raw accum1 */
    case 0x3C: return interp->base0;   /* BASE_1AND0 read */
    default: return 0;
    }
}

static void sio_interp_write(int interp_idx, uint32_t reg_offset, uint32_t val) {
    int core = sio_current_core();
    interp_state_t *interp = &sio_interp[core][interp_idx];

    switch (reg_offset) {
    case 0x00: interp->accum0 = val; break;
    case 0x04: interp->accum1 = val; break;
    case 0x08: interp->base0 = val; break;
    case 0x0C: interp->base1 = val; break;
    case 0x10: interp->base2 = val; break;
    case 0x2C: interp->ctrl0 = val; break;
    case 0x30: interp->ctrl1 = val; break;
    case 0x34: interp->accum0 += val; break;  /* ACCUM0_ADD */
    case 0x38: interp->accum1 += val; break;  /* ACCUM1_ADD */
    case 0x3C:  /* BASE_1AND0: write BASE0 (lower 16) and BASE1 (upper 16) */
        interp->base0 = val & 0xFFFF;
        interp->base1 = val >> 16;
        break;
    default: break;
    }
}

static uint32_t sio_fifo_sticky[NUM_CORES] = {0};

static inline int sio_current_core(void) {
    int core_id = get_active_core();
    if (core_id < 0 || core_id >= NUM_CORES) {
        return CORE0;
    }
    return core_id;
}

static inline sio_divider_state_t *sio_active_divider(void) {
    return &sio_dividers[sio_current_core()];
}

static void sio_divider_finish(sio_divider_state_t *div, uint32_t quotient, uint32_t remainder) {
    div->quotient = quotient;
    div->remainder = remainder;
    div->csr = SIO_DIV_CSR_READY | SIO_DIV_CSR_DIRTY;
}

static void sio_divider_run_unsigned(sio_divider_state_t *div) {
    if (div->udivisor == 0) {
        sio_divider_finish(div, 0xFFFFFFFFu, div->udividend);
        return;
    }

    sio_divider_finish(div,
                       div->udividend / div->udivisor,
                       div->udividend % div->udivisor);
}

static void sio_divider_run_signed(sio_divider_state_t *div) {
    int32_t dividend = (int32_t)div->sdividend;
    int32_t divisor = (int32_t)div->sdivisor;

    if (divisor == 0) {
        sio_divider_finish(div, 0xFFFFFFFFu, (uint32_t)dividend);
        return;
    }

    /* INT32_MIN / -1 is undefined in C but wraps on RP2040 hardware */
    if (dividend == INT32_MIN && divisor == -1) {
        sio_divider_finish(div, 0x80000000u, 0);
        return;
    }

    sio_divider_finish(div,
                       (uint32_t)(dividend / divisor),
                       (uint32_t)(dividend % divisor));
}

static void sio_write_divider(uint32_t offset, uint32_t val) {
    sio_divider_state_t *div = sio_active_divider();

    switch (offset) {
    case SIO_DIV_UDIVIDEND_OFFSET:
        div->udividend = val;
        sio_divider_run_unsigned(div);
        break;
    case SIO_DIV_UDIVISOR_OFFSET:
        div->udivisor = val;
        sio_divider_run_unsigned(div);
        break;
    case SIO_DIV_SDIVIDEND_OFFSET:
        div->sdividend = val;
        sio_divider_run_signed(div);
        break;
    case SIO_DIV_SDIVISOR_OFFSET:
        div->sdivisor = val;
        sio_divider_run_signed(div);
        break;
    case SIO_DIV_QUOTIENT_OFFSET:
        div->quotient = val;
        div->csr = SIO_DIV_CSR_READY | SIO_DIV_CSR_DIRTY;
        break;
    case SIO_DIV_REMAINDER_OFFSET:
        div->remainder = val;
        div->csr = SIO_DIV_CSR_READY | SIO_DIV_CSR_DIRTY;
        break;
    default:
        break;
    }
}

static uint32_t sio_read_divider(uint32_t offset) {
    sio_divider_state_t *div = sio_active_divider();

    switch (offset) {
    case SIO_DIV_UDIVIDEND_OFFSET:
        return div->udividend;
    case SIO_DIV_UDIVISOR_OFFSET:
        return div->udivisor;
    case SIO_DIV_SDIVIDEND_OFFSET:
        return div->sdividend;
    case SIO_DIV_SDIVISOR_OFFSET:
        return div->sdivisor;
    case SIO_DIV_QUOTIENT_OFFSET:
        div->csr &= ~SIO_DIV_CSR_DIRTY;
        return div->quotient;
    case SIO_DIV_REMAINDER_OFFSET:
        return div->remainder;
    case SIO_DIV_CSR_OFFSET:
        return div->csr;
    default:
        return 0;
    }
}

static uint32_t sio_spinlock_state_bitmap(void) {
    uint32_t bits = 0;
    for (uint32_t i = 0; i < SPINLOCK_SIZE; i++) {
        if (spinlocks[i] & SPINLOCK_LOCKED) {
            bits |= 1u << i;
        }
    }
    return bits;
}

static uint32_t sio_fifo_status(int core_id) {
    uint32_t status = sio_fifo_sticky[core_id];

    if (!fifo_is_empty(core_id)) {
        status |= SIO_FIFO_ST_VLD;
    }
    if (!fifo_is_full(core_id)) {
        status |= SIO_FIFO_ST_RDY;
    }

    return status;
}

static void sio_write32(uint32_t offset, uint32_t val) {
    int core_id = sio_current_core();
    int other_core = (core_id == CORE0) ? CORE1 : CORE0;

    /* Interpolator writes (0x80-0xBF = INTERP0, 0xC0-0xFF = INTERP1) */
    if (offset >= 0x80 && offset <= 0xFC) {
        int interp_idx = (offset >= 0xC0) ? 1 : 0;
        uint32_t reg = (offset - (interp_idx ? 0xC0 : 0x80));
        sio_interp_write(interp_idx, reg, val);
        return;
    }

    switch (offset) {
    case SIO_FIFO_ST_OFFSET:
        sio_fifo_sticky[core_id] &= ~(val & (SIO_FIFO_ST_ROE | SIO_FIFO_ST_WOF));
        break;
    case SIO_FIFO_WR_OFFSET:
        if (other_core == CORE1 && sio_core1_bootrom_handle_fifo_write(val)) {
            break;
        }
        if (!fifo_try_push(other_core, val)) {
            sio_fifo_sticky[core_id] |= SIO_FIFO_ST_WOF;
        }
        break;
    case SIO_DIV_UDIVIDEND_OFFSET:
    case SIO_DIV_UDIVISOR_OFFSET:
    case SIO_DIV_SDIVIDEND_OFFSET:
    case SIO_DIV_SDIVISOR_OFFSET:
    case SIO_DIV_QUOTIENT_OFFSET:
    case SIO_DIV_REMAINDER_OFFSET:
        sio_write_divider(offset, val);
        break;
    default:
        break;
    }
}

static uint32_t sio_read32(uint32_t offset) {
    int core_id = sio_current_core();
    uint32_t val = 0;

    /* Interpolator reads (0x80-0xBF = INTERP0, 0xC0-0xFF = INTERP1) */
    if (offset >= 0x80 && offset <= 0xFC) {
        int interp_idx = (offset >= 0xC0) ? 1 : 0;
        uint32_t reg = (offset - (interp_idx ? 0xC0 : 0x80));
        return sio_interp_read(interp_idx, reg);
    }

    /* GPIO offsets in SIO space - delegate to gpio module */
    if ((offset >= SIO_GPIO_IN_OFFSET && offset <= 0x2C) ||
        (offset >= 0x30 && offset <= 0x4C)) {
        /* 0x04-0x2C = GPIO bank 0, 0x30-0x4C = GPIO_HI (QSPI) */
        if (offset == SIO_GPIO_HI_IN_OFFSET) {
            /* QSPI GPIO input: 6 pins (SCLK=0, SS=1, SD0-3=2-5) */
            /* Default: CS(SS) high, data lines high (pulled up) */
            return 0x3E;
        }
        return gpio_read32(SIO_BASE + offset);
    }

    switch (offset) {
    case SIO_CPUID_OFFSET:
        return (uint32_t)core_id;
    case SIO_FIFO_ST_OFFSET:
        return sio_fifo_status(core_id);
    case SIO_FIFO_RD_OFFSET:
        if (!fifo_try_pop(core_id, &val)) {
            sio_fifo_sticky[core_id] |= SIO_FIFO_ST_ROE;
            return 0;
        }
        return val;
    case SIO_SPINLOCK_ST_OFFSET:
        return sio_spinlock_state_bitmap();
    case SIO_DIV_UDIVIDEND_OFFSET:
    case SIO_DIV_UDIVISOR_OFFSET:
    case SIO_DIV_SDIVIDEND_OFFSET:
    case SIO_DIV_SDIVISOR_OFFSET:
    case SIO_DIV_QUOTIENT_OFFSET:
    case SIO_DIV_REMAINDER_OFFSET:
    case SIO_DIV_CSR_OFFSET:
        return sio_read_divider(offset);
    default:
        return 0;
    }
}

/* ========================================================================
 * Single-Core Memory Access Functions
 * ======================================================================== */

void mem_write32(uint32_t addr, uint32_t val) {
    if (__builtin_expect(gdb.active, 0))
        gdb_check_watchpoint_write(addr, 4);

    /* SRAM alias translation (0x21xxxxxx -> 0x20xxxxxx) */
    addr = sram_alias_translate(addr);

    /* Fast path: RAM writes are the most common case */
    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        uint32_t offset = addr - active_ram_base;
        memcpy(&get_ram()[offset], &val, 4);
        icache_invalidate_addr(addr);
        jit_invalidate_addr(addr);
        if (__builtin_expect(watch_count > 0, 0)) watch_check_write(addr, val, 4);
        if (__builtin_expect(mem_heatmap_enabled, 0)) mem_heatmap_record_write(addr);
        return;
    }

    /* Writes to XIP flash (and uncached aliases) are ignored */
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return;
    }
    if (addr >= XIP_NOALLOC_BASE && addr < XIP_NOALLOC_BASE + FLASH_SIZE) return;
    if (addr >= XIP_NOCACHE_BASE && addr < XIP_NOCACHE_BASE + FLASH_SIZE) return;
    if (addr >= XIP_NOCACHE_NOALLOC && addr < XIP_NOCACHE_NOALLOC + FLASH_SIZE) return;

    /* XIP cache control registers */
    if (addr >= XIP_CTRL_BASE && addr < XIP_CTRL_BASE + 0x20) {
        xip_ctrl_write(addr - XIP_CTRL_BASE, val);
        return;
    }

    /* XIP SRAM (cache as SRAM) */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        uint32_t offset = addr - XIP_SRAM_BASE;
        memcpy(&xip_sram[offset], &val, 4);
        return;
    }

    /* XIP SSI registers (including atomic aliases: XOR +0x1000, SET +0x2000, CLR +0x3000) */
    if (addr >= XIP_SSI_BASE && addr < XIP_SSI_BASE + 0x4000) {
        uint32_t alias = (addr - XIP_SSI_BASE) & 0x3000;
        uint32_t offset = (addr - XIP_SSI_BASE) & 0xFFF;
        if (alias == 0x0000) {
            xip_ssi_write(offset, val);
        } else if (alias == 0x2000) {  /* SET */
            uint32_t cur = xip_ssi_read(offset);
            xip_ssi_write(offset, cur | val);
        } else if (alias == 0x3000) {  /* CLR */
            uint32_t cur = xip_ssi_read(offset);
            xip_ssi_write(offset, cur & ~val);
        } else {  /* XOR 0x1000 */
            uint32_t cur = xip_ssi_read(offset);
            xip_ssi_write(offset, cur ^ val);
        }
        return;
    }

    /* (RAM handled in fast path above) */

    /* UART registers (including atomic aliases) */
    {
        int uart_num = uart_match(addr);
        if (uart_num >= 0) {
            uint32_t alias = addr & 0x3000;
            uint32_t offset = (addr & 0xFFF);
            if (alias == 0x0000) {
                uart_write32(uart_num, offset, val);
            } else if (alias == 0x2000) {  /* SET */
                uint32_t cur = uart_read32(uart_num, offset);
                uart_write32(uart_num, offset, cur | val);
            } else if (alias == 0x3000) {  /* CLR */
                uint32_t cur = uart_read32(uart_num, offset);
                uart_write32(uart_num, offset, cur & ~val);
            } else {  /* XOR 0x1000 */
                uint32_t cur = uart_read32(uart_num, offset);
                uart_write32(uart_num, offset, cur ^ val);
            }
            return;
        }
    }

    /* NVIC registers (0xE000E000 - 0xE000EFFF) */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        nvic_write_register(addr, val);
        return;
    }

    /* Timer registers (including atomic aliases: XOR +0x1000, SET +0x2000, CLR +0x3000) */
    if (addr >= TIMER_BASE && addr < TIMER_BASE + 0x4000) {
        uint32_t alias = (addr - TIMER_BASE) & 0x3000;
        uint32_t reg_addr = TIMER_BASE + ((addr - TIMER_BASE) & 0xFFF);
        if (alias == 0x0000) {
            timer_write32(reg_addr, val);
        } else if (alias == 0x2000) {  /* SET */
            uint32_t cur = timer_read32(reg_addr);
            timer_write32(reg_addr, cur | val);
        } else if (alias == 0x3000) {  /* CLR */
            uint32_t cur = timer_read32(reg_addr);
            timer_write32(reg_addr, cur & ~val);
        } else {  /* XOR 0x1000 */
            uint32_t cur = timer_read32(reg_addr);
            timer_write32(reg_addr, cur ^ val);
        }
        return;
    }

    /* SIO core-local registers */
    if (addr >= SIO_BASE && addr < SIO_BASE + 0x100) {
        sio_write32(addr - SIO_BASE, val);
        return;
    }

    /* GPIO registers - IO_BANK0 base + atomic alias regions for interrupt registers.
     * Aliases at +0x1000 (XOR), +0x2000 (SET), +0x3000 (CLR) used by hw_set_bits/hw_clear_bits. */
    if (gpio_bus_match(addr)) {
        gpio_write32(addr, val);
        return;
    }

    /* SIO spinlocks */
    if (addr >= SPINLOCK_BASE && addr < SPINLOCK_BASE + SPINLOCK_SIZE * 4) {
        uint32_t lock_num = (addr - SPINLOCK_BASE) / 4;
        spinlock_release(lock_num);
        return;
    }

    /* Clock-domain peripherals (Resets, Clocks, XOSC, PLLs, Watchdog) */
    if (is_clocks_addr(addr)) {
        clocks_write32(addr, val);
        return;
    }

    /* ADC */
    if (is_adc_addr(addr)) {
        adc_write32(addr, val);
        return;
    }

    /* SPI peripherals */
    {
        int spi_num = spi_match(addr);
        if (spi_num >= 0) {
            uint32_t alias = addr & 0x3000;
            uint32_t off = addr & 0xFFF;
            if (alias == 0x0000) {
                spi_write32(spi_num, off, val);
            } else if (alias == 0x2000) {
                spi_write32(spi_num, off, spi_read32(spi_num, off) | val);
            } else if (alias == 0x3000) {
                spi_write32(spi_num, off, spi_read32(spi_num, off) & ~val);
            } else {
                spi_write32(spi_num, off, spi_read32(spi_num, off) ^ val);
            }
            return;
        }
    }

    /* I2C peripherals */
    {
        int i2c_num = i2c_match(addr);
        if (i2c_num >= 0) {
            uint32_t alias = addr & 0x3000;
            uint32_t off = addr & 0xFFF;
            if (alias == 0x0000) {
                i2c_write32(i2c_num, off, val);
            } else if (alias == 0x2000) {
                i2c_write32(i2c_num, off, i2c_read32(i2c_num, off) | val);
            } else if (alias == 0x3000) {
                i2c_write32(i2c_num, off, i2c_read32(i2c_num, off) & ~val);
            } else {
                i2c_write32(i2c_num, off, i2c_read32(i2c_num, off) ^ val);
            }
            return;
        }
    }

    /* PWM */
    if (pwm_match(addr)) {
        uint32_t alias = addr & 0x3000;
        uint32_t off = addr & 0xFFF;
        if (alias == 0x0000) {
            pwm_write32(off, val);
        } else if (alias == 0x2000) {
            pwm_write32(off, pwm_read32(off) | val);
        } else if (alias == 0x3000) {
            pwm_write32(off, pwm_read32(off) & ~val);
        } else {
            pwm_write32(off, pwm_read32(off) ^ val);
        }
        return;
    }

    /* DMA controller */
    if (dma_match(addr)) {
        uint32_t alias = addr & 0x3000;
        uint32_t off = addr & 0xFFF;
        if (alias == 0x0000) {
            dma_write32(off, val);
        } else if (alias == 0x2000) {
            dma_write32(off, dma_read32(off) | val);
        } else if (alias == 0x3000) {
            dma_write32(off, dma_read32(off) & ~val);
        } else {
            dma_write32(off, dma_read32(off) ^ val);
        }
        return;
    }

    /* PIO */
    {
        int pio_num = pio_match(addr);
        if (pio_num >= 0) {
            uint32_t alias = addr & 0x3000;
            uint32_t off = addr & 0xFFF;
            if (alias == 0x0000) {
                pio_write32(pio_num, off, val);
            } else if (alias == 0x2000) {
                pio_write32(pio_num, off, pio_read32(pio_num, off) | val);
            } else if (alias == 0x3000) {
                pio_write32(pio_num, off, pio_read32(pio_num, off) & ~val);
            } else {
                pio_write32(pio_num, off, pio_read32(pio_num, off) ^ val);
            }
            return;
        }
    }

    /* USB controller */
    if (usb_match(addr)) {
        usb_write32(addr, val);
        return;
    }

    /* RTC */
    if (rtc_match(addr)) {
        uint32_t alias = addr & 0x3000;
        uint32_t off = addr & 0xFFF;
        if (alias == 0x0000) {
            rtc_write32(off, val);
        } else if (alias == 0x2000) {
            rtc_write32(off, rtc_read32(off) | val);
        } else if (alias == 0x3000) {
            rtc_write32(off, rtc_read32(off) & ~val);
        } else {
            rtc_write32(off, rtc_read32(off) ^ val);
        }
        return;
    }

    /* SYSINFO (read-only, writes ignored) */
    if (addr >= SYSINFO_BASE && addr < SYSINFO_BASE + 0x4000) return;

    /* IO_QSPI registers (including atomic aliases) */
    if (io_qspi_match(addr)) {
        uint32_t alias = addr & 0x3000;
        uint32_t offset = addr & 0xFFF;
        if (alias == 0x0000) {
            io_qspi_write(offset, val);
        } else if (alias == 0x2000) {
            io_qspi_write(offset, io_qspi_read(offset) | val);
        } else if (alias == 0x3000) {
            io_qspi_write(offset, io_qspi_read(offset) & ~val);
        } else {
            io_qspi_write(offset, io_qspi_read(offset) ^ val);
        }
        return;
    }

    /* PADS_QSPI registers (including atomic aliases) */
    if (pads_qspi_match(addr)) {
        uint32_t alias = addr & 0x3000;
        uint32_t offset = addr & 0xFFF;
        if (alias == 0x0000) {
            pads_qspi_write(offset, val);
        } else if (alias == 0x2000) {
            pads_qspi_write(offset, pads_qspi_read(offset) | val);
        } else if (alias == 0x3000) {
            pads_qspi_write(offset, pads_qspi_read(offset) & ~val);
        } else {
            pads_qspi_write(offset, pads_qspi_read(offset) ^ val);
        }
        return;
    }

    /* BUSCTRL */
    if (busctrl_match(addr)) {
        uint32_t alias = addr & 0x3000;
        uint32_t offset = addr & 0xFFF;
        if (alias == 0x0000) {
            busctrl_write(offset, val);
        } else if (alias == 0x2000) {
            busctrl_write(offset, busctrl_read(offset) | val);
        } else if (alias == 0x3000) {
            busctrl_write(offset, busctrl_read(offset) & ~val);
        } else {
            busctrl_write(offset, busctrl_read(offset) ^ val);
        }
        return;
    }

    /* VREG_AND_CHIP_RESET (0x40064000) */
    if (vreg_match(addr)) {
        uint32_t alias = addr & 0x3000;
        uint32_t offset = addr & 0xFFF;
        if (alias == 0x0000) {
            vreg_write(offset, val);
        } else if (alias == 0x2000) {
            vreg_write(offset, vreg_read(offset) | val);
        } else if (alias == 0x3000) {
            vreg_write(offset, vreg_read(offset) & ~val);
        } else {
            vreg_write(offset, vreg_read(offset) ^ val);
        }
        return;
    }

    /* SYSCFG (0x40004000) */
    if (syscfg_match(addr)) {
        uint32_t alias = addr & 0x3000;
        uint32_t offset = addr & 0xFFF;
        if (alias == 0x0000) {
            syscfg_write(offset, val);
        } else if (alias == 0x2000) {
            syscfg_write(offset, syscfg_read(offset) | val);
        } else if (alias == 0x3000) {
            syscfg_write(offset, syscfg_read(offset) & ~val);
        } else {
            syscfg_write(offset, syscfg_read(offset) ^ val);
        }
        return;
    }

    /* TBMAN (0x4006C000) */
    if (tbman_match(addr)) {
        tbman_write(addr & 0xFFF, val);
        return;
    }

    /* Stub out other peripheral writes for now. */
    if (addr >= 0x40000000 && addr < 0x50000000) {
        if (mem_debug_unmapped)
            fprintf(stderr, "[MEM] unmapped write32: 0x%08X = 0x%08X\n", addr, val);
        return;
    }
    if (addr >= SIO_BASE && addr < SIO_BASE + 0x1000) {
        if (mem_debug_unmapped)
            fprintf(stderr, "[MEM] unmapped SIO write32: 0x%08X = 0x%08X\n", addr, val);
        return;
    }

    if (mem_debug_unmapped)
        fprintf(stderr, "[MEM] unmapped write32: 0x%08X = 0x%08X\n", addr, val);
}

void mem_write16(uint32_t addr, uint16_t val) {
    if (__builtin_expect(gdb.active, 0))
        gdb_check_watchpoint_write(addr, 2);
    addr = sram_alias_translate(addr);

    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return;
    }

    /* XIP SRAM */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        uint32_t offset = addr - XIP_SRAM_BASE;
        memcpy(&xip_sram[offset], &val, 2);
        return;
    }

    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        uint32_t offset = addr - active_ram_base;
        memcpy(&get_ram()[offset], &val, 2);
        icache_invalidate_addr(addr);
        jit_invalidate_addr(addr);
        return;
    }

    /* NVIC registers - align to 32-bit boundary for subword access */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        uint32_t addr32 = addr & ~0x3;
        uint32_t current = nvic_read_register(addr32);
        uint8_t offset = addr & 0x3;
        uint32_t mask = 0xFFFF << (offset * 8);
        uint32_t new_val = (current & ~mask) | ((uint32_t)val << (offset * 8));
        nvic_write_register(addr32, new_val);
        return;
    }

    /* USB DPRAM/registers - subword access support */
    if (usb_match(addr)) {
        uint32_t a32 = addr & ~0x3;
        uint32_t cur = usb_read32(a32);
        uint32_t bo = addr & 0x2;  /* halfword offset: 0 or 2 */
        uint32_t mask = 0xFFFF << (bo * 8);
        uint32_t new_val = (cur & ~mask) | ((uint32_t)val << (bo * 8));
        usb_write32(a32, new_val);
        return;
    }

    /* GPIO */
    if (gpio_bus_match(addr)) {
        gpio_write32(addr & ~0x3, val);
        return;
    }

    /* Stub out peripheral writes for now. */
    if (addr >= 0x40000000 && addr < 0x50000000) return;   /* APB/AHB peripherals */
    if (addr >= SIO_BASE     && addr < SIO_BASE + 0x1000) return;
}

void mem_write8(uint32_t addr, uint8_t val) {
    if (__builtin_expect(gdb.active, 0))
        gdb_check_watchpoint_write(addr, 1);
    addr = sram_alias_translate(addr);

    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return;  /* Flash writes ignored */
    }

    /* XIP SRAM */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        xip_sram[addr - XIP_SRAM_BASE] = val;
        return;
    }

    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        get_ram()[addr - active_ram_base] = val;
        icache_invalidate_addr(addr);
        jit_invalidate_addr(addr);
        return;
    }

    /* USB DPRAM/registers - subword access support */
    if (usb_match(addr)) {
        uint32_t a32 = addr & ~0x3;
        uint32_t cur = usb_read32(a32);
        uint32_t bo = addr & 0x3;
        uint32_t mask8 = 0xFF << (bo * 8);
        uint32_t new_val = (cur & ~mask8) | ((uint32_t)val << (bo * 8));
        usb_write32(a32, new_val);
        return;
    }

    /* NVIC registers - align to 32-bit boundary for byte access */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        uint32_t addr32 = addr & ~0x3;
        uint32_t current = nvic_read_register(addr32);
        uint8_t offset = addr & 0x3;
        uint32_t mask = 0xFF << (offset * 8);
        uint32_t new_val = (current & ~mask) | ((uint32_t)val << (offset * 8));
        nvic_write_register(addr32, new_val);
        return;
    }

    /* GPIO */
    if (gpio_bus_match(addr)) {
        gpio_write32(addr & ~0x3, val);
        return;
    }

    /* Stub out peripheral writes for now. */
    if (addr >= 0x40000000 && addr < 0x50000000) return;   /* APB/AHB peripherals */
    if (addr >= SIO_BASE     && addr < SIO_BASE + 0x1000) return;
}

uint32_t mem_read32(uint32_t addr) {
    if (__builtin_expect(gdb.active, 0))
        gdb_check_watchpoint_read(addr, 4);
    /* SRAM alias translation */
    addr = sram_alias_translate(addr);

    /* Fast path: RAM reads are extremely common */
    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        uint32_t offset = addr - active_ram_base;
        uint32_t val;
        memcpy(&val, &get_ram()[offset], 4);
        if (__builtin_expect(watch_count > 0, 0)) watch_check_read(addr, val, 4);
        if (__builtin_expect(mem_heatmap_enabled, 0)) mem_heatmap_record_read(addr);
        return val;
    }

    /* ROM (0x00000000 - 0x00000FFF) */
    if (addr < ROM_SIZE) {
        return rom_read32(addr);
    }

    /* XIP flash (and uncached aliases read from same backing store) */
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint32_t val;
        memcpy(&val, &cpu.flash[offset], 4);
        return val;
    }
    if (addr >= XIP_NOALLOC_BASE && addr < XIP_NOALLOC_BASE + FLASH_SIZE) {
        uint32_t offset = addr - XIP_NOALLOC_BASE;
        uint32_t val;
        memcpy(&val, &cpu.flash[offset], 4);
        return val;
    }
    if (addr >= XIP_NOCACHE_BASE && addr < XIP_NOCACHE_BASE + FLASH_SIZE) {
        uint32_t offset = addr - XIP_NOCACHE_BASE;
        uint32_t val;
        memcpy(&val, &cpu.flash[offset], 4);
        return val;
    }
    if (addr >= XIP_NOCACHE_NOALLOC && addr < XIP_NOCACHE_NOALLOC + FLASH_SIZE) {
        uint32_t offset = addr - XIP_NOCACHE_NOALLOC;
        uint32_t val;
        memcpy(&val, &cpu.flash[offset], 4);
        return val;
    }

    /* XIP cache control registers */
    if (addr >= XIP_CTRL_BASE && addr < XIP_CTRL_BASE + 0x20) {
        return xip_ctrl_read(addr - XIP_CTRL_BASE);
    }

    /* XIP SRAM */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        uint32_t offset = addr - XIP_SRAM_BASE;
        uint32_t val;
        memcpy(&val, &xip_sram[offset], 4);
        return val;
    }

    /* XIP SSI registers (including atomic aliases) */
    if (addr >= XIP_SSI_BASE && addr < XIP_SSI_BASE + 0x4000) {
        uint32_t offset = (addr - XIP_SSI_BASE) & 0xFFF;
        return xip_ssi_read(offset);
    }

    /* (RAM handled in fast path above) */

    /* UART registers (including atomic aliases) */
    {
        int uart_num = uart_match(addr);
        if (uart_num >= 0) {
            uint32_t offset = addr & 0xFFF;
            return uart_read32(uart_num, offset);
        }
    }

    /* NVIC registers (0xE000E000 - 0xE000EFFF) */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        return nvic_read_register(addr);
    }

    /* Timer registers (including atomic aliases) */
    if (addr >= TIMER_BASE && addr < TIMER_BASE + 0x4000) {
        uint32_t reg_addr = TIMER_BASE + ((addr - TIMER_BASE) & 0xFFF);
        return timer_read32(reg_addr);
    }

    /* SIO core-local registers */
    if (addr >= SIO_BASE && addr < SIO_BASE + 0x100) {
        return sio_read32(addr - SIO_BASE);
    }

    /* GPIO registers */
    if (gpio_bus_match(addr)) {
        return gpio_read32(addr);
    }

    /* SIO spinlock state and lock registers */
    if (addr == SIO_BASE + 0x5C) {
        return sio_spinlock_state_bitmap();
    }
    if (addr >= SPINLOCK_BASE && addr < SPINLOCK_BASE + SPINLOCK_SIZE * 4) {
        uint32_t lock_num = (addr - SPINLOCK_BASE) / 4;
        return spinlock_acquire(lock_num);
    }

    /* Clock-domain peripherals (Resets, Clocks, XOSC, PLLs, Watchdog) */
    if (is_clocks_addr(addr)) {
        return clocks_read32(addr);
    }

    /* ADC */
    if (is_adc_addr(addr)) {
        return adc_read32(addr);
    }

    /* SPI peripherals */
    {
        int spi_num = spi_match(addr);
        if (spi_num >= 0) {
            return spi_read32(spi_num, addr & 0xFFF);
        }
    }

    /* I2C peripherals */
    {
        int i2c_num = i2c_match(addr);
        if (i2c_num >= 0) {
            return i2c_read32(i2c_num, addr & 0xFFF);
        }
    }

    /* PWM */
    if (pwm_match(addr)) {
        return pwm_read32(addr & 0xFFF);
    }

    /* DMA controller */
    if (dma_match(addr)) {
        return dma_read32(addr & 0xFFF);
    }

    /* PIO */
    {
        int pio_num = pio_match(addr);
        if (pio_num >= 0) {
            return pio_read32(pio_num, addr & 0xFFF);
        }
    }

    /* USB controller */
    if (usb_match(addr)) {
        return usb_read32(addr);
    }

    /* RTC */
    if (rtc_match(addr)) {
        return rtc_read32(addr & 0xFFF);
    }

    /* SYSINFO */
    if (addr >= SYSINFO_BASE && addr < SYSINFO_BASE + 0x4000) {
        return sysinfo_read(addr & 0xFFF);
    }

    /* IO_QSPI */
    if (io_qspi_match(addr)) {
        return io_qspi_read(addr & 0xFFF);
    }

    /* PADS_QSPI */
    if (pads_qspi_match(addr)) {
        return pads_qspi_read(addr & 0xFFF);
    }

    /* BUSCTRL */
    if (busctrl_match(addr)) {
        return busctrl_read(addr & 0xFFF);
    }

    /* VREG_AND_CHIP_RESET (0x40064000) */
    if (vreg_match(addr)) {
        return vreg_read(addr & 0xFFF);
    }

    /* SYSCFG (0x40004000) */
    if (syscfg_match(addr)) {
        return syscfg_read(addr & 0xFFF);
    }

    /* TBMAN (0x4006C000) */
    if (tbman_match(addr)) {
        return tbman_read(addr & 0xFFF);
    }

    /* Stub peripheral reads: return 0 for now. */
    if (addr >= SIO_BASE && addr < SIO_BASE + 0x1000) {
        if (mem_debug_unmapped)
            fprintf(stderr, "[MEM] unmapped SIO read32: 0x%08X\n", addr);
        return 0;
    }
    if (addr >= 0x40000000 && addr < 0x50000000) {
        if (mem_debug_unmapped)
            fprintf(stderr, "[MEM] unmapped read32: 0x%08X\n", addr);
        return 0;
    }

    if (mem_debug_unmapped)
        fprintf(stderr, "[MEM] unmapped read32: 0x%08X\n", addr);
    return 0;
}

uint16_t mem_read16(uint32_t addr) {
    if (__builtin_expect(gdb.active, 0))
        gdb_check_watchpoint_read(addr, 2);
    addr = sram_alias_translate(addr);

    /* ROM */
    if (addr < ROM_SIZE) {
        return rom_read16(addr);
    }

    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint16_t val;
        memcpy(&val, &cpu.flash[offset], 2);
        return val;
    }

    /* XIP SRAM */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        uint32_t offset = addr - XIP_SRAM_BASE;
        uint16_t val;
        memcpy(&val, &xip_sram[offset], 2);
        return val;
    }

    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        uint32_t offset = addr - active_ram_base;
        uint16_t val;
        memcpy(&val, &get_ram()[offset], 2);
        return val;
    }

    /* NVIC registers - align to 32-bit boundary for subword access */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        uint32_t addr32 = addr & ~0x3;
        uint32_t val32 = nvic_read_register(addr32);
        uint8_t offset = addr & 0x3;
        return (uint16_t)((val32 >> (offset * 8)) & 0xFFFF);
    }

    /* GPIO */
    if (gpio_bus_match(addr)) {
        uint32_t val32 = gpio_read32(addr & ~0x3);
        return (uint16_t)(val32 & 0xFFFF);
    }

    /* USB DPRAM/registers */
    if (usb_match(addr)) {
        uint32_t val32 = usb_read32(addr & ~0x3);
        uint32_t bo = addr & 0x2;
        return (uint16_t)((val32 >> (bo * 8)) & 0xFFFF);
    }

    /* No 16-bit peripheral emulation yet. */
    return 0;
}

uint8_t mem_read8(uint32_t addr) {
    if (__builtin_expect(gdb.active, 0))
        gdb_check_watchpoint_read(addr, 1);
    addr = sram_alias_translate(addr);

    /* ROM */
    if (addr < ROM_SIZE) {
        return rom_read8(addr);
    }

    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return cpu.flash[addr - FLASH_BASE];
    }

    /* XIP SRAM */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        return xip_sram[addr - XIP_SRAM_BASE];
    }

    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        return get_ram()[addr - active_ram_base];
    }

    /* NVIC registers - align to 32-bit boundary for byte access */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        uint32_t addr32 = addr & ~0x3;
        uint32_t val32 = nvic_read_register(addr32);
        uint8_t offset = addr & 0x3;
        return (uint8_t)((val32 >> (offset * 8)) & 0xFF);
    }

    /* GPIO */
    if (gpio_bus_match(addr)) {
        uint32_t val32 = gpio_read32(addr & ~0x3);
        uint8_t byte_offset = addr & 0x3;
        return (uint8_t)((val32 >> (byte_offset * 8)) & 0xFF);
    }

    /* USB DPRAM/registers */
    if (usb_match(addr)) {
        uint32_t val32 = usb_read32(addr & ~0x3);
        uint32_t bo = addr & 0x3;
        return (uint8_t)((val32 >> (bo * 8)) & 0xFF);
    }

    return 0xFF;  /* Unmapped reads return 0xFF */
}

/* ========================================================================
 * Dual-Core Memory Access Functions
 * ======================================================================== */

void mem_write32_dual(int core_id, uint32_t addr, uint32_t val) {
    (void)core_id;
    addr = sram_alias_translate(addr);

    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return;  /* Flash writes ignored */
    }

    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        uint32_t offset = addr - RAM_BASE;
        memcpy(&cpu.ram[offset], &val, 4);
        return;
    }

    /* Route peripheral writes through single-core memory bus */
    mem_write32(addr, val);
}

uint16_t mem_read16_dual(int core_id, uint32_t addr) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    if (addr & 2) {
        return (word >> 16) & 0xFFFF;
    } else {
        return word & 0xFFFF;
    }
}

void mem_write16_dual(int core_id, uint32_t addr, uint16_t val) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    if (addr & 2) {
        word = (word & 0xFFFF) | ((uint32_t)val << 16);
    } else {
        word = (word & 0xFFFF0000) | val;
    }
    mem_write32_dual(core_id, addr & ~3, word);
}

uint8_t mem_read8_dual(int core_id, uint32_t addr) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    return (word >> ((addr & 3) * 8)) & 0xFF;
}

void mem_write8_dual(int core_id, uint32_t addr, uint8_t val) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    uint32_t shift = (addr & 3) * 8;
    word = (word & ~(0xFF << shift)) | ((uint32_t)val << shift);
    mem_write32_dual(core_id, addr & ~3, word);
}

uint32_t mem_read32_dual(int core_id, uint32_t addr) {
    (void)core_id;
    addr = sram_alias_translate(addr);

    /* ROM */
    if (addr < ROM_SIZE) {
        return rom_read32(addr);
    }

    /* Flash is shared across all cores (stored in cpu.flash) */
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint32_t val = 0;
        memcpy(&val, &cpu.flash[offset], 4);
        return val;
    }

    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        uint32_t offset = addr - RAM_BASE;
        uint32_t val = 0;
        memcpy(&val, &cpu.ram[offset], 4);
        return val;
    }

    /* Route peripheral reads through single-core memory bus */
    return mem_read32(addr);
}
