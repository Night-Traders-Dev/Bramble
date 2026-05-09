/*
 * RP2350 Memory Map and Constants
 *
 * Key differences from RP2040:
 *   - 520KB SRAM (vs 264KB)
 *   - Up to 16MB external flash (vs 2MB)
 *   - Additional peripherals: HSTX, TRNG, SHA-256, OTP, TICKS, CORESIGHT
 *   - Dual Cortex-M33 OR dual Hazard3 RISC-V cores
 *   - TrustZone security extensions (M33 only)
 *   - 150MHz default clock (vs 125MHz)
 */

#ifndef RP2350_MEMMAP_H
#define RP2350_MEMMAP_H

#include <stdint.h>

/* ========================================================================
 * RP2350 Memory Regions
 * ======================================================================== */

/* ROM: 32KB (vs 16KB on RP2040) */
#define RP2350_ROM_BASE         0x00000000
#define RP2350_ROM_SIZE         (32 * 1024)

/* XIP Flash: up to 16MB (vs 2MB) */
#define RP2350_FLASH_BASE       0x10000000
#define RP2350_FLASH_MAX_SIZE   (16 * 1024 * 1024)
#define RP2350_FLASH_DEFAULT    (4 * 1024 * 1024)   /* Pico 2 ships with 4MB */

/* XIP aliases / auxiliary windows */
#define RP2350_XIP_END                     0x14000000
#define RP2350_XIP_SRAM_BASE              0x13FFC000
#define RP2350_XIP_SRAM_SIZE        (16 * 1024)
#define RP2350_XIP_NOCACHE_NOALLOC_BASE   0x14000000
#define RP2350_XIP_MAINTENANCE_BASE       0x18000000
#define RP2350_XIP_NOCACHE_NOTRANSLATE_BASE 0x1C000000

/* SRAM: 520KB (10 banks of 64KB + 8KB scratch) */
#define RP2350_SRAM_BASE        0x20000000
#define RP2350_SRAM_SIZE        (520 * 1024)
#define RP2350_SRAM_END         (RP2350_SRAM_BASE + RP2350_SRAM_SIZE)

/* SRAM aliases */
#define RP2350_SRAM_ALIAS_BASE  0x21000000

/* ========================================================================
 * RP2350 Peripheral Base Addresses
 * (RP2350 reshuffles many APB base addresses compared to RP2040)
 * ======================================================================== */

/* Core APB blocks */
#define RP2350_SYSINFO_BASE     0x40000000
#define RP2350_SYSCFG_BASE      0x40008000
#define RP2350_CLOCKS_BASE      0x40010000
#define RP2350_PSM_BASE         0x40018000
#define RP2350_RESETS_BASE      0x40020000
#define RP2350_IO_BANK0_BASE    0x40028000
#define RP2350_IO_QSPI_BASE     0x40030000
#define RP2350_PADS_BANK0_BASE  0x40038000
#define RP2350_PADS_QSPI_BASE   0x40040000
#define RP2350_XOSC_BASE        0x40048000
#define RP2350_PLL_SYS_BASE     0x40050000
#define RP2350_PLL_USB_BASE     0x40058000
#define RP2350_ACCESSCTRL_BASE  0x40060000
#define RP2350_BUSCTRL_BASE     0x40068000
#define RP2350_UART0_BASE       0x40070000
#define RP2350_UART1_BASE       0x40078000
#define RP2350_SPI0_BASE        0x40080000
#define RP2350_SPI1_BASE        0x40088000
#define RP2350_I2C0_BASE        0x40090000
#define RP2350_I2C1_BASE        0x40098000
#define RP2350_ADC_BASE         0x400A0000
#define RP2350_PWM_BASE         0x400A8000
#define RP2350_TIMER0_BASE      0x400B0000
#define RP2350_TIMER1_BASE      0x400B8000
#define RP2350_HSTX_BASE        0x400C0000
#define RP2350_XIP_CTRL_BASE    0x400C8000
#define RP2350_QMI_BASE         0x400D0000
#define RP2350_WATCHDOG_BASE    0x400D8000
#define RP2350_BOOTRAM_BASE     0x400E0000
#define RP2350_BOOTRAM_END      0x400E0400
#define RP2350_ROSC_BASE        0x400E8000
#define RP2350_TRNG_BASE        0x400F0000
#define RP2350_SHA256_BASE      0x400F8000

/* New RP2350-specific peripherals */
#define RP2350_POWMAN_BASE      0x40100000  /* Power manager */
#define RP2350_TICKS_BASE       0x40108000  /* Tick generators */
#define RP2350_OTP_BASE         0x40120000  /* One-Time Programmable memory */
#define RP2350_OTP_DATA_BASE    0x40130000  /* OTP data readout */
#define RP2350_CORESIGHT_BASE   0x40140000  /* CoreSight trace */
#define RP2350_GLITCH_BASE      0x40158000  /* Glitch detector */
#define RP2350_TBMAN_BASE       0x40160000

/* AHB peripherals (same as RP2040) */
#define RP2350_DMA_BASE         0x50000000
#define RP2350_USBCTRL_BASE     0x50100000
#define RP2350_USBCTRL_REGS_BASE 0x50110000
#define RP2350_PIO0_BASE        0x50200000
#define RP2350_PIO1_BASE        0x50300000
#define RP2350_PIO2_BASE        0x50400000  /* New: third PIO block */
#define RP2350_XIP_AUX_BASE     0x50500000

/* SIO */
#define RP2350_SIO_BASE         0xD0000000

/* ========================================================================
 * RISC-V Specific (Hazard3)
 * ======================================================================== */

/* RISC-V boot: entry point from bootrom */
#define RV_BOOT_ENTRY           0x10000000  /* Flash base (after bootrom) */

/* Hazard3 clock: 150MHz default */
#define RP2350_DEFAULT_CLOCK_MHZ 150

/* RISC-V interrupt controller (CLINT-style, mapped in SIO space) */
#define RP2350_MTIME_BASE       0xD0000100  /* Machine timer (64-bit) */
#define RP2350_MTIMECMP_BASE    0xD0000108  /* Timer compare (per-hart) */

/* Number of external interrupts */
#define RP2350_NUM_IRQS         52  /* vs 26 on RP2040 */

/* ========================================================================
 * RP2350 Chip Identification
 * ======================================================================== */

#define RP2350_CHIP_ID          0x00000002  /* RP2350 */
#define RP2350_CHIP_REVISION    0x00000001  /* B0 */

#endif /* RP2350_MEMMAP_H */
