#ifndef CLOCKS_H
#define CLOCKS_H

#include <stdint.h>

/* ========================================================================
 * RP2040 Clock-Domain Peripherals
 * Resets, Clocks, XOSC, PLL_SYS, PLL_USB, Watchdog
 * ======================================================================== */

/* Resets Peripheral (0x4000C000) */
#define RESETS_BASE             0x4000C000
#define RESETS_RESET            (RESETS_BASE + 0x00)  /* Reset control */
#define RESETS_WDSEL            (RESETS_BASE + 0x04)  /* Watchdog select */
#define RESETS_RESET_DONE       (RESETS_BASE + 0x08)  /* Reset done status */

/* All 24 resetable peripherals */
#define RESETS_ALL_MASK         0x01FFFFFF

/* Clocks Peripheral (0x40008000) */
#define CLOCKS_BASE             0x40008000
#define NUM_CLOCK_GENERATORS    10  /* GPOUT0-3, REF, SYS, PERI, USB, ADC, RTC */

/* Clock generator indices */
#define CLK_GPOUT0              0
#define CLK_GPOUT1              1
#define CLK_GPOUT2              2
#define CLK_GPOUT3              3
#define CLK_REF                 4
#define CLK_SYS                 5
#define CLK_PERI                6
#define CLK_USB                 7
#define CLK_ADC                 8
#define CLK_RTC                 9

/* Per-clock register offsets (stride = 0x0C) */
#define CLK_CTRL_OFFSET         0x00
#define CLK_DIV_OFFSET          0x04
#define CLK_SELECTED_OFFSET     0x08

/* Additional clocks registers */
#define CLK_SYS_RESUS_CTRL      (CLOCKS_BASE + 0x78)
#define CLK_SYS_RESUS_STATUS    (CLOCKS_BASE + 0x7C)
#define CLK_FC0_REF_KHZ         (CLOCKS_BASE + 0x80)
#define CLK_FC0_STATUS          (CLOCKS_BASE + 0x98)
#define CLK_FC0_RESULT          (CLOCKS_BASE + 0x9C)

/* XOSC (0x40024000) */
#define XOSC_BASE               0x40024000
#define XOSC_CTRL               (XOSC_BASE + 0x00)
#define XOSC_STATUS             (XOSC_BASE + 0x04)
#define XOSC_DORMANT            (XOSC_BASE + 0x08)
#define XOSC_STARTUP            (XOSC_BASE + 0x0C)
#define XOSC_COUNT              (XOSC_BASE + 0x1C)

/* XOSC STATUS bits */
#define XOSC_STATUS_STABLE      (1u << 31)
#define XOSC_STATUS_ENABLED     (1u << 12)

/* PLL_SYS (0x40028000) and PLL_USB (0x4002C000) */
#define PLL_SYS_BASE            0x40028000
#define PLL_USB_BASE            0x4002C000

/* PLL register offsets */
#define PLL_CS_OFFSET           0x00
#define PLL_PWR_OFFSET          0x04
#define PLL_FBDIV_INT_OFFSET    0x08
#define PLL_PRIM_OFFSET         0x0C

/* PLL CS bits */
#define PLL_CS_LOCK             (1u << 31)

/* Watchdog (0x40058000) */
#define WATCHDOG_BASE           0x40058000
#define WATCHDOG_CTRL           (WATCHDOG_BASE + 0x00)
#define WATCHDOG_LOAD           (WATCHDOG_BASE + 0x04)
#define WATCHDOG_REASON         (WATCHDOG_BASE + 0x08)
#define WATCHDOG_SCRATCH0       (WATCHDOG_BASE + 0x0C)
#define WATCHDOG_SCRATCH1       (WATCHDOG_BASE + 0x10)
#define WATCHDOG_SCRATCH2       (WATCHDOG_BASE + 0x14)
#define WATCHDOG_SCRATCH3       (WATCHDOG_BASE + 0x18)
#define WATCHDOG_SCRATCH4       (WATCHDOG_BASE + 0x1C)
#define WATCHDOG_SCRATCH5       (WATCHDOG_BASE + 0x20)
#define WATCHDOG_SCRATCH6       (WATCHDOG_BASE + 0x24)
#define WATCHDOG_SCRATCH7       (WATCHDOG_BASE + 0x28)
#define WATCHDOG_TICK           (WATCHDOG_BASE + 0x2C)

/* Watchdog TICK bits */
#define WATCHDOG_TICK_RUNNING   (1u << 10)
#define WATCHDOG_TICK_ENABLE    (1u << 9)

/* Number of watchdog scratch registers */
#define WATCHDOG_NUM_SCRATCH    8

/* PSM - Power State Machine (0x40010000) */
#define PSM_BASE                0x40010000
#define PSM_FRCE_ON_OFFSET      0x00
#define PSM_FRCE_OFF_OFFSET     0x04
#define PSM_WDSEL_OFFSET        0x08
#define PSM_DONE_OFFSET         0x0C
#define PSM_ALL_MASK            0x0001FFFF
#define PSM_FRCE_OFF_PROC1_BITS (1u << 16)

/* PLL state (shared between PLL_SYS and PLL_USB) */
typedef struct {
    uint32_t cs;
    uint32_t pwr;
    uint32_t fbdiv;
    uint32_t prim;
} pll_state_t;

/* Combined clock-domain peripheral state */
typedef struct {
    /* Resets */
    uint32_t reset;             /* Peripherals held in reset */
    uint32_t wdsel;             /* Watchdog select */

    /* Clocks */
    uint32_t clk_ctrl[NUM_CLOCK_GENERATORS];
    uint32_t clk_div[NUM_CLOCK_GENERATORS];

    /* XOSC */
    uint32_t xosc_ctrl;
    uint32_t xosc_startup;
    uint32_t xosc_count;

    /* PLLs */
    pll_state_t pll_sys;
    pll_state_t pll_usb;

    /* Watchdog */
    uint32_t wdog_ctrl;
    uint32_t wdog_load;
    uint32_t wdog_scratch[WATCHDOG_NUM_SCRATCH];
    uint32_t wdog_tick;

    /* PSM */
    uint32_t psm_frce_on;
    uint32_t psm_frce_off;
    uint32_t psm_wdsel;
} clocks_state_t;

/* Functions */
void clocks_init(void);
void clocks_reset(void);
uint32_t clocks_read32(uint32_t addr);
void clocks_write32(uint32_t addr, uint32_t val);

extern clocks_state_t clocks_state;

#endif /* CLOCKS_H */
