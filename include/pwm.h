#ifndef PWM_H
#define PWM_H

#include <stdint.h>

/* Per-chip PWM layout (pico-sdk src/rp2350/.../regs/pwm.h and rp2040/...).
 *
 * The two chips disagree on almost everything above the per-slice registers:
 * RP2040 has 8 slices with the global block at 0xA0, RP2350 has 12 slices with
 * it at 0xF0 and *two* interrupt outputs. The per-slice registers (CSR, DIV,
 * CTR, CC, TOP at stride 0x14) are the same on both.
 *
 *   register              RP2040    RP2350
 *   slices                   8         12
 *   slice region          0x00-0x9F 0x00-0xEF
 *   EN                     0xA0      0xF0
 *   INTR                   0xA4      0xF4
 *   IRQ0_INTE              0xA8      0xF8
 *   IRQ0_INTF              0xAC      0xFC
 *   IRQ0_INTS              0xB0      0x100
 *   IRQ1_INTE              --        0x104
 *   IRQ1_INTF              --        0x108
 *   IRQ1_INTS              --        0x10C
 *   global width          8 bits    12 bits
 *
 * The emulator modelled RP2040's layout unconditionally, so on RP2350 every
 * write to the global registers landed on a slice register or on nothing --
 * PWM_EN in particular was masked to 8 bits, so slices 8-11 could never be
 * enabled.
 *
 * Note RP2040's base 0x40050000 is RP2350's PLL_SYS; see pwm_match(). */
#define PWM_BASE        0x40050000   /* RP2040 */
#define PWM_BLOCK_SIZE  0x1000

#define PWM_NUM_SLICES_RP2040  8
#define PWM_NUM_SLICES_RP2350 12
#define PWM_NUM_SLICES     PWM_NUM_SLICES_RP2350   /* max, for the state array */

/* Global register offsets, per chip. */
#define PWM2040_G_EN      0xA0
#define PWM2040_G_INTR    0xA4
#define PWM2040_G_INTE    0xA8
#define PWM2040_G_INTF    0xAC
#define PWM2040_G_INTS    0xB0

#define PWM2350_G_EN      0xF0
#define PWM2350_G_INTR    0xF4
#define PWM2350_G_IRQ0_INTE 0xF8
#define PWM2350_G_IRQ0_INTF 0xFC
#define PWM2350_G_IRQ0_INTS 0x100
#define PWM2350_G_IRQ1_INTE 0x104
#define PWM2350_G_IRQ1_INTF 0x108
#define PWM2350_G_IRQ1_INTS 0x10C

/* Aliases for the RP2040 names; on RP2350 they select interrupt output 0. */
#define PWM_EN            PWM2040_G_EN
#define PWM_INTR          PWM2040_G_INTR
#define PWM_INTE          PWM2040_G_INTE
#define PWM_INTF          PWM2040_G_INTF
#define PWM_INTS          PWM2040_G_INTS

/* Per-slice register offsets (each slice occupies 0x14 bytes) */
#define PWM_CH_CSR      0x00    /* Control and status */
#define PWM_CH_DIV      0x04    /* Clock divider (8.4 fixed point) */
#define PWM_CH_CTR      0x08    /* Counter */
#define PWM_CH_CC       0x0C    /* Compare values (A high, B low) */
#define PWM_CH_TOP      0x10    /* Wrap value */

/* CSR bits */
#define PWM_CSR_EN      (1u << 0)   /* Slice enable */
#define PWM_CSR_PH_CORRECT (1u << 1)
#define PWM_CSR_DIVMODE_SHIFT 4

/* Per-slice state */
typedef struct {
    uint32_t csr;   /* Control/status */
    uint32_t div;   /* Clock divider */
    uint32_t ctr;   /* Counter value */
    uint32_t cc;    /* Compare A (high 16) | Compare B (low 16) */
    uint32_t top;   /* Wrap value */
} pwm_slice_t;

typedef struct {
    pwm_slice_t slice[PWM_NUM_SLICES];
    uint32_t en;      /* Global enable bits (1 per slice) */
    uint32_t intr;    /* Raw interrupts */
    uint32_t inte[2]; /* Interrupt enable, per output (RP2040 uses [0] only) */
    uint32_t intf[2]; /* Interrupt force, per output */
} pwm_state_t;

extern pwm_state_t pwm_state;

void pwm_init(void);
uint32_t pwm_read32(uint32_t offset);
void pwm_write32(uint32_t offset, uint32_t val);
int pwm_match(uint32_t addr);

#endif /* PWM_H */
