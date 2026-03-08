#ifndef PWM_H
#define PWM_H

#include <stdint.h>

/* RP2040 PWM: 8 slices at 0x40050000 */
#define PWM_BASE        0x40050000
#define PWM_BLOCK_SIZE  0x1000
#define PWM_NUM_SLICES  8

/* Per-slice register offsets (each slice occupies 0x14 bytes) */
#define PWM_CH_CSR      0x00    /* Control and status */
#define PWM_CH_DIV      0x04    /* Clock divider (8.4 fixed point) */
#define PWM_CH_CTR      0x08    /* Counter */
#define PWM_CH_CC       0x0C    /* Compare values (A high, B low) */
#define PWM_CH_TOP      0x10    /* Wrap value */

/* Global PWM registers */
#define PWM_EN          0xA0    /* Enable register (1 bit per slice) */
#define PWM_INTR        0xA4    /* Raw interrupt status */
#define PWM_INTE        0xA8    /* Interrupt enable */
#define PWM_INTF        0xAC    /* Interrupt force */
#define PWM_INTS        0xB0    /* Interrupt status (after masking) */

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
    uint32_t en;    /* Global enable bits */
    uint32_t intr;  /* Raw interrupts */
    uint32_t inte;  /* Interrupt enable */
    uint32_t intf;  /* Interrupt force */
} pwm_state_t;

extern pwm_state_t pwm_state;

void pwm_init(void);
uint32_t pwm_read32(uint32_t offset);
void pwm_write32(uint32_t offset, uint32_t val);
int pwm_match(uint32_t addr);

#endif /* PWM_H */
