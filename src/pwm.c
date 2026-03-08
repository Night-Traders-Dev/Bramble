#include <string.h>
#include "pwm.h"

pwm_state_t pwm_state;

void pwm_init(void) {
    memset(&pwm_state, 0, sizeof(pwm_state));
    /* Default TOP = 0xFFFF for all slices */
    for (int i = 0; i < PWM_NUM_SLICES; i++) {
        pwm_state.slice[i].top = 0xFFFF;
        pwm_state.slice[i].div = 0x10;  /* Divider = 1.0 (integer 1, frac 0) */
    }
}

int pwm_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;
    return (base >= PWM_BASE && base < PWM_BASE + PWM_BLOCK_SIZE) ? 1 : 0;
}

uint32_t pwm_read32(uint32_t offset) {
    /* Per-slice registers: slices 0-7 at offsets 0x00-0x9F (0x14 per slice) */
    if (offset < PWM_NUM_SLICES * 0x14) {
        int slice = offset / 0x14;
        int reg = offset % 0x14;
        pwm_slice_t *s = &pwm_state.slice[slice];

        switch (reg) {
        case PWM_CH_CSR: return s->csr;
        case PWM_CH_DIV: return s->div;
        case PWM_CH_CTR: return s->ctr;
        case PWM_CH_CC:  return s->cc;
        case PWM_CH_TOP: return s->top;
        default: return 0;
        }
    }

    /* Global registers */
    switch (offset) {
    case PWM_EN:   return pwm_state.en;
    case PWM_INTR: return pwm_state.intr;
    case PWM_INTE: return pwm_state.inte;
    case PWM_INTF: return pwm_state.intf;
    case PWM_INTS: return (pwm_state.intr | pwm_state.intf) & pwm_state.inte;
    default: return 0;
    }
}

void pwm_write32(uint32_t offset, uint32_t val) {
    /* Per-slice registers */
    if (offset < PWM_NUM_SLICES * 0x14) {
        int slice = offset / 0x14;
        int reg = offset % 0x14;
        pwm_slice_t *s = &pwm_state.slice[slice];

        switch (reg) {
        case PWM_CH_CSR: s->csr = val & 0xFF; break;
        case PWM_CH_DIV: s->div = val & 0x0FFF; break;
        case PWM_CH_CTR: s->ctr = val & 0xFFFF; break;
        case PWM_CH_CC:  s->cc  = val; break;
        case PWM_CH_TOP: s->top = val & 0xFFFF; break;
        default: break;
        }
        return;
    }

    /* Global registers */
    switch (offset) {
    case PWM_EN:
        pwm_state.en = val & 0xFF;
        break;
    case PWM_INTR:
        /* Write-1-to-clear */
        pwm_state.intr &= ~(val & 0xFF);
        break;
    case PWM_INTE:
        pwm_state.inte = val & 0xFF;
        break;
    case PWM_INTF:
        pwm_state.intf = val & 0xFF;
        break;
    default:
        break;
    }
}
