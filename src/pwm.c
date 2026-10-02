#include <string.h>
#include "pwm.h"
#include "rp2350_rv/rp2350_memmap.h"
#include "nvic.h"
#include "emulator.h"  /* membus_rp2350_mode */

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

    /* RP2040's PWM is at 0x40050000, which on RP2350 is PLL_SYS. Emulating the
     * PWM there shadowed the PLL, so RISC-V firmware that configured a PLL read
     * back PWM register values -- and RP2350's real PWM at 0x400A8000 was
     * unmapped entirely. Only claim the block on the chip that has it. */
    if (!membus_rp2350_mode) {
        if (base >= PWM_BASE && base < PWM_BASE + PWM_BLOCK_SIZE)
            return 1;
    } else if (base >= RP2350_PWM_BASE && base < RP2350_PWM_BASE + PWM_BLOCK_SIZE) {
        return 1;
    }
    return 0;
}

/* Slice count and the global register block, per chip. RP2350 has 12 slices and
 * two interrupt outputs at different offsets; RP2040 has 8 and one. */
static int pwm_num_slices(void) {
    return membus_rp2350_mode ? PWM_NUM_SLICES_RP2350 : PWM_NUM_SLICES_RP2040;
}

static uint32_t pwm_global_mask(void) {
    return membus_rp2350_mode ? 0xFFFu : 0xFFu;
}

uint32_t pwm_read32(uint32_t offset) {
    /* Per-slice registers. The region is per-chip: 8 slices on RP2040
     * (0x00-0x9F), 12 on RP2350 (0x00-0xEF). */
    if (offset < (uint32_t)pwm_num_slices() * 0x14u) {
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

    /* Global registers. RP2350's block sits 0x50 further on and has two
     * interrupt outputs, so it cannot be matched with the RP2040 offsets. */
    switch (offset) {
    case PWM2040_G_EN: case PWM2350_G_EN:
        return pwm_state.en;
    case PWM2040_G_INTR: case PWM2350_G_INTR:
        return pwm_state.intr;
    case PWM2040_G_INTE: case PWM2350_G_IRQ0_INTE:
        return pwm_state.inte[0];
    case PWM2040_G_INTF: case PWM2350_G_IRQ0_INTF:
        return pwm_state.intf[0];
    case PWM2040_G_INTS: case PWM2350_G_IRQ0_INTS:
        return (pwm_state.intr | pwm_state.intf[0]) & pwm_state.inte[0];
    case PWM2350_G_IRQ1_INTE: return pwm_state.inte[1];
    case PWM2350_G_IRQ1_INTF: return pwm_state.intf[1];
    case PWM2350_G_IRQ1_INTS:
        return (pwm_state.intr | pwm_state.intf[1]) & pwm_state.inte[1];
    default: return 0;
    }
}

void pwm_write32(uint32_t offset, uint32_t val) {
    /* Per-slice registers */
    if (offset < (uint32_t)pwm_num_slices() * 0x14u) {
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

    /* Global registers (per-chip offsets; see pwm_read32). */
    {
        uint32_t m = pwm_global_mask();
        switch (offset) {
        case PWM2040_G_EN: case PWM2350_G_EN:
            /* Aliases the per-slice CSR enable bits, so it must be pushed down.
             * The mask was 0xFF, so on RP2350 slices 8-11 were unreachable. */
            pwm_state.en = val & m;
            for (int i = 0; i < pwm_num_slices(); i++) {
                if (val & (1u << i)) pwm_state.slice[i].csr |= PWM_CSR_EN;
                else              pwm_state.slice[i].csr &= ~PWM_CSR_EN;
            }
            break;
        case PWM2040_G_INTR: case PWM2350_G_INTR:
            pwm_state.intr &= ~(val & m);      /* write-1-to-clear */
            break;
        case PWM2040_G_INTE: case PWM2350_G_IRQ0_INTE:
            pwm_state.inte[0] = val & m;
            break;
        case PWM2040_G_INTF: case PWM2350_G_IRQ0_INTF:
            pwm_state.intf[0] = val & m;
            break;
        case PWM2350_G_IRQ1_INTE:
            pwm_state.inte[1] = val & m;
            break;
        case PWM2350_G_IRQ1_INTF:
            pwm_state.intf[1] = val & m;
            break;
        default:
            break;
        }
    }

    /* Signal NVIC if any masked interrupt is active on either output. On
     * RP2350 the two outputs are distinct vectors, so each is raised
     * separately. */
    if ((pwm_state.intr | pwm_state.intf[0]) & pwm_state.inte[0])
        nvic_signal_irq(IRQ_PWM_IRQ_WRAP);
    if (membus_rp2350_mode &&
        (pwm_state.intr | pwm_state.intf[1]) & pwm_state.inte[1])
        nvic_signal_irq(IRQ_PWM_IRQ_WRAP);   /* nvic_irq_number() maps this */
}
