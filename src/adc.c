#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "emulator.h"

/* Global ADC state */
adc_state_t adc_state;

/* Initialize ADC */
void adc_init(void) {
    adc_reset();
}

/* Reset ADC to power-on defaults */
void adc_reset(void) {
    memset(&adc_state, 0, sizeof(adc_state_t));

    /* Default channel values: 0 for GPIO channels */
    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
        adc_state.channel_values[i] = 0;
    }

    /*
     * Temperature sensor default: ~27C
     * RP2040 formula: T = 27 - (ADC_voltage - 0.706) / 0.001721
     * At 27C: voltage = 0.706V, ADC value = 0.706 / 3.3 * 4095 = ~876
     * Use 0x036C (876 decimal)
     */
    adc_state.channel_values[ADC_TEMP_CHANNEL] = 0x036C;
}

/* Set a channel's analog value externally */
void adc_set_channel_value(uint8_t channel, uint16_t value) {
    if (channel < ADC_NUM_CHANNELS) {
        adc_state.channel_values[channel] = value & 0x0FFF; /* 12-bit */
    }
}

/* ========================================================================
 * FIFO helpers
 * ======================================================================== */

static int adc_fifo_push(uint16_t val) {
    if (adc_state.fifo_count >= ADC_FIFO_DEPTH) {
        adc_state.fifo_over = 1;
        return 0;  /* Full — sample dropped */
    }
    adc_state.fifo[adc_state.fifo_wr] = val;
    adc_state.fifo_wr = (adc_state.fifo_wr + 1) % ADC_FIFO_DEPTH;
    adc_state.fifo_count++;
    return 1;
}

static uint16_t adc_fifo_pop(void) {
    if (adc_state.fifo_count == 0) {
        adc_state.fifo_under = 1;
        return 0;
    }
    uint16_t val = adc_state.fifo[adc_state.fifo_rd];
    adc_state.fifo_rd = (adc_state.fifo_rd + 1) % ADC_FIFO_DEPTH;
    adc_state.fifo_count--;
    return val;
}

/* ========================================================================
 * Conversion engine
 * ======================================================================== */

/* Advance AINSEL to next channel in round-robin mask */
static void adc_rrobin_advance(void) {
    uint32_t rrobin = (adc_state.cs & ADC_CS_RROBIN_MASK) >> ADC_CS_RROBIN_SHIFT;
    if (rrobin == 0) return;  /* No round-robin */

    uint32_t ainsel = (adc_state.cs & ADC_CS_AINSEL_MASK) >> ADC_CS_AINSEL_SHIFT;

    /* Find next enabled channel after current */
    for (int i = 1; i <= ADC_NUM_CHANNELS; i++) {
        uint32_t next = (ainsel + i) % ADC_NUM_CHANNELS;
        if (rrobin & (1u << next)) {
            adc_state.cs = (adc_state.cs & ~ADC_CS_AINSEL_MASK) |
                           (next << ADC_CS_AINSEL_SHIFT);
            return;
        }
    }
}

/* Perform one ADC conversion */
void adc_do_conversion(void) {
    uint32_t ainsel = (adc_state.cs & ADC_CS_AINSEL_MASK) >> ADC_CS_AINSEL_SHIFT;
    uint16_t result = 0;

    if (ainsel < ADC_NUM_CHANNELS) {
        result = adc_state.channel_values[ainsel] & 0x0FFF;
    }

    /* Push to FIFO if enabled */
    if (adc_state.fcs & ADC_FCS_EN) {
        uint16_t fifo_val = result;
        if (adc_state.fcs & ADC_FCS_SHIFT) {
            fifo_val = (result >> 4) & 0xFF;  /* Right-shift to 8 bits */
        }
        adc_fifo_push(fifo_val);
    }

    /* Update FIFO interrupt: FIFO level >= threshold */
    uint32_t thresh = (adc_state.fcs & ADC_FCS_THRESH_MASK) >> ADC_FCS_THRESH_SHIFT;
    if (adc_state.fifo_count >= thresh && thresh > 0) {
        adc_state.intr |= 1;  /* FIFO interrupt */
    }

    /* Advance round-robin */
    adc_rrobin_advance();
}

/* ========================================================================
 * Register read
 * ======================================================================== */

uint32_t adc_read32(uint32_t addr) {
    uint32_t offset = addr & 0xFFF;

    switch (offset) {
        case 0x00: /* CS */
            /* Always report READY when not actively converting */
            return (adc_state.cs & ~ADC_CS_READY) | ADC_CS_READY;

        case 0x04: { /* RESULT */
            uint32_t ainsel = (adc_state.cs & ADC_CS_AINSEL_MASK)
                               >> ADC_CS_AINSEL_SHIFT;
            if (ainsel < ADC_NUM_CHANNELS) {
                return adc_state.channel_values[ainsel];
            }
            return 0;
        }

        case 0x08: { /* FCS — build from writable bits + computed status */
            uint32_t fcs = adc_state.fcs & (ADC_FCS_EN | ADC_FCS_SHIFT |
                                             ADC_FCS_ERR | ADC_FCS_DREQ_EN |
                                             ADC_FCS_THRESH_MASK);
            /* Read-only status bits */
            if (adc_state.fifo_count == 0) fcs |= ADC_FCS_EMPTY;
            if (adc_state.fifo_count >= ADC_FIFO_DEPTH) fcs |= ADC_FCS_FULL;
            if (adc_state.fifo_under) fcs |= ADC_FCS_UNDER;
            if (adc_state.fifo_over) fcs |= ADC_FCS_OVER;
            fcs |= ((uint32_t)adc_state.fifo_count << ADC_FCS_LEVEL_SHIFT) &
                   ADC_FCS_LEVEL_MASK;
            return fcs;
        }

        case 0x0C: { /* FIFO — pop from FIFO */
            if (adc_state.fifo_count > 0) {
                return adc_fifo_pop();
            }
            /* Empty FIFO: set underflow, return 0 */
            adc_state.fifo_under = 1;
            return 0;
        }

        case 0x10: /* DIV */
            return adc_state.div;

        case 0x14: /* INTR */
            return adc_state.intr;

        case 0x18: /* INTE */
            return adc_state.inte;

        case 0x1C: /* INTF */
            return 0;

        case 0x20: /* INTS */
            return adc_state.intr & adc_state.inte;

        default:
            return 0;
    }
}

/* ========================================================================
 * Register write
 * ======================================================================== */

void adc_write32(uint32_t addr, uint32_t val) {
    /* Decode atomic alias */
    uint32_t offset_full = addr - (addr & ~0x3FFF);
    uint32_t alias = (offset_full >> 12) & 0x3;
    uint32_t offset = addr & 0xFFF;

    switch (offset) {
        case 0x00: { /* CS */
            uint32_t new_cs;
            switch (alias) {
                case 0: new_cs = val; break;
                case 1: new_cs = adc_state.cs ^ val; break;
                case 2: new_cs = adc_state.cs | val; break;
                case 3: new_cs = adc_state.cs & ~val; break;
                default: new_cs = val; break;
            }
            adc_state.cs = new_cs & (ADC_CS_EN | ADC_CS_TS_EN |
                                     ADC_CS_START_ONCE | ADC_CS_START_MANY |
                                     ADC_CS_AINSEL_MASK | ADC_CS_RROBIN_MASK);
            /* START_ONCE: trigger one conversion, then auto-clear */
            if (adc_state.cs & ADC_CS_START_ONCE) {
                adc_do_conversion();
                adc_state.cs &= ~ADC_CS_START_ONCE;
            }
            break;
        }

        case 0x08: { /* FCS */
            uint32_t new_fcs;
            switch (alias) {
                case 0: new_fcs = val; break;
                case 1: new_fcs = adc_state.fcs ^ val; break;
                case 2: new_fcs = adc_state.fcs | val; break;
                case 3: new_fcs = adc_state.fcs & ~val; break;
                default: new_fcs = val; break;
            }
            /* Writable config bits */
            adc_state.fcs = new_fcs & (ADC_FCS_EN | ADC_FCS_SHIFT |
                                       ADC_FCS_ERR | ADC_FCS_DREQ_EN |
                                       ADC_FCS_THRESH_MASK);
            /* UNDER and OVER are W1C */
            if (val & ADC_FCS_UNDER) adc_state.fifo_under = 0;
            if (val & ADC_FCS_OVER) adc_state.fifo_over = 0;
            break;
        }

        case 0x10: /* DIV */
            switch (alias) {
                case 0: adc_state.div = val; break;
                case 1: adc_state.div ^= val; break;
                case 2: adc_state.div |= val; break;
                case 3: adc_state.div &= ~val; break;
            }
            break;

        case 0x18: /* INTE */
            switch (alias) {
                case 0: adc_state.inte = val; break;
                case 1: adc_state.inte ^= val; break;
                case 2: adc_state.inte |= val; break;
                case 3: adc_state.inte &= ~val; break;
            }
            break;

        default:
            break;
    }
}
