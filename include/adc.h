#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/* ========================================================================
 * RP2040 ADC (Analog-to-Digital Converter)
 * 4 external channels + 1 internal temperature sensor
 * ======================================================================== */

/* ADC Base Address */
#define ADC_BASE                0x4004C000

/* ADC Registers */
#define ADC_CS                  (ADC_BASE + 0x00)  /* Control and status */
#define ADC_RESULT              (ADC_BASE + 0x04)  /* Conversion result */
#define ADC_FCS                 (ADC_BASE + 0x08)  /* FIFO control/status */
#define ADC_FIFO_REG            (ADC_BASE + 0x0C)  /* FIFO read */
#define ADC_DIV                 (ADC_BASE + 0x10)  /* Clock divider */
#define ADC_INTR                (ADC_BASE + 0x14)  /* Raw interrupts */
#define ADC_INTE                (ADC_BASE + 0x18)  /* Interrupt enable */
#define ADC_INTF                (ADC_BASE + 0x1C)  /* Interrupt force */
#define ADC_INTS                (ADC_BASE + 0x20)  /* Interrupt status */

/* ADC CS bits */
#define ADC_CS_EN               (1u << 0)   /* ADC enable */
#define ADC_CS_TS_EN            (1u << 1)   /* Temperature sensor enable */
#define ADC_CS_START_ONCE       (1u << 2)   /* Start single conversion */
#define ADC_CS_START_MANY       (1u << 3)   /* Start free-running conversions */
#define ADC_CS_READY            (1u << 8)   /* Conversion complete */
#define ADC_CS_ERR              (1u << 9)   /* Conversion error */
#define ADC_CS_ERR_STICKY       (1u << 10)  /* Sticky error bit */
#define ADC_CS_AINSEL_SHIFT     12
#define ADC_CS_AINSEL_MASK      (0x7u << ADC_CS_AINSEL_SHIFT)
#define ADC_CS_RROBIN_SHIFT     16
#define ADC_CS_RROBIN_MASK      (0x1Fu << ADC_CS_RROBIN_SHIFT)

/* ADC FCS bits */
#define ADC_FCS_EN              (1u << 0)   /* FIFO enable */
#define ADC_FCS_SHIFT           (1u << 1)   /* Right-shift result to 8 bits */
#define ADC_FCS_ERR             (1u << 2)   /* Include error bit in FIFO */
#define ADC_FCS_DREQ_EN         (1u << 3)   /* Assert DMA request */
#define ADC_FCS_EMPTY           (1u << 8)   /* FIFO is empty (read-only) */
#define ADC_FCS_FULL            (1u << 9)   /* FIFO is full (read-only) */
#define ADC_FCS_UNDER           (1u << 10)  /* Underflow (W1C) */
#define ADC_FCS_OVER            (1u << 11)  /* Overflow (W1C) */
#define ADC_FCS_LEVEL_SHIFT     16
#define ADC_FCS_LEVEL_MASK      (0xFu << ADC_FCS_LEVEL_SHIFT)
#define ADC_FCS_THRESH_SHIFT    24
#define ADC_FCS_THRESH_MASK     (0xFu << ADC_FCS_THRESH_SHIFT)

/* Number of ADC channels (4 GPIO + 1 temperature sensor) */
#define ADC_NUM_CHANNELS        5
#define ADC_TEMP_CHANNEL        4

/* ADC FIFO depth */
#define ADC_FIFO_DEPTH          4

/* ADC state */
typedef struct {
    uint32_t cs;                /* Control/status */
    uint32_t fcs;               /* FIFO control/status (writable bits only) */
    uint32_t div;               /* Clock divider */
    uint32_t intr;              /* Raw interrupts */
    uint32_t inte;              /* Interrupt enable */
    uint16_t channel_values[ADC_NUM_CHANNELS]; /* Per-channel values (12-bit) */

    /* FIFO */
    uint16_t fifo[ADC_FIFO_DEPTH];  /* Circular buffer (12-bit results) */
    uint8_t  fifo_rd;               /* Read pointer */
    uint8_t  fifo_wr;               /* Write pointer */
    uint8_t  fifo_count;            /* Number of entries */
    uint8_t  fifo_under;            /* Underflow flag */
    uint8_t  fifo_over;             /* Overflow flag */
} adc_state_t;

/* Functions */
void adc_init(void);
void adc_reset(void);
uint32_t adc_read32(uint32_t addr);
void adc_write32(uint32_t addr, uint32_t val);

/* Set a channel's analog value (for testing or external injection) */
void adc_set_channel_value(uint8_t channel, uint16_t value);

/* Perform one ADC conversion (called when START_ONCE or free-running) */
void adc_do_conversion(void);

extern adc_state_t adc_state;

#endif /* ADC_H */
