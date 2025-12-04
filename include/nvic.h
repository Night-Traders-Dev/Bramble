#ifndef NVIC_H
#define NVIC_H

#include <stdint.h>

/* ========================================================================
 * ARM Cortex-M0+ NVIC (Nested Vectored Interrupt Controller)
 * ======================================================================== */

/* NVIC Base Address */
#define NVIC_BASE                   0xE000E000

/* NVIC Registers */
#define NVIC_ISER                   (NVIC_BASE + 0x100)   /* Interrupt Set Enable Register */
#define NVIC_ICER                   (NVIC_BASE + 0x180)   /* Interrupt Clear Enable Register */
#define NVIC_ISPR                   (NVIC_BASE + 0x200)   /* Interrupt Set Pending Register */
#define NVIC_ICPR                   (NVIC_BASE + 0x280)   /* Interrupt Clear Pending Register */
#define NVIC_IABR                   (NVIC_BASE + 0x300)  /* IRQ Active Bit Register */
#define NVIC_IPR                    (NVIC_BASE + 0x400)   /* Interrupt Priority Register (0-7) */

/* System Handler Base */
#define SCB_BASE                    (NVIC_BASE + 0xD00)
#define SCB_ICSR                    (SCB_BASE + 0x04)     /* Interrupt Control and State Register */
#define SCB_VTOR                    (SCB_BASE + 0x08)     /* Vector Table Offset Register */

/* System Handlers (Exceptions 0-15) */
#define EXC_RESET                   1
#define EXC_NMI                     2
#define EXC_HARDFAULT               3
#define EXC_MEMFAULT                4      /* M0+ doesn't have this, but kept for reference */
#define EXC_BUSFAULT                5      /* M0+ doesn't have this, but kept for reference */
#define EXC_USAGEFAULT              6      /* M0+ doesn't have this, but kept for reference */
#define EXC_SVCALL                  11
#define EXC_PENDSV                  14
#define EXC_SYSTICK                 15

/* Cortex-M0+ Interrupt Sources (16-31) */
#define IRQ_TIMER_IRQ_0             0      /* TIMER_IRQ_0 */
#define IRQ_TIMER_IRQ_1             1      /* TIMER_IRQ_1 */
#define IRQ_TIMER_IRQ_2             2      /* TIMER_IRQ_2 */
#define IRQ_TIMER_IRQ_3             3      /* TIMER_IRQ_3 */
#define IRQ_PWM_IRQ_WRAP            4      /* PWM_IRQ_WRAP */
#define IRQ_USBCTRL_IRQ_ACPI        5      /* USBCTRL_IRQ */
#define IRQ_XIP_IRQ                 6      /* XIP_IRQ */
#define IRQ_PIO0_IRQ_0              7      /* PIO0_IRQ_0 */
#define IRQ_PIO0_IRQ_1              8      /* PIO0_IRQ_1 */
#define IRQ_PIO1_IRQ_0              9      /* PIO1_IRQ_0 */
#define IRQ_PIO1_IRQ_1              10     /* PIO1_IRQ_1 */
#define IRQ_DMA_IRQ_0               11     /* DMA_IRQ_0 */
#define IRQ_DMA_IRQ_1               12     /* DMA_IRQ_1 */
#define IRQ_IO_IRQ_BANK0            13     /* IO_IRQ_BANK0 */
#define IRQ_IO_IRQ_QSPI             14     /* IO_IRQ_QSPI */
#define IRQ_SIO_IRQ_PROC0           15     /* SIO_IRQ_PROC0 */
#define IRQ_SIO_IRQ_PROC1           16     /* SIO_IRQ_PROC1 */
#define IRQ_CLOCKS_IRQ              17     /* CLOCKS_IRQ */
#define IRQ_SPI0_IRQ                18     /* SPI0_IRQ */
#define IRQ_SPI1_IRQ                19     /* SPI1_IRQ */
#define IRQ_UART0_IRQ               20     /* UART0_IRQ */
#define IRQ_UART1_IRQ               21     /* UART1_IRQ */
#define IRQ_ADC_IRQ_FIFO            22     /* ADC_IRQ_FIFO */
#define IRQ_I2C0_IRQ                23     /* I2C0_IRQ */
#define IRQ_I2C1_IRQ                24     /* I2C1_IRQ */
#define IRQ_RTC_IRQ                 25     /* RTC_IRQ */

#define NUM_EXTERNAL_IRQS           26     /* RP2040 has 26 external IRQs */
#define NUM_EXCEPTIONS              16     /* System exceptions (0-15) */
#define NUM_TOTAL_IRQS              (NUM_EXCEPTIONS + NUM_EXTERNAL_IRQS)  /* 42 total */

/* Priority levels (Cortex-M0+ supports 4 levels, using bits 6-7 of IPR) */
#define IRQ_PRIORITY_0              0      /* Highest priority */
#define IRQ_PRIORITY_1              1
#define IRQ_PRIORITY_2              2
#define IRQ_PRIORITY_3              3      /* Lowest priority */

/* ICSR Register Bits */
#define ICSR_VECTPENDING_SHIFT      12
#define ICSR_VECTPENDING_MASK       0xFF
#define ICSR_ISRPENDING             (1 << 22)
#define ICSR_ISRPREEMPT             (1 << 23)
#define ICSR_PENDSVSET              (1 << 28)
#define ICSR_PENDSVCLR              (1 << 29)
#define ICSR_PENDSTSET              (1 << 30)
#define ICSR_PENDSTCLR              (1 << 31)

/* NVIC State Structure */
typedef struct {
    uint32_t enable;                /* ISER - Interrupt enable bits */
    uint32_t pending;               /* ISPR - Pending interrupt bits */
    uint8_t priority[NUM_EXTERNAL_IRQS];  /* IPR - Priority for each IRQ */
    uint32_t active_exceptions;     /* Bitmask of currently executing exceptions */
    uint32_t iabr;             /* IABR - Active Bit Register */
} nvic_state_t;

/* Functions */
void nvic_init(void);
void nvic_reset(void);

/* Interrupt control */
void nvic_enable_irq(uint32_t irq);
void nvic_disable_irq(uint32_t irq);
void nvic_set_pending(uint32_t irq);
void nvic_clear_pending(uint32_t irq);
void nvic_set_priority(uint32_t irq, uint8_t priority);

/* Interrupt processing */
uint32_t nvic_get_pending_irq(void);
uint32_t nvic_read_register(uint32_t addr);
void nvic_write_register(uint32_t addr, uint32_t val);

/* Signal from peripherals that interrupt occurred */
void nvic_signal_irq(uint32_t irq);

extern nvic_state_t nvic_state;

#endif /* NVIC_H */
