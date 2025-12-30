/*
 * MEMORY ADDRESSES - Bramble RP2040 Emulator
 * 
 * This file documents all correct memory addresses for assembly test code.
 * All addresses match the emulator.h definitions.
 */

#ifndef MEMORY_ADDRESSES_H
#define MEMORY_ADDRESSES_H

/* ========================================================================
 * FLASH (XIP) - Read-only Code/Data
 * ======================================================================== */
#define FLASH_BASE              0x10000000
#define FLASH_SIZE              (2 * 1024 * 1024)    /* 2 MB typical */

/* Boot stages */
#define BOOT2_START             0x10000000           /* Bootrom second stage */
#define BOOT2_SIZE              0x100                /* 256 bytes */
#define VECTOR_TABLE_BASE       0x10000100           /* User app vector table */
#define USER_CODE_START         0x10000100           /* User application code */

/* ========================================================================
 * RAM - Read/Write Data/Stack
 * ======================================================================== */
#define RAM_BASE                0x20000000
#define RAM_SIZE                (264 * 1024)         /* 264 KB on-chip SRAM */
#define RAM_TOP                 (RAM_BASE + RAM_SIZE) /* 0x20042000 */

/* IMPORTANT: All test code must use 0x20042000 as initial SP */

/* ========================================================================
 * UART 0 - Serial Communication (PL011-style)
 * ======================================================================== */
#define UART0_BASE              0x40034000
#define UART0_DR                (UART0_BASE + 0x000) /* Data Register */
#define UART0_ECR               (UART0_BASE + 0x004) /* Error Clear Register */
#define UART0_FR                (UART0_BASE + 0x018) /* Flag Register */
#define UART0_IBRD              (UART0_BASE + 0x024) /* Integer Baud Rate Divisor */
#define UART0_FBRD              (UART0_BASE + 0x028) /* Fractional Baud Rate Divisor */
#define UART0_LCRH              (UART0_BASE + 0x02C) /* Line Control Register High */
#define UART0_CR                (UART0_BASE + 0x030) /* Control Register */
#define UART0_IFLS              (UART0_BASE + 0x034) /* Interrupt FIFO Level Select */
#define UART0_IMSC              (UART0_BASE + 0x038) /* Interrupt Mask Set/Clear */
#define UART0_RIS               (UART0_BASE + 0x03C) /* Raw Interrupt Status */
#define UART0_MIS               (UART0_BASE + 0x040) /* Masked Interrupt Status */
#define UART0_ICR               (UART0_BASE + 0x044) /* Interrupt Clear Register */

/* UART0_FR Flag Bits */
#define UART_FR_CTS             (1 << 0)  /* Clear to Send */
#define UART_FR_DSR             (1 << 1)  /* Data Set Ready */
#define UART_FR_DCD             (1 << 2)  /* Data Carrier Detect */
#define UART_FR_BUSY            (1 << 3)  /* UART Busy */
#define UART_FR_RXFE            (1 << 4)  /* Receive FIFO Empty */
#define UART_FR_TXFF            (1 << 5)  /* Transmit FIFO Full */
#define UART_FR_RXFF            (1 << 6)  /* Receive FIFO Full */
#define UART_FR_TXFE            (1 << 7)  /* Transmit FIFO Empty */

/* ========================================================================
 * TIMER (Monotonic Increasing Counter)
 * ======================================================================== */
#define TIMER_BASE              0x40054000
#define TIMER_TIMEHW            (TIMER_BASE + 0x00)  /* Write high word (pause) */
#define TIMER_TIMELW            (TIMER_BASE + 0x04)  /* Write low word */
#define TIMER_TIMEHR            (TIMER_BASE + 0x08)  /* Read high word */
#define TIMER_TIMELR            (TIMER_BASE + 0x0C)  /* Read low word (latches high) */
#define TIMER_ALARM0            (TIMER_BASE + 0x10)  /* Alarm 0 compare value */
#define TIMER_ALARM1            (TIMER_BASE + 0x14)  /* Alarm 1 compare value */
#define TIMER_ALARM2            (TIMER_BASE + 0x18)  /* Alarm 2 compare value */
#define TIMER_ALARM3            (TIMER_BASE + 0x1C)  /* Alarm 3 compare value */
#define TIMER_ARMED             (TIMER_BASE + 0x20)  /* Bitmask of armed alarms */
#define TIMER_TIMERAWH          (TIMER_BASE + 0x24)  /* Raw high word (no pause) */
#define TIMER_TIMERAWL          (TIMER_BASE + 0x28)  /* Raw low word */
#define TIMER_DBGPAUSE          (TIMER_BASE + 0x2C)  /* Debug pause control */
#define TIMER_PAUSE             (TIMER_BASE + 0x30)  /* Pause timer (write 1 to pause) */
#define TIMER_INTR              (TIMER_BASE + 0x34)  /* Raw interrupt status */
#define TIMER_INTE              (TIMER_BASE + 0x38)  /* Interrupt enable */
#define TIMER_INTF              (TIMER_BASE + 0x3C)  /* Interrupt force (write 1) */
#define TIMER_INTS              (TIMER_BASE + 0x40)  /* Interrupt status (after masking) */

/* Timer interrupt bits (one per alarm) */
#define TIMER_INTR_ALARM0       (1 << 0)
#define TIMER_INTR_ALARM1       (1 << 1)
#define TIMER_INTR_ALARM2       (1 << 2)
#define TIMER_INTR_ALARM3       (1 << 3)

/* ========================================================================
 * GPIO (General Purpose I/O)
 * ======================================================================== */

/* SIO (Single-cycle I/O) - Fast GPIO access */
#define SIO_BASE                0xD0000000
#define SIO_GPIO_IN             (SIO_BASE + 0x004)   /* GPIO input values (RO) */
#define SIO_GPIO_OUT            (SIO_BASE + 0x010)   /* GPIO output values */
#define SIO_GPIO_OUT_SET        (SIO_BASE + 0x014)   /* Atomic GPIO OUT |= mask */
#define SIO_GPIO_OUT_CLR        (SIO_BASE + 0x018)   /* Atomic GPIO OUT &= ~mask */
#define SIO_GPIO_OUT_XOR        (SIO_BASE + 0x01C)   /* Atomic GPIO OUT ^= mask */
#define SIO_GPIO_OE             (SIO_BASE + 0x020)   /* Output enable */
#define SIO_GPIO_OE_SET         (SIO_BASE + 0x024)   /* Atomic OE |= mask */
#define SIO_GPIO_OE_CLR         (SIO_BASE + 0x028)   /* Atomic OE &= ~mask */
#define SIO_GPIO_OE_XOR         (SIO_BASE + 0x02C)   /* Atomic OE ^= mask */

/* IO_BANK0 - Per-pin configuration (register-based) */
#define IO_BANK0_BASE           0x40014000
#define GPIO_STATUS_BASE        (IO_BANK0_BASE + 0x000) /* GPIO status registers */
#define GPIO_CTRL_BASE          (IO_BANK0_BASE + 0x004) /* GPIO control registers */

/* GPIO pin register offsets (8 bytes per pin: status + ctrl) */
#define GPIO_STATUS_OFFSET(n)   (GPIO_STATUS_BASE + (n) * 8)
#define GPIO_CTRL_OFFSET(n)     (GPIO_CTRL_BASE + (n) * 8)

/* Convenience macros for GPIO 25 (LED on Pico) */
#define GPIO25_STATUS           0x40014000 + (25 * 8)  /* Actually: 0x40014000 + 0xC8 = 0x400140C8 */
#define GPIO25_CTRL             0x40014004 + (25 * 8)  /* Actually: 0x40014004 + 0xC8 = 0x400140CC */

/* GPIO function select values */
#define GPIO_FUNC_XIP           0   /* XIP (flash) */
#define GPIO_FUNC_SPI           1   /* SPI */
#define GPIO_FUNC_UART          2   /* UART */
#define GPIO_FUNC_I2C           3   /* I2C */
#define GPIO_FUNC_PWM           4   /* PWM */
#define GPIO_FUNC_SIO           5   /* SIO (software I/O) */
#define GPIO_FUNC_PIO0          6   /* PIO 0 */
#define GPIO_FUNC_PIO1          7   /* PIO 1 */
#define GPIO_FUNC_CLOCK         8   /* Clock */
#define GPIO_FUNC_USB           9   /* USB */

/* PADS_BANK0 - GPIO pad control (slew rate, drive strength, etc.) */
#define PADS_BANK0_BASE         0x4001C000
#define PADS_BANK0_VOLTAGE_SELECT (PADS_BANK0_BASE + 0x00)
#define PADS_BANK0_GPIO(n)      (PADS_BANK0_BASE + 0x04 + (n) * 4) /* Per-pin pad control */

/* ========================================================================
 * NVIC (Nested Vectored Interrupt Controller)
 * ======================================================================== */
#define NVIC_BASE               0xE000E000

/* Interrupt Set/Clear */
#define NVIC_ISER               (NVIC_BASE + 0x100)  /* Interrupt Set Enable Register */
#define NVIC_ICER               (NVIC_BASE + 0x180)  /* Interrupt Clear Enable Register */
#define NVIC_ISPR               (NVIC_BASE + 0x200)  /* Interrupt Set Pending Register */
#define NVIC_ICPR               (NVIC_BASE + 0x280)  /* Interrupt Clear Pending Register */
#define NVIC_IABR               (NVIC_BASE + 0x300)  /* Interrupt Active Bit Register */

/* Priority */
#define NVIC_IPR0               (NVIC_BASE + 0x400)  /* Interrupt Priority 0-3 */
#define NVIC_IPR1               (NVIC_BASE + 0x404)  /* Interrupt Priority 4-7 */
#define NVIC_IPR2               (NVIC_BASE + 0x408)  /* Interrupt Priority 8-11 */
#define NVIC_IPR3               (NVIC_BASE + 0x40C)  /* Interrupt Priority 12-15 */
#define NVIC_IPR4               (NVIC_BASE + 0x410)  /* Interrupt Priority 16-19 */
#define NVIC_IPR5               (NVIC_BASE + 0x414)  /* Interrupt Priority 20-23 */
#define NVIC_IPR6               (NVIC_BASE + 0x418)  /* Interrupt Priority 24-27 */
#define NVIC_IPR7               (NVIC_BASE + 0x41C)  /* Interrupt Priority 28-31 */

/* System Control Block (SCB) */
#define SCB_BASE                (NVIC_BASE + 0xD00)
#define SCB_CPUID               (SCB_BASE + 0x00)
#define SCB_ICSR                (SCB_BASE + 0x04)    /* Interrupt Control and State Register */
#define SCB_VTOR                (SCB_BASE + 0x08)    /* Vector Table Offset Register (RW on M0+) */
#define SCB_AIRCR               (SCB_BASE + 0x0C)
#define SCB_SCR                 (SCB_BASE + 0x10)
#define SCB_CCR                 (SCB_BASE + 0x14)
#define SCB_SHPR2               (SCB_BASE + 0x1C)
#define SCB_SHPR3               (SCB_BASE + 0x20)
#define SCB_SHCSR               (SCB_BASE + 0x24)

/* RP2040 External IRQ Sources (vector number = IRQ number + 16) */
#define IRQ_TIMER_0             0   /* Vector 16 */
#define IRQ_TIMER_1             1   /* Vector 17 */
#define IRQ_TIMER_2             2   /* Vector 18 */
#define IRQ_TIMER_3             3   /* Vector 19 */
#define IRQ_PWM_WRAP            4   /* Vector 20 */
/* ... etc (see nvic.h for full list) */

/* ========================================================================
 * QUICK REFERENCE TABLE
 * ======================================================================== */
/*
 * Resource            Address         Size     Notes
 * ================================================================
 * Boot2               0x10000000      256 B    Second-stage bootloader
 * Vector Table        0x10000100      256 B    User app exceptions
 * User Code           0x10000200      ...      Application starts here
 * 
 * RAM                 0x20000000      264 KB   R/W data, stack, heap
 * RAM Top             0x20042000      (end)    Initial SP value
 * 
 * UART0               0x40034000      ...      Serial (print_string uses 0x40034000)
 * Timer               0x40054000      ...      Monotonic counter
 * GPIO SIO            0xD0000000      ...      Fast GPIO access
 * GPIO IO_BANK0       0x40014000      ...      Per-pin configuration
 * GPIO PADS_BANK0     0x4001C000      ...      Pad control
 * 
 * NVIC                0xE000E000      ...      Interrupt controller
 * SCB                 0xE000ED00      ...      System Control Block
 */

#endif /* MEMORY_ADDRESSES_H */
