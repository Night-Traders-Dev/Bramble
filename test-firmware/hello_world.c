#include <stdint.h>

/* UART0 Register Addresses */
#define UART0_BASE  0x40034000
#define UART0_DR    (UART0_BASE + 0x000)   /* Data Register */
#define UART0_FR    (UART0_BASE + 0x018)   /* Flag Register */

/* Simple UART write function */
void uart_putc(char c) {
    volatile uint32_t *uart_dr = (uint32_t *)UART0_DR;
    *uart_dr = (uint32_t)c;
}

/* Print string to UART */
void uart_puts(const char *str) {
    while (*str) {
        uart_putc(*str++);
    }
}

/* Entry point - called by boot.S */
int main(void) {
    /* Print message */
    uart_puts("\n=== Bramble RP2040 Emulator ===\n");
    uart_puts("Hello, Bramble!\n");
    uart_puts("Firmware executed successfully.\n\n");
    
    /* Terminate with breakpoint for clean exit */
    __asm__ volatile("bkpt #0");
    
    return 0;
}
