/* hello_world.c - UART only */
#define UART0_BASE  0x40034000
#define UART0_DR    (*(volatile unsigned int *)(UART0_BASE + 0x00))
#define UART0_FR    (*(volatile unsigned int *)(UART0_BASE + 0x18))

static void uart_putc(char c) {
    while (UART0_FR & (1 << 5)) {}  /* Wait while TXFF set */
    UART0_DR = c;
}

static void uart_print(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

void main(void) {
    uart_print("Hello from bare-metal!");
    while (1) {
        __asm__ volatile ("wfi");
    }
}
