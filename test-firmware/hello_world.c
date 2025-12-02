/**
 * Bramble Test Firmware - Hello World for RP2040
 * 
 * This program demonstrates:
 * - UART output via printf
 * - GPIO toggling (LED on Pin 25)
 * - Basic timing with sleep_ms()
 * 
 * Build with:
 *   cd test_firmware/build
 *   cmake ..
 *   make
 * 
 * Output UF2 file: test_firmware/build/hello_world.uf2
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"

/* LED is on GPIO 25 (built-in on Pico board) */
const uint LED_PIN = 25;

/* UART0: TX=GPIO0, RX=GPIO1 */
const uint UART_TX_PIN = 0;
const uint UART_RX_PIN = 1;

/**
 * main() - Entry point for Pico SDK
 * 
 * The Pico SDK startup code (pico_crt0.S) calls this function.
 * Do NOT define _start() - that breaks the startup chain.
 */
int main(void) {
    /* Initialize all standard I/O (UART, USB, etc) */
    stdio_init_all();
    
    /* Add binary metadata (optional, for debugging) */
    bi_decl(bi_program_description("Bramble Test Firmware"));
    bi_decl(bi_program_version_string("0.1.0"));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
    
    /* Initialize LED GPIO */
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);  /* Turn on LED */
    
    /* Print startup banner */
    printf("\n");
    printf("╔════════════════════════════════════╗\n");
    printf("║   Bramble RP2040 Emulator Test     ║\n");
    printf("║         Hello World v0.1.0         ║\n");
    printf("╚════════════════════════════════════╝\n");
    printf("\n");
    
    /* Print system info */
    printf("[SYSTEM] Initializing RP2040 firmware\n");
    printf("[SYSTEM] UART0 configured at 115200 baud\n");
    printf("[SYSTEM] LED on GPIO %d\n", LED_PIN);
    printf("\n");
    
    /* Counter for testing */
    int counter = 0;
    
    /* Main loop */
    while (1) {
        /* Toggle LED */
        gpio_put(LED_PIN, !(gpio_get(LED_PIN)));
        
        /* Print loop counter */
        printf("[TEST %04d] Hello from Bramble emulator!\n", counter++);
        
        /* Detailed output for first few iterations */
        if (counter <= 3) {
            printf("           - Testing UART output\n");
            printf("           - Testing GPIO control\n");
            printf("           - Testing timing\n");
        }
        
        /* Sleep for 1 second */
        sleep_ms(1000);
    }
    
    return 0;
}
