#ifndef EMULATOR_H
#define EMULATOR_H

#include <stdint.h>
#include <stddef.h>

/* Memory Layout (RP2040) */
#define FLASH_BASE  0x10000000
#define FLASH_SIZE  (2 * 1024 * 1024)      /* 2 MB XIP flash on typical Pico boards */
#define RAM_BASE    0x20000000
#define RAM_SIZE    (264 * 1024)           /* 264 KB on-chip SRAM */
#define RAM_TOP (RAM_BASE + RAM_SIZE)
/* MEM_END is an emulator guard, not a hardware boundary */
#define MEM_END     0x30000000

/* SIO (Single-cycle I/O) Base (RP2040) */
#define SIO_BASE    0xD0000000             /* SIO registers block base address */

/* UART0 Registers (RP2040, PL011-style) */
#define UART0_BASE  0x40034000
#define UART0_DR    (UART0_BASE + 0x000)   /* Data Register */
#define UART0_FR    (UART0_BASE + 0x018)   /* Flag Register */

#define STEPS       0

/* CPU State */
typedef struct {
    uint8_t  flash[FLASH_SIZE];
    uint8_t  ram[RAM_SIZE];
    uint32_t r[16];        /* R0-R15 (R13=SP, R14=LR, R15=PC) */
    uint32_t xpsr;         /* Application Program Status Register */
    uint32_t step_count;
} cpu_state_t;

extern cpu_state_t cpu;

/* Function Prototypes */
void     mem_write32(uint32_t addr, uint32_t val);
uint32_t mem_read32(uint32_t addr);
uint16_t mem_read16(uint32_t addr);

void cpu_init(void);
void cpu_step(void);
int  cpu_is_halted(void);

int  load_uf2(const char *filename);

#endif
