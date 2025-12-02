#ifndef EMULATOR_H
#define EMULATOR_H

#include <stdint.h>

// --- Memory Constants ---
#define FLASH_BASE 0x10000000
#define FLASH_SIZE (2 * 1024 * 1024) // 2 MB
#define RAM_BASE   0x20000000
#define RAM_SIZE   (264 * 1024)      // 264 KB

// --- System State ---
typedef struct {
    uint8_t flash[FLASH_SIZE];
    uint8_t ram[RAM_SIZE];
    uint32_t r[16];  // Registers R0-R15
    uint32_t xpsr;   // Status Register
} cpu_state_t;

// Global CPU instance (defined in cpu.c)
extern cpu_state_t cpu;

// --- Function Prototypes ---
void mem_write32(uint32_t addr, uint32_t val);
uint32_t mem_read32(uint32_t addr);
void cpu_step(void);
int load_uf2(const char *filename);

#endif
