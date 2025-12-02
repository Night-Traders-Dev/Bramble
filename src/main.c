#include <stdio.h>
#include <string.h>
#include "emulator.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <firmware.uf2>
", argv[0]);
        return 1;
    }

    // 1. Initialize Memory
    // Note: cpu is defined in cpu.c, so we can access it here
    memset(cpu.flash, 0, FLASH_SIZE);
    memset(cpu.ram, 0, RAM_SIZE);
    memset(cpu.r, 0, sizeof(cpu.r));

    // 2. Load Firmware
    if (!load_uf2(argv[1])) {
        return 1;
    }

    // 3. Boot Sequence
    // Standard RP2040 UF2s have a "Boot2" stage. The actual Application 
    // Vector Table is usually at 0x10000100 (256 bytes in).
    // We shortcut Boot2 and jump straight to the App Reset Handler.
    
    uint32_t vector_table = FLASH_BASE + 0x100; 
    
    // Initial SP is the first word in the Vector Table
    cpu.r[13] = mem_read32(vector_table); 
    
    // Reset Vector (PC) is the second word
    cpu.r[15] = mem_read32(vector_table + 4); 
    
    // Thumb bit: PC LSB must be 1, masked off for execution logic
    cpu.r[15] &= ~1; 

    printf("[System] Booting... SP=0x%08X, PC=0x%08X
", cpu.r[13], cpu.r[15]);

    // 4. Run Loop
    // Limit iterations for safety in this basic test
    for (int i = 0; i < 100; i++) {
        cpu_step();
    }
    
    printf("[System] Halted.
");
    return 0;
}
