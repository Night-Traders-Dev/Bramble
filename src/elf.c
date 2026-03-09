/*
 * ELF Loader for RP2040 Emulator
 *
 * Loads ELF32 ARM binaries directly into flash memory,
 * avoiding the need for UF2 conversion.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "emulator.h"

/* ELF32 Header */
typedef struct {
    uint8_t  e_ident[16];    /* Magic number and other info */
    uint16_t e_type;         /* Object file type */
    uint16_t e_machine;      /* Architecture */
    uint32_t e_version;      /* Object file version */
    uint32_t e_entry;        /* Entry point virtual address */
    uint32_t e_phoff;        /* Program header table file offset */
    uint32_t e_shoff;        /* Section header table file offset */
    uint32_t e_flags;        /* Processor-specific flags */
    uint16_t e_ehsize;       /* ELF header size in bytes */
    uint16_t e_phentsize;    /* Program header table entry size */
    uint16_t e_phnum;        /* Program header table entry count */
    uint16_t e_shentsize;    /* Section header table entry size */
    uint16_t e_shnum;        /* Section header table entry count */
    uint16_t e_shstrndx;     /* Section header string table index */
} elf32_ehdr_t;

/* ELF32 Program Header */
typedef struct {
    uint32_t p_type;         /* Segment type */
    uint32_t p_offset;       /* Segment file offset */
    uint32_t p_vaddr;        /* Segment virtual address */
    uint32_t p_paddr;        /* Segment physical address */
    uint32_t p_filesz;       /* Segment size in file */
    uint32_t p_memsz;        /* Segment size in memory */
    uint32_t p_flags;        /* Segment flags */
    uint32_t p_align;        /* Segment alignment */
} elf32_phdr_t;

/* ELF constants */
#define EI_MAG0     0
#define ELFMAG0     0x7f
#define ELFMAG1     'E'
#define ELFMAG2     'L'
#define ELFMAG3     'F'
#define ELFCLASS32  1
#define ELFDATA2LSB 1       /* Little endian */
#define EM_ARM      40      /* ARM architecture */
#define PT_LOAD     1       /* Loadable segment */

int load_elf(const char *filename) {
    FILE *f = fopen(filename, "rb");
    if (!f) {
        perror("[ELF] Failed to open file");
        return 0;
    }

    /* Read ELF header */
    elf32_ehdr_t ehdr;
    if (fread(&ehdr, 1, sizeof(ehdr), f) != sizeof(ehdr)) {
        printf("[ELF] ERROR: Failed to read ELF header\n");
        fclose(f);
        return 0;
    }

    /* Validate ELF magic */
    if (ehdr.e_ident[EI_MAG0] != ELFMAG0 ||
        ehdr.e_ident[1] != ELFMAG1 ||
        ehdr.e_ident[2] != ELFMAG2 ||
        ehdr.e_ident[3] != ELFMAG3) {
        printf("[ELF] ERROR: Not a valid ELF file\n");
        fclose(f);
        return 0;
    }

    /* Check 32-bit, little-endian, ARM */
    if (ehdr.e_ident[4] != ELFCLASS32) {
        printf("[ELF] ERROR: Not a 32-bit ELF (class=%d)\n", ehdr.e_ident[4]);
        fclose(f);
        return 0;
    }
    if (ehdr.e_ident[5] != ELFDATA2LSB) {
        printf("[ELF] ERROR: Not little-endian\n");
        fclose(f);
        return 0;
    }
    if (ehdr.e_machine != EM_ARM) {
        printf("[ELF] ERROR: Not ARM architecture (machine=%d)\n", ehdr.e_machine);
        fclose(f);
        return 0;
    }

    printf("[ELF] Valid ELF32 ARM binary\n");
    printf("[ELF] Entry point: 0x%08X\n", ehdr.e_entry);
    printf("[ELF] Program headers: %d (offset 0x%X)\n", ehdr.e_phnum, ehdr.e_phoff);

    if (ehdr.e_phnum == 0 || ehdr.e_phoff == 0) {
        printf("[ELF] ERROR: No program headers\n");
        fclose(f);
        return 0;
    }

    /* Read and process program headers */
    int segments_loaded = 0;

    for (int i = 0; i < ehdr.e_phnum; i++) {
        elf32_phdr_t phdr;
        long offset = (long)ehdr.e_phoff + (long)i * ehdr.e_phentsize;

        if (fseek(f, offset, SEEK_SET) != 0) {
            printf("[ELF] ERROR: Failed to seek to program header %d\n", i);
            continue;
        }

        if (fread(&phdr, 1, sizeof(phdr), f) != sizeof(phdr)) {
            printf("[ELF] ERROR: Failed to read program header %d\n", i);
            continue;
        }

        /* Only load PT_LOAD segments */
        if (phdr.p_type != PT_LOAD) {
            continue;
        }

        printf("[ELF] LOAD segment %d: vaddr=0x%08X paddr=0x%08X filesz=%u memsz=%u\n",
               i, phdr.p_vaddr, phdr.p_paddr, phdr.p_filesz, phdr.p_memsz);

        /* Load segments to their runtime virtual address.
         *
         * Pico SDK ELFs commonly use p_paddr as the load memory address (LMA)
         * in flash for initialized RAM data, while p_vaddr is the execution
         * address. Using p_paddr for all segments traps startup in crt0's
         * data copy loop because .data never actually appears in RAM.
         *
         * However, crt0 still copies initialized RAM data from the flash LMA.
         * For RAM segments with a flash p_paddr, we therefore need both:
         *   1. the runtime bytes present in SRAM at p_vaddr, and
         *   2. the same file bytes mirrored into flash at p_paddr.
         */
        uint32_t target = phdr.p_vaddr;

        /* Load into flash */
        if (target >= FLASH_BASE && target + phdr.p_memsz <= FLASH_BASE + FLASH_SIZE) {
            uint32_t flash_offset = target - FLASH_BASE;

            /* Zero the memory region first (for .bss-like sections where memsz > filesz) */
            if (phdr.p_memsz > 0) {
                memset(&cpu.flash[flash_offset], 0, phdr.p_memsz);
            }

            /* Load file data */
            if (phdr.p_filesz > 0) {
                if (fseek(f, (long)phdr.p_offset, SEEK_SET) != 0) {
                    printf("[ELF] ERROR: Failed to seek to segment data\n");
                    continue;
                }

                size_t read = fread(&cpu.flash[flash_offset], 1, phdr.p_filesz, f);
                if (read != phdr.p_filesz) {
                    printf("[ELF] WARNING: Only read %zu of %u bytes\n", read, phdr.p_filesz);
                }
            }

            printf("[ELF] Loaded %u bytes to flash[0x%08X]\n", phdr.p_filesz, flash_offset);
            segments_loaded++;
        }
        /* Load into RAM */
        else if (target >= RAM_BASE && target + phdr.p_memsz <= RAM_BASE + RAM_SIZE) {
            uint32_t ram_offset = target - RAM_BASE;

            if (phdr.p_memsz > 0) {
                memset(&cpu.ram[ram_offset], 0, phdr.p_memsz);
            }

            if (phdr.p_filesz > 0) {
                if (fseek(f, (long)phdr.p_offset, SEEK_SET) != 0) {
                    printf("[ELF] ERROR: Failed to seek to segment data\n");
                    continue;
                }

                size_t read = fread(&cpu.ram[ram_offset], 1, phdr.p_filesz, f);
                if (read != phdr.p_filesz) {
                    printf("[ELF] WARNING: Only read %zu of %u bytes\n", read, phdr.p_filesz);
                }
            }

            printf("[ELF] Loaded %u bytes to RAM[0x%08X]\n", phdr.p_filesz, ram_offset);

            if (phdr.p_paddr >= FLASH_BASE &&
                phdr.p_paddr + phdr.p_filesz <= FLASH_BASE + FLASH_SIZE &&
                phdr.p_paddr != phdr.p_vaddr) {
                uint32_t flash_offset = phdr.p_paddr - FLASH_BASE;

                if (fseek(f, (long)phdr.p_offset, SEEK_SET) != 0) {
                    printf("[ELF] ERROR: Failed to seek to RAM LMA data\n");
                    continue;
                }

                size_t read = fread(&cpu.flash[flash_offset], 1, phdr.p_filesz, f);
                if (read != phdr.p_filesz) {
                    printf("[ELF] WARNING: Only mirrored %zu of %u bytes to flash LMA\n",
                           read, phdr.p_filesz);
                } else {
                    printf("[ELF] Mirrored %u RAM-init bytes to flash[0x%08X]\n",
                           phdr.p_filesz, flash_offset);
                }
            }

            segments_loaded++;
        }
        else {
            printf("[ELF] WARNING: Segment target 0x%08X outside flash/RAM bounds, skipping\n", target);
        }
    }

    fclose(f);
    printf("[ELF] Load complete: %d segments loaded\n", segments_loaded);
    return (segments_loaded > 0);
}
