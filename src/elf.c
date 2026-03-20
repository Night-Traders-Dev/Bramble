/*
 * ELF Loader for RP2040 Emulator
 *
 * Loads ELF32 ARM binaries directly into flash memory,
 * avoiding the need for UF2 conversion.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
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
#define EM_RISCV    243     /* RISC-V architecture */
#define PT_LOAD     1       /* Loadable segment */

/* Detected architecture (shared with uf2.c via loader_detected_arch()) */
extern int loader_detected_arch(void);

static int checked_seek(FILE *f, uint64_t offset) {
    if (offset > (uint64_t)LONG_MAX) {
        return 0;
    }
    return fseek(f, (long)offset, SEEK_SET) == 0;
}

static int region_contains(uint32_t base, uint32_t region_size,
                           uint32_t addr, uint32_t size) {
    if (addr < base) {
        return 0;
    }

    uint64_t offset = (uint64_t)addr - (uint64_t)base;
    return offset <= region_size && size <= region_size - offset;
}

int load_elf(const char *filename) {
    FILE *f = fopen(filename, "rb");
    if (!f) {
        perror("[ELF] Failed to open file");
        return 0;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fprintf(stderr, "[ELF] ERROR: Failed to size ELF file\n");
        fclose(f);
        return 0;
    }
    long file_size_long = ftell(f);
    if (file_size_long < 0) {
        fprintf(stderr, "[ELF] ERROR: Failed to size ELF file\n");
        fclose(f);
        return 0;
    }
    uint64_t file_size = (uint64_t)file_size_long;
    if (fseek(f, 0, SEEK_SET) != 0) {
        fprintf(stderr, "[ELF] ERROR: Failed to rewind ELF file\n");
        fclose(f);
        return 0;
    }

    /* Read ELF header */
    elf32_ehdr_t ehdr;
    if (fread(&ehdr, 1, sizeof(ehdr), f) != sizeof(ehdr)) {
        fprintf(stderr, "[ELF] ERROR: Failed to read ELF header\n");
        fclose(f);
        return 0;
    }

    /* Validate ELF magic */
    if (ehdr.e_ident[EI_MAG0] != ELFMAG0 ||
        ehdr.e_ident[1] != ELFMAG1 ||
        ehdr.e_ident[2] != ELFMAG2 ||
        ehdr.e_ident[3] != ELFMAG3) {
        fprintf(stderr, "[ELF] ERROR: Not a valid ELF file\n");
        fclose(f);
        return 0;
    }

    /* Check 32-bit, little-endian, ARM */
    if (ehdr.e_ident[4] != ELFCLASS32) {
        fprintf(stderr, "[ELF] ERROR: Not a 32-bit ELF (class=%d)\n", ehdr.e_ident[4]);
        fclose(f);
        return 0;
    }
    if (ehdr.e_ident[5] != ELFDATA2LSB) {
        fprintf(stderr, "[ELF] ERROR: Not little-endian\n");
        fclose(f);
        return 0;
    }
    if (ehdr.e_machine != EM_ARM && ehdr.e_machine != EM_RISCV) {
        fprintf(stderr, "[ELF] ERROR: Unsupported architecture (machine=%d, expected ARM=%d or RISC-V=%d)\n",
                ehdr.e_machine, EM_ARM, EM_RISCV);
        fclose(f);
        return 0;
    }

    /* Set detected architecture for auto-detection */
    extern int detected_arch;
    if (ehdr.e_machine == EM_RISCV) {
        detected_arch = FW_ARCH_RV32;
        fprintf(stderr, "[ELF] Valid ELF32 RISC-V binary\n");
    } else {
        detected_arch = FW_ARCH_ARM_M0P;
        fprintf(stderr, "[ELF] Valid ELF32 ARM binary\n");
    }
    fprintf(stderr, "[ELF] Entry point: 0x%08X\n", ehdr.e_entry);
    fprintf(stderr, "[ELF] Program headers: %d (offset 0x%X)\n", ehdr.e_phnum, ehdr.e_phoff);

    if (ehdr.e_phnum == 0 || ehdr.e_phoff == 0) {
        fprintf(stderr, "[ELF] ERROR: No program headers\n");
        fclose(f);
        return 0;
    }
    if (ehdr.e_phentsize < sizeof(elf32_phdr_t)) {
        fprintf(stderr, "[ELF] ERROR: Program headers are too small (%u)\n", ehdr.e_phentsize);
        fclose(f);
        return 0;
    }

    uint64_t phdr_table_size = (uint64_t)ehdr.e_phentsize * (uint64_t)ehdr.e_phnum;
    if ((uint64_t)ehdr.e_phoff > file_size || phdr_table_size > file_size - ehdr.e_phoff) {
        fprintf(stderr, "[ELF] ERROR: Program header table outside file bounds\n");
        fclose(f);
        return 0;
    }

    /* Read and process program headers */
    int segments_loaded = 0;

    for (int i = 0; i < ehdr.e_phnum; i++) {
        elf32_phdr_t phdr;
        uint64_t offset = (uint64_t)ehdr.e_phoff + (uint64_t)i * ehdr.e_phentsize;

        if (!checked_seek(f, offset)) {
            fprintf(stderr, "[ELF] ERROR: Failed to seek to program header %d\n", i);
            continue;
        }

        if (fread(&phdr, 1, sizeof(phdr), f) != sizeof(phdr)) {
            fprintf(stderr, "[ELF] ERROR: Failed to read program header %d\n", i);
            continue;
        }

        /* Only load PT_LOAD segments */
        if (phdr.p_type != PT_LOAD) {
            continue;
        }

        fprintf(stderr, "[ELF] LOAD segment %d: vaddr=0x%08X paddr=0x%08X filesz=%u memsz=%u\n",
               i, phdr.p_vaddr, phdr.p_paddr, phdr.p_filesz, phdr.p_memsz);

        if (phdr.p_filesz > phdr.p_memsz) {
            fprintf(stderr,
                    "[ELF] WARNING: Segment %d has filesz > memsz (%u > %u), skipping\n",
                    i, phdr.p_filesz, phdr.p_memsz);
            continue;
        }
        if ((uint64_t)phdr.p_offset > file_size ||
            (uint64_t)phdr.p_filesz > file_size - phdr.p_offset) {
            fprintf(stderr, "[ELF] WARNING: Segment %d file range outside ELF bounds, skipping\n", i);
            continue;
        }

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
        if (region_contains(FLASH_BASE, FLASH_SIZE, target, phdr.p_memsz)) {
            uint32_t flash_offset = target - FLASH_BASE;

            /* Zero the memory region first (for .bss-like sections where memsz > filesz) */
            if (phdr.p_memsz > 0) {
                memset(&cpu.flash[flash_offset], 0, phdr.p_memsz);
            }

            /* Load file data */
            if (phdr.p_filesz > 0) {
                if (!checked_seek(f, phdr.p_offset)) {
                    fprintf(stderr, "[ELF] ERROR: Failed to seek to segment data\n");
                    continue;
                }

                size_t read = fread(&cpu.flash[flash_offset], 1, phdr.p_filesz, f);
                if (read != phdr.p_filesz) {
                    fprintf(stderr, "[ELF] WARNING: Only read %zu of %u bytes\n", read, phdr.p_filesz);
                }
            }

            fprintf(stderr, "[ELF] Loaded %u bytes to flash[0x%08X]\n", phdr.p_filesz, flash_offset);
            segments_loaded++;
        }
        /* Load into RAM */
        else if (region_contains(RAM_BASE, RAM_SIZE, target, phdr.p_memsz)) {
            uint32_t ram_offset = target - RAM_BASE;

            if (phdr.p_memsz > 0) {
                memset(&cpu.ram[ram_offset], 0, phdr.p_memsz);
            }

            if (phdr.p_filesz > 0) {
                if (!checked_seek(f, phdr.p_offset)) {
                    fprintf(stderr, "[ELF] ERROR: Failed to seek to segment data\n");
                    continue;
                }

                size_t read = fread(&cpu.ram[ram_offset], 1, phdr.p_filesz, f);
                if (read != phdr.p_filesz) {
                    fprintf(stderr, "[ELF] WARNING: Only read %zu of %u bytes\n", read, phdr.p_filesz);
                }
            }

            fprintf(stderr, "[ELF] Loaded %u bytes to RAM[0x%08X]\n", phdr.p_filesz, ram_offset);

            if (region_contains(FLASH_BASE, FLASH_SIZE, phdr.p_paddr, phdr.p_filesz) &&
                phdr.p_paddr != phdr.p_vaddr) {
                uint32_t flash_offset = phdr.p_paddr - FLASH_BASE;

                if (!checked_seek(f, phdr.p_offset)) {
                    fprintf(stderr, "[ELF] ERROR: Failed to seek to RAM LMA data\n");
                    continue;
                }

                size_t read = fread(&cpu.flash[flash_offset], 1, phdr.p_filesz, f);
                if (read != phdr.p_filesz) {
                    fprintf(stderr, "[ELF] WARNING: Only mirrored %zu of %u bytes to flash LMA\n",
                           read, phdr.p_filesz);
                } else {
                    fprintf(stderr, "[ELF] Mirrored %u RAM-init bytes to flash[0x%08X]\n",
                           phdr.p_filesz, flash_offset);
                }
            }

            segments_loaded++;
        }
        else {
            fprintf(stderr, "[ELF] WARNING: Segment target 0x%08X outside flash/RAM bounds, skipping\n", target);
        }
    }

    fclose(f);
    fprintf(stderr, "[ELF] Load complete: %d segments loaded\n", segments_loaded);
    return (segments_loaded > 0);
}
