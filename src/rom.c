#include <string.h>
#include <stdio.h>
#include "rom.h"

/* ROM image buffer */
uint8_t rom_image[ROM_SIZE];

/*
 * ROM Layout:
 *   0x0010: Magic ('M', 'u', 0x01)
 *   0x0014: 16-bit pointer to function table (0x0100)
 *   0x0016: 16-bit pointer to data table (0x0180)
 *   0x0018: 16-bit pointer to lookup function (0x0201, Thumb bit set)
 *   0x0100: Function table entries [16-bit code, 16-bit func_ptr] ...
 *   0x0180: Data table (empty, end marker)
 *   0x0200: Lookup function (Thumb code)
 *   0x0300: memcpy (Thumb code)
 *   0x0320: memset (Thumb code)
 *   0x0340: popcount32 (Thumb code)
 *   0x0360: clz32 (Thumb code)
 *   0x0380: ctz32 (Thumb code)
 *   0x03A0: reverse32 stub (returns 0)
 */

/* Helper: write a 16-bit value at a ROM offset (little-endian) */
static void rom_write16(uint32_t offset, uint16_t val) {
    rom_image[offset]     = val & 0xFF;
    rom_image[offset + 1] = (val >> 8) & 0xFF;
}

/* Place Thumb code for the table lookup function at ROM offset 0x0200.
 *
 * rom_table_lookup(uint16_t *table, uint32_t code):
 *   r0 = table pointer, r1 = code to find
 *   Returns function pointer in r0 (or 0 if not found)
 *
 * 0x0200: ldrh r2, [r0, #0]     ; Load entry code
 * 0x0202: cmp  r2, #0           ; End of table?
 * 0x0204: beq  not_found        ; -> 0x0212
 * 0x0206: cmp  r2, r1           ; Match?
 * 0x0208: beq  found            ; -> 0x020E
 * 0x020A: adds r0, #4           ; Next entry
 * 0x020C: b    loop             ; -> 0x0200
 * found:
 * 0x020E: ldrh r0, [r0, #2]     ; Load function pointer
 * 0x0210: bx   lr
 * not_found:
 * 0x0212: movs r0, #0
 * 0x0214: bx   lr
 */
static void rom_place_lookup_fn(void) {
    rom_write16(0x0200, 0x8802);  /* ldrh r2, [r0, #0] */
    rom_write16(0x0202, 0x2A00);  /* cmp r2, #0 */
    rom_write16(0x0204, 0xD005);  /* beq +5 -> 0x0212 */
    rom_write16(0x0206, 0x428A);  /* cmp r2, r1 */
    rom_write16(0x0208, 0xD001);  /* beq +1 -> 0x020E */
    rom_write16(0x020A, 0x3004);  /* adds r0, #4 */
    rom_write16(0x020C, 0xE7F8);  /* b -8 -> 0x0200 */
    rom_write16(0x020E, 0x8840);  /* ldrh r0, [r0, #2] */
    rom_write16(0x0210, 0x4770);  /* bx lr */
    rom_write16(0x0212, 0x2000);  /* movs r0, #0 */
    rom_write16(0x0214, 0x4770);  /* bx lr */
}

/* memcpy at 0x0300: r0=dst, r1=src, r2=count (bytes). Returns r0=dst.
 *
 * 0x0300: push {r4, lr}
 * 0x0302: movs r3, r0           ; save dst for return
 * 0x0304: cmp  r2, #0
 * 0x0306: ble  done             ; -> 0x0314
 * loop:
 * 0x0308: ldrb r4, [r1, #0]
 * 0x030A: strb r4, [r3, #0]
 * 0x030C: adds r3, #1
 * 0x030E: adds r1, #1
 * 0x0310: subs r2, #1
 * 0x0312: bgt  loop             ; -> 0x0308
 * done:
 * 0x0314: pop  {r4, pc}
 */
static void rom_place_memcpy(void) {
    rom_write16(0x0300, 0xB510);  /* push {r4, lr} */
    rom_write16(0x0302, 0x0003);  /* lsls r3, r0, #0 (movs r3, r0) */
    rom_write16(0x0304, 0x2A00);  /* cmp r2, #0 */
    rom_write16(0x0306, 0xDD05);  /* ble +5 -> 0x0314 */
    rom_write16(0x0308, 0x780C);  /* ldrb r4, [r1, #0] */
    rom_write16(0x030A, 0x701C);  /* strb r4, [r3, #0] */
    rom_write16(0x030C, 0x3301);  /* adds r3, #1 */
    rom_write16(0x030E, 0x3101);  /* adds r1, #1 */
    rom_write16(0x0310, 0x3A01);  /* subs r2, #1 */
    rom_write16(0x0312, 0xDCF9);  /* bgt -7 -> 0x0308 */
    rom_write16(0x0314, 0xBD10);  /* pop {r4, pc} */
}

/* memset at 0x0320: r0=dst, r1=value, r2=count (bytes). Returns r0=dst.
 *
 * 0x0320: movs r3, r0           ; save dst
 * 0x0322: cmp  r2, #0
 * 0x0324: ble  done             ; -> 0x032E
 * loop:
 * 0x0326: strb r1, [r3, #0]
 * 0x0328: adds r3, #1
 * 0x032A: subs r2, #1
 * 0x032C: bgt  loop             ; -> 0x0326
 * done:
 * 0x032E: bx   lr
 */
static void rom_place_memset(void) {
    rom_write16(0x0320, 0x0003);  /* lsls r3, r0, #0 (movs r3, r0) */
    rom_write16(0x0322, 0x2A00);  /* cmp r2, #0 */
    rom_write16(0x0324, 0xDD03);  /* ble +3 -> 0x032E */
    rom_write16(0x0326, 0x7019);  /* strb r1, [r3, #0] */
    rom_write16(0x0328, 0x3301);  /* adds r3, #1 */
    rom_write16(0x032A, 0x3A01);  /* subs r2, #1 */
    rom_write16(0x032C, 0xDCFB);  /* bgt -5 -> 0x0326 */
    rom_write16(0x032E, 0x4770);  /* bx lr */
}

/* popcount32 at 0x0340: r0=value, returns bit count in r0.
 * Uses Kernighan's algorithm: n &= (n-1) clears lowest set bit.
 *
 * 0x0340: movs r1, #0           ; count = 0
 * 0x0342: cmp  r0, #0
 * 0x0344: beq  done             ; -> 0x0350
 * loop:
 * 0x0346: movs r2, r0           ; r2 = n
 * 0x0348: subs r2, #1           ; r2 = n-1
 * 0x034A: ands r0, r2           ; n &= (n-1)
 * 0x034C: adds r1, #1           ; count++
 * 0x034E: b    check            ; -> 0x0342
 * done:
 * 0x0350: movs r0, r1           ; return count
 * 0x0352: bx   lr
 */
static void rom_place_popcount(void) {
    rom_write16(0x0340, 0x2100);  /* movs r1, #0 */
    rom_write16(0x0342, 0x2800);  /* cmp r0, #0 */
    rom_write16(0x0344, 0xD004);  /* beq +4 -> 0x0350 */
    rom_write16(0x0346, 0x0002);  /* lsls r2, r0, #0 (movs r2, r0) */
    rom_write16(0x0348, 0x3A01);  /* subs r2, #1 */
    rom_write16(0x034A, 0x4010);  /* ands r0, r2 */
    rom_write16(0x034C, 0x3101);  /* adds r1, #1 */
    rom_write16(0x034E, 0xE7F8);  /* b -8 -> 0x0342 */
    rom_write16(0x0350, 0x0008);  /* lsls r0, r1, #0 (movs r0, r1) */
    rom_write16(0x0352, 0x4770);  /* bx lr */
}

/* clz32 at 0x0360: r0=value, returns leading zero count in r0.
 *
 * 0x0360: movs r1, #0           ; count = 0
 * 0x0362: cmp  r0, #0
 * 0x0364: beq  ret32            ; -> 0x0372
 * loop:
 * 0x0366: lsls r0, r0, #1      ; shift left
 * 0x0368: bcs  done             ; -> 0x036E (carry = found MSB)
 * 0x036A: adds r1, #1           ; count++
 * 0x036C: b    loop             ; -> 0x0366
 * done:
 * 0x036E: movs r0, r1
 * 0x0370: bx   lr
 * ret32:
 * 0x0372: movs r0, #32
 * 0x0374: bx   lr
 */
static void rom_place_clz(void) {
    rom_write16(0x0360, 0x2100);  /* movs r1, #0 */
    rom_write16(0x0362, 0x2800);  /* cmp r0, #0 */
    rom_write16(0x0364, 0xD005);  /* beq +5 -> 0x0372 */
    rom_write16(0x0366, 0x0040);  /* lsls r0, r0, #1 */
    rom_write16(0x0368, 0xD201);  /* bcs +1 -> 0x036E */
    rom_write16(0x036A, 0x3101);  /* adds r1, #1 */
    rom_write16(0x036C, 0xE7FB);  /* b -5 -> 0x0366 */
    rom_write16(0x036E, 0x0008);  /* lsls r0, r1, #0 (movs r0, r1) */
    rom_write16(0x0370, 0x4770);  /* bx lr */
    rom_write16(0x0372, 0x2020);  /* movs r0, #32 */
    rom_write16(0x0374, 0x4770);  /* bx lr */
}

/* ctz32 at 0x0380: r0=value, returns trailing zero count in r0.
 *
 * 0x0380: movs r1, #0           ; count = 0
 * 0x0382: cmp  r0, #0
 * 0x0384: beq  ret32            ; -> 0x0396
 * loop:
 * 0x0386: movs r2, #1
 * 0x0388: tst  r0, r2           ; test bit 0
 * 0x038A: bne  done             ; -> 0x0392
 * 0x038C: lsrs r0, r0, #1
 * 0x038E: adds r1, #1
 * 0x0390: b    loop             ; -> 0x0386
 * done:
 * 0x0392: movs r0, r1
 * 0x0394: bx   lr
 * ret32:
 * 0x0396: movs r0, #32
 * 0x0398: bx   lr
 */
static void rom_place_ctz(void) {
    rom_write16(0x0380, 0x2100);  /* movs r1, #0 */
    rom_write16(0x0382, 0x2800);  /* cmp r0, #0 */
    rom_write16(0x0384, 0xD007);  /* beq +7 -> 0x0396 */
    rom_write16(0x0386, 0x2201);  /* movs r2, #1 */
    rom_write16(0x0388, 0x4210);  /* tst r0, r2 */
    rom_write16(0x038A, 0xD102);  /* bne +2 -> 0x0392 */
    rom_write16(0x038C, 0x0840);  /* lsrs r0, r0, #1 */
    rom_write16(0x038E, 0x3101);  /* adds r1, #1 */
    rom_write16(0x0390, 0xE7F9);  /* b -7 -> 0x0386 */
    rom_write16(0x0392, 0x0008);  /* lsls r0, r1, #0 (movs r0, r1) */
    rom_write16(0x0394, 0x4770);  /* bx lr */
    rom_write16(0x0396, 0x2020);  /* movs r0, #32 */
    rom_write16(0x0398, 0x4770);  /* bx lr */
}

/* reverse32 stub at 0x03A0: returns 0 (not commonly needed) */
static void rom_place_reverse_stub(void) {
    rom_write16(0x03A0, 0x2000);  /* movs r0, #0 */
    rom_write16(0x03A2, 0x4770);  /* bx lr */
}

/* Flash function stubs: all just 'bx lr' (no-ops).
 * 0x03B0: connect_internal_flash
 * 0x03B4: flash_exit_xip
 * 0x03B8: flash_range_erase
 * 0x03BC: flash_range_program
 * 0x03C0: flash_flush_cache
 * 0x03C4: flash_enter_cmd_xip */
static void rom_place_flash_stubs(void) {
    for (uint32_t addr = 0x03B0; addr <= 0x03C4; addr += 4) {
        rom_write16(addr, 0x4770);  /* bx lr */
    }
}

/* Build the function table at 0x0100.
 * Each entry: [16-bit code][16-bit func_ptr with Thumb bit] */
static void rom_build_func_table(void) {
    uint32_t off = 0x0100;

    /* memcpy (MC) -> 0x0300 */
    rom_write16(off, ROM_FUNC_MEMCPY);    rom_write16(off + 2, 0x0301); off += 4;
    /* memcpy44 (C4) -> 0x0300 (same impl) */
    rom_write16(off, ROM_FUNC_MEMCPY44);  rom_write16(off + 2, 0x0301); off += 4;
    /* memset (MS) -> 0x0320 */
    rom_write16(off, ROM_FUNC_MEMSET);    rom_write16(off + 2, 0x0321); off += 4;
    /* memset4 (S4) -> 0x0320 (same impl) */
    rom_write16(off, ROM_FUNC_MEMSET4);   rom_write16(off + 2, 0x0321); off += 4;
    /* popcount32 (P3) -> 0x0340 */
    rom_write16(off, ROM_FUNC_POPCOUNT32); rom_write16(off + 2, 0x0341); off += 4;
    /* clz32 (L3) -> 0x0360 */
    rom_write16(off, ROM_FUNC_CLZ32);     rom_write16(off + 2, 0x0361); off += 4;
    /* ctz32 (T3) -> 0x0380 */
    rom_write16(off, ROM_FUNC_CTZ32);     rom_write16(off + 2, 0x0381); off += 4;
    /* reverse32 (R3) -> 0x03A0 */
    rom_write16(off, ROM_FUNC_REVERSE32); rom_write16(off + 2, 0x03A1); off += 4;
    /* Flash functions (no-op stubs) */
    rom_write16(off, ROM_FUNC_CONNECT_INTERNAL_FLASH); rom_write16(off + 2, 0x03B1); off += 4;
    rom_write16(off, ROM_FUNC_FLASH_EXIT_XIP);         rom_write16(off + 2, 0x03B5); off += 4;
    rom_write16(off, ROM_FUNC_FLASH_RANGE_ERASE);      rom_write16(off + 2, 0x03B9); off += 4;
    rom_write16(off, ROM_FUNC_FLASH_RANGE_PROGRAM);    rom_write16(off + 2, 0x03BD); off += 4;
    rom_write16(off, ROM_FUNC_FLASH_FLUSH_CACHE);      rom_write16(off + 2, 0x03C1); off += 4;
    rom_write16(off, ROM_FUNC_FLASH_ENTER_CMD_XIP);    rom_write16(off + 2, 0x03C5); off += 4;
    /* End marker */
    rom_write16(off, 0x0000);             rom_write16(off + 2, 0x0000);
}

/* Build data table at 0x0180 (empty for now) */
static void rom_build_data_table(void) {
    rom_write16(0x0180, 0x0000);  /* end marker */
    rom_write16(0x0182, 0x0000);
}

/* Initialize ROM image */
void rom_init(void) {
    memset(rom_image, 0, ROM_SIZE);

    /* Magic at offset 0x10: 'M', 'u', version=1 */
    rom_image[0x10] = 'M';
    rom_image[0x11] = 'u';
    rom_image[0x12] = 0x01;

    /* Pointers at 0x14/0x16/0x18 */
    rom_write16(ROM_FUNC_TABLE_PTR, 0x0100);   /* func_table at 0x0100 */
    rom_write16(ROM_DATA_TABLE_PTR, 0x0180);   /* data_table at 0x0180 */
    rom_write16(ROM_LOOKUP_FN_PTR,  0x0201);   /* lookup_fn at 0x0200 | Thumb bit */

    /* Build function table */
    rom_build_func_table();

    /* Build data table */
    rom_build_data_table();

    /* Place Thumb code stubs */
    rom_place_lookup_fn();
    rom_place_memcpy();
    rom_place_memset();
    rom_place_popcount();
    rom_place_clz();
    rom_place_ctz();
    rom_place_reverse_stub();
    rom_place_flash_stubs();

    printf("[ROM] Initialized function table with 14 entries\n");
}

/* Read from ROM */
uint32_t rom_read32(uint32_t addr) {
    uint32_t off = addr & (ROM_SIZE - 1);
    if (off + 3 < ROM_SIZE) {
        return rom_image[off] |
               ((uint32_t)rom_image[off + 1] << 8) |
               ((uint32_t)rom_image[off + 2] << 16) |
               ((uint32_t)rom_image[off + 3] << 24);
    }
    return 0;
}

uint16_t rom_read16(uint32_t addr) {
    uint32_t off = addr & (ROM_SIZE - 1);
    if (off + 1 < ROM_SIZE) {
        return rom_image[off] | ((uint16_t)rom_image[off + 1] << 8);
    }
    return 0;
}

uint8_t rom_read8(uint32_t addr) {
    uint32_t off = addr & (ROM_SIZE - 1);
    if (off < ROM_SIZE) {
        return rom_image[off];
    }
    return 0;
}
