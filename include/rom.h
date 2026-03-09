#ifndef ROM_H
#define ROM_H

#include <stdint.h>

/* ROM is mapped at 0x00000000, 16KB (matches real RP2040 bootrom size) */
#define ROM_BASE    0x00000000
#define ROM_SIZE    0x4000

/* ROM layout offsets */
#define ROM_MAGIC_OFFSET    0x10    /* 'M', 'u', version */
#define ROM_FUNC_TABLE_PTR  0x14    /* 16-bit pointer to function table */
#define ROM_DATA_TABLE_PTR  0x16    /* 16-bit pointer to data table */
#define ROM_LOOKUP_FN_PTR   0x18    /* 16-bit pointer to lookup function */

/* ROM table code macro (matches SDK) */
#define ROM_TABLE_CODE(c1, c2) ((c1) | ((c2) << 8))

/* Known ROM function codes */
#define ROM_FUNC_MEMCPY     ROM_TABLE_CODE('M', 'C')
#define ROM_FUNC_MEMCPY44   ROM_TABLE_CODE('C', '4')
#define ROM_FUNC_MEMSET     ROM_TABLE_CODE('M', 'S')
#define ROM_FUNC_MEMSET4    ROM_TABLE_CODE('S', '4')
#define ROM_FUNC_POPCOUNT32 ROM_TABLE_CODE('P', '3')
#define ROM_FUNC_CLZ32      ROM_TABLE_CODE('L', '3')
#define ROM_FUNC_CTZ32      ROM_TABLE_CODE('T', '3')
#define ROM_FUNC_REVERSE32  ROM_TABLE_CODE('R', '3')

/* Flash ROM function codes */
#define ROM_FUNC_CONNECT_INTERNAL_FLASH ROM_TABLE_CODE('I', 'F')
#define ROM_FUNC_FLASH_EXIT_XIP         ROM_TABLE_CODE('E', 'X')
#define ROM_FUNC_FLASH_RANGE_ERASE      ROM_TABLE_CODE('R', 'E')
#define ROM_FUNC_FLASH_RANGE_PROGRAM    ROM_TABLE_CODE('R', 'P')
#define ROM_FUNC_FLASH_FLUSH_CACHE      ROM_TABLE_CODE('F', 'C')
#define ROM_FUNC_FLASH_ENTER_CMD_XIP    ROM_TABLE_CODE('C', 'X')

/* ROM data table codes */
#define ROM_DATA_SOFT_FLOAT  ROM_TABLE_CODE('S', 'F')
#define ROM_DATA_SOFT_DOUBLE ROM_TABLE_CODE('S', 'D')

/* ROM float/double function stub address ranges.
 * Each stub is 2 bytes (BX LR). The index within the range identifies
 * the operation (fadd=0, fsub=1, fmul=2, ...).
 * cpu_step intercepts execution at these addresses. */
#define ROM_FLOAT_FUNC_BASE   0x0500
#define ROM_FLOAT_FUNC_COUNT  20
#define ROM_DOUBLE_FUNC_BASE  0x0540
#define ROM_DOUBLE_FUNC_COUNT 20

/* Float/double function indices (matches RP2040 ROM layout) */
#define ROM_FLOAT_FADD        0
#define ROM_FLOAT_FSUB        1
#define ROM_FLOAT_FMUL        2
#define ROM_FLOAT_FDIV        3
#define ROM_FLOAT_DEPRECATED  4
#define ROM_FLOAT_FSQRT       5
#define ROM_FLOAT_FLOAT2INT   6
#define ROM_FLOAT_FLOAT2FIX   7
#define ROM_FLOAT_FLOAT2UINT  8
#define ROM_FLOAT_FLOAT2UFIX  9
#define ROM_FLOAT_INT2FLOAT   10
#define ROM_FLOAT_FIX2FLOAT   11
#define ROM_FLOAT_UINT2FLOAT  12
#define ROM_FLOAT_UFIX2FLOAT  13
#define ROM_FLOAT_FCOS        14
#define ROM_FLOAT_FSIN        15
#define ROM_FLOAT_FTAN        16
#define ROM_FLOAT_UNUSED17    17
#define ROM_FLOAT_FEXP        18
#define ROM_FLOAT_FLN         19

/* ROM flash function stub addresses */
#define ROM_FLASH_RANGE_ERASE_ADDR   0x03B8
#define ROM_FLASH_RANGE_PROGRAM_ADDR 0x03BC

/* ROM image buffer */
extern uint8_t rom_image[ROM_SIZE];

/* Initialize ROM image with function table and Thumb code stubs */
void rom_init(void);

/* Read from ROM (used by membus) */
uint32_t rom_read32(uint32_t addr);
uint16_t rom_read16(uint32_t addr);
uint8_t  rom_read8(uint32_t addr);

/* ROM function interception: returns 1 if PC was intercepted */
int rom_intercept(uint32_t pc);

#endif /* ROM_H */
