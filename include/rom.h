#ifndef ROM_H
#define ROM_H

#include <stdint.h>

/* ROM is mapped at 0x00000000, 4KB */
#define ROM_BASE    0x00000000
#define ROM_SIZE    0x1000

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

/* ROM image buffer */
extern uint8_t rom_image[ROM_SIZE];

/* Initialize ROM image with function table and Thumb code stubs */
void rom_init(void);

/* Read from ROM (used by membus) */
uint32_t rom_read32(uint32_t addr);
uint16_t rom_read16(uint32_t addr);
uint8_t  rom_read8(uint32_t addr);

#endif /* ROM_H */
