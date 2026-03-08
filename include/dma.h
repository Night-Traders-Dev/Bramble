/*
 * RP2040 DMA Controller Emulation
 *
 * 12 independent DMA channels with:
 * - READ_ADDR, WRITE_ADDR, TRANS_COUNT, CTRL_TRIG per channel
 * - 4 alias register blocks per channel (AL1-AL3 reorder fields)
 * - Immediate (synchronous) transfer on trigger
 * - INCR_READ / INCR_WRITE address increment
 * - DATA_SIZE: byte / halfword / word
 * - CHAIN_TO for channel chaining
 * - Global interrupt registers (INTR, INTE0/1, INTF0/1, INTS0/1)
 * - Atomic register aliases (SET/CLR/XOR)
 */

#ifndef DMA_H
#define DMA_H

#include <stdint.h>

/* ========================================================================
 * DMA Base Address and Sizing
 * ======================================================================== */

#define DMA_BASE            0x50000000
#define DMA_NUM_CHANNELS    12
#define DMA_CH_STRIDE       0x40    /* 64 bytes per channel */
#define DMA_BLOCK_SIZE      0x4C0   /* channels (0x300) + global regs */

/* ========================================================================
 * Per-Channel Register Offsets (within 0x40-byte channel block)
 * ======================================================================== */

/* Default layout (alias 0) */
#define DMA_CH_READ_ADDR        0x00
#define DMA_CH_WRITE_ADDR       0x04
#define DMA_CH_TRANS_COUNT      0x08
#define DMA_CH_CTRL_TRIG        0x0C

/* Alias 1 */
#define DMA_CH_AL1_CTRL         0x10
#define DMA_CH_AL1_READ_ADDR    0x14
#define DMA_CH_AL1_WRITE_ADDR   0x18
#define DMA_CH_AL1_TRANS_COUNT_TRIG 0x1C

/* Alias 2 */
#define DMA_CH_AL2_CTRL         0x20
#define DMA_CH_AL2_TRANS_COUNT  0x24
#define DMA_CH_AL2_READ_ADDR    0x28
#define DMA_CH_AL2_WRITE_ADDR_TRIG  0x2C

/* Alias 3 */
#define DMA_CH_AL3_CTRL         0x30
#define DMA_CH_AL3_WRITE_ADDR   0x34
#define DMA_CH_AL3_TRANS_COUNT  0x38
#define DMA_CH_AL3_READ_ADDR_TRIG   0x3C

/* ========================================================================
 * CTRL_TRIG Bitfields
 * ======================================================================== */

#define DMA_CTRL_EN             (1 << 0)
#define DMA_CTRL_HIGH_PRIORITY  (1 << 1)
#define DMA_CTRL_DATA_SIZE_MASK (0x3 << 2)
#define DMA_CTRL_DATA_SIZE_SHIFT 2
#define DMA_CTRL_INCR_READ      (1 << 4)
#define DMA_CTRL_INCR_WRITE     (1 << 5)
#define DMA_CTRL_RING_SIZE_MASK (0xF << 6)
#define DMA_CTRL_RING_SIZE_SHIFT 6
#define DMA_CTRL_RING_SEL       (1 << 10)
#define DMA_CTRL_CHAIN_TO_MASK  (0xF << 11)
#define DMA_CTRL_CHAIN_TO_SHIFT 11
#define DMA_CTRL_TREQ_SEL_MASK  (0x3F << 15)
#define DMA_CTRL_TREQ_SEL_SHIFT 15
#define DMA_CTRL_IRQ_QUIET      (1 << 21)
#define DMA_CTRL_BSWAP          (1 << 22)
#define DMA_CTRL_SNIFF_EN       (1 << 23)
#define DMA_CTRL_BUSY           (1 << 24)   /* read-only */
#define DMA_CTRL_WRITE_ERROR    (1 << 29)   /* W1C */
#define DMA_CTRL_READ_ERROR     (1 << 30)   /* W1C */
#define DMA_CTRL_AHB_ERROR      (1 << 31)   /* read-only */

/* DATA_SIZE values */
#define DMA_SIZE_BYTE           0
#define DMA_SIZE_HALFWORD       1
#define DMA_SIZE_WORD           2

/* ========================================================================
 * Global Register Offsets (from DMA_BASE)
 * ======================================================================== */

#define DMA_INTR                0x400
#define DMA_INTE0               0x404
#define DMA_INTF0               0x408
#define DMA_INTS0               0x40C
/* 0x410 reserved */
#define DMA_INTE1               0x414
#define DMA_INTF1               0x418
#define DMA_INTS1               0x41C
#define DMA_TIMER0              0x420
#define DMA_TIMER1              0x424
#define DMA_TIMER2              0x428
#define DMA_TIMER3              0x42C
#define DMA_MULTI_CHAN_TRIGGER  0x430
#define DMA_SNIFF_CTRL          0x434
#define DMA_SNIFF_DATA          0x438
#define DMA_FIFO_LEVELS         0x440
#define DMA_CHAN_ABORT           0x444
#define DMA_N_CHANNELS          0x448

/* ========================================================================
 * Per-Channel State
 * ======================================================================== */

typedef struct {
    uint32_t read_addr;
    uint32_t write_addr;
    uint32_t trans_count;
    uint32_t ctrl;          /* CTRL_TRIG minus the read-only bits */
} dma_channel_t;

/* ========================================================================
 * Global DMA State
 * ======================================================================== */

typedef struct {
    dma_channel_t ch[DMA_NUM_CHANNELS];

    /* Interrupt registers */
    uint32_t intr;          /* Raw interrupt status (W1C) */
    uint32_t inte0;         /* Interrupt enable for IRQ 0 */
    uint32_t intf0;         /* Interrupt force for IRQ 0 */
    uint32_t inte1;         /* Interrupt enable for IRQ 1 */
    uint32_t intf1;         /* Interrupt force for IRQ 1 */

    /* Misc */
    uint32_t timer[4];      /* Pacing timers */
    uint32_t sniff_ctrl;
    uint32_t sniff_data;
} dma_state_t;

extern dma_state_t dma_state;

/* ========================================================================
 * API
 * ======================================================================== */

void     dma_init(void);
int      dma_match(uint32_t addr);   /* Returns 1 if addr is in DMA range */
uint32_t dma_read32(uint32_t offset);
void     dma_write32(uint32_t offset, uint32_t val);

#endif /* DMA_H */
