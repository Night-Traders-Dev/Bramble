/*
 * TMDS encoder for the RP2350 SIO block.
 *
 * The RP2350 SIO contains a DVI-compliant TMDS encoder, documented in the
 * RP2350 datasheet SIO register list at offsets 0x1c0-0x1e4. Bramble previously
 * left the whole range unmapped, so firmware using the DVI output read zeros.
 *
 * Copyright (c) 2026 Br contributors. See LICENSE.
 */
#ifndef BRAMBLE_TMDS_H
#define BRAMBLE_TMDS_H

#include <stdint.h>

/* Register offsets from SIO_BASE. */
#define TMDS_CTRL              0x1C0
#define TMDS_WDATA             0x1C4
#define TMDS_PEEK_SINGLE       0x1C8
#define TMDS_POP_SINGLE        0x1CC
#define TMDS_PEEK_DOUBLE_L0    0x1D0
#define TMDS_POP_DOUBLE_L0     0x1D4
#define TMDS_PEEK_DOUBLE_L1    0x1D8
#define TMDS_POP_DOUBLE_L1     0x1DC
#define TMDS_PEEK_DOUBLE_L2    0x1E0
#define TMDS_POP_DOUBLE_L2     0x1E4

/* TMDS_CTRL bit fields. */
#define TMDS_CTRL_CLEAR_BALANCE_SHIFT  28
#define TMDS_CTRL_PIX2_NOSHIFT         (1u << 27)
#define TMDS_CTRL_PIX_SHIFT_SHIFT      24
#define TMDS_CTRL_PIX_SHIFT_MASK       0x7u
#define TMDS_CTRL_INTERLEAVE           (1u << 23)
#define TMDS_CTRL_NBITS_SHIFT(lane)    (12 + 3 * (lane))
#define TMDS_CTRL_NBITS_MASK           0x7u
#define TMDS_CTRL_ROT_SHIFT(lane)      (4 * (lane))
#define TMDS_CTRL_ROT_MASK             0xFu

/* PIX_SHIFT encodes the shift as an index into this table; 0 means no shift. */
#define TMDS_SHIFT_TABLE      { 0, 1, 2, 4, 8, 16 }

typedef struct {
    uint32_t ctrl;
    uint32_t colour;    /* colour accumulator; only the low 16 bits are used */
    /* Running DC balance (disparity) per encoder layer. PEEK does not shift the
     * colour register but still advances these; POP does both. */
    int32_t balance[2][3];
} tmds_state_t;

void tmds_init(tmds_state_t *st);

/* Returns 1 if the offset was a TMDS register and was handled. */
int tmds_read(tmds_state_t *st, uint32_t offset, uint32_t *out);
int tmds_write(tmds_state_t *st, uint32_t offset, uint32_t val);

/* Exposed for tests: encode one pixel from the colour accumulator.
 * layer 0 is the first pixel, layer 1 the second.
 * Writes the three 10-bit lane symbols into sym[0..2] for the given layer. */
void tmds_encode_pixel(tmds_state_t *st, unsigned layer,
                       uint16_t sym[3]);

#endif /* BRAMBLE_TMDS_H */