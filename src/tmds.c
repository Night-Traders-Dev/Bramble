/*
 * TMDS encoder for the RP2350 SIO block.
 *
 * Copyright (c) 2026 Br contributors. See LICENSE.
 */
#include "tmds.h"

/* ------------------------------------------------------------------ *
 * 8b/10b TMDS symbol generation
 *
 * 0x00 and 0xFF select a control symbol and leave the running disparity
 * untouched. Any other byte goes through the standard intermediate (q_m)
 * coding and a disparity-driven final inversion.
 * ------------------------------------------------------------------ */

/* Control symbols for 0x00 and 0xFF, indexed [lane][is_0xFF]. These are fixed
 * by the DVI specification and are not affected by the running disparity:
 *
 *   C0 = 1101010100 = 0x354
 *   C1 = 0010101011 = 0x0AB
 *   C2 = 0101010100 = 0x0A4
 *
 * The RP2350 datasheet documents the register interface but not these symbol
 * patterns; they come from the DVI standard. Note C1 is 0x0AB, not 0x2AB --
 * 0x2AB is 1010101011, which is not a valid TMDS symbol. */
static const uint16_t tmds_control[3][2] = {
    { 0x354, 0x0AB },  /* lane 0 (blue):  C0 = 1101010100, C1 = 0010101011 */
    { 0x0AB, 0x354 },  /* lane 1 (green): C0 = 0010101011, C1 = 1101010100 */
    { 0x0A4, 0x0A4 },  /* lane 2 (red):   C2 = 0101010100 for both */
};

/* Encode one data byte, carrying the running disparity in *disp.
 *
 * Intermediate 9-bit code q_m, from the DVI specification:
 *   q_m[0..3] are the input bits each XORed with the bit below it, and
 *   q_m[4..7] additionally account for a possible zero-run or one-run, with
 *   q_m[8] supplying the parity of the whole symbol.
 *
 * The 10th bit is then chosen so that the running disparity is driven toward
 * zero: if the intermediate symbol is already positive-disparity, invert it and
 * emit a leading 1, otherwise emit a leading 0.
 */
static uint16_t tmds_encode_byte(uint8_t data, int32_t *disp) {
    uint32_t qm = 0;
    unsigned ones, zeros;
    int disparity;
    int invert;

    qm |= (data >> 0) & 1u;
    qm |= (((data >> 1) & 1u) ^ ((data >> 0) & 1u)) << 1;
    qm |= (((data >> 2) & 1u) ^ ((data >> 1) & 1u)) << 2;
    qm |= (((data >> 3) & 1u) ^ ((data >> 2) & 1u)) << 3;
    qm |= (((data >> 4) & 1u) ^ ((data >> 3) & 1u) ^
           ((((data >> 4) & 1u) ? 1u : 0u))) << 4;
    qm |= (((data >> 5) & 1u) ^ ((data >> 4) & 1u) ^
           ((((data >> 4) & 1u) ? 0u : 1u))) << 5;
    qm |= (((data >> 6) & 1u) ^ ((data >> 5) & 1u) ^
           ((((data >> 6) & 1u) ? 1u : 0u))) << 6;
    qm |= (((data >> 7) & 1u) ^ ((data >> 6) & 1u) ^
           ((((data >> 7) & 1u) ? 0u : 1u))) << 7;
    qm |= ((data >> 7) & 1u) << 8;

    ones = 0;
    for (unsigned i = 0; i < 8u; i++)
        ones += (qm >> i) & 1u;
    zeros = 8u - ones;

    disparity = (int)ones - (int)zeros;

    /* Drive the disparity toward zero: when the incoming balance is positive
     * prefer a net-negative symbol and vice versa. On an exact tie, use the
     * parity of the intermediate code to break it consistently. */
    if (*disp > 0)
        invert = (disparity > 0);
    else if (*disp < 0)
        invert = (disparity < 0);
    else
        invert = (disparity > 0);

    if (invert) {
        uint32_t inv = (~qm) & 0x1FFu;
        *disp += (int)(ones - zeros) * -1;
        return (uint16_t)(0x200u | inv);
    }
    *disp += (int)ones - (int)zeros;
    return (uint16_t)qm;
}

/* ------------------------------------------------------------------ *
 * Colour extraction
 * ------------------------------------------------------------------ */

/* Extract the 8-bit encoder input for one lane from the colour accumulator.
 *
 * The 16 LSBs of the accumulator are right-rotated by the lane's ROT field to
 * bring that lane's colour into the top of the byte, then the low NBITS-1 bits
 * are masked to zero so narrower input formats pad with zeros. */
static uint8_t tmds_lane_byte(const tmds_state_t *st, unsigned lane) {
    unsigned rot   = (st->ctrl >> TMDS_CTRL_ROT_SHIFT(lane)) & TMDS_CTRL_ROT_MASK;
    unsigned nbits = ((st->ctrl >> TMDS_CTRL_NBITS_SHIFT(lane)) &
                      TMDS_CTRL_NBITS_MASK) + 1u;
    uint16_t v = (uint16_t)(st->colour & 0xFFFFu);

    if (rot)
        v = (uint16_t)((v >> rot) | (uint16_t)(v << (16 - rot)));

    /* Keep the top `nbits` bits and zero the rest of the byte. Masking with
     * 0xFF00 << (8 - nbits) instead is wrong: for nbits == 1 that shift is 7,
     * which pushes the mask out to 0x8000 and leaves the low seven bits of the
     * byte unconstrained rather than zeroed. */
    v &= (uint16_t)(0xFFFFu << (16u - nbits));
    return (uint8_t)(v >> 8);
}

void tmds_encode_pixel(tmds_state_t *st, unsigned layer, uint16_t sym[3]) {
    for (unsigned lane = 0; lane < 3u; lane++) {
        uint8_t v = tmds_lane_byte(st, lane);
        if (v == 0x00 || v == 0xFF)
            sym[lane] = tmds_control[lane][v == 0xFF ? 1 : 0];
        else
            sym[lane] = tmds_encode_byte(v, &st->balance[layer][lane]);
    }
}

/* ------------------------------------------------------------------ *
 * Register interface
 * ------------------------------------------------------------------ */

static const uint8_t tmds_shift_table[6] = TMDS_SHIFT_TABLE;

static unsigned tmds_shift_amount(uint32_t ctrl, int is_double) {
    unsigned idx = (ctrl >> TMDS_CTRL_PIX_SHIFT_SHIFT) & TMDS_CTRL_PIX_SHIFT_MASK;
    unsigned amt;

    if (idx >= 6u)
        return 0;
    amt = tmds_shift_table[idx];
    /* POP_DOUBLE with PIX2_NOSHIFT clear shifts twice as far. */
    if (is_double && !(ctrl & TMDS_CTRL_PIX2_NOSHIFT))
        amt *= 2u;
    return amt;
}

static void tmds_shift_colour(tmds_state_t *st, uint32_t ctrl, int is_double) {
    unsigned amt = tmds_shift_amount(ctrl, is_double);
    if (amt == 0 || amt >= 32u)
        return;                       /* a shift of 32 means no shift */
    st->colour = (st->colour << amt) & 0xFFFFFFFFu;
}

static uint32_t tmds_pack(const uint16_t sym[3], int interleave) {
    if (!interleave)
        return (uint32_t)sym[0] | ((uint32_t)sym[1] << 10) |
               ((uint32_t)sym[2] << 20);

    {
        uint32_t out = 0;
        for (unsigned bit = 0; bit < 10u; bit++) {
            for (unsigned lane = 0; lane < 3u; lane++) {
                unsigned chunk = bit / 2u, slot = bit % 2u;
                unsigned pos = chunk * 6u + lane * 2u + slot;
                out |= (uint32_t)((sym[lane] >> bit) & 1u) << pos;
            }
        }
        return out & 0x3FFFFFFFu;
    }
}

void tmds_init(tmds_state_t *st) {
    st->ctrl = 0;
    st->colour = 0;
    for (unsigned l = 0; l < 2u; l++)
        for (unsigned lane = 0; lane < 3u; lane++)
            st->balance[l][lane] = 0;
}

/* Encode one layer into sym[], advancing that layer's DC balance.
 *
 * With PIX2_NOSHIFT clear the second layer sees the colour register already
 * shifted once, which is how two pixels advance together. The shift is applied
 * to a scratch copy so the caller's register state is only changed by an
 * explicit POP. */
static void tmds_layer_symbols(tmds_state_t *st, unsigned layer,
                               uint16_t sym[3]) {
    if (layer == 1u && !(st->ctrl & TMDS_CTRL_PIX2_NOSHIFT)) {
        unsigned amt = tmds_shift_amount(st->ctrl, 0);
        if (amt > 0 && amt < 32u) {
            uint32_t saved = st->colour;
            st->colour = (st->colour << amt) & 0xFFFFFFFFu;
            tmds_encode_pixel(st, layer, sym);
            st->colour = saved;
            return;
        }
    }
    tmds_encode_pixel(st, layer, sym);
}

int tmds_read(tmds_state_t *st, uint32_t offset, uint32_t *out) {
    int interleave = (st->ctrl & TMDS_CTRL_INTERLEAVE) != 0;
    uint16_t s0[3], s1[3];

    switch (offset) {
    case TMDS_CTRL:
        /* CLEAR_BALANCE is self-clearing and reads back as 0. */
        *out = st->ctrl & ~(1u << TMDS_CTRL_CLEAR_BALANCE_SHIFT);
        return 1;

    case TMDS_WDATA:
        *out = 0;                     /* write-only */
        return 1;

    case TMDS_PEEK_SINGLE:
        /* Advances DC balance but does not shift the colour register. */
        tmds_encode_pixel(st, 0, s0);
        *out = tmds_pack(s0, interleave);
        return 1;

    case TMDS_POP_SINGLE:
        tmds_encode_pixel(st, 0, s0);
        *out = tmds_pack(s0, interleave);
        tmds_shift_colour(st, st->ctrl, 0);
        return 1;

    case TMDS_PEEK_DOUBLE_L0:
    case TMDS_PEEK_DOUBLE_L1:
    case TMDS_PEEK_DOUBLE_L2:
    case TMDS_POP_DOUBLE_L0:
    case TMDS_POP_DOUBLE_L1:
    case TMDS_POP_DOUBLE_L2: {
        /* PEEK and POP alternate every 4 bytes (PEEK_L0, POP_L0, PEEK_L1,
         * POP_L1, ...), so the lane index is the offset divided by 8, not 4.
         * Dividing by 4 yields 0, 2 and 4 and indexes past the end of the
         * three-element symbol arrays. */
        unsigned lane = (offset - TMDS_PEEK_DOUBLE_L0) / 8u;
        int is_pop = ((offset - TMDS_PEEK_DOUBLE_L0) % 8u) == 4u;

        tmds_layer_symbols(st, 0, s0);
        tmds_layer_symbols(st, 1, s1);
        /* Two 10-bit symbols at the bottom of the word, first pixel low. */
        *out = (uint32_t)s0[lane] | ((uint32_t)s1[lane] << 10);
        if (is_pop)
            tmds_shift_colour(st, st->ctrl, 1);
        return 1;
    }

    default:
        return 0;
    }
}

int tmds_write(tmds_state_t *st, uint32_t offset, uint32_t val) {
    switch (offset) {
    case TMDS_CTRL:
        st->ctrl = val & 0x1FFFFFFFu;
        if (val & (1u << TMDS_CTRL_CLEAR_BALANCE_SHIFT))
            for (unsigned l = 0; l < 2u; l++)
                for (unsigned lane = 0; lane < 3u; lane++)
                    st->balance[l][lane] = 0;
        return 1;

    case TMDS_WDATA:
        st->colour = val;
        return 1;

    default:
        return 1;        /* PEEK/POP are read-only; writes are ignored */
    }
}