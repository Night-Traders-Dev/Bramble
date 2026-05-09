/*
 * eMMC SPI-Mode Emulation
 *
 * Implements eMMC device protocol over SPI. Similar to SD card SPI mode
 * but uses CMD1 for initialization (no ACMD41) and has eMMC-specific
 * CSD/CID register contents.
 *
 * eMMC always uses block (sector) addressing for read/write commands.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "emmc.h"

/* eMMC command numbers */
#define CMD0    0   /* GO_IDLE_STATE */
#define CMD1    1   /* SEND_OP_COND (eMMC init) */
#define CMD8    8   /* SEND_EXT_CSD */
#define CMD9    9   /* SEND_CSD */
#define CMD10   10  /* SEND_CID */
#define CMD12   12  /* STOP_TRANSMISSION */
#define CMD13   13  /* SEND_STATUS */
#define CMD16   16  /* SET_BLOCKLEN */
#define CMD17   17  /* READ_SINGLE_BLOCK */
#define CMD18   18  /* READ_MULTIPLE_BLOCK */
#define CMD24   24  /* WRITE_BLOCK */
#define CMD25   25  /* WRITE_MULTIPLE_BLOCK */
#define CMD58   58  /* READ_OCR */

/* R1 response bits */
#define R1_IDLE         0x01
#define R1_ILLEGAL_CMD  0x04
#define R1_ADDR_ERR     0x20
#define R1_PARAM_ERR    0x40

/* Data tokens */
#define DATA_TOKEN_SINGLE   0xFE
#define DATA_TOKEN_MULTI    0xFC
#define DATA_TOKEN_STOP     0xFD

/* Data response */
#define DATA_RESP_ACCEPTED  0x05
#define DATA_RESP_WRITE_ERR 0x0D

/* ========================================================================
 * CSD/CID for eMMC
 * ======================================================================== */

/* eMMC CSD (v1.2 / EXT_CSD for capacity) */
static void build_emmc_csd(uint8_t *csd, size_t card_size) {
    memset(csd, 0, 16);
    /* CSD_STRUCTURE = 3 (v4.x eMMC) */
    uint32_t c_size = (uint32_t)(card_size / (512 * 1024)) - 1;

    csd[0]  = 0xD0;            /* CSD_STRUCTURE = 3 */
    csd[1]  = 0x0E;            /* TAAC */
    csd[2]  = 0x00;            /* NSAC */
    csd[3]  = 0x5A;            /* TRAN_SPEED = 26 MHz */
    csd[4]  = 0x5B;            /* CCC */
    csd[5]  = 0x59;            /* READ_BL_LEN = 9 */
    csd[6]  = 0x00;
    csd[7]  = (uint8_t)((c_size >> 16) & 0x3F);
    csd[8]  = (uint8_t)((c_size >> 8) & 0xFF);
    csd[9]  = (uint8_t)(c_size & 0xFF);
    csd[10] = 0x7F;
    csd[11] = 0x80;
    csd[12] = 0x0A;
    csd[13] = 0x40;
    csd[14] = 0x00;
    csd[15] = 0x01;
}

static void build_emmc_cid(uint8_t *cid) {
    memset(cid, 0, 16);
    cid[0]  = 0x45;            /* MID: Bramble eMMC */
    cid[1]  = 0x00;            /* reserved (eMMC) */
    cid[2]  = 0x00;            /* OID */
    cid[3]  = 'B';             /* PNM: "BRMMC" */
    cid[4]  = 'R';
    cid[5]  = 'M';
    cid[6]  = 'M';
    cid[7]  = 'C';
    cid[8]  = 0x00;
    cid[9]  = 0x10;            /* PRV: 1.0 */
    cid[10] = 0x00;            /* PSN */
    cid[11] = 0x00;
    cid[12] = 0x00;
    cid[13] = 0x01;
    cid[14] = 0x99;            /* MDT */
    cid[15] = 0x01;            /* CRC */
}

/* Build a minimal EXT_CSD (512 bytes) for CMD8 */
static void build_ext_csd(uint8_t *ext_csd, size_t card_size) {
    memset(ext_csd, 0, 512);
    /* SEC_COUNT at bytes [215:212] */
    uint32_t sectors = (uint32_t)(card_size / EMMC_BLOCK_SIZE);
    ext_csd[212] = (uint8_t)(sectors);
    ext_csd[213] = (uint8_t)(sectors >> 8);
    ext_csd[214] = (uint8_t)(sectors >> 16);
    ext_csd[215] = (uint8_t)(sectors >> 24);
    /* EXT_CSD_REV at byte [192] */
    ext_csd[192] = 0x06;  /* v4.5 */
    /* DEVICE_TYPE at byte [196] */
    ext_csd[196] = 0x03;  /* HS + 26/52 MHz */
}

/* ========================================================================
 * Initialization / Cleanup
 * ======================================================================== */

int emmc_init(emmc_t *em, const char *path, size_t size) {
    memset(em, 0, sizeof(*em));
    em->state = EMMC_STATE_IDLE;
    snprintf(em->path, sizeof(em->path), "%s", path);
    em->size = size;

    em->data = calloc(1, size);
    if (!em->data) {
        fprintf(stderr, "[eMMC] Failed to allocate %zu bytes\n", size);
        return -1;
    }

    FILE *f = fopen(path, "rb");
    if (f) {
        size_t n = fread(em->data, 1, size, f);
        fclose(f);
        fprintf(stderr, "[eMMC] Loaded %zu bytes from %s\n", n, path);
    } else {
        fprintf(stderr, "[eMMC] Created new %zu MB device: %s\n",
                size / (1024 * 1024), path);
    }

    return 0;
}

void emmc_cleanup(emmc_t *em) {
    emmc_flush(em);
    if (em->data) {
        free(em->data);
        em->data = NULL;
    }
}

void emmc_flush(emmc_t *em) {
    if (!em->dirty || !em->data) return;

    FILE *f = fopen(em->path, "wb");
    if (f) {
        fwrite(em->data, 1, em->size, f);
        fclose(f);
        em->dirty = 0;
        fprintf(stderr, "[eMMC] Flushed to %s\n", em->path);
    }
}

/* ========================================================================
 * Response helpers
 * ======================================================================== */

static void emmc_queue_r1(emmc_t *em, uint8_t status) {
    em->resp[0] = status;
    em->resp_len = 1;
    em->resp_pos = 0;
}

static void emmc_queue_r3(emmc_t *em, uint8_t r1, uint32_t data) {
    em->resp[0] = r1;
    em->resp[1] = (uint8_t)(data >> 24);
    em->resp[2] = (uint8_t)(data >> 16);
    em->resp[3] = (uint8_t)(data >> 8);
    em->resp[4] = (uint8_t)(data);
    em->resp_len = 5;
    em->resp_pos = 0;
}

static void emmc_queue_data(emmc_t *em, const uint8_t *data, int len) {
    int pos = em->resp_len;
    em->resp[pos++] = 0xFE;
    memcpy(&em->resp[pos], data, len);
    pos += len;
    em->resp[pos++] = 0x00;  /* CRC16 dummy */
    em->resp[pos++] = 0x00;
    em->resp_len = pos;
}

/* ========================================================================
 * Command processing
 * ======================================================================== */

static uint32_t emmc_cmd_arg(emmc_t *em) {
    return ((uint32_t)em->cmd_buf[1] << 24) |
           ((uint32_t)em->cmd_buf[2] << 16) |
           ((uint32_t)em->cmd_buf[3] << 8)  |
           ((uint32_t)em->cmd_buf[4]);
}

static void emmc_process_command(emmc_t *em) {
    uint8_t cmd = em->cmd_buf[0] & 0x3F;
    uint32_t arg = emmc_cmd_arg(em);
    uint8_t r1 = em->initialized ? 0x00 : R1_IDLE;

    switch (cmd) {
    case CMD0:
        em->initialized = 0;
        em->state = EMMC_STATE_IDLE;
        emmc_queue_r1(em, R1_IDLE);
        break;

    case CMD1:
        /* SEND_OP_COND: eMMC initialization */
        if (!em->initialized) {
            em->initialized = 1;
            em->state = EMMC_STATE_READY;
        }
        emmc_queue_r1(em, 0x00);
        break;

    case CMD8: {
        /* SEND_EXT_CSD: 512-byte extended CSD */
        emmc_queue_r1(em, r1);
        uint8_t ext_csd[512];
        build_ext_csd(ext_csd, em->size);
        /* Queue as data block — need larger response buffer for this */
        /* For SPI mode, send data token + 512 bytes + CRC */
        int pos = em->resp_len;
        em->resp[pos++] = 0xFE;
        /* Only copy what fits in resp buffer (it's BLOCK_SIZE + 16) */
        memcpy(&em->resp[pos], ext_csd, EMMC_BLOCK_SIZE);
        pos += EMMC_BLOCK_SIZE;
        em->resp[pos++] = 0x00;
        em->resp[pos++] = 0x00;
        em->resp_len = pos;
        break;
    }

    case CMD9: {
        uint8_t csd[16];
        build_emmc_csd(csd, em->size);
        emmc_queue_r1(em, r1);
        emmc_queue_data(em, csd, 16);
        break;
    }

    case CMD10: {
        uint8_t cid[16];
        build_emmc_cid(cid);
        emmc_queue_r1(em, r1);
        emmc_queue_data(em, cid, 16);
        break;
    }

    case CMD12:
        em->state = EMMC_STATE_READY;
        emmc_queue_r1(em, r1);
        break;

    case CMD13:
        em->resp[0] = r1;
        em->resp[1] = 0x00;
        em->resp_len = 2;
        em->resp_pos = 0;
        break;

    case CMD16:
        if (arg != EMMC_BLOCK_SIZE) {
            emmc_queue_r1(em, R1_PARAM_ERR);
        } else {
            emmc_queue_r1(em, r1);
        }
        break;

    case CMD17: {
        /* eMMC always uses sector addressing */
        uint32_t addr = arg * EMMC_BLOCK_SIZE;
        if (addr + EMMC_BLOCK_SIZE > em->size) {
            emmc_queue_r1(em, R1_ADDR_ERR);
        } else {
            emmc_queue_r1(em, r1);
            emmc_queue_data(em, &em->data[addr], EMMC_BLOCK_SIZE);
        }
        break;
    }

    case CMD18: {
        uint32_t addr = arg * EMMC_BLOCK_SIZE;
        if (addr + EMMC_BLOCK_SIZE > em->size) {
            emmc_queue_r1(em, R1_ADDR_ERR);
        } else {
            em->state = EMMC_STATE_SEND_MULTI;
            em->multi_addr = addr;
            emmc_queue_r1(em, r1);
            emmc_queue_data(em, &em->data[addr], EMMC_BLOCK_SIZE);
        }
        break;
    }

    case CMD24: {
        uint32_t addr = arg * EMMC_BLOCK_SIZE;
        if (addr + EMMC_BLOCK_SIZE > em->size) {
            emmc_queue_r1(em, R1_ADDR_ERR);
        } else {
            em->state = EMMC_STATE_RECEIVING;
            em->wr_addr = addr;
            em->wr_pos = 0;
            em->wr_expected = 1 + EMMC_BLOCK_SIZE + 2;
            emmc_queue_r1(em, r1);
        }
        break;
    }

    case CMD25: {
        uint32_t addr = arg * EMMC_BLOCK_SIZE;
        if (addr + EMMC_BLOCK_SIZE > em->size) {
            emmc_queue_r1(em, R1_ADDR_ERR);
        } else {
            em->state = EMMC_STATE_RECV_MULTI;
            em->wr_addr = addr;
            em->multi_addr = addr;
            em->wr_pos = 0;
            em->wr_expected = 1 + EMMC_BLOCK_SIZE + 2;
            emmc_queue_r1(em, r1);
        }
        break;
    }

    case CMD58: {
        /* OCR: eMMC, 3.3V, sector mode, power up complete */
        uint32_t ocr = 0xC0FF8080;
        if (!em->initialized) ocr &= ~0x80000000u;
        emmc_queue_r3(em, r1, ocr);
        break;
    }

    default:
        emmc_queue_r1(em, R1_ILLEGAL_CMD);
        break;
    }
}

/* ========================================================================
 * SPI transfer callback
 * ======================================================================== */

uint8_t emmc_spi_xfer(void *ctx, uint8_t mosi) {
    emmc_t *em = (emmc_t *)ctx;

    if (!em->cs_active) return 0xFF;

    /* Send queued response */
    if (em->resp_pos < em->resp_len) {
        uint8_t out = em->resp[em->resp_pos++];

        if (em->resp_pos >= em->resp_len) {
            if (em->state == EMMC_STATE_SEND_MULTI) {
                em->multi_addr += EMMC_BLOCK_SIZE;
                if (em->multi_addr + EMMC_BLOCK_SIZE <= em->size) {
                    em->resp_len = 0;
                    em->resp_pos = 0;
                    emmc_queue_data(em, &em->data[em->multi_addr], EMMC_BLOCK_SIZE);
                } else {
                    em->state = EMMC_STATE_READY;
                }
            } else if (em->state == EMMC_STATE_SENDING) {
                em->state = EMMC_STATE_READY;
            }
        }

        return out;
    }

    /* Receiving write data */
    if (em->state == EMMC_STATE_RECEIVING || em->state == EMMC_STATE_RECV_MULTI) {
        em->wr_buf[em->wr_pos++] = mosi;

        if (em->wr_pos >= em->wr_expected) {
            uint8_t token = em->wr_buf[0];
            int valid = 0;

            if (em->state == EMMC_STATE_RECEIVING && token == DATA_TOKEN_SINGLE) {
                valid = 1;
            } else if (em->state == EMMC_STATE_RECV_MULTI) {
                if (token == DATA_TOKEN_STOP) {
                    em->state = EMMC_STATE_READY;
                    return 0xFF;
                }
                if (token == DATA_TOKEN_MULTI) {
                    valid = 1;
                }
            }

            if (valid && em->wr_addr + EMMC_BLOCK_SIZE <= em->size) {
                memcpy(&em->data[em->wr_addr], &em->wr_buf[1], EMMC_BLOCK_SIZE);
                em->dirty = 1;

                em->resp[0] = DATA_RESP_ACCEPTED;
                em->resp[1] = 0x00;
                em->resp_len = 2;
                em->resp_pos = 0;

                if (em->state == EMMC_STATE_RECV_MULTI) {
                    em->wr_addr += EMMC_BLOCK_SIZE;
                    em->wr_pos = 0;
                } else {
                    em->state = EMMC_STATE_READY;
                }
            } else {
                em->resp[0] = DATA_RESP_WRITE_ERR;
                em->resp_len = 1;
                em->resp_pos = 0;
                em->state = EMMC_STATE_READY;
            }
        }

        return 0xFF;
    }

    /* Command assembly */
    if (em->cmd_pos == 0) {
        if ((mosi & 0xC0) == 0x40) {
            em->cmd_buf[em->cmd_pos++] = mosi;
        }
        return 0xFF;
    }

    em->cmd_buf[em->cmd_pos++] = mosi;

    if (em->cmd_pos >= 6) {
        em->cmd_pos = 0;
        emmc_process_command(em);
    }

    return 0xFF;
}

void emmc_spi_cs(void *ctx, int cs_active) {
    emmc_t *em = (emmc_t *)ctx;
    em->cs_active = cs_active;

    if (!cs_active) {
        em->cmd_pos = 0;
        em->resp_len = 0;
        em->resp_pos = 0;
    }
}
