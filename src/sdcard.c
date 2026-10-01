/*
 * SD Card SPI-Mode Emulation
 *
 * Implements the SD card SPI protocol state machine. Each SPI byte
 * exchange (MOSI/MISO) is processed through a command parser and
 * response generator.
 *
 * SPI protocol summary:
 *   - Commands are 6 bytes: 0x40|cmd, arg[3:0], CRC
 *   - R1 response: 1 byte status (bit 0 = idle)
 *   - Read: R1 + wait bytes (0xFF) + data token (0xFE) + data + CRC16
 *   - Write: host sends data token (0xFE) + data + CRC16, card responds
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdcard.h"

/* SD command numbers */
#define CMD0    0   /* GO_IDLE_STATE */
#define CMD8    8   /* SEND_IF_COND */
#define CMD9    9   /* SEND_CSD */
#define CMD10   10  /* SEND_CID */
#define CMD12   12  /* STOP_TRANSMISSION */
#define CMD13   13  /* SEND_STATUS */
#define CMD16   16  /* SET_BLOCKLEN */
#define CMD17   17  /* READ_SINGLE_BLOCK */
#define CMD18   18  /* READ_MULTIPLE_BLOCK */
#define CMD24   24  /* WRITE_BLOCK */
#define CMD25   25  /* WRITE_MULTIPLE_BLOCK */
#define CMD55   55  /* APP_CMD */
#define CMD58   58  /* READ_OCR */
#define ACMD41  41  /* SD_SEND_OP_COND */

/* R1 response bits */
#define R1_IDLE         0x01
#define R1_ERASE_RESET  0x02
#define R1_ILLEGAL_CMD  0x04
#define R1_CRC_ERR      0x08
#define R1_ERASE_ERR    0x10
#define R1_ADDR_ERR     0x20
#define R1_PARAM_ERR    0x40

/* Data tokens */
#define DATA_TOKEN_SINGLE   0xFE
#define DATA_TOKEN_MULTI    0xFC
#define DATA_TOKEN_STOP     0xFD

/* Data response tokens */
#define DATA_RESP_ACCEPTED  0x05
#define DATA_RESP_CRC_ERR   0x0B
#define DATA_RESP_WRITE_ERR 0x0D

/* ========================================================================
 * CSD/CID register builders
 * ======================================================================== */

/* Build CSD v2.0 (SDHC) register - 16 bytes */
static void build_csd_v2(uint8_t *csd, size_t card_size) {
    memset(csd, 0, 16);
    uint32_t c_size = (uint32_t)(card_size / (512 * 1024)) - 1;

    csd[0]  = 0x40;            /* CSD_STRUCTURE = 1 (v2.0) */
    csd[1]  = 0x0E;            /* TAAC */
    csd[2]  = 0x00;            /* NSAC */
    csd[3]  = 0x5A;            /* TRAN_SPEED = 25 MHz */
    csd[4]  = 0x5B;            /* CCC high */
    csd[5]  = 0x59;            /* CCC low + READ_BL_LEN = 9 (512) */
    csd[6]  = 0x00;
    csd[7]  = (uint8_t)((c_size >> 16) & 0x3F);
    csd[8]  = (uint8_t)((c_size >> 8) & 0xFF);
    csd[9]  = (uint8_t)(c_size & 0xFF);
    csd[10] = 0x7F;            /* ERASE_BLK_EN=1, SECTOR_SIZE */
    csd[11] = 0x80;            /* SECTOR_SIZE + WP_GRP_SIZE */
    csd[12] = 0x0A;            /* R2W_FACTOR = 2 */
    csd[13] = 0x40;            /* WRITE_BL_LEN = 9 */
    csd[14] = 0x00;
    csd[15] = 0x01;            /* CRC placeholder */
}

/* Build CID register - 16 bytes */
static void build_cid(uint8_t *cid) {
    memset(cid, 0, 16);
    cid[0]  = 0x02;            /* MID: Bramble */
    cid[1]  = 'B';             /* OID */
    cid[2]  = 'R';
    cid[3]  = 'B';             /* PNM: "BRMSD" */
    cid[4]  = 'R';
    cid[5]  = 'M';
    cid[6]  = 'S';
    cid[7]  = 'D';
    cid[8]  = 0x10;            /* PRV: 1.0 */
    cid[9]  = 0x00;            /* PSN */
    cid[10] = 0x00;
    cid[11] = 0x00;
    cid[12] = 0x01;
    cid[13] = 0x01;            /* MDT: Jan 2025 */
    cid[14] = 0x99;
    cid[15] = 0x01;            /* CRC placeholder */
}

/* ========================================================================
 * Initialization / Cleanup
 * ======================================================================== */

int sdcard_init(sdcard_t *sd, const char *path, size_t size) {
    memset(sd, 0, sizeof(*sd));
    sd->state = SD_STATE_IDLE;
    sd->sdhc = (size > 2UL * 1024 * 1024 * 1024) ? 0 : 1; /* SDHC for <=2GB too for simplicity */
    sd->sdhc = 1;  /* Always SDHC mode for clean block addressing */

    strncpy(sd->path, path, sizeof(sd->path) - 1);
    sd->size = size;

    /* Allocate backing storage */
    sd->data = calloc(1, size);
    if (!sd->data) {
        fprintf(stderr, "[SDCard] Failed to allocate %zu bytes\n", size);
        return -1;
    }

    /* Try to load existing image */
    FILE *f = fopen(path, "rb");
    if (f) {
        size_t n = fread(sd->data, 1, size, f);
        fclose(f);
        fprintf(stderr, "[SDCard] Loaded %zu bytes from %s\n", n, path);
    } else {
        /* Initialize as blank (all 0x00) */
        fprintf(stderr, "[SDCard] Created new %zu MB card: %s\n",
                size / (1024 * 1024), path);
    }

    return 0;
}

void sdcard_cleanup(sdcard_t *sd) {
    sdcard_flush(sd);
    if (sd->data) {
        free(sd->data);
        sd->data = NULL;
    }
}

void sdcard_flush(sdcard_t *sd) {
    if (!sd->dirty || !sd->data) return;

    FILE *f = fopen(sd->path, "wb");
    if (f) {
        fwrite(sd->data, 1, sd->size, f);
        fclose(f);
        sd->dirty = 0;
        fprintf(stderr, "[SDCard] Flushed to %s\n", sd->path);
    }
}

/* ========================================================================
 * Command response helpers
 * ======================================================================== */

static void queue_r1(sdcard_t *sd, uint8_t status) {
    sd->resp[0] = status;
    sd->resp_len = 1;
    sd->resp_pos = 0;
}

static void queue_r3_r7(sdcard_t *sd, uint8_t r1, uint32_t data) {
    sd->resp[0] = r1;
    sd->resp[1] = (uint8_t)(data >> 24);
    sd->resp[2] = (uint8_t)(data >> 16);
    sd->resp[3] = (uint8_t)(data >> 8);
    sd->resp[4] = (uint8_t)(data);
    sd->resp_len = 5;
    sd->resp_pos = 0;
}

static void queue_data_block(sdcard_t *sd, const uint8_t *data, int len) {
    int pos = sd->resp_len;
    /* Wait byte */
    sd->resp[pos++] = 0xFE;  /* Data token */
    memcpy(&sd->resp[pos], data, len);
    pos += len;
    /* CRC16 (dummy) */
    sd->resp[pos++] = 0x00;
    sd->resp[pos++] = 0x00;
    sd->resp_len = pos;
}

/* ========================================================================
 * Command processing
 * ======================================================================== */

static uint32_t cmd_arg(sdcard_t *sd) {
    return ((uint32_t)sd->cmd_buf[1] << 24) |
           ((uint32_t)sd->cmd_buf[2] << 16) |
           ((uint32_t)sd->cmd_buf[3] << 8)  |
           ((uint32_t)sd->cmd_buf[4]);
}

static void process_command(sdcard_t *sd) {
    uint8_t cmd = sd->cmd_buf[0] & 0x3F;
    uint32_t arg = cmd_arg(sd);
    uint8_t r1 = sd->initialized ? 0x00 : R1_IDLE;

    /* Handle ACMD (application-specific commands) */
    if (sd->app_cmd) {
        sd->app_cmd = 0;
        switch (cmd) {
        case ACMD41:
            /* SD_SEND_OP_COND: initialize card */
            if (!sd->initialized) {
                sd->initialized = 1;
                sd->state = SD_STATE_READY;
            }
            queue_r1(sd, 0x00);  /* Ready */
            return;
        default:
            queue_r1(sd, R1_ILLEGAL_CMD);
            return;
        }
    }

    switch (cmd) {
    case CMD0:
        /* GO_IDLE_STATE: reset card */
        sd->initialized = 0;
        sd->state = SD_STATE_IDLE;
        sd->app_cmd = 0;
        queue_r1(sd, R1_IDLE);
        break;

    case CMD8:
        /* SEND_IF_COND: voltage check (SD v2) */
        /* Echo back check pattern in lower 12 bits */
        queue_r3_r7(sd, R1_IDLE, 0x000001AA);
        break;

    case CMD9: {
        /* SEND_CSD */
        queue_r1(sd, r1);
        uint8_t csd[16];
        build_csd_v2(csd, sd->size);
        queue_data_block(sd, csd, 16);
        break;
    }

    case CMD10: {
        /* SEND_CID */
        queue_r1(sd, r1);
        uint8_t cid[16];
        build_cid(cid);
        queue_data_block(sd, cid, 16);
        break;
    }

    case CMD12:
        /* STOP_TRANSMISSION */
        sd->state = SD_STATE_READY;
        queue_r1(sd, r1);
        break;

    case CMD13:
        /* SEND_STATUS: R2 response (2 bytes) */
        sd->resp[0] = r1;
        sd->resp[1] = 0x00;
        sd->resp_len = 2;
        sd->resp_pos = 0;
        break;

    case CMD16:
        /* SET_BLOCKLEN: only 512 supported */
        if (arg != SD_BLOCK_SIZE) {
            queue_r1(sd, R1_PARAM_ERR);
        } else {
            queue_r1(sd, r1);
        }
        break;

    case CMD17: {
        /* READ_SINGLE_BLOCK */
        uint64_t addr = sd->sdhc ? (uint64_t)arg * SD_BLOCK_SIZE : arg;
        if (addr + SD_BLOCK_SIZE > (uint64_t)sd->size) {
            queue_r1(sd, R1_ADDR_ERR);
        } else {
            queue_r1(sd, r1);
            queue_data_block(sd, &sd->data[addr], SD_BLOCK_SIZE);
        }
        break;
    }

    case CMD18: {
        /* READ_MULTIPLE_BLOCK */
        uint64_t addr = sd->sdhc ? (uint64_t)arg * SD_BLOCK_SIZE : arg;
        if (addr + SD_BLOCK_SIZE > (uint64_t)sd->size) {
            queue_r1(sd, R1_ADDR_ERR);
        } else {
            sd->state = SD_STATE_SEND_MULTI;
            sd->multi_addr = addr;
            queue_r1(sd, r1);
            /* Queue first block */
            queue_data_block(sd, &sd->data[addr], SD_BLOCK_SIZE);
        }
        break;
    }

    case CMD24: {
        /* WRITE_BLOCK */
        uint64_t addr = sd->sdhc ? (uint64_t)arg * SD_BLOCK_SIZE : arg;
        if (addr + SD_BLOCK_SIZE > (uint64_t)sd->size) {
            queue_r1(sd, R1_ADDR_ERR);
        } else {
            sd->state = SD_STATE_RECEIVING;
            sd->wr_addr = addr;
            sd->wr_pos = 0;
            sd->wr_expected = 1 + SD_BLOCK_SIZE + 2; /* token + data + CRC */
            queue_r1(sd, r1);
        }
        break;
    }

    case CMD25: {
        /* WRITE_MULTIPLE_BLOCK */
        uint64_t addr = sd->sdhc ? (uint64_t)arg * SD_BLOCK_SIZE : arg;
        if (addr + SD_BLOCK_SIZE > (uint64_t)sd->size) {
            queue_r1(sd, R1_ADDR_ERR);
        } else {
            sd->state = SD_STATE_RECV_MULTI;
            sd->wr_addr = addr;
            sd->multi_addr = addr;
            sd->wr_pos = 0;
            sd->wr_expected = 1 + SD_BLOCK_SIZE + 2;
            queue_r1(sd, r1);
        }
        break;
    }

    case CMD55:
        /* APP_CMD: next command is ACMD */
        sd->app_cmd = 1;
        queue_r1(sd, r1);
        break;

    case CMD58:
        /* READ_OCR */
        {
            uint32_t ocr = 0x40FF8000;  /* SDHC, 3.3V */
            if (sd->initialized) ocr |= 0x80000000;  /* Power up complete */
            queue_r3_r7(sd, r1, ocr);
        }
        break;

    default:
        queue_r1(sd, R1_ILLEGAL_CMD);
        break;
    }
}

/* ========================================================================
 * SPI transfer callback
 * ======================================================================== */

uint8_t sdcard_spi_xfer(void *ctx, uint8_t mosi) {
    sdcard_t *sd = (sdcard_t *)ctx;

    if (!sd->cs_active) return 0xFF;

    /* If we have a queued response, send it */
    if (sd->resp_pos < sd->resp_len) {
        uint8_t out = sd->resp[sd->resp_pos++];

        /* Check if we finished sending the response */
        if (sd->resp_pos >= sd->resp_len) {
            /* Multi-block read: queue next block */
            if (sd->state == SD_STATE_SEND_MULTI) {
                sd->multi_addr += SD_BLOCK_SIZE;
                if ((uint64_t)sd->multi_addr + SD_BLOCK_SIZE <= (uint64_t)sd->size) {
                    sd->resp_len = 0;
                    sd->resp_pos = 0;
                    queue_data_block(sd, &sd->data[sd->multi_addr], SD_BLOCK_SIZE);
                } else {
                    sd->state = SD_STATE_READY;
                }
            } else if (sd->state == SD_STATE_SENDING) {
                sd->state = SD_STATE_READY;
            }
        }

        return out;
    }

    /* Receiving write data */
    if (sd->state == SD_STATE_RECEIVING || sd->state == SD_STATE_RECV_MULTI) {
        sd->wr_buf[sd->wr_pos++] = mosi;

        if (sd->wr_pos >= sd->wr_expected) {
            /* Check for valid data token */
            uint8_t token = sd->wr_buf[0];
            int valid = 0;

            if (sd->state == SD_STATE_RECEIVING && token == DATA_TOKEN_SINGLE) {
                valid = 1;
            } else if (sd->state == SD_STATE_RECV_MULTI) {
                if (token == DATA_TOKEN_STOP) {
                    /* Stop multi-block write */
                    sd->state = SD_STATE_READY;
                    return 0xFF;
                }
                if (token == DATA_TOKEN_MULTI) {
                    valid = 1;
                }
            }

            if (valid && (uint64_t)sd->wr_addr + SD_BLOCK_SIZE <= (uint64_t)sd->size) {
                /* Write data to card (skip token byte, ignore CRC) */
                memcpy(&sd->data[sd->wr_addr], &sd->wr_buf[1], SD_BLOCK_SIZE);
                sd->dirty = 1;

                /* Queue data response */
                sd->resp[0] = DATA_RESP_ACCEPTED;
                sd->resp[1] = 0x00;  /* Busy done */
                sd->resp_len = 2;
                sd->resp_pos = 0;

                if (sd->state == SD_STATE_RECV_MULTI) {
                    /* Advance to next block */
                    sd->wr_addr += SD_BLOCK_SIZE;
                    sd->wr_pos = 0;
                } else {
                    sd->state = SD_STATE_READY;
                }
            } else {
                /* Write error */
                sd->resp[0] = DATA_RESP_WRITE_ERR;
                sd->resp_len = 1;
                sd->resp_pos = 0;
                sd->state = SD_STATE_READY;
            }
        }

        return 0xFF;  /* Card busy/ready during write */
    }

    /* Command assembly: commands start with bit 7=0, bit 6=1 */
    if (sd->cmd_pos == 0) {
        if ((mosi & 0xC0) == 0x40) {
            sd->cmd_buf[sd->cmd_pos++] = mosi;
        }
        return 0xFF;
    }

    sd->cmd_buf[sd->cmd_pos++] = mosi;

    if (sd->cmd_pos >= 6) {
        /* Complete command received */
        sd->cmd_pos = 0;
        process_command(sd);
    }

    return 0xFF;
}

void sdcard_spi_cs(void *ctx, int cs_active) {
    sdcard_t *sd = (sdcard_t *)ctx;
    sd->cs_active = cs_active;

    if (!cs_active) {
        /* CS deasserted: reset command state */
        sd->cmd_pos = 0;
        sd->resp_len = 0;
        sd->resp_pos = 0;
    }
}
