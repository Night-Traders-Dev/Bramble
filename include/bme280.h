#ifndef BME280_H
#define BME280_H

#include <stdint.h>

/* ========================================================================
 * BME280 Temperature/Humidity/Pressure Sensor (I2C Device Plugin)
 *
 * Emulates the Bosch BME280 environmental sensor over I2C.
 * Default address 0x76 (SDO=GND), alternate 0x77 (SDO=VDDIO).
 *
 * Register map:
 *   0x88-0x9F  Calibration data (temp + pressure)
 *   0xA1       Calibration data (humidity)
 *   0xD0       Chip ID (read-only, 0x60)
 *   0xE0       Reset (write 0xB6 for soft reset)
 *   0xE1-0xE7  Calibration data (humidity)
 *   0xF2       ctrl_hum  (humidity oversampling)
 *   0xF3       status    (measuring | im_update)
 *   0xF4       ctrl_meas (temp/press oversampling + mode)
 *   0xF5       config    (standby, filter, spi3w_en)
 *   0xF7-0xF9  press_msb, press_lsb, press_xlsb
 *   0xFA-0xFC  temp_msb, temp_lsb, temp_xlsb
 *   0xFD-0xFE  hum_msb, hum_lsb
 *
 * I2C protocol: write sets register pointer, subsequent reads auto-
 * increment through the register map.
 *
 * Usage:
 *   bme280_t bme280;
 *   bme280_init(&bme280);
 *   i2c_attach_device(i2c_num, 0x76,
 *                     bme280_i2c_write, bme280_i2c_read,
 *                     bme280_i2c_start, bme280_i2c_stop, &bme280);
 * ======================================================================== */

#define BME280_REG_SIZE     256     /* Full register address space */

/* Chip identification */
#define BME280_CHIP_ID      0x60

/* Register addresses */
#define BME280_REG_CALIB00      0x88    /* Calibration 0x88-0x9F (26 bytes) */
#define BME280_REG_CALIB25      0xA1    /* Calibration humidity (1 byte) */
#define BME280_REG_CHIP_ID      0xD0    /* Chip ID */
#define BME280_REG_RESET        0xE0    /* Soft reset */
#define BME280_REG_CALIB26      0xE1    /* Calibration humidity (7 bytes) */
#define BME280_REG_CTRL_HUM     0xF2    /* Humidity control */
#define BME280_REG_STATUS       0xF3    /* Status */
#define BME280_REG_CTRL_MEAS    0xF4    /* Measurement control */
#define BME280_REG_CONFIG       0xF5    /* Configuration */
#define BME280_REG_PRESS_MSB    0xF7    /* Pressure data */
#define BME280_REG_PRESS_LSB    0xF8
#define BME280_REG_PRESS_XLSB   0xF9
#define BME280_REG_TEMP_MSB     0xFA    /* Temperature data */
#define BME280_REG_TEMP_LSB     0xFB
#define BME280_REG_TEMP_XLSB    0xFC
#define BME280_REG_HUM_MSB      0xFD    /* Humidity data */
#define BME280_REG_HUM_LSB      0xFE

/* Reset command */
#define BME280_RESET_CMD    0xB6

/* ctrl_meas mode bits */
#define BME280_MODE_SLEEP   0x00
#define BME280_MODE_FORCED  0x01
#define BME280_MODE_NORMAL  0x03

/* BME280 device state */
typedef struct {
    uint8_t regs[BME280_REG_SIZE];  /* Full register file */
    uint8_t reg_ptr;                /* Current register pointer */
    int     ptr_set;                /* Register pointer has been set */
} bme280_t;

/* Initialize BME280 with default calibration and readings */
void bme280_init(bme280_t *dev);

/* Set simulated sensor readings (updates raw data registers) */
void bme280_set_temperature(bme280_t *dev, float temp_c);
void bme280_set_pressure(bme280_t *dev, float pressure_pa);
void bme280_set_humidity(bme280_t *dev, float humidity_pct);

/* I2C device callbacks (for i2c_attach_device) */
int     bme280_i2c_write(void *ctx, uint8_t data);
uint8_t bme280_i2c_read(void *ctx);
void    bme280_i2c_start(void *ctx);
void    bme280_i2c_stop(void *ctx);

#endif /* BME280_H */
