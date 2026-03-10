/*
 * BME280 Temperature/Humidity/Pressure Sensor (I2C Device Plugin)
 *
 * Emulates the Bosch BME280 environmental sensor at the register level.
 * The I2C protocol uses a register pointer that auto-increments on read.
 * A write of one byte sets the register pointer; subsequent writes store
 * data at consecutive addresses.
 *
 * Calibration data is pre-filled with values that produce sensible
 * results through the BME280 compensation formulas. The raw ADC values
 * in the data registers are computed to yield the desired physical
 * readings when processed with the stored calibration constants.
 *
 * Default readings: 25.0 C, 101325 Pa, 50% RH.
 */

#include <string.h>
#include "bme280.h"

/* ========================================================================
 * Calibration data
 *
 * The BME280 compensation formulas use factory calibration constants.
 * We choose simple values and then compute raw ADC codes that, when
 * run through the standard formulas, yield the desired readings.
 *
 * Temperature calibration (0x88-0x8D):
 *   dig_T1 (uint16): base offset
 *   dig_T2 (int16):  linear coefficient
 *   dig_T3 (int16):  quadratic coefficient
 *
 * Pressure calibration (0x8E-0x9F):
 *   dig_P1..dig_P9
 *
 * Humidity calibration (0xA1, 0xE1-0xE7):
 *   dig_H1..dig_H6
 * ======================================================================== */

/* Write a little-endian uint16 into the register file */
static void reg_write16(uint8_t *regs, uint8_t addr, uint16_t val) {
    regs[addr]     = val & 0xFF;
    regs[addr + 1] = (val >> 8) & 0xFF;
}

/* Write a little-endian int16 into the register file */
static void reg_write_s16(uint8_t *regs, uint8_t addr, int16_t val) {
    reg_write16(regs, addr, (uint16_t)val);
}

/* Fill calibration registers with known-good constants.
 *
 * These values are representative of a real BME280 and produce
 * reasonable compensation results. They are derived from the
 * Bosch reference driver example data. */
static void fill_calibration(uint8_t *regs) {
    /* Temperature: dig_T1=27504, dig_T2=26435, dig_T3=-1000 */
    reg_write16(regs,  0x88, 27504u);      /* dig_T1 */
    reg_write_s16(regs, 0x8A, 26435);      /* dig_T2 */
    reg_write_s16(regs, 0x8C, -1000);      /* dig_T3 */

    /* Pressure: dig_P1..P9 */
    reg_write16(regs,  0x8E, 36477u);      /* dig_P1 */
    reg_write_s16(regs, 0x90, -10685);     /* dig_P2 */
    reg_write_s16(regs, 0x92, 3024);       /* dig_P3 */
    reg_write_s16(regs, 0x94, 2855);       /* dig_P4 */
    reg_write_s16(regs, 0x96, 140);        /* dig_P5 */
    reg_write_s16(regs, 0x98, -7);         /* dig_P6 */
    reg_write_s16(regs, 0x9A, 15500);      /* dig_P7 */
    reg_write_s16(regs, 0x9C, -14600);     /* dig_P8 */
    reg_write_s16(regs, 0x9E, 6000);       /* dig_P9 */

    /* Humidity: dig_H1 (0xA1), dig_H2..H6 (0xE1-0xE7) */
    regs[0xA1] = 75;                        /* dig_H1 (uint8) */
    reg_write_s16(regs, 0xE1, 370);        /* dig_H2 (int16) */
    regs[0xE3] = 0;                          /* dig_H3 (uint8) */
    /* dig_H4 (int16, 12-bit): [0xE4]<<4 | [0xE5]&0x0F */
    regs[0xE4] = (313 >> 4) & 0xFF;         /* dig_H4 high bits */
    regs[0xE5] = (313 & 0x0F);              /* dig_H4 low nibble */
    /* dig_H5 (int16, 12-bit): [0xE5]>>4 | [0xE6]<<4 */
    regs[0xE5] |= (0 << 4) & 0xF0;          /* dig_H5 low nibble */
    regs[0xE6] = 0;                           /* dig_H5 high bits */
    regs[0xE7] = 30;                          /* dig_H6 (int8) */
}

/* ========================================================================
 * Raw ADC value computation
 *
 * The BME280 data registers contain 20-bit (temp, press) or 16-bit
 * (humidity) raw ADC values. Firmware applies the compensation formulas
 * using the calibration constants to convert these to physical units.
 *
 * Rather than inverting the full compensation formula, we use the
 * forward formula to find raw values that produce the desired output.
 * For simplicity, we use pre-computed values that work with our
 * chosen calibration constants.
 * ======================================================================== */

/* Store 20-bit raw value into press/temp data registers (MSB, LSB, XLSB) */
static void store_raw20(uint8_t *regs, uint8_t msb_addr, uint32_t raw) {
    regs[msb_addr]     = (raw >> 12) & 0xFF;
    regs[msb_addr + 1] = (raw >> 4) & 0xFF;
    regs[msb_addr + 2] = (raw << 4) & 0xF0;
}

/* Store 16-bit raw humidity value */
static void store_raw_hum(uint8_t *regs, uint16_t raw) {
    regs[BME280_REG_HUM_MSB] = (raw >> 8) & 0xFF;
    regs[BME280_REG_HUM_LSB] = raw & 0xFF;
}

/* ========================================================================
 * Temperature raw value calculation
 *
 * BME280 compensation formula (from datasheet):
 *   var1 = ((adc_T/16384.0 - dig_T1/1024.0)) * dig_T2
 *   var2 = ((adc_T/131072.0 - dig_T1/8192.0)^2) * dig_T3
 *   t_fine = var1 + var2
 *   T = t_fine / 5120.0
 *
 * We invert this to find adc_T for a given temperature. With our
 * calibration constants (T1=27504, T2=26435, T3=-1000), we solve
 * for adc_T numerically using a linear approximation.
 * ======================================================================== */

static uint32_t temp_to_raw(const uint8_t *regs, float temp_c) {
    /* Read calibration */
    uint16_t dig_T1 = regs[0x88] | ((uint16_t)regs[0x89] << 8);
    int16_t  dig_T2 = (int16_t)(regs[0x8A] | ((uint16_t)regs[0x8B] << 8));
    int16_t  dig_T3 = (int16_t)(regs[0x8C] | ((uint16_t)regs[0x8D] << 8));

    /* Target t_fine from desired temperature */
    float t_fine_target = temp_c * 5120.0f;

    /* Newton's method: find adc_T such that forward formula gives t_fine_target.
     * Start with a reasonable initial guess. */
    float adc = (float)dig_T1 * 16.0f + 100000.0f;

    for (int i = 0; i < 20; i++) {
        float var1 = (adc / 16384.0f - (float)dig_T1 / 1024.0f) * (float)dig_T2;
        float var2_base = adc / 131072.0f - (float)dig_T1 / 8192.0f;
        float var2 = var2_base * var2_base * (float)dig_T3;
        float t_fine = var1 + var2;
        float err = t_fine - t_fine_target;

        if (err > -1.0f && err < 1.0f) break;

        /* Derivative of t_fine w.r.t. adc_T */
        float dvar1 = (float)dig_T2 / 16384.0f;
        float dvar2 = 2.0f * var2_base * (float)dig_T3 / 131072.0f;
        float deriv = dvar1 + dvar2;
        if (deriv == 0.0f) break;

        adc -= err / deriv;
    }

    if (adc < 0.0f) adc = 0.0f;
    if (adc > 1048575.0f) adc = 1048575.0f;
    return (uint32_t)adc;
}

/* ========================================================================
 * Pressure raw value calculation
 *
 * Uses t_fine from temperature compensation as an input. We first
 * compute t_fine from the current temperature raw value, then invert
 * the pressure formula.
 * ======================================================================== */

static float compute_t_fine(const uint8_t *regs, uint32_t adc_T) {
    uint16_t dig_T1 = regs[0x88] | ((uint16_t)regs[0x89] << 8);
    int16_t  dig_T2 = (int16_t)(regs[0x8A] | ((uint16_t)regs[0x8B] << 8));
    int16_t  dig_T3 = (int16_t)(regs[0x8C] | ((uint16_t)regs[0x8D] << 8));

    float var1 = ((float)adc_T / 16384.0f - (float)dig_T1 / 1024.0f) *
                 (float)dig_T2;
    float var2_base = (float)adc_T / 131072.0f - (float)dig_T1 / 8192.0f;
    float var2 = var2_base * var2_base * (float)dig_T3;
    return var1 + var2;
}

static uint32_t pressure_to_raw(const uint8_t *regs, float pressure_pa,
                                float t_fine) {
    /* Read pressure calibration */
    uint16_t dig_P1 = regs[0x8E] | ((uint16_t)regs[0x8F] << 8);
    int16_t  dig_P2 = (int16_t)(regs[0x90] | ((uint16_t)regs[0x91] << 8));
    int16_t  dig_P3 = (int16_t)(regs[0x92] | ((uint16_t)regs[0x93] << 8));
    int16_t  dig_P4 = (int16_t)(regs[0x94] | ((uint16_t)regs[0x95] << 8));
    int16_t  dig_P5 = (int16_t)(regs[0x96] | ((uint16_t)regs[0x97] << 8));
    int16_t  dig_P6 = (int16_t)(regs[0x98] | ((uint16_t)regs[0x99] << 8));
    int16_t  dig_P7 = (int16_t)(regs[0x9A] | ((uint16_t)regs[0x9B] << 8));
    int16_t  dig_P8 = (int16_t)(regs[0x9C] | ((uint16_t)regs[0x9D] << 8));
    int16_t  dig_P9 = (int16_t)(regs[0x9E] | ((uint16_t)regs[0x9F] << 8));

    /* Binary search for adc_P that produces target pressure.
     * The forward formula is complex (rational function), so we
     * evaluate it directly and search. */
    uint32_t lo = 0, hi = 1048575;

    for (int i = 0; i < 30; i++) {
        uint32_t mid = (lo + hi) / 2;

        /* Forward pressure formula (from BME280 datasheet, float version) */
        float var1 = t_fine / 2.0f - 64000.0f;
        float var2 = var1 * var1 * (float)dig_P6 / 32768.0f;
        var2 = var2 + var1 * (float)dig_P5 * 2.0f;
        var2 = var2 / 4.0f + (float)dig_P4 * 65536.0f;
        var1 = ((float)dig_P3 * var1 * var1 / 524288.0f +
                (float)dig_P2 * var1) / 524288.0f;
        var1 = (1.0f + var1 / 32768.0f) * (float)dig_P1;

        if (var1 == 0.0f) return mid;

        float p = 1048576.0f - (float)mid;
        p = (p - var2 / 4096.0f) * 6250.0f / var1;
        var1 = (float)dig_P9 * p * p / 2147483648.0f;
        var2 = p * (float)dig_P8 / 32768.0f;
        p = p + (var1 + var2 + (float)dig_P7) / 16.0f;

        if (p < pressure_pa)
            hi = mid;
        else
            lo = mid;

        if (hi - lo <= 1) break;
    }

    return (lo + hi) / 2;
}

/* ========================================================================
 * Humidity raw value calculation
 * ======================================================================== */

static uint16_t humidity_to_raw(const uint8_t *regs, float humidity_pct,
                                float t_fine) {
    /* Read humidity calibration */
    uint8_t  dig_H1 = regs[0xA1];
    int16_t  dig_H2 = (int16_t)(regs[0xE1] | ((uint16_t)regs[0xE2] << 8));
    uint8_t  dig_H3 = regs[0xE3];
    int16_t  dig_H4 = ((int16_t)regs[0xE4] << 4) | (regs[0xE5] & 0x0F);
    int16_t  dig_H5 = ((int16_t)regs[0xE6] << 4) | ((regs[0xE5] >> 4) & 0x0F);
    int8_t   dig_H6 = (int8_t)regs[0xE7];

    /* Binary search for adc_H */
    uint32_t lo = 0, hi = 65535;

    for (int i = 0; i < 25; i++) {
        uint32_t mid = (lo + hi) / 2;

        /* Forward humidity formula */
        float h = t_fine - 76800.0f;
        if (h == 0.0f) return (uint16_t)mid;

        float x1 = (float)mid - ((float)dig_H4 * 64.0f +
                   ((float)dig_H5 / 16384.0f) * h);
        float x2 = (float)dig_H2 / 65536.0f *
                   (1.0f + ((float)dig_H6 / 67108864.0f) * h *
                   (1.0f + ((float)dig_H3 / 67108864.0f) * h));
        float hum = x1 * x2;
        hum = hum * (1.0f - (float)dig_H1 * hum / 524288.0f);

        if (hum < 0.0f) hum = 0.0f;
        if (hum > 100.0f) hum = 100.0f;

        if (hum < humidity_pct)
            lo = mid;
        else
            hi = mid;

        if (hi - lo <= 1) break;
    }

    return (uint16_t)((lo + hi) / 2);
}

/* ========================================================================
 * Initialization
 * ======================================================================== */

void bme280_init(bme280_t *dev) {
    memset(dev, 0, sizeof(*dev));

    /* Chip ID (read-only) */
    dev->regs[BME280_REG_CHIP_ID] = BME280_CHIP_ID;

    /* Fill calibration data */
    fill_calibration(dev->regs);

    /* Status: idle (not measuring, no NVM copy in progress) */
    dev->regs[BME280_REG_STATUS] = 0x00;

    /* Default mode: sleep */
    dev->regs[BME280_REG_CTRL_MEAS] = BME280_MODE_SLEEP;

    /* Set default readings */
    bme280_set_temperature(dev, 25.0f);
    bme280_set_pressure(dev, 101325.0f);
    bme280_set_humidity(dev, 50.0f);
}

/* ========================================================================
 * Set sensor readings
 * ======================================================================== */

void bme280_set_temperature(bme280_t *dev, float temp_c) {
    uint32_t raw = temp_to_raw(dev->regs, temp_c);
    store_raw20(dev->regs, BME280_REG_TEMP_MSB, raw);
}

void bme280_set_pressure(bme280_t *dev, float pressure_pa) {
    /* Pressure compensation depends on t_fine from current temp reading */
    uint32_t adc_T = ((uint32_t)dev->regs[BME280_REG_TEMP_MSB] << 12) |
                     ((uint32_t)dev->regs[BME280_REG_TEMP_LSB] << 4) |
                     ((uint32_t)dev->regs[BME280_REG_TEMP_XLSB] >> 4);
    float t_fine = compute_t_fine(dev->regs, adc_T);

    uint32_t raw = pressure_to_raw(dev->regs, pressure_pa, t_fine);
    store_raw20(dev->regs, BME280_REG_PRESS_MSB, raw);
}

void bme280_set_humidity(bme280_t *dev, float humidity_pct) {
    /* Humidity compensation depends on t_fine from current temp reading */
    uint32_t adc_T = ((uint32_t)dev->regs[BME280_REG_TEMP_MSB] << 12) |
                     ((uint32_t)dev->regs[BME280_REG_TEMP_LSB] << 4) |
                     ((uint32_t)dev->regs[BME280_REG_TEMP_XLSB] >> 4);
    float t_fine = compute_t_fine(dev->regs, adc_T);

    uint16_t raw = humidity_to_raw(dev->regs, humidity_pct, t_fine);
    store_raw_hum(dev->regs, raw);
}

/* ========================================================================
 * Soft reset
 * ======================================================================== */

static void bme280_soft_reset(bme280_t *dev) {
    /* Preserve calibration data, reset control registers and data */
    dev->regs[BME280_REG_CTRL_HUM]  = 0x00;
    dev->regs[BME280_REG_STATUS]    = 0x00;
    dev->regs[BME280_REG_CTRL_MEAS] = 0x00;
    dev->regs[BME280_REG_CONFIG]    = 0x00;

    /* Reset data registers to defaults */
    dev->regs[BME280_REG_PRESS_MSB]  = 0x80;
    dev->regs[BME280_REG_PRESS_LSB]  = 0x00;
    dev->regs[BME280_REG_PRESS_XLSB] = 0x00;
    dev->regs[BME280_REG_TEMP_MSB]   = 0x80;
    dev->regs[BME280_REG_TEMP_LSB]   = 0x00;
    dev->regs[BME280_REG_TEMP_XLSB]  = 0x00;
    dev->regs[BME280_REG_HUM_MSB]    = 0x80;
    dev->regs[BME280_REG_HUM_LSB]    = 0x00;
}

/* ========================================================================
 * I2C interface callbacks
 *
 * I2C protocol:
 *   Write transaction: first byte = register address, subsequent = data
 *   Read transaction:  reads from current register pointer, auto-increments
 * ======================================================================== */

int bme280_i2c_write(void *ctx, uint8_t data) {
    bme280_t *dev = (bme280_t *)ctx;

    if (!dev->ptr_set) {
        /* First byte of write: set register pointer */
        dev->reg_ptr = data;
        dev->ptr_set = 1;
    } else {
        /* Subsequent bytes: write to register */
        uint8_t addr = dev->reg_ptr;

        /* Handle special registers */
        if (addr == BME280_REG_RESET && data == BME280_RESET_CMD) {
            bme280_soft_reset(dev);
        } else if (addr == BME280_REG_CHIP_ID) {
            /* Read-only, ignore write */
        } else if (addr >= 0x88 && addr <= 0x9F) {
            /* Calibration data, read-only */
        } else if (addr == 0xA1) {
            /* Calibration data, read-only */
        } else if (addr >= 0xE1 && addr <= 0xE7) {
            /* Calibration data, read-only */
        } else {
            dev->regs[addr] = data;
        }

        dev->reg_ptr++;
    }

    return 0;  /* ACK */
}

uint8_t bme280_i2c_read(void *ctx) {
    bme280_t *dev = (bme280_t *)ctx;
    uint8_t val = dev->regs[dev->reg_ptr];
    dev->reg_ptr++;
    return val;
}

void bme280_i2c_start(void *ctx) {
    bme280_t *dev = (bme280_t *)ctx;
    dev->ptr_set = 0;  /* Next write byte will be register address */
}

void bme280_i2c_stop(void *ctx) {
    (void)ctx;
    /* No special handling needed */
}
