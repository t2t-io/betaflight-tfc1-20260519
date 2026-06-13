/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * MMC560XNJ Magnetometer Driver
 *
 * References:
 * MMC560XNJ datasheet - https://www.memsic.com/Public/Uploads/uploadfile/files/20220119/MMC560XNJDatasheetRev.B.pdf
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "build/debug_print.h"

#ifdef USE_MAG_MMC560X

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_mmc560x.h"

// MMC560X I2C Address
#define MMC560X_I2C_ADDRESS             0x30

// Register addresses
#define MMC560X_REG_XOUT0               0x00    // X output [17:10]
#define MMC560X_REG_XOUT1               0x01    // X output [9:2]
#define MMC560X_REG_YOUT0               0x02    // Y output [17:10]
#define MMC560X_REG_YOUT1               0x03    // Y output [9:2]
#define MMC560X_REG_ZOUT0               0x04    // Z output [17:10]
#define MMC560X_REG_ZOUT1               0x05    // Z output [9:2]
#define MMC560X_REG_XOUT2               0x06    // X output [1:0]
#define MMC560X_REG_YOUT2               0x07    // Y output [1:0]
#define MMC560X_REG_ZOUT2               0x08    // Z output [1:0]
#define MMC560X_REG_TOUT                0x09    // Temperature output
#define MMC560X_REG_STATUS1             0x18    // Status register 1
#define MMC560X_REG_ODR                 0x1A    // Output data rate
#define MMC560X_REG_CTRL0               0x1B    // Control register 0
#define MMC560X_REG_CTRL1               0x1C    // Control register 1
#define MMC560X_REG_CTRL2               0x1D    // Control register 2
#define MMC560X_REG_ST_X_TH             0x1E    // Self-test X threshold
#define MMC560X_REG_ST_Y_TH             0x1F    // Self-test Y threshold
#define MMC560X_REG_ST_Z_TH             0x20    // Self-test Z threshold
#define MMC560X_REG_ST_X                0x27    // Self-test X value
#define MMC560X_REG_ST_Y                0x28    // Self-test Y value
#define MMC560X_REG_ST_Z                0x29    // Self-test Z value
#define MMC560X_REG_PRODUCT_ID          0x39    // Product ID register

// Product ID value
#define MMC560X_PRODUCT_ID              0x10

// STATUS1 register bits
#define MMC560X_STATUS1_MEAS_M_DONE     0x01    // Measurement done
#define MMC560X_STATUS1_MEAS_T_DONE     0x02    // Temperature measurement done
#define MMC560X_STATUS1_SAT_SENSOR      0x20    // Sensor saturated
#define MMC560X_STATUS1_OTP_READ_DONE   0x10    // OTP read done

// CTRL0 register bits
#define MMC560X_CTRL0_TM_M              0x01    // Take measurement (magnetic)
#define MMC560X_CTRL0_TM_T              0x02    // Take measurement (temperature)
#define MMC560X_CTRL0_INT_DONE_EN       0x04    // Interrupt on measurement done
#define MMC560X_CTRL0_SET               0x08    // SET operation
#define MMC560X_CTRL0_RESET             0x10    // RESET operation
#define MMC560X_CTRL0_AUTO_ST_EN        0x20    // Auto self-test enable
#define MMC560X_CTRL0_AUTO_SR_EN        0x40    // Auto SET/RESET enable
#define MMC560X_CTRL0_CMM_FREQ_EN       0x80    // Continuous mode frequency enable

// CTRL1 register bits
#define MMC560X_CTRL1_BW0               0x01    // Bandwidth bit 0
#define MMC560X_CTRL1_BW1               0x02    // Bandwidth bit 1
#define MMC560X_CTRL1_X_INHIBIT         0x04    // X channel inhibit
#define MMC560X_CTRL1_Y_INHIBIT         0x08    // Y channel inhibit
#define MMC560X_CTRL1_Z_INHIBIT         0x10    // Z channel inhibit
#define MMC560X_CTRL1_ST_ENP            0x20    // Self-test positive
#define MMC560X_CTRL1_ST_ENM            0x40    // Self-test negative
#define MMC560X_CTRL1_SW_RST            0x80    // Software reset

// CTRL2 register bits
#define MMC560X_CTRL2_PRD_SET_0         0x01    // Periodic SET bit 0
#define MMC560X_CTRL2_PRD_SET_1         0x02    // Periodic SET bit 1
#define MMC560X_CTRL2_PRD_SET_2         0x04    // Periodic SET bit 2
#define MMC560X_CTRL2_EN_PRD_SET        0x08    // Enable periodic SET
#define MMC560X_CTRL2_CMM_EN            0x10    // Continuous mode enable
#define MMC560X_CTRL2_INT_MDT_EN        0x20    // Interrupt on motion detection threshold
#define MMC560X_CTRL2_INT_MDT           0x40    // Motion detection threshold triggered
#define MMC560X_CTRL2_HPOWER            0x80    // High power mode (lower noise)

// Bandwidth settings (CTRL1 bits 1:0)
#define MMC560X_BW_6_6MS                0x00    // 6.6ms measurement time
#define MMC560X_BW_3_5MS                0x01    // 3.5ms measurement time
#define MMC560X_BW_2_0MS                0x02    // 2.0ms measurement time
#define MMC560X_BW_1_2MS                0x03    // 1.2ms measurement time

// Output data rates (when CMM_FREQ_EN = 1)
#define MMC560X_ODR_OFF                 0x00    // ODR off
#define MMC560X_ODR_1HZ                 0x01
#define MMC560X_ODR_5HZ                 0x05
#define MMC560X_ODR_10HZ                0x0A
#define MMC560X_ODR_20HZ                0x14
#define MMC560X_ODR_50HZ                0x32
#define MMC560X_ODR_100HZ               0x64
#define MMC560X_ODR_200HZ               0xC8
#define MMC560X_ODR_255HZ               0xFF

// Data scaling
// 18-bit mode: 1 LSB = 0.0625 mG (16384 counts/Gauss, ±32 Gauss range)
// Full scale: 2^18 = 262144
// Zero field output = 131072 (2^17)
#define MMC560X_ZERO_FIELD              131072
#define MMC560X_COUNTS_PER_GAUSS        16384.0f

#ifdef USE_MAG_MMC560X_ASYNC_READ
static uint8_t m_read_buf[9];
static bool m_read_in_progress = false;
#endif

/**
 * @brief Initialize the MMC560X magnetometer.
 *
 * Performs software reset, SET operation for offset elimination, and configures
 * the sensor for continuous measurement mode at 100Hz with auto SET/RESET.
 *
 * @param magDev Pointer to the magnetometer device structure.
 * @return true if initialization succeeded, false otherwise.
 */
static bool mmc560xInit(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;
    bool ack;

    busDeviceRegister(dev);

    // Software reset
    ack = busWriteRegister(dev, MMC560X_REG_CTRL1, MMC560X_CTRL1_SW_RST);
    if (!ack) {
        return false;
    }
    delay(20);  // Wait for reset to complete

    // Perform SET operation to eliminate offset
    ack = busWriteRegister(dev, MMC560X_REG_CTRL0, MMC560X_CTRL0_SET);
    if (!ack) {
        return false;
    }
    delay(1);

    // Configure for continuous measurement mode
    // Set bandwidth to 2.0ms measurement time (good balance of speed/noise)
    ack = busWriteRegister(dev, MMC560X_REG_CTRL1, MMC560X_BW_2_0MS);
    if (!ack) {
        return false;
    }

    // Set ODR to 100Hz
    ack = busWriteRegister(dev, MMC560X_REG_ODR, MMC560X_ODR_100HZ);
    if (!ack) {
        return false;
    }

    // Per datasheet: ODR → CMM_FREQ_EN (CTRL0) → CMM_EN (CTRL2)
    // Enable CMM frequency first (continuous mode uses ODR register)
    ack = busWriteRegister(dev, MMC560X_REG_CTRL0, MMC560X_CTRL0_CMM_FREQ_EN | MMC560X_CTRL0_AUTO_SR_EN);
    if (!ack) {
        return false;
    }

    // Then enable continuous mode with automatic SET/RESET for temperature compensation
    // Use HPOWER mode for lower noise
    uint8_t ctrl2 = MMC560X_CTRL2_CMM_EN | MMC560X_CTRL2_HPOWER | MMC560X_CTRL2_EN_PRD_SET;
    ack = busWriteRegister(dev, MMC560X_REG_CTRL2, ctrl2);
    if (!ack) {
        return false;
    }

#ifdef USE_MAG_MMC560X_ASYNC_READ
    DBG("MMC560X Async Read: Enabled");
#else
    DBG("MMC560X Async Read: Disabled");
#endif

    magDev->magOdrHz = 100;
    return true;
}

/**
 * @brief Read magnetic field data from the MMC560X sensor.
 *
 * Reads all 9 bytes of 18-bit magnetic data and converts to signed 16-bit values.
 * In continuous mode, data is always available at the configured ODR (100Hz).
 *
 * @param magDev  Pointer to the magnetometer device structure.
 * @param magData Array to store X, Y, Z magnetic field readings (16-bit signed).
 * @return true if read succeeded, false otherwise.
 */
static bool mmc560xRead(magDev_t *magDev, int16_t *magData)
{
#ifdef USE_MAG_MMC560X_ASYNC_READ
    uint8_t *buf = m_read_buf;
#else
    uint8_t buf[9];  // 9 bytes for full 18-bit data
#endif
    extDevice_t *dev = &magDev->dev;

    // In continuous mode at 100Hz, data should always be available
    // Just read the data directly without checking status

#ifdef USE_MAG_MMC560X_ASYNC_READ
    // Using sync i2c read takes around 160us to complete, which can cause LOAD issue to 
    // disable ARMING.
    //      MMC5603 data size: 9 bytes (3 bytes per axis)
    //      Extra bytes: 2 ~ 3 bytes (status register, control register, etc.)
    //      800KHz I2C bus → 1 bit takes 1.25us → 11-12 bytes takes around 108us (1.25 * 8 * 11-12)
    //      Considering ACK: 108us * 1.25 (25% overhead for ACK) = 135us
    //      Other overheads: 140 ~ 180us (e.g. i2c start, driver overhead, register handling, etc)
    // 
    // But, the remaining cpu cycles after Gyro/PID execution is around 77us (in max), that means
    // Mag task update is always treated as "LATE" task because 160us > 77us, which causes LOAD issue to 
    // disable ARMING. That's why we want to use async read to start the i2c read and return immediately 
    // without waiting for the read to complete, then check the read completion in the next Mag task update. 
    // This way, we can avoid blocking i2c read and reduce the Mag task execution time, which should 
    // help to avoid LOAD issue on PICO.
    // 
    // The feature has one concern: In barometer_bmp5xx.c, the implementation has one comment:
    //
    //     `Use synchronous read for I2C - async reads can get stuck on PICO`
    //
    // sensors/barometer.c does not have such issue even it uses sync i2c read. For example, in barometer_bmp5xx.c, 
    // it uses synchronous read to read 6 bytes of data that takes around 120us, but it does not cause LOAD issue.
    // The reason is because barometer separates `i2c read data` and `data calculation` into two times of task update
    // function call. So, even the `i2c read data` takes around 120us, the next task update function call for `data calculation` 
    // will take very short time because it only does data parsing and calculation without i2c read.
    // So, only half times of task update function call takes long time (120us), that will not cause LOAD issue.

    // [TODO] Need to verify if async read can work reliably on PICO for MMC560X. If it can work 
    //        reliably, then we can use async read to avoid blocking I2C read which may cause 
    //        issues on PICO. If async read cannot work reliably, then we should fall back to 
    //        synchronous read to ensure data can be read correctly from the sensor.
    // 
    // In compass.c, it already checks if the bus is busy before calling read, so we 
    // can attempt an async read here without additional checks
    if (m_read_in_progress) {
        // Previous async read is still in progress, checks bus is busy to decide 
        // the previous read has completed or not. If bus is busy, it means the previous read 
        // is still in progress, so we should not start a new read and just return false to indicate 
        // no new data is available yet.
        if (busBusy(dev, NULL)) {
            return false; // Previous read still in progress, no new data available yet
        }
        else {
            // Previous read has completed, process the data
            m_read_in_progress = false;
        }
    }
    else {
        if (busReadRegisterBufferStart(dev, MMC560X_REG_XOUT0, buf, 9)) {
            // Successfully started async read, mark it as in progress and return false to indicate data is not ready yet
            m_read_in_progress = true;
            return false; // Async read started, data will be available on next call
        }
        else {
            return false; // Failed to start async read
        }
    }
#else
    // Read all 9 bytes of magnetic data (18-bit resolution)
    // Registers 0x00-0x08: X[17:10], X[9:2], Y[17:10], Y[9:2], Z[17:10], Z[9:2], X[1:0], Y[1:0], Z[1:0]
    if (!busReadRegisterBuffer(dev, MMC560X_REG_XOUT0, buf, 9)) {
        return false;
    }
#endif /* USE_MAG_MMC560X_ASYNC_READ */

    // Reconstruct 18-bit values
    // High part: [17:10] from OUT0, [9:2] from OUT1, [1:0] from OUT2
    uint32_t rawX = ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | ((buf[6] >> 6) & 0x03);
    uint32_t rawY = ((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2) | ((buf[7] >> 6) & 0x03);
    uint32_t rawZ = ((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2) | ((buf[8] >> 6) & 0x03);

    // Convert to signed values by subtracting zero offset
    // Full scale is 0 to 262143, with 131072 being zero field
    int32_t x = (int32_t)rawX - MMC560X_ZERO_FIELD;
    int32_t y = (int32_t)rawY - MMC560X_ZERO_FIELD;
    int32_t z = (int32_t)rawZ - MMC560X_ZERO_FIELD;

    // Scale down to 16-bit range for compatibility
    // 18-bit signed value (-131072 to +131071) -> 16-bit signed (-32768 to +32767)
    // Divide by 4 (shift right by 2)
    magData[X] = (int16_t)(x >> 2);
    magData[Y] = (int16_t)(y >> 2);
    magData[Z] = (int16_t)(z >> 2);

    return true;
}

/**
 * @brief Detect and configure the MMC560X magnetometer.
 *
 * Reads the product ID register to verify the sensor is present, then sets up
 * the device callbacks for initialization and reading.
 *
 * @param magDev Pointer to the magnetometer device structure.
 * @return true if MMC560X was detected, false otherwise.
 */
bool mmc560xDetect(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    // Set I2C address if not already configured
    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = MMC560X_I2C_ADDRESS;
    }

    // Read product ID
    uint8_t productId = 0;
    delay(10);  // Allow sensor to power up
    
    bool ack = busReadRegisterBuffer(dev, MMC560X_REG_PRODUCT_ID, &productId, 1);
    if (!ack || productId != MMC560X_PRODUCT_ID) {
        return false;
    }

    // Set up the device
    magDev->init = mmc560xInit;
    magDev->read = mmc560xRead;

    return true;
}

#endif // USE_MAG_MMC560X
