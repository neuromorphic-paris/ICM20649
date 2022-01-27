/**
 * @file
 * @brief Code to interface with the ICM20649 IMU via I2C.
 */

#ifndef ICM20649_HPP
#define ICM20649_HPP

#include <cstddef>
extern "C"
{
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
}
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace ICM20649
{
/**
 * @name Constants
 * @{
 */
#define G_TO_METER_PER_SECOND_SQUARED                                    \
  9.80664999999998 /**< Constant to convert from acceleration of gravity \
                    * \f$[G]\f$ to \f$[m/s^{2}]\f$                       \
                    */
#define DEGREE_TO_RADIAN                                            \
  M_PI / 180.0 /**< Constant to convert from  \f$[\text{deg}]\f$ to \
                  \f$[\text{rad}]\f$*/
#define RADIAN_TO_DEGREE                                           \
  180.0 / M_PI /**< Constant to convert from \f$[\text{rad}]\f$ to \
                  \f$[\text{deg}]\f$ */
/**@}*/

/**
 * @name ICM20649 register banks
 * @{
 */
#define ICM20649_BANK_0 (0 << 7) /**< Register bank 0 */
#define ICM20649_BANK_1 (1 << 7) /**< Register bank 1 */
#define ICM20649_BANK_2 (2 << 7) /**< Register bank 2 */
#define ICM20649_BANK_3 (3 << 7) /**< Register bank 3 */
#define ICM20649_I2C_ADDR 0x68   /**< Default I2C address */
/**@}*/

/**
 * @name Register and associated bit definitions
 * @{
 */
/* Bank 0 register map */

#define ICM20649_REG_WHO_AM_I \
  (ICM20649_BANK_0 | 0x00) /**< Device ID register */

#define ICM20649_REG_USER_CTRL \
  (ICM20649_BANK_0 | 0x03)                /**< User control register */
#define ICM20649_BIT_DMP_EN 0x80          /**< DMP enable bit */
#define ICM20649_BIT_FIFO_EN 0x40         /**< FIFO enable bit */
#define ICM20649_BIT_I2C_MST_EN 0x20      /**< I2C master I/F enable bit */
#define ICM20649_BIT_I2C_IF_DIS 0x10      /**< Disable I2C, enable SPI bit */
#define ICM20649_BIT_DMP_RST 0x08         /**< DMP module reset bit */
#define ICM20649_BIT_DIAMOND_DMP_RST 0x04 /**< SRAM module reset bit */

#define ICM20649_REG_LP_CONFIG \
  (ICM20649_BANK_0 | 0x05)              /**< Low Power mode config register */
#define ICM20649_BIT_I2C_MST_CYCLE 0x40 /**< I2C master cycle mode enable */
#define ICM20649_BIT_ACCEL_CYCLE 0x20   /**< Accelerometer cycle mode enable */
#define ICM20649_BIT_GYRO_CYCLE 0x10    /**< Gyroscope cycle mode enable */

#define ICM20649_REG_PWR_MGMT_1 \
  (ICM20649_BANK_0 | 0x06)         /**< Power Management 1 register */
#define ICM20649_BIT_H_RESET 0x80  /**< Device reset bit */
#define ICM20649_BIT_SLEEP 0x40    /**< Sleep mode enable bit */
#define ICM20649_BIT_LP_EN 0x20    /**< Low Power feature enable bit */
#define ICM20649_BIT_TEMP_DIS 0x08 /**< Temperature sensor disable bit */
#define ICM20649_BIT_CLK_PLL 0x01  /**< Auto clock source selection setting */

#define ICM20649_REG_PWR_MGMT_2 \
  (ICM20649_BANK_0 | 0x07)               /**< Power Management 2 register */
#define ICM20649_BIT_PWR_ACCEL_STBY 0x38 /**< Disable accelerometer */
#define ICM20649_BIT_PWR_GYRO_STBY 0x07  /**< Disable gyroscope */
#define ICM20649_BIT_PWR_ALL_OFF 0x7F    /**< Disable both accel and gyro */

#define ICM20649_REG_INT_PIN_CFG \
  (ICM20649_BANK_0 | 0x0F)         /**< Interrupt Pin Configuration register */
#define ICM20649_BIT_INT_ACTL 0x80 /**< Active low setting bit */
#define ICM20649_BIT_INT_OPEN 0x40 /**< Open collector onfiguration bit */
#define ICM20649_BIT_INT_LATCH_EN 0x20 /**< Latch enable bit */

#define ICM20649_REG_INT_ENABLE \
  (ICM20649_BANK_0 | 0x10)           /**< Interrupt Enable register */
#define ICM20649_BIT_WOM_INT_EN 0x08 /**< Wake-up On Motion enable bit */

#define ICM20649_REG_INT_ENABLE_1 \
  (ICM20649_BANK_0 | 0x11) /**< Interrupt Enable 1 register */
#define ICM20649_BIT_RAW_DATA_0_RDY_EN \
  0x01 /**< Raw data ready interrupt enable bit */

#define ICM20649_REG_INT_ENABLE_2 \
  (ICM20649_BANK_0 | 0x12) /**< Interrupt Enable 2 register */
#define ICM20649_BIT_FIFO_OVERFLOW_EN_0 \
  0x01 /**< FIFO overflow interrupt enable bit */

#define ICM20649_REG_INT_ENABLE_3 \
  (ICM20649_BANK_0 | 0x13) /**< Interrupt Enable 2 register */

#define ICM20649_REG_INT_STATUS \
  (ICM20649_BANK_0 | 0x19) /**< Interrupt Status register */
#define ICM20649_BIT_WOM_INT \
  0x08 /**< Wake-up on motion interrupt occured bit */
#define ICM20649_BIT_PLL_RDY 0x04 /**< PLL ready interrupt occured bit */

#define ICM20649_REG_INT_STATUS_1 \
  (ICM20649_BANK_0 | 0x1A) /**< Interrupt Status 1 register */
#define ICM20649_BIT_RAW_DATA_0_RDY_INT \
  0x01 /**< Raw data ready interrupt occured bit */

#define ICM20649_REG_INT_STATUS_2 \
  (ICM20649_BANK_0 | 0x1B) /**< Interrupt Status 2 register */

#define ICM20649_REG_ACCEL_XOUT_H_SH \
  (ICM20649_BANK_0 | 0x2D) /**< Accelerometer X-axis data high byte */
#define ICM20649_REG_ACCEL_XOUT_L_SH \
  (ICM20649_BANK_0 | 0x2E) /**< Accelerometer X-axis data low byte */
#define ICM20649_REG_ACCEL_YOUT_H_SH \
  (ICM20649_BANK_0 | 0x2F) /**< Accelerometer Y-axis data high byte */
#define ICM20649_REG_ACCEL_YOUT_L_SH \
  (ICM20649_BANK_0 | 0x30) /**< Accelerometer Y-axis data low byte */
#define ICM20649_REG_ACCEL_ZOUT_H_SH \
  (ICM20649_BANK_0 | 0x31) /**< Accelerometer Z-axis data high byte */
#define ICM20649_REG_ACCEL_ZOUT_L_SH \
  (ICM20649_BANK_0 | 0x32) /**< Accelerometer Z-axis data low byte */

#define ICM20649_REG_GYRO_XOUT_H_SH \
  (ICM20649_BANK_0 | 0x33) /**< Gyroscope X-axis data high byte */
#define ICM20649_REG_GYRO_XOUT_L_SH \
  (ICM20649_BANK_0 | 0x34) /**< Gyroscope X-axis data low byte */
#define ICM20649_REG_GYRO_YOUT_H_SH \
  (ICM20649_BANK_0 | 0x35) /**< Gyroscope Y-axis data high byte */
#define ICM20649_REG_GYRO_YOUT_L_SH \
  (ICM20649_BANK_0 | 0x36) /**< Gyroscope Y-axis data low byte */
#define ICM20649_REG_GYRO_ZOUT_H_SH \
  (ICM20649_BANK_0 | 0x37) /**< Gyroscope Z-axis data high byte */
#define ICM20649_REG_GYRO_ZOUT_L_SH \
  (ICM20649_BANK_0 | 0x38) /**< Gyroscope Z-axis data low byte */

#define ICM20649_REG_EXT_SLV_SENS_DATA_00                                  \
  (ICM20649_BANK_0 | 0x3B) /** External sensor data low byte for I2C slave \
                              0**/

#define ICM20649_REG_TEMPERATURE_H \
  (ICM20649_BANK_0 | 0x39) /**< Temperature data high byte */
#define ICM20649_REG_TEMPERATURE_L \
  (ICM20649_BANK_0 | 0x3A) /**< Temperature data low byte */
#define ICM20649_REG_TEMP_CONFIG \
  (ICM20649_BANK_0 | 0x53) /**< Temperature Configuration register */

#define ICM20649_REG_FIFO_EN_1 \
  (ICM20649_BANK_0 | 0x66) /**< FIFO Enable 1 register */

#define ICM20649_REG_FIFO_EN_2 \
  (ICM20649_BANK_0 | 0x67) /**< FIFO Enable 2 register */
#define ICM20649_BIT_ACCEL_FIFO_EN \
  0x10 /**< Enable writing acceleration data to FIFO bit */
#define ICM20649_BITS_GYRO_FIFO_EN \
  0x0E /**< Enable writing gyroscope data to FIFO bit */

#define ICM20649_REG_FIFO_RST \
  (ICM20649_BANK_0 | 0x68) /**< FIFO Reset register */
#define ICM20649_REG_FIFO_MODE \
  (ICM20649_BANK_0 | 0x69) /**< FIFO Mode register */

#define ICM20649_REG_FIFO_COUNT_H \
  (ICM20649_BANK_0 | 0x70) /**< FIFO data count high byte */
#define ICM20649_REG_FIFO_COUNT_L \
  (ICM20649_BANK_0 | 0x71) /**< FIFO data count low byte */
#define ICM20649_REG_FIFO_R_W \
  (ICM20649_BANK_0 | 0x72) /**< FIFO Read/Write register */

#define ICM20649_REG_DATA_RDY_STATUS \
  (ICM20649_BANK_0 | 0x74)               /**< Data Ready Status register */
#define ICM20649_BIT_RAW_DATA_0_RDY 0x01 /**< Raw Data Ready bit */

#define ICM20649_REG_FIFO_CFG \
  (ICM20649_BANK_0 | 0x76) /**< FIFO Configuration register */
#define ICM20649_BIT_MULTI_FIFO_CFG \
  0x01 /**< Interrupt status for each sensor is required */
#define ICM20649_BIT_SINGLE_FIFO_CFG \
  0x00 /**< Interrupt status for only a single sensor is required */

/* Bank 1 register map */

#define ICM20649_REG_XA_OFFSET_H \
  (ICM20649_BANK_1 |             \
   0x14) /**< Acceleration sensor X-axis offset cancellation high byte */
#define ICM20649_REG_XA_OFFSET_L \
  (ICM20649_BANK_1 |             \
   0x15) /**< Acceleration sensor X-axis offset cancellation low byte */
#define ICM20649_REG_YA_OFFSET_H \
  (ICM20649_BANK_1 |             \
   0x17) /**< Acceleration sensor Y-axis offset cancellation high byte */
#define ICM20649_REG_YA_OFFSET_L \
  (ICM20649_BANK_1 |             \
   0x18) /**< Acceleration sensor Y-axis offset cancellation low byte */
#define ICM20649_REG_ZA_OFFSET_H \
  (ICM20649_BANK_1 |             \
   0x1A) /**< Acceleration sensor Z-axis offset cancellation high byte */
#define ICM20649_REG_ZA_OFFSET_L \
  (ICM20649_BANK_1 |             \
   0x1B) /**< Acceleration sensor Z-axis offset cancellation low byte */

#define ICM20649_REG_TIMEBASE_CORR_PLL \
  (ICM20649_BANK_1 | 0x28) /**< PLL Timebase Correction register */

/* Bank 2 register map */

#define ICM20649_REG_GYRO_SMPLRT_DIV \
  (ICM20649_BANK_2 | 0x00) /**< Gyroscope Sample Rate Divider regiser */

#define ICM20649_REG_GYRO_CONFIG_1 \
  (ICM20649_BANK_2 | 0x01) /**< Gyroscope Configuration 1 register */
#define ICM20649_BIT_GYRO_FCHOICE \
  0x01 /**< Gyro Digital Low-Pass Filter enable bit */
#define ICM20649_SHIFT_GYRO_FS_SEL 1 /**< Gyro Full Scale Select bit shift */
#define ICM20649_SHIFT_GYRO_DLPCFG 3 /**< Gyro DLPF Config bit shift */
#define ICM20649_MASK_GYRO_FULLSCALE                                   \
  0x06                             /**< Gyro Full Scale Select bitmask \
                                    */
#define ICM20649_MASK_GYRO_BW 0x39 /**< Gyro Bandwidth Select bitmask */
#define ICM20649_GYRO_FULLSCALE_500DPS \
  (0x00 << ICM20649_SHIFT_GYRO_FS_SEL) /**< Gyro Full Scale = 500 deg/sec */
#define ICM20649_GYRO_FULLSCALE_1000DPS \
  (0x01 << ICM20649_SHIFT_GYRO_FS_SEL) /**< Gyro Full Scale = 1000 deg/sec */
#define ICM20649_GYRO_FULLSCALE_2000DPS \
  (0x02 << ICM20649_SHIFT_GYRO_FS_SEL) /**< Gyro Full Scale = 2000 deg/sec */
#define ICM20649_GYRO_FULLSCALE_4000DPS \
  (0x03 << ICM20649_SHIFT_GYRO_FS_SEL) /**< Gyro Full Scale = 4000 deg/sec */
#define ICM20649_GYRO_BW_12100HZ \
  (0x00 << ICM20649_SHIFT_GYRO_DLPCFG) /**< Gyro Bandwidth = 12100 Hz */
#define ICM20649_GYRO_BW_360HZ            \
  ((0x07 << ICM20649_SHIFT_GYRO_DLPCFG) | \
   ICM20649_BIT_GYRO_FCHOICE) /**< Gyro Bandwidth = 360 Hz */
#define ICM20649_GYRO_BW_200HZ            \
  ((0x00 << ICM20649_SHIFT_GYRO_DLPCFG) | \
   ICM20649_BIT_GYRO_FCHOICE) /**< Gyro Bandwidth = 200 Hz */
#define ICM20649_GYRO_BW_150HZ            \
  ((0x01 << ICM20649_SHIFT_GYRO_DLPCFG) | \
   ICM20649_BIT_GYRO_FCHOICE) /**< Gyro Bandwidth = 150 Hz */
#define ICM20649_GYRO_BW_120HZ            \
  ((0x02 << ICM20649_SHIFT_GYRO_DLPCFG) | \
   ICM20649_BIT_GYRO_FCHOICE) /**< Gyro Bandwidth = 120 Hz */
#define ICM20649_GYRO_BW_51HZ             \
  ((0x03 << ICM20649_SHIFT_GYRO_DLPCFG) | \
   ICM20649_BIT_GYRO_FCHOICE) /**< Gyro Bandwidth = 51 Hz */
#define ICM20649_GYRO_BW_24HZ             \
  ((0x04 << ICM20649_SHIFT_GYRO_DLPCFG) | \
   ICM20649_BIT_GYRO_FCHOICE) /**< Gyro Bandwidth = 24 Hz */
#define ICM20649_GYRO_BW_12HZ             \
  ((0x05 << ICM20649_SHIFT_GYRO_DLPCFG) | \
   ICM20649_BIT_GYRO_FCHOICE) /**< Gyro Bandwidth = 12 Hz */
#define ICM20649_GYRO_BW_6HZ              \
  ((0x06 << ICM20649_SHIFT_GYRO_DLPCFG) | \
   ICM20649_BIT_GYRO_FCHOICE) /**< Gyro Bandwidth = 6 Hz */

#define ICM20649_REG_GYRO_CONFIG_2 \
  (ICM20649_BANK_2 | 0x02)          /**< Gyroscope Configuration 2 register */
#define ICM20649_BIT_GYRO_CTEN 0x38 /**< Gyroscope Self-Test Enable bits */

#define ICM20649_REG_XG_OFFS_USRH \
  (ICM20649_BANK_2 |              \
   0x03) /**< Gyroscope sensor X-axis offset cancellation high byte */
#define ICM20649_REG_XG_OFFS_USRL \
  (ICM20649_BANK_2 |              \
   0x04) /**< Gyroscope sensor X-axis offset cancellation low byte */
#define ICM20649_REG_YG_OFFS_USRH \
  (ICM20649_BANK_2 |              \
   0x05) /**< Gyroscope sensor Y-axis offset cancellation high byte */
#define ICM20649_REG_YG_OFFS_USRL \
  (ICM20649_BANK_2 |              \
   0x06) /**< Gyroscope sensor Y-axis offset cancellation low byte */
#define ICM20649_REG_ZG_OFFS_USRH \
  (ICM20649_BANK_2 |              \
   0x07) /**< Gyroscope sensor Z-axis offset cancellation high byte */
#define ICM20649_REG_ZG_OFFS_USRL \
  (ICM20649_BANK_2 |              \
   0x08) /**< Gyroscope sensor Z-axis offset cancellation low byte */

#define ICM20649_REG_ODR_ALIGN_EN \
  (ICM20649_BANK_2 | 0x09) /**< Output Data Rate start time alignment */

#define ICM20649_REG_ACCEL_SMPLRT_DIV_1 \
  (ICM20649_BANK_2 |                    \
   0x10) /**< Acceleration Sensor Sample Rate Divider 1 register */
#define ICM20649_REG_ACCEL_SMPLRT_DIV_2 \
  (ICM20649_BANK_2 |                    \
   0x11) /**< Acceleration Sensor Sample Rate Divider 2 register */

#define ICM20649_REG_ACCEL_INTEL_CTRL \
  (ICM20649_BANK_2 |                  \
   0x12) /**< Accelerometer Hardware Intelligence Control register */
#define ICM20649_BIT_ACCEL_INTEL_EN 0x02   /**< Wake-up On Motion enable bit */
#define ICM20649_BIT_ACCEL_INTEL_MODE 0x01 /**< WOM algorithm selection bit */

#define ICM20649_REG_ACCEL_WOM_THR \
  (ICM20649_BANK_2 | 0x13) /**< Wake-up On Motion Threshold register */

#define ICM20649_REG_ACCEL_CONFIG \
  (ICM20649_BANK_2 | 0x14) /**< Accelerometer Configuration register */
#define ICM20649_BIT_ACCEL_FCHOICE \
  0x01 /**< Accel Digital Low-Pass Filter enable bit */
#define ICM20649_SHIFT_ACCEL_FS 1     /**< Accel Full Scale Select bit shift */
#define ICM20649_SHIFT_ACCEL_DLPCFG 3 /**< Accel DLPF Config bit shift */
#define ICM20649_MASK_ACCEL_FULLSCALE \
  0x06                              /**< Accel Full Scale Select bitmask */
#define ICM20649_MASK_ACCEL_BW 0x39 /**< Accel Bandwidth Select bitmask */
#define ICM20649_ACCEL_FULLSCALE_2G \
  (0x00 << ICM20649_SHIFT_ACCEL_FS) /**< Accel Full Scale = 2 g */
#define ICM20649_ACCEL_FULLSCALE_4G \
  (0x01 << ICM20649_SHIFT_ACCEL_FS) /**< Accel Full Scale = 4 g */
#define ICM20649_ACCEL_FULLSCALE_8G \
  (0x02 << ICM20649_SHIFT_ACCEL_FS) /**< Accel Full Scale = 8 g */
#define ICM20649_ACCEL_FULLSCALE_16G \
  (0x03 << ICM20649_SHIFT_ACCEL_FS) /**< Accel Full Scale = 16 g */
#define ICM20649_ACCEL_BW_1210HZ \
  (0x00 << ICM20649_SHIFT_ACCEL_DLPCFG) /**< Accel Bandwidth = 1210 Hz */
#define ICM20649_ACCEL_BW_470HZ            \
  ((0x07 << ICM20649_SHIFT_ACCEL_DLPCFG) | \
   ICM20649_BIT_ACCEL_FCHOICE) /**< Accel Bandwidth = 470 Hz */
#define ICM20649_ACCEL_BW_246HZ            \
  ((0x00 << ICM20649_SHIFT_ACCEL_DLPCFG) | \
   ICM20649_BIT_ACCEL_FCHOICE) /**< Accel Bandwidth = 246 Hz */
#define ICM20649_ACCEL_BW_111HZ            \
  ((0x02 << ICM20649_SHIFT_ACCEL_DLPCFG) | \
   ICM20649_BIT_ACCEL_FCHOICE) /**< Accel Bandwidth = 111 Hz */
#define ICM20649_ACCEL_BW_50HZ             \
  ((0x03 << ICM20649_SHIFT_ACCEL_DLPCFG) | \
   ICM20649_BIT_ACCEL_FCHOICE) /**< Accel Bandwidth = 50 Hz */
#define ICM20649_ACCEL_BW_24HZ             \
  ((0x04 << ICM20649_SHIFT_ACCEL_DLPCFG) | \
   ICM20649_BIT_ACCEL_FCHOICE) /**< Accel Bandwidth = 24 Hz */
#define ICM20649_ACCEL_BW_12HZ             \
  ((0x05 << ICM20649_SHIFT_ACCEL_DLPCFG) | \
   ICM20649_BIT_ACCEL_FCHOICE) /**< Accel Bandwidth = 12 Hz */
#define ICM20649_ACCEL_BW_6HZ              \
  ((0x06 << ICM20649_SHIFT_ACCEL_DLPCFG) | \
   ICM20649_BIT_ACCEL_FCHOICE) /**< Accel Bandwidth = 6 Hz */

#define ICM20649_REG_ACCEL_CONFIG_2 \
  (ICM20649_BANK_2 | 0x15) /**< Accelerometer Configuration 2 register */
#define ICM20649_BIT_ACCEL_CTEN                 \
  0x1C /**< Accelerometer Self-Test Enable bits \
        */

/* Bank 3 register map */

#define ICM20649_REG_I2C_MST_ODR_CONFIG \
  (ICM20649_BANK_3 |                    \
   0x00) /**< I2C Master Output Data Rate Configuration register */

#define ICM20649_REG_I2C_MST_CTRL \
  (ICM20649_BANK_3 | 0x01) /**< I2C Master Control register */
#define ICM20649_BIT_I2C_MST_P_NSR          \
  0x10 /**< Stop between reads enabling bit \
        */

#define ICM20649_REG_I2C_MST_DELAY_CTRL \
  (ICM20649_BANK_3 | 0x02)            /**< I2C Master Delay Control register */
#define ICM20649_BIT_SLV0_DLY_EN 0x01 /**< I2C Slave0 Delay Enable bit */
#define ICM20649_BIT_SLV1_DLY_EN 0x02 /**< I2C Slave1 Delay Enable bit */
#define ICM20649_BIT_SLV2_DLY_EN 0x04 /**< I2C Slave2 Delay Enable bit */
#define ICM20649_BIT_SLV3_DLY_EN 0x08 /**< I2C Slave3 Delay Enable bit */

#define ICM20649_REG_I2C_SLV0_ADDR \
  (ICM20649_BANK_3 | 0x03) /**< I2C Slave0 Physical Address register */
#define ICM20649_REG_I2C_SLV0_REG \
  (ICM20649_BANK_3 | 0x04) /**< I2C Slave0 Register Address register */
#define ICM20649_REG_I2C_SLV0_CTRL \
  (ICM20649_BANK_3 | 0x05) /**< I2C Slave0 Control register */
#define ICM20649_REG_I2C_SLV0_DO \
  (ICM20649_BANK_3 | 0x06) /**< I2C Slave0 Data Out register */

#define ICM20649_REG_I2C_SLV1_ADDR \
  (ICM20649_BANK_3 | 0x07) /**< I2C Slave1 Physical Address register */
#define ICM20649_REG_I2C_SLV1_REG \
  (ICM20649_BANK_3 | 0x08) /**< I2C Slave1 Register Address register */
#define ICM20649_REG_I2C_SLV1_CTRL \
  (ICM20649_BANK_3 | 0x09) /**< I2C Slave1 Control register */
#define ICM20649_REG_I2C_SLV1_DO \
  (ICM20649_BANK_3 | 0x0A) /**< I2C Slave1 Data Out register */

#define ICM20649_REG_I2C_SLV2_ADDR \
  (ICM20649_BANK_3 | 0x0B) /**< I2C Slave2 Physical Address register */
#define ICM20649_REG_I2C_SLV2_REG \
  (ICM20649_BANK_3 | 0x0C) /**< I2C Slave2 Register Address register */
#define ICM20649_REG_I2C_SLV2_CTRL \
  (ICM20649_BANK_3 | 0x0D) /**< I2C Slave2 Control register */
#define ICM20649_REG_I2C_SLV2_DO \
  (ICM20649_BANK_3 | 0x0E) /**< I2C Slave2 Data Out register */

#define ICM20649_REG_I2C_SLV3_ADDR \
  (ICM20649_BANK_3 | 0x0F) /**< I2C Slave3 Physical Address register */
#define ICM20649_REG_I2C_SLV3_REG \
  (ICM20649_BANK_3 | 0x10) /**< I2C Slave3 Register Address register */
#define ICM20649_REG_I2C_SLV3_CTRL \
  (ICM20649_BANK_3 | 0x11) /**< I2C Slave3 Control register */
#define ICM20649_REG_I2C_SLV3_DO \
  (ICM20649_BANK_3 | 0x12) /**< I2C Slave3 Data Out register */

#define ICM20649_REG_I2C_SLV4_ADDR \
  (ICM20649_BANK_3 | 0x13) /**< I2C Slave4 Physical Address register */
#define ICM20649_REG_I2C_SLV4_REG \
  (ICM20649_BANK_3 | 0x14) /**< I2C Slave4 Register Address register */
#define ICM20649_REG_I2C_SLV4_CTRL \
  (ICM20649_BANK_3 | 0x15) /**< I2C Slave4 Control register */
#define ICM20649_REG_I2C_SLV4_DO \
  (ICM20649_BANK_3 | 0x16) /**< I2C Slave4 Data Out register */
#define ICM20649_REG_I2C_SLV4_DI \
  (ICM20649_BANK_3 | 0x17) /**< I2C Slave4 Data In register */

#define ICM20649_BIT_I2C_SLV_EN 0x80  /**< I2C Slave Enable bit */
#define ICM20649_BIT_I2C_BYTE_SW 0x40 /**< I2C Slave Byte Swap enable bit */
#define ICM20649_BIT_I2C_REG_DIS \
  0x20 /**< I2C Slave Do Not Write Register Value bit */
#define ICM20649_BIT_I2C_GRP 0x10  /**< I2C Slave Group bit */
#define ICM20649_BIT_I2C_READ 0x80 /**< I2C Slave R/W bit */

/* Register common for all banks */
#define ICM20649_REG_BANK_SEL 0x7F /**< Bank Select register */

#define ICM20649_DEVICE_ID 0xE1 /**< ICM20649 Device ID value */
                                /**@}*/

/**
 * @brief Throws a runtime error trying to read from a register.
 */
class error_reading_register : public std::runtime_error
{
 public:
  error_reading_register(const uint16_t reg)
      : std::runtime_error("Could not read register '" + std::to_string(reg) +
                           "'. " + std::string(strerror(errno)))
  {
  }
};

/**
 * @brief Throws a runtime error trying to write to a register.
 */
class error_writing_register : public std::runtime_error
{
 public:
  error_writing_register(const uint16_t reg)
      : std::runtime_error("Could not write register '" + std::to_string(reg) +
                           "'. " + std::string(strerror(errno)))
  {
  }
};

/**
 * @brief Throws a runtime error trying to validate a device ID.
 */
class invalid_device_id : public std::runtime_error
{
 public:
  invalid_device_id(const uint8_t id_read, const uint8_t id)
      : std::runtime_error("Invalid device ID '" + std::to_string(id_read) +
                           "'. It should be '" + std::to_string(id) + "'.")
  {
  }
};

/**
 * @brief Throws a runtime error trying to open and read from a device.
 */
class unreadable_device : public std::runtime_error
{
 public:
  unreadable_device(const uint8_t bus, const uint8_t address)
      : std::runtime_error("Could not open the device on the bus '" +
                           std::to_string(bus) + "', address '" +
                           std::to_string(address) + "'. " +
                           std::string(strerror(errno)))
  {
  }
};

/**
 * @brief Throws a runtime error trying to open and read from a file.
 */
class unreadable_file : public std::runtime_error
{
 public:
  unreadable_file(const std::string& filename)
      : std::runtime_error("Could not open the file '" + filename +
                           "' for reading. " + std::string(strerror(errno)))
  {
  }
};

/**
 * @brief Configure a device.
 *
 * This struct stores the supported features to configure and initialize a
 * device.
 */
struct Config
{
  /**
   * @brief Sample rate (defaults to 1100\f$[\text{Hz}]\f$).
   */
  float sample_rate = 1100;

  /**
   * @brief Bandwidth of the accelerometer (defaults to 246\f$[\text{Hz}]\f$).
   * Use the ICM20649_ACCEL_BW_vHZ macros defined in this header file.
   * The value v can be 6, 12, 24, 50, 111, 246, 470 or 1210.
   */
  uint8_t accelerometer_bandwidth = ICM20649_ACCEL_BW_246HZ;
  /**
   * @brief Bandwidth of the gyroscope (defaults to 200\f$[\text{Hz}]\f$).
   * Use the ICM20649_GYRO_BW_vHZ macros defined in this header file.
   * The value v can be 6, 12, 24, 51, 120, 150, 200, 360 or 12100.
   */
  uint8_t gyroscope_bandwidth = ICM20649_GYRO_BW_200HZ;

  /**
   * @brief Full scale of the accelerometer (defaults to 2\f$[\text{G}]\f$).
   * Use the ICM20649_ACCEL_FULLSCALE_vG macros defined in this header file.
   * The value v can be 2, 4, 8, 16.
   */
  uint8_t accelerometer_scale = ICM20649_ACCEL_FULLSCALE_2G;
  /**
   * @brief Full scale of the gyroscope (defaults to
   * 500\f$[^{\circ}/\text{sec}]\f$). Use the ICM20649_GYRO_FULLSCALE_vDPS
   * macros defined in this header file. The value v can be 500, 1000, 2000 or
   * 4000.
   */
  uint8_t gyroscope_scale = ICM20649_GYRO_FULLSCALE_500DPS;

  /**
   * @brief Flag to enable the accelerometer (defaults to true).
   */
  bool accelerometer = true;
  /**
   * @brief Flag to enable the gyroscope (defaults to true).
   */
  bool gyroscope = true;
  /**
   * @brief Flag to enable the temperature sensor (defaults to true).
   */
  bool temperature = true;
};

class Device
{
 public:
  /**
   * @brief Constructor.
   */
  Device(const Config& config);
  /**
   * @brief Deleted copy constructor.
   */
  Device(const Device&) = delete;
  /**
   * @brief Default move constructor.
   */
  Device(Device&&) = default;
  /**
   * @brief Deleted copy assignment operator.
   */
  Device&
  operator=(const Device&) = delete;
  /**
   * @brief Default move assignment operator.
   */
  Device&
  operator=(Device&&) = default;
  /**
   * @brief Destructor.
   */
  ~Device();

  /**
   * @brief Perform an accelerometer measurement in \f$[m/s^{2}]\f$.
   *
   * @param[out] ax Accelerometer measurement on the X axis.
   * @param[out] ay Accelerometer measurement on the Y axis.
   * @param[out] ay Accelerometer measurement on the Z axis.
   */
  void
  accelerometer(float& ax, float& ay, float& az) const;

  /**
   * @brief Perform an accelerometer measurement in \f$[G]\f$.
   *
   * @param[out] ax Accelerometer measurement on the X axis.
   * @param[out] ay Accelerometer measurement on the Y axis.
   * @param[out] ay Accelerometer measurement on the Z axis.
   */
  void
  accelerometer_g(float& ax, float& ay, float& az) const;

  /**
   * @brief Sets the bandwidth of the accelerometer.
   *
   * @param bandwidth Accelerometer bandwidth value.
   * Use the ICM20649_ACCEL_BW_vHZ macros which are define in this header file.
   * The value of v can be 6, 12, 24, 50, 111, 246, 470 or 1210.
   */
  void
  accelerometer_bandwidth(const uint8_t bandwidth) const;

  /**
   * @brief Returns the accelerometer bit resolution.
   *
   * @return Accelerometer bit resolution.
   */
  float
  accelerometer_resolution() const;

  /**
   * @brief Sets the accelerometer resolution.
   *
   * @param scale_setting Full scale setting value.
   * Use the ICM20649_ACCEL_FULLSCALE_vG macros define in this header file.
   * The value v can be 2, 4, 8 or 16.
   */
  void
  accelerometer_resolution(const uint8_t scale_setting) const;

  /**
   * @brief Sets the accelerometer sample rate.
   *
   * @param rate Sample rate value (\f$[\text{Hz}]\f$).
   *
   * @return Actual sample rate, which may differ from the input value because
   * of the finite and discrete number of divider settings.
   */
  float
  accelerometer_sample_rate(const float rate) const;

  /**
   * @brief Perform a gyroscope measurement in \f$[\text{rad}/s]\f$.
   *
   * @param[out] wx Gyroscope measurement on the X axis.
   * @param[out] wy Gyroscope measurement on the Y axis.
   * @param[out] wy Gyroscope measurement on the Z axis.
   */
  void
  gyroscope(float& wx, float& wy, float& wz) const;

  /**
   * @brief Perform a gyroscope measurement in \f$[\text{deg}/s]\f$.
   *
   * @param[out] wx Gyroscope measurement on the X axis.
   * @param[out] wy Gyroscope measurement on the Y axis.
   * @param[out] wy Gyroscope measurement on the Z axis.
   */
  void
  gyroscope_d(float& wx, float& wy, float& wz) const;

  /**
   * @brief Sets the bandwidth of the gyroscope.
   *
   * @param bandwidth Gyroscope bandwidth value.
   * Use the ICM20649_GYRO_BW_vHZ macros which are define in this header file.
   * The value of v can be 6, 12, 24, 51, 120, 150, 200, 360 or 12100.
   */
  void
  gyroscope_bandwidth(const uint8_t bandwidth) const;

  /**
   * @brief Returns the gyroscope bit resolution.
   *
   * @return Gyroscope bit resolution.
   */
  float
  gyroscope_resolution() const;

  /**
   * @brief Sets the gyroscope resolution.
   *
   * @param scale_setting Full scale setting value. Use the
   * ICM20649_GYRO_FULLSCALE_vDPS macros define in this header file. The value v
   * can be 500, 1000, 2000 or 4000.
   */
  void
  gyroscope_resolution(const uint8_t scale_setting) const;

  /**
   * @brief Sets the gyroscope sample rate.
   *
   * @param rate Sample rate value (\f$[\text{Hz}]\f$).
   *
   * @return Actual sample rate, which may differ from the input value because
   * of the finite and discrete number of divider settings.
   */
  float
  gyroscope_sample_rate(const float rate) const;

  /**
   * @brief Performs a soft reset of the IMU.
   */
  void
  reset() const;

  /**
   * @brief Sets the sample rate of both of the accelerometer and the gyroscope.
   *
   * @param rate Sample rate value (\f$[\text{Hz}]\f$).
   */
  void
  sample_rate(const float rate) const;

  /**
   * @brief Enables/disables the IMU sensors.
   *
   * @param accelerometer If true enables the accelerometer sensor, otherwise
   * disables it.
   * @param gyroscope If true enables the gyroscope sensor, otherwise disables
   * it.
   * @param temperature If true enables the temperature sensor, otherwise
   * disables it.
   */
  void
  sensors(const bool accelerometer, const bool gyroscope,
          const bool temperature) const;

  /**
   * @brief Perform a temperature measurement in \f$[^{\circ}\text{C}]\f$.
   *
   * @return The temperature measurement.
   */
  float
  temperature() const;

  /**
   * @brief Reads and returns the device ID.
   *
   * @return The device ID from the WHO_AM_I register.
   * This value should match the value of the ICM20649_DEVICE_ID macro defined
   * in thsi header file.
   */
  uint8_t
  who_am_i() const;

 protected:
  /**
   * @brief Reads a byte from a register from the IMU.
   *
   * @param reg The register address to read from.
   *
   * @return The byte read from the register.
   */
  uint8_t
  read_register(const uint16_t reg) const;

  /**
   * @brief Writes a byte to a register from the IMU.
   *
   * @param reg The register address to write to.
   */
  void
  write_register(const uint16_t reg, const uint8_t val) const;

  /**
   * @brief I2C bus.
   */
  const uint8_t i2c_bus_;
  /**
   * @brief I2C device file descriptor.
   */
  const int fid_;
};
}  // namespace ICM20649

#endif  // ICM20649_HPP
