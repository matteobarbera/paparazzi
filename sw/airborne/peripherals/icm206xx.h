/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/icm206xx.h
 *
 * ICM-602XX driver common interface (I2C and SPI).
 */

#ifndef ICM206XX_H
#define ICM206XX_H

#include "std.h"

/* Include address and register definition */
#include "peripherals/icm206xx_regs.h"

/// Default sample rate divider
#define ICM206XX_DEFAULT_SMPLRT_DIV 0
/// Default gyro full scale range +- 2000°/s
#define ICM206XX_DEFAULT_FS_SEL ICM206XX_GYRO_RANGE_1000
/// Default accel full scale range +- 16g
#define ICM206XX_DEFAULT_AFS_SEL ICM206XX_ACCEL_RANGE_8G
/// Default internal sampling (1kHz, 42Hz LP Bandwidth)
#define ICM206XX_DEFAULT_DLPF_ACCEL_CFG ICM206XX_DLPF_ACCEL_44HZ
/// Default internal sampling (1kHz, 42Hz LP Bandwidth)
#define ICM206XX_DEFAULT_DLPF_GYRO_CFG ICM206XX_DLPF_GYRO_41HZ
/// Default interrupt config: DATA_RDY_EN
#define ICM206XX_DEFAULT_INT_CFG 1
/// Default clock: PLL with X gyro reference
#define ICM206XX_DEFAULT_CLK_SEL 1

/** default gyro sensitivy from the datasheet
 * sens = 1/ [LSB/(deg/s)] * pi/180 * 2^INT32_RATE_FRAC
 * ex: ICM with 1000 deg/s has 32.8 LSB/(deg/s)
 *     sens = 1/32.8 * pi/180 * 4096 = 2.17953
 */
#define ICM206XX_GYRO_SENS_250 0.544883
#define ICM206XX_GYRO_SENS_250_NUM 19327
#define ICM206XX_GYRO_SENS_250_DEN 35470
#define ICM206XX_GYRO_SENS_500 1.08977
#define ICM206XX_GYRO_SENS_500_NUM 57663
#define ICM206XX_GYRO_SENS_500_DEN 52913
#define ICM206XX_GYRO_SENS_1000 2.17953
#define ICM206XX_GYRO_SENS_1000_NUM 18271
#define ICM206XX_GYRO_SENS_1000_DEN 8383
#define ICM206XX_GYRO_SENS_2000 4.35906
#define ICM206XX_GYRO_SENS_2000_NUM 36542
#define ICM206XX_GYRO_SENS_2000_DEN 8383

// Get default sensitivity from a table
extern const float ICM206XX_GYRO_SENS[4];
// Get default sensitivity numerator and denominator from a table
extern const int32_t ICM206XX_GYRO_SENS_FRAC[4][2];

/** default accel sensitivy from the datasheet
 * sens = 9.81 [m/s^2] / [LSB/g] * 2^INT32_ACCEL_FRAC
 * ex: ICM with 8g has 4096 LSB/g
 *     sens = 9.81 [m/s^2] / 4096 [LSB/g] * 2^INT32_ACCEL_FRAC = 2.4525
 */
#define ICM206XX_ACCEL_SENS_2G 0.613125
#define ICM206XX_ACCEL_SENS_2G_NUM 981
#define ICM206XX_ACCEL_SENS_2G_DEN 1600
#define ICM206XX_ACCEL_SENS_4G 1.22625
#define ICM206XX_ACCEL_SENS_4G_NUM 981
#define ICM206XX_ACCEL_SENS_4G_DEN 800
#define ICM206XX_ACCEL_SENS_8G 2.4525
#define ICM206XX_ACCEL_SENS_8G_NUM 981
#define ICM206XX_ACCEL_SENS_8G_DEN 400
#define ICM206XX_ACCEL_SENS_16G 4.905
#define ICM206XX_ACCEL_SENS_16G_NUM 981
#define ICM206XX_ACCEL_SENS_16G_DEN 200

// Get default sensitivity from a table
extern const float ICM206XX_ACCEL_SENS[4];
// Get default sensitivity numerator and denominator from a table
extern const int32_t ICM206XX_ACCEL_SENS_FRAC[4][2];

enum Icm206xxConfStatus {
  ICM206XX_CONF_UNINIT,
  ICM206XX_CONF_RESET,
  ICM206XX_CONF_USER_RESET,
  ICM206XX_CONF_PWR,
  ICM206XX_CONF_SD,
  ICM206XX_CONF_DLPF_ACCEL,
  ICM206XX_CONF_DLPF_GYRO,
  ICM206XX_CONF_GYRO,
  ICM206XX_CONF_ACCEL,
  ICM206XX_CONF_INT_ENABLE,
  ICM206XX_CONF_DONE
};

/// Configuration function prototype
typedef void (*Icm206xxConfigSet)(void *icm, uint8_t _reg, uint8_t _val);

struct Icm206xxConfig {
  uint8_t smplrt_div;                     ///< Sample rate divider
  enum Icm206xxDLPFAccel dlpf_accel_cfg;  ///< Digital Low Pass Filter for accelerometer
  enum Icm206xxDLPFGyro dlpf_gyro_cfg;    ///< Digital Low Pass Filter for gyroscope
  enum Icm206xxGyroRanges gyro_range;     ///< deg/s Range
  enum Icm206xxAccelRanges accel_range;   ///< g Range
  bool drdy_int_enable;                   ///< Enable Data Ready Interrupt
  uint8_t clk_sel;                        ///< Clock select
  uint8_t nb_bytes;                       ///< number of bytes to read starting with ICM206XX_REG_INT_STATUS
  enum Icm206xxConfStatus init_status;    ///< init status
  bool initialized;                       ///< config done flag
};

extern void icm206xx_set_default_config(struct Icm206xxConfig *c);

/// Configuration sequence called once before normal use
extern void icm206xx_send_config(Icm206xxConfigSet icm_set, void *icm, struct Icm206xxConfig *config);

#endif // ICM206XX_H
