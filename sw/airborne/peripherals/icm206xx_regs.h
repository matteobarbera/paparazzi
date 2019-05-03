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
 * @file peripherals/icm206xx_regs.h
 *
 * Register and address definitions for ICM-602XX
 */

#ifndef ICM206XX_REGS_H
#define ICM206XX_REGS_H

/* default I2C address */
#define ICM206XX_ADDR                0xD0
#define ICM206XX_ADDR_ALT            0xD2

#define ICM206XX_SPI_READ            0x80

// Power and Interface
#define ICM206XX_REG_AUX_VDDIO       0x01  // Must be set to 0 on ICM206XX
#define ICM206XX_REG_USER_CTRL       0x6A
#define ICM206XX_REG_PWR_MGMT_1      0x6B
#define ICM206XX_REG_PWR_MGMT_2      0x6C

// FIFO
#define ICM206XX_REG_FIFO_EN         0x23
#define ICM206XX_REG_FIFO_COUNT_H    0x72
#define ICM206XX_REG_FIFO_COUNT_L    0x73
#define ICM206XX_REG_FIFO_R_W        0x74

// Measurement Settings
#define ICM206XX_REG_SMPLRT_DIV      0x19
#define ICM206XX_REG_CONFIG          0x1A
#define ICM206XX_REG_GYRO_CONFIG     0x1B
#define ICM206XX_REG_ACCEL_CONFIG    0x1C
#define ICM206XX_REG_ACCEL_CONFIG_2  0x1D

// Interrupt
#define ICM206XX_REG_INT_PIN_CFG     0x37
#define ICM206XX_REG_INT_ENABLE      0x38
#define ICM206XX_REG_INT_STATUS      0x3A

// Accelero
#define ICM206XX_REG_ACCEL_XOUT_H    0x3B
#define ICM206XX_REG_ACCEL_XOUT_L    0x3C
#define ICM206XX_REG_ACCEL_YOUT_H    0x3D
#define ICM206XX_REG_ACCEL_YOUT_L    0x3E
#define ICM206XX_REG_ACCEL_ZOUT_H    0x3F
#define ICM206XX_REG_ACCEL_ZOUT_L    0x40

// Temperature
#define ICM206XX_REG_TEMP_OUT_H      0x41
#define ICM206XX_REG_TEMP_OUT_L      0x42

// Gyro
#define ICM206XX_REG_GYRO_XOUT_H     0x43
#define ICM206XX_REG_GYRO_XOUT_L     0x44
#define ICM206XX_REG_GYRO_YOUT_H     0x45
#define ICM206XX_REG_GYRO_YOUT_L     0x46
#define ICM206XX_REG_GYRO_ZOUT_H     0x47
#define ICM206XX_REG_GYRO_ZOUT_L     0x48

// Who am I
#define ICM206XX_REG_WHO_AM_I        0x75
#define ICM206XX_WHOAMI_REPLY        0x11

// in ICM206XX_REG_USER_CTRL
#define ICM206XX_SIG_COND_RESET      0
#define ICM206XX_FIFO_RESET          2
#define ICM206XX_FIFO_EN             6

/** Digital Low Pass Filter Options
 */
enum Icm206xxDLPFGyro {
  ICM206XX_DLPF_GYRO_250HZ  = 0x0,  // internal sampling rate 8kHz
  ICM206XX_DLPF_GYRO_176HZ  = 0x1,  // internal sampling rate 1kHz
  ICM206XX_DLPF_GYRO_92HZ   = 0x2,
  ICM206XX_DLPF_GYRO_41HZ   = 0x3,
  ICM206XX_DLPF_GYRO_20HZ   = 0x4,
  ICM206XX_DLPF_GYRO_10HZ   = 0x5,
  ICM206XX_DLPF_GYRO_05HZ   = 0x6,
  ICM206XX_DLPF_GYRO_3281HZ = 0x7  // internal sampling rate 8kHz
};

enum Icm206xxDLPFAccel {
  ICM206XX_DLPF_ACCEL_218HZ = 0x0,  // internal sampling rate 1kHz
  //ICM206XX_DLPF_ACCEL_218HZ = 0x1,  // TODO check if really same as 0
  ICM206XX_DLPF_ACCEL_99HZ  = 0x2,
  ICM206XX_DLPF_ACCEL_44HZ  = 0x3,
  ICM206XX_DLPF_ACCEL_21HZ  = 0x4,
  ICM206XX_DLPF_ACCEL_10HZ  = 0x5,
  ICM206XX_DLPF_ACCEL_05HZ  = 0x6,
  ICM206XX_DLPF_ACCEL_420HZ = 0x7
};

/**
 * Selectable gyro range
 */
enum Icm206xxGyroRanges {
  ICM206XX_GYRO_RANGE_250  = 0x00,
  ICM206XX_GYRO_RANGE_500  = 0x01,
  ICM206XX_GYRO_RANGE_1000 = 0x02,
  ICM206XX_GYRO_RANGE_2000 = 0x03
};

/**
 * Selectable accel range
 */
enum Icm206xxAccelRanges {
  ICM206XX_ACCEL_RANGE_2G  = 0x00,
  ICM206XX_ACCEL_RANGE_4G  = 0x01,
  ICM206XX_ACCEL_RANGE_8G  = 0x02,
  ICM206XX_ACCEL_RANGE_16G = 0x03
};

#endif /* ICM206XX_REGS_H */

