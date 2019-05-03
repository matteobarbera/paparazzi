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
 * @file subsystems/imu/imu_icm206xx_spi.h
 *
 * IMU driver for the ICM206XX using SPI
 *
 */

#ifndef IMU_ICM206XX_SPI_H
#define IMU_ICM206XX_SPI_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/icm206xx_spi.h"


#ifndef IMU_ICM206XX_GYRO_RANGE
#define IMU_ICM206XX_GYRO_RANGE ICM206XX_GYRO_RANGE_1000
#endif

#ifndef IMU_ICM206XX_ACCEL_RANGE
#define IMU_ICM206XX_ACCEL_RANGE ICM206XX_ACCEL_RANGE_8G
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS ICM206XX_GYRO_SENS[IMU_ICM206XX_GYRO_RANGE]
#define IMU_GYRO_P_SENS_NUM ICM206XX_GYRO_SENS_FRAC[IMU_ICM206XX_GYRO_RANGE][0]
#define IMU_GYRO_P_SENS_DEN ICM206XX_GYRO_SENS_FRAC[IMU_ICM206XX_GYRO_RANGE][1]
#define IMU_GYRO_Q_SENS ICM206XX_GYRO_SENS[IMU_ICM206XX_GYRO_RANGE]
#define IMU_GYRO_Q_SENS_NUM ICM206XX_GYRO_SENS_FRAC[IMU_ICM206XX_GYRO_RANGE][0]
#define IMU_GYRO_Q_SENS_DEN ICM206XX_GYRO_SENS_FRAC[IMU_ICM206XX_GYRO_RANGE][1]
#define IMU_GYRO_R_SENS ICM206XX_GYRO_SENS[IMU_ICM206XX_GYRO_RANGE]
#define IMU_GYRO_R_SENS_NUM ICM206XX_GYRO_SENS_FRAC[IMU_ICM206XX_GYRO_RANGE][0]
#define IMU_GYRO_R_SENS_DEN ICM206XX_GYRO_SENS_FRAC[IMU_ICM206XX_GYRO_RANGE][1]
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS ICM206XX_ACCEL_SENS[IMU_ICM206XX_ACCEL_RANGE]
#define IMU_ACCEL_X_SENS_NUM ICM206XX_ACCEL_SENS_FRAC[IMU_ICM206XX_ACCEL_RANGE][0]
#define IMU_ACCEL_X_SENS_DEN ICM206XX_ACCEL_SENS_FRAC[IMU_ICM206XX_ACCEL_RANGE][1]
#define IMU_ACCEL_Y_SENS ICM206XX_ACCEL_SENS[IMU_ICM206XX_ACCEL_RANGE]
#define IMU_ACCEL_Y_SENS_NUM ICM206XX_ACCEL_SENS_FRAC[IMU_ICM206XX_ACCEL_RANGE][0]
#define IMU_ACCEL_Y_SENS_DEN ICM206XX_ACCEL_SENS_FRAC[IMU_ICM206XX_ACCEL_RANGE][1]
#define IMU_ACCEL_Z_SENS ICM206XX_ACCEL_SENS[IMU_ICM206XX_ACCEL_RANGE]
#define IMU_ACCEL_Z_SENS_NUM ICM206XX_ACCEL_SENS_FRAC[IMU_ICM206XX_ACCEL_RANGE][0]
#define IMU_ACCEL_Z_SENS_DEN ICM206XX_ACCEL_SENS_FRAC[IMU_ICM206XX_ACCEL_RANGE][1]
#endif


struct ImuIcm206xx {
  struct Icm206xx_Spi icm;

  struct spi_transaction wait_slave4_trans;
  volatile uint8_t wait_slave4_tx_buf[1];
  volatile uint8_t wait_slave4_rx_buf[2];
  volatile bool slave4_ready;
};

extern struct ImuIcm206xx imu_icm206xx;

extern void imu_icm206xx_init(void);
extern void imu_icm206xx_periodic(void);
extern void imu_icm206xx_event(void);

#endif /* IMU_ICM206XX_SPI_H */
