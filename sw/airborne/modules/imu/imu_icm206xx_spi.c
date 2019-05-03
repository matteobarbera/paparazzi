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
 * @file subsystems/imu/imu_icm206xx_spi.c
 *
 * IMU driver for the ICM206XX using SPI
 *
 */
#include "modules/imu/imu_icm206xx_spi.h"

#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/spi.h"

/* SPI defaults set in subsystem makefile, can be configured from airframe file */
PRINT_CONFIG_VAR(IMU_ICM206XX_SPI_SLAVE_IDX)
PRINT_CONFIG_VAR(IMU_ICM206XX_SPI_DEV)


#if !defined IMU_ICM206XX_GYRO_LOWPASS_FILTER && !defined IMU_ICM206XX_ACCEL_LOWPASS_FILTER && !defined  IMU_ICM206XX_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz
 * Gyroscope: Bandwidth 41Hz, Delay 5.9ms sampling 1kHz
 * Output rate: 100Hz
 */
#define IMU_ICM206XX_GYRO_LOWPASS_FILTER ICM206XX_DLPF_GYRO_41HZ
#define IMU_ICM206XX_ACCEL_LOWPASS_FILTER ICM206XX_DLPF_ACCEL_44HZ
#define IMU_ICM206XX_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 218Hz
 * Gyroscope: Bandwidth 250Hz, Delay 0.97ms sampling 8kHz
 * Output rate: 2kHz
 */
#define IMU_ICM206XX_GYRO_LOWPASS_FILTER ICM206XX_DLPF_GYRO_250HZ
#define IMU_ICM206XX_ACCEL_LOWPASS_FILTER ICM206XX_DLPF_ACCEL_218HZ
#define IMU_ICM206XX_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#else
/* By default, don't go too fast */
#define IMU_ICM206XX_SMPLRT_DIV 9
#define IMU_ICM206XX_GYRO_LOWPASS_FILTER ICM206XX_DLPF_GYRO_41HZ
#define IMU_ICM206XX_ACCEL_LOWPASS_FILTER ICM206XX_DLPF_ACCEL_44HZ
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#endif
#endif
PRINT_CONFIG_VAR(IMU_ICM206XX_SMPLRT_DIV)
PRINT_CONFIG_VAR(IMU_ICM206XX_GYRO_LOWPASS_FILTER)
PRINT_CONFIG_VAR(IMU_ICM206XX_ACCEL_LOWPASS_FILTER)

PRINT_CONFIG_VAR(IMU_ICM206XX_GYRO_RANGE)
PRINT_CONFIG_VAR(IMU_ICM206XX_ACCEL_RANGE)

// Default channels order
#ifndef IMU_ICM206XX_CHAN_X
#define IMU_ICM206XX_CHAN_X 0
#endif
PRINT_CONFIG_VAR(IMU_ICM206XX_CHAN_X)
#ifndef IMU_ICM206XX_CHAN_Y
#define IMU_ICM206XX_CHAN_Y 1
#endif
PRINT_CONFIG_VAR(IMU_ICM206XX_CHAN_Y)
#ifndef IMU_ICM206XX_CHAN_Z
#define IMU_ICM206XX_CHAN_Z 2
#endif
PRINT_CONFIG_VAR(IMU_ICM206XX_CHAN_Z)

#ifndef IMU_ICM206XX_X_SIGN
#define IMU_ICM206XX_X_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_ICM206XX_X_SIGN)
#ifndef IMU_ICM206XX_Y_SIGN
#define IMU_ICM206XX_Y_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_ICM206XX_Y_SIGN)
#ifndef IMU_ICM206XX_Z_SIGN
#define IMU_ICM206XX_Z_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_ICM206XX_Z_SIGN)


struct ImuIcm206xx imu_icm206xx;

void imu_icm206xx_init(void)
{
  /* ICM206XX */
  icm206xx_spi_init(&imu_icm206xx.icm, &(IMU_ICM206XX_SPI_DEV), IMU_ICM206XX_SPI_SLAVE_IDX);
  // change the default configuration
  imu_icm206xx.icm.config.smplrt_div = IMU_ICM206XX_SMPLRT_DIV;
  imu_icm206xx.icm.config.dlpf_gyro_cfg = IMU_ICM206XX_GYRO_LOWPASS_FILTER;
  imu_icm206xx.icm.config.dlpf_accel_cfg = IMU_ICM206XX_ACCEL_LOWPASS_FILTER;
  imu_icm206xx.icm.config.gyro_range = IMU_ICM206XX_GYRO_RANGE;
  imu_icm206xx.icm.config.accel_range = IMU_ICM206XX_ACCEL_RANGE;

}

void imu_icm206xx_periodic(void)
{
  icm206xx_spi_periodic(&imu_icm206xx.icm);
}

void imu_icm206xx_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  // If the ICM206XX SPI transaction has succeeded: convert the data
  icm206xx_spi_event(&imu_icm206xx.icm);

  if (imu_icm206xx.icm.data_available) {
    // set channel order
    struct Int32Vect3 accel = {
      IMU_ICM206XX_X_SIGN * (int32_t)(imu_icm206xx.icm.data_accel.value[IMU_ICM206XX_CHAN_X]),
      IMU_ICM206XX_Y_SIGN * (int32_t)(imu_icm206xx.icm.data_accel.value[IMU_ICM206XX_CHAN_Y]),
      IMU_ICM206XX_Z_SIGN * (int32_t)(imu_icm206xx.icm.data_accel.value[IMU_ICM206XX_CHAN_Z])
    };
    struct Int32Rates rates = {
      IMU_ICM206XX_X_SIGN * (int32_t)(imu_icm206xx.icm.data_rates.value[IMU_ICM206XX_CHAN_X]),
      IMU_ICM206XX_Y_SIGN * (int32_t)(imu_icm206xx.icm.data_rates.value[IMU_ICM206XX_CHAN_Y]),
      IMU_ICM206XX_Z_SIGN * (int32_t)(imu_icm206xx.icm.data_rates.value[IMU_ICM206XX_CHAN_Z])
    };
    // unscaled vector
    VECT3_COPY(imu.accel_unscaled, accel);
    RATES_COPY(imu.gyro_unscaled, rates);

    imu_icm206xx.icm.data_available = false;

    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_ICM206XX_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_ICM206XX_ID, now_ts, &imu.accel);
  }

}

