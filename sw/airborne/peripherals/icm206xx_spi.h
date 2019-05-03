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
 * @file peripherals/icm206xx_spi.h
 *
 * Driver for the ICM-602XX using SPI.
 */

#ifndef ICM206XX_SPI_H
#define ICM206XX_SPI_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/spi.h"

/* Include common ICM206XX options and definitions */
#include "peripherals/icm206xx.h"


#define ICM206XX_BUFFER_LEN 32

enum Icm206xxSpiSlaveInitStatus {
  ICM206XX_SPI_CONF_UNINIT,
  ICM206XX_SPI_CONF_I2C_MST_CLK,
  ICM206XX_SPI_CONF_I2C_MST_DELAY,
  ICM206XX_SPI_CONF_I2C_MST_EN,
  ICM206XX_SPI_CONF_SLAVES_CONFIGURE,
  ICM206XX_SPI_CONF_DONE
};

struct Icm206xx_Spi {
  struct spi_periph *spi_p;
  struct spi_transaction spi_trans;
  volatile uint8_t tx_buf[2];
  volatile uint8_t rx_buf[ICM206XX_BUFFER_LEN];
  volatile bool data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;         ///< accel data vector in accel coordinate system
    int16_t value[3];               ///< accel data values accessible by channel index
  } data_accel;
  union {
    struct Int16Rates rates;        ///< rates data as angular rates in gyro coordinate system
    int16_t value[3];               ///< rates data values accessible by channel index
  } data_rates;
  struct Icm206xxConfig config;
};

// Functions
extern void icm206xx_spi_init(struct Icm206xx_Spi *icm, struct spi_periph *spi_p, uint8_t addr);
extern void icm206xx_spi_start_configure(struct Icm206xx_Spi *icm);
extern void icm206xx_spi_read(struct Icm206xx_Spi *icm);
extern void icm206xx_spi_event(struct Icm206xx_Spi *icm);

/// convenience function: read or start configuration if not already initialized
static inline void icm206xx_spi_periodic(struct Icm206xx_Spi *icm)
{
  if (icm->config.initialized) {
    icm206xx_spi_read(icm);
  } else {
    icm206xx_spi_start_configure(icm);
  }
}

#endif // ICM206XX_SPI_H
