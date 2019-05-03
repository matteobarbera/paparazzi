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
 * @file peripherals/icm206xx_spi.c
 *
 * Driver for the ICM-602XX using SPI.
 *
 */

#include "peripherals/icm206xx_spi.h"

void icm206xx_spi_init(struct Icm206xx_Spi *icm, struct spi_periph *spi_p, uint8_t slave_idx)
{
  /* set spi_peripheral */
  icm->spi_p = spi_p;

  /* configure spi transaction */
  icm->spi_trans.cpol = SPICpolIdleHigh;
  icm->spi_trans.cpha = SPICphaEdge2;
  icm->spi_trans.dss = SPIDss8bit;
  icm->spi_trans.bitorder = SPIMSBFirst;
  icm->spi_trans.cdiv = SPIDiv64;

  icm->spi_trans.select = SPISelectUnselect;
  icm->spi_trans.slave_idx = slave_idx;
  icm->spi_trans.output_length = 2;
  icm->spi_trans.input_length = ICM206XX_BUFFER_LEN;
  icm->spi_trans.before_cb = NULL;
  icm->spi_trans.after_cb = NULL;
  icm->spi_trans.input_buf = &(icm->rx_buf[0]);
  icm->spi_trans.output_buf = &(icm->tx_buf[0]);

  /* set inital status: Success or Done */
  icm->spi_trans.status = SPITransDone;

  /* set default ICM206XX config options */
  icm206xx_set_default_config(&(icm->config));

  icm->data_available = false;
  icm->config.initialized = false;
  icm->config.init_status = ICM206XX_CONF_UNINIT;
}


static void icm206xx_spi_write_to_reg(void *icm, uint8_t _reg, uint8_t _val)
{
  struct Icm206xx_Spi *icm_spi = (struct Icm206xx_Spi *)(icm);
  icm_spi->spi_trans.output_length = 2;
  icm_spi->spi_trans.input_length = 0;
  icm_spi->tx_buf[0] = _reg;
  icm_spi->tx_buf[1] = _val;
  spi_submit(icm_spi->spi_p, &(icm_spi->spi_trans));
}

// Configuration function called once before normal use
void icm206xx_spi_start_configure(struct Icm206xx_Spi *icm)
{
  if (icm->config.init_status == ICM206XX_CONF_UNINIT) {
    // First check if we found the chip (succesfull WHO_AM_I response)
    if (icm->spi_trans.status == SPITransSuccess && icm->rx_buf[1] == ICM206XX_WHOAMI_REPLY) {
      icm->config.init_status++;
      icm->spi_trans.status = SPITransDone;
      icm206xx_send_config(icm206xx_spi_write_to_reg, (void *)icm, &(icm->config));
    }
    // Send WHO_AM_I to check if chip is there
    else if (icm->spi_trans.status != SPITransRunning && icm->spi_trans.status != SPITransPending) {
      icm->spi_trans.output_length = 1;
      icm->spi_trans.input_length = 2;
      icm->tx_buf[0] = ICM206XX_REG_WHO_AM_I | ICM206XX_SPI_READ;
      spi_submit(icm->spi_p, &(icm->spi_trans));
    }
  }
}

void icm206xx_spi_read(struct Icm206xx_Spi *icm)
{
  if (icm->config.initialized && icm->spi_trans.status == SPITransDone) {
    icm->spi_trans.output_length = 1;
    icm->spi_trans.input_length = 1 + icm->config.nb_bytes;
    /* set read bit and multiple byte bit, then address */
    icm->tx_buf[0] = ICM206XX_REG_INT_STATUS | ICM206XX_SPI_READ;
    spi_submit(icm->spi_p, &(icm->spi_trans));
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void icm206xx_spi_event(struct Icm206xx_Spi *icm)
{
  if (icm->config.initialized) {
    if (icm->spi_trans.status == SPITransFailed) {
      icm->spi_trans.status = SPITransDone;
    } else if (icm->spi_trans.status == SPITransSuccess) {
      // Successfull reading
      if (bit_is_set(icm->rx_buf[1], 0)) {
        // new data
        icm->data_accel.vect.x = Int16FromBuf(icm->rx_buf, 2);
        icm->data_accel.vect.y = Int16FromBuf(icm->rx_buf, 4);
        icm->data_accel.vect.z = Int16FromBuf(icm->rx_buf, 6);
        icm->data_rates.rates.p = Int16FromBuf(icm->rx_buf, 10);
        icm->data_rates.rates.q = Int16FromBuf(icm->rx_buf, 12);
        icm->data_rates.rates.r = Int16FromBuf(icm->rx_buf, 14);
        icm->data_available = true;
      }
      icm->spi_trans.status = SPITransDone;
    }
  } else if (icm->config.init_status != ICM206XX_CONF_UNINIT) { // Configuring but not yet initialized
    switch (icm->spi_trans.status) {
      case SPITransFailed:
        icm->config.init_status--; // Retry config (TODO max retry)
        /* Falls through. */
      case SPITransSuccess:
      case SPITransDone:
        icm206xx_send_config(icm206xx_spi_write_to_reg, (void *)icm, &(icm->config));
        if (icm->config.initialized) {
          icm->spi_trans.status = SPITransDone;
        }
        break;
      default:
        break;
    }
  }
}

