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
 * @file peripherals/icm206xx.c
 *
 * ICM-602XX driver common functions (I2C and SPI).
 *
 * Still needs the either I2C or SPI specific implementation.
 */

#include "peripherals/icm206xx.h"

const float ICM206XX_GYRO_SENS[4] = {
  ICM206XX_GYRO_SENS_250,
  ICM206XX_GYRO_SENS_500,
  ICM206XX_GYRO_SENS_1000,
  ICM206XX_GYRO_SENS_2000
};

const int32_t ICM206XX_GYRO_SENS_FRAC[4][2] = {
  { ICM206XX_GYRO_SENS_250_NUM, ICM206XX_GYRO_SENS_250_DEN },
  { ICM206XX_GYRO_SENS_500_NUM, ICM206XX_GYRO_SENS_500_DEN },
  { ICM206XX_GYRO_SENS_1000_NUM, ICM206XX_GYRO_SENS_1000_DEN },
  { ICM206XX_GYRO_SENS_2000_NUM, ICM206XX_GYRO_SENS_2000_DEN }
};

const float ICM206XX_ACCEL_SENS[4] = {
  ICM206XX_ACCEL_SENS_2G,
  ICM206XX_ACCEL_SENS_4G,
  ICM206XX_ACCEL_SENS_8G,
  ICM206XX_ACCEL_SENS_16G
};

const int32_t ICM206XX_ACCEL_SENS_FRAC[4][2] = {
  { ICM206XX_ACCEL_SENS_2G_NUM, ICM206XX_ACCEL_SENS_2G_DEN },
  { ICM206XX_ACCEL_SENS_4G_NUM, ICM206XX_ACCEL_SENS_4G_DEN },
  { ICM206XX_ACCEL_SENS_8G_NUM, ICM206XX_ACCEL_SENS_8G_DEN },
  { ICM206XX_ACCEL_SENS_16G_NUM, ICM206XX_ACCEL_SENS_16G_DEN }
};

void icm206xx_set_default_config(struct Icm206xxConfig *c)
{
  c->clk_sel = ICM206XX_DEFAULT_CLK_SEL;
  c->smplrt_div = ICM206XX_DEFAULT_SMPLRT_DIV;
  c->dlpf_gyro_cfg = ICM206XX_DEFAULT_DLPF_GYRO_CFG;
  c->dlpf_accel_cfg = ICM206XX_DEFAULT_DLPF_ACCEL_CFG;
  c->gyro_range = ICM206XX_DEFAULT_FS_SEL;
  c->accel_range = ICM206XX_DEFAULT_AFS_SEL;
  c->drdy_int_enable = false;

  /* Number of bytes to read starting with ICM206XX_REG_INT_STATUS
   * By default read only gyro and accel data -> 15 bytes.
   */
  c->nb_bytes = 15;
}

void icm206xx_send_config(Icm206xxConfigSet icm_set, void *icm, struct Icm206xxConfig *config)
{
  switch (config->init_status) {
    case ICM206XX_CONF_RESET:
      /* device reset, set register values to defaults */
      icm_set(icm, ICM206XX_REG_PWR_MGMT_1, (1 << 6));
      config->init_status++;
      break;
    case ICM206XX_CONF_USER_RESET:
      /* trigger FIFO, I2C_MST and SIG_COND resets */
      icm_set(icm, ICM206XX_REG_USER_CTRL, ((1 << ICM206XX_FIFO_RESET) |
                                           (1 << ICM206XX_SIG_COND_RESET)));
      config->init_status++;
      break;
    case ICM206XX_CONF_PWR:
      /* switch to gyroX clock by default */
      icm_set(icm, ICM206XX_REG_PWR_MGMT_1, ((config->clk_sel) | (0 << 6)));
      config->init_status++;
      break;
    case ICM206XX_CONF_SD:
      /* configure sample rate divider */
      icm_set(icm, ICM206XX_REG_SMPLRT_DIV, config->smplrt_div);
      config->init_status++;
      break;
    case ICM206XX_CONF_DLPF_GYRO:
      /* configure digital low pass filter for gyroscope */
      icm_set(icm, ICM206XX_REG_CONFIG, config->dlpf_gyro_cfg);
      config->init_status++;
      break;
    case ICM206XX_CONF_DLPF_ACCEL:
      /* configure digital low pass filter fir accelerometer */
      icm_set(icm, ICM206XX_REG_ACCEL_CONFIG_2, config->dlpf_accel_cfg);
      config->init_status++;
      break;
    case ICM206XX_CONF_GYRO:
      /* configure gyro range */
      icm_set(icm, ICM206XX_REG_GYRO_CONFIG, (config->gyro_range << 3));
      config->init_status++;
      break;
    case ICM206XX_CONF_ACCEL:
      /* configure accelerometer range */
      icm_set(icm, ICM206XX_REG_ACCEL_CONFIG, (config->accel_range << 3));
      config->init_status++;
      break;
    case ICM206XX_CONF_INT_ENABLE:
      /* configure data ready interrupt */
      icm_set(icm, ICM206XX_REG_INT_ENABLE, (config->drdy_int_enable << 0));
      config->init_status++;
      break;
    case ICM206XX_CONF_DONE:
      config->initialized = true;
      break;
    default:
      break;
  }
}
