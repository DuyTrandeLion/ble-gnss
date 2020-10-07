/**
 ******************************************************************************
 * @file    iks01a2_conf_template.h
 * @author  MEMS Application Team
 * @brief   IKS01A2 configuration template file.
 *          This file should be copied to the application folder and renamed
 *          to iks01a2_conf.h.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Replace the header file names with the ones of the target platform */
#include "errno.h"
#include "com_bus.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IKS01A2_CONF_H__
#define __IKS01A2_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN 1 */
#define USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0            0U
#define USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0      1U
#define USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0      1U
/* USER CODE END 1 */

#define IKS01A2_I2C_Init        i2c_bus_init
#define IKS01A2_I2C_DeInit      i2c_bus_deinit
#define IKS01A2_I2C_ReadReg     i2c_bus_read_reg
#define IKS01A2_I2C_WriteReg    i2c_bus_write_reg
#define IKS01A2_GetTick         NULL

#ifdef __cplusplus
}
#endif

#endif /* __IKS01A2_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

