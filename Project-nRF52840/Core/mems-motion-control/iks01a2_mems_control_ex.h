/**
 ******************************************************************************
 * @file    iks01a2_mems_control_ex.h
 * @author  MEMS Application Team
 * @brief   This file contains the MEMS sensors interface for X-NUCLEO-IKS01A2
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IKS01A2_MEMS_CONTROL_H
#define IKS01A2_MEMS_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "iks01a2_motion_sensors.h"
#include "iks01a2_motion_sensors_ex.h"

void BSP_SENSOR_ACC_GetOrientation(char *Orientation);
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
void BSP_SENSOR_GYR_GetOrientation(char *Orientation);
#endif
void BSP_SENSOR_MAG_GetOrientation(char *Orientation);

void BSP_SENSOR_ACC_Read_Register(uint8_t Reg, uint8_t *Data);
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
void BSP_SENSOR_GYR_Read_Register(uint8_t Reg, uint8_t *Data);
#endif
void BSP_SENSOR_MAG_Read_Register(uint8_t Reg, uint8_t *Data);

void BSP_SENSOR_ACC_Write_Register(uint8_t Reg, uint8_t Data);
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
void BSP_SENSOR_GYR_Write_Register(uint8_t Reg, uint8_t Data);
#endif
void BSP_SENSOR_MAG_Write_Register(uint8_t Reg, uint8_t Data);

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
void BSP_SENSOR_ACC_SetDRDYMode(uint8_t Mode);
void BSP_SENSOR_GYR_SetDRDYMode(uint8_t Mode);
void BSP_SENSOR_ACC_SetDRDYInt(uint8_t Enable);
void BSP_SENSOR_GYR_SetDRDYInt(uint8_t Enable);
#endif
void BSP_SENSOR_ACC_GetDRDYStatus(uint8_t *Status);

void BSP_ACC_GYR_Read_FSM_Data(uint8_t *Data);
void BSP_ACC_GYR_Read_MLC_Data(uint8_t *Data);

#endif /* IKS01A2_MEMS_CONTROL_H */
