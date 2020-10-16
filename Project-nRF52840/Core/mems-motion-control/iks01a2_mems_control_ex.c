/**
 ******************************************************************************
 * @file    iks01a2_mems_control_ex.c
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

#include "iks01a2_mems_control_ex.h"

/**
  * @brief  Get accelerometer sensor orientation
  * @param  Orientation Pointer to sensor orientation
  * @retval None
  */
void BSP_SENSOR_ACC_GetOrientation(char *Orientation)
{
  Orientation[0] = 'n';
  Orientation[1] = 'w';
  Orientation[2] = 'u';
}

/**
  * @brief  Get gyroscope sensor orientation
  * @param  Orientation Pointer to sensor orientation
  * @retval None
  */
void BSP_SENSOR_GYR_GetOrientation(char *Orientation)
{
  Orientation[0] = 'n';
  Orientation[1] = 'w';
  Orientation[2] = 'u';
}

/**
  * @brief  Get magnetometer sensor orientation
  * @param  Orientation Pointer to sensor orientation
  * @retval None
  */
void BSP_SENSOR_MAG_GetOrientation(char *Orientation)
{
  Orientation[0] = 'n';
  Orientation[1] = 'e';
  Orientation[2] = 'u';
}


/**
  * @brief  Get the register value from accelerometer
  * @param  Reg address to be read
  * @param  Data pointer where the value is written to
  * @retval None
  */
void BSP_SENSOR_ACC_Read_Register(uint8_t Reg, uint8_t *Data)
{
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Read_Register(IKS01A2_LSM6DSL_0, Reg, Data);
#elif (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Read_Register(IKS01A2_LSM303AGR_ACC_0, Reg, Data);
#endif
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Get the register value from gyroscope
  * @param  Reg address to be read
  * @param  Data pointer where the value is written to
  * @retval None
  */
void BSP_SENSOR_GYR_Read_Register(uint8_t Reg, uint8_t *Data)
{
  (void)IKS01A2_MOTION_SENSOR_Read_Register(IKS01A2_LSM6DSL_0, Reg, Data);
}
#endif

/**
  * @brief  Get the register value from magnetometer
  * @param  Reg address to be read
  * @param  Data pointer where the value is written to
  * @retval None
  */
void BSP_SENSOR_MAG_Read_Register(uint8_t Reg, uint8_t *Data)
{
  (void)IKS01A2_MOTION_SENSOR_Read_Register(IKS01A2_LSM303AGR_MAG_0, Reg, Data);
}

/**
  * @brief  Set the register value in accelerometer
  * @param  Reg address to be read
  * @param  Data value to be written
  * @retval None
  */
void BSP_SENSOR_ACC_Write_Register(uint8_t Reg, uint8_t Data)
{
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Write_Register(IKS01A2_LSM6DSL_0, Reg, Data);

#elif (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Write_Register(IKS01A2_LSM303AGR_ACC_0, Reg, Data);
#endif
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Set the register value in gyroscope
  * @param  Reg address to be read
  * @param  Data value to be written
  * @retval None
  */
void BSP_SENSOR_GYR_Write_Register(uint8_t Reg, uint8_t Data)
{
  (void)IKS01A2_MOTION_SENSOR_Write_Register(IKS01A2_LSM6DSL_0, Reg, Data);
}
#endif

/**
  * @brief  Set the register value in magnetometer
  * @param  Reg address to be read
  * @param  Data value to be written
  * @retval None
  */
void BSP_SENSOR_MAG_Write_Register(uint8_t Reg, uint8_t Data)
{
  (void)IKS01A2_MOTION_SENSOR_Write_Register(IKS01A2_LSM303AGR_MAG_0, Reg, Data);
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Set DRDY interrupt mode for accelerometer
  * @param  Mode Mode to be set (1 means pulsed mode otherwise latched mode)
  * @retval None
  */
void BSP_SENSOR_ACC_SetDRDYMode(uint8_t Mode)
{
  if (Mode == 1)
  {
    (void)IKS01A2_MOTION_SENSOR_DRDY_Set_Mode(IKS01A2_LSM6DSL_0, LSM6DSL_DRDY_PULSED);
  }
  else
  {
    (void)IKS01A2_MOTION_SENSOR_DRDY_Set_Mode(IKS01A2_LSM6DSL_0, LSM6DSL_DRDY_LATCHED);
  }
}
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Set DRDY interrupt mode for gyroscope
  * @param  Mode Mode to be set (1 means pulsed mode otherwise latched mode)
  * @retval None
  */
void BSP_SENSOR_GYR_SetDRDYMode(uint8_t Mode)
{
  if (Mode == 1)
  {
    (void)IKS01A2_MOTION_SENSOR_DRDY_Set_Mode(IKS01A2_LSM6DSL_0, LSM6DSL_DRDY_PULSED);
  }
  else
  {
    (void)IKS01A2_MOTION_SENSOR_DRDY_Set_Mode(IKS01A2_LSM6DSL_0, LSM6DSL_DRDY_LATCHED);
  }
}
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Enables/Disables DRDY signal from accelerometer
  * @param  Enable Define if DRDY signal is enable or disabled
  * @retval None
  */
void BSP_SENSOR_ACC_SetDRDYInt(uint8_t Enable)
{
  uint8_t reg = 0;

  (void)IKS01A2_MOTION_SENSOR_Read_Register(IKS01A2_LSM6DSL_0, LSM6DSL_INT1_CTRL, &reg);

  if (Enable == 1)
  {
    reg = reg | 0x01;
  }
  else
  {
    reg = reg & ~0x01;
  }

  (void)IKS01A2_MOTION_SENSOR_Write_Register(IKS01A2_LSM6DSL_0, LSM6DSL_INT1_CTRL, reg);
}
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Enables/Disables DRDY signal from gyroscope
  * @param  Enable Define if DRDY signal is enable or disabled
  * @retval None
  */
void BSP_SENSOR_GYR_SetDRDYInt(uint8_t Enable)
{
  uint8_t reg = 0;

  (void)IKS01A2_MOTION_SENSOR_Read_Register(IKS01A2_LSM6DSL_0, LSM6DSL_INT1_CTRL, &reg);

  if (Enable == 1)
  {
    reg = reg | 0x02;
  }
  else
  {
    reg = reg & ~0x02;
  }

  (void)IKS01A2_MOTION_SENSOR_Write_Register(IKS01A2_LSM6DSL_0, LSM6DSL_INT1_CTRL, reg);
}
#endif

/**
  * @brief  Get DRDY status from acceleromter
  * @param  Status DRDY bit status
  * @retval None
  */
void BSP_SENSOR_ACC_GetDRDYStatus(uint8_t *Status)
{
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Get_DRDY_Status(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, Status);
#elif (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Get_DRDY_Status(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO, Status);
#endif
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Reads data from FSM
  * @param  Data pointer where the value is written to
  * @retval None
  */
void BSP_ACC_GYR_Read_FSM_Data(uint8_t *Data)
{
  /* FSM not available on LSM6DSL */
}


/**
  * @brief  Reads data from MLC
  * @param  Data pointer where the value is written to
  * @retval None
  */
void BSP_ACC_GYR_Read_MLC_Data(uint8_t *Data)
{
  /* MLC not available on LSM6DSL */
}
#endif
