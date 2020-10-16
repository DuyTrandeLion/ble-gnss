/**
 ******************************************************************************
 * @file    iks01a2_mems_control.c
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

#include "iks01a2_mems_control.h"

/**
  * @brief  Initializes accelerometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_ACC_Init(void)
{
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO);
#elif (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);
#endif
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Initializes gyroscope
  * @param  None
  * @retval None
  */
void BSP_SENSOR_GYR_Init(void)
{
  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_GYRO);
}
#endif

/**
  * @brief  Initializes magnetometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_MAG_Init(void)
{
  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
}

/**
  * @brief  Enables accelerometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_ACC_Enable(void)
{
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_ACCELERO);
#elif (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);
#endif
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Enables gyroscope
  * @param  None
  * @retval None
  */
void BSP_SENSOR_GYR_Enable(void)
{
  (void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_GYRO);
}
#endif

/**
  * @brief  Enables magnetometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_MAG_Enable(void)
{
  (void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
}

/**
  * @brief  Disables accelerometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_ACC_Disable(void)
{
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Disable(IKS01A2_LSM6DSL_0, MOTION_ACCELERO);
#elif (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_Disable(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);
#endif
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Disables gyroscope
  * @param  None
  * @retval None
  */
void BSP_SENSOR_GYR_Disable(void)
{
  (void)IKS01A2_MOTION_SENSOR_Disable(IKS01A2_LSM6DSL_0, MOTION_GYRO);
}
#endif

/**
  * @brief  Disables magnetometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_MAG_Disable(void)
{
  (void)IKS01A2_MOTION_SENSOR_Disable(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
}

/**
  * @brief  Get accelerometer data
  * @param  Axes pointer to axes data structure
  * @retval None
  */
void BSP_SENSOR_ACC_GetAxes(IKS01A2_MOTION_SENSOR_Axes_t *Axes)
{
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, Axes);
#elif (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO, Axes);
#endif
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Get gyroscope data
  * @param  Axes pointer to axes data structure
  * @retval None
  */
void BSP_SENSOR_GYR_GetAxes(IKS01A2_MOTION_SENSOR_Axes_t *Axes)
{
  (void)IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM6DSL_0, MOTION_GYRO, Axes);
}
#endif

/**
  * @brief  Get magnetometer data
  * @param  Axes pointer to axes data structure
  * @retval None
  */
void BSP_SENSOR_MAG_GetAxes(IKS01A2_MOTION_SENSOR_Axes_t *Axes)
{
  (void)IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO, Axes);
}

/**
  * @brief  Set output data rate for accelerometer
  * @param  Odr Output Data Rate value to be set
  * @retval None
  */
void BSP_SENSOR_ACC_SetOutputDataRate(float Odr)
{
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, Odr);
#elif (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO, Odr);
#endif
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Set output data rate for gyroscope
  * @param  Odr Output Data Rate value to be set
  * @retval None
  */
void BSP_SENSOR_GYR_SetOutputDataRate(float Odr)
{
  (void)IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM6DSL_0, MOTION_GYRO, Odr);
}
#endif

/**
  * @brief  Set output data rate for magnetometer
  * @param  Odr Output Data Rate value to be set
  * @retval None
  */
void BSP_SENSOR_MAG_SetOutputDataRate(float Odr)
{
  (void)IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO, Odr);
}

/**
  * @brief  Get output data rate for accelerometer
  * @param  Odr Output Data Rate value
  * @retval None
  */
void BSP_SENSOR_ACC_GetOutputDataRate(float *Odr)
{
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_GetOutputDataRate(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, Odr);
#elif (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_GetOutputDataRate(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO, Odr);
#endif
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Get output data rate for gyroscope
  * @param  Odr Output Data Rate value
  * @retval None
  */
void BSP_SENSOR_GYR_GetOutputDataRate(float *Odr)
{
  (void)IKS01A2_MOTION_SENSOR_GetOutputDataRate(IKS01A2_LSM6DSL_0, MOTION_GYRO, Odr);
}
#endif

/**
  * @brief  Get output data rate for magnetometer
  * @param  Odr Output Data Rate value
  * @retval None
  */
void BSP_SENSOR_MAG_GetOutputDataRate(float *Odr)
{
  (void)IKS01A2_MOTION_SENSOR_GetOutputDataRate(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO, Odr);
}

/**
  * @brief  Set full scale for acclerometer
  * @param  Fullscale Fullscale value to be set
  * @retval None
  */
void BSP_SENSOR_ACC_SetFullScale(int32_t Fullscale)
{
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_SetFullScale(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, Fullscale);
#elif (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_SetFullScale(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO, Fullscale);
#endif
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Set full scale for gyroscope
  * @param  Fullscale Fullscale value to be set
  * @retval None
  */
void BSP_SENSOR_GYR_SetFullScale(int32_t Fullscale)
{
  (void)IKS01A2_MOTION_SENSOR_SetFullScale(IKS01A2_LSM6DSL_0, MOTION_GYRO, Fullscale);
}
#endif

/**
  * @brief  Set full scale for magnetometer
  * @param  Fullscale Fullscale value to be set
  * @retval None
  */
void BSP_SENSOR_MAG_SetFullScale(int32_t Fullscale)
{
  (void)IKS01A2_MOTION_SENSOR_SetFullScale(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO, Fullscale);
}

/**
  * @brief  Get full scale for acclerometer
  * @param  Fullscale Fullscale value
  * @retval None
  */
void BSP_SENSOR_ACC_GetFullScale(int32_t *Fullscale)
{
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_GetFullScale(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, Fullscale);
#elif (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  (void)IKS01A2_MOTION_SENSOR_GetFullScale(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO, Fullscale);
#endif
}

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
/**
  * @brief  Get full scale for gyroscope
  * @param  Fullscale Fullscale value
  * @retval None
  */
void BSP_SENSOR_GYR_GetFullScale(int32_t *Fullscale)
{
  (void)IKS01A2_MOTION_SENSOR_GetFullScale(IKS01A2_LSM6DSL_0, MOTION_GYRO, Fullscale);
}
#endif

/**
  * @brief  Get full scale for magnetometer
  * @param  Fullscale Fullscale value
  * @retval None
  */
void BSP_SENSOR_MAG_GetFullScale(int32_t *Fullscale)
{
  (void)IKS01A2_MOTION_SENSOR_GetFullScale(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO, Fullscale);
}
