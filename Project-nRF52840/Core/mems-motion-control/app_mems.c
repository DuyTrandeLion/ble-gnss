/**
  ******************************************************************************
  * File Name          : app_mems.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.8.0.0 instances.
  ******************************************************************************
  *
  * COPYRIGHT 2020 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_mems.h"
#include <stdio.h>

#include "motion_mc_manager.h"
#include "motion_ec_manager.h"

#include "iks01a2_mems_control.h"
#include "iks01a2_mems_control_ex.h"

#include "demo_serial.h"
#include "com_bus.h"
#include "nrf_delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DWT_LAR_KEY  0xC5ACCE55 /* DWT register unlock key */
#define ALGO_FREQ  50U /* Algorithm frequency >= 50Hz */
#define ALGO_FREQ_MS  (20)
#define ACC_ODR  ((float)ALGO_FREQ)
#define ACC_FS  2 /* FS = <-2g, 2g> */
#define MAG_ODR  ((float)ALGO_FREQ)

/* Public variables ----------------------------------------------------------*/
volatile uint8_t DataLoggerActive = 0;
volatile uint32_t SensorsEnabled = 0;
char LibVersion[35];
int LibVersionLen;
volatile uint8_t SensorReadRequest = 0;
uint8_t UseOfflineData = 0;
offline_data_t OfflineData[OFFLINE_DATA_SIZE];
int OfflineDataReadIndex = 0;
int OfflineDataWriteIndex = 0;
int OfflineDataCount = 0;
uint32_t AlgoFreq = ALGO_FREQ;

/* Extern variables ----------------------------------------------------------*/
static const nrf_drv_timer_t m_mems_timer = NRF_DRV_TIMER_INSTANCE(MEMS_TIMER_INSTANCE);
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static MOTION_SENSOR_Axes_t AccValue;
static MOTION_SENSOR_Axes_t GyrValue;
static MOTION_SENSOR_Axes_t MagValue;
static const uint32_t ReportInterval = 1000U / (float)ALGO_FREQ; /* Algorithm report interval [ms] between 10 and 50 ms */
static volatile uint32_t TimeStamp = 0;
static MOTION_SENSOR_Axes_t MagValueComp; /* Compensated magnetometer data [mGauss] */

/* Private function prototypes -----------------------------------------------*/
static void MX_ECompass_Init(void);
static void MX_ECompass_Process(void);
static void MC_Data_Handler(TMsg *Msg);
static void EC_Data_Handler(TMsg *Msg);
static void Init_Sensors(void);
static void Accelero_Sensor_Handler(TMsg *Msg);
static void Gyro_Sensor_Handler(TMsg *Msg);
static void Magneto_Sensor_Handler(TMsg *Msg);
static void TIM_Init();
static void DWT_Init(void);
static void DWT_Start(void);
static uint32_t DWT_Stop(void);

uint8_t device_id;

/**
 * @brief  Build an array from the float
 * @param  Dest destination
 * @param  Data source
 * @retval None
 */
static void FloatToArray(uint8_t *Dest, float Data)
{
  (void)memcpy(Dest, (void *)&Data, 4);
}

/**
 * @brief  Build an array from the uint32_t (LSB first)
 * @param  Dest destination
 * @param  Source source
 * @param  Len number of bytes
 * @retval None
 */
static void Serialize_s32(uint8_t *Dest, int32_t Source, uint32_t Len)
{
  uint32_t i;
  uint32_t source_uint32;

  for (i = 0; i < Len; i++)
  {
    source_uint32 = (uint32_t)Source;
    Dest[i] = (uint8_t)(source_uint32 & 0xFFU);
    source_uint32 >>= 8;
    Source = (int32_t)source_uint32;
  }
}

void MX_MEMS_Init(void)
{
  /* Initialize ECompass. */
  /* Configure Timer to run with desired algorithm frequency */
  TIM_Init();

  /* Initialize (disabled) sensors */
  Init_Sensors();

  /* Magnetometer Calibration API initialization function */
  MotionMC_manager_init((int)ReportInterval, 1);

  /* E-Compass API initialization function */
  MotionEC_manager_init((float)ALGO_FREQ);

  /* OPTIONAL */
  /* Get library version */
  MotionEC_manager_get_version(LibVersion, &LibVersionLen);

  DWT_Init();
}

/*
 * LM background task
 */
void MX_MEMS_Process(void)
{
    /* Process ECompass. */
    static TMsg msg_dat;
    static TMsg msg_cmd;

    if (SensorReadRequest == 1U)
    {
        SensorReadRequest = 0;

        /* Acquire data from enabled sensors and fill Msg stream */
        Accelero_Sensor_Handler(&msg_dat);
        Gyro_Sensor_Handler(&msg_dat);
        Magneto_Sensor_Handler(&msg_dat);

        /* Magnetometer Calibration specific part */
        MC_Data_Handler(&msg_dat);

        /* E-Compass specific part */
        EC_Data_Handler(&msg_dat);

        if (UseOfflineData == 1U)
        {
            OfflineDataCount--;
            if (OfflineDataCount < 0)
            {
                OfflineDataCount = 0;
            }

            OfflineDataReadIndex++;
            if (OfflineDataReadIndex >= OFFLINE_DATA_SIZE)
            {
                OfflineDataReadIndex = 0;
            }

            if (OfflineDataCount > 0)
            {
                SensorReadRequest = 1;
            }
        }
    }
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Period elapsed callback
 * @param  htim pointer to a TIM_HandleTypeDef structure that contains
 *              the configuration information for TIM module.
 * @retval None
 */
void timer_mems_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
        {
            SensorReadRequest = 1;
            break;
        }

        default:
            //Do nothing.
            break;
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void Init_Sensors(void)
{
  BSP_SENSOR_ACC_Init();
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  BSP_SENSOR_GYR_Init();
#endif
  BSP_SENSOR_MAG_Init();

  BSP_SENSOR_ACC_SetOutputDataRate(ACC_ODR);
  BSP_SENSOR_ACC_SetFullScale(ACC_FS);
  BSP_SENSOR_MAG_SetOutputDataRate(MAG_ODR);
}


/**
 * @brief  Magnetometer Calibration data handler
 * @param  Msg the Magnetometer Calibration data part of the stream
 * @retval None
 */
static void MC_Data_Handler(TMsg *Msg)
{
  MMC_Input_t data_in;
  MMC_Output_t data_out;

  if ((SensorsEnabled & MAGNETIC_SENSOR) != MAGNETIC_SENSOR)
  {
    return;
  }

  /* Convert magnetometer data from [mGauss] to [uT] */
  data_in.Mag[0] = (float)MagValue.x / 10.0f;
  data_in.Mag[1] = (float)MagValue.y / 10.0f;
  data_in.Mag[2] = (float)MagValue.z / 10.0f;

  /* Time stamp [ms] */
  data_in.TimeStamp = (int)(TimeStamp * ReportInterval);

  /* Run Magnetometer Calibration algorithm */
  MotionMC_manager_update(&data_in);

  /* Get the magnetometer compensation for hard/soft iron */
  MotionMC_manager_get_params(&data_out);

  /* Do hard & soft iron calibration */
  MotionMC_manager_compensate(&MagValue, &MagValueComp);

  /* Calibrated data */
  /* NOTE: Magnetometer data unit is [mGauss] */
  Serialize_s32(&Msg->Data[43], MagValueComp.x, 4);
  Serialize_s32(&Msg->Data[47], MagValueComp.y, 4);
  Serialize_s32(&Msg->Data[51], MagValueComp.z, 4);

  /* Calibration quality */
  Msg->Data[124] = (uint8_t)data_out.CalQuality;
}

/**
 * @brief  E-Compass data handler
 * @param  Msg the E-Compass data part of the stream
 * @retval None
 */
static void EC_Data_Handler(TMsg *Msg)
{
  uint32_t elapsed_time_us = 0U;
  MEC_input_t  data_in;
  MEC_output_t data_out;

  if ((SensorsEnabled & ACCELEROMETER_SENSOR) != ACCELEROMETER_SENSOR)
  {
    return;
  }

  if ((SensorsEnabled & MAGNETIC_SENSOR) != MAGNETIC_SENSOR)
  {
    return;
  }

  /* Do sensor orientation transformation */
  MotionEC_manager_transform_orientation(&AccValue, &MagValueComp, data_in.acc, data_in.mag);

  /* Convert raw accelerometer data from [mg] to [g] */
  data_in.acc[0] = data_in.acc[0] / 1000.0f; /* East */
  data_in.acc[1] = data_in.acc[1] / 1000.0f; /* North */
  data_in.acc[2] = data_in.acc[2] / 1000.0f; /* Up */

  /* Convert compensated magnetometer data from [mGauss] to [uT / 50], [mGauss / 5] */
  data_in.mag[0] = data_in.mag[0] / 5.0f; /* East */
  data_in.mag[1] = data_in.mag[1] / 5.0f; /* North */
  data_in.mag[2] = data_in.mag[2] / 5.0f; /* Up */

  /* Delta time [s] */
  data_in.deltatime_s = (float)ReportInterval / 1000.0f;

  /* Run E-Compass algorithm */
  DWT_Start();
  MotionEC_manager_run(&data_in, &data_out);
  elapsed_time_us = DWT_Stop();

  /* Write data to output stream */
  FloatToArray(&Msg->Data[55], data_out.quaternion[0]);
  FloatToArray(&Msg->Data[59], data_out.quaternion[1]);
  FloatToArray(&Msg->Data[63], data_out.quaternion[2]);
  FloatToArray(&Msg->Data[67], data_out.quaternion[3]);

  FloatToArray(&Msg->Data[71], data_out.euler[0]);
  FloatToArray(&Msg->Data[75], data_out.euler[1]);
  FloatToArray(&Msg->Data[79], data_out.euler[2]);

  FloatToArray(&Msg->Data[83], data_out.i_gyro[0]);
  FloatToArray(&Msg->Data[87], data_out.i_gyro[1]);
  FloatToArray(&Msg->Data[91], data_out.i_gyro[2]);

  FloatToArray(&Msg->Data[95], data_out.gravity[0]);
  FloatToArray(&Msg->Data[99], data_out.gravity[1]);
  FloatToArray(&Msg->Data[103], data_out.gravity[2]);

  FloatToArray(&Msg->Data[107], data_out.linear[0]);
  FloatToArray(&Msg->Data[111], data_out.linear[1]);
  FloatToArray(&Msg->Data[115], data_out.linear[2]);

  float heading;
  int heading_valid;

  MotionEC_manager_calc_heading(data_out.quaternion, &heading, &heading_valid);

  FloatToArray(&Msg->Data[119], heading);
  Msg->Data[123] = (uint8_t)heading_valid;

  Serialize_s32(&Msg->Data[125], (int32_t)elapsed_time_us, 4);
}

/**
 * @brief  Handles the ACC axes data getting/sending
 * @param  Msg the ACC part of the stream
 * @retval None
 */
static void Accelero_Sensor_Handler(TMsg *Msg)
{
  if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
  {
    if (UseOfflineData == 1)
    {
      AccValue.x = OfflineData[OfflineDataReadIndex].acceleration_x_mg;
      AccValue.y = OfflineData[OfflineDataReadIndex].acceleration_y_mg;
      AccValue.z = OfflineData[OfflineDataReadIndex].acceleration_z_mg;
    }
    else
    {
      BSP_SENSOR_ACC_GetAxes(&AccValue);
    }

    Serialize_s32(&Msg->Data[19], (int32_t)AccValue.x, 4);
    Serialize_s32(&Msg->Data[23], (int32_t)AccValue.y, 4);
    Serialize_s32(&Msg->Data[27], (int32_t)AccValue.z, 4);
  }
}

/**
 * @brief  Handles the GYR axes data getting/sending
 * @param  Msg the GYR part of the stream
 * @retval None
 */
static void Gyro_Sensor_Handler(TMsg *Msg)
{
  if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
  {
    if (UseOfflineData == 1)
    {
      GyrValue.x = OfflineData[OfflineDataReadIndex].angular_rate_x_mdps;
      GyrValue.y = OfflineData[OfflineDataReadIndex].angular_rate_y_mdps;
      GyrValue.z = OfflineData[OfflineDataReadIndex].angular_rate_z_mdps;
    }
    else
    {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
      BSP_SENSOR_GYR_GetAxes(&GyrValue);
#endif
    }

    Serialize_s32(&Msg->Data[31], GyrValue.x, 4);
    Serialize_s32(&Msg->Data[35], GyrValue.y, 4);
    Serialize_s32(&Msg->Data[39], GyrValue.z, 4);
  }
}

/**
 * @brief  Handles the MAG axes data getting/sending
 * @param  Msg the MAG part of the stream
 * @retval None
 */
static void Magneto_Sensor_Handler(TMsg *Msg)
{
  if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
  {
    if (UseOfflineData == 1)
    {
     MagValue.x = OfflineData[OfflineDataReadIndex].magnetic_field_x_mgauss;
     MagValue.y = OfflineData[OfflineDataReadIndex].magnetic_field_y_mgauss;
     MagValue.z = OfflineData[OfflineDataReadIndex].magnetic_field_z_mgauss;
    }
    else
    {
      BSP_SENSOR_MAG_GetAxes(&MagValue);
    }

    Serialize_s32(&Msg->Data[43], MagValue.x, 4);
    Serialize_s32(&Msg->Data[47], MagValue.y, 4);
    Serialize_s32(&Msg->Data[51], MagValue.z, 4);
  }
}


/**
 * @brief  Timer configuration
 * @param  Freq the desired Timer frequency
 * @retval None
 */
static void TIM_Init(void)
{
    ret_code_t err_code;
    
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;

    err_code = nrf_drv_timer_init(&m_mems_timer, &timer_cfg, timer_mems_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_extended_compare(&m_mems_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   nrf_drv_timer_ms_to_ticks(&m_mems_timer, ALGO_FREQ_MS),
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
    nrf_drv_timer_enable(&m_mems_timer);
}

/**
 * @brief  Initialize DWT register for counting clock cycles purpose
 * @param  None
 * @retval None
 */
static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; /* Disable counter */
}

/**
 * @brief  Start counting clock cycles
 * @param  None
 * @retval None
 */
static void DWT_Start(void)
{
  DWT->CYCCNT = 0; /* Clear count of clock cycles */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* Enable counter */
}

/**
 * @brief  Stop counting clock cycles and calculate elapsed time in [us]
 * @param  None
 * @retval Elapsed time in [us]
 */
static uint32_t DWT_Stop(void)
{
  volatile uint32_t cycles_count = 0U;
  uint32_t system_core_clock_mhz = 0U;

  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; /* Disable counter */
  cycles_count = DWT->CYCCNT; /* Read count of clock cycles */

  /* Calculate elapsed time in [us] */
  system_core_clock_mhz = SystemCoreClock / 1000000U;
  return cycles_count / system_core_clock_mhz;
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
