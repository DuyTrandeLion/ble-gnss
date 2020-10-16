/**
 *******************************************************************************
 * @file    demo_serial.h
 * @author  MEMS Software Solutions Team
 * @brief   Header for demo_serial.c
 *******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under Software License Agreement
 * SLA0077, (the "License"). You may not use this file except in compliance
 * with the License. You may obtain a copy of the License at:
 *
 *     www.st.com/content/st_com/en/search.html#q=SLA0077-t=keywords-page=1
 *
 *******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEMO_SERIAL_H
#define DEMO_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef USE_USB_OTG_HS
#define TMsg_MaxLen             512
#else
#define TMsg_MaxLen             256
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint8_t subsec;
  float pressure;
  float humidity;
  float temperature;
  int32_t acceleration_x_mg;
  int32_t acceleration_y_mg;
  int32_t acceleration_z_mg;
  int32_t angular_rate_x_mdps;
  int32_t angular_rate_y_mdps;
  int32_t angular_rate_z_mdps;
  int32_t magnetic_field_x_mgauss;
  int32_t magnetic_field_y_mgauss;
  int32_t magnetic_field_z_mgauss;
} offline_data_t;

/**
 * @brief  Serial message structure definition
 */
typedef struct
{
  uint32_t Len;
  uint8_t Data[TMsg_MaxLen];
} TMsg;

/* Exported defines ----------------------------------------------------------*/
#define SENDER_UART  0x01
#define SENDER_USB   0x02
#define SENDER_SPI   0x03

#define DEV_ADDR  50U
#define I2C_DATA_MAX_LENGTH_BYTES  16
#define OFFLINE_DATA_SIZE  8

/* Enable sensor masks */
#define PRESSURE_SENSOR       0x00000001U
#define TEMPERATURE_SENSOR    0x00000002U
#define HUMIDITY_SENSOR       0x00000004U
#define UV_SENSOR             0x00000008U /* for future use */
#define ACCELEROMETER_SENSOR  0x00000010U
#define GYROSCOPE_SENSOR      0x00000020U
#define MAGNETIC_SENSOR       0x00000040U

#define STREAMING_MSG_LENGTH  129

#define REQUIRED_DATA  (ACCELEROMETER_SENSOR + MAGNETIC_SENSOR)

#define TMsg_EOF                0xF0
#define TMsg_BS                 0xF1
#define TMsg_BS_EOF             0xF2

/* Exported variables --------------------------------------------------------*/
extern volatile uint8_t DataLoggerActive;
extern volatile uint32_t SensorsEnabled;
extern volatile uint8_t SensorReadRequest;
extern uint8_t UseOfflineData;
extern offline_data_t OfflineData[OFFLINE_DATA_SIZE];
extern int OfflineDataReadIndex;
extern int OfflineDataWriteIndex;
extern int OfflineDataCount;
extern uint32_t AlgoFreq;

/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* DEMO_SERIAL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
