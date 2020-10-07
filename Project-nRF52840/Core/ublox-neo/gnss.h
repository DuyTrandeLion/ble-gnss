#ifndef __GNSS_H__
#define __GNSS_H__

#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "UBX_GNSS.h"
#include "Miscellaneous.h"

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "boards.h"

#include "app_timer.h"
#include "nrf_gpio.h"
#include "nrf_drv_timer.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "app_uart.h"
#include "nrf_drv_twi.h"

#ifdef __cplusplus
extern "C" {
#endif


#define GNSS_UART_INSTANCE              0
#define GNSS_TWI_INSTANCE               0
#define GNSS_TIMER_INSTANCE             1
#define GNSS_UBX_TIMER_INSTANCE         2

#define GNSS_UART_RX_PIN                ARDUINO_0_PIN
#define GNSS_UART_TX_PIN                ARDUINO_1_PIN

#define GNSS_I2C_SDA_PIN                ARDUINO_A4_PIN
#define GNSS_I2C_SCL_PIN                ARDUINO_A5_PIN

#define INTERFACE_SELECT_PIN            ARDUINO_A0_PIN

#define UART_TIMER_STEP                 (25)   /* ms */
#define TWI_TIMER_STEP                  (100)  /* ms */
#define UART_TIMEOUT_PERIOD             (30)
#define UART_FAULT_DATA_CLEAR           (10)
#define TWI_PROCESS_DATA_PERIOD         (10)  /* TWI_TIMER_STEP */

#define MAX_RECEIVED_SENTENCE_SIZE      (UBXGNSS_NMEA_MAX_LENGTH)


#define GNSS_UART_TX_BUFF_SIZE		(512)
#define GNSS_UART_RX_BUFF_SIZE		(512)


extern UBXGNSS_Def_t GNSS_DEV;


void gnss_i2c_evt_handler(nrf_drv_twi_evt_t const * p_event,
                                           void * p_context);


void gnss_init();


#ifdef __cplusplus
}
#endif

#endif /* __GNSS_H__ */