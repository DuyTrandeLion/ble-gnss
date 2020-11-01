/*
 * MIT License
 *
 * Copyright (c) 2020 <Duy Lion Tran>. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the
 * Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @file GNSS_Type.h
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#ifndef __GNSS_TYPE_H__
#define __GNSS_TYPE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MINMEA_MAX_LENGTH               (128)

#define GNSS_UART_INTERFACE             (0)
#define GNSS_I2C_INTERFACE              (1)

typedef enum
{
    UBXGNSS_OK = 0,
    UBXGNSS_FAIL,
    UBXGNSS_CRC_FAIL,
    UBXGNSS_TIMEOUT,
    UBXGNSS_COMMAND_UNKNOWN,
    UBXGNSS_OUT_OF_RANGE,
    UBXGNSS_INVALID_ARG,
    UBXGNSS_INVALID_OPERATION,
    UBXGNSS_MEM_ERR,
    UBXGNSS_HW_ERR,
    UBXGNSS_DATA_SENT,
    UBXGNSS_DATA_RECEIVED,
    UBXGNSS_I2C_COMM_FAILURE
} UBXGNSS_State_t;


/* Registers */
#define UBX_SYNCH_1   0xB5
#define UBX_SYNCH_2   0x62


typedef enum
{
    MINMEA_INVALID  = -1,
    MINMEA_UNKNOWN  = 0,
    SOURCE_xxRMC,
    SOURCE_xxGGA,
    SOURCE_xxGSA,
    SOURCE_xxGLL,
    SOURCE_xxGST,
    SOURCE_xxGSV,
    SOURCE_xxVTG,
    SOURCE_xxZDA
}  data_source_t;


typedef enum
{
    GP = 0x00,		  /**< GPS, SBAS, QZSS.*/
    GL = 0x01,            /**< GLONASS.*/
    GA = 0x02,            /**< Galileo.*/
    GB = 0x03,            /**< BeiDou.*/
    COMBINED_GNSS = 0x04  /**< Any combination of GNSS.*/
} talker_id_t;


typedef enum
{
    A = 0,		  /**< Valid.*/
    V = 1                 /**< Invalid.*/
} gps_status_t;


typedef enum
{
    Au = 0,               /**< Autonomous.*/
    D  = 1,               /**< Differential.*/
    R  = 2,               /**< Fixed RTK.*/
    F  = 3,               /**< Float RTK.*/
    E  = 4,               /**< Dead Reckoning.*/
    AA = 5,               /**< Combined GNSS/dead reckoning fix */
    AD = 6,               /**< Combined GNSS/dead reckoning fix */
    DD = 7,               /**< Combined GNSS/dead reckoning fix */
    N  = 8                /**< None.*/
} gps_mode_indicator_t;


typedef enum
{
    NORTH = 0,
    SOUTH = 1
} northing_indicator_t;


typedef enum
{
    EAST = 0,
    WEST = 1
} easting_indicator_t;


typedef struct
{
    uint8_t date;
    uint8_t month;
    uint8_t year;
} gps_date_t;


typedef struct
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t hundredths;
} gps_time_t;


typedef struct
{
    char		  id[5];
    gps_time_t		  time;
    gps_status_t	  status;
    double		  latitude;
    northing_indicator_t  n_s;
    double		  longitude;
    easting_indicator_t	  e_w;
    float		  speed;
    /* float		  course; */      /* Course parsing - not available in RMC of NEO-M8N */
    gps_date_t		  date;
    gps_mode_indicator_t  mode;
    /* uint16_t		  check_sum; */   /* Update later */
} rmc_t;

typedef struct
{
    char		  id[5];
    gps_time_t		  time;
    double		  latitude;
    northing_indicator_t  n_s;
    double		  longitude;
    easting_indicator_t	  e_w;
    gps_mode_indicator_t  mode;
    uint8_t		  num_of_satellites;
    float		  HDOP;                     /**< Horizontal Dilution of Precision */
    float		  altitude;
    float		  geoid_separation;
    /* float                 differential_age; */   /* Not available in GNS of NEO-M8N */
    uint16_t              differential_station_id;
    /* gps_status_t	  status; */                /* Not available in GNS of NEO-M8N */
    /* uint16_t		  check_sum; */             /* Update later */
} gns_t;

typedef struct
{
    char		  id[5];
    gps_time_t		  time;
    float		  latitude;
    northing_indicator_t  n_s;
    float		  longitude;
    easting_indicator_t   e_w;
    uint8_t		  quality;
    uint8_t		  num_of_satellites;
    float		  HDOP;                     /**< Horizontal Dilution of Precision */
    float		  altitude;
    char		  altitude_unit;
    float		  geoid_separation;
    char		  separation_uint;
    uint32_t		  age_of_differential;
    uint32_t		  ref_station_if;
    /* uint16_t		  check_sum; */           /* Update later */
} gga_t;


typedef struct
{
    char		  id[5];
    float		  latitude;
    northing_indicator_t  n_s;
    float		  longitude;
    easting_indicator_t   e_w;
    gps_time_t		  time;
    gps_status_t	  status;
    gps_mode_indicator_t  mode;
    uint16_t		  check_sum;
} gll_t;


typedef enum
{
    UART_UBX_EVENT_TRANSMIT          = 0x00,
    UART_UBX_EVENT_RECEIVE           = 0x01,
    UART_UBX_EVENT_TRANSMIT_RECEIVE  = 0x02,
    UART_UBX_EVENT_ABORT_TRANSMIT    = 0x03,
    UART_UBX_EVENT_ABORT_RECEIVE     = 0x04,
} ADSADC_UART_Event_t;


typedef UBXGNSS_State_t (*UBXGNSS_UART_Handle_t)(ADSADC_UART_Event_t, uint8_t *, uint16_t, void *);

/**
 * Busy time counting handle callback.
 *
 * @param[in] uint32_t	Delay time
 * @retval true 		If the device is busy
 */
typedef uint8_t (*UBXGNSS_Delay_Handle_t)(uint32_t);


/** Ublox GNSS definition structure.  */
typedef struct
{
        uint8_t                 interface;
        uint8_t                 address;
	uint32_t		timeout;
	UBXGNSS_UART_Handle_t 	uartHandle;
	UBXGNSS_Delay_Handle_t 	delayHandle;
} UBXGNSS_Def_t;


#ifdef __cplusplus
}
#endif

#endif /* _GNSS_TYPE_H_ */