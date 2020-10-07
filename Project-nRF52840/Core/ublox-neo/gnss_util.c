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
 * @file GNSS_Util.c
 * @date Feb 14th 2020
 * @version  0.9.0
 *
 */

#include "gnss_type.h"
#include "gnss_util.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h" 


void gnss_parse_rmc(char *str_rmc, rmc_t *rmc_st)
{
    uint32_t raw_int_time, raw_date; 

    /* Message ID parsing */
    char *token = strtok(str_rmc, ",");
    memcpy(rmc_st->id, token + 1, 5);

    /* Time parsing */
    token = strtok(NULL, ",");
    /* Convert string to time */
    raw_int_time = atoi(token);
    NRF_LOG_INFO("Raw int time %d", raw_int_time);
    rmc_st->time.seconds = raw_int_time % 100;
    raw_int_time         = raw_int_time / 100;
    rmc_st->time.minutes = raw_int_time % 100;
    raw_int_time         = raw_int_time / 100;
    rmc_st->time.hours   = raw_int_time % 100;

    /* Status parsing */
    token = strtok(NULL, ",");
    if (*(token) == 'A')
    {
	rmc_st->status = A;
    }
    else
    {
	rmc_st->status = V;
    }
    
    /* Latitude parsing */
    token = strtok(NULL, ",");
    rmc_st->latitude	= strtof(token, NULL);
    NRF_LOG_INFO("Latitude " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(rmc_st->latitude));

    /* North South parsing */
    token = strtok(NULL, ",");
    if (*(token) == 'N')
    {
	rmc_st->n_s = NORTH;
    }
    else if (*(token) == 'S')
    {
	rmc_st->n_s = SOUTH;
    }

    /* Longitude parsing */
    token = strtok(NULL, ",");
    rmc_st->longitude = strtof(token, NULL);
    NRF_LOG_INFO("Longitude " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(rmc_st->longitude));

    /* East West parsing */
    token = strtok(NULL, ",");
    if (*(token) == 'E')
    {
	rmc_st->e_w = EAST;
    }
    else if (*(token) == 'W')
    {
	rmc_st->e_w = WEST;
    }

    /* Speed parsing */
    token = strtok(NULL, ",");
    rmc_st->speed = atof(token);

    /* Course parsing - not available in RMC of NEO-M8N */
    /*
    token = strtok(NULL, ",");
    rmc_st->course = atof(token);
    */

    /* Date parsing */
    token = strtok(NULL, ",");
    /* Convert string to date */
    raw_date = atoi(token);
    NRF_LOG_INFO("Raw date %d", raw_date);
    rmc_st->date.year  = raw_date % 100;
    raw_date           = raw_date / 100;
    rmc_st->date.month = raw_date % 100;
    raw_date           = raw_date / 100;
    rmc_st->date.date  = raw_date % 100;
    
    /* Mode parsing */
    token = strtok(NULL, "*");
    switch (token[0])
    {
        case 'A':
        {
            rmc_st->mode = Au;
            break;
        }

        case 'D':
        {
            rmc_st->mode = D;
            break;
        }

        case 'R':
        {
            rmc_st->mode = R;
            break;
        }

        case 'F':
        {
            rmc_st->mode = F;
            break;
        }

        case 'E':
        {
            rmc_st->mode = E;
            break;
        }

        case 'N':
        {
            rmc_st->mode = N;
            break;
        }

        default:
        {
            rmc_st->mode = N;
            break;
        }
    }
}

void gnss_parse_gns(char *str_gns, gns_t *gns_st)
{
    uint32_t raw_int_time; 

    /* Message ID parsing */
    char *token = strtok(str_gns, ",");
    memcpy(gns_st->id, token + 1, 5);

    /* Time parsing */
    token = strtok(NULL, ",");
    /* Convert string to time */
    raw_int_time = atoi(token);
    NRF_LOG_INFO("Raw int time %d", raw_int_time);
    gns_st->time.seconds = raw_int_time % 100;
    raw_int_time         = raw_int_time / 100;
    gns_st->time.minutes = raw_int_time % 100;
    raw_int_time         = raw_int_time / 100;
    gns_st->time.hours   = raw_int_time % 100;

    /* Latitude parsing */
    token = strtok(NULL, ",");
    gns_st->latitude	= strtof(token, NULL);
    NRF_LOG_INFO("Latitude " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(gns_st->latitude));

    /* North South parsing */
    token = strtok(NULL, ",");
    if (*(token) == 'N')
    {
	gns_st->n_s = NORTH;
    }
    else if (*(token) == 'S')
    {
	gns_st->n_s = SOUTH;
    }

    /* Longitude parsing */
    token = strtok(NULL, ",");
    gns_st->longitude = strtof(token, NULL);
    NRF_LOG_INFO("Longitude " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(gns_st->longitude));

    /* East West parsing */
    token = strtok(NULL, ",");
    if (*(token) == 'E')
    {
	gns_st->e_w = EAST;
    }
    else if (*(token) == 'W')
    {
	gns_st->e_w = WEST;
    }

    /* Mode parsing */
    token = strtok(NULL, ",");
    switch (token[0])
    {
        case 'A':
        {
            if (strlen(token) > 1)
            {
                if ('A' == token[1])
                {
                    gns_st->mode = AA;
                }
                else if ('D' == token[1])
                {
                    gns_st->mode = AD;
                }
                else
                {
                    gns_st->mode = N;
                }
            }
            else
            {
                gns_st->mode = Au;
            }          
            break;
        }

        case 'D':
        {
            if (strlen(token) > 1)
            {
                if ('A' == token[1])
                {
                    gns_st->mode = AD;
                }
                else if ('D' == token[1])
                {
                    gns_st->mode = DD;
                }
                else
                {
                    gns_st->mode = N;
                }
            }
            else
            {
                gns_st->mode = D;
            }  
            break;
        }

        case 'E':
        {
            gns_st->mode = E;
            break;
        }

        case 'N':
        {
            gns_st->mode = N;
            break;
        }

        default:
        {
            gns_st->mode = N;
            break;
        }
    }

    /* Number of satellites used parsing */
    token = strtok(NULL, ",");
    gns_st->num_of_satellites = atoi(token);
    NRF_LOG_INFO("Number of satellites %d", gns_st->num_of_satellites);

    /* Horizontal Dilution of Precision parsing */
    token = strtok(NULL, ",");
    gns_st->HDOP = strtof(token, NULL);
    NRF_LOG_INFO("Horizontal Dilution of Precision " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(gns_st->HDOP));

    /* Altitude parsing */
    token = strtok(NULL, ",");
    gns_st->altitude = strtof(token, NULL);
    NRF_LOG_INFO("Altitude " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(gns_st->altitude));

    /* Geoid separation parsing */
    token = strtok(NULL, ",");
    gns_st->geoid_separation = strtof(token, NULL);
    NRF_LOG_INFO("Geoid separation " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(gns_st->geoid_separation));

    /* ID of station providing differential corrections  parsing */
    token = strtok(NULL, ",");
    gns_st->differential_station_id = atoi(token);
    NRF_LOG_INFO("Station ID %d", gns_st->differential_station_id);
}

void gnss_parse_gga(char *str_gga, gga_t *gga_st)
{

}

void gnss_parse_gll(char *str_gll, gll_t *gll_st)
{
    uint32_t raw_int_time;

    /* Message ID parsing */
    char *token = strtok(str_gll, ",");
    memcpy(gll_st->id, token + 1, 5);

    /* Latitude parsing */
    token = strtok(NULL, ",");
    gll_st->latitude	= strtof(token, NULL);
    NRF_LOG_INFO("Latitude " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(gll_st->latitude));

    /* North South parsing */
    token = strtok(NULL, ",");
    if (*(token) == 'N')
    {
	gll_st->n_s = NORTH;
    }
    else if (*(token) == 'S')
    {
	gll_st->n_s = SOUTH;
    }

    /* Longitude parsing */
    token = strtok(NULL, ",");
    gll_st->longitude = strtof(token, NULL);
    NRF_LOG_INFO("Longitude " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(gll_st->longitude));

    /* East West parsing */
    token = strtok(NULL, ",");
    if (*(token) == 'E')
    {
	gll_st->e_w = EAST;
    }
    else if (*(token) == 'W')
    {
	gll_st->e_w = WEST;
    }

    /* Time parsing */
    token = strtok(NULL, ",");
    /* Convert string to time */
    raw_int_time = atoi(token);
    NRF_LOG_INFO("Raw int time %d", raw_int_time);
    gll_st->time.seconds = raw_int_time % 100;
    raw_int_time         = raw_int_time / 100;
    gll_st->time.minutes = raw_int_time % 100;
    raw_int_time         = raw_int_time / 100;
    gll_st->time.hours   = raw_int_time % 100;

    /* Status parsing */
    token = strtok(NULL, ",");
    if (*(token) == 'A')
    {
	gll_st->status = A;
    }
    else
    {
	gll_st->status = V;
    }

    /* Mode parsing */
    token = strtok(NULL, "*");
    switch (token[0])
    {
        case 'A':
        {
            gll_st->mode = Au;
            break;
        }

        case 'D':
        {
            gll_st->mode = D;
            break;
        }

        case 'E':
        {
            gll_st->mode = E;
            break;
        }

        case 'N':
        {
            gll_st->mode = N;
            break;
        }

        default:
        {
            gll_st->mode = N;
            break;
        }
    }
}
