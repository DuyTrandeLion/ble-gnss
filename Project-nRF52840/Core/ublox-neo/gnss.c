#include "gnss.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


static const nrf_drv_timer_t GNSS_TIMER = NRF_DRV_TIMER_INSTANCE(GNSS_TIMER_INSTANCE);
static const nrf_drv_timer_t UBX_TIMER = NRF_DRV_TIMER_INSTANCE(GNSS_UBX_TIMER_INSTANCE);
static const nrf_drv_twi_t m_gnss_twi = NRF_DRV_TWI_INSTANCE(GNSS_TWI_INSTANCE);

static uint8_t      gnss_uart_data[MAX_RECEIVED_SENTENCE_SIZE];
static uint8_t      gnss_twi_data[UBXGNSS_UBX_MAX_LENGHTH];
static uint8_t      gnss_raw_data[MAX_RECEIVED_SENTENCE_SIZE];
static uint8_t      gnss_data_valid;

static uint16_t     gnss_char_count;
static uint16_t     gnss_twi_data_size;
static uint16_t     gnss_uart_time    = 0xFFFF;
static uint16_t     gnss_twi_time;
static uint32_t     gnss_idle_time;


/* Test data */
//static uint8_t      test_data[MAX_RECEIVED_SENTENCE_SIZE] = ""
//"$GNRMC,033242.00,A,1051.94964,N,10636.62319,E,0.064,,010119,,,D*6B\r\n"
//"$GNGGA,033242.00,1051.94964,N,10636.62319,E,2,09,1.02,10.1,M,-2.9,M,,0000*67\r\n";
UBXGNSS_State_t gnss_ret;

static void gnss_uart_evt_handler(app_uart_evt_t * p_event);
static void timer_gnss_event_handler(nrf_timer_event_t event_type, void* p_context);
static void timer_ubx_event_handler(nrf_timer_event_t event_type, void* p_context);
static UBXGNSS_State_t gnss_comm_handler(UBXGNSS_Comm_Event_t event, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t data_size, void *p_context);


UBXGNSS_Def_t GNSS_DEV =
{
    .interface = UBXGNSS_UART_INTERFACE | UBXGNSS_I2C_INTERFACE,
    .address = 0x21,
    .commHandle = gnss_comm_handler
};


void gnss_init()
{
    ret_code_t err_code;

    const app_uart_comm_params_t comm_params =
    {
        GNSS_UART_RX_PIN,
        GNSS_UART_TX_PIN,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
#if defined (UART_PRESENT)
        NRF_UART_BAUDRATE_115200
#else
        NRF_UARTE_BAUDRATE_115200
#endif
    };

    nrf_drv_twi_config_t gnss_twi_config = NRF_DRV_TWI_DEFAULT_CONFIG;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;

    gnss_twi_config.sda = GNSS_I2C_SDA_PIN;
    gnss_twi_config.scl = GNSS_I2C_SCL_PIN;

    APP_UART_FIFO_INIT(&comm_params,
                         GNSS_UART_RX_BUFF_SIZE,
                         GNSS_UART_TX_BUFF_SIZE,
                         gnss_uart_evt_handler,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);
    APP_ERROR_CHECK(err_code);

    APP_ERROR_CHECK(nrf_drv_timer_init(&GNSS_TIMER, &timer_cfg, timer_gnss_event_handler));
    APP_ERROR_CHECK(nrf_drv_timer_init(&UBX_TIMER, &timer_cfg, timer_ubx_event_handler));
    APP_ERROR_CHECK(nrf_drv_twi_init(&m_gnss_twi, &gnss_twi_config, NULL, NULL));
//    nrf_gpio_cfg_output(INTERFACE_SELECT_PIN);
//    nrf_gpio_pin_set(INTERFACE_SELECT_PIN);

    nrf_drv_timer_extended_compare(&GNSS_TIMER,
                                   NRF_TIMER_CC_CHANNEL0,
                                   nrf_drv_timer_ms_to_ticks(&GNSS_TIMER, UART_TIMER_STEP),
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
    nrf_drv_timer_extended_compare(&UBX_TIMER,
                                   NRF_TIMER_CC_CHANNEL1,
                                   nrf_drv_timer_ms_to_ticks(&UBX_TIMER, TWI_TIMER_STEP),
                                   NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK,
                                   true);
    nrf_drv_twi_enable(&m_gnss_twi);
    nrfx_timer_enable(&GNSS_TIMER);
    nrfx_timer_enable(&UBX_TIMER);

    GNSS_DEV.interface = (UBXGNSS_UART_INTERFACE | UBXGNSS_I2C_INTERFACE);
    GNSS_DEV.timeout   = 1000;
    UBXGNSS_Init(&GNSS_DEV);
}


/**
 * @brief Handler for serial events.
 */
void gnss_uart_evt_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
	case APP_UART_DATA_READY:
	{
	    UNUSED_VARIABLE(app_uart_get(&gnss_uart_data[gnss_char_count]));
	    gnss_raw_data[gnss_char_count] = gnss_uart_data[gnss_char_count];
            gnss_char_count++;

            gnss_uart_time = 0;
            gnss_idle_time = 0;
	    break;
	}

	case APP_UART_COMMUNICATION_ERROR:
	{
//            APP_ERROR_HANDLER(p_event->data.error_communication);
	    break;
	}

	case APP_UART_FIFO_ERROR:
	{
            APP_ERROR_HANDLER(p_event->data.error_code);
	    break;
	}

	default: break;
    }
}


void gnss_i2c_evt_handler(nrf_drv_twi_evt_t const * p_event,
                                           void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
        {
            break;
        }

        default: break;
    }
}


/**
 * @brief Handler for timer events.
 */
void timer_gnss_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
        {
            if (0xFFFF != gnss_uart_time)
            {
                gnss_uart_time++;
            }

            gnss_idle_time++;

            /* Check data availability in FIFO.
               Process if there is data and UART_TIMEOUT_PERIOD timeout */
            if (UART_TIMEOUT_PERIOD == gnss_idle_time)
            {
                /* Process data */
                UBXGNSS_ProcessData(&GNSS_DEV,
                                    gnss_raw_data,
                                    strlen(gnss_raw_data));

                gnss_idle_time = 0;
                gnss_char_count = 0;

                CLEAR_BUFFER(gnss_uart_data);
                CLEAR_BUFFER(gnss_raw_data);

                gnss_data_valid = UBXGNSS_IS_DATA_VALID(GNSS_DEV);
            }

            break;
        }

        default:
            //Do nothing.
            break;
    }
}


/**
 * @brief Handler for timer events.
 */
void timer_ubx_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE1:
        {
            if (0xFFFF != gnss_twi_time)
            {
                gnss_twi_time++;
            }

            if (TWI_PROCESS_DATA_PERIOD == gnss_twi_time)
            {
                UBXGNSS_GetUBXMessage(&GNSS_DEV, gnss_twi_data, &gnss_twi_data_size);
                UBXGNSS_ProcessUBXMsg(&GNSS_DEV, gnss_twi_data, gnss_twi_data_size);

                gnss_twi_time = 0;
            }

            break;
        }

        default:
            //Do nothing.
            break;
    }
}


static UBXGNSS_State_t gnss_comm_handler(UBXGNSS_Comm_Event_t event, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t data_size, void *p_context)
{
    UBXGNSS_State_t ret;

    switch (event)
    {
        case UART_UBX_EVENT_TRANSMIT:
        {
            break;
        }

        case I2C_UBX_EVENT_TRANSMIT:
        {
            return nrf_drv_twi_tx(&m_gnss_twi, dev_addr, data, data_size, true);
        }

        case I2C_UBX_EVENT_RECEIVE:
        {
            return nrf_drv_twi_rx(&m_gnss_twi, dev_addr, data, data_size);
        }

        default: break;
    }
}
