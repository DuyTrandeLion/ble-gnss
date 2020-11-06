#include "peripherals.h"

#include "gnss.h"
#include "barometer.h"


static uint8_t      gnss_uart_data[MAX_RECEIVED_SENTENCE_SIZE];
static uint8_t      gnss_twi_data[UBXGNSS_UBX_MAX_LENGHTH];
static uint8_t      gnss_raw_data[MAX_RECEIVED_SENTENCE_SIZE];
static uint8_t      gnss_data_valid;

static uint16_t     gnss_char_count;
static uint16_t     gnss_twi_data_size;
static uint16_t     gnss_uart_time = 0xFFFF;
static uint16_t     gnss_twi_time;
static uint32_t     gnss_idle_time;

UBXGNSS_State_t gnss_ret;

static void gnss_comm_polling_handle(void);
static void gnss_app_uart_data_ready_handle(void);
static void gnss_timer_event_handle(void);
static void ubx_timer_event_handler(void);

static UBXGNSS_State_t gnss_comm_handler(UBXGNSS_Comm_Event_t event, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t data_size, void *p_context);


UBXGNSS_Def_t GNSS_DEV =
{
    .interface = UBXGNSS_UART_INTERFACE | UBXGNSS_I2C_INTERFACE,
    .address = 0x21,
    .commHandle = gnss_comm_handler
};


void gnss_init(void)
{
    GNSS_DEV.interface = (UBXGNSS_UART_INTERFACE | UBXGNSS_I2C_INTERFACE);
    GNSS_DEV.timeout   = 1000;

    peripherals_assign_comm_handle(GNSS_COMM, gnss_comm_polling_handle);
    peripherals_assign_comm_handle(UART_DATA_READY, gnss_app_uart_data_ready_handle);
    peripherals_assign_comm_handle(TIMER_GNSS, gnss_timer_event_handle);
    peripherals_assign_comm_handle(TIMER_UBX, ubx_timer_event_handler);

    UBXGNSS_Init(&GNSS_DEV);
}


static void gnss_comm_polling_handle(void)
{
    if (GNSS_TWI_PROCESS_DATA_PERIOD <= gnss_twi_time)
    {
        UBXGNSS_GetUBXMessage(&GNSS_DEV, gnss_twi_data, &gnss_twi_data_size);
        UBXGNSS_ProcessUBXMsg(&GNSS_DEV, gnss_twi_data, gnss_twi_data_size);

        gnss_twi_time = 0;
    }
}


static void gnss_app_uart_data_ready_handle(void)
{
    UNUSED_VARIABLE(peripherals_app_uart_get(&gnss_uart_data[gnss_char_count]));
    gnss_raw_data[gnss_char_count] = gnss_uart_data[gnss_char_count];
    gnss_char_count++;

    gnss_uart_time = 0;
    gnss_idle_time = 0;
}


static void gnss_timer_event_handle(void)
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
}


static void ubx_timer_event_handler(void)
{
    if (0xFFFF != gnss_twi_time)
    {
        gnss_twi_time++;
    }
}


static UBXGNSS_State_t gnss_comm_handler(UBXGNSS_Comm_Event_t event, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t data_size, void *p_context)
{
    uint8_t register_address = reg_addr;
    uint8_t tx_data[UBXGNSS_UBX_MAX_LENGHTH + 1];

    switch (event)
    {
        case UART_UBX_EVENT_TRANSMIT:
        {
            break;
        }

        case I2C_UBX_EVENT_TRANSMIT:
        {
            memcpy(&tx_data[0], &register_address, 1);
            memcpy(&tx_data[1], data, data_size);
            if (UBXGNSS_OK != peripherals_twi_tx(dev_addr, tx_data, data_size + 1, false))
            {
                return UBXGNSS_I2C_COMM_FAILURE;
            }

            return UBXGNSS_OK;
            break;
        }

        case I2C_UBX_EVENT_RECEIVE:
        {
            if (UBXGNSS_OK != peripherals_twi_tx(dev_addr, &register_address, 1, true))
            {
                return UBXGNSS_I2C_COMM_FAILURE;
            }

            if (UBXGNSS_OK != peripherals_twi_rx(dev_addr, data, data_size))
            {
                return UBXGNSS_I2C_COMM_FAILURE;
            }

            return UBXGNSS_OK;
            break;
        }

        default: break;
    }
}



