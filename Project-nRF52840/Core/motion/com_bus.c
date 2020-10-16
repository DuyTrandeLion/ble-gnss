#include "errno.h"
#include "com_bus.h"
#include "lsm303agr.h"

#include "nrf_drv_twi.h"

static const nrf_drv_twi_t m_iks01a2_twi = NRF_DRV_TWI_INSTANCE(IKS01A2_TWI_INSTANCE);
static int8_t m_twi_initialized = 0;

int32_t i2c_bus_init(void)
{
    ret_code_t err_code;

    if (0 == m_twi_initialized)
    {
        nrf_drv_twi_config_t iks01a2_twi_config = NRF_DRV_TWI_DEFAULT_CONFIG;
        iks01a2_twi_config.scl = IKS01A2_I2C_SCL_PIN;
        iks01a2_twi_config.sda = IKS01A2_I2C_SDA_PIN;

        err_code = nrf_drv_twi_init(&m_iks01a2_twi, &iks01a2_twi_config, NULL, NULL);
    
        if (NRF_SUCCESS != err_code)
        {
            APP_ERROR_CHECK(err_code);
        }

        m_twi_initialized = 1;

        nrf_drv_twi_enable(&m_iks01a2_twi);
    }

    return BSP_ERROR_NONE;
}

void I2C_Init(void)
{
 
}


int32_t i2c_bus_deinit(void)
{
    return BSP_ERROR_NONE;
}


int32_t i2c_bus_read_reg(uint16_t dev_addr, uint16_t reg, uint8_t *p_data, uint16_t data_length)
{
    ret_code_t err_code;
    uint8_t mems_address_buffer[MEMS_ADDRESS_SIZE];

    mems_address_buffer[0] = reg;

    /* The device address is shifted right 1 bit because of the read/write bit is handle by Nordic TWI library. */
    err_code = nrf_drv_twi_tx(&m_iks01a2_twi, dev_addr >> 1, mems_address_buffer, MEMS_ADDRESS_SIZE, true);

    if (NRF_SUCCESS != err_code)
    {
        APP_ERROR_CHECK(err_code);
        return BSP_ERROR_PERIPH_FAILURE;
    }

    err_code = nrf_drv_twi_rx(&m_iks01a2_twi, dev_addr >> 1, p_data, data_length);

    if (NRF_SUCCESS != err_code)
    {
        APP_ERROR_CHECK(err_code);
        return BSP_ERROR_PERIPH_FAILURE;
    }

    return BSP_ERROR_NONE;
}


int32_t i2c_bus_write_reg(uint16_t dev_addr, uint16_t reg, uint8_t *p_data, uint16_t data_length)
{
    ret_code_t err_code;
    uint8_t mems_data[MEMS_MAX_DATA_SIZE] = {0};

    mems_data[0] = reg;
    memcpy(&mems_data[1], p_data, data_length);
    
    /* The device address is shifted right 1 bit because of the read/write bit is handle by Nordic TWI library. */
    err_code = nrf_drv_twi_tx(&m_iks01a2_twi, dev_addr >> 1, mems_data, data_length + MEMS_ADDRESS_SIZE, false);

    if (NRF_SUCCESS != err_code)
    {
        APP_ERROR_CHECK(err_code);
        return BSP_ERROR_PERIPH_FAILURE;
    }

    return BSP_ERROR_NONE;
}