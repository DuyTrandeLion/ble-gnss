#include "errno.h"
#include "com_bus.h"
#include "lsm303agr.h"

#include "nrf_drv_twi.h"

static const nrf_drv_twi_t m_iks01a2_twi = NRF_DRV_TWI_INSTANCE(IKS01A2_TWI_INSTANCE);

int32_t i2c_bus_init(void)
{
    ret_code_t err_code;

    nrf_drv_twi_config_t iks01a2_twi_config = NRF_DRV_TWI_DEFAULT_CONFIG;
    iks01a2_twi_config.scl = IKS01A2_I2C_SCL_PIN;
    iks01a2_twi_config.sda = IKS01A2_I2C_SDA_PIN;

    err_code = nrf_drv_twi_init(&m_iks01a2_twi, &iks01a2_twi_config, NULL, NULL);
    
    if (NRF_SUCCESS != err_code)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    return BSP_ERROR_NONE;
}


int32_t i2c_bus_deinit(void)
{
    return BSP_ERROR_NONE;
}


int32_t i2c_bus_read_reg(uint16_t dev_addr, uint16_t reg, uint8_t *p_data, uint16_t data_length)
{
    ret_code_t err_code;
    uint8_t mems_address_buffer[MEMS_ADDRESS_SIZE];

    mems_address_buffer[0] = (reg >> 8 ) & 0xFF;
    mems_address_buffer[1] = reg;

    err_code = nrf_drv_twi_tx(&m_iks01a2_twi, dev_addr, mems_address_buffer, MEMS_ADDRESS_SIZE, true);

    if (NRF_SUCCESS != err_code)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    err_code = nrf_drv_twi_rx(&m_iks01a2_twi, dev_addr, p_data, data_length);

    if (NRF_SUCCESS != err_code)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    return BSP_ERROR_NONE;
}


int32_t i2c_bus_write_reg(uint16_t dev_addr, uint16_t reg, uint8_t *p_data, uint16_t data_length)
{
    ret_code_t err_code;
    uint8_t mems_data[MEMS_MAX_DATA_SIZE] = {0};

    mems_data[0] = (reg >> 8 ) && 0xFF;
    mems_data[1] = reg;
    memcpy(&mems_data[2], p_data, data_length);

    err_code = nrf_drv_twi_tx(&m_iks01a2_twi, dev_addr, mems_data, data_length + MEMS_ADDRESS_SIZE, false);

    if (NRF_SUCCESS != err_code)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    return BSP_ERROR_NONE;
}