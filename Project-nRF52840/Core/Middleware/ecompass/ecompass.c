#include "ecompass.h"

#include <string.h>

#include "peripherals.h"

#include "bno055.h"
#include "BsxFusionLibrary.h"
#include "BsxLibraryDataTypes.h"
#include "BsxLibraryConstants.h"
#include "BsxLibraryCalibConstants.h"
#include "BsxLibraryErrorConstants.h"

struct bno055_t m_bno055;


static int8_t bno055_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t r_len)
{
    uint8_t rx_data[255];
    uint8_t register_address = reg_addr;

    if (BNO055_SUCCESS != peripherals_twi_tx(dev_addr, &register_address, 1, true))
    {
        return BNO055_ERROR;
    }

    if (BNO055_SUCCESS != peripherals_twi_rx(dev_addr, rx_data, r_len))
    {
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}

static int8_t bno055_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t wr_len)
{
    uint8_t tx_data[255];

    tx_data[0] = reg_addr;
    memcpy(&tx_data[1], reg_data, wr_len);

    if (BNO055_SUCCESS != peripherals_twi_tx(dev_addr, tx_data, wr_len + 1, false))
    {
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}


static void bno055_delay_ms(uint32_t delay_time)
{

}


void ecompass_init(void)
{
    m_bno055.bus_write = bno055_i2c_write;
    m_bno055.bus_read = bno055_i2c_read;
    m_bno055.delay_msec = bno055_delay_ms;
}