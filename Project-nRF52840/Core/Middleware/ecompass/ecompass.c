#include "ecompass.h"

#include <string.h>

#include "peripherals.h"

#include "bno055.h"
#include "BsxFusionLibrary.h"
#include "BsxLibraryDataTypes.h"
#include "BsxLibraryConstants.h"
#include "BsxLibraryCalibConstants.h"
#include "BsxLibraryErrorConstants.h"


static uint16_t m_ecompass_twi_time;
static uint32_t m_ecompass_calib_check_time;
static uint32_t m_calibration_status;

static struct bno055_t m_bno055;
static struct bno055_euler_t m_euler;
static struct bno055_euler_float_t m_euler_data;
static initParam_t m_bsx_init_param;

static void ecompass_comm_polling_handle(void);
static void ecompass_timer_event_handler(void);
static void read_calibration_status();


static int8_t bno055_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t r_len)
{
    uint8_t register_address = reg_addr;

    if (BNO055_SUCCESS != peripherals_twi_tx(dev_addr, &register_address, 1, true))
    {
        return BNO055_ERROR;
    }

    if (BNO055_SUCCESS != peripherals_twi_rx(dev_addr, reg_data, r_len))
    {
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}

static int8_t bno055_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t wr_len)
{
    uint8_t tx_data[255];
    uint8_t register_address = reg_addr;

    memcpy(&tx_data[0], &register_address, 1);
    memcpy(&tx_data[1], reg_data, wr_len);

    if (BNO055_SUCCESS != peripherals_twi_tx(dev_addr, tx_data, wr_len + 1, false))
    {
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}


static void bno055_delay_ms(uint32_t delay_time)
{
    peripherals_delay_ms(delay_time);
}


static void ecompass_comm_polling_handle(void)
{
    if (ECOMPASS_CALIB_CHECK_PERIOD <= m_ecompass_calib_check_time)
    {
        read_calibration_status();
        m_ecompass_calib_check_time = 0;
    }

    if (ECOMPASS_TWI_PROCESS_DATA_PERIOD <= m_ecompass_twi_time)
    {
//        APP_ERROR_CHECK(bno055_read_euler_hrp(&m_euler));
        APP_ERROR_CHECK(bno055_convert_float_euler_hpr_deg(&m_euler_data));
        m_ecompass_twi_time = 0;
    }
}


static void ecompass_timer_event_handler(void)
{
    if (0xFFFF != m_ecompass_twi_time)
    {
        m_ecompass_twi_time++;
    }
}


static void read_calibration_status()
{
    uint8_t accel_calib_status = 0;
    uint8_t gyro_calib_status = 0;
    uint8_t mag_calib_status = 0;
    uint8_t sys_calib_status = 0;

    APP_ERROR_CHECK(bno055_get_accel_calib_stat(&accel_calib_status));
    APP_ERROR_CHECK(bno055_get_gyro_calib_stat(&gyro_calib_status));
    APP_ERROR_CHECK(bno055_get_mag_calib_stat(&mag_calib_status));
    APP_ERROR_CHECK(bno055_get_sys_calib_stat(&sys_calib_status));

    m_calibration_status = (accel_calib_status << 24) | (gyro_calib_status << 16) | (mag_calib_status << 8) | sys_calib_status;
}


void ecompass_init(void)
{
    m_bno055.bus_write = bno055_i2c_write;
    m_bno055.bus_read = bno055_i2c_read;
    m_bno055.delay_msec = bno055_delay_ms;
    m_bno055.dev_addr = BNO055_I2C_ADDR1;

    peripherals_assign_comm_handle(ECOMPASS_COMM, ecompass_comm_polling_handle);
    peripherals_assign_comm_handle(TIMER_ECOMPASS, ecompass_timer_event_handler);

    APP_ERROR_CHECK(bno055_init(&m_bno055));
    APP_ERROR_CHECK(bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF));

    read_calibration_status();

    m_bsx_init_param.accelspec = NULL;
    m_bsx_init_param.gyrospec = NULL;
    m_bsx_init_param.magspec = NULL;
    m_bsx_init_param.usecase = NULL;

    APP_ERROR_CHECK(bsx_init(&m_bsx_init_param));
}


void ecompass_read_calibration_status(uint32_t *calibration_status)
{
    *calibration_status = m_calibration_status;
}


void ecompass_read_heading(float *heading)
{
    *heading = m_euler_data.h;
}