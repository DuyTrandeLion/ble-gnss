#ifndef COM_BUS_H
#define COM_BUS_H

#include <stdlib.h>
#include <stdint.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "boards.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IKS01A2_TWI_INSTANCE  1
#define MEMS_ADDRESS_SIZE     2
#define MEMS_MAX_DATA_SIZE    256

#define IKS01A2_I2C_SDA_PIN   ARDUINO_SDA_PIN
#define IKS01A2_I2C_SCL_PIN   ARDUINO_SCL_PIN

int32_t i2c_bus_init(void);
int32_t i2c_bus_deinit(void);
int32_t i2c_bus_read_reg(uint16_t dev_addr, uint16_t reg, uint8_t *p_data, uint16_t data_length); 
int32_t i2c_bus_write_reg(uint16_t dev_addr, uint16_t reg, uint8_t *p_data, uint16_t data_length);


#ifdef __cplusplus
}
#endif

#endif /* COM_BUS_H */