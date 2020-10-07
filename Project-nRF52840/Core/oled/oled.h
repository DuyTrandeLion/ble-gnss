#ifndef __OLED_H__
#define __OLED_H__

#include <stdint.h>

#include "ble_lns.h"
#include "nrf_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif


#define OLED_SPI_INSTANCE	    2

#define OLED_SCK_PIN	      ARDUINO_11_PIN
#define OLED_MOSI_PIN	      ARDUINO_12_PIN
#define OLED_SS_PIN	      ARDUINO_A2_PIN
#define OLED_DC_PIN           ARDUINO_9_PIN
#define OLED_RES_PIN          ARDUINO_13_PIN

#ifndef GPIO_PIN_SET
#define GPIO_PIN_SET          1
#endif

#ifndef GPIO_PIN_RESET
#define GPIO_PIN_RESET        0
#endif


#define NAVIGATION_0          0
#define NAVIGATION_1          1
#define NAVIGATION_2          2
#define NAVIGATION_3          3
#define FINAL_NAV             (NAVIGATION_3 + 1)


typedef enum
{
    OLED_SPI_READY = 0,
    OLED_SPI_BUSY,
    OLED_SPI_NOT_AVAILABLE
} oled_spi_state_t;


#define oled_write_string(string, x, y) if (false == ssd1309_Rotated())			      \
					{						      \
					    ssd1309_SetCursor(x, y);			      \
					}						      \
					else						      \
					{						      \
                                            ssd1309_SetCursor(x + (7 * strlen(string)), y);   \
					}						      \
                                        ssd1309_WriteString(string, Font_7x10, White)

#define oled_write_mstring(string, x, y) if (false == ssd1309_Rotated())                      \
					{						      \
					    ssd1309_SetCursor(x, y);			      \
					}						      \
					else						      \
					{						      \
                                            ssd1309_SetCursor(x + (11 * strlen(string)), y);  \
					}						      \
                                        ssd1309_WriteString(string, Font_11x18, White)

#define oled_write_symbol(symbol, x, y) ssd1309_WriteSymbol(symbol, x, y)

#define oled_update_screen()		ssd1309_UpdateScreen()
#define oled_clear_screen()		ssd1309_Fill(Black)


void oled_init();
void oled_navigation_screen(ble_lns_t *p_lns, uint8_t display_page);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __OLED_H__ */