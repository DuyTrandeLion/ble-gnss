#ifndef __OLED_H__
#define __OLED_H__

#include <stdint.h>
#include <string.h>

#include "ble_lns.h"

#include "ssd1309.h"
#include "ssd1309_fonts.h"
#include "Miscellaneous.h"

#ifdef __cplusplus
extern "C" {
#endif


#define NAVIGATION_0          0
#define NAVIGATION_1          (NAVIGATION_0 + 1)
#define NAVIGATION_2          (NAVIGATION_1 + 1)
#define NAVIGATION_3          (NAVIGATION_2 + 1)
#define NOTI_0                (NAVIGATION_3 + 1)
#define NOTI_1                (NOTI_0 + 1)
#define CALIB_STATUS          (NOTI_1 + 1)
#define ALL_SCREEN            (CALIB_STATUS + 1)


#define SCREEN_TIMEOUT_PERIOD   7000
#define MSG_CONTENT_LENGTH      8

#define MSG_DISPLAY_LENGTH(str) ((strlen(str) < MSG_CONTENT_LENGTH)? strlen(str) : MSG_CONTENT_LENGTH)

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
void oled_set_new_noti(const uint8_t *p_category, uint8_t *p_content, uint8_t alert_count);
void oled_set_unread_noti(const uint8_t *p_category, uint8_t alert_count);
void oled_set_calib_status(uint8_t accel, uint8_t gyro, uint8_t mag, uint8_t system);
void oled_navigation_screen(ble_lns_t *p_lns, uint8_t display_page);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __OLED_H__ */