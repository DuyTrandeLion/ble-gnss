#include "oled.h"

#include "gnss.h"
#include "ecompass.h"
#include "barometer.h"

static uint8_t  m_accel_calib_status;
static uint8_t  m_gyro_calib_status;
static uint8_t  m_mag_calib_status;
static uint8_t  m_system_calib_status;
static uint8_t  m_new_alert_count;
static uint8_t  m_unread_alert_count;

static const uint8_t  *m_new_alert_category;
static uint8_t  *m_new_alert_content;
static const uint8_t  *m_unread_alert_category;


typedef struct
{
    uint8_t line;
    uint8_t y_coord;
} Row_t;


static const Row_t OLEDRow[] =
{
  {1, 0},
  {2, 16},
  {3, 32},
  {4, 48}
};


#if defined(SSD1309_USE_SPI)
void spi_oled_comm_handler(uint8_t hdl_type, uint8_t *hdl_buffer, size_t hdl_buffer_size)
{
    switch (hdl_type)
    {
        case OLED_RESET:
        {
            peripherals_oled_reset();
            break;
        }

        case OLED_WRITE_COMMAND:
        {
            peripherals_oled_write_command(hdl_buffer, hdl_buffer_size);
            break;
        }

        case OLED_WRITE_DATA:
        {
            peripherals_oled_write_data(hdl_buffer, hdl_buffer_size);
            break;
        }

        case OLED_DELAY:
        {
            peripherals_delay_ms(*hdl_buffer);
            break;
        }

        default: break;
    }
}
#endif


void oled_init()
{
    ssd1309_Init(spi_oled_comm_handler);
    ssd1309_Fill(Black);

    oled_clear_screen();
    oled_write_string("CatTrack", 32, OLEDRow[0].y_coord);
    oled_write_string("Snow Lion", 32, OLEDRow[1].y_coord);
    oled_update_screen();
}


void oled_set_new_noti(const uint8_t *p_category, uint8_t *p_content, uint8_t alert_count)
{
    m_new_alert_category = p_category;
    m_new_alert_content = p_content;
    m_new_alert_count = alert_count;
}


void oled_set_unread_noti(const uint8_t *p_category, uint8_t alert_count)
{
    m_unread_alert_category = p_category;
    m_unread_alert_count = alert_count;
}


void oled_set_calib_status(uint8_t accel, uint8_t gyro, uint8_t mag, uint8_t system)
{
    m_accel_calib_status = accel;
    m_gyro_calib_status = gyro;
    m_mag_calib_status = mag;
    m_system_calib_status = system;
}


void oled_navigation_screen(ble_lns_t *p_lns, uint8_t display_page)
{
    char print_buffer[32];
    char *name_token;

    oled_clear_screen();

    switch (display_page)
    {
        case NAVIGATION_0:
        {
            sprintf(print_buffer, "Lat.: %dd", p_lns->p_location_speed->latitude);
            oled_write_string(print_buffer, 0, OLEDRow[0].y_coord);

            sprintf(print_buffer, "Long.: %dd", p_lns->p_location_speed->longitude);
            oled_write_string(print_buffer, 0, OLEDRow[1].y_coord);

            sprintf(print_buffer, "Alt.: %.2fm", (float)(p_lns->p_location_speed->elevation) / 100.0);
            oled_write_string(print_buffer, 0, OLEDRow[2].y_coord);
            break;
        }

        case NAVIGATION_1:
        {
            sprintf(print_buffer, "Distance: %dm", p_lns->p_location_speed->total_distance);
            oled_write_string(print_buffer, 0, OLEDRow[0].y_coord);
            
            sprintf(print_buffer, "Hdng: %.2fd", (float)(p_lns->p_location_speed->heading) / 100.0);
            oled_write_string(print_buffer, 0, OLEDRow[1].y_coord);

            sprintf(print_buffer, "Rollin time: %ds", p_lns->p_location_speed->rolling_time);
            oled_write_string(print_buffer, 0, OLEDRow[2].y_coord);
            break;
        }

        case NAVIGATION_2:
        {
            switch (p_lns->p_position_quality->position_status)
            {
                case BLE_LNS_NO_POSITION:
                {
                    sprintf(print_buffer, "NO POS. s%d v%d", p_lns->p_position_quality->number_of_satellites_in_solution, p_lns->p_position_quality->number_of_satellites_in_view);
                    oled_write_string(print_buffer, 0, OLEDRow[0].y_coord);
                    break;
                }

                case BLE_LNS_POSITION_OK:
                {
                    sprintf(print_buffer, "POS. OK s%d v%d", p_lns->p_position_quality->number_of_satellites_in_solution, p_lns->p_position_quality->number_of_satellites_in_view);
                    oled_write_string(print_buffer, 0, OLEDRow[0].y_coord);
                    break;
                }

                case BLE_LNS_ESTIMATED:
                {
                    sprintf(print_buffer, "POS. EST s%d v%d", p_lns->p_position_quality->number_of_satellites_in_solution, p_lns->p_position_quality->number_of_satellites_in_view);
                    oled_write_string(print_buffer, 0, OLEDRow[0].y_coord);
                    break;
                }

                default: 
                {
                    sprintf(print_buffer, "NO POS. s%d v%d", p_lns->p_position_quality->number_of_satellites_in_solution, p_lns->p_position_quality->number_of_satellites_in_view);
                    oled_write_string(print_buffer, 0, OLEDRow[0].y_coord);
                    break;
                }
            }

            sprintf(print_buffer, "hdop: %d vdop: %d", p_lns->p_position_quality->hdop, p_lns->p_position_quality->vdop);
            oled_write_string(print_buffer, 0, OLEDRow[1].y_coord);

            sprintf(print_buffer, "T2FFix: %.2fs", (float)(p_lns->p_position_quality->time_to_first_fix) / 10.0);
            oled_write_string(print_buffer, 0, OLEDRow[2].y_coord);

            break;
        }

        case NAVIGATION_3:
        {
            if (BLE_LNS_SPEED_DISTANCE_FORMAT_2D == p_lns->p_location_speed->data_format)
            {
                oled_write_string("Spd Distnc. 2D", 0, OLEDRow[0].y_coord);
            }
            else if (BLE_LNS_SPEED_DISTANCE_FORMAT_3D == p_lns->p_location_speed->data_format)
            {
                oled_write_string("Spd Distnc. 3D", 0, OLEDRow[0].y_coord);
            }

            switch (p_lns->p_location_speed->elevation_source)
            {
                case BLE_LNS_ELEV_SOURCE_POSITIONING_SYSTEM:
                {
                    oled_write_string("Elev src: Pos.", 0, OLEDRow[1].y_coord);
                    break;
                }

                case BLE_LNS_ELEV_SOURCE_BAROMETRIC:
                {
                    oled_write_string("Elev src: Baro.", 0, OLEDRow[1].y_coord);
                    break;
                }

                case BLE_LNS_ELEV_SOURCE_DATABASE_SERVICE:
                {
                    oled_write_string("Elev source: DTB.", 0, OLEDRow[1].y_coord);
                    break;
                }

                case BLE_LNS_ELEV_SOURCE_OTHER:
                {
                    oled_write_string("Elev source: Other", 0, OLEDRow[1].y_coord);
                    break;
                }

                default: break;
            }

            oled_write_string("Headin src: Cmpss", 0, OLEDRow[2].y_coord);

            break;
        }

        case NOTI_0:
        {
            sprintf(print_buffer, "New %s", m_new_alert_category);
            oled_write_string(print_buffer, 0, OLEDRow[0].y_coord);
            
            name_token = strtok(m_new_alert_content, " ");

            sprintf(print_buffer, "From %s", name_token);
            oled_write_string(print_buffer, 0, OLEDRow[1].y_coord);
            break;
        }

        case NOTI_1:
        {
            sprintf(print_buffer, "New Alert: %d", m_new_alert_count);
            oled_write_string(print_buffer, 0, OLEDRow[0].y_coord);

            sprintf(print_buffer, "Unread %d", m_unread_alert_count);
            oled_write_string(print_buffer, 0, OLEDRow[1].y_coord);     
            break;
        }

        case CALIB_STATUS:
        {
            
            sprintf(print_buffer, "A: %d, G: %d, M: %d",
                                                        m_accel_calib_status,
                                                        m_gyro_calib_status,
                                                        m_mag_calib_status);
            oled_write_string(print_buffer, 0, OLEDRow[0].y_coord);

            sprintf(print_buffer, "System %d", m_system_calib_status);
            oled_write_string(print_buffer, 0, OLEDRow[1].y_coord);
            break;
        }

        default: break;
    }

    sprintf(print_buffer, "%d/%d/%d %d:%d:%d", p_lns->p_location_speed->utc_time.month,
                                         p_lns->p_location_speed->utc_time.day,
                                         p_lns->p_location_speed->utc_time.year,
                                         p_lns->p_location_speed->utc_time.hours,
                                         p_lns->p_location_speed->utc_time.minutes,
                                         p_lns->p_location_speed->utc_time.seconds);
    oled_write_string(print_buffer, 0, OLEDRow[3].y_coord);

    oled_update_screen();
}