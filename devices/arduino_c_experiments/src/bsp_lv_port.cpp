
#include <esp_timer.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_st7789.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_master.h>
#include <driver/ledc.h>
#include <driver/gpio.h>

#include <Arduino.h>
#include <lvgl.h>

#include "bsp_lv_port.h"
#include "bsp_cst816.h"


static const char *TAG = "bsp_lv_port";

static SemaphoreHandle_t lvgl_api_mux = NULL;
static SemaphoreHandle_t flush_wait_sem = nullptr;

lv_display_t *display1;
esp_lcd_panel_handle_t panel_handle;


static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    BaseType_t hptw;
    xSemaphoreGiveFromISR(flush_wait_sem, &hptw);
    return false;
}

static void example_lvgl_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *pixel_map)
{
    //printf("flush\r\n");
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, pixel_map);
}

static void example_flush_wait_cb(lv_display_t *display)
{
    // wait until dma copy is done
    xSemaphoreTake(flush_wait_sem, portMAX_DELAY);
}

static void example_lvgl_touch_cb(lv_indev_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x;
    uint16_t touchpad_y;
    bsp_touch_read();
    if (bsp_touch_get_coordinates(&touchpad_x, &touchpad_y))
    {
        data->point.x = touchpad_x;
        data->point.y = touchpad_y;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}


void local_display_init(void)
{

    ESP_LOGI(TAG, "SPI BUS init");
    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = EXAMPLE_PIN_NUM_LCD_SCLK;
    buscfg.mosi_io_num = EXAMPLE_PIN_NUM_LCD_MOSI;
    buscfg.miso_io_num = EXAMPLE_PIN_NUM_LCD_MISO;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4000;
    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "Install panel IO");

    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC;
    io_config.cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS;
    io_config.pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ;
    io_config.lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS;
    io_config.lcd_param_bits = EXAMPLE_LCD_PARAM_BITS;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;
    io_config.on_color_trans_done = example_notify_lvgl_flush_ready;    // this may be called too often but is not a problem mostly https://github.com/espressif/esp-idf/issues/14860
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_SPI_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = 16;
    ESP_LOGI(TAG, "Install ST7789 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    // configure backlight

    gpio_set_direction((gpio_num_t)EXAMPLE_PIN_NUM_LCD_BL, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)EXAMPLE_PIN_NUM_LCD_BL, 1);

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = EXAMPLE_LCD_BL_LEDC_MODE;
    ledc_timer.timer_num = EXAMPLE_LCD_BL_LEDC_TIMER;
    ledc_timer.duty_resolution = EXAMPLE_LCD_BL_LEDC_DUTY_RES;
    ledc_timer.freq_hz = EXAMPLE_LCD_BL_LEDC_FREQUENCY; // Set output frequency at 5 kHz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode = EXAMPLE_LCD_BL_LEDC_MODE;
    ledc_channel.channel = EXAMPLE_LCD_BL_LEDC_CHANNEL;
    ledc_channel.timer_sel = EXAMPLE_LCD_BL_LEDC_TIMER;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num = EXAMPLE_PIN_NUM_LCD_BL;
    ledc_channel.duty = 0, // Set duty to 0
        ledc_channel.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // for now just set to full brightness
    ESP_ERROR_CHECK(ledc_set_duty(EXAMPLE_LCD_BL_LEDC_MODE, EXAMPLE_LCD_BL_LEDC_CHANNEL, 1023));
    ESP_ERROR_CHECK(ledc_update_duty(EXAMPLE_LCD_BL_LEDC_MODE, EXAMPLE_LCD_BL_LEDC_CHANNEL));
}

void local_touch_init(void)
{
    while (bsp_touch_init(&Wire, EXAMPLE_LCD_ROTATION, EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES) == false)
    {
        delay(1000);
    }
}

void bsp_lv_port_init(void)
{
    local_display_init();
    local_touch_init();

    lvgl_api_mux = xSemaphoreCreateRecursiveMutex();
    flush_wait_sem = xSemaphoreCreateBinary();

    lv_init();
    lv_tick_set_cb(xTaskGetTickCount);

    display1 = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
#define BYTES_PER_PIXEL (LV_COLOR_DEPTH >> 3)   // LV_COLOR_DEPTH / 8
#define BUFFER_SIZE (EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * BYTES_PER_PIXEL / 5)
    uint8_t *buf1 = (uint8_t *)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_INTERNAL);
    assert(buf1);
    uint8_t *buf2 = (uint8_t *)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_INTERNAL);
    assert(buf2);

    lv_display_set_buffers(display1, buf1, buf2, BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display1, example_lvgl_flush_cb);
    lv_display_set_flush_wait_cb(display1, example_flush_wait_cb);

    static lv_indev_t *indev_touchscreen = lv_indev_create();
    lv_indev_set_type(indev_touchscreen, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_touchscreen, example_lvgl_touch_cb);
    //lv_indev_set_display()    // maybe needed to use display other than default display but we only have one so doesn't matter

    printf("past init\r\n");
}

static void task(void *param)
{
    for (;;)
    {
        // lock happens internally in lv_timer_handler() (https://docs.lvgl.io/master/details/integration/adding-lvgl-to-your-project/threading.html#method-2-use-a-mutex)
        uint32_t task_delay_ms = lv_timer_handler();
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) // This will happen when LV_NO_TIMER_READY is returned
            task_delay_ms = LV_DEF_REFR_PERIOD; 
        else if (task_delay_ms < 1)
            continue;

        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }

    vTaskDelete(NULL);
}

void bsp_lv_port_run(void)
{
    xTaskCreatePinnedToCore(task, "bsp_lv_port_task", 1024 * 10, NULL, 5, NULL, 1);
    printf("task created\r\n");
}
