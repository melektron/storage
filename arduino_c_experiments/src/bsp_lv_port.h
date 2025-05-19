#pragma once
#include <stdio.h>
#include <stdbool.h>

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ          (80 * 1000 * 1000)

#define EXAMPLE_SPI_HOST                    SPI2_HOST
#define EXAMPLE_PIN_NUM_LCD_SCLK            39
#define EXAMPLE_PIN_NUM_LCD_MOSI            38
#define EXAMPLE_PIN_NUM_LCD_MISO            40
#define EXAMPLE_PIN_NUM_LCD_DC              42
#define EXAMPLE_PIN_NUM_LCD_RST             -1
#define EXAMPLE_PIN_NUM_LCD_CS              45

#define EXAMPLE_LCD_CMD_BITS                8
#define EXAMPLE_LCD_PARAM_BITS              8

#define EXAMPLE_LCD_ROTATION                1
// The following are AFTER rotation
#define EXAMPLE_LCD_H_RES                   320
#define EXAMPLE_LCD_V_RES                   240

#define EXAMPLE_PIN_NUM_LCD_BL            1
#define EXAMPLE_LCD_BL_LEDC_TIMER LEDC_TIMER_0
#define EXAMPLE_LCD_BL_LEDC_MODE LEDC_LOW_SPEED_MODE
#define EXAMPLE_LCD_BL_LEDC_CHANNEL LEDC_CHANNEL_0
#define EXAMPLE_LCD_BL_LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define EXAMPLE_LCD_BL_LEDC_DUTY (1024)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define EXAMPLE_LCD_BL_LEDC_FREQUENCY (10000)          // Frequency in Hertz. Set frequency at 5 kHz

#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS      500


void bsp_lv_port_init(void);
void bsp_lv_port_run(void);
