
#include <Arduino.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>


extern "C" void app_main() {
    initArduino();
    for (;;)
    {
        printf("Hello, World!\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
} 