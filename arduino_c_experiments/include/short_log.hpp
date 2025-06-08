/*
ELEKTRON © 2025 - now
Written by melektron
www.elektron.work
08.06.25, 13:41

Shorter style log so i don't have to add the tag every time
*/

#pragma once

#include <esp_log.h>

#ifndef TAG
#error "TAG must be defined to use short_log.h"
#endif

#define LOGE(format, __ARGS__...) ESP_LOGE(TAG, format, ## __ARGS__)
#define LOGW(format, __ARGS__...) ESP_LOGW(TAG, format, ## __ARGS__)
#define LOGI(format, __ARGS__...) ESP_LOGI(TAG, format, ## __ARGS__)
#define LOGD(format, __ARGS__...) ESP_LOGD(TAG, format, ## __ARGS__)
