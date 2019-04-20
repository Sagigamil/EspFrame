/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

#include "wifi_manager_task.h"
#include "ili9225_driver.h"
#include "ugui.h"

#define TAG "main"

void wifi_connected(void * param)
{
    ESP_LOGD(TAG, "Wifi connected!");
}

void wifi_disconnected(void * param)
{
    ESP_LOGD(TAG, "Wifi disconnected!");
}

void app_main()
{
    ESP_LOGD(TAG, "Test!");
    wifi_manager_start_task(wifi_connected, NULL, wifi_disconnected, NULL);
}
