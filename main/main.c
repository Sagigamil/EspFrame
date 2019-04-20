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

#define TFT_RST 4
#define TFT_RS  5
#define TFT_CS  22  // SS
#define TFT_SDI 23  // MOSI
#define TFT_CLK 18  // SCK
#define TFT_LED 0   // 0 if wired to +5V directly

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
    //wifi_manager_start_task(wifi_connected, NULL, wifi_disconnected, NULL);
    UG_GetGUI();
    ili9225_init(TFT_SDI, 
    	TFT_CLK,
    	TFT_CS,
    	TFT_RST,
    	TFT_RS,
    	VSPI_HOST,
    	2,
    	SPI_MASTER_FREQ_26M);
}
	