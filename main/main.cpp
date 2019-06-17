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
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#include "FreeRTOS.h"
#include "TFT_22_ILI9225.h"
#include "ugui.h"
#include "FTPSync.h"

extern "C"
{
#include "wifi_manager_task.h"
}

#define TAG "main"

#define TFT_RST 19
#define TFT_RS  17
#define TFT_CS  22  // SS
#define TFT_SDI 23  // MOSI
#define TFT_CLK 18  // SCK
#define TFT_LED 0   // 0 if wired to +5V directly

#define PIN_NUM_MISO GPIO_NUM_2
#define PIN_NUM_MOSI GPIO_NUM_15
#define PIN_NUM_CLK  GPIO_NUM_14
#define PIN_NUM_CS   GPIO_NUM_13

TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_SDI, TFT_CLK);
FreeRTOS::EventFlags flags;


void ftp_log_message(const std::string& output)
{
  std::cout << output << std::endl;
}

void wifi_connected(void * param)
{
  flags.set(1);
  ESP_LOGI(TAG, "Wifi Connected!");
}

void wifi_disconnected(void * param)
{
  ESP_LOGI(TAG, "Wifi Connected!");
}

void init_fs()
{
  ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = PIN_NUM_MISO;
    slot_config.gpio_mosi = PIN_NUM_MOSI;
    slot_config.gpio_sck  = PIN_NUM_CLK;
    slot_config.gpio_cs   = PIN_NUM_CS;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 1024 * 16
    };

    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    ESP_LOGI(TAG, "Initialize FTP client");
}


FTPSync ftp_sync;
void sync_folder()
{
    ftp_sync.connect("145.14.144.209",
                     21,
                     "sharedfolder",
                     "0502117580",
                     "/shared/",
                     "/sdcard/");
    ftp_sync.sync();
}

void sync_handle()
{
    /* Connect to wifi */
    esp_log_level_set("dns_server", ESP_LOG_DEBUG);
    wifi_manager_start_task(wifi_connected, NULL, wifi_disconnected, NULL);
    flags.wait(1);
    /* Sync the folder */
    sync_folder();
}

void show_random_image()
{
    
}

void program()
{
    nvs_flash_init();
    init_fs();

    sync_handle();
}

extern "C" void app_main()
{
    program();
}
	