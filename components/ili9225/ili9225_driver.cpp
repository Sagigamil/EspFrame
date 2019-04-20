#include "ili9225_driver.h"

static spi_device_handle_t s_spi;

static uint16_t s_screen_buffer[ILI9225_LCD_WIDTH * ILI9225_LCD_HEIGHT] = { 0 };

static void ili9225_spi_init(gpio_num_t mosi,
	gpio_num_t clk,
	spi_host_device_t host,
	int dma_channel,
	int clk_speed)
{
    esp_err_t ret = ESP_FAIL;
    spi_bus_config_t buscfg;
    spi_device_interface_config_t devcfg;

    buscfg.mosi_io_num = mosi;
    buscfg.miso_io_num =  -1;
    buscfg.sclk_io_num = clk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = sizeof(s_screen_buffer);
    buscfg.flags = SPICOMMON_BUSFLAG_MASTER;
    buscfg.intr_flags = 0;

    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.mode = 0;
    devcfg.duty_cycle_pos = 0;
    devcfg.cs_ena_pretrans = 0;
    devcfg.cs_ena_posttrans = 0;
    devcfg.clock_speed_hz = clk_speed;
    devcfg.input_delay_ns = 0;
    devcfg.spics_io_num = -1;
    devcfg.flags = 0;
    devcfg.queue_size = 4096;
    devcfg.pre_cb = NULL;
    devcfg.post_cb = NULL;

    ret = spi_bus_initialize(host, &buscfg, dma_channel);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(host, &devcfg, &s_spi);
    ESP_ERROR_CHECK(ret);
}

void ili9225_set_gpio(gpio_num_t pin)
{
	esp_err_t res = ESP_FAIL;
	gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1 << pin;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    res = gpio_config(&io_conf);
    ESP_ERROR_CHECK(res);
}

void ili9225_set_gpio_level(gpio_num_t pin, bool value)
{
	ESP_ERROR_CHECK(gpio_set_level(pin, value ? 1 : 0));
}

void ili9225_init(gpio_num_t mosi,
	gpio_num_t clk,
	gpio_num_t cs,
	gpio_num_t rst,
	gpio_num_t rs,
	spi_host_device_t host,
	int dma_channel,
	int clk_speed)
{
    ili9225_spi_init(mosi, clk, host, dma_channel, clk_speed);

    ili9225_set_gpio(rst);
    ili9225_set_gpio(rs);

    ili9225_set_gpio_level(rst, false);
    ili9225_set_gpio_level(rs, false);
}

void ili9225_set_pixel(uint8_t red, 
						uint8_t blue,
						uint8_t green,
						size_t x,
						size_t y);

void ili9225_fill(uint8_t red,
				  uint8_t blue,
				  uint8_t green);

void ili9225_update();