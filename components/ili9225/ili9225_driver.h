#pragma once
#include <stddef.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

#define ILI9225_LCD_WIDTH  176
#define ILI9225_LCD_HEIGHT 220

void ili9225_init(gpio_num_t mosi,
					gpio_num_t clk,
					gpio_num_t cs,
					gpio_num_t rst,
					gpio_num_t rs,
					spi_host_device_t host,
					int dma_channel,
					int clk_speed);

void ili9225_set_pixel(uint8_t red, 
						uint8_t blue,
						uint8_t green,
						size_t x,
						size_t y);

void ili9225_fill(uint8_t red,
				  uint8_t blue,
				  uint8_t green);

void ili9225_update();