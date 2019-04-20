#include "ili9225_driver.h"

/* ILI9225 LCD Registers */
enum EILI9225Registers 
{
    ILI9225_DRIVER_OUTPUT_CTRL       = (0x01u),  // Driver Output Control
    ILI9225_LCD_AC_DRIVING_CTRL      = (0x02u),  // LCD AC Driving Control
    ILI9225_ENTRY_MODE               = (0x03u),  // Entry Mode
    ILI9225_DISP_CTRL1               = (0x07u),  // Display Control 1
    ILI9225_BLANK_PERIOD_CTRL1       = (0x08u),  // Blank Period Control
    ILI9225_FRAME_CYCLE_CTRL         = (0x0Bu),  // Frame Cycle Control
    ILI9225_INTERFACE_CTRL           = (0x0Cu),  // Interface Control
    ILI9225_OSC_CTRL                 = (0x0Fu),  // Osc Control
    ILI9225_POWER_CTRL1              = (0x10u),  // Power Control 1
    ILI9225_POWER_CTRL2              = (0x11u),  // Power Control 2
    ILI9225_POWER_CTRL3              = (0x12u),  // Power Control 3
    ILI9225_POWER_CTRL4              = (0x13u),  // Power Control 4
    ILI9225_POWER_CTRL5              = (0x14u),  // Power Control 5
    ILI9225_VCI_RECYCLING            = (0x15u),  // VCI Recycling
    ILI9225_RAM_ADDR_SET1            = (0x20u),  // Horizontal GRAM Address Set
    ILI9225_RAM_ADDR_SET2            = (0x21u),  // Vertical GRAM Address Set
    ILI9225_GRAM_DATA_REG            = (0x22u),  // GRAM Data Register
    ILI9225_GATE_SCAN_CTRL           = (0x30u),  // Gate Scan Control Register
    ILI9225_VERTICAL_SCROLL_CTRL1    = (0x31u),  // Vertical Scroll Control 1 Register
    ILI9225_VERTICAL_SCROLL_CTRL2    = (0x32u),  // Vertical Scroll Control 2 Register
    ILI9225_VERTICAL_SCROLL_CTRL3    = (0x33u),  // Vertical Scroll Control 3 Register
    ILI9225_PARTIAL_DRIVING_POS1     = (0x34u),  // Partial Driving Position 1 Register
    ILI9225_PARTIAL_DRIVING_POS2     = (0x35u),  // Partial Driving Position 2 Register
    ILI9225_HORIZONTAL_WINDOW_ADDR1  = (0x36u),  // Horizontal Address Start Position
    ILI9225_HORIZONTAL_WINDOW_ADDR2  = (0x37u),  // Horizontal Address End Position
    ILI9225_VERTICAL_WINDOW_ADDR1    = (0x38u),  // Vertical Address Start Position
    ILI9225_VERTICAL_WINDOW_ADDR2    = (0x39u),  // Vertical Address End Position
    ILI9225_GAMMA_CTRL1              = (0x50u),  // Gamma Control 1
    ILI9225_GAMMA_CTRL2              = (0x51u),  // Gamma Control 2
    ILI9225_GAMMA_CTRL3              = (0x52u),  // Gamma Control 3
    ILI9225_GAMMA_CTRL4              = (0x53u),  // Gamma Control 4
    ILI9225_GAMMA_CTRL5              = (0x54u),  // Gamma Control 5
    ILI9225_GAMMA_CTRL6              = (0x55u),  // Gamma Control 6
    ILI9225_GAMMA_CTRL7              = (0x56u),  // Gamma Control 7
    ILI9225_GAMMA_CTRL8              = (0x57u),  // Gamma Control 8
    ILI9225_GAMMA_CTRL9              = (0x58u),  // Gamma Control 9
    ILI9225_GAMMA_CTRL10             = (0x59u),  // Gamma Control 10
};

static spi_device_handle_t s_spi;

static gpio_num_t s_cs;
static gpio_num_t s_rst;
static gpio_num_t s_rs;


static uint16_t s_screen_buffer[ILI9225_LCD_WIDTH * ILI9225_LCD_HEIGHT] = { 0 };

static void ili9225_sleep_ms(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

static void ili9225_spi_write(void * data, size_t size)
{
    esp_err_t ret;

    spi_transaction_t trans_desc;
    trans_desc.flags     = 0;
    trans_desc.length    = 8 * size;
    trans_desc.rxlength  = 0;
    trans_desc.tx_buffer = data;
    trans_desc.rx_buffer = NULL;

    ret = ::spi_device_transmit(s_spi, &trans_desc);

    ESP_ERROR_CHECK(ret);
}

static void ili9225_write_byte(uint8_t byte)
{

    ili9225_spi_write(&byte, sizeof(byte));
}

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

static void ili9225_set_gpio(gpio_num_t pin)
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

static void ili9225_set_gpio_level(gpio_num_t pin, bool value)
{
	ESP_ERROR_CHECK(gpio_set_level(pin, value ? 1 : 0));
}

static void ili9225_start_write()
{
    ili9225_set_gpio_level(s_cs, false);
}

static void ili9225_stop_write()
{
    ili9225_set_gpio_level(s_cs, true);
}

static void ili9225_write_data(uint16_t data)
{
    ili9225_set_gpio_level(s_rs, true);
    ili9225_write_byte(data >> 8);
    ili9225_write_byte(data & 0xFF);
}

static void ili9225_write_command(EILI9225Registers reg)
{
    uint16_t register_value = (uint16_t)reg;
    ili9225_set_gpio_level(s_rs, false);
    ili9225_write_byte(register_value >> 8);
    ili9225_write_byte(register_value & 0xFF);
}


static void ili9225_write_register(EILI9225Registers reg, uint16_t value)
{
    ili9225_write_command(reg);
    ili9225_write_data(value);
}

static void ili9225_registers_init()
{
    /* Reset the ili9225 */
    ili9225_set_gpio_level(s_rst, true);
    ili9225_sleep_ms(1);
    ili9225_set_gpio_level(s_rst, false);
    ili9225_sleep_ms(10);
    ili9225_set_gpio_level(s_rst, true);
    ili9225_sleep_ms(50);

    ili9225_start_write();
    ili9225_write_register(ILI9225_POWER_CTRL1, 0x0000); // Set SAP,DSTB,STB
    ili9225_write_register(ILI9225_POWER_CTRL2, 0x0000); // Set APON,PON,AON,VCI1EN,VC
    ili9225_write_register(ILI9225_POWER_CTRL3, 0x0000); // Set BT,DC1,DC2,DC3
    ili9225_write_register(ILI9225_POWER_CTRL4, 0x0000); // Set GVDD
    ili9225_write_register(ILI9225_POWER_CTRL5, 0x0000); // Set VCOMH/VCOML voltage
    ili9225_stop_write();

    ili9225_sleep_ms(40);

    // Power-on sequence
    ili9225_start_write();
    ili9225_write_register(ILI9225_POWER_CTRL2, 0x0018); // Set APON,PON,AON,VCI1EN,VC
    ili9225_write_register(ILI9225_POWER_CTRL3, 0x6121); // Set BT,DC1,DC2,DC3
    ili9225_write_register(ILI9225_POWER_CTRL4, 0x006F); // Set GVDD   /*007F 0088 */
    ili9225_write_register(ILI9225_POWER_CTRL5, 0x495F); // Set VCOMH/VCOML voltage
    ili9225_write_register(ILI9225_POWER_CTRL1, 0x0800); // Set SAP,DSTB,STB
    ili9225_stop_write();
    ili9225_sleep_ms(10);

    ili9225_start_write();
    ili9225_write_register(ILI9225_POWER_CTRL2, 0x103B); // Set APON,PON,AON,VCI1EN,VC
    ili9225_stop_write();
    ili9225_sleep_ms(50);

    ili9225_start_write();
    ili9225_write_register(ILI9225_DRIVER_OUTPUT_CTRL, 0x011C); // set the display line number and display direction
    ili9225_write_register(ILI9225_LCD_AC_DRIVING_CTRL, 0x0100); // set 1 line inversion
    ili9225_write_register(ILI9225_ENTRY_MODE, 0x1038); // set GRAM write direction and BGR=1.
    ili9225_write_register(ILI9225_DISP_CTRL1, 0x0000); // Display off
    ili9225_write_register(ILI9225_BLANK_PERIOD_CTRL1, 0x0808); // set the back porch and front porch
    ili9225_write_register(ILI9225_FRAME_CYCLE_CTRL, 0x1100); // set the clocks number per line
    ili9225_write_register(ILI9225_INTERFACE_CTRL, 0x0000); // CPU interface
    ili9225_write_register(ILI9225_OSC_CTRL, 0x0D01); // Set Osc  /*0e01*/
    ili9225_write_register(ILI9225_VCI_RECYCLING, 0x0020); // Set VCI recycling
    ili9225_write_register(ILI9225_RAM_ADDR_SET1, 0x0000); // RAM Address
    ili9225_write_register(ILI9225_RAM_ADDR_SET2, 0x0000); // RAM Address

    /* Set GRAM area */
    ili9225_write_register(ILI9225_GATE_SCAN_CTRL, 0x0000); 
    ili9225_write_register(ILI9225_VERTICAL_SCROLL_CTRL1, 0x00DB); 
    ili9225_write_register(ILI9225_VERTICAL_SCROLL_CTRL2, 0x0000); 
    ili9225_write_register(ILI9225_VERTICAL_SCROLL_CTRL3, 0x0000); 
    ili9225_write_register(ILI9225_PARTIAL_DRIVING_POS1, 0x00DB); 
    ili9225_write_register(ILI9225_PARTIAL_DRIVING_POS2, 0x0000); 
    ili9225_write_register(ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF); 
    ili9225_write_register(ILI9225_HORIZONTAL_WINDOW_ADDR2, 0x0000); 
    ili9225_write_register(ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB); 
    ili9225_write_register(ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000); 

    /* Set GAMMA curve */
    ili9225_write_register(ILI9225_GAMMA_CTRL1, 0x0000); 
    ili9225_write_register(ILI9225_GAMMA_CTRL2, 0x0808); 
    ili9225_write_register(ILI9225_GAMMA_CTRL3, 0x080A); 
    ili9225_write_register(ILI9225_GAMMA_CTRL4, 0x000A); 
    ili9225_write_register(ILI9225_GAMMA_CTRL5, 0x0A08); 
    ili9225_write_register(ILI9225_GAMMA_CTRL6, 0x0808); 
    ili9225_write_register(ILI9225_GAMMA_CTRL7, 0x0000); 
    ili9225_write_register(ILI9225_GAMMA_CTRL8, 0x0A00); 
    ili9225_write_register(ILI9225_GAMMA_CTRL9, 0x0710); 
    ili9225_write_register(ILI9225_GAMMA_CTRL10, 0x0710); 

    ili9225_write_register(ILI9225_DISP_CTRL1, 0x0012); 
    ili9225_stop_write();
    ili9225_sleep_ms(50); 
    ili9225_start_write();
    ili9225_write_register(ILI9225_DISP_CTRL1, 0x1017);
    ili9225_stop_write();
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
    s_cs = cs;
    s_rst = rst;
    s_rs = rs;

    ili9225_spi_init(mosi, clk, host, dma_channel, clk_speed);

    ili9225_set_gpio(s_rst);
    ili9225_set_gpio(s_rs);

    ili9225_set_gpio_level(s_rst, false);
    ili9225_stop_write();

    ili9225_registers_init();
}

void ili9225_set_pixel(uint8_t red, 
						uint8_t blue,
						uint8_t green,
						size_t x,
						size_t y);

void ili9225_fill(uint8_t red,
				  uint8_t blue,
				  uint8_t green);

void ili9225_update()
{
    ili9225_set_gpio_level(s_rs, true);
    ili9225_spi_write(s_screen_buffer, sizeof(s_screen_buffer));
}