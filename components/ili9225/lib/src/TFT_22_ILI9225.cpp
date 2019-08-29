#include "TFT_22_ILI9225.h"

#include <limits.h>
#include <GPIO.h>
#include <FreeRTOS.h>
#include "pgmspace.h"


/* ILI9225 LCD Registers */
#define ILI9225_DRIVER_OUTPUT_CTRL      (0x01u)  // Driver Output Control
#define ILI9225_LCD_AC_DRIVING_CTRL     (0x02u)  // LCD AC Driving Control
#define ILI9225_ENTRY_MODE              (0x03u)  // Entry Mode
#define ILI9225_DISP_CTRL1              (0x07u)  // Display Control 1
#define ILI9225_BLANK_PERIOD_CTRL1      (0x08u)  // Blank Period Control
#define ILI9225_FRAME_CYCLE_CTRL        (0x0Bu)  // Frame Cycle Control
#define ILI9225_INTERFACE_CTRL          (0x0Cu)  // Interface Control
#define ILI9225_OSC_CTRL                (0x0Fu)  // Osc Control
#define ILI9225_POWER_CTRL1             (0x10u)  // Power Control 1
#define ILI9225_POWER_CTRL2             (0x11u)  // Power Control 2
#define ILI9225_POWER_CTRL3             (0x12u)  // Power Control 3
#define ILI9225_POWER_CTRL4             (0x13u)  // Power Control 4
#define ILI9225_POWER_CTRL5             (0x14u)  // Power Control 5
#define ILI9225_VCI_RECYCLING           (0x15u)  // VCI Recycling
#define ILI9225_RAM_ADDR_SET1           (0x20u)  // Horizontal GRAM Address Set
#define ILI9225_RAM_ADDR_SET2           (0x21u)  // Vertical GRAM Address Set
#define ILI9225_GRAM_DATA_REG           (0x22u)  // GRAM Data Register
#define ILI9225_GATE_SCAN_CTRL          (0x30u)  // Gate Scan Control Register
#define ILI9225_VERTICAL_SCROLL_CTRL1   (0x31u)  // Vertical Scroll Control 1 Register
#define ILI9225_VERTICAL_SCROLL_CTRL2   (0x32u)  // Vertical Scroll Control 2 Register
#define ILI9225_VERTICAL_SCROLL_CTRL3   (0x33u)  // Vertical Scroll Control 3 Register
#define ILI9225_PARTIAL_DRIVING_POS1    (0x34u)  // Partial Driving Position 1 Register
#define ILI9225_PARTIAL_DRIVING_POS2    (0x35u)  // Partial Driving Position 2 Register
#define ILI9225_HORIZONTAL_WINDOW_ADDR1 (0x36u)  // Horizontal Address Start Position
#define ILI9225_HORIZONTAL_WINDOW_ADDR2 (0x37u)  // Horizontal Address End Position
#define ILI9225_VERTICAL_WINDOW_ADDR1   (0x38u)  // Vertical Address Start Position
#define ILI9225_VERTICAL_WINDOW_ADDR2   (0x39u)  // Vertical Address End Position
#define ILI9225_GAMMA_CTRL1             (0x50u)  // Gamma Control 1
#define ILI9225_GAMMA_CTRL2             (0x51u)  // Gamma Control 2
#define ILI9225_GAMMA_CTRL3             (0x52u)  // Gamma Control 3
#define ILI9225_GAMMA_CTRL4             (0x53u)  // Gamma Control 4
#define ILI9225_GAMMA_CTRL5             (0x54u)  // Gamma Control 5
#define ILI9225_GAMMA_CTRL6             (0x55u)  // Gamma Control 6
#define ILI9225_GAMMA_CTRL7             (0x56u)  // Gamma Control 7
#define ILI9225_GAMMA_CTRL8             (0x57u)  // Gamma Control 8
#define ILI9225_GAMMA_CTRL9             (0x58u)  // Gamma Control 9
#define ILI9225_GAMMA_CTRL10            (0x59u)  // Gamma Control 10

#define ILI9225C_INVOFF  0x20
#define ILI9225C_INVON   0x21

#define OUTPUT 0
#define HIGH   1
#define LOW    0


#define SPI_DC_HIGH()           digitalWrite(_rs, HIGH)
#define SPI_DC_LOW()            digitalWrite(_rs, LOW)
#define SPI_CS_HIGH()           digitalWrite(_cs, HIGH)
#define SPI_CS_LOW()            digitalWrite(_cs, LOW)

#define SSPI_MOSI_HIGH()        digitalWrite(_sdi, HIGH)
#define SSPI_MOSI_LOW()         digitalWrite(_sdi, LOW)
#define SSPI_SCK_HIGH()         digitalWrite(_clk, HIGH)
#define SSPI_SCK_LOW()          digitalWrite(_clk, LOW)

#define HSPI_WRITE(b)            _spiWrite(b)

inline void pinMode(int pin, int mode)
{
    (void)mode;
    ESP32CPP::GPIO::setOutput((gpio_num_t)pin);
}

inline void digitalWrite(int pin, int value)
{
    ESP32CPP::GPIO::write((gpio_num_t)pin, value);
}

inline void delay(uint32_t ms)
{
    FreeRTOS::sleep(ms);
}

inline int bitRead(uint32_t x, uint8_t n) {
    return ((x & 1 << n) != 0);
}

// Constructor when using software SPI.  All output pins are configurable.
TFT_22_ILI9225::TFT_22_ILI9225(int8_t rst, int8_t rs, int8_t cs, int8_t sdi, int8_t clk) {
    _rst  = rst;
    _rs   = rs;
    _cs   = cs;
    _sdi  = sdi;
    _clk  = clk;
}

void TFT_22_ILI9225::_spi_init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg;
    spi_device_interface_config_t devcfg;

    buscfg.mosi_io_num = _sdi;
    buscfg.miso_io_num =  -1;
    buscfg.sclk_io_num = _clk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 0;
    buscfg.flags = SPICOMMON_BUSFLAG_MASTER;

    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.mode = 0;
    devcfg.duty_cycle_pos = 0;
    devcfg.cs_ena_pretrans = 0;
    devcfg.cs_ena_posttrans = 0;
    devcfg.clock_speed_hz = 26600000;
    devcfg.input_delay_ns = 0;
    devcfg.spics_io_num = -1;
    devcfg.flags = 0;
    devcfg.queue_size = 4096;
    devcfg.pre_cb = NULL;
    devcfg.post_cb = NULL;

    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);
}

void TFT_22_ILI9225::begin()
{
    // Set up reset pin
    if (_rst > 0) {
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst, LOW);
    }

    // Control pins
    pinMode(_rs, OUTPUT);
    digitalWrite(_rs, LOW);
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    _spi_init();

    // Initialization Code
    if (_rst > 0) {
        digitalWrite(_rst, HIGH); // Pull the reset pin high to release the ILI9225C from the reset status
        delay(1);
        digitalWrite(_rst, LOW); // Pull the reset pin low to reset ILI9225
        delay(10);
        digitalWrite(_rst, HIGH); // Pull the reset pin high to release the ILI9225C from the reset status
        delay(50);
    }

    /* Start Initial Sequence */

    /* Set SS bit and direction output from S528 to S1 */
    startWrite();
    _writeRegister(ILI9225_POWER_CTRL1, 0x0000); // Set SAP,DSTB,STB
    _writeRegister(ILI9225_POWER_CTRL2, 0x0000); // Set APON,PON,AON,VCI1EN,VC
    _writeRegister(ILI9225_POWER_CTRL3, 0x0000); // Set BT,DC1,DC2,DC3
    _writeRegister(ILI9225_POWER_CTRL4, 0x0000); // Set GVDD
    _writeRegister(ILI9225_POWER_CTRL5, 0x0000); // Set VCOMH/VCOML voltage
    endWrite();
    delay(40);

    // Power-on sequence
    startWrite();
    _writeRegister(ILI9225_POWER_CTRL2, 0x0018); // Set APON,PON,AON,VCI1EN,VC
    _writeRegister(ILI9225_POWER_CTRL3, 0x6121); // Set BT,DC1,DC2,DC3
    _writeRegister(ILI9225_POWER_CTRL4, 0x006F); // Set GVDD   /*007F 0088 */
    _writeRegister(ILI9225_POWER_CTRL5, 0x495F); // Set VCOMH/VCOML voltage
    _writeRegister(ILI9225_POWER_CTRL1, 0x0800); // Set SAP,DSTB,STB
    endWrite();
    delay(10);
    startWrite();
    _writeRegister(ILI9225_POWER_CTRL2, 0x103B); // Set APON,PON,AON,VCI1EN,VC
    endWrite();
    delay(50);

    startWrite();
    _writeRegister(ILI9225_DRIVER_OUTPUT_CTRL, 0x011C); // set the display line number and display direction
    _writeRegister(ILI9225_LCD_AC_DRIVING_CTRL, 0x0100); // set 1 line inversion
    _writeRegister(ILI9225_ENTRY_MODE, 0x1038); // set GRAM write direction and BGR=1.
    _writeRegister(ILI9225_DISP_CTRL1, 0x0000); // Display off
    _writeRegister(ILI9225_BLANK_PERIOD_CTRL1, 0x0808); // set the back porch and front porch
    _writeRegister(ILI9225_FRAME_CYCLE_CTRL, 0x1100); // set the clocks number per line
    _writeRegister(ILI9225_INTERFACE_CTRL, 0x0000); // CPU interface
    _writeRegister(ILI9225_OSC_CTRL, 0x0D01); // Set Osc  /*0e01*/
    _writeRegister(ILI9225_VCI_RECYCLING, 0x0020); // Set VCI recycling
    _writeRegister(ILI9225_RAM_ADDR_SET1, 0x0000); // RAM Address
    _writeRegister(ILI9225_RAM_ADDR_SET2, 0x0000); // RAM Address

    /* Set GRAM area */
    _writeRegister(ILI9225_GATE_SCAN_CTRL, 0x0000);
    _writeRegister(ILI9225_VERTICAL_SCROLL_CTRL1, 0x00DB);
    _writeRegister(ILI9225_VERTICAL_SCROLL_CTRL2, 0x0000);
    _writeRegister(ILI9225_VERTICAL_SCROLL_CTRL3, 0x0000);
    _writeRegister(ILI9225_PARTIAL_DRIVING_POS1, 0x00DB);
    _writeRegister(ILI9225_PARTIAL_DRIVING_POS2, 0x0000);
    _writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF);
    _writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR2, 0x0000);
    _writeRegister(ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB);
    _writeRegister(ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000);

    /* Set GAMMA curve */
    _writeRegister(ILI9225_GAMMA_CTRL1, 0x0000);
    _writeRegister(ILI9225_GAMMA_CTRL2, 0x0808);
    _writeRegister(ILI9225_GAMMA_CTRL3, 0x080A);
    _writeRegister(ILI9225_GAMMA_CTRL4, 0x000A);
    _writeRegister(ILI9225_GAMMA_CTRL5, 0x0A08);
    _writeRegister(ILI9225_GAMMA_CTRL6, 0x0808);
    _writeRegister(ILI9225_GAMMA_CTRL7, 0x0000);
    _writeRegister(ILI9225_GAMMA_CTRL8, 0x0A00);
    _writeRegister(ILI9225_GAMMA_CTRL9, 0x0710);
    _writeRegister(ILI9225_GAMMA_CTRL10, 0x0710);

    _writeRegister(ILI9225_DISP_CTRL1, 0x0012);
    endWrite();
    delay(50);
    startWrite();
    _writeRegister(ILI9225_DISP_CTRL1, 0x1017);
    endWrite();

    setOrientation(0);

    clear();
}

void TFT_22_ILI9225::_spiWrite(const void * data, size_t size)
{
    spi_transaction_t trans_desc;
    trans_desc.flags     = 0;
    trans_desc.length    = size * 8;
    trans_desc.rxlength  = 0;
    trans_desc.tx_buffer = data;
    trans_desc.rx_buffer = NULL;

    esp_err_t rc = ::spi_device_transmit(_spi, &trans_desc);
}

void TFT_22_ILI9225::_spiWrite(uint8_t b) {
    _spiWrite(&b, sizeof(b));
}


void TFT_22_ILI9225::_spiWriteCommand(uint8_t c) {
    SPI_DC_LOW();
    SPI_CS_LOW();
    _spiWrite(c);
    SPI_CS_HIGH();
}


void TFT_22_ILI9225::_spiWriteData(uint8_t c) {
    SPI_DC_HIGH();
    SPI_CS_LOW();
    _spiWrite(c);
    SPI_CS_HIGH();
}

void TFT_22_ILI9225::_orientCoordinates(uint16_t &x1, uint16_t &y1) {

    switch (_orientation) {
    case 0:  // ok
        break;
    case 1: // ok
        y1 = _maxY - y1 - 1;
        _swap(x1, y1);
        break;
    case 2: // ok
        x1 = _maxX - x1 - 1;
        y1 = _maxY - y1 - 1;
        break;
    case 3: // ok
        x1 = _maxX - x1 - 1;
        _swap(x1, y1);
        break;
    }
}

void TFT_22_ILI9225::setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    _setWindow(x0, y0, x1, y1, L2R_TopDown);
}

void TFT_22_ILI9225::_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    _setWindow( x0, y0, x1, y1, TopDown_L2R ); // default for drawing characters
}

void TFT_22_ILI9225::_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, autoIncMode_t mode) {
    
    // clip to TFT-Dimensions
    x0 = std::min( x0, (uint16_t) (_maxX - 1) );
    x1 = std::min( x1, (uint16_t) (_maxX - 1) );
    y0 = std::min( y0, (uint16_t) (_maxY - 1) );
    y1 = std::min( y1, (uint16_t) (_maxY - 1) );
    _orientCoordinates(x0, y0);
    _orientCoordinates(x1, y1);

    if (x1 < x0) _swap(x0, x1);
    if (y1 < y0) _swap(y0, y1);

    startWrite();
    // autoincrement mode
    if ( _orientation > 0 ) mode = modeTab[_orientation - 1][mode];
    _writeRegister(ILI9225_ENTRY_MODE, 0x1000 | ( mode << 3) );
    _writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR1, x1);
    _writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR2, x0);

    _writeRegister(ILI9225_VERTICAL_WINDOW_ADDR1, y1);
    _writeRegister(ILI9225_VERTICAL_WINDOW_ADDR2, y0);
    
    // starting position within window and increment/decrement direction
    switch ( mode >> 1 ) {
    case 0:
        _writeRegister(ILI9225_RAM_ADDR_SET1, x1);
        _writeRegister(ILI9225_RAM_ADDR_SET2, y1);
        break;
    case 1:
        _writeRegister(ILI9225_RAM_ADDR_SET1, x0);
        _writeRegister(ILI9225_RAM_ADDR_SET2, y1);
        break;
    case 2:
        _writeRegister(ILI9225_RAM_ADDR_SET1, x1);
        _writeRegister(ILI9225_RAM_ADDR_SET2, y0);
        break;
    case 3:
        _writeRegister(ILI9225_RAM_ADDR_SET1, x0);
        _writeRegister(ILI9225_RAM_ADDR_SET2, y0);
        break;
    }
    _writeCommand16( ILI9225_GRAM_DATA_REG );

    //_writeRegister(ILI9225_RAM_ADDR_SET1,x0);
    //_writeRegister(ILI9225_RAM_ADDR_SET2,y0);

    //_writeCommand(0x00, 0x22);

    endWrite();
}

void TFT_22_ILI9225::drawBitmapImage(const uint16_t * image)
{ 
    for (uint16_t i = 0; i < ILI9225_LCD_HEIGHT; i++)
    {
        for (uint16_t j = 0; j < ILI9225_LCD_WIDTH; j++)
        {
            _writeData16(image[ILI9225_LCD_WIDTH * i + j]);
        }
    }
}

void TFT_22_ILI9225::_resetWindow() {
    _writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF);
    _writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR2, 0x0000);
    _writeRegister(ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB);
    _writeRegister(ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000);

}

void TFT_22_ILI9225::clear() {
}

void TFT_22_ILI9225::invert(bool flag) {
    startWrite();
    _writeCommand16(flag ? ILI9225C_INVON : ILI9225C_INVOFF);
    //_writeCommand(0x00, flag ? ILI9225C_INVON : ILI9225C_INVOFF);
    endWrite();
}


void TFT_22_ILI9225::setDisplay(bool flag) {
    if (flag) {
        startWrite();
        _writeRegister(0x00ff, 0x0000);
        _writeRegister(ILI9225_POWER_CTRL1, 0x0000);
        endWrite();
        delay(50);
        startWrite();
        _writeRegister(ILI9225_DISP_CTRL1, 0x1017);
        endWrite();
        delay(200);
    } else {
        startWrite();
        _writeRegister(0x00ff, 0x0000);
        _writeRegister(ILI9225_DISP_CTRL1, 0x0000);
        endWrite();
        delay(50);
        startWrite();
        _writeRegister(ILI9225_POWER_CTRL1, 0x0003);
        endWrite();
        delay(200);
    }
}


void TFT_22_ILI9225::setOrientation(uint8_t orientation) {

    _orientation = orientation % 4;

    switch (_orientation) {
    case 0:
        _maxX = ILI9225_LCD_WIDTH;
        _maxY = ILI9225_LCD_HEIGHT;

        break;
    case 1:
        _maxX = ILI9225_LCD_HEIGHT;
        _maxY = ILI9225_LCD_WIDTH;
        break;
    case 2:
        _maxX = ILI9225_LCD_WIDTH;
        _maxY = ILI9225_LCD_HEIGHT;
        break;
    case 3:
        _maxX = ILI9225_LCD_HEIGHT;
        _maxY = ILI9225_LCD_WIDTH;
        break;
    }
}


uint8_t TFT_22_ILI9225::getOrientation() {
    return _orientation;
}


uint16_t TFT_22_ILI9225::maxX() {
    return _maxX;
}


uint16_t TFT_22_ILI9225::maxY() {
    return _maxY;
}


uint16_t TFT_22_ILI9225::setColor(uint8_t red8, uint8_t green8, uint8_t blue8) {
    // rgb16 = red5 green6 blue5
    return (red8 >> 3) << 11 | (green8 >> 2) << 5 | (blue8 >> 3);
}


void TFT_22_ILI9225::splitColor(uint16_t rgb, uint8_t &red, uint8_t &green, uint8_t &blue) {
    // rgb16 = red5 green6 blue5
    red   = (rgb & 0b1111100000000000) >> 11 << 3;
    green = (rgb & 0b0000011111100000) >>  5 << 2;
    blue  = (rgb & 0b0000000000011111)       << 3;
}


void TFT_22_ILI9225::_swap(uint16_t &a, uint16_t &b) {
    uint16_t w = a;
    a = b;
    b = w;
}

void TFT_22_ILI9225::_writeCommand16(uint16_t command) {
    SPI_DC_LOW();
    SPI_CS_LOW();
# ifdef HSPI_WRITE16
    HSPI_WRITE16(command);
#else
    HSPI_WRITE(command >> 8);
    HSPI_WRITE(0x00ff & command);
#endif

    SPI_CS_HIGH();
}

void TFT_22_ILI9225::_writeData16(uint16_t data) {
    SPI_DC_HIGH();
    SPI_CS_LOW();
# ifdef HSPI_WRITE16
    HSPI_WRITE16(data);
#else
    HSPI_WRITE(data >> 8);
    HSPI_WRITE(0x00ff & data);
#endif
    SPI_CS_HIGH();
}

void TFT_22_ILI9225::_writeRegister(uint16_t reg, uint16_t data) {
    _writeCommand16(reg);
    _writeData16(data);
}

void TFT_22_ILI9225::startWrite(void) {
    SPI_CS_LOW();
}


void TFT_22_ILI9225::endWrite(void) {
    SPI_CS_HIGH();
}