
#include "TMC5160.h"

TMC5160_SPI::TMC5160_SPI( uint8_t chipSelectPin, uint32_t fclk, const SPISettings &spiSettings, SPIClass &spi )
: TMC5160(fclk), _CS(chipSelectPin), _spiSettings(spiSettings), _spi(&spi)
{
	pinMode(chipSelectPin, OUTPUT);
}


// calls to read/write registers must be bracketed by the begin/endTransaction calls

void _chipSelect( uint8_t pin, bool select )
{
	digitalWrite(pin, select?LOW:HIGH);
}

void TMC5160_SPI::_beginTransaction()
{
	_spi->beginTransaction(_spiSettings);
	_chipSelect(_CS, true);
}

void TMC5160_SPI::_endTransaction()
{
	_chipSelect(_CS, false);
	_spi->endTransaction();
}

uint32_t TMC5160_SPI::readRegister(uint8_t address)
{
    _beginTransaction();
    _spi->transfer(address);

    uint32_t value = 0;
    for (int8_t shift = 24; shift >= 0; shift -= 8) {
        value |= (uint32_t)_spi->transfer(0x00) << shift;
    }
    _endTransaction();

    return value;
}


uint8_t TMC5160_SPI::writeRegister(uint8_t address, uint32_t data)
{
    // address register
    _beginTransaction();
    uint8_t status = _spi->transfer(address | WRITE_ACCESS);

    // send new register value
    for (int8_t shift = 24; shift >= 0; shift -= 8) {
        _spi->transfer((data >> shift) & 0xFF);
    }
    _endTransaction();

    return status;
}


