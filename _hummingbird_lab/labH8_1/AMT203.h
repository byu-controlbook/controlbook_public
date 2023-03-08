/**
 * \file AMT203.h
 * \author Daniel Koch <daniel.p.koch@gmail.com>
 *
 * Arduino library for CUI AMT203 encoder operating in SPI mode
 */

#ifndef AMT203_H
#define AMT203_H

#include <SPI.h>
#include <stdint.h>

// commands
#define AMT203_CMD_NOP_A5 0x00
#define AMT203_CMD_RD_POS 0x10
#define AMT203_CMD_SET_ZERO_POINT 0x70

// responses
#define AMT203_RES_WAIT 0xA5
#define AMT203_RES_RD_POS AMT203_CMD_RD_POS
#define AMT203_RES_ZERO_SUCCESS 0x80

// SPI settings
#define AMT203_SPI_SPEED 1000000  //could mess with this.1000000 originally 
#define AMT203_SPI_ORDER MSBFIRST
#define AMT203_SPI_MODE SPI_MODE0

// other
#define AMT203_POSITIONS 4096
#define AMT203_DELAY_US 20

class AMT203
{
public:
  AMT203(int cs_pin) :
    cs_pin_(cs_pin),
    spi_settings_(AMT203_SPI_SPEED, AMT203_SPI_ORDER, AMT203_SPI_MODE)
  {}

  void init()
  {
    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);
  }

  float read()
  {
    SPI.beginTransaction(spi_settings_);

    transfer_byte(AMT203_CMD_RD_POS);
    delayMicroseconds(AMT203_DELAY_US);

    unsigned int count = 0;
    while (transfer_byte(AMT203_CMD_NOP_A5) != AMT203_RES_RD_POS && count++ < READ_TIMEOUT)
    {
      delayMicroseconds(AMT203_DELAY_US);
    }
    delayMicroseconds(AMT203_DELAY_US);

    uint8_t msb = transfer_byte(AMT203_CMD_NOP_A5);
    delayMicroseconds(AMT203_DELAY_US);
    uint8_t lsb = transfer_byte(AMT203_CMD_NOP_A5);

    SPI.endTransaction();

    uint16_t data = (msb << 8) | lsb;
    float angle = (float) data / AMT203_POSITIONS * 2*M_PI;

    while (angle > M_PI)
    {
      angle -= 2*M_PI;
    }
    while (angle < -M_PI)
    {
      angle += 2*M_PI;
    }

    return angle;
  }

  bool zero()
  {
    SPI.beginTransaction(spi_settings_);

    transfer_byte(AMT203_CMD_SET_ZERO_POINT);
    delayMicroseconds(AMT203_DELAY_US);

    unsigned int count = 0;
    while (transfer_byte(AMT203_CMD_NOP_A5) != AMT203_RES_ZERO_SUCCESS && count++ < ZERO_TIMEOUT)
    {
      delayMicroseconds(AMT203_DELAY_US);
    }
    SPI.endTransaction();

    if (count >= ZERO_TIMEOUT)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

private:
  static const unsigned int READ_TIMEOUT = 10;
  static const unsigned int ZERO_TIMEOUT = 500;

  int cs_pin_;
  SPISettings spi_settings_;

  uint8_t transfer_byte(uint8_t byte)
  {
    digitalWrite(cs_pin_, LOW);
    uint8_t received = SPI.transfer(byte);
    digitalWrite(cs_pin_, HIGH);
    return received;
  }
};

#endif // AMT203_H
