#ifndef SPIO
#define SPIO

#include "stdlib.h"
#include "Arduino.h"
#include "SPI.h"
#define REVERSE -1
#define FORWARD 1
#define STOP 0
#define BYTESWAP32(z) ((uint32_t)((z & 0xFF) << 24 | ((z >> 8) & 0xFF) << 16 | ((z >> 16) & 0xFF) << 8 | ((z >> 24) & 0xFF)))

class SWSPI {
public:

  SWSPI();

  SWSPI(uint8_t dataIn, uint8_t dataOut, uint8_t clock);

  void init();

  void end();

  //  LSBFIRST or MSBFIRST
  void setBitOrder(uint8_t bitOrder);

  // SPI_MODE0 .. SPI_MODE3
  void setDataMode(uint8_t dataMode);

  // void setClockDivider(uint8_t clockDiv)


  //  TRANSFER FUNCTIONS
  void beginTransaction(uint8_t bitOrder, uint8_t dataMode);

  void endTransaction();

  //  blocking for ...
  //  optimization, look at FastShiftInOut, fix #9
  uint8_t transfer(uint8_t data);
  uint16_t transfer16(uint16_t data);
  uint32_t transfer24(uint32_t data);
  uint32_t transfer32(uint32_t data);

  void transfer(const void* buf, size_t count);

  uint8_t _transfer(uint8_t data);

  // from https://github.com/RobTillaart/bitHelpers
  inline uint8_t _bitReverse(uint8_t value);

  uint16_t _bitReverse(uint16_t value);
  uint32_t _bitReverse(uint32_t value);

  //  SPI pins
  uint8_t _dataIn;
  uint8_t _dataOut;
  uint8_t _clock;

  //  SPI parameters
  uint8_t _bitOrder = MSBFIRST;
  uint8_t _dataMode = SPI_MODE0;
  uint8_t _clockPolarity = 0;
  uint8_t _clockPhase = 0;
};

class Spio {
public:
  Spio();
  Spio(uint8_t ss, uint8_t clock, uint8_t dataIn, uint8_t dataOut);
  uint32_t bufferInput;
  uint32_t bufferOutput;
  SWSPI SpiInstance;
  uint8_t pinDataIn;
  uint8_t pinDataOut;
  uint8_t pinClock;
  uint8_t pinSS;
  void init();
  void onLoop();
  uint8_t readBit(uint32_t buffer, uint8_t bitNumber);
  void writeBit(uint16_t bitNumber, uint8_t state);
  void setMotor(uint8_t index, int8_t direction);
  uint32_t reverseBytes(uint32_t buffer, uint8_t numberOfByte);
};

#endif