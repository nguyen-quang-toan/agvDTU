#include "BKD_Spio.h"

Spio::Spio() {
  Spio(40, 41, 38, 39);
}

Spio::Spio(uint8_t ss, uint8_t clock, uint8_t dataIn, uint8_t dataOut) {
  this->pinDataIn = dataIn;
  this->pinDataOut = dataOut;
  this->pinClock = clock;
  this->pinSS = ss;
}

void Spio::init() {
  this->SpiInstance = SWSPI(this->pinDataIn, this->pinDataOut, this->pinClock);
  this->SpiInstance.init();
  pinMode(this->pinSS, OUTPUT);
}

void Spio::writeBit(uint16_t bitNumber, uint8_t state) {
  if (state == 1) {
    this->bufferOutput |= (uint32_t)  (1UL << bitNumber);
  } else {
    this->bufferOutput &= (uint32_t) ~(1UL << bitNumber);
  }
}

uint8_t Spio::readBit(uint32_t buffer, uint8_t bitNumber) {
  return (buffer >> bitNumber) & 0x01;
}

uint32_t Spio::reverseBytes(uint32_t buffer, uint8_t numberOfByte) {
  uint32_t res = 0;
  for (uint8_t i = 0; i < numberOfByte / 2; i++) {
    uint8_t lowerByte = (buffer >> (8 * i)) & 0xFF;
    uint8_t higherByte = (buffer >> (8 * ((numberOfByte - 1) - i))) & 0xFF;
    uint8_t dumpByte = lowerByte;
    lowerByte = higherByte;
    higherByte = dumpByte;
    res |= lowerByte << (8 * i);
    res |= higherByte << (8 * (4 - i));
  }
  return res;
}

void Spio::setMotor(uint8_t index, int8_t direction) {
  if (direction == 0) {
    this->writeBit(index * 2, 0);
    this->writeBit(index * 2 + 1, 0);
  } else if (direction == 1) {
    this->writeBit(index * 2, 1);
    this->writeBit(index * 2 + 1, 0);
  } else if (direction == -1) {
    this->writeBit(index * 2, 0);
    this->writeBit(index * 2 + 1, 1);
  }
}

void Spio::onLoop() {
  this->SpiInstance.beginTransaction(MSBFIRST, 3);
  digitalWrite(this->pinSS, 1);
  this->bufferInput = this->SpiInstance.transfer32(bufferOutput);
  this->bufferInput = BYTESWAP32(this->bufferInput);
  // this->bufferInput = reverseBytes(this->bufferInput, 4);
  digitalWrite(this->pinSS, 0);
  this->SpiInstance.endTransaction();
}

SWSPI::SWSPI(uint8_t dataIn, uint8_t dataOut, uint8_t clock) {
  _dataIn = dataIn;
  _dataOut = dataOut;
  _clock = clock;

  pinMode(_dataIn, INPUT_PULLUP);
  pinMode(_dataOut, OUTPUT);
  pinMode(_clock, OUTPUT);
  digitalWrite(_dataOut, LOW);
  digitalWrite(_clock, LOW);
}

SWSPI::SWSPI() {}

void SWSPI::init() {
  _bitOrder = MSBFIRST;
  _dataMode = SPI_MODE0;
  _clockPolarity = 0;
  _clockPhase = 0;
};

void SWSPI::end(){};

void SWSPI::setBitOrder(uint8_t bitOrder) {
  _bitOrder = bitOrder;
};

void SWSPI::setDataMode(uint8_t dataMode) {
  //  if (dataMode > 3) return;
  _dataMode = dataMode;
  _clockPhase = _dataMode & 0x01;
  _clockPolarity = ((_dataMode & 0x02) > 0);
};

void SWSPI::beginTransaction(uint8_t bitOrder, uint8_t dataMode) {
  _bitOrder = bitOrder;
  _dataMode = dataMode;

  //  MODE 0 = default
  _clockPolarity = ((_dataMode & 0x02) > 0);

  //  initialize clock.
  digitalWrite(_clock, _clockPolarity);
  //  noInterrupts();
};

void SWSPI::endTransaction(){
  //  interrupts();
};

uint8_t SWSPI::transfer(uint8_t data) {

  if (_bitOrder == MSBFIRST) {
    data = _bitReverse(data);
  }
  uint8_t value = _transfer(data);
  return value;
};

uint16_t SWSPI::transfer16(uint16_t data) {
  if (_bitOrder == MSBFIRST) {
    data = _bitReverse(data);
  }
  uint16_t value = 0;
  value += _transfer(data & 0xFF);
  value <<= 8;
  data >>= 8;
  value += _transfer(data & 0xFF);
  return value;
};


uint32_t SWSPI::transfer24(uint32_t data) {
  if (_bitOrder == MSBFIRST) {
    data = _bitReverse(data) >> 8;
  }
  uint32_t value = 0;
  value += _transfer(data & 0xFF);
  value <<= 8;
  data >>= 8;
  value += _transfer(data & 0xFF);
  value <<= 8;
  data >>= 8;
  value += _transfer(data & 0xFF);
  return value;
};

uint32_t SWSPI::transfer32(uint32_t data) {
  if (_bitOrder == MSBFIRST) {
    data = _bitReverse(data);
  }
  uint32_t value = 0;
  value += _transfer(data & 0xFF);
  value <<= 8;
  data >>= 8;
  value += _transfer(data & 0xFF);
  value <<= 8;
  data >>= 8;
  value += _transfer(data & 0xFF);
  value <<= 8;
  data >>= 8;
  value += _transfer(data & 0xFF);
  return value;
};

void SWSPI::transfer(const void *buf, size_t count) {
  for (size_t i = 0; i < count; i++) {
    transfer(((uint8_t *)buf)[i]);
  }
};

uint8_t SWSPI::_transfer(uint8_t data) {
  uint8_t rv = 0;
  uint8_t _data = data;

  if (_dataMode == 0) {
    //  SPI_MODE0
    //  sampled on rising edge
    //  shift out on falling edge
    for (uint8_t mask = 0x01; mask; mask <<= 1) {
      digitalWrite(_dataOut, (_data & mask) > 0);
      //  Serial.print((_data & mask) > 0);
      digitalWrite(_clock, HIGH);
      digitalWrite(_clock, LOW);
      rv <<= 1;
      rv += digitalRead(_dataIn);
    }
  } else if (_dataMode == 1) {
    //  SPI_MODE1
    //  shift out on rising edge
    //  sampled on falling edge
    for (uint8_t mask = 0x01; mask; mask <<= 1) {
      digitalWrite(_clock, HIGH);
      rv <<= 1;
      rv += digitalRead(_dataIn);
      digitalWrite(_dataOut, (_data & mask) > 0);
      //  Serial.print((_data & mask) > 0);
      digitalWrite(_clock, LOW);
    }
  } else if (_dataMode == 2) {
    //  SPI_MODE2
    //  sampled on falling edge
    //  shift out on rising edge
    for (uint8_t mask = 0x01; mask; mask <<= 1) {
      digitalWrite(_dataOut, (_data & mask) > 0);
      //  Serial.print((_data & mask) > 0);
      digitalWrite(_clock, LOW);
      digitalWrite(_clock, HIGH);
      rv <<= 1;
      rv += digitalRead(_dataIn);
    }
  } else if (_dataMode == 3) {
    //  SPI_MODE3
    //  sampled on rising edge
    //  shift out on falling edge
    for (uint8_t mask = 0x01; mask; mask <<= 1) {
      digitalWrite(_clock, LOW);
      rv <<= 1;
      rv += digitalRead(_dataIn);
      digitalWrite(_dataOut, (_data & mask) > 0);
      //  Serial.print((_data & mask) > 0);
      digitalWrite(_clock, HIGH);
    }
  }
  //  Serial.print("  ");
  return rv;
};

// from https://github.com/RobTillaart/bitHelpers
inline uint8_t SWSPI::_bitReverse(uint8_t value) {
  uint8_t x = value;
  x = (((x & 0xAA) >> 1) | ((x & 0x55) << 1));
  x = (((x & 0xCC) >> 2) | ((x & 0x33) << 2));
  x = (x >> 4) | (x << 4);
  return x;
}


uint16_t SWSPI::_bitReverse(uint16_t value) {
  uint16_t x = value;
  x = (((x & 0xAAAA) >> 1) | ((x & 0x5555) << 1));
  x = (((x & 0xCCCC) >> 2) | ((x & 0x3333) << 2));
  x = (((x & 0xF0F0) >> 4) | ((x & 0x0F0F) << 4));
  x = (x >> 8) | (x << 8);
  return x;
}

uint32_t SWSPI::_bitReverse(uint32_t value) {
  uint32_t x = value;
  x = (((x & 0xAAAAAAAA) >> 1) | ((x & 0x55555555) << 1));
  x = (((x & 0xCCCCCCCC) >> 2) | ((x & 0x33333333) << 2));
  x = (((x & 0xF0F0F0F0) >> 4) | ((x & 0x0F0F0F0F) << 4));
  x = (((x & 0xFF00FF00) >> 8) | ((x & 0x00FF00FF) << 8));
  x = (x >> 16) | (x << 16);
  return x;
}