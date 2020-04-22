// Copyright (c) 2013 Jeff Rowberg
// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// 2013-06-05 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2020-04-XX - remove Arduino stuffs, refactor the code and able library usage in Raspberry boards
//      2013-05-06 - add Francesco Ferrara's Fastwire v0.24 implementation with small modifications
//      2013-05-05 - fix issue with writing bit values to words (Sasquatch/Farzanegan)
//      2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                 - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//      2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//      2011-10-03 - added automatic Arduino version detection for ease of use
//      2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
//      2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//      2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//      2011-08-02 - added support for 16-bit registers
//                 - fixed incorrect Doxygen comments on some methods
//                 - added timeout value for read operations (thanks mem @ Arduino forums)
//      2011-07-30 - changed read/write function structures to return success or byte counts
//                 - made all methods static for multi-device memory savings
//      2011-07-28 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "mpu6050_driver/i2c_dev.hpp"
#include "wiringPiI2C.h"  

I2Cdev::I2Cdev() {
}

int8_t I2Cdev::readBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t *data, uint16_t timeout) {
  uint8_t b;
  uint8_t count = readByte(dev_addr, reg_addr, &b, timeout);
  *data = b & (1 << bit_num);
  return count;
}

int8_t I2Cdev::readBitW(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint16_t *data, uint16_t timeout) {
  uint16_t b;
  uint8_t count = readWord(dev_addr, reg_addr, &b, timeout);
  *data = b & (1 << bit_num);
  return count;
}

int8_t I2Cdev::readBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start,
                        uint8_t length, uint8_t *data, uint16_t timeout) {
  // 01101001 read byte
  // 76543210 bit numbers
  //    xxx   args: bit_start=4, length=3
  //    010   masked
  //   -> 010 shifted
  uint8_t count, b;
  if ((count = readByte(dev_addr, reg_addr, &b, timeout)) != 0) {
      uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
      b &= mask;
      b >>= (bit_start - length + 1);
      *data = b;
  }

  return count;
}

int8_t I2Cdev::readBitsW(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start,
                         uint8_t length, uint16_t *data, uint16_t timeout) {
  // 1101011001101001 read byte
  // fedcba9876543210 bit numbers
  //    xxx           args: bit_start=12, length=3
  //    010           masked
  //           -> 010 shifted
  uint8_t count;
  uint16_t w;
  if ((count = readWord(dev_addr, reg_addr, &w, timeout)) != 0) {
      uint16_t mask = ((1 << length) - 1) << (bit_start - length + 1);
      w &= mask;
      w >>= (bit_start - length + 1);
      *data = w;
  }
  return count;
}

int8_t I2Cdev::readByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t timeout) {
    return readBytes(dev_addr, reg_addr, 1, data, timeout);
}

int8_t I2Cdev::readWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t *data, uint16_t timeout) {
    return readWords(dev_addr, reg_addr, 1, data, timeout);
}

int8_t I2Cdev::readBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t *data, uint16_t timeout) {

}

int8_t I2Cdev::readWords(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint16_t *data, uint16_t timeout) {
}


bool I2Cdev::writeBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t data) {
  uint8_t b;
  readByte(dev_addr, reg_addr, &b);
  b = (data != 0) ? (b | (1 << bit_num)) : (b & ~(1 << bit_num));
  return writeByte(dev_addr, reg_addr, b);
}

bool I2Cdev::writeBitW(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint16_t data) {
  uint16_t w;
  readWord(dev_addr, reg_addr, &w);
  w = (data != 0) ? (w | (1 << bit_num)) : (w & ~(1 << bit_num));
  return writeWord(dev_addr, reg_addr, w);
}

bool I2Cdev::writeBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data) {
  //      010 value to write
  // 76543210 bit numbers
  //    xxx   args: bit_start=4, length=3
  // 00011100 mask byte
  // 10101111 original value (sample)
  // 10100011 original & ~mask
  // 10101011 masked | value
  uint8_t b;
  if (readByte(dev_addr, reg_addr, &b) != 0) {
    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    data <<= (bit_start - length + 1);  // shift data into correct position
    data &= mask;  // zero all non-important bits in data
    b &= ~(mask);  // zero all important bits in existing byte
    b |= data;  // combine data with existing byte
    return writeByte(dev_addr, reg_addr, b);
  }

  return false;
}

bool I2Cdev::writeBitsW(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t data) {
  //              010 value to write
  // fedcba9876543210 bit numbers
  //    xxx           args: bit_start=12, length=3
  // 0001110000000000 mask word
  // 1010111110010110 original value (sample)
  // 1010001110010110 original & ~mask
  // 1010101110010110 masked | value
  uint16_t w;
  if (readWord(dev_addr, reg_addr, &w) != 0) {
    uint16_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    data <<= (bit_start - length + 1);  // shift data into correct position
    data &= mask;  // zero all non-important bits in data
    w &= ~(mask);  // zero all important bits in existing word
    w |= data;  // combine data with existing word
    return writeWord(dev_addr, reg_addr, w);
  }

  return false;
}

bool I2Cdev::writeByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    return writeBytes(dev_addr, reg_addr, 1, &data);
}

bool I2Cdev::writeWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t data) {
    return writeWords(dev_addr, reg_addr, 1, &data);
}

bool I2Cdev::writeBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t* data) {

}

bool I2Cdev::writeWords(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint16_t* data) {
}
