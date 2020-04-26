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

#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <stdexcept>

#include "mpu6050_driver/i2c_dev.hpp"

I2Cdev::I2Cdev(uint8_t dev_addr) : dev_addr_(dev_addr) {}

void I2Cdev::openI2CBus(const std::string& bus_uri) {
  if ((i2c_bus_fd_ = open(bus_uri.c_str(), O_RDWR)) < 0)
    throwIOSystemError("Error obtaining I2C system file descriptor");

  if (ioctl(i2c_bus_fd_, I2C_SLAVE, dev_addr_) < 0) {
    std::stringstream error_msg;
    error_msg << "Failed to set I2C_SLAVE at address: 0x" << std::hex << dev_addr_;
    throwIOSystemError(error_msg.str());
  }
}

void I2Cdev::readBytes(uint8_t reg_addr, uint8_t length, uint8_t *data) {
  if (write(i2c_bus_fd_, &reg_addr, 1) < 0) throwIOSystemError("Failed to write to the I2C bus");

  int bytes_received = read(i2c_bus_fd_, data, length);
  if (bytes_received < 0 || bytes_received != length) throwIOSystemError("Failed to read from the I2C bus");
}

void I2Cdev::readWords(uint8_t reg_addr, uint8_t length, uint16_t *data) {
  if (write(i2c_bus_fd_, &reg_addr, 1) < 0) throwIOSystemError("Failed to write to the I2C bus");

  int in_buf_length = length * 2;
  uint8_t *in_buf = new uint8_t[in_buf_length];

  int bytes_received = read(i2c_bus_fd_, data, in_buf_length);
  if (bytes_received < 0 || bytes_received != in_buf_length) {
    delete[] in_buf;
    throwIOSystemError("Failed to read from the I2C bus");
  }

  for (size_t i = 0; i < length; i++) data[i] = (in_buf[i] << 8) | in_buf[i+1];

  delete[] in_buf;
}

uint8_t I2Cdev::readByte(uint8_t reg_addr) {
  uint8_t byte_read;
  this->readBytes(reg_addr, 1, &byte_read);
  return byte_read;
}

uint16_t I2Cdev::readWord(uint8_t reg_addr) {
  uint16_t word_read;
  this->readWords(reg_addr, 1, &word_read);
  return word_read;
}

uint8_t I2Cdev::readBitOfByte(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint8_t>(bit_num);
  return this->readByte(reg_addr) & (1 << bit_num);
}

uint16_t I2Cdev::readBitOfWord(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint16_t>(bit_num);
  return this->readWord(reg_addr) & (1 << bit_num);
}

uint8_t I2Cdev::readBitsOfByte(uint8_t reg_addr, uint8_t bit_start, uint8_t length) {
  this->checkBitsRange<uint8_t>(bit_start, length);
  uint8_t mask = this->generateBitMask<uint8_t>(bit_start, length);
  uint8_t byte_read = this->readByte(reg_addr);
  return (byte_read & mask) >> bit_start;
}

uint16_t I2Cdev::readBitsOfWord(uint8_t reg_addr, uint8_t bit_start, uint8_t length) {
  this->checkBitsRange<uint16_t>(bit_start, length);
  uint16_t mask = this->generateBitMask<uint16_t>(bit_start, length);
  uint16_t byte_read = this->readWord(reg_addr);
  return (byte_read & mask) >> bit_start;
}


void I2Cdev::writeBytes(uint8_t reg_addr, uint8_t length, uint8_t* data) {
  uint8_t *out_buf = new uint8_t[length + 1];
  out_buf[0] = reg_addr;
  memcpy(&out_buf[1], data, length);

  if (write(i2c_bus_fd_, out_buf, length + 1) < 0) {
    delete[] out_buf;
    throwIOSystemError("Failed to write to the I2C bus");
  }

  delete[] out_buf;
}

void I2Cdev::writeWords(uint8_t reg_addr, uint8_t length, uint16_t* data) {
  size_t out_buf_length = (2 * length) + 1;
  uint8_t *out_buf = new uint8_t[out_buf_length];
  out_buf[0] = reg_addr;

  for (size_t i = 1; i <= length; i++) {
    out_buf[i] = data[i] >> 8;
    out_buf[i+1] = data[i];
  }

  if (write(i2c_bus_fd_, out_buf, out_buf_length) < 0) {
    delete[] out_buf;
    throwIOSystemError("Failed to write to the I2C bus");
  }

  delete[] out_buf;
}

void I2Cdev::writeByte(uint8_t reg_addr, uint8_t data) {
  this->writeBytes(reg_addr, 1, &data);
}

void I2Cdev::writeWord(uint8_t reg_addr, uint16_t data) {
  this->writeWords(reg_addr, 1, &data);
}

void I2Cdev::setByteBit(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint8_t>(bit_num);
  uint8_t byte_read = this->readByte(reg_addr);
  return writeByte(reg_addr, byte_read | (1 << bit_num));
}

void I2Cdev::clearByteBit(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint8_t>(bit_num);
  uint8_t byte_read = this->readByte(reg_addr);
  return writeByte(reg_addr, byte_read & ~(1 << bit_num));
}

void I2Cdev::setWordBit(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint16_t>(bit_num);
  uint16_t word_read = this->readWord(reg_addr);
  return writeWord(reg_addr, word_read | (1 << bit_num));
}

void I2Cdev::clearWordBit(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint16_t>(bit_num);
  uint16_t word_read = this->readByte(reg_addr);
  return writeByte(reg_addr, word_read & ~(1 << bit_num));
}

void I2Cdev::writeByteBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data) {
  //      110 value to write
  // 76543210 bit numbers
  //    xxx   args: bit_start=4, length=3
  // 01110000 mask byte
  // 10101111 original value (sample)
  // 10001111 original & ~mask
  // 11101011 masked | value
  this->checkBitsRange<uint8_t>(bit_start, length);
  uint8_t byte_read = readByte(reg_addr);
  uint8_t mask = this->generateBitMask<uint8_t>(bit_start, length);

  data <<= bit_start;     // shift data into correct position
  data &= mask;           // zero all non-important bits in data
  byte_read &= ~(mask);   // zero all important bits in existing byte
  byte_read |= data;      // combine data with existing byte

  writeByte(reg_addr, byte_read);
}

void I2Cdev::writeWordBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t data) {
  //              110 value to write
  // fedcba9876543210 bit numbers
  //    xxx           args: bit_start=12, length=3
  // 0111000000000000 mask word
  // 1010111110010110 original value (sample)
  // 1000111110010110 original & ~mask
  // 1110101110010110 masked | value
  this->checkBitsRange<uint16_t>(bit_start, length);
  uint16_t word_read = readByte(reg_addr);
  uint16_t mask = this->generateBitMask<uint16_t>(bit_start, length);

  data <<= bit_start;     // shift data into correct position
  data &= mask;           // zero all non-important bits in data
  word_read &= ~(mask);   // zero all important bits in existing byte
  word_read |= data;      // combine data with existing byte

  writeWord(reg_addr, word_read);
}

void I2Cdev::throwIOSystemError(const std::string& msg) {
  std::stringstream error_msg;
  error_msg << msg << ": " << std::strerror(errno);
  throw std::runtime_error(error_msg.str());
}

I2Cdev::~I2Cdev() {
  close(i2c_bus_fd_);
}
