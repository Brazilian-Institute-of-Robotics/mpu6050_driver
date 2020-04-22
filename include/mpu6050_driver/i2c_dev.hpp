// Copyright (c) 2013 Jeff Rowberg
// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 2013-06-05 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2020-04-XX - remove Arduino stuffs, refactor the code and able library usage in Raspberry boards
//      2015-10-30 - simondlevy : support i2c_t3 for Teensy3.1
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

#ifndef MPU6050_DRIVER_I2C_DEV_HPP_
#define MPU6050_DRIVER_I2C_DEV_HPP_

#include <cstdint>

// 1000ms default read timeout (modify with "I2Cdev::read_timeout = [ms];")
#define I2CDEV_DEFAULT_READ_TIMEOUT     1000

class I2Cdev {
 public:
  /**
    * @brief Construct a new I2Cdev object
    * 
    */
  I2Cdev();

  /** 
   * @brief Read a single bit from an 8-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register regAddr to read from
   * @param bit_num Bit position to read (0-7)
   * @param data Container for single bit value
   * @param timeout Optional read timeout in milliseconds (0 to disable, leave 
   * off to use default class value in I2Cdev::readTimeout)
   * @return Status of read operation (true = success)
   */
  static int8_t readBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num,
                        uint8_t *data, uint16_t timeout = I2Cdev::read_timeout);

  /** 
   * @brief Read a single bit from a 16-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register reg_addr to read from
   * @param bit_num Bit position to read (0-15)
   * @param data Container for single bit value
   * @param timeout Optional read timeout in milliseconds (0 to disable, leave
   * off to use default class value in I2Cdev::readTimeout)
   * @return Status of read operation (true = success)
   */
  static int8_t readBitW(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num,
                         uint16_t *data, uint16_t timeout = I2Cdev::read_timeout);

  /** 
   * @brief Read multiple bits from an 8-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register reg_addr to read from
   * @param bit_start First bit position to read (0-7)
   * @param length Number of bits to read (not more than 8)
   * @param data Container for right-aligned value (i.e. '101' read from any bit_start position will equal 0x05)
   * @param timeout Optional read timeout in milliseconds (0 to disable, leave 
   * off to use default class value in I2Cdev::readTimeout)
   * @return Status of read operation (true = success)
   */
  static int8_t readBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length,
                         uint8_t *data, uint16_t timeout = I2Cdev::read_timeout);

  /** 
   * @brief Read multiple bits from a 16-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register reg_addr to read from
   * @param bit_start First bit position to read (0-15)
   * @param length Number of bits to read (not more than 16)
   * @param data Container for right-aligned value (i.e. '101' read from any bit_start position will equal 0x05)
   * @param timeout Optional read timeout in milliseconds (0 to disable, leave 
   * off to use default class value in I2Cdev::readTimeout)
   * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
   */
  static int8_t readBitsW(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length,
                          uint16_t *data, uint16_t timeout = I2Cdev::read_timeout);

  /** 
   * @brief Read single byte from an 8-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register reg_addr to read from
   * @param data Container for byte value read from device
   * @param timeout Optional read timeout in milliseconds (0 to disable, leave 
   * off to use default class value in I2Cdev::readTimeout)
   * @return Status of read operation (true = success)
   */
  static int8_t readByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t timeout = I2Cdev::read_timeout);

  /** 
   * @brief Read single word from a 16-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register reg_addr to read from
   * @param data Container for word value read from device
   * @param timeout Optional read timeout in milliseconds (0 to disable, leave
   * off to use default class value in I2Cdev::readTimeout)
   * @return Status of read operation (true = success)
   */
  static int8_t readWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t *data, uint16_t timeout = I2Cdev::read_timeout);

  /** 
   * @brief Read multiple bytes from an 8-bit device register.
   * @param dev_addr I2C slave device address
   * @param reg_addr First register reg_addr to read from
   * @param length Number of bytes to read
   * @param data Buffer to store read data in
   * @param timeout Optional read timeout in milliseconds (0 to disable, leave off
   * to use default class value in I2Cdev::readTimeout)
   * @return Number of bytes read (-1 indicates failure)
   */
  static int8_t readBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length,
                          uint8_t *data, uint16_t timeout = I2Cdev::read_timeout);

  /** 
   * @brief Read multiple words from a 16-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr First register reg_addr to read from
   * @param length Number of words to read
   * @param data Buffer to store read data in
   * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
   * @return Number of words read (-1 indicates failure)
   */
  static int8_t readWords(uint8_t dev_addr, uint8_t reg_addr, uint8_t length,
                          uint16_t *data, uint16_t timeout = I2Cdev::read_timeout);

  /** 
   * @brief Write a single bit in an 8-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register reg_addr to write to
   * @param bit_num Bit position to write (0-7)
   * @param value New bit value to write
   * @return Status of operation (true = success)
   */
  static bool writeBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t data);

  /** 
   * @brief Write a single bit in a 16-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register reg_addr to write to
   * @param bit_num Bit position to write (0-15)
   * @param value New bit value to write
   * @return Status of operation (true = success)
   */
  static bool writeBitW(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint16_t data);

  /** 
   * @brief Write multiple bits in an 8-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register reg_addr to write to
   * @param bit_start First bit position to write (0-7)
   * @param length Number of bits to write (not more than 8)
   * @param data Right-aligned value to write
   * @return Status of operation (true = success)
   */
  static bool writeBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);

  /** 
   * @brief Write multiple bits in a 16-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register reg_addr to write to
   * @param bit_start First bit position to write (0-15)
   * @param length Number of bits to write (not more than 16)
   * @param data Right-aligned value to write
   * @return Status of operation (true = success)
   */
  static bool writeBitsW(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t data);

  /** 
   * @brief Write single byte to an 8-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register address to write to
   * @param data New byte value to write
   * @return Status of operation (true = success)
   */
  static bool writeByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

  /** 
   * @brief Write single word to a 16-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr Register address to write to
   * @param data New word value to write
   * @return Status of operation (true = success)
   */
  static bool writeWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t data);

  /** 
   * @brief Write multiple bytes to an 8-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr First register address to write to
   * @param length Number of bytes to write
   * @param data Buffer to copy new data from
   * @return Status of operation (true = success)
   */
  static bool writeBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);

  /** 
   * @brief Write multiple words to a 16-bit device register.
   * 
   * @param dev_addr I2C slave device address
   * @param reg_addr First register address to write to
   * @param length Number of words to write
   * @param data Buffer to copy new data from
   * @return Status of operation (true = success)
   */
  static bool writeWords(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint16_t *data);

  static uint16_t read_timeout;
};

#endif  // MPU6050_DRIVER_I2C_DEV_HPP_
