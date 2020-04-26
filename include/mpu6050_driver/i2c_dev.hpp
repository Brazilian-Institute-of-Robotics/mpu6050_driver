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
//                 - made all methods for multi-device memory savings
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
#include <string>
#include <limits>

class I2Cdev {
 public:
  /**
    * @brief Construct a new I2Cdev object
    * 
    * @param dev_addr Address of the I2C device to communicate
    */
  explicit I2Cdev(uint8_t dev_addr);

  /**
   * @brief Destroy the I2Cdev object
   * 
   */
  ~I2Cdev();

  /**
   * @brief Open I2C bus
   * 
   * @param bus_uri The name of the I2C bus uri
   * @throw std::runtime_error Thrown if an error occurs when obtaining a file
   * descriptor for I2C read/write operations
   */
  void openI2CBus(const std::string& bus_uri);

  /** 
   * @brief Read multiple bytes from an 8-bit device register.

   * @param reg_addr First register reg_addr to read from
   * @param length Number of bytes to read
   * @param data Buffer to store read data in
   * @throw std::runtime_error Thrown if read operation fails
   */
  void readBytes(uint8_t reg_addr, uint8_t length, uint8_t *data);

  /** 
   * @brief Read multiple words from a 16-bit device register.
   * 
   * @param reg_addr First register reg_addr to read from
   * @param length Number of words to read
   * @param data Buffer to store read data in
   * @throw std::runtime_error Thrown if read operation fails
   */
  void readWords(uint8_t reg_addr, uint8_t length, uint16_t *dat);

  /** 
   * @brief Read single byte from an 8-bit device register.
   * 
   * @param reg_addr Register reg_addr to read from
   * @throw std::runtime_error Thrown if read operation fails
   */
  uint8_t readByte(uint8_t reg_addr);

  /** 
   * @brief Read single word from a 16-bit device register.
   * 
   * @param reg_addr Register reg_addr to read from
   * @throw std::runtime_error Thrown if read operation fails
   */
  uint16_t readWord(uint8_t reg_addr);

  /** 
   * @brief Read a single bit from an 8-bit device register.
   * 
   * @param reg_addr Register regAddr to read from
   * @param bit_num Bit position to read (0-7)
   * @throw std::runtime_error If bit num is out of range allowed or read operation fails
   */
  uint8_t readBitOfByte(uint8_t reg_addr, uint8_t bit_num);


  /** 
   * @brief Read a single bit from a 16-bit device register.
   * 
   * @param reg_addr Register reg_addr to read from
   * @param bit_num Bit position to read (0-15)
   * @throw std::runtime_error If bit num is out of range allowed or read operation fails
   */
  uint16_t readBitOfWord(uint8_t reg_addr, uint8_t bit_num);

  /** 
   * @brief Read multiple bits from an 8-bit device register.
   * 
   * @param reg_addr Register reg_addr to read from
   * @param bit_start First bit position to read (0-7)
   * @param length Number of bits to read (bit_start + length have to <= number of bits in a byte)
   * @throw std::runtime_error If bits range is invalid or read operation fails
   * @return Right-aligned value (i.e. '101' read from any bit_start position wi1ll equal 0x05)
   */
  uint8_t readBitsOfByte(uint8_t reg_addr, uint8_t bit_start, uint8_t length);

  /** 
   * @brief Read multiple bits from a 16-bit device register.
   * 
   * @param reg_addr Register reg_addr to read from
   * @param bit_start First bit position to read (0-15)
   * @param length Number of bits to read (bit_start + length have to <= number of bits in a byte)
   * @throw std::runtime_error If bits range is invalid or read operation fails1
   * @return Right-aligned value (i.e. '101' read from any bit_start position will equal 0x05)
   */
  uint16_t readBitsOfWord(uint8_t reg_addr, uint8_t bit_start, uint8_t length);

  /** 
   * @brief Write multiple bytes to an 8-bit device register.
   * 
   * @param reg_addr First register address to write to
   * @param length Number of bytes to write
   * @param data Buffer to copy new data from
   * @throw std::runtime_error Thrown if write operation fails
   */
  void writeBytes(uint8_t reg_addr, uint8_t length, uint8_t *data);

  /** 
   * @brief Write multiple words to a 16-bit device register.
   * 
   * @param reg_addr First register address to write to
   * @param length Number of words to write
   * @param data Buffer to copy new data from
   * @throw std::runtime_error Thrown if write operation fails
   */
  void writeWords(uint8_t reg_addr, uint8_t length, uint16_t *data);

  /** 
   * @brief Write single byte to an 8-bit device register.
   * 
   * @param reg_addr Register address to write to
   * @param data New byte value to write
   * @throw std::runtime_error Thrown if write operation fails
   */
  void writeByte(uint8_t reg_addr, uint8_t data);

  /** 
   * @brief Write single word to a 16-bit device register.
   * 
   * @param reg_addr Register address to write to
   * @param data New word value to write
   * @throw std::runtime_error Thrown if write operation fails
   */
  void writeWord(uint8_t reg_addr, uint16_t data);

  /** 
   * @brief Set a single bit in an 8-bit device register.
   * 
   * @param reg_addr Register reg_addr to write to
   * @param bit_num Bit position to set (0-7)
   * @throw std::runtime_error If bit num is out of range allowed or write in register operation fails
   */
  void setByteBit(uint8_t reg_addr, uint8_t bit_num);

  /** 
   * @brief Clear a single bit in an 8-bit device register.
   * 
   * @param reg_addr Register reg_addr to write to
   * @param bit_num Bit position to clear (0-7)
   * @throw std::runtime_error If bit num is out of range allowed or write in register operation fails
   */
  void clearByteBit(uint8_t reg_addr, uint8_t bit_num);

  /** 
   * @brief Set a single bit in a 16-bit device register.
   * 
   * @param reg_addr Register reg_addr to write to
   * @param bit_num Bit position to write (0-15)
   * @throw std::runtime_error If bit num is out of range allowed or write in register operation fails
   */
  void setWordBit(uint8_t reg_addr, uint8_t bit_num);

  /** 
   * @brief Clear a single bit in an 16-bit device register.
   * 
   * @param reg_addr Register reg_addr to write to
   * @param bit_num Bit position to clear (0-7)
   * @throw std::runtime_error If bit num is out of range allowed or write in register operation fails
   */
  void clearWordBit(uint8_t reg_addr, uint8_t bit_num);

  /** 
   * @brief Write multiple bits in an 8-bit device register.
   * 
   * @param reg_addr Register reg_addr to write to
   * @param bit_start First bit position to write (0-7)
   * @param length Number of bits to write (not more than 8)
   * @param data Right-aligned value to write
   * @throw std::runtime_error If bits range is invalid or getting actual register value fails
   * or write the new value in register fails
   */
  void writeByteBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);

  /** 
   * @brief Write multiple bits in a 16-bit device register.
   * 
   * @param reg_addr Register reg_addr to write to
   * @param bit_start First bit position to write (0-15)
   * @param length Number of bits to write (not more than 16)
   * @param data Right-aligned value to write
   * @throw std::runtime_error If bits range is invalid or getting actual register value fails
   * or write the new value in register fails
   */
  void writeWordBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t data);


  uint16_t read_timeout;

 private:
  int i2c_bus_fd_;
  std::string bur_uri_;
  uint8_t dev_addr_;
  void throwIOSystemError(const std::string& msg);

  template <typename T>
  void checkBitNumber(uint8_t bit_number) {
    if (bit_number >= std::numeric_limits<T>::digits) throw std::runtime_error("Bit index invalid");
  }

  template <typename T>
  void checkBitsRange(uint8_t bit_start, uint8_t length) {
    if ((bit_start + length) > std::numeric_limits<T>::digits) throw std::runtime_error("Bits range invalid");
  }

  template <typename T>
  T generateBitMask(uint8_t bit_start, uint8_t length) {
    return ((1 << length) - 1) << bit_start;
  }
};

#endif  // MPU6050_DRIVER_I2C_DEV_HPP_
