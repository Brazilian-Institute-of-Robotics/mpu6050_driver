// Copyright (c) 2013 Jeff Rowberg
// Abstracts bit and byte I2C R/W functions into a convenient class
// 2020-05-01 port to raspberry boards by Mateus Menezes
// 2013-06-05 by Jeff Rowberg <jeff@rowberg.net>
/* ============================================
I2CDevice library code is placed under the MIT license
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

#ifndef I2C_DEVICE_HPP_
#define I2C_DEVICE_HPP_

#include <stdexcept>
#include <cstdint>
#include <string>
#include <limits>

class I2CDevice {
 public:
  /**
    * @brief Construct a new I2CDevice object
    * 
    */
  I2CDevice();

  /**
   * @brief Destroy the I2CDevice object
   * 
   */
  ~I2CDevice();

  /**
   * @brief Open I2C bus
   * 
   * @param bus_uri The name of the I2C bus uri
   * @param dev_addr Address of the I2C device to communicate
   * @throw std::runtime_error Thrown if an error occurs when obtaining a file
   * descriptor for I2C read/write operations
   */
  void openI2CBus(const std::string& bus_uri, uint8_t dev_addr);

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
  void readWords(uint8_t reg_addr, uint8_t length, uint16_t *data);

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
  uint8_t readByteBit(uint8_t reg_addr, uint8_t bit_num);


  /** 
   * @brief Read a single bit from a 16-bit device register.
   * 
   * @param reg_addr Register reg_addr to read from
   * @param bit_num Bit position to read (0-15)
   * @throw std::runtime_error If bit num is out of range allowed or read operation fails
   */
  uint16_t readWordBit(uint8_t reg_addr, uint8_t bit_num);

  /** 
   * @brief Read multiple bits from an 8-bit device register.
   * 
   * @param reg_addr Register reg_addr to read from
   * @param bit_start First bit position to read (0-7)
   * @param length Number of bits to read (bit_start + length have to <= number of bits in a byte)
   * @throw std::runtime_error If bits range is invalid or read operation fails
   * @return Right-aligned value (i.e. '101' read from any bit_start position wi1ll equal 0x05)
   */
  uint8_t readByteBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length);

  /** 
   * @brief Read multiple bits from a 16-bit device register.
   * 
   * @param reg_addr Register reg_addr to read from
   * @param bit_start First bit position to read (0-15)
   * @param length Number of bits to read (bit_start + length have to <= number of bits in a byte)
   * @throw std::runtime_error If bits range is invalid or read operation fails1
   * @return Right-aligned value (i.e. '101' read from any bit_start position will equal 0x05)
   */
  uint16_t readWordBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length);

  /** 
   * @brief Write multiple bytes to an 8-bit device register.
   * 
   * @param reg_addr First register address to write to
   * @param length Number of bytes to write
   * @param data Buffer to copy new data from
   * @throw std::runtime_error Thrown if write operation fails
   */
  void writeBytes(uint8_t reg_addr, uint8_t length, const uint8_t *data);

  /** 
   * @brief Write multiple words to a 16-bit device register.
   * 
   * @param reg_addr First register address to write to
   * @param length Number of words to write
   * @param data Buffer to copy new data from
   * @throw std::runtime_error Thrown if write operation fails
   */
  void writeWords(uint8_t reg_addr, uint8_t length, const uint16_t *data);

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
   * @brief Set multiple bits in an 8-bit device register.
   * 
   * @param reg_addr Register reg_addr to write to
   * @param bit_start First bit position to write (0-7)
   * @param length Number of bits to write (not more than 8)
   * @param data Right-aligned value to write
   * @throw std::runtime_error If bits range is invalid or getting actual register value fails
   * or write the new value in register fails
   */
  void setByteBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);

  /** 
   * @brief Set multiple bits in a 16-bit device register.
   * 
   * @param reg_addr Register reg_addr to write to
   * @param bit_start First bit position to write (0-15)
   * @param length Number of bits to write (not more than 16)
   * @param data Right-aligned value to be set
   * @throw std::runtime_error If bits range is invalid or getting actual register value fails
   * or write the new value in register fails
   */
  void setWordBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t data);

  /**
   * @brief Change the value of a byte bit
   * 
   * @param reg_addr Register reg_addr to write to
   * @param bit_num Bit position to clear (0-7)
   * @param bit_value Value to be assigned to the bit 
   * @throw std::runtime_error If bit num is out of range allowed or write register operation fails
   */
  void changeByteBitValue(uint8_t reg_addr, uint8_t bit_num, bool bit_value);

  /**
   * @brief Change the value of a word bit
   * 
   * @param reg_addr Register reg_addr to write to
   * @param bit_num Bit position to clear (0-15)
   * @param bit_value Value to be assigned to the bit
   * @throw std::runtime_error If bit num is out of range allowed or write in register operation fails
   */
  void changeWordBitValue(uint8_t reg_addr, uint8_t bit_num, bool bit_value);

 private:
  /** File descriptor of I2C file descriptor **/
  int i2c_bus_fd_;

  /** Address of the I2C device **/
  uint8_t dev_addr_;

  /**
   * @brief Throws an error when problems in I/O operations
   * 
   * @param msg Error message
   */
  void throwIOSystemError(const std::string& msg);

  /**
   * @brief Checks whether the @p bit_num doesn't exceeds the number of maximum bits allowed
   * 
   * Depending of the type passed to the template, this method will check if the
   * @p bit_num doesn't exceeds the number of bits of the type passed
   * 
   * @tparam T Specify if checking proccess is for one byte or word (uint8_t or uint16_t basically)
   * @param bit_number The number of the bit to check if it is valid
   * @throw std::runtime_error If @p bit_num exceeds the maximum allowed
   */
  template <typename T>
  void checkBitNumber(uint8_t bit_number) {
    if (bit_number >= std::numeric_limits<T>::digits) throw std::runtime_error("Bit index invalid");
  }

  /**
   * @brief Checks whether the last bit doesn't exceeds the number of maximum bits allowed
   * 
   * Depending of the type passed to the template, this method will check if the
   * @p bit_start + @p length doesn't exceeds the number of bits of the type passed 
   * 
   * @tparam T Specify if checking proccess is for one byte or word (uint8_t or uint16_t basically)
   * @param bit_start The firs position of the range that will be checked
   * @param length Quantity of bits after (including) @p bit_start
   * @throw std::runtime_error If @p bit_num + @p length exceeds the maximum allowed
   */
  template <typename T>
  void checkBitsRange(uint8_t bit_start, uint8_t length) {
    if ((bit_start + length) > std::numeric_limits<T>::digits) throw std::runtime_error("Bits range invalid");
  }

  /**
   * @brief Generates a mask of bits set to one
   * 
   * @tparam T Specify whether the mask will be generated for a byte or word (uint8_t or uint16_t basically)
   * @param bit_start The first position of the range that will be set
   * @param length Quantity of bits after (including) @p bit_start will be set
   * @return T Mask generated
   */
  template <typename T>
  T generateBitMaskOfOnes(uint8_t bit_start, uint8_t length) {
    return ((1 << length) - 1) << bit_start;
  }

  /**
   * @brief Set the bits of @p current_data with @p bits_to_be_set  
   * 
   * @tparam T Specify whether the bits is to be set in a byte or word (uint8_t or uint16_t basically)
   * @param bit_start First bit position to write
   * @param length Number of bits to write
   * @param bits_to_be_set Right-aligned value to be set
   * @param current_data Current data in the register
   * @return T The byte/word to be written in the register
   */
  template <typename T>
  T setBits(uint8_t bit_start, uint8_t length, T bits_to_be_set, T current_data) {
    // Example for a byte
    //      110 value to write
    // 76543210 bit numbers
    //    xxx   args: bit_start=4, length=3
    // 01110000 mask byte
    // 10101111 original value (sample)
    // 10001111 original & ~mask
    // 11101011 bits_to_be_set | current_data
    T mask = this->generateBitMaskOfOnes<T>(bit_start, length);
    bits_to_be_set <<= bit_start;     // shift bits_value into correct position
    bits_to_be_set &= mask;           // zero all non-important bits in bits_value
    current_data &= ~(mask);          // zero all important bits in existing byte
    current_data |= bits_to_be_set;   // combine bits_value with existing byte
    return current_data;
  }
};

#endif  // I2C_DEVICE_HPP_
