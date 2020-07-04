// Copyright (c) 2012 Jeff Rowberg
// I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 6.12 implementation
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 2020-05-01 port to raspberry boards by Mateus Menezes <mateusmenezes95@gmail.com>
// 5/20/2013 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
// 2019/7/10 - I incorporated DMP Firmware Version 6.12 Latest as of today with many features and bug fixes.
//           - MPU6050 Registers have not changed just the DMP Image so that full backwards compatibility is present
//           - Run-time calibration routine is enabled which calibrates after no motion state is detected
//           - once no motion state is detected Calibration completes within 0.5 seconds
//           - The Drawback is that the firmware image is larger.
//     ... - ongoing debug release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

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
#include <cmath>
#include <cstring>
#include <stdexcept>
#include "mpu6050_driver/mpu6050_dmp.hpp"

namespace mpu6050_driver {

MPU6050DMP::MPU6050DMP(I2CDevice *const mpu_device) : mpu_device_(mpu_device) {}

/* this is the most basic initialization I can create. with the intent that we
access the register bytes as few times as needed to get the job done. For detailed
descriptins of all registers and there purpose google "MPU-6000/MPU-6050 Register
Map and Descriptions" */
uint8_t MPU6050DMP::initialize() {  // Lets get it over with fast Write everything once and set it up necely
  uint8_t val;
  uint16_t ival;
  // Reset procedure per instructions in the "MPU-6000/MPU-6050 Register Map and Descriptions" page 41
  mpu_device_->setByteBit(0x6B, 0);  // PWR_MGMT_1: reset with 100ms delay
  usleep(1000000);
  mpu_device_->setByteBits(0x6A, 0, 3, 0x07);  // full SIGNAL_PATH_RESET: with another 100ms delay
  usleep(1000000);
  mpu_device_->writeByte(0x6B, 0x01);  // 1000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
  mpu_device_->writeByte(0x38, 0x00);  // 0000 0000 INT_ENABLE: no Interrupt
  mpu_device_->writeByte(0x23, 0x00);  // 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
  mpu_device_->writeByte(0x1C, 0x00);  // 0000 0000 ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
  // ACTL The logic level for int pin is active low. and interrupt status bits are cleared only by reading INT_STATUS
  mpu_device_->writeByte(0x37, 0x80);  // 1000 0000 INT_PIN_CFG
  mpu_device_->writeByte(0x6B, 0x01);  // 0000 0001 PWR_MGMT_1: Clock Source Select PLL_X_gyro
  // Divides the internal sample rate 400Hz ( Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
  mpu_device_->writeByte(0x19, 0x04);  // 0000 0100 SMPLRT_DIV
  // Digital Low Pass Filter (DLPF) Configuration 188HZ  //Im betting this will be the beat
  mpu_device_->writeByte(0x1A, 0x01);  // 0000 0001 CONFIG

  // Loads the DMP image into the MPU6050 Memory // Should Never Fail
  this->writeProgMemoryBlock(dmp_memory, MPU6050_DMP_CODE_SIZE);

  mpu_device_->writeWord(0x70, 0x0400);  // DMP Program Start Address
  mpu_device_->writeByte(0x1B, 0x18);  // 0001 1000 GYRO_CONFIG: 3 = +2000 Deg/sec
  mpu_device_->writeByte(0x6A, 0xC0);  // 1100 1100 USER_CTRL: Enable Fifo and Reset Fifo
  mpu_device_->writeByte(0x38, 0x02);  // 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
  mpu_device_->setByteBit(0x6A, 0x04);  // Reset FIFO one last time just for kicks

  this->setEnabled(false);  // disable DMP for compatibility with the MPU6050 library
  dmp_packet_size_ = 28;
  return 0;
}

bool MPU6050DMP::isPacketAvailable() {
  return mpu_device_->readWord(MPU6050_RA_FIFO_COUNTH) >= dmp_packet_size_;
}

uint8_t MPU6050DMP::getAccel(int32_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0) packet = dmp_packet_buffer_;
  data[0] = (((uint32_t)packet[16] << 8) | packet[17]);
  data[1] = (((uint32_t)packet[18] << 8) | packet[19]);
  data[2] = (((uint32_t)packet[20] << 8) | packet[21]);
  return 0;
}

uint8_t MPU6050DMP::getAccel(int16_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0) packet = dmp_packet_buffer_;
  data[0] = (packet[16] << 8) | packet[17];
  data[1] = (packet[18] << 8) | packet[19];
  data[2] = (packet[20] << 8) | packet[21];
  return 0;
}

uint8_t MPU6050DMP::getAccel(VectorInt16 *v, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0) packet = dmp_packet_buffer_;
  v -> x = (packet[16] << 8) | packet[17];
  v -> y = (packet[18] << 8) | packet[19];
  v -> z = (packet[20] << 8) | packet[21];
  return 0;
}

uint8_t MPU6050DMP::getQuaternion(int32_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0) packet = dmp_packet_buffer_;
  data[0] = (((uint32_t)packet[0] << 24) | ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3]);
  data[1] = (((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7]);
  data[2] = (((uint32_t)packet[8] << 24) | ((uint32_t)packet[9] << 16) | ((uint32_t)packet[10] << 8) | packet[11]);
  data[3] = (((uint32_t)packet[12] << 24) | ((uint32_t)packet[13] << 16) | ((uint32_t)packet[14] << 8) | packet[15]);
  return 0;
}

uint8_t MPU6050DMP::getQuaternion(int16_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0) packet = dmp_packet_buffer_;
  data[0] = ((packet[0] << 8) | packet[1]);
  data[1] = ((packet[4] << 8) | packet[5]);
  data[2] = ((packet[8] << 8) | packet[9]);
  data[3] = ((packet[12] << 8) | packet[13]);
  return 0;
}

uint8_t MPU6050DMP::getQuaternion(Quaternion *q, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  int16_t qI[4];
  uint8_t status = getQuaternion(qI, packet);
  if (status == 0) {
    q->w = static_cast<float>(qI[0]) / 16384.0f;
    q->x = static_cast<float>(qI[1]) / 16384.0f;
    q->y = static_cast<float>(qI[2]) / 16384.0f;
    q->z = static_cast<float>(qI[3]) / 16384.0f;
    return 0;
  }
  return status;  // int16 return value, indicates error if this line is reached
}

uint8_t MPU6050DMP::getGyro(int32_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0) packet = dmp_packet_buffer_;
  data[0] = (((uint32_t)packet[22] << 8) | packet[23]);
  data[1] = (((uint32_t)packet[24] << 8) | packet[25]);
  data[2] = (((uint32_t)packet[26] << 8) | packet[27]);
  return 0;
}

uint8_t MPU6050DMP::getGyro(int16_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0) packet = dmp_packet_buffer_;
  data[0] = (packet[22] << 8) | packet[23];
  data[1] = (packet[24] << 8) | packet[25];
  data[2] = (packet[26] << 8) | packet[27];
  return 0;
}

uint8_t MPU6050DMP::getGyro(VectorInt16 *v, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0) packet = dmp_packet_buffer_;
  v->x = (packet[22] << 8) | packet[23];
  v->y = (packet[24] << 8) | packet[25];
  v->z = (packet[26] << 8) | packet[27];
  return 0;
}

uint8_t MPU6050DMP::getLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
  // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
  v->x = vRaw->x - gravity->x*8192;
  v->y = vRaw->y - gravity->y*8192;
  v->z = vRaw->z - gravity->z*8192;
  return 0;
}

uint8_t MPU6050DMP::getLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
  // rotate measured 3D acceleration vector into original state
  // frame of reference based on orientation quaternion
  memcpy(v, vReal, sizeof(VectorInt16));
  v -> rotate(q);
  return 0;
}

uint8_t MPU6050DMP::getGravity(int16_t *data, const uint8_t* packet) {
  /* +1g corresponds to +8192, sensitivity is 2g. */
  int16_t qI[4];
  uint8_t status = getQuaternion(qI, packet);
  data[0] = ((int32_t)qI[1] * qI[3] - (int32_t)qI[0] * qI[2]) / 16384;
  data[1] = ((int32_t)qI[0] * qI[1] + (int32_t)qI[2] * qI[3]) / 16384;
  data[2] = ((int32_t)qI[0] * qI[0] - (int32_t)qI[1] * qI[1]
          - (int32_t)qI[2] * qI[2] + (int32_t)qI[3] * qI[3]) / (2 * 16384);
  return status;
}

uint8_t MPU6050DMP::getGravity(VectorFloat *v, Quaternion *q) {
  v->x = 2 * (q->x * q->z - q->w * q->y);
  v->y = 2 * (q->w * q->x + q->y * q->z);
  v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
  return 0;
}

uint8_t MPU6050DMP::getEuler(float *data, Quaternion *q) {
  data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1);   // psi
  data[1] = -asin(2 * q->x * q->z + 2 * q->w * q->y);                              // theta
  data[2] = atan2(2 * q->y * q->z - 2 * q->w * q->x, 2 * q->w * q->w + 2 * q->z * q->z - 1);   // phi
  return 0;
}

#ifdef USE_OLD_DMPGETYAWPITCHROLL
uint8_t MPU6050DMP::getYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}
#else
uint8_t MPU6050DMP::getYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
  // yaw: (about Z axis)
  data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
  // pitch: (nose up/down, about Y axis)
  data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
  // roll: (tilt left/right, about X axis)
  data[2] = atan2(gravity -> y , gravity -> z);
  if (gravity -> z < 0) {
    if (data[1] > 0) {
      data[1] = M_PI - data[1];
    } else {
      data[1] = -M_PI - data[1];
    }
  }
  return 0;
}
#endif

uint16_t MPU6050DMP::getFIFOPacketSize() {
  return dmp_packet_size_;
}

// INT_ENABLE register (DMP functions)
bool MPU6050DMP::getIntPLLReadyEnabled() {
  return mpu_device_->readByteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT);
}

void MPU6050DMP::setIntPLLReadyEnabled(bool enabled) {
  mpu_device_->changeByteBitValue(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, enabled);
}

bool MPU6050DMP::getIntEnabled() {
  return mpu_device_->readByteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT);
}

void MPU6050DMP::setIntEnabled(bool enabled) {
  mpu_device_->changeByteBitValue(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, enabled);
}

// DMP_INT_STATUS

bool MPU6050DMP::getInt5Status() {
  return mpu_device_->readByteBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_5_BIT);
}

bool MPU6050DMP::getInt4Status() {
  return mpu_device_->readByteBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_4_BIT);
}

bool MPU6050DMP::getInt3Status() {
  return mpu_device_->readByteBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_3_BIT);
}

bool MPU6050DMP::getInt2Status() {
  return mpu_device_->readByteBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_2_BIT);
}

bool MPU6050DMP::getInt1Status() {
  return mpu_device_->readByteBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_1_BIT);
}

bool MPU6050DMP::getInt0Status() {
  return mpu_device_->readByteBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_0_BIT);
}

// INT_STATUS register (DMP functions)

bool MPU6050DMP::getIntPLLReadyStatus() {
  return mpu_device_->readByteBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_PLL_RDY_INT_BIT);
}

bool MPU6050DMP::getIntStatus() {
  return mpu_device_->readByteBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DMP_INT_BIT);
}

// USER_CTRL register (DMP functions)

bool MPU6050DMP::getEnabled() {
  return mpu_device_->readByteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT);
}

void MPU6050DMP::setEnabled(bool enabled) {
  mpu_device_->changeByteBitValue(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}

void MPU6050DMP::reset() {
  mpu_device_->setByteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT);
}

// BANK_SEL register

void MPU6050DMP::setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
  bank &= 0x1F;
  if (userBank) bank |= 0x20;
  if (prefetchEnabled) bank |= 0x40;
  mpu_device_->writeByte(MPU6050_RA_BANK_SEL, bank);
}

// MEM_START_ADDR register

void MPU6050DMP::setMemoryStartAddress(uint8_t address) {
  mpu_device_->writeByte(MPU6050_RA_MEM_START_ADDR, address);
}

// MEM_R_W register

uint8_t MPU6050DMP::readMemoryByte() {
  return mpu_device_->readByte(MPU6050_RA_MEM_R_W);
}

void MPU6050DMP::writeMemoryByte(uint8_t data) {
  mpu_device_->writeByte(MPU6050_RA_MEM_R_W, data);
}

void MPU6050DMP::readMemoryBlock(uint8_t *data, uint16_t data_size, uint8_t bank, uint8_t address) {
  setMemoryBank(bank);
  setMemoryStartAddress(address);
  uint8_t chunk_size;
  for (uint16_t i = 0; i < data_size;) {
    // determine correct chunk size according to bank position and data size
    chunk_size = MPU6050_DMP_MEMORY_CHUNK_SIZE;

    // make sure we don't go past the data size
    if (i + chunk_size > data_size) chunk_size = data_size - i;

    // make sure this chunk doesn't go past the bank boundary (256 bytes)
    if (chunk_size > 256 - address) chunk_size = 256 - address;

    // read the chunk of data as specified
    mpu_device_->readBytes(MPU6050_RA_MEM_R_W, chunk_size, data + i);

    // increase byte index by [chunk_size]
    i += chunk_size;

    // uint8_t automatically wraps to 0 at 256
    address += chunk_size;

    // if we aren't done, update bank (if necessary) and address
    if (i < data_size) {
        if (address == 0) bank++;
        setMemoryBank(bank);
        setMemoryStartAddress(address);
    }
  }
}

bool MPU6050DMP::writeMemoryBlock(const uint8_t *data, uint16_t data_size, uint8_t bank,
                                  uint8_t address, bool verify) {
  setMemoryBank(bank);
  setMemoryStartAddress(address);

  uint8_t chunk_size;
  uint8_t *verify_buffer_ptr = 0;
  const uint8_t *prog_buffer_ptr = 0;
  uint16_t i;
  uint8_t j;

  if (verify) verify_buffer_ptr = new uint8_t[MPU6050_DMP_MEMORY_CHUNK_SIZE];
  for (i = 0; i < data_size;) {
    // determine correct chunk size according to bank position and data size
    chunk_size = MPU6050_DMP_MEMORY_CHUNK_SIZE;

    // make sure we don't go past the data size
    if (i + chunk_size > data_size) chunk_size = data_size - i;

    // make sure this chunk doesn't go past the bank boundary (256 bytes)
    if (chunk_size > 256 - address) chunk_size = 256 - address;

    // write the chunk of data as specified
    prog_buffer_ptr = data + i;

    mpu_device_->writeBytes(MPU6050_RA_MEM_R_W, chunk_size, prog_buffer_ptr);

    // verify data if needed
    if (verify && verify_buffer_ptr) {
      setMemoryBank(bank);
      setMemoryStartAddress(address);

      mpu_device_->readBytes(MPU6050_RA_MEM_R_W, chunk_size, verify_buffer_ptr);

      if (memcmp(prog_buffer_ptr, verify_buffer_ptr, chunk_size) != 0) {
        delete(verify_buffer_ptr);
        throw std::runtime_error("Chunk of DMP firmware wrote is diferent of the chunk read from MPU memory");
      }
    }

    // increase byte index by [chunk_size]
    i += chunk_size;

    // uint8_t automatically wraps to 0 at 256
    address += chunk_size;

    // if we aren't done, update bank (if necessary) and address
    if (i < data_size) {
      if (address == 0) bank++;
      setMemoryBank(bank);
      setMemoryStartAddress(address);
    }
  }

  if (verify) delete(verify_buffer_ptr);
  return true;
}

bool MPU6050DMP::writeProgMemoryBlock(const uint8_t *data, uint16_t data_size, uint8_t bank,
                                      uint8_t address, bool verify) {
  return writeMemoryBlock(data, data_size, bank, address, verify);
}

bool MPU6050DMP::writeDMPConfigurationSet(const uint8_t *data, uint16_t data_size, bool useProgMem) {
  const uint8_t *prog_buffer_ptr = 0;
  uint8_t success, special;
  uint16_t i, j;

  // config set data is a long string of blocks with the following structure:
  // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
  uint8_t bank, offset, length;
  for (i = 0; i < data_size;) {
    bank = data[i++];
    offset = data[i++];
    length = data[i++];

    // write data or perform special action
    if (length > 0) {
      prog_buffer_ptr = data + i;
      success = writeMemoryBlock(prog_buffer_ptr, length, bank, offset, true);
      i += length;
    } else {
      // special instruction
      // NOTE: this kind of behavior (what and when to do certain things)
      // is totally undocumented. This code is in here based on observed
      // behavior only, and exactly why (or even whether) it has to be here
      // is anybody's guess for now.
      special = data[i++];

      if (special == 0x01) {
        mpu_device_->writeByte(MPU6050_RA_INT_ENABLE, 0x32);  // single operation
        success = true;
      } else {
        // unknown special command
        success = false;
      }
    }

    if (!success) return false;  // uh oh
  }

  return true;
}

bool MPU6050DMP::writeProgDMPConfigurationSet(const uint8_t *data, uint16_t data_size) {
  return writeDMPConfigurationSet(data, data_size, true);
}

// DMP_CFG_1 register
uint8_t MPU6050DMP::getConfig1() {
  return mpu_device_->readByte(MPU6050_RA_DMP_CFG_1);
}

void MPU6050DMP::setConfig1(uint8_t config) {
  mpu_device_->writeByte(MPU6050_RA_DMP_CFG_1, config);
}

// DMP_CFG_2 register
uint8_t MPU6050DMP::getConfig2() {
  return mpu_device_->readByte(MPU6050_RA_DMP_CFG_2);
}

void MPU6050DMP::setConfig2(uint8_t config) {
  mpu_device_->writeByte(MPU6050_RA_DMP_CFG_2, config);
}

}  // namespace mpu6050_driver
