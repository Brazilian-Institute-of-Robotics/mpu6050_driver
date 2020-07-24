// Copyright (c) 2012 Jeff Rowberg
// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 2020-05-01 port to raspberry boards by Mateus Menezes <mateusmenezes95@gmail.com>
// 8/24/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//  2019-07-08 - Added Auto Calibration routine
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

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

#include "mpu6050_driver/mpu6050.hpp"

namespace mpu6050_driver {

MPU6050::MPU6050(uint8_t address): mpu_addr_(address) {}

MPU6050::MPU6050() {}

void MPU6050::initialize(const std::string& i2c_bus_uri) {
  mpu_device_.openI2CBus(i2c_bus_uri, mpu_addr_);

  setClockSource(MPU6050_CLOCK_PLL_XGYRO);

  setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  setDLPFMode(MPU6050_DLPF_BW_10);

  accel_lsb_sensitivity_ = this->getAccelLSBSensitivity();
  gyro_lsb_sensitivity_ = this->getGyroLSBSensitivity();

  setSleepEnabled(false);  // thanks to Jack Elston for pointing this one out!
}

void MPU6050::setAddress(uint8_t addr) {
  this->mpu_addr_ = addr;
}

bool MPU6050::testConnection() {
  return getDeviceID() == 0x34;
}

uint8_t MPU6050::getAuxVDDIOLevel() {
  return mpu_device_.readByteBit(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT);
}

void MPU6050::setAuxVDDIOLevel(uint8_t level) {
  mpu_device_.changeByteBitValue(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, level);
}

uint8_t MPU6050::getRate() {
  return mpu_device_.readByte(MPU6050_RA_SMPLRT_DIV);
}

void MPU6050::setRate(uint8_t rate) {
  mpu_device_.writeByte(MPU6050_RA_SMPLRT_DIV, rate);
}

uint8_t MPU6050::getExternalFrameSync() {
  return mpu_device_.readByteBits(MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH);
}

void MPU6050::setExternalFrameSync(uint8_t sync) {
  mpu_device_.setByteBits(MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

uint8_t MPU6050::getDLPFMode() {
  return mpu_device_.readByteBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH);
}

void MPU6050::setDLPFMode(uint8_t mode) {
  mpu_device_.setByteBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

uint8_t MPU6050::getFullScaleGyroRange() {
  return mpu_device_.readByteBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH);
}

float MPU6050::getGyroLSBSensitivity() {
  uint8_t fs_sel_value = this->getFullScaleGyroRange();

  switch (fs_sel_value) {
    case MPU6050_GYRO_FS_250:
      return GYRO_SCALE_MODIFIER_250DEG;
    case MPU6050_GYRO_FS_500:
      return GYRO_SCALE_MODIFIER_500DEG;
    case MPU6050_GYRO_FS_1000:
      return GYRO_SCALE_MODIFIER_1000DEG;
    case MPU6050_GYRO_FS_2000:
      return GYRO_SCALE_MODIFIER_2000DEG;
    default:
      throw("Invalid gyro full scale mode received");
  }
}

void MPU6050::setFullScaleGyroRange(uint8_t range) {
  mpu_device_.setByteBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
  gyro_lsb_sensitivity_ = this->getGyroLSBSensitivity();
}

uint8_t MPU6050::getAccelXSelfTestFactoryTrim() {
  uint8_t buffer[2];
  buffer[0] = mpu_device_.readByte(MPU6050_RA_SELF_TEST_X);
  buffer[1] = mpu_device_.readByte(MPU6050_RA_SELF_TEST_A);
  return (buffer[0] >> 3) | ((buffer[1] >> 4) & 0x03);
}

uint8_t MPU6050::getAccelYSelfTestFactoryTrim() {
  uint8_t buffer[2];
  buffer[0] = mpu_device_.readByte(MPU6050_RA_SELF_TEST_Y);
  buffer[1] = mpu_device_.readByte(MPU6050_RA_SELF_TEST_A);
  return (buffer[0] >> 3) | ((buffer[1] >> 2) & 0x03);
}

uint8_t MPU6050::getAccelZSelfTestFactoryTrim() {
  uint8_t buffer[2];
  mpu_device_.readBytes(MPU6050_RA_SELF_TEST_Z, sizeof(buffer), buffer);
  return (buffer[0] >> 3) | (buffer[1] & 0x03);
}

uint8_t MPU6050::getGyroXSelfTestFactoryTrim() {
  return mpu_device_.readByte(MPU6050_RA_SELF_TEST_X) & 0x1F;
}

uint8_t MPU6050::getGyroYSelfTestFactoryTrim() {
  return mpu_device_.readByte(MPU6050_RA_SELF_TEST_Y) & 0x1F;
}

uint8_t MPU6050::getGyroZSelfTestFactoryTrim() {
  return mpu_device_.readByte(MPU6050_RA_SELF_TEST_Z) & 0x1F;
}

bool MPU6050::getAccelXSelfTest() {
  return mpu_device_.readByteBit(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT);
}

void MPU6050::setAccelXSelfTest(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, enabled);
}

bool MPU6050::getAccelYSelfTest() {
  return mpu_device_.readByteBit(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT);
}

void MPU6050::setAccelYSelfTest(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, enabled);
}

bool MPU6050::getAccelZSelfTest() {
  return mpu_device_.readByteBit(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT);
}

void MPU6050::setAccelZSelfTest(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, enabled);
}

uint8_t MPU6050::getFullScaleAccelRange() {
  return mpu_device_.readByteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH);
}

float MPU6050::getAccelLSBSensitivity() {
  uint8_t afs_sel_value = this->getFullScaleAccelRange();

  switch (afs_sel_value) {
    case MPU6050_ACCEL_FS_2:
      return ACCEL_SCALE_MODIFIER_2G;
    case MPU6050_ACCEL_FS_4:
      return ACCEL_SCALE_MODIFIER_4G;
    case MPU6050_ACCEL_FS_8:
      return ACCEL_SCALE_MODIFIER_8G;
    case MPU6050_ACCEL_FS_16:
      return ACCEL_SCALE_MODIFIER_16G;
    default:
      throw("Invalid accelerometer full scale mode received");
  }
}

void MPU6050::setFullScaleAccelRange(uint8_t range) {
  mpu_device_.setByteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
  accel_lsb_sensitivity_ = this->getFullScaleAccelRange();
}

uint8_t MPU6050::getDHPFMode() {
  return mpu_device_.readByteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT,
                                  MPU6050_ACONFIG_ACCEL_HPF_LENGTH);
}

void MPU6050::setDHPFMode(uint8_t mode) {
  mpu_device_.setByteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT,
                  MPU6050_ACONFIG_ACCEL_HPF_LENGTH, mode);
}

uint8_t MPU6050::getFreefallDetectionThreshold() {
  return mpu_device_.readByte(MPU6050_RA_FF_THR);
}

void MPU6050::setFreefallDetectionThreshold(uint8_t threshold) {
  mpu_device_.writeByte(MPU6050_RA_FF_THR, threshold);
}

uint8_t MPU6050::getFreefallDetectionDuration() {
  return mpu_device_.readByte(MPU6050_RA_FF_DUR);
}

void MPU6050::setFreefallDetectionDuration(uint8_t duration) {
  mpu_device_.writeByte(MPU6050_RA_FF_DUR, duration);
}

uint8_t MPU6050::getMotionDetectionThreshold() {
  return mpu_device_.readByte(MPU6050_RA_MOT_THR);
}

void MPU6050::setMotionDetectionThreshold(uint8_t threshold) {
  mpu_device_.writeByte(MPU6050_RA_MOT_THR, threshold);
}

uint8_t MPU6050::getMotionDetectionDuration() {
  return mpu_device_.readByte(MPU6050_RA_MOT_DUR);
}

void MPU6050::setMotionDetectionDuration(uint8_t duration) {
  mpu_device_.writeByte(MPU6050_RA_MOT_DUR, duration);
}

uint8_t MPU6050::getZeroMotionDetectionThreshold() {
  return mpu_device_.readByte(MPU6050_RA_ZRMOT_THR);
}

void MPU6050::setZeroMotionDetectionThreshold(uint8_t threshold) {
  mpu_device_.writeByte(MPU6050_RA_ZRMOT_THR, threshold);
}

uint8_t MPU6050::getZeroMotionDetectionDuration() {
  return mpu_device_.readByte(MPU6050_RA_ZRMOT_DUR);
}

void MPU6050::setZeroMotionDetectionDuration(uint8_t duration) {
  mpu_device_.writeByte(MPU6050_RA_ZRMOT_DUR, duration);
}

bool MPU6050::getTempFIFOEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT);
}

void MPU6050::setTempFIFOEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, enabled);
}

bool MPU6050::getXGyroFIFOEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT);
}

void MPU6050::setXGyroFIFOEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, enabled);
}

bool MPU6050::getYGyroFIFOEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT);
}

void MPU6050::setYGyroFIFOEnabled(bool enabled) {
  mpu_device_.setByteBit(MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT);
}

bool MPU6050::getZGyroFIFOEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT);
}

void MPU6050::setZGyroFIFOEnabled(bool enabled) {
  mpu_device_.setByteBit(MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT);
}

bool MPU6050::getAccelFIFOEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT);
}

void MPU6050::setAccelFIFOEnabled(bool enabled) {
  mpu_device_.setByteBit(MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT);
}

bool MPU6050::getSlave2FIFOEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT);
}

void MPU6050::setSlave2FIFOEnabled(bool enabled) {
  mpu_device_.setByteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT);
}

bool MPU6050::getSlave1FIFOEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT);
}

void MPU6050::setSlave1FIFOEnabled(bool enabled) {
  mpu_device_.setByteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT);
}

bool MPU6050::getSlave0FIFOEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT);
}

void MPU6050::setSlave0FIFOEnabled(bool enabled) {
  mpu_device_.setByteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT);
}

bool MPU6050::getMultiMasterEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT);
}

void MPU6050::setMultiMasterEnabled(bool enabled) {
  mpu_device_.setByteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT);
}

bool MPU6050::getWaitForExternalSensorEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT);
}

void MPU6050::setWaitForExternalSensorEnabled(bool enabled) {
  mpu_device_.setByteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT);
}

bool MPU6050::getSlave3FIFOEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT);
}

void MPU6050::setSlave3FIFOEnabled(bool enabled) {
  mpu_device_.setByteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT);
}

bool MPU6050::getSlaveReadWriteTransitionEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT);
}

void MPU6050::setSlaveReadWriteTransitionEnabled(bool enabled) {
  mpu_device_.setByteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT);
}

uint8_t MPU6050::getMasterClockSpeed() {
  return mpu_device_.readByteBits(MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_LENGTH);
}

void MPU6050::setMasterClockSpeed(uint8_t speed) {
  mpu_device_.setByteBits(MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_LENGTH, speed);
}

uint8_t MPU6050::getSlaveAddress(uint8_t num) {
  if (num > 3) return 0;
  return mpu_device_.readByte(MPU6050_RA_I2C_SLV0_ADDR + (num * 3));
}

void MPU6050::setSlaveAddress(uint8_t num, uint8_t address) {
  if (num > 3) return;
  mpu_device_.writeByte(MPU6050_RA_I2C_SLV0_ADDR + (num * 3), address);
}

uint8_t MPU6050::getSlaveRegister(uint8_t num) {
  if (num > 3) return 0;
  return mpu_device_.readByte(MPU6050_RA_I2C_SLV0_REG + (num * 3));
}

void MPU6050::setSlaveRegister(uint8_t num, uint8_t reg) {
  if (num > 3) return;
  mpu_device_.writeByte(MPU6050_RA_I2C_SLV0_REG + (num * 3), reg);
}

bool MPU6050::getSlaveEnabled(uint8_t num) {
  if (num > 3) return 0;
  return mpu_device_.readByteBit(MPU6050_RA_I2C_SLV0_CTRL + (num * 3), MPU6050_I2C_SLV_EN_BIT);
}

void MPU6050::setSlaveEnabled(uint8_t num, bool enabled) {
  if (num > 3) return;
  mpu_device_.changeByteBitValue(MPU6050_RA_I2C_SLV0_CTRL + (num * 3), MPU6050_I2C_SLV_EN_BIT, enabled);
}

bool MPU6050::getSlaveWordByteSwap(uint8_t num) {
  if (num > 3) return 0;
  return mpu_device_.readByteBit(MPU6050_RA_I2C_SLV0_CTRL + (num * 3), MPU6050_I2C_SLV_BYTE_SW_BIT);
}

void MPU6050::setSlaveWordByteSwap(uint8_t num, bool enabled) {
  if (num > 3) return;
  mpu_device_.changeByteBitValue(MPU6050_RA_I2C_SLV0_CTRL + (num * 3), MPU6050_I2C_SLV_BYTE_SW_BIT, enabled);
}

bool MPU6050::getSlaveWriteMode(uint8_t num) {
  if (num > 3) return 0;
  return mpu_device_.readByteBit(MPU6050_RA_I2C_SLV0_CTRL + (num * 3), MPU6050_I2C_SLV_REG_DIS_BIT);
}

void MPU6050::setSlaveWriteMode(uint8_t num, bool mode) {
  if (num > 3) return;
  mpu_device_.changeByteBitValue(MPU6050_RA_I2C_SLV0_CTRL + (num * 3), MPU6050_I2C_SLV_REG_DIS_BIT, mode);
}

bool MPU6050::getSlaveWordGroupOffset(uint8_t num) {
  if (num > 3) return 0;
  return mpu_device_.readByteBit(MPU6050_RA_I2C_SLV0_CTRL + (num * 3), MPU6050_I2C_SLV_GRP_BIT);
}

void MPU6050::setSlaveWordGroupOffset(uint8_t num, bool enabled) {
  if (num > 3) return;
  mpu_device_.changeByteBitValue(MPU6050_RA_I2C_SLV0_CTRL + (num * 3), MPU6050_I2C_SLV_GRP_BIT, enabled);
}

uint8_t MPU6050::getSlaveDataLength(uint8_t num) {
  if (num > 3) return 0;
  return mpu_device_.readByteBits(MPU6050_RA_I2C_SLV0_CTRL + (num * 3), MPU6050_I2C_SLV_LEN_BIT,
                                  MPU6050_I2C_SLV_LEN_LENGTH);
}

void MPU6050::setSlaveDataLength(uint8_t num, uint8_t length) {
  if (num > 3) return;
  mpu_device_.setByteBits(MPU6050_RA_I2C_SLV0_CTRL + (num * 3), MPU6050_I2C_SLV_LEN_BIT,
                          MPU6050_I2C_SLV_LEN_LENGTH, length);
}

uint8_t MPU6050::getSlave4Address() {
  return mpu_device_.readByte(MPU6050_RA_I2C_SLV4_ADDR);
}

void MPU6050::setSlave4Address(uint8_t address) {
  mpu_device_.writeByte(MPU6050_RA_I2C_SLV4_ADDR, address);
}

uint8_t MPU6050::getSlave4Register() {
  return mpu_device_.readByte(MPU6050_RA_I2C_SLV4_REG);
}

void MPU6050::setSlave4Register(uint8_t reg) {
  mpu_device_.writeByte(MPU6050_RA_I2C_SLV4_REG, reg);
}

void MPU6050::setSlave4OutputByte(uint8_t data) {
  mpu_device_.writeByte(MPU6050_RA_I2C_SLV4_DO, data);
}

bool MPU6050::getSlave4Enabled() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT);
}

void MPU6050::setSlave4Enabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT, enabled);
}

bool MPU6050::getSlave4InterruptEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT);
}

void MPU6050::setSlave4InterruptEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, enabled);
}

bool MPU6050::getSlave4WriteMode() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_REG_DIS_BIT);
}

void MPU6050::setSlave4WriteMode(bool mode) {
  mpu_device_.changeByteBitValue(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_REG_DIS_BIT, mode);
}

uint8_t MPU6050::getSlave4MasterDelay() {
  return mpu_device_.readByteBits(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT,
                                  MPU6050_I2C_SLV4_MST_DLY_LENGTH);
}

void MPU6050::setSlave4MasterDelay(uint8_t delay) {
  mpu_device_.setByteBits(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT,
                          MPU6050_I2C_SLV4_MST_DLY_LENGTH, delay);
}

uint8_t MPU6050::getSlate4InputByte() {
  return mpu_device_.readByte(MPU6050_RA_I2C_SLV4_DI);
}

bool MPU6050::getPassthroughStatus() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_PASS_THROUGH_BIT);
}

bool MPU6050::getSlave4IsDone() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_DONE_BIT);
}

bool MPU6050::getLostArbitration() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_LOST_ARB_BIT);
}

bool MPU6050::getSlave4Nack() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_NACK_BIT);
}

bool MPU6050::getSlave3Nack() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV3_NACK_BIT);
}

bool MPU6050::getSlave2Nack() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV2_NACK_BIT);
}

bool MPU6050::getSlave1Nack() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV1_NACK_BIT);
}

bool MPU6050::getSlave0Nack() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV0_NACK_BIT);
}

bool MPU6050::getInterruptMode() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT);
}

void MPU6050::setInterruptMode(bool mode) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, mode);
}

bool MPU6050::getInterruptDrive() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT);
}

void MPU6050::setInterruptDrive(bool drive) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, drive);
}

bool MPU6050::getInterruptLatch() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT);
}

void MPU6050::setInterruptLatch(bool latch) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, latch);
}

bool MPU6050::getInterruptLatchClear() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT);
}

void MPU6050::setInterruptLatchClear(bool clear) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}

bool MPU6050::getFSyncInterruptLevel() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT);
}

void MPU6050::setFSyncInterruptLevel(bool level) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, level);
}

bool MPU6050::getFSyncInterruptEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT);
}

void MPU6050::setFSyncInterruptEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, enabled);
}

bool MPU6050::getI2CBypassEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT);
}

void MPU6050::setI2CBypassEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

bool MPU6050::getClockOutputEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT);
}

void MPU6050::setClockOutputEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, enabled);
}

uint8_t MPU6050::getIntEnabled() {
  return mpu_device_.readByte(MPU6050_RA_INT_ENABLE);
}

void MPU6050::setIntEnabled(uint8_t enabled) {
  mpu_device_.writeByte(MPU6050_RA_INT_ENABLE, enabled);
}

bool MPU6050::getIntFreefallEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT);
}

void MPU6050::setIntFreefallEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT, enabled);
}

bool MPU6050::getIntMotionEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT);
}

void MPU6050::setIntMotionEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, enabled);
}

bool MPU6050::getIntZeroMotionEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT);
}

void MPU6050::setIntZeroMotionEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, enabled);
}

bool MPU6050::getIntFIFOBufferOverflowEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT);
}

void MPU6050::setIntFIFOBufferOverflowEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}

bool MPU6050::getIntI2CMasterEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT);
}

void MPU6050::setIntI2CMasterEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, enabled);
}

bool MPU6050::getIntDataReadyEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT);
}

void MPU6050::setIntDataReadyEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, enabled);
}

uint8_t MPU6050::getIntStatus() {
  return mpu_device_.readByte(MPU6050_RA_INT_STATUS);
}

bool MPU6050::getIntFreefallStatus() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_FF_BIT);
}

bool MPU6050::getIntMotionStatus() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_MOT_BIT);
}

bool MPU6050::getIntZeroMotionStatus() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_ZMOT_BIT);
}

bool MPU6050::getIntFIFOBufferOverflowStatus() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_FIFO_OFLOW_BIT);
}

bool MPU6050::getIntI2CMasterStatus() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_I2C_MST_INT_BIT);
}

bool MPU6050::getIntDataReadyStatus() {
  return mpu_device_.readByteBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT);
}

// TODO(somebody): magnetometer integration
// void MPU6050::getMotion9(float* ax, float* ay, float* az,
//                          float* gx, float* gy, float* gz,
//                          float* mx, float* my, float* mz) {
//   getMotion6(ax, ay, az, gx, gy, gz);
// }

IMUData<int16_t> MPU6050::getRawMotion6() {
  uint16_t buffer[7];
  IMUData<int16_t> raw_imu_data;

  mpu_device_.readWords(MPU6050_RA_ACCEL_XOUT_H, 7, buffer);

  raw_imu_data.accel.x = buffer[0];
  raw_imu_data.accel.y = buffer[1];
  raw_imu_data.accel.z = buffer[2];

  raw_imu_data.gyro.x = buffer[4];
  raw_imu_data.gyro.y = buffer[5];
  raw_imu_data.gyro.z = buffer[6];

  return raw_imu_data;
}

IMUData<float> MPU6050::getMotion6() {
  IMUData<float> imu_data;
  IMUData<int16_t> raw_data;

  raw_data = this->getRawMotion6();

  imu_data.accel.x = static_cast<float>(raw_data.accel.x) / accel_lsb_sensitivity_;
  imu_data.accel.y = static_cast<float>(raw_data.accel.y) / accel_lsb_sensitivity_;
  imu_data.accel.z = static_cast<float>(raw_data.accel.z) / accel_lsb_sensitivity_;

  imu_data.gyro.x = static_cast<float>(raw_data.gyro.x) /  gyro_lsb_sensitivity_;
  imu_data.gyro.y = static_cast<float>(raw_data.gyro.y) /  gyro_lsb_sensitivity_;
  imu_data.gyro.z = static_cast<float>(raw_data.gyro.z) /  gyro_lsb_sensitivity_;

  return imu_data;
}

AccelData<int16_t> MPU6050::getRawAcceleration() {
  uint16_t buffer[3];
  AccelData<int16_t> raw_accel;

  mpu_device_.readWords(MPU6050_RA_ACCEL_XOUT_H, 3, buffer);

  raw_accel.x = buffer[0];
  raw_accel.y = buffer[1];
  raw_accel.z = buffer[2];

  return raw_accel;
}

AccelData<float> MPU6050::getAcceleration() {
  AccelData<float> accel_data;
  AccelData<int16_t> raw_accel;

  raw_accel = this->getRawAcceleration();

  accel_data.x = static_cast<float>(raw_accel.x) / accel_lsb_sensitivity_;
  accel_data.y = static_cast<float>(raw_accel.y) / accel_lsb_sensitivity_;
  accel_data.z = static_cast<float>(raw_accel.z) / accel_lsb_sensitivity_;

  return accel_data;
}

float MPU6050:: getAccelerationX() {
  return mpu_device_.readWord(MPU6050_RA_ACCEL_XOUT_H) / accel_lsb_sensitivity_;
}

float MPU6050::getAccelerationY() {
  return mpu_device_.readWord(MPU6050_RA_ACCEL_YOUT_H) / accel_lsb_sensitivity_;
}

float MPU6050::getAccelerationZ() {
  return mpu_device_.readWord(MPU6050_RA_ACCEL_ZOUT_H) / accel_lsb_sensitivity_;
}

float MPU6050::getTemperature() {
  int16_t raw_temp = (int16_t)mpu_device_.readWord(MPU6050_RA_TEMP_OUT_H);
  return (static_cast<float>(raw_temp) / 340.0) + 36.5;
}

GyroData<int16_t> MPU6050::getRawRotation() {
  uint16_t buffer[3];
  GyroData<int16_t> raw_gyro_data;

  mpu_device_.readWords(MPU6050_RA_GYRO_XOUT_H, 3, buffer);

  raw_gyro_data.x = buffer[0];
  raw_gyro_data.y = buffer[1];
  raw_gyro_data.z = buffer[2];

  return raw_gyro_data;
}

GyroData<float> MPU6050::getRotation() {
  GyroData<float> gyro_data;
  GyroData<int16_t> raw_gyro_data;

  raw_gyro_data = this->getRawRotation();

  gyro_data.x = static_cast<float>(raw_gyro_data.x) / gyro_lsb_sensitivity_;
  gyro_data.y = static_cast<float>(raw_gyro_data.y) / gyro_lsb_sensitivity_;
  gyro_data.z = static_cast<float>(raw_gyro_data.z) / gyro_lsb_sensitivity_;

  return gyro_data;
}

float MPU6050::getRotationX() {
  return mpu_device_.readWord(MPU6050_RA_GYRO_XOUT_H) / gyro_lsb_sensitivity_;
}

float MPU6050::getRotationY() {
  return mpu_device_.readWord(MPU6050_RA_GYRO_YOUT_H) / gyro_lsb_sensitivity_;
}

float MPU6050::getRotationZ() {
  return mpu_device_.readWord(MPU6050_RA_GYRO_ZOUT_H) / gyro_lsb_sensitivity_;
}

uint8_t MPU6050::getExternalSensorByte(int position) {
  return mpu_device_.readByte(MPU6050_RA_EXT_SENS_DATA_00 + position);
}

uint16_t MPU6050::getExternalSensorWord(int position) {
  return mpu_device_.readWord(MPU6050_RA_EXT_SENS_DATA_00);
}

uint32_t MPU6050::getExternalSensorDWord(int position) {
  uint8_t buffer[4];
  mpu_device_.readBytes(MPU6050_RA_EXT_SENS_DATA_00 + position, 4, buffer);
  return (((uint32_t)buffer[0]) << 24) | (((uint32_t)buffer[1]) << 16) | (((uint16_t)buffer[2]) << 8) | buffer[3];
}

uint8_t MPU6050::getMotionStatus() {
  return mpu_device_.readByte(MPU6050_RA_MOT_DETECT_STATUS);
}

bool MPU6050::getXNegMotionDetected() {
  return mpu_device_.readByteBit(MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XNEG_BIT);
}

bool MPU6050::getXPosMotionDetected() {
  return mpu_device_.readByteBit(MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XPOS_BIT);
}

bool MPU6050::getYNegMotionDetected() {
  return mpu_device_.readByteBit(MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YNEG_BIT);
}

bool MPU6050::getYPosMotionDetected() {
  return mpu_device_.readByteBit(MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YPOS_BIT);
}

bool MPU6050::getZNegMotionDetected() {
  return mpu_device_.readByteBit(MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZNEG_BIT);
}

bool MPU6050::getZPosMotionDetected() {
  return mpu_device_.readByteBit(MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZPOS_BIT);
}

bool MPU6050::getZeroMotionDetected() {
  return mpu_device_.readByteBit(MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZRMOT_BIT);
}

void MPU6050::setSlaveOutputByte(uint8_t num, uint8_t data) {
  if (num > 3) return;
  mpu_device_.writeByte(MPU6050_RA_I2C_SLV0_DO + num, data);
}

bool MPU6050::getExternalShadowDelayEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_DELAY_CTRL, MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT);
}

void MPU6050::setExternalShadowDelayEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_I2C_MST_DELAY_CTRL, MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}

bool MPU6050::getSlaveDelayEnabled(uint8_t num) {
  // MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT is 4, SLV3 is 3, etc.
  if (num > 4) return 0;
  return mpu_device_.readByteBit(MPU6050_RA_I2C_MST_DELAY_CTRL, num);
}

void MPU6050::setSlaveDelayEnabled(uint8_t num, bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_I2C_MST_DELAY_CTRL, num, enabled);
}

void MPU6050::resetGyroscopePath() {
  mpu_device_.setByteBit(MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_GYRO_RESET_BIT);
}

void MPU6050::resetAccelerometerPath() {
  mpu_device_.setByteBit(MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_ACCEL_RESET_BIT);
}

void MPU6050::resetTemperaturePath() {
  mpu_device_.setByteBit(MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_TEMP_RESET_BIT);
}

uint8_t MPU6050::getAccelerometerPowerOnDelay() {
  return mpu_device_.readByteBits(MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_ON_DELAY_BIT,
                          MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH);
}

void MPU6050::setAccelerometerPowerOnDelay(uint8_t delay) {
  mpu_device_.setByteBits(MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_ON_DELAY_BIT,
                  MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}

uint8_t MPU6050::getFreefallDetectionCounterDecrement() {
  return mpu_device_.readByteBits(MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT,
                                  MPU6050_DETECT_FF_COUNT_LENGTH);
}

void MPU6050::setFreefallDetectionCounterDecrement(uint8_t decrement) {
  mpu_device_.setByteBits(MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT,
                          MPU6050_DETECT_FF_COUNT_LENGTH, decrement);
}

uint8_t MPU6050::getMotionDetectionCounterDecrement() {
  return mpu_device_.readByteBits(MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT,
                                  MPU6050_DETECT_MOT_COUNT_LENGTH);
}

void MPU6050::setMotionDetectionCounterDecrement(uint8_t decrement) {
  mpu_device_.setByteBits(MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT,
                          MPU6050_DETECT_MOT_COUNT_LENGTH, decrement);
}

bool MPU6050::getFIFOEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT);
}

void MPU6050::setFIFOEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

bool MPU6050::getI2CMasterModeEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);
}

void MPU6050::setI2CMasterModeEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU6050::switchSPIEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_IF_DIS_BIT, enabled);
}

void MPU6050::resetFIFO() {
  mpu_device_.setByteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT);
}

void MPU6050::resetI2CMaster() {
  mpu_device_.setByteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT);
}

void MPU6050::resetSensors() {
  mpu_device_.setByteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_SIG_COND_RESET_BIT);
}

void MPU6050::reset() {
  mpu_device_.setByteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT);
}

bool MPU6050::getSleepEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT);
}

void MPU6050::setSleepEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

bool MPU6050::getWakeCycleEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT);
}

void MPU6050::setWakeCycleEnabled(bool enabled) {
  if (enabled) {
    mpu_device_.setByteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT);
    return;
  }
  mpu_device_.clearByteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT);
}

bool MPU6050::getTempSensorEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT) == 0;  // 1 is actually disabled here
}

void MPU6050::setTempSensorEnabled(bool enabled) {
  if (enabled) {
    mpu_device_.clearByteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT);
    return;
  }
  mpu_device_.setByteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT);
}

uint8_t MPU6050::getClockSource() {
  return mpu_device_.readByteBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH);
}

void MPU6050::setClockSource(uint8_t source) {
  mpu_device_.setByteBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

uint8_t MPU6050::getWakeFrequency() {
  return mpu_device_.readByteBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT,
                                  MPU6050_PWR2_LP_WAKE_CTRL_LENGTH);
}

void MPU6050::setWakeFrequency(uint8_t frequency) {
  mpu_device_.setByteBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT,
                          MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}

bool MPU6050::getStandbyXAccelEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT);
}

void MPU6050::setStandbyXAccelEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, enabled);
}

bool MPU6050::getStandbyYAccelEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT);
}

void MPU6050::setStandbyYAccelEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, enabled);
}

bool MPU6050::getStandbyZAccelEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT);
}

void MPU6050::setStandbyZAccelEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, enabled);
}

bool MPU6050::getStandbyXGyroEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT);
}

void MPU6050::setStandbyXGyroEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, enabled);
}

bool MPU6050::getStandbyYGyroEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT);
}

void MPU6050::setStandbyYGyroEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, enabled);
}

bool MPU6050::getStandbyZGyroEnabled() {
  return mpu_device_.readByteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT);
}

void MPU6050::setStandbyZGyroEnabled(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, enabled);
}

uint16_t MPU6050::getFIFOCount() {
  return mpu_device_.readWord(MPU6050_RA_FIFO_COUNTH);
}

uint8_t MPU6050::getFIFOByte() {
  return mpu_device_.readByte(MPU6050_RA_FIFO_R_W);
}

void MPU6050::getFIFOBytes(uint8_t *data, uint8_t length) {
  if (length > 0) {
    mpu_device_.readBytes(MPU6050_RA_FIFO_R_W, length, data);
    return;
  }
  *data = 0;
}

void MPU6050::setFIFOByte(uint8_t data) {
  mpu_device_.writeByte(MPU6050_RA_FIFO_R_W, data);
}

uint8_t MPU6050::getDeviceID() {
  return mpu_device_.readByteBits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH);
}

void MPU6050::setDeviceID(uint8_t id) {
  mpu_device_.setByteBits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id);
}

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register

uint8_t MPU6050::getOTPBankValid() {
  return mpu_device_.readByteBit(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT);
}

void MPU6050::setOTPBankValid(bool enabled) {
  mpu_device_.changeByteBitValue(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}

int8_t MPU6050::getXGyroOffsetTC() {
  return mpu_device_.readByteBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH);
}

void MPU6050::setXGyroOffsetTC(int8_t offset) {
  mpu_device_.setByteBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// YG_OFFS_TC register

int8_t MPU6050::getYGyroOffsetTC() {
  return mpu_device_.readByteBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH);
}

void MPU6050::setYGyroOffsetTC(int8_t offset) {
  mpu_device_.setByteBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register

int8_t MPU6050::getZGyroOffsetTC() {
  return mpu_device_.readByteBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH);
}

void MPU6050::setZGyroOffsetTC(int8_t offset) {
  mpu_device_.setByteBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// X_FINE_GAIN register

int8_t MPU6050::getXFineGain() {
  return mpu_device_.readByte(MPU6050_RA_X_FINE_GAIN);
}

void MPU6050::setXFineGain(int8_t gain) {
  mpu_device_.writeByte(MPU6050_RA_X_FINE_GAIN, gain);
}

// Y_FINE_GAIN register

int8_t MPU6050::getYFineGain() {
  return mpu_device_.readByte(MPU6050_RA_Y_FINE_GAIN);
}

void MPU6050::setYFineGain(int8_t gain) {
  mpu_device_.writeByte(MPU6050_RA_Y_FINE_GAIN, gain);
}

// Z_FINE_GAIN register

int8_t MPU6050::getZFineGain() {
  return mpu_device_.readByte(MPU6050_RA_Z_FINE_GAIN);
}

void MPU6050::setZFineGain(int8_t gain) {
    mpu_device_.writeByte(MPU6050_RA_Z_FINE_GAIN, gain);
}

// XA_OFFS_* registers

int16_t MPU6050::getXAccelOffset() {
  uint8_t slave_address = getDeviceID() < 0x38 ? MPU6050_RA_XA_OFFS_H : 0x77;  // MPU6050,MPU9150 Vs MPU6500,MPU9250
  return mpu_device_.readWord(slave_address);
}

void MPU6050::setXAccelOffset(int16_t offset) {
  uint8_t slave_address = getDeviceID() < 0x38 ? MPU6050_RA_XA_OFFS_H : 0x77;  // MPU6050,MPU9150 Vs MPU6500,MPU9250
  mpu_device_.writeWord(slave_address, offset);
}

// YA_OFFS_* register

int16_t MPU6050::getYAccelOffset() {
  uint8_t slave_address = getDeviceID() < 0x38 ? MPU6050_RA_YA_OFFS_H : 0x7A;  // MPU6050,MPU9150 Vs MPU6500,MPU9250
  return mpu_device_.readWord(slave_address);
}

void MPU6050::setYAccelOffset(int16_t offset) {
  uint8_t slave_address = getDeviceID() < 0x38 ? MPU6050_RA_YA_OFFS_H : 0x7A;  // MPU6050,MPU9150 Vs MPU6500,MPU9250
  mpu_device_.writeWord(slave_address, offset);
}

// ZA_OFFS_* register

int16_t MPU6050::getZAccelOffset() {
  uint8_t slave_address = getDeviceID() < 0x38 ? MPU6050_RA_ZA_OFFS_H : 0x7D;  // MPU6050,MPU9150 Vs MPU6500,MPU9250
  return mpu_device_.readWord(slave_address);
}

void MPU6050::setZAccelOffset(int16_t offset) {
  uint8_t slave_address = getDeviceID() < 0x38 ? MPU6050_RA_ZA_OFFS_H : 0x7D;  // MPU6050,MPU9150 Vs MPU6500,MPU9250
  mpu_device_.writeWord(slave_address, offset);
}

// XG_OFFS_USR* registers

int16_t MPU6050::getXGyroOffset() {
  return mpu_device_.readWord(MPU6050_RA_XG_OFFS_USRH);
}

void MPU6050::setXGyroOffset(int16_t offset) {
  mpu_device_.writeWord(MPU6050_RA_XG_OFFS_USRH, offset);
}

// YG_OFFS_USR* register

int16_t MPU6050::getYGyroOffset() {
  return mpu_device_.readWord(MPU6050_RA_YG_OFFS_USRH);
}

void MPU6050::setYGyroOffset(int16_t offset) {
  mpu_device_.writeWord(MPU6050_RA_YG_OFFS_USRH, offset);
}

// ZG_OFFS_USR* register

int16_t MPU6050::getZGyroOffset() {
  return mpu_device_.readWord(MPU6050_RA_ZG_OFFS_USRH);
}

void MPU6050::setZGyroOffset(int16_t offset) {
  mpu_device_.writeWord(MPU6050_RA_ZG_OFFS_USRH, offset);
}

IMUData<int16_t> MPU6050::getOffsets() {
  IMUData<int16_t> offsets;

  offsets.accel.x = this->getXAccelOffset();
  offsets.accel.y = this->getYAccelOffset();
  offsets.accel.z = this->getZAccelOffset();
  offsets.gyro.x = this->getXGyroOffset();
  offsets.gyro.y = this->getYGyroOffset();
  offsets.gyro.z = this->getZGyroOffset();

  return offsets;
}

void MPU6050::resetOffsets() {
  this->setXAccelOffset(0);
  this->setYAccelOffset(0);
  this->setZAccelOffset(0);
  this->setXGyroOffset(0);
  this->setYGyroOffset(0);
  this->setZGyroOffset(0);
}

}  // namespace mpu6050_driver
