//
// MPU9250.cc
//
// Implementation of the MPU9250 class used for basic control of the MPU9250
// motion sensor.
//

#include "MPU9250.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <hardware/spi.h>

MPU9250::reg_t MPU9250::mpu9250_regs[] = {
    {"MPU9250_SELF_TEST_X_GYRO",   MPU9250_SELF_TEST_X_GYRO},
    {"MPU9250_SELF_TEST_Y_GYRO",   MPU9250_SELF_TEST_Y_GYRO},
    {"MPU9250_SELF_TEST_Z_GYRO",   MPU9250_SELF_TEST_Z_GYRO},
    {"MPU9250_SELF_TEST_X_ACCEL",  MPU9250_SELF_TEST_X_ACCEL},
    {"MPU9250_SELF_TEST_Y_ACCEL",  MPU9250_SELF_TEST_Y_ACCEL},
    {"MPU9250_SELF_TEST_Z_ACCEL",  MPU9250_SELF_TEST_Z_ACCEL},
    {"MPU9250_XG_OFFSET_H",        MPU9250_XG_OFFSET_H},
    {"MPU9250_XG_OFFSET_L",        MPU9250_XG_OFFSET_L},
    {"MPU9250_YG_OFFSET_H",        MPU9250_YG_OFFSET_H},
    {"MPU9250_YG_OFFSET_L",        MPU9250_YG_OFFSET_L},
    {"MPU9250_ZG_OFFSET_H",        MPU9250_ZG_OFFSET_H},
    {"MPU9250_ZG_OFFSET_L",        MPU9250_ZG_OFFSET_L},
    {"MPU9250_SMPLRT_DIV",         MPU9250_SMPLRT_DIV},
    {"MPU9250_CONFIG",             MPU9250_CONFIG},
    {"MPU9250_GYRO_CONFIG",        MPU9250_GYRO_CONFIG},
    {"MPU9250_ACCEL_CONFIG",       MPU9250_ACCEL_CONFIG},
    {"MPU9250_ACCEL_CONFIG2",      MPU9250_ACCEL_CONFIG2},
    {"MPU9250_LP_ACCEL_ODR",       MPU9250_LP_ACCEL_ODR},
    {"MPU9250_WOM_THR",            MPU9250_WOM_THR},
    {"MPU9250_FIFO_EN",            MPU9250_FIFO_EN},
    {"MPU9250_I2C_MST_CTRL",       MPU9250_I2C_MST_CTRL},
    {"MPU9250_I2C_SLV0_ADDR",      MPU9250_I2C_SLV0_ADDR},
    {"MPU9250_I2C_SLV0_REG",       MPU9250_I2C_SLV0_REG},
    {"MPU9250_I2C_SLV0_CTRL",      MPU9250_I2C_SLV0_CTRL},
    {"MPU9250_I2C_SLV1_ADDR",      MPU9250_I2C_SLV1_ADDR},
    {"MPU9250_I2C_SLV1_REG",       MPU9250_I2C_SLV1_REG},
    {"MPU9250_I2C_SLV1_CTRL",      MPU9250_I2C_SLV1_CTRL},
    {"MPU9250_I2C_SLV2_ADDR",      MPU9250_I2C_SLV2_ADDR},
    {"MPU9250_I2C_SLV2_REG",       MPU9250_I2C_SLV2_REG},
    {"MPU9250_I2C_SLV2_CTRL",      MPU9250_I2C_SLV2_CTRL},
    {"MPU9250_I2C_SLV3_ADDR",      MPU9250_I2C_SLV3_ADDR},
    {"MPU9250_I2C_SLV3_REG",       MPU9250_I2C_SLV3_REG},
    {"MPU9250_I2C_SLV3_CTRL",      MPU9250_I2C_SLV3_CTRL},
    {"MPU9250_I2C_SLV4_ADDR",      MPU9250_I2C_SLV4_ADDR},
    {"MPU9250_I2C_SLV4_REG",       MPU9250_I2C_SLV4_REG},
    {"MPU9250_I2C_SLV4_DO",        MPU9250_I2C_SLV4_DO},
    {"MPU9250_I2C_SLV4_CTRL",      MPU9250_I2C_SLV4_CTRL},
    {"MPU9250_I2C_SLV4_DI",        MPU9250_I2C_SLV4_DI},
    {"MPU9250_I2C_MST_STATUS",     MPU9250_I2C_MST_STATUS},
    {"MPU9250_INT_PIN_CFG",        MPU9250_INT_PIN_CFG},
    {"MPU9250_INT_ENABLE",         MPU9250_INT_ENABLE},
    {"MPU9250_INT_STATUS",         MPU9250_INT_STATUS},
    {"MPU9250_ACCEL_XOUT_H",       MPU9250_ACCEL_XOUT_H},
    {"MPU9250_ACCEL_XOUT_L",       MPU9250_ACCEL_XOUT_L},
    {"MPU9250_ACCEL_YOUT_H",       MPU9250_ACCEL_YOUT_H},
    {"MPU9250_ACCEL_YOUT_L",       MPU9250_ACCEL_YOUT_L},
    {"MPU9250_ACCEL_ZOUT_H",       MPU9250_ACCEL_ZOUT_H},
    {"MPU9250_ACCEL_ZOUT_L",       MPU9250_ACCEL_ZOUT_L},
    {"MPU9250_TEMP_OUT_H",         MPU9250_TEMP_OUT_H},
    {"MPU9250_TEMP_OUT_L",         MPU9250_TEMP_OUT_L},
    {"MPU9250_GYRO_XOUT_H",        MPU9250_GYRO_XOUT_H},
    {"MPU9250_GYRO_XOUT_L",        MPU9250_GYRO_XOUT_L},
    {"MPU9250_GYRO_YOUT_H",        MPU9250_GYRO_YOUT_H},
    {"MPU9250_GYRO_YOUT_L",        MPU9250_GYRO_YOUT_L},
    {"MPU9250_GYRO_ZOUT_H",        MPU9250_GYRO_ZOUT_H},
    {"MPU9250_GYRO_ZOUT_L",        MPU9250_GYRO_ZOUT_L},
    {"MPU9250_EXT_SENS_DATA_00",   MPU9250_EXT_SENS_DATA_00},
    {"MPU9250_EXT_SENS_DATA_01",   MPU9250_EXT_SENS_DATA_01},
    {"MPU9250_EXT_SENS_DATA_02",   MPU9250_EXT_SENS_DATA_02},
    {"MPU9250_EXT_SENS_DATA_03",   MPU9250_EXT_SENS_DATA_03},
    {"MPU9250_EXT_SENS_DATA_04",   MPU9250_EXT_SENS_DATA_04},
    {"MPU9250_EXT_SENS_DATA_05",   MPU9250_EXT_SENS_DATA_05},
    {"MPU9250_EXT_SENS_DATA_06",   MPU9250_EXT_SENS_DATA_06},
    {"MPU9250_EXT_SENS_DATA_07",   MPU9250_EXT_SENS_DATA_07},
    {"MPU9250_EXT_SENS_DATA_08",   MPU9250_EXT_SENS_DATA_08},
    {"MPU9250_EXT_SENS_DATA_09",   MPU9250_EXT_SENS_DATA_09},
    {"MPU9250_EXT_SENS_DATA_10",   MPU9250_EXT_SENS_DATA_10},
    {"MPU9250_EXT_SENS_DATA_11",   MPU9250_EXT_SENS_DATA_11},
    {"MPU9250_EXT_SENS_DATA_12",   MPU9250_EXT_SENS_DATA_12},
    {"MPU9250_EXT_SENS_DATA_13",   MPU9250_EXT_SENS_DATA_13},
    {"MPU9250_EXT_SENS_DATA_14",   MPU9250_EXT_SENS_DATA_14},
    {"MPU9250_EXT_SENS_DATA_15",   MPU9250_EXT_SENS_DATA_15},
    {"MPU9250_EXT_SENS_DATA_16",   MPU9250_EXT_SENS_DATA_16},
    {"MPU9250_EXT_SENS_DATA_17",   MPU9250_EXT_SENS_DATA_17},
    {"MPU9250_EXT_SENS_DATA_18",   MPU9250_EXT_SENS_DATA_18},
    {"MPU9250_EXT_SENS_DATA_19",   MPU9250_EXT_SENS_DATA_19},
    {"MPU9250_EXT_SENS_DATA_20",   MPU9250_EXT_SENS_DATA_20},
    {"MPU9250_EXT_SENS_DATA_21",   MPU9250_EXT_SENS_DATA_21},
    {"MPU9250_EXT_SENS_DATA_22",   MPU9250_EXT_SENS_DATA_22},
    {"MPU9250_EXT_SENS_DATA_23",   MPU9250_EXT_SENS_DATA_23},
    {"MPU9250_I2C_SLV0_DO",        MPU9250_I2C_SLV0_DO},
    {"MPU9250_I2C_SLV1_DO",        MPU9250_I2C_SLV1_DO},
    {"MPU9250_I2C_SLV2_DO",        MPU9250_I2C_SLV2_DO},
    {"MPU9250_I2C_SLV3_DO",        MPU9250_I2C_SLV3_DO},
    {"MPU9250_I2C_MST_DELAY_CTRL", MPU9250_I2C_MST_DELAY_CTRL},
    {"MPU9250_SIGNAL_PATH_RESET",  MPU9250_SIGNAL_PATH_RESET},
    {"MPU9250_MOT_DETECT_CTRL",    MPU9250_MOT_DETECT_CTRL},
    {"MPU9250_USER_CTRL",          MPU9250_USER_CTRL},
    {"MPU9250_PWR_MGMT_1",         MPU9250_PWR_MGMT_1},
    {"MPU9250_PWR_MGMT_2",         MPU9250_PWR_MGMT_2},
    {"MPU9250_FIFO_COUNTH",        MPU9250_FIFO_COUNTH},
    {"MPU9250_FIFO_COUNTL",        MPU9250_FIFO_COUNTL},
    {"MPU9250_FIFO_R_W",           MPU9250_FIFO_R_W},
    {"MPU9250_WHO_AM_I",           MPU9250_WHO_AM_I},
    {"MPU9250_XA_OFFSET_H",        MPU9250_XA_OFFSET_H},
    {"MPU9250_XA_OFFSET_L",        MPU9250_XA_OFFSET_L},
    {"MPU9250_YA_OFFSET_H",        MPU9250_YA_OFFSET_H},
    {"MPU9250_YA_OFFSET_L",        MPU9250_YA_OFFSET_L},
    {"MPU9250_ZA_OFFSET_H",        MPU9250_ZA_OFFSET_H},
    {"MPU9250_ZA_OFFSET_L",        MPU9250_ZA_OFFSET_L},
    {nullptr,                      0}
};

MPU9250::reg_t MPU9250::ak8963_regs[] = {
    {"AK8963_WHO_AM_I", AK8963_WHO_AM_I},
    {"AK8963_INFO",     AK8963_INFO},
    {"AK8963_ST1",      AK8963_ST1},
    {"AK8963_HXL",      AK8963_HXL},
    {"AK8963_HXH",      AK8963_HXH},
    {"AK8963_HYL",      AK8963_HYL},
    {"AK8963_HYH",      AK8963_HYH},
    {"AK8963_HZL",      AK8963_HZL},
    {"AK8963_HZH",      AK8963_HZH},
    {"AK8963_ST2",      AK8963_ST2},
    {"AK8963_CNTL1",    AK8963_CNTL1},
    {"AK8963_CNTL2",    AK8963_CNTL2},
    {"AK8963_ASTC",     AK8963_ASTC},
    {"AK8963_TS1",      AK8963_TS1},
    {"AK8963_TS2",      AK8963_TS2},
    {"AK8963_I2CDIS",   AK8963_I2CDIS},
    {"AK8963_ASAX",     AK8963_ASAX},
    {"AK8963_ASAY",     AK8963_ASAY},
    {"AK8963_ASAZ",     AK8963_ASAZ},
    {nullptr,           0}
};


//
// MPU9250::MPU925
//
// Arguments: None
//
// Returns: Nothing
//
// Constructor which does basic initialization
//

MPU9250::MPU9250(void) {
    mpu9250_i2c_addr_ = MPU9250_I2CADDR_DEFAULT;
    i2c_ = i2c0;
    ak8963_i2c_addr_ = AK8963_I2CADDR_DEFAULT;
    bypass_ = false;
    aux_auto_sample_ = true;
}

MPU9250::~MPU9250(void) {}


//
// MPU9250::dump_regs
//
// Arguments: None
//
// Returns: Nothing
//
// Dump the values of all registers of the MPU9250 and AK8963 to stdout
//

void MPU9250::dump_regs() {
    reg_t *regs_iter;
    uint8_t u;

    for (regs_iter = mpu9250_regs; regs_iter->name != nullptr; regs_iter++) {
        uint8_t u = mpu9250_read(regs_iter->val);
        printf("%s(0x%02x) = 0x%02x\n",
               regs_iter->name, regs_iter->val, (uint)u);
    }

    for (regs_iter = ak8963_regs; regs_iter->name != nullptr; regs_iter++) {
        uint8_t u = mpu9250_read(regs_iter->val);
        printf("%s(0x%02x) = 0x%02x\n",
               regs_iter->name, regs_iter->val, (uint)u);
    }
}

//
// MPU9250::init
//
// Arguments: i2c -- pointer to I2C object used to access the I2C bus
//            mpu9250_i2c_addr -- Address of MPU9250 on the I2C bus
//            ak8963_i2c_addr -- Address of AK8963 on the I2C bus
//            i2c_aux_master -- Use MPU9250 as auxiliary i2c bus master
//                              as opposed to i2c pass-through.
//            aux_auto_sample -- Autosample the aux i2c bus
//                               as opposed to manual sample.
//                               (only valid when i2c_aux_master is true)
//
// Returns: success -- 0
//          failure -- 2 - MPU9250 was not found
//                     3 - AK8963  was not found
//
// Initialize the MPU9250 for operation.  Must be called before anything
// useful can happen.  The main routine should initialize the I2C bus before
// calling this routine.
//

int MPU9250::init(i2c_inst_t *i2c,
                  uint8_t mpu9250_i2c_addr, uint8_t ak8963_i2c_addr,
                  bool i2c_aux_master, bool aux_auto_sample) {
    sleep_ms(100);

    i2c_ = i2c;
    spi_ = nullptr;

    mpu9250_i2c_addr_ = mpu9250_i2c_addr;
    ak8963_i2c_addr_ = ak8963_i2c_addr;
    bypass_ = !i2c_aux_master;
    if (bypass_) {
        aux_auto_sample_ = false;
    } else {
        aux_auto_sample_ = aux_auto_sample;
    }

    return common_init();
}

//
// MPU9250::init
//
// Arguments: spi -- pointer to SPI object used to access the SPI bus
//            mpu9250_spi_csn -- GPIO used as the chip select for the MPU9250
//            ak8963_i2c_addr -- Address of AK8963 on the I2C bus
//            aux_auto_sample -- Autosample the aux i2c bus
//                               as opposed to manual sample.
//
// Returns: success -- 0
//          failure -- 2 - MPU9250 was not found
//                     3 - AK8963  was not found
//
// Initialize the MPU9250 for operation.  Must be called before anything
// useful can happen.  The main routine should initialize the SPI bus before
// calling this routine.
//

int MPU9250::init(spi_inst_t *spi,
                  uint mpu9250_spi_csn, uint8_t ak8963_i2c_addr,
                  bool aux_auto_sample) {
    sleep_ms(100);

    spi_ = spi;
    i2c_ = nullptr;

    mpu9250_spi_csn_ = mpu9250_spi_csn;
    bypass_ = false;
    aux_auto_sample_ = aux_auto_sample;

    return common_init();
}

//
// MPU9250::common_init
//
// Arguments: None
//
// Returns: success -- 0
//          failure -- 2 - MPU9250 was not found
//                     3 - AK8963  was not found
//
// Core initialize items used by both SPI and I2C init methods.
//

int MPU9250::common_init() {
    if (spi_) {
        set_i2c_disable(true);
    }

    // reset the MPU9250
    mpu9250_write(MPU9250_PWR_MGMT_1, 0x80);
    sleep_ms(100);
    while (mpu9250_read(MPU9250_PWR_MGMT_1) & 0x80) {
        sleep_ms(1);
    }

    if (spi_) {
        set_i2c_disable(true);
    }

    if (mpu9250_read(MPU9250_WHO_AM_I) != MPU9250_DEVICE_ID) {
        return 2;
    }

    set_i2c_bypass(bypass_);

    // reset the AK8963
    ak8963_write(AK8963_CNTL2, 0x1);
    while (ak8963_read(AK8963_CNTL2) & 0x1) {
        sleep_ms(1);
    }

    if (ak8963_read(AK8963_WHO_AM_I) != AK8963_DEVICE_ID) {
        return 3;
    }

    set_gyro_filter_bandwidth(MPU9250_GYRO_BAND_184_HZ);
    set_accel_filter_bandwidth(MPU9250_ACCEL_BAND_184_HZ);

    set_gyro_range(MPU9250_RANGE_500_DPS);

    set_accel_range(MPU9250_RANGE_2_G);

    // set clock config to PLL with Gyro X reference
    mpu9250_write(MPU9250_PWR_MGMT_1, 0x01);

    // enter fuse access mode of mag which is required to read adjustment data
    set_mag_mode(AK8963_MODE_FUSE_ROM);

    // read three bytes of sensitivity adjustment data (one byte for each axis)
    uint8_t raw[3];
    ak8963_read(AK8963_ASAX, raw, 3);

    // store sensitivity adjustment per axis (see data sheet for calculation)
    asa_.x = (static_cast<float>(raw[0]) - 128) / 256 + 1;
    asa_.y = (static_cast<float>(raw[1]) - 128) / 256 + 1;
    asa_.z = (static_cast<float>(raw[2]) - 128) / 256 + 1;

    // Finish setting up magnetometer
    set_mag_mode(AK8963_MODE_100HZ);
    set_mag_sensitivity(AK8963_SENSITIVITY_16b);
    sleep_ms(100);

    // Set up MPU9250 I2C master
    if (!bypass_ && aux_auto_sample_) {
        config_i2c_slave_sample();
    }

    return 0;
}

//
//
// MPU9250::set_i2c_bypass
//
// Arguments: bypass -- true to enable bypass or false to disable
//
// Returns: Nothing
//
// Enable or disable the I2C bypass on the MPU9250.  When bypass is enabled
// the auxiliary I2C pins of the MPU9250 are electrically connected to the
// primary I2C pins and the AK8963 becomes visible from the primary I2C bus.
//

void MPU9250::set_i2c_bypass(bool bypass) {
    bypass_ = bypass;
    uint8_t u = mpu9250_read(MPU9250_INT_PIN_CFG);
    u = (u & ~(1 << 1)) | (bypass << 1);
    mpu9250_write(MPU9250_INT_PIN_CFG, u);

    u = mpu9250_read(MPU9250_USER_CTRL);
    u = (u & ~(1 << 5)) | (!bypass << 5);
    mpu9250_write(MPU9250_USER_CTRL, u);
}

//
//
// MPU9250::set_i2c_disable
//
// Arguments: disable -- Disable or enable the interface
//
// Returns: Nothing
//
// Enable or disable the primary I2C interface on the MPU9250.
//

void MPU9250::set_i2c_disable(bool disable) {
    uint8_t u = mpu9250_read(MPU9250_USER_CTRL);
    u &= ~(1 << 4);
    u |= (disable << 4);
    mpu9250_write(MPU9250_USER_CTRL, u);
}

//
// MPU9250::config_i2c_slave_sample
//
// Arguments: None
//
// Returns: Nothing
//
// Configure and start slave I2C sampling of the AK8963
//

void MPU9250::config_i2c_slave_sample(void) {
    // disable sampling
    mpu9250_write(MPU9250_I2C_SLV0_CTRL, 0x00);

    // disable using fifo for sensor reads
    uint8_t u = mpu9250_read(MPU9250_FIFO_EN);
    u &= ~0x1;
    mpu9250_write(MPU9250_FIFO_EN, u);

    // set I2C clock to 400MHz
    u = mpu9250_read(MPU9250_I2C_MST_CTRL);
    u &= ~0xf;
    u |= 13;
    mpu9250_write(MPU9250_I2C_MST_CTRL, u);

    // decrease sample rate to something faster than the mag sensor update
    // rate (100Hz) but not obnoxiously fast.  Only valid if DLPF is active
    mpu9250_gyro_bandwidth_t bw = get_gyro_filter_bandwidth();
    if ((bw != MPU9250_GYRO_BAND_3600_HZ)
            && (bw != MPU9250_GYRO_BAND_8800_HZ)) {
        mpu9250_write(MPU9250_I2C_MST_DELAY_CTRL, 1);
        u = mpu9250_read(MPU9250_I2C_SLV4_CTRL);
        u &= ~0x1f;
        u |= 0x4;
        mpu9250_write(MPU9250_I2C_SLV4_CTRL, u);
    }

    // set up device address, register address
    mpu9250_write(MPU9250_I2C_SLV0_ADDR, 0x80 | ak8963_i2c_addr_);
    mpu9250_write(MPU9250_I2C_SLV0_REG, AK8963_ST1);

    // set size of read and enable sampling
    mpu9250_write(MPU9250_I2C_SLV0_CTRL, 0x88);
}

//
// MPU9250::get_clock
//
// Arguments: None
//
// Returns: The clock setting
//
// Get the currently programmed clock source as one of the enumerated values
// of type mpu9250_clock_select_t
//

mpu9250_clock_select_t MPU9250::get_clock(void) {
    uint8_t pwr_mgmt = mpu9250_read(MPU9250_PWR_MGMT_1);
    return (mpu9250_clock_select_t)(pwr_mgmt & 0x7);
}

//
// MPU9250::set_clock
//
// Arguments: new_clock -- New clock source
//
// Returns: Nothing
//
// Program the clock source for the sensors.  The new clock should be
// one of the enumerated values of type mpu9250_clock_select_t.
//

void MPU9250::set_clock(mpu9250_clock_select_t new_clock) {
    uint8_t pwr_mgmt = mpu9250_read(MPU9250_PWR_MGMT_1);
    pwr_mgmt &= ~0x7;
    pwr_mgmt |= new_clock & 0x7;
    mpu9250_write(MPU9250_PWR_MGMT_1, pwr_mgmt);
}

//
// MPU9250::get_accel_range
//
// Arguments: None
//
// Returns: The accelerometer range
//
// Get the currently programmed accelerometer range as one of the
// enumerated values of type mpu9250_accel_range_t
//

mpu9250_accel_range_t MPU9250::get_accel_range(void) {
    uint8_t accel_config = mpu9250_read(MPU9250_ACCEL_CONFIG);
    uint8_t accel_range = (accel_config >> 3) & 0x3;

    return (mpu9250_accel_range_t)accel_range;
}

//
// MPU9250::set_accel_range
//
// Arguments: new_range -- new range to which the MPU9250 should be programmed
//
// Returns: Nothing
//
// Program the MPU9250 to the user specified range.  The range should be
// one of the enumberated values of type mpu9250_accel_range_t
//

void MPU9250::set_accel_range(mpu9250_accel_range_t new_range) {
    uint8_t accel_config = mpu9250_read(MPU9250_ACCEL_CONFIG);
    accel_config &= ~(0x3 << 3);
    accel_config |= (uint8_t)new_range << 3;

    mpu9250_write(MPU9250_ACCEL_CONFIG, accel_config);
}

//
// MPU9250::get_gyro_range
//
// Arguments: None
//
// Returns: The gyro range
//
// Get the currently programmed gyro range as one of the
// enumerated values of type mpu9250_gyro_range_t
//

mpu9250_gyro_range_t MPU9250::get_gyro_range(void) {
    uint8_t gyro_config = mpu9250_read(MPU9250_GYRO_CONFIG);
    uint8_t gyro_range = (gyro_config >> 3) & 0x3;

    return (mpu9250_gyro_range_t)gyro_range;
}

//
//
// MPU9250::set_gyro_range
//
// Arguments: new_range -- new range to which the MPU9250 should be programmed
//
// Returns: Nothing
//
// Program the MPU9250 to the user specified range.  The range should be
// one of the enumerated values of type mpu9250_gyro_range_t
//

void MPU9250::set_gyro_range(mpu9250_gyro_range_t new_range) {
    uint8_t gyro_config = mpu9250_read(MPU9250_GYRO_CONFIG);
    gyro_config &= ~(0x3 << 3);
    gyro_config |= (uint8_t)new_range << 3;

    mpu9250_write(MPU9250_GYRO_CONFIG, gyro_config);
}

//
// MPU9250::get_accel_filter_bandwidth
//
// Arguments: None
//
// Returns: The accelerometer bandwidth setting
//
// Get the current accelerometer filter bandwidth as one of the enumerated
// values of type mpu9250_accel_bandwidth_t
//

mpu9250_accel_bandwidth_t MPU9250::get_accel_filter_bandwidth(void) {
    uint8_t u = mpu9250_read(MPU9250_ACCEL_CONFIG2);
    u &= 0xf;
    if (u & 0x8) {
        return MPU9250_ACCEL_BAND_1130_HZ;
    } else if (u == 0x7) {
        return MPU9250_ACCEL_BAND_460_HZ;
    } else {
        return (mpu9250_accel_bandwidth_t)u;
    }
}

//
// MPU9250::set_accel_filter_bandwidth
//
// Arguments: bandwidth - new bandwidth for the accelerometer
//
// Returns: Nothing
//
// Program the accelerometer filter bandwidth as one of the enumerated values
// of type mpu9250_accel_bandwidth_t
//

void MPU9250::set_accel_filter_bandwidth(mpu9250_accel_bandwidth_t bandwidth) {
    uint8_t u = mpu9250_read(MPU9250_ACCEL_CONFIG2);
    u &= 0xf;
    mpu9250_write(MPU9250_ACCEL_CONFIG2, u | bandwidth);
}

//
// MPU9250::get_gyro_filter_bandwidth
//
// Arguments: None
//
// Returns: The gyro bandwidth setting
//
// Get the current gyro filter bandwidth as one of the enumerated values
// of type mpu9250_gyro_bandwidth_t
//

mpu9250_gyro_bandwidth_t MPU9250::get_gyro_filter_bandwidth(void) {
    uint8_t u = mpu9250_read(MPU9250_CONFIG);
    u &= 0x7;

    uint8_t v = mpu9250_read(MPU9250_GYRO_CONFIG);
    v &= 0x3;

    if (v & 0x1) {
        return MPU9250_GYRO_BAND_8800_HZ;
    } else if (v == 0x2) {
        return MPU9250_GYRO_BAND_3600_HZ;
    } else {
        return (mpu9250_gyro_bandwidth_t)(u);
    }
}

//
// MPU9250::set_gyro_filter_bandwidth
//
// Arguments: bandwidth - new bandwidth for the gyro
//
// Returns: Nothing
//
// Program the gyro filter bandwidth as one of the enumerated values
// of type mpu9250_gyro_bandwidth_t
//

void MPU9250::set_gyro_filter_bandwidth(mpu9250_gyro_bandwidth_t bandwidth) {
    uint8_t u = mpu9250_read(MPU9250_GYRO_CONFIG);
    u &= ~0x3;
    mpu9250_write(MPU9250_GYRO_CONFIG, u | ((bandwidth >> 3) & 0x3));

    u = mpu9250_read(MPU9250_CONFIG);
    u &= ~0x7;
    mpu9250_write(MPU9250_CONFIG, u | (bandwidth & 0x7));

    if (!bypass_ && aux_auto_sample_) {
        config_i2c_slave_sample();
    }
}

//
// MPU9250::get_mag_mode
//
// Arguments: None
//
// Returns: The mag mode
//
// Get the currently programmed mode of the AK8963 magnetometer as one
// of the enumerated values of type ak8963_mag_mode_t
//

ak8963_mag_mode_t MPU9250::get_mag_mode(void) {
    switch (ak8963_read(AK8963_CNTL1) & 0xf) {
        case 0:
            return AK8963_MODE_POWER_DOWN;
        case 1:
            return AK8963_MODE_SINGLE;
        case 2:
            return AK8963_MODE_8HZ;
        case 4:
            return AK8963_MODE_EXT_TRIG;
        case 6:
            return AK8963_MODE_100HZ;
        case 8:
            return AK8963_MODE_SELF_TEST;
        case 15:
            return AK8963_MODE_FUSE_ROM;
        default:
            return AK8963_MODE_POWER_DOWN;
    }
}

//
// MPU9250::set_mag_mode
//
// Arguments: new_mode -- New mode for the magnetometer
//
// Returns: Nothing
//
// Program the mode of the AK8963 magnetometer.  The mode should be one of the
// enumerated values of type ak8963_mag_mode_t.
//

void MPU9250::set_mag_mode(ak8963_mag_mode_t new_mode) {
    uint8_t mag_mode = ak8963_read(AK8963_CNTL1);
    mag_mode &= ~0xf;
    ak8963_write(AK8963_CNTL1, mag_mode | (uint8_t)AK8963_MODE_POWER_DOWN);
    sleep_ms(10);

    switch (new_mode) {
        case AK8963_MODE_SINGLE:
            mag_mode |= (uint8_t)AK8963_MODE_SINGLE;
            break;
        case AK8963_MODE_8HZ:
            mag_mode |= (uint8_t)AK8963_MODE_8HZ;
            break;
        case AK8963_MODE_EXT_TRIG:
            mag_mode |= (uint8_t)AK8963_MODE_EXT_TRIG;
            break;
        case AK8963_MODE_100HZ:
            mag_mode |= (uint8_t)AK8963_MODE_100HZ;
            break;
        case AK8963_MODE_SELF_TEST:
            mag_mode |= (uint8_t)AK8963_MODE_SELF_TEST;
            break;
        case AK8963_MODE_FUSE_ROM:
            mag_mode |= (uint8_t)AK8963_MODE_FUSE_ROM;
            break;
        default:
            mag_mode |= (uint8_t)AK8963_MODE_POWER_DOWN;
            break;
    }
    ak8963_write(AK8963_CNTL1, mag_mode);
    sleep_ms(10);
}

//
// MPU9250::get_mag_sensitivity
//
// Arguments: None
//
// Returns: The mag sensitivity
//
// Get the currently programmed sensitivity of the AK8963 magnetometer as one
// of the enumerated values of type ak8963_mag_mode_t
//

ak8963_mag_sensitivity_t MPU9250::get_mag_sensitivity(void) {
    uint8_t cntl = ak8963_read(AK8963_CNTL1);
    uint8_t mag_sensitivity = (cntl >> 4) & 0x1;
    return mag_sensitivity ? AK8963_SENSITIVITY_16b : AK8963_SENSITIVITY_14b;
}

//
// MPU9250::set_mag_sensitivity
//
// Arguments: new_range -- New sensitivity for the magnetometer
//
// Returns: Nothing
//
// Program the sensitivity of the AK8963 magnetometer.  The sensitivity
// should be one of the enumerated values of type ak8963_mag_sensitivity_t.
//

void MPU9250::set_mag_sensitivity(ak8963_mag_sensitivity_t new_range) {
    uint8_t cntl = ak8963_read(AK8963_CNTL1);
    cntl &= ~(1 << 4);
    ak8963_write(AK8963_CNTL1, cntl | ((uint8_t)new_range << 4));
}

//
// MPU9250::read
//
// Arguments: accel -- User allocated structure to hold the latest values
//                     of X, Y, and Z from the accelerometer
//            gyro -- User allocated structure to hold the latest values
//                    of X, Y, and Z from the gyro
//            mag -- User allocated structure to hold the latest values
//                   of X, Y, and Z from the magnetometer
//            temperature -- pointer to location to hold the latest temperature
//                           reading
//
// Returns: true -- new data was available
//          false -- no new data available
//
// Read the latest sensor data from the MPU9250 and write it to the user
// provided structures.  Any structure pointer which is NULL will be ignored.
//

bool MPU9250::read(tuple<float> *accel, tuple<float> *gyro, tuple<float> *mag,
                  float *temperature) {
    uint8_t buffer[14];
    bool    new_data = false;

    // get raw readings of temp, accel, and gyro if new data is available
    if (mpu9250_read(MPU9250_INT_STATUS) & 0x1) {
        new_data = true;
        mpu9250_read(MPU9250_ACCEL_XOUT_H, buffer, 14);

        int16_t rawAccX, rawAccY, rawAccZ;
        rawAccX = buffer[0] << 8 | buffer[1];
        rawAccY = buffer[2] << 8 | buffer[3];
        rawAccZ = buffer[4] << 8 | buffer[5];

        int16_t rawTemp;
        rawTemp = buffer[6] << 8 | buffer[7];

        int16_t rawGyroX, rawGyroY, rawGyroZ;
        rawGyroX = buffer[8] << 8 | buffer[9];
        rawGyroY = buffer[10] << 8 | buffer[11];
        rawGyroZ = buffer[12] << 8 | buffer[13];

        // adjust raw data for temp, accel, gyro
        temperature_ = (rawTemp / 340.0) + 36.53;

        mpu9250_accel_range_t accel_range = get_accel_range();

        float accel_scale = 1;
        if (accel_range == MPU9250_RANGE_16_G) {
            accel_scale = 2048;
        }
        if (accel_range == MPU9250_RANGE_8_G) {
            accel_scale = 4096;
        }
        if (accel_range == MPU9250_RANGE_4_G) {
            accel_scale = 8192;
        }
        if (accel_range == MPU9250_RANGE_2_G) {
            accel_scale = 16384;
        }

        accel_.x = static_cast<float>(rawAccX) / accel_scale * MPU9250_G_TO_MPS2;
        accel_.y = static_cast<float>(rawAccY) / accel_scale * MPU9250_G_TO_MPS2;
        accel_.z = static_cast<float>(rawAccZ) / accel_scale * MPU9250_G_TO_MPS2;

        mpu9250_gyro_range_t gyro_range = get_gyro_range();

        float gyro_scale = 1;
        if (gyro_range == MPU9250_RANGE_250_DPS) {
            gyro_scale = 131;
        }
        if (gyro_range == MPU9250_RANGE_500_DPS) {
            gyro_scale = 65.5;
        }
        if (gyro_range == MPU9250_RANGE_1000_DPS) {
            gyro_scale = 32.8;
        }
        if (gyro_range == MPU9250_RANGE_2000_DPS) {
            gyro_scale = 16.4;
        }

        gyro_.x = static_cast<float>(rawGyroX) / gyro_scale * MPU9250_DPS_TO_RPS;
        gyro_.y = static_cast<float>(rawGyroY) / gyro_scale * MPU9250_DPS_TO_RPS;
        gyro_.z = static_cast<float>(rawGyroZ) / gyro_scale * MPU9250_DPS_TO_RPS;
    }

    // Read mag sensor
    if (bypass_ || !aux_auto_sample_) {
        ak8963_read(AK8963_ST1, buffer, 8);
    } else {
        mpu9250_read(MPU9250_EXT_SENS_DATA_00, buffer, 8);
    }

    uint8_t st1 = buffer[0];
    uint8_t st2 = buffer[7];
    uint8_t *data = &buffer[1];

    if (st1 & 0x1) {
        new_data = true;

        // ignore new data if there was overflow
        if (!(st2 & 0x8)) {
            int16_t rawMagX, rawMagY, rawMagZ;
            rawMagX = data[1] << 8 | data[0];
            rawMagY = data[3] << 8 | data[2];
            rawMagZ = data[5] << 8 | data[4];

            ak8963_mag_sensitivity_t mag_sensitivity = get_mag_sensitivity();
            float mag_scale;
            if (mag_sensitivity == AK8963_SENSITIVITY_16b) {
                // 16b sensitivity
                mag_scale = 0.15;
            } else {
                // 14b sensitivity
                mag_scale = 0.6;
            }

            // The magnetometer in the MPU9250 is not aligned the same way as
            // the accelerometer and gyro.  The x and y axes of the acceleromter
            // are swapped when aligned with the coordinate system of the
            // magnetometer and the z directions are reversed.  Adjust the
            // assignments of mag.x, mag.y, and mag.z so the coodinate systems
            // match
            mag_.x = asa_.y * rawMagY * mag_scale;
            mag_.y = asa_.x * rawMagX * mag_scale;
            mag_.z = -(asa_.z * rawMagZ * mag_scale);
        }
    }

    if (accel) {
        *accel = accel_;
    }
    if (gyro) {
        *gyro = gyro_;
    }
    if (mag) {
        *mag = mag_;
    }
    if (temperature) {
        *temperature = temperature_;
    }

    return new_data;
}

//
// MPU9250::mpu9250_read
//
// Arguments: reg_addr -- Address of MPU9250 register to read
//
// Returns: success - value of register
//          failure - 0 (may conflict with valid register value)
//
// Function to read a register from the MPU9250.  The value of the
// register is returned with no reliable way to return an error.  Use the
// version of this function with user buffer and buffer size if error checking
// is desired.
//

uint8_t MPU9250::mpu9250_read(uint8_t reg_addr) {
    uint8_t data;
    if (i2c_) {
        if (!i2c_read(i2c_, mpu9250_i2c_addr_, reg_addr, &data, 1)) {
            return 0;
        }
    } else {
        if (!spi_read(spi_, mpu9250_spi_csn_, reg_addr, &data, 1)) {
            return 0;
        }
    }

    return data;
}

//
// MPU9250::mpu9250_read
//
// Arguments: reg_addr -- Address of MPU9250 register to read
//            buffer -- User provided buffer for result of the read
//            len -- Number of bytes to read
//
// Returns: success -- true
//          failure -- false
//
// Function to read a register from the MPU9250 using I2C.  The value of the
// register is written to the user provided buffer which must be capable of
// holding at least "len" bytes.  On success true is returned and any error
// will result in false being returned.
//

bool MPU9250::mpu9250_read(uint8_t reg_addr, uint8_t *buffer, size_t len) {
    if (i2c_) {
        return i2c_read(i2c_, mpu9250_i2c_addr_, reg_addr, buffer, len);
    } else {
        return spi_read(spi_, mpu9250_spi_csn_, reg_addr, buffer, len);
    }
}

//
// MPU9250::ak8963_read
//
// Arguments: reg_addr -- Address of AK8963 register to read
//
// Returns: success - value of register
//          failure - 0 (may conflict with valid register value)
//
// Function to read a register from the AK8963 magnetometer within the MPU9250.
// The value of the register is returned with no reliable way to return an
// error.  Use the version of this function with user buffer and buffer size
// if error checking is desired.
//

uint8_t MPU9250::ak8963_read(uint8_t reg_addr) {
    uint8_t data;

    if (bypass_) {
        if (!i2c_read(i2c_, ak8963_i2c_addr_, reg_addr, &data, 1)) {
            return 0;
        }
    } else {
        ak8963_read(reg_addr, &data, 1);
    }

    return data;
}

//
// MPU9250::ak8963_read
//
// Arguments: reg_addr -- Address of AK8963 register to read
//            buffer -- User provided buffer for result of the read
//            len -- Number of bytes to read
//
// Returns: success -- true
//          failure -- false
//
// Function to read a register from the AK8963 magnetometer within the MPU9250.
// The value of the register is written to the user provided buffer which must
// be capable of holding at least "len" bytes.  On success true is returned and
// any error will result in false being returned.
//

bool MPU9250::ak8963_read(uint8_t reg_addr, uint8_t *buffer, size_t len) {
    if (bypass_) {
        return i2c_read(i2c_, ak8963_i2c_addr_, reg_addr, buffer, len);
    } else {
        for (int i = 0; i < len; i++) {
            // set up AK8963 address and r/w# bit
            mpu9250_write(MPU9250_I2C_SLV4_ADDR, (1 << 7) | ak8963_i2c_addr_);

            // set up the AK8963 register
            mpu9250_write(MPU9250_I2C_SLV4_REG, reg_addr + i);

            // Start operation
            uint8_t u = mpu9250_read(MPU9250_I2C_SLV4_CTRL);
            u |= 0x80;
            mpu9250_write(MPU9250_I2C_SLV4_CTRL, u);

            // wait for completion or error
            do {
                u = mpu9250_read(MPU9250_I2C_MST_STATUS);
            } while (!(u & (0x7 << 4)));

            // check for error
            if (u & (0x3 << 4)) {
                return false;
            }

            // extract result
            buffer[i] = mpu9250_read(MPU9250_I2C_SLV4_DI);
        }
        return true;
    }
}

//
// MPU9250::mpu9250_write
//
// Arguments: reg_addr -- Address of MPU9250 register to write
//
// Returns: Nothing
//
// Function to write a register in the MPU9250.  No value is returned.
// Use the version of this function with user buffer and buffer size
// if error checking is desired.
//

void MPU9250::mpu9250_write(uint8_t reg_addr, uint8_t data) {
    if (i2c_) {
        i2c_write(i2c_, mpu9250_i2c_addr_, reg_addr, &data, 1);
    } else {
        spi_write(spi_, mpu9250_spi_csn_, reg_addr, &data, 1);
    }
}

//
// MPU9250::mpu9250_write
//
// Arguments: reg_addr -- Address of MPU9250 register to write
//            buffer -- User provided buffer with data to write to the register
//            len -- Number of bytes to write
//
// Returns: success -- true
//          failure -- false
//
// Function to write a register in the MPU9250.  The value written to the
// register is taken from the user provided buffer of size "len".  On
// success true is returned and any error will result in false being returned.
//

bool MPU9250::mpu9250_write(uint8_t reg_addr, const uint8_t *buffer,
                            size_t len) {
    if (i2c_) {
        return i2c_write(i2c_, mpu9250_i2c_addr_, reg_addr, buffer, len);
    } else {
        return spi_write(spi_, mpu9250_spi_csn_, reg_addr, buffer, len);
    }
}

//
// MPU9250::ak8963_write
//
// Arguments: reg_addr -- Address of AK8963 register to write
//
// Returns: Nothing
//
// Function to write a register in the AK8963 magnetometer in the MPU9250.
// No value is returned.  Use the version of this function with user buffer and
// buffer size if error checking is desired.
//

void MPU9250::ak8963_write(uint8_t reg_addr, uint8_t data) {
    if (bypass_) {
        i2c_write(i2c_, ak8963_i2c_addr_, reg_addr, &data, 1);
    } else {
        ak8963_write(reg_addr, &data, 1);
    }
}

//
// MPU9250::ak8963_write
//
// Arguments: reg_addr -- Address of AK8963 register to write
//            buffer -- User provided buffer with data to write to the register
//            len -- Number of bytes to write
//
// Returns: success -- true
//          failure -- false
//
// Function to write a register in the AK8963 magnetometer in the MPU9250
// The value written to the register is taken from the user provided
// buffer of size "len".  On success true is returned and any error will
// result in false being returned.
//

bool MPU9250::ak8963_write(uint8_t reg_addr, const uint8_t *buffer,
                           size_t len) {
    if (bypass_) {
        return i2c_write(i2c_, ak8963_i2c_addr_, reg_addr, buffer, len);
    } else {
        for (int i = 0; i < len; i++) {
            // set up AK8963 address and r/w# bit
            mpu9250_write(MPU9250_I2C_SLV4_ADDR, ak8963_i2c_addr_);

            // set up the AK8963 register
            mpu9250_write(MPU9250_I2C_SLV4_REG, reg_addr + i);

            // write data
            mpu9250_write(MPU9250_I2C_SLV4_DO, buffer[i]);

            // Start operation
            uint8_t u = mpu9250_read(MPU9250_I2C_SLV4_CTRL);
            u |= 0x80;
            mpu9250_write(MPU9250_I2C_SLV4_CTRL, u);

            // wait for completion or error
            do {
                u = mpu9250_read(MPU9250_I2C_MST_STATUS);
            } while (!(u & (0x7 << 4)));

            // check for error
            if (u & (0x3 << 4)) {
                return false;
            }
        }
    }

    return true;
}

//
// MPU9250::i2c_read
//
// Arguments: i2c -- i2c object controlling the I2C bus
//            device_addr -- Address of the device on the I2C bus
//            reg_addr -- Address of register to read
//            buffer -- User provided buffer for result of the read
//            len -- Number of bytes to read
//
// Returns: success -- true
//          failure -- false
//
// Function to read a register on I2C.  It assumes a single byte of address
// must be written to the device with the register address and that subsequent
// reads will access the register.
//

bool MPU9250::i2c_read(i2c_inst_t *i2c, uint8_t device_addr, uint8_t reg_addr,
                       uint8_t *buffer, size_t len) {
    // write register address
    if (i2c_write_blocking(i2c, device_addr, &reg_addr, 1, true) != 1) {
        i2c_write_blocking(i2c, device_addr, &reg_addr, 0, false);
        return false;
    }

    // read data from register
    if (i2c_read_blocking(i2c, device_addr, buffer, len, false) != len) {
        return false;
    }

    return true;
}

//
// MPU9250::i2c_write
//
// Arguments: i2c -- i2c object controlling the I2C bus
//            device_addr -- Address of the device on the I2C bus
//            reg_addr -- Address of register to write
//            buffer -- Buffer of data bytes to write
//            len -- Number of bytes to write
//
// Returns: success -- true
//          failure -- false
//
// Function to write a register on I2C.  It assumes a single byte of address
// must be written to the device with the register address and that subsequent
// writes will access the register.
//

bool MPU9250::i2c_write(i2c_inst_t *i2c, uint8_t device_addr, uint8_t reg_addr,
                        const uint8_t *buffer, size_t len) {
    uint8_t i2c_data[33];

    if (len >= sizeof(i2c_data)) {
        return false;
    }

    i2c_data[0] = reg_addr;
    memcpy(&i2c_data[1], buffer, len);

    // write register address and data
    int retval = i2c_write_blocking(i2c, device_addr, i2c_data, len + 1, false);
    return retval == (len + 1);
}

//
// MPU9250::spi_read
//
// Arguments: spi -- spi object controlling the SPI bus
//            csn-- GPIO of chip select for device being read
//            reg_addr -- Address of register to read
//            buffer -- User provided buffer for result of the read
//            len -- Number of bytes to read
//
// Returns: success -- true
//          failure -- false
//
// Function to read a register on SPI.  It assumes a single byte of address
// must be written to the device with the register address and that subsequent
// reads will access the register.
//

bool MPU9250::spi_read(spi_inst_t *spi, uint csn, uint8_t reg_addr,
                       uint8_t *buffer, size_t len) {

    uint8_t spi_rx_data[33];
    uint8_t spi_tx_data[33];

    if (len >= sizeof(spi_tx_data)) {
        return false;
    }

    // Mark operation as a read and insert address in tx buffer
    spi_tx_data[0] = reg_addr | 0x80;

    // Do the operation full-duplex
    gpio_put(csn, 0);
    int retval = spi_write_read_blocking(spi, spi_tx_data,
                                         spi_rx_data, len + 1);
    gpio_put(csn, 1);
    memcpy(buffer, &spi_rx_data[1], len);
    return retval == (len + 1);
}

//
// MPU9250::spi_write
//
// Arguments: spi -- spi object controlling the SPI bus
//            csn-- GPIO of chip select for device being read
//            reg_addr -- Address of register to write
//            buffer -- Buffer of data bytes to write
//            len -- Number of bytes to write
//
// Returns: success -- true
//          failure -- false
//
// Function to srite a register on SPI.  It assumes a single byte of address
// must be written to the device with the register address and that subsequent
// writes will access the register.
//

bool MPU9250::spi_write(spi_inst_t *spi, uint csn, uint8_t reg_addr,
                        const uint8_t *buffer, size_t len) {
    uint8_t spi_tx_data[33];

    if (len >= sizeof(spi_tx_data)) {
        return false;
    }

    spi_tx_data[0] = reg_addr;
    memcpy(&spi_tx_data[1], buffer, len);

    // write register address and data
    gpio_put(csn, 0);
    int retval = spi_write_blocking(spi, spi_tx_data, len + 1);
    gpio_put(csn, 1);
    return retval == (len + 1);
}
