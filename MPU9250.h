//
// MPU9250.h
//
// Declaration of the MPU9250 class used for control of the MPU9250
// motion sensor.
//

#ifndef MPU9250_H
#define MPU9250_H

#include <hardware/i2c.h>
#include <hardware/spi.h>
#include <string.h>
#include <stdint.h>

// basic conversion constants
#define MPU9250_G_TO_MPS2  9.80665
#define MPU9250_DPS_TO_RPS 0.017453293
#define MPU9250_RPS_TO_DPS 57.29578

// MPU9250 accelerometer and gyro
#define MPU9250_I2CADDR_DEFAULT 0x68
#define MPU9250_DEVICE_ID 0x71

// see MPU9250 register map
#define MPU9250_SELF_TEST_X_GYRO   0x0
#define MPU9250_SELF_TEST_Y_GYRO   0x1
#define MPU9250_SELF_TEST_Z_GYRO   0x2
#define MPU9250_SELF_TEST_X_ACCEL  0xD
#define MPU9250_SELF_TEST_Y_ACCEL  0xE
#define MPU9250_SELF_TEST_Z_ACCEL  0xF
#define MPU9250_XG_OFFSET_H        0x13
#define MPU9250_XG_OFFSET_L        0x14
#define MPU9250_YG_OFFSET_H        0x15
#define MPU9250_YG_OFFSET_L        0x16
#define MPU9250_ZG_OFFSET_H        0x17
#define MPU9250_ZG_OFFSET_L        0x18
#define MPU9250_SMPLRT_DIV         0x19
#define MPU9250_CONFIG             0x1A
#define MPU9250_GYRO_CONFIG        0x1B
#define MPU9250_ACCEL_CONFIG       0x1C
#define MPU9250_ACCEL_CONFIG2      0x1D
#define MPU9250_LP_ACCEL_ODR       0x1E
#define MPU9250_WOM_THR            0x1F
#define MPU9250_FIFO_EN            0x23
#define MPU9250_I2C_MST_CTRL       0x24
#define MPU9250_I2C_SLV0_ADDR      0x25
#define MPU9250_I2C_SLV0_REG       0x26
#define MPU9250_I2C_SLV0_CTRL      0x27
#define MPU9250_I2C_SLV1_ADDR      0x28
#define MPU9250_I2C_SLV1_REG       0x29
#define MPU9250_I2C_SLV1_CTRL      0x2A
#define MPU9250_I2C_SLV2_ADDR      0x2B
#define MPU9250_I2C_SLV2_REG       0x2C
#define MPU9250_I2C_SLV2_CTRL      0x2D
#define MPU9250_I2C_SLV3_ADDR      0x2E
#define MPU9250_I2C_SLV3_REG       0x2F
#define MPU9250_I2C_SLV3_CTRL      0x30
#define MPU9250_I2C_SLV4_ADDR      0x31
#define MPU9250_I2C_SLV4_REG       0x32
#define MPU9250_I2C_SLV4_DO        0x33
#define MPU9250_I2C_SLV4_CTRL      0x34
#define MPU9250_I2C_SLV4_DI        0x35
#define MPU9250_I2C_MST_STATUS     0x36
#define MPU9250_INT_PIN_CFG        0x37
#define MPU9250_INT_ENABLE         0x38
#define MPU9250_INT_STATUS         0x3A
#define MPU9250_ACCEL_XOUT_H       0x3B
#define MPU9250_ACCEL_XOUT_L       0x3C
#define MPU9250_ACCEL_YOUT_H       0x3D
#define MPU9250_ACCEL_YOUT_L       0x3E
#define MPU9250_ACCEL_ZOUT_H       0x3F
#define MPU9250_ACCEL_ZOUT_L       0x40
#define MPU9250_TEMP_OUT_H         0x41
#define MPU9250_TEMP_OUT_L         0x42
#define MPU9250_GYRO_XOUT_H        0x43
#define MPU9250_GYRO_XOUT_L        0x44
#define MPU9250_GYRO_YOUT_H        0x45
#define MPU9250_GYRO_YOUT_L        0x46
#define MPU9250_GYRO_ZOUT_H        0x47
#define MPU9250_GYRO_ZOUT_L        0x48
#define MPU9250_EXT_SENS_DATA_00   0x49
#define MPU9250_EXT_SENS_DATA_01   0x4A
#define MPU9250_EXT_SENS_DATA_02   0x4B
#define MPU9250_EXT_SENS_DATA_03   0x4C
#define MPU9250_EXT_SENS_DATA_04   0x4D
#define MPU9250_EXT_SENS_DATA_05   0x4E
#define MPU9250_EXT_SENS_DATA_06   0x4F
#define MPU9250_EXT_SENS_DATA_07   0x50
#define MPU9250_EXT_SENS_DATA_08   0x51
#define MPU9250_EXT_SENS_DATA_09   0x52
#define MPU9250_EXT_SENS_DATA_10   0x53
#define MPU9250_EXT_SENS_DATA_11   0x54
#define MPU9250_EXT_SENS_DATA_12   0x55
#define MPU9250_EXT_SENS_DATA_13   0x56
#define MPU9250_EXT_SENS_DATA_14   0x57
#define MPU9250_EXT_SENS_DATA_15   0x58
#define MPU9250_EXT_SENS_DATA_16   0x59
#define MPU9250_EXT_SENS_DATA_17   0x5A
#define MPU9250_EXT_SENS_DATA_18   0x5B
#define MPU9250_EXT_SENS_DATA_19   0x5C
#define MPU9250_EXT_SENS_DATA_20   0x5D
#define MPU9250_EXT_SENS_DATA_21   0x5E
#define MPU9250_EXT_SENS_DATA_22   0x5F
#define MPU9250_EXT_SENS_DATA_23   0x60
#define MPU9250_I2C_SLV0_DO        0x63
#define MPU9250_I2C_SLV1_DO        0x64
#define MPU9250_I2C_SLV2_DO        0x65
#define MPU9250_I2C_SLV3_DO        0x66
#define MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_SIGNAL_PATH_RESET  0x68
#define MPU9250_MOT_DETECT_CTRL    0x69
#define MPU9250_USER_CTRL          0x6A
#define MPU9250_PWR_MGMT_1         0x6B
#define MPU9250_PWR_MGMT_2         0x6C
#define MPU9250_FIFO_COUNTH        0x72
#define MPU9250_FIFO_COUNTL        0x73
#define MPU9250_FIFO_R_W           0x74
#define MPU9250_WHO_AM_I           0x75
#define MPU9250_XA_OFFSET_H        0x77
#define MPU9250_XA_OFFSET_L        0x78
#define MPU9250_YA_OFFSET_H        0x7A
#define MPU9250_YA_OFFSET_L        0x7B
#define MPU9250_ZA_OFFSET_H        0x7D
#define MPU9250_ZA_OFFSET_L        0x7E

typedef enum clock_select {
    MPU9250_INTERNAL_20MHz,
    MPU9250_PLL,
    MPU9250_STOP = 7,
} mpu9250_clock_select_t;

typedef enum {
    MPU9250_RANGE_2_G = 0b00,
    MPU9250_RANGE_4_G = 0b01,
    MPU9250_RANGE_8_G = 0b10,
    MPU9250_RANGE_16_G = 0b11,
} mpu9250_accel_range_t;

typedef enum {
    MPU9250_RANGE_250_DPS,
    MPU9250_RANGE_500_DPS,
    MPU9250_RANGE_1000_DPS,
    MPU9250_RANGE_2000_DPS,
} mpu9250_gyro_range_t;

typedef enum {
    MPU9250_GYRO_BAND_250_HZ,
    MPU9250_GYRO_BAND_184_HZ,
    MPU9250_GYRO_BAND_92_HZ,
    MPU9250_GYRO_BAND_41_HZ,
    MPU9250_GYRO_BAND_20_HZ,
    MPU9250_GYRO_BAND_10_HZ,
    MPU9250_GYRO_BAND_5_HZ,
    MPU9250_GYRO_BAND_3600_HZ,
    MPU9250_GYRO_BAND_8800_HZ,
} mpu9250_gyro_bandwidth_t;

typedef enum {
    MPU9250_ACCEL_BAND_460_HZ,
    MPU9250_ACCEL_BAND_184_HZ,
    MPU9250_ACCEL_BAND_92_HZ,
    MPU9250_ACCEL_BAND_41_HZ,
    MPU9250_ACCEL_BAND_20_HZ,
    MPU9250_ACCEL_BAND_10_HZ,
    MPU9250_ACCEL_BAND_5_HZ,
    MPU9250_ACCEL_BAND_1130_HZ = 0b1000
} mpu9250_accel_bandwidth_t;

// AK8963 magnetometer
#define AK8963_I2CADDR_DEFAULT 0x0c
#define AK8963_DEVICE_ID 0x48

// see AK8963 register map
#define AK8963_WHO_AM_I 0x0
#define AK8963_INFO     0x1
#define AK8963_ST1      0x2
#define AK8963_HXL      0x3
#define AK8963_HXH      0x4
#define AK8963_HYL      0x5
#define AK8963_HYH      0x6
#define AK8963_HZL      0x7
#define AK8963_HZH      0x8
#define AK8963_ST2      0x9
#define AK8963_CNTL1    0xA
#define AK8963_CNTL2    0xB
#define AK8963_ASTC     0xC
#define AK8963_TS1      0xD
#define AK8963_TS2      0xE
#define AK8963_I2CDIS   0xF
#define AK8963_ASAX     0x10
#define AK8963_ASAY     0x11
#define AK8963_ASAZ     0x12

typedef enum {
    AK8963_MODE_POWER_DOWN,
    AK8963_MODE_SINGLE,
    AK8963_MODE_8HZ,
    AK8963_MODE_EXT_TRIG = 0x4,
    AK8963_MODE_100HZ = 0x6,
    AK8963_MODE_SELF_TEST = 0x8,
    AK8963_MODE_FUSE_ROM = 0xf
} ak8963_mag_mode_t;

typedef enum {
    AK8963_SENSITIVITY_14b,
    AK8963_SENSITIVITY_16b
} ak8963_mag_sensitivity_t;

class MPU9250 {
 public:
    MPU9250();
    ~MPU9250();

    template <typename T>
    struct tuple {
        T x;
        T y;
        T z;
    };

    int init(i2c_inst_t *i2c,
             uint8_t mpu9250_i2c_addr = MPU9250_I2CADDR_DEFAULT,
             uint8_t ak8963_i2c_addr = AK8963_I2CADDR_DEFAULT,
             bool i2c_aux_master = true, bool aux_auto_sample = true);
    int init(spi_inst_t *spi,
             uint spi_csn = PICO_DEFAULT_SPI_CSN_PIN,
             uint8_t ak8963_i2c_addr = AK8963_I2CADDR_DEFAULT,
             bool aux_auto_sample = true);

    mpu9250_clock_select_t get_clock(void);
    void set_clock(mpu9250_clock_select_t);

    mpu9250_accel_range_t get_accel_range(void);
    void set_accel_range(mpu9250_accel_range_t);

    mpu9250_gyro_range_t get_gyro_range(void);
    void set_gyro_range(mpu9250_gyro_range_t);

    mpu9250_accel_bandwidth_t get_accel_filter_bandwidth(void);
    void set_accel_filter_bandwidth(mpu9250_accel_bandwidth_t bandwidth);

    mpu9250_gyro_bandwidth_t get_gyro_filter_bandwidth(void);
    void set_gyro_filter_bandwidth(mpu9250_gyro_bandwidth_t bandwidth);

    ak8963_mag_mode_t get_mag_mode(void);
    void set_mag_mode(ak8963_mag_mode_t mag_mode);

    ak8963_mag_sensitivity_t get_mag_sensitivity(void);
    void set_mag_sensitivity(ak8963_mag_sensitivity_t mag_sensitivity);

    bool read(tuple<float> *accel, tuple<float> *gyro, tuple<float> *mag,
              float *temperature);

    void dump_regs(void);

 private:
    typedef struct reg_s {
        const char *name;
        uint       val;
    } reg_t;
    static reg_t mpu9250_regs[];
    static reg_t ak8963_regs[];

    float temperature_;   // Last reading's temperature (C)
    tuple<float> accel_;  // Last reading's accelerometer X, Y, Z axis m/s^2
    tuple<float> gyro_;   // Last readings' gyro X, Y, Z in rad/s
    tuple<float> mag_;    // Last readings magnetometer X, Y, Z in uT
    tuple<float> asa_;    // mag sensitivity adjustment data for X, Y, and Z

    i2c_inst_t *i2c_;
    uint8_t mpu9250_i2c_addr_;
    uint8_t ak8963_i2c_addr_;
    spi_inst_t *spi_;
    uint mpu9250_spi_csn_;

    bool bypass_;
    bool aux_auto_sample_;

    int common_init(void);

    void set_i2c_bypass(bool bypass);
    void set_i2c_disable(bool disable);
    void config_i2c_slave_sample(void);

    uint8_t mpu9250_read(uint8_t reg_addr);
    bool mpu9250_read(uint8_t reg_addr, uint8_t *buffer, size_t len);
    void mpu9250_write(uint8_t reg_addr, uint8_t data);
    bool mpu9250_write(uint8_t reg_addr, const uint8_t *buffer, size_t len);

    uint8_t ak8963_read(uint8_t reg_addr);
    bool ak8963_read(uint8_t reg_addr, uint8_t *buffer, size_t len);
    void ak8963_write(uint8_t reg_addr, uint8_t data);
    bool ak8963_write(uint8_t reg_addr, const uint8_t *buffer, size_t len);

    static bool i2c_read(i2c_inst_t *i2c, uint8_t device_addr, uint8_t reg_addr,
                         uint8_t *buffer, size_t len);
    static bool i2c_write(i2c_inst_t *i2c, uint8_t device_addr, uint8_t reg_addr,
                          const uint8_t *buffer, size_t len);

    static bool spi_read(spi_inst_t *spi, uint csn, uint8_t reg_addr,
                         uint8_t *buffer, size_t len);
    static bool spi_write(spi_inst_t *spi, uint csn, uint8_t reg_addr,
                          const uint8_t *buffer, size_t len);
};

#endif
