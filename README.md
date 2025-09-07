# pico-MPU9250
This is a library for using the MPU9250 9-DoF accelerometer, gyro,
and magnetometer with the Raspberry Pi Pico SDK.  It supports the I2C and
SPI interfaces.  The API is described below.  See the
[examples](examples/README.md) for sample usage of the API.

## API
### MPU9250()
The API is based on an object of the class MPU9250.  The first step in using
the API is to create an object of this class.  The constructor does basic
initialization and should b e followed by the init() method.

    MPU9250:MPU9250();

| Argument | Description |
|----------|-------------|
| None     |             |

| Return Value | Description |
|--------------|-------------|
| Nothing      |      |

### init()
The init function must be called before any other function in the API.  It has
two versions to support I2C and SPI.  The user must initialize the I2C or SPI
system before calling init().

#### I2C

    int MPU9250::init(i2c_inst_t *i2c, uint8_t mpu9250_i2c_addr = 0x68, uint8_t ak8963_i2c_addr = 0x0C, bool i2c_aux_master = true, bool aux_auto_sample = true);

| Argument         | Description |
|------------------|-------------|
| i2c              | Pointer to I2C object used to access the I2C bus.  The default I2C object defined in the SDK is i2c_default |
| mpu9250_i2c_addr | Address of MPU9250 on the I2C bus |
| ak8963_i2c_addr  | Address of AK8963 on the I2C bus |
| i2c_aux_master   | Use MPU9250 as auxiliary I2C bus master as opposed to pass-through mode where the primary I2C is logically connected to the auxiliary bus and the Pico is the master. Generally should be true. |
| aux_auto_sample  | Autosample the auxiliary I2C bus using the MPU9250 I2C master as opposed to manual sampling initiated by the Pico.  This is only valid when i2c_aux_master is true and generally should be true for efficiency. |

| Return Value | Description |
|--------------|-------------|
| 0            | success     |
| 2            | MPU9250 was not found |
| 3            | AK8963 was not found |

#### SPI

    int MPU9250::init(spi_inst_t *spi, uint spi_csn = PICO_DEFAULT_SPI_CSN_PIN, uint8_t ak8963_i2c_addr = 0x0C, bool aux_auto_sample = true);

| Argument        | Description |
|-----------------|-------------|
| spi             | Pointer to SPI object used to access the SPI bus.  The default SPI object defined in the SDK is spi_default |
| spi_csn         | GPIO used as the chip select for the MPU9250 |
| ak8963_i2c_addr | Address of AK8963 on the I2C bus |
| aux_auto_sample | Autosample the auxiliary I2C bus using the MPU9250 I2C master as opposed to manual sampling initiated by the Pico.  It generally should be true for efficiency. |

| Return Value | Description |
|--------------|-------------|
| 0            | success     |
| 2            | MPU9250 was not found |
| 3            | AK8963 was not found |

### mpu9250_clock_select_t

    typedef enum clock_select {
        MPU9250_INTERNAL_20MHz,
        MPU9250_PLL,
        MPU9250_STOP = 7,
    } mpu9250_clock_select_t;

### get_clock()

Get the currently programmed clock source as one of the enumerated values of
type mpu9250_clock_select_t.

    mpu9250_clock_select_t MPU9250::get_clock(void);

| Argument | Description |
|----------|-------------|
| None     |             |

| Return Value | Description |
|--------------|-------------|
|              | Current clock source |

### set_clock()

Program the clock source for the sensors.  The new clock should be one of the
enumerated values of type mpu9250_clock_select_t.

    void MPU9250::set_clock(mpu9250_clock_select_t new_clock);


| Argument  | Description |
|-----------|-------------|
| new_clock | New clock source |

| Return Value | Description |
|--------------|-------------|
| Nothing      |             |

### mpu9250_accel_range_t

    typedef enum {
        MPU9250_RANGE_2_G = 0b00,
        MPU9250_RANGE_4_G = 0b01,
        MPU9250_RANGE_8_G = 0b10,
        MPU9250_RANGE_16_G = 0b11,
    } mpu9250_accel_range_t;

### get_accel_range()

Get the currently programmed accelerometer range as one of the enumerated
values of type mpu9250_accel_range_t

    mpu9250_accel_range_t MPU9250::get_accel_range(void);

| Argument | Description |
|----------|-------------|
| None     |             |

| Return Value | Description |
|--------------|-------------|
|              | Current accelerometer range |

### set_accel_range()

Program the MPU9250 to the user specified range.  The range should be one of
the enumerated values of type mpu9250_accel_range_t

    void MPU9250::set_accel_range(mpu9250_accel_range_t new_range);

| Argument | Description |
|----------|-------------|
| new_range | new range to which the MPU9250 should be programmed |

| Return Value | Description |
|--------------|-------------|
| Nothing      |             |

### mpu9250_gyro_range_t

    typedef enum {
        MPU9250_RANGE_250_DPS,
        MPU9250_RANGE_500_DPS,
        MPU9250_RANGE_1000_DPS,
        MPU9250_RANGE_2000_DPS,
    } mpu9250_gyro_range_t;

### get_gyro_range()

Get the currently programmed gyro range as one of the enumerated values of type
mpu9250_gyro_range_t

    mpu9250_gyro_range_t MPU9250::get_gyro_range(void);

| Argument | Description |
|----------|-------------|
| None     |             |

| Return Value | Description |
|--------------|-------------|
|              | Current gyro range |

### set_gyro_range()

Program the MPU9250 to the user specified range.  The range should be one
of the enumerated values of type mpu9250_accel_range_t

    void MPU9250::set_gyro_range(mpu9250_gyro_range_t new_range);

| Argument | Description |
|----------|-------------|
| new_range | new range to which the MPU9250 should be programmed |

| Return Value | Description |
|--------------|-------------|
| Nothing      |             |

### mpu9250_accel_bandwidth_t

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

### get_accel_filter_bandwidth()

Get the current accelerometer filter bandwidth as one of the enumerated values
of type mpu9250_accel_bandwidth_t

    mpu9250_accel_bandwidth_t MPU9250::get_accel_filter_bandwidth(void);

| Argument | Description |
|----------|-------------|
| None     |             |

| Return Value | Description |
|--------------|-------------|
|              | The accelerometer bandwidth setting |

### set_accel_filter_bandwidth()
Program the accelerometer filter bandwidth as one of the enumerated values of
type mpu9250_accel_bandwidth_t

    void MPU9250::set_accel_filter_bandwidth(mpu9250_accel_bandwidth_t bandwidth);

| Argument | Description |
|----------|-------------|
| bandwidth | new bandwidth foer the accelerometer |

| Return Value | Description |
|--------------|-------------|
| Nothing      |             |

### mpu9250_gyro_bandwidth_t
    typedef enum {
        MPU9250_GYRO_BAND_250_HZ,    
        MPU9250_GYRO_BAND_184_HZ,
        MPU9250_GYRO_BAND_92_HZ,
        MPU9250_GYRO_BAND_41_HZ,
        MPU9250_GYRO_BAND_20_HZ,
        MPU9250_GYRO_BAND_10_HZ,
        MPU9250_GYRO_BAND_5_HZ,
        MPU9250_GYRO_BAND_3600_HZ,
        MPU9250_GYRO_BAND_8800_HZ
    } mpu9250_gyro_bandwidth_t;

### get_gyro_filter_bandwidth()

Get the current gyro filter bandwidth as one of the enumerated values of type
mpu9250_gyro_bandwidth_t

    mpu9250_gyro_bandwidth_t MPU9250::get_gyro_filter_bandwidth(void);

| Argument | Description |
|----------|-------------|
| None     |             |

| Return Value | Description |
|--------------|-------------|
|              | The gyro bandwidth setting |


### set_gyro_filter_bandwidth()

    MPU9250::set_gyro_filter_bandwidth(mpu9250_gyro_bandwidth_t bandwidth);

| Argument | Description |
|----------|-------------|
| bandwidth | new bandwidth for the gyro |

| Return Value | Description |
|--------------|-------------|
| Nothing      |             |

### ak8963_mag_mode_t
    typedef enum {
        AK8963_MODE_POWER_DOWN,
        AK8963_MODE_SINGLE,
        AK8963_MODE_8HZ,
        AK8963_MODE_EXT_TRIG = 0x4,
        AK8963_MODE_100HZ = 0x6,
        AK8963_MODE_SELF_TEST = 0x8,
        AK8963_MODE_FUSE_ROM = 0xf
    } ak8963_mag_mode_t;

### get_mag_mode()

Get the currently programmed mode of the AK8963 magnetometer as one
of the enumerated values of type ak8963_mag_mode_t

    ak8963_mag_mode_t MPU9250::get_mag_mode(void);

| Argument | Description |
|----------|-------------|
| None     |             |

| Return Value | Description |
|--------------|-------------|
|              | The magnetometer mode |

### set_mag_mode() 

Program the mode of the AK8963 magnetometer.  The mode should be one of the
enumerated values of type ak8963_mag_mode_t.

    void MPU9250::set_mag_mode(ak8963_mag_mode_t mag_mode);

| Argument | Description |
|----------|-------------|
| mag_mode | new mode for the magnetometer |

| Return Value | Description |
|--------------|-------------|
| Nothing      |             |

### ak8963_mag_sensitivity_t

    typedef enum {
        AK8963_SENSITIVITY_14b,
        AK8963_SENSITIVITY_16b
    } ak8963_mag_sensitivity_t;

### get_mag_sensitivity()

Get the currently programmed sensitivity of the AK8963 magnetometer as one
of the enumerated values of type ak8963_mag_mode_t

    ak8963_mag_sensitivity_t MPU9250::get_mag_sensitivity(void);

| Argument | Description |
|----------|-------------|
| None     |             |

| Return Value | Description |
|--------------|-------------|
|              | The magnetometer sensitivity |

### set_mag_sensitivity()

Program the mode of the AK8963 magnetometer.  The mode should be one of the
enumerated values of type ak8963_mag_mode_t

    void MPU9250::set_mag_sensitivity(ak8963_mag_sensitivity_t mag_sensitivity);

| Argument | Description |
|----------|-------------|
| mag_sensitivity | new sensitivity for the magnetometer |

| Return Value | Description |
|--------------|-------------|
| Nothing      |             |

### tuple

    template <typename T>
    struct tuple {
        T x;
        T y;
        T z;
    };

### read()

Read the latest sensor data from the MPU9250 and write it to the user
provided structures.  Any structure pointer which is NULL will be ignored.

    bool MPU9250::read(tuple<float> *accel, tuple<float> *gyro, tuple<float> *mag, float *temperature);

| Argument | Description |
|----------|-------------|
| accel    | User allocated structure to hold the latest values of X, Y, and Z from the accelerometer in units of meters per second squared|
| gyro     | User allocated structure to hold the latest values of X, Y, and Z from the gyro |
| mag      | User allocated structure to hold the latest values of X, Y, and Z from the magnetometer |
| temperature | Pointer to location to hold the latest temperature reading in units of degrees farenheit |

| Return Value | Description |
|--------------|-------------|
| true         | New data was available |
| false        | New data was not available (returned data is unchanged from previous read) |

### dump_regs()

Dump the values of all registers of the MPU9250 and AK8963 to stdout

    void MPU9250::dump_regs(void);

| Argument | Description |
|----------|-------------|
| None     |             |

| Return Value | Description |
|--------------|-------------|
| Nothing      |             |
