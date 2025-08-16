//
// Example use of the MPU9250 orientation sensor.
//
// Copyright 2025  John Fredine
//

#include "MPU9250.h"

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "pico/time.h"

MPU9250 mpu9250;

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 50

typedef struct {
    MPU9250::tuple<float> mag_hard_iron;  // MotionCal Magnetic Offset
    struct {
        MPU9250::tuple<float> x;
        MPU9250::tuple<float> y;
        MPU9250::tuple<float> z;
    } mag_soft_iron;                     // MotionCal Magnetic Mapping
    MPU9250::tuple<float> accel;
    MPU9250::tuple<float> gyro;
} sensor_adj_t;

// plug in the values found in MotionCal here.
sensor_adj_t sensor_adj = {{15.75, 23.12, 33.59},
                           {{  0.951, -0.010,  0.014},
                            { -0.010,  0.961, -0.007},
                            {  0.014, -0.007,  1.094}},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0}};

int main() {
    bi_decl(bi_program_description("AHRS example using the MPU9250"));

    stdio_init_all();

#ifdef MPU9250_SPI
    spi_init(spi_default, 1000000);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_SCK_PIN,
                               PICO_DEFAULT_SPI_RX_PIN,
                               PICO_DEFAULT_SPI_TX_PIN,
                               GPIO_FUNC_SPI));

    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

    bool aux_auto_sample = true;
    int retval = mpu9250.init(spi_default,
                              PICO_DEFAULT_SPI_CSN_PIN, AK8963_I2CADDR_DEFAULT,
                              aux_auto_sample);
#else
    i2c_init(i2c_default, 400000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN,
                               PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    bool i2c_aux_master = true;
    bool aux_auto_sample = true;
    int retval = mpu9250.init(i2c_default,
                              MPU9250_I2CADDR_DEFAULT, AK8963_I2CADDR_DEFAULT,
                              i2c_aux_master, aux_auto_sample);
#endif

    if (retval != 0) {
        if (retval == 1) {
            puts("Failed to locate MPU9250 I2C address");
            while (1) sleep_ms(10);
        } else if (retval == 2) {
            puts("Failed to identify MPU9250");
            while (1) sleep_ms(10);
        } else if (retval == 3) {
            puts("Failed to identify AK8963");
            while (1) sleep_ms(10);
        } else {
            puts("Unknown initialization error");
            while (1) sleep_ms(10);
        }
    }

    uint8_t counter = 0;
    while (1) {

        // Read the motion sensors
        MPU9250::tuple<float> accel, gyro, mag;
        float temperature;
        mpu9250.read(&accel, &gyro, &mag, &temperature);

        float mag_x = mag.x - sensor_adj.mag_hard_iron.x;
        float mag_y = mag.y - sensor_adj.mag_hard_iron.y;
        float mag_z = mag.z - sensor_adj.mag_hard_iron.z;

        mag.x = mag_x * sensor_adj.mag_soft_iron.x.x
                + mag_y * sensor_adj.mag_soft_iron.x.y
                + mag_z * sensor_adj.mag_soft_iron.x.z;

        mag.y = mag_x * sensor_adj.mag_soft_iron.y.x
                + mag_y * sensor_adj.mag_soft_iron.y.y
                + mag_z * sensor_adj.mag_soft_iron.y.z;

        mag.z = mag_x * sensor_adj.mag_soft_iron.z.x
                + mag_y * sensor_adj.mag_soft_iron.z.y
                + mag_z * sensor_adj.mag_soft_iron.z.z;

        gyro.x -= sensor_adj.gyro.x;
        gyro.y -= sensor_adj.gyro.y;
        gyro.z -= sensor_adj.gyro.z;

        accel.x -= sensor_adj.accel.x;
        accel.y -= sensor_adj.accel.y;
        accel.z -= sensor_adj.accel.z;

        // print the output less often to avoid overloading the serial interface
        if (counter++ >= PRINT_EVERY_N_UPDATES) {
            counter = 0;

#ifdef MPU9250_SPI
            printf("aux_auto_sample = %s\n",
                    aux_auto_sample ? "true" : "false");
#else
            printf("i2c_aux_master: %s, aux_auto_sample = %s\n",
                    i2c_aux_master ? "true" : "false",
                    aux_auto_sample ? "true" : "false");
#endif
            printf("Accel: %.2f, %.2f, %.2f\n", accel.x, accel.y, accel.z);
            printf("Mag: %.2f, %.2f, %.2f\n\n", mag.x, mag.y, mag.z);
        }

        sleep_us( 1000000 / FILTER_UPDATE_RATE_HZ);
    }
}
