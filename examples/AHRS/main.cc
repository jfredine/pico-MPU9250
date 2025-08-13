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
#include "pico/time.h"

MPU9250 mpu9250;

// Full orientation sensing using Madgwick/Mahony
//
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface


#define MAHONY
#ifdef MAHONY
#include "Mahony.h"
Mahony filter;    // faster but less accurate
#else
#include "Madgwick.h"
Madgwick filter;  // slower but more accurate
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

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

    i2c_init(i2c_default, 400000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN,
                               PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    int retval = mpu9250.init();
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

        // Update the AHRS fusion filter
        filter.update(gyro.x, gyro.y, gyro.z,
                      accel.x, accel.y, accel.z,
                      mag.x, mag.y, mag.z,
                      1.0 / FILTER_UPDATE_RATE_HZ);

        // print the output less often to avoid overloading the serial interface
        if (counter++ >= PRINT_EVERY_N_UPDATES) {
            counter = 0;

            // print the orientation in euler angles and quaternions
            float roll = filter.get_roll();
            float pitch = filter.get_pitch();
            float heading = filter.get_yaw();
            printf("Orientation: %.2f, %.2f, %.2f\n", heading, pitch, roll);

            float qw, qx, qy, qz;
            filter.get_quaternion(&qw, &qx, &qy, &qz);
            printf("Quaternion: %.4f, %.4f, %.4f, %.4f\n", qw, qx, qy, qz);
        }

        sleep_us( 1000000 / FILTER_UPDATE_RATE_HZ);
    }
}
