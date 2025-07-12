//
// Example use of the MPU9250 orientation sensor.
// This is for doing calibration of the sensor using motion sensor calibration
// tool at https://github.com/PaulStoffregen/MotionCal.  See the write up
// from Adafruit at https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/magnetic-calibration-with-motioncal
// or PJRC at https://www.pjrc.com/store/prop_shield.html.  Attempting AHRS
// without proper magnetic compensation will yield poor results.
//
// Copyright 2025  John Fredine
//


#include <stdio.h>
#include <stdint.h>
#include <pico/stdio.h>
#include <pico/binary_info.h>
#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <MPU9250.h>

MPU9250 mpu9250;

int main() {
    sleep_ms(1000);
    bi_decl(bi_program_description("MPU9250 sensor calibration example"));

    stdio_init_all();

    i2c_init(i2c_default, 100000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN,
                               PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    int retval = mpu9250.init();

    // scan the bus
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        int ret;
        if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78) {
            // Skip over any reserved addresses.
            ret = -1;
        } else {
            uint8_t rxdata;
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);
        }

        putchar(ret < 0 ? '.' : '@');
        printf(((addr % 16) == 15) ? "\n" : "  ");
    }

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

    MPU9250::tuple<float> accel, gyro, mag;
    float temperature;

    while(1) {
        mpu9250.read(&accel, &gyro, &mag, &temperature);

        printf("Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
               static_cast<int>(accel.x * 8192 / 9.8),
               static_cast<int>(accel.y * 8192 / 9.8),
               static_cast<int>(accel.z * 8192 / 9.8),
               static_cast<int>(gyro.x * MPU9250_RPS_TO_DPS * 16),
               static_cast<int>(gyro.y * MPU9250_RPS_TO_DPS * 16),
               static_cast<int>(gyro.z * MPU9250_RPS_TO_DPS * 16),
               static_cast<int>(mag.x * 10),
               static_cast<int>(mag.y * 10),
               static_cast<int>(mag.z * 10));

        sleep_ms(25);
    }
}
