/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "hal.h"
#include <mooncake_log.h>
#include <bmi270_bmm150.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cmath>

#ifndef BMI2_I2C_PRIM_ADDR
#define BMI2_I2C_PRIM_ADDR 0x68
#endif

static const std::string_view _tag        = "HAL-IMU";
static bmi270_bmm150_handle_t _imu_sensor = NULL;
static SemaphoreHandle_t _imu_mutex      = nullptr;
static bool _imu_ready                   = false;
static uint32_t _last_sensor_time        = 0;
static bool _has_sensor_time             = false;

void Hal::imu_init()
{
    mclog::tagInfo(_tag, "init");

    bmi270_bmm150_config_t sensor_conf = {
        .i2c_addr        = BMI2_I2C_PRIM_ADDR,
        .config_file_ptr = NULL,
        // This driver mode enables both accelerometer and gyroscope, without BMM150.
        .mode = BOSCH_ACCELEROMETER_ONLY,
    };

    if (bmi270_bmm150_sensor_create(_i2c_bus, &_imu_sensor, &sensor_conf) != ESP_OK) {
        mclog::tagError(_tag, "init failed");
        _imu_sensor = NULL;
        return;
    }

    _imu_mutex = xSemaphoreCreateMutex();
    if (_imu_mutex == nullptr) {
        mclog::tagError(_tag, "mutex allocation failed");
        return;
    }

    bmi2_sens_config config[2] = {};
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;
    if (bmi270_get_sensor_config(config, 2, &_imu_sensor->bmi2) != BMI2_OK) {
        mclog::tagError(_tag, "sensor configuration unavailable");
        return;
    }
    config[0].cfg.acc.odr   = BMI2_ACC_ODR_100HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    config[1].cfg.gyr.odr   = BMI2_GYR_ODR_100HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    const uint8_t sensors[2] = {BMI2_ACCEL, BMI2_GYRO};
    if (bmi270_set_sensor_config(config, 2, &_imu_sensor->bmi2) != BMI2_OK ||
        bmi270_sensor_enable(sensors, 2, &_imu_sensor->bmi2) != BMI2_OK) {
        mclog::tagError(_tag, "sensor configuration failed");
        return;
    }
    _imu_ready = true;
}

bool Hal::isImuReady() const
{
    return _imu_ready;
}

Hal::ImuSampleResult Hal::readImuSample(ImuSample& sample)
{
    sample = {};
    if (!_imu_ready || xSemaphoreTake(_imu_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return ImuSampleResult::Error;
    }
    uint8_t status = 0;
    if (bmi2_get_regs(BMI2_STATUS_ADDR, &status, 1, &_imu_sensor->bmi2) != BMI2_OK) {
        xSemaphoreGive(_imu_mutex);
        return ImuSampleResult::Error;
    }
    constexpr uint8_t ready_mask = BMI2_DRDY_ACC_MASK | BMI2_DRDY_GYR_MASK;
    if ((status & ready_mask) != ready_mask) {
        xSemaphoreGive(_imu_mutex);
        return ImuSampleResult::NotReady;
    }
    bmi2_sens_data data = {};
    const int8_t result = bmi2_get_sensor_data(&data, &_imu_sensor->bmi2);
    const uint64_t timestamp = static_cast<uint64_t>(esp_timer_get_time());
    const bool repeated = _has_sensor_time && data.sens_time == _last_sensor_time;
    if (result == BMI2_OK && !repeated) {
        _last_sensor_time = data.sens_time;
        _has_sensor_time  = true;
    }
    xSemaphoreGive(_imu_mutex);
    if (result != BMI2_OK) {
        return ImuSampleResult::Error;
    }
    if (repeated) {
        return ImuSampleResult::NotReady;
    }
    // Keep the sensor's native right-handed axes for quaternion fusion.
    sample.data.accelX = static_cast<float>(data.acc.x) / 8192.0f;
    sample.data.accelY = static_cast<float>(data.acc.y) / 8192.0f;
    sample.data.accelZ = static_cast<float>(data.acc.z) / 8192.0f;
    sample.data.gyroX  = static_cast<float>(data.gyr.x) / 16.384f;
    sample.data.gyroY  = static_cast<float>(data.gyr.y) / 16.384f;
    sample.data.gyroZ  = static_cast<float>(data.gyr.z) / 16.384f;
    sample.timestampUs = timestamp;
    if (!std::isfinite(sample.data.accelX) || !std::isfinite(sample.data.accelY) ||
        !std::isfinite(sample.data.accelZ) || !std::isfinite(sample.data.gyroX) ||
        !std::isfinite(sample.data.gyroY) || !std::isfinite(sample.data.gyroZ)) {
        sample = {};
        return ImuSampleResult::Error;
    }
    return ImuSampleResult::Ready;
}

void Hal::updateImuData()
{
    ImuSample sample;
    if (readImuSample(sample) == ImuSampleResult::Ready) {
        // Preserve the legacy IMU view's X/Y mapping.
        _imu_data = sample.data;
        _imu_data.accelX = sample.data.accelY;
        _imu_data.accelY = sample.data.accelX;
        _imu_data.gyroX  = sample.data.gyroY;
        _imu_data.gyroY  = sample.data.gyroX;
    }
}
