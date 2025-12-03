#include "cpp_entry.hpp"
#include "main.h"
#include "lsm6dsr.hpp"
#include "gpio.h"
#include "tim.h"

LSM6DSR imu(&hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin);

void setup() {
    HAL_GPIO_WritePin(USB_PLUG_GPIO_Port, USB_PLUG_Pin, GPIO_PIN_SET);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    // Power UP sensor
    // HAL_Delay(3000);
    // HAL_GPIO_WritePin(SENSOR_PWR_EN_GPIO_Port, SENSOR_PWR_EN_Pin, GPIO_PIN_SET);
    // HAL_Delay(1000);
    // HAL_GPIO_WritePin(SENSOR_PWR_EN_GPIO_Port, SENSOR_PWR_EN_Pin, GPIO_PIN_RESET);
    // HAL_Delay(100);
    // imu
    volatile LSM6DSR::Status status = imu.init();
    if (status == LSM6DSR::Status::OK) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }else {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }
    imu.set_data_rate(LSM6DSR::Dev::ACCEL, LSM6DSR::DataRate::D208HZ);
    imu.set_data_rate(LSM6DSR::Dev::GYRO, LSM6DSR::DataRate::D12_5HZ);
    imu.set_scale(LSM6DSR::Dev::ACCEL, LSM6DSR::Scale::SCALE_4G_1000DPS);
    imu.set_scale(LSM6DSR::Dev::GYRO, LSM6DSR::Scale::SCALE_16G_500DPS);
    HAL_Delay(100);
}


double imu_data[7] = {0};

void loop() {
    volatile LSM6DSR::Status status;
    while (true) {
        status = imu.read(imu_data);
        HAL_Delay(100);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 3000);
    }
}