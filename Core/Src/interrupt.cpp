#include "gpio.h"
#include "lsm6dsr.hpp"

extern LSM6DSR imu;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    imu.gpio_interrupt_handler(GPIO_Pin);
}