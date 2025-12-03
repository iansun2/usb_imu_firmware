#include "lsm6dsr.hpp"
#include <cstring>

#define BIT(x) (1 << (x))
#define CHECK_ERR_HAL(x) if((x) != HAL_OK) return status_from_hal(x)
#define CHECK_ERR(x) do{Status _ret = (x); if((x) != Status::OK) return _ret;} while(0)
#define MDEG_TO_RAD (0.0000174532925199)
#define MG_TO_MS2 (0.00980665)


LSM6DSR::LSM6DSR(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin):
    hspi(hspi),
    cs_port(cs_port),
    cs_pin(cs_pin),
    int_pin(GPIO_PIN_All),
    current_accel_scale(Scale::SCALE_2G_250DPS),
    current_gyro_scale(Scale::SCALE_8G_2000DPS),    // 4000DPS default, but set to 2000 in init 
    new_data_avail(false),
    last_read_time(0)
{
    
}

LSM6DSR::~LSM6DSR(){

}


/**
 * @brief Initialize LSM6DSR
 * 
 * @return LSM6DSR::Status 
 * Check ID and Reboot. Then write default setting
 */
LSM6DSR::Status LSM6DSR::init() {
    HAL_StatusTypeDef ret;
    // Check WHO_AM_I
    uint8_t who_am_i = 0;
    CHECK_ERR_HAL(read_reg(Reg::WHO_AM_I, &who_am_i));
    if (who_am_i != 0x6B) {
        return Status::ERROR;
    }
    // Reboot
    CHECK_ERR_HAL(write_reg(Reg::CTRL3_C, BIT(0), BIT(0), false));
    HAL_Delay(100);
    // Set LPF1_SEL_G
    CHECK_ERR_HAL(write_reg(Reg::CTRL4_C, BIT(1), BIT(1), true));
    // Set HP_EN_G
    CHECK_ERR_HAL(write_reg(Reg::CTRL7_G, BIT(7) | BIT(6), BIT(7) | BIT(6), true));
    // Set Gyro Scale
    CHECK_ERR(set_scale(Dev::GYRO, Scale::SCALE_8G_2000DPS));
    return Status::OK;
}


/**
 * @brief Set data rate
 * 
 * @param dev       accel or gyro
 * @param data_rate ODR data rate
 * @return LSM6DSR::Status 
 */
LSM6DSR::Status LSM6DSR::set_data_rate(LSM6DSR::Dev dev, LSM6DSR::DataRate data_rate) {
    uint8_t data = static_cast<uint8_t>(data_rate) << 4;
    if (dev == Dev::ACCEL) {
        CHECK_ERR_HAL(write_reg(Reg::CTRL1_XL, data, 0xF0, true));
    } else if (dev == Dev::GYRO) {
        CHECK_ERR_HAL(write_reg(Reg::CTRL2_G, data, 0xF0, true));
    }
    return Status::OK;
}


/**
 * @brief Set scale
 * 
 * @param dev   accel or gyro
 * @param scale scale
 * @return LSM6DSR::Status 
 */
LSM6DSR::Status LSM6DSR::set_scale(LSM6DSR::Dev dev, LSM6DSR::Scale scale) {
    uint8_t data = static_cast<uint8_t>(scale) << 2;
    if (dev == Dev::ACCEL) {
        CHECK_ERR_HAL(write_reg(Reg::CTRL1_XL, data, BIT(2) | BIT(3), true));
        uint8_t test = 0;
        read_reg(Reg::CTRL1_XL, &test);
        current_accel_scale = scale;
    } else if (dev == Dev::GYRO) {
        CHECK_ERR_HAL(write_reg(Reg::CTRL2_G, data, 0x0F, true));
        current_gyro_scale = scale;
    }
    return Status::OK;
}


/**
 * @brief Read sensor data
 * 
 * @param data      output {temp deg C, gyro rad/s[3], accel m/s[3]}
 * @param blocking  false will only read when new data available
 * @return LSM6DSR::Status 
 */
LSM6DSR::Status LSM6DSR::read(double data[7], bool blocking) {
    // blocking / new data / timeout
    if (blocking || new_data_avail || last_read_time - HAL_GetTick() >= 30) {
        last_read_time = HAL_GetTick();
        uint8_t raw[14];
        CHECK_ERR_HAL(read_regs(Reg::OUT_TEMP_L, raw, 14));
        parse_data(raw, data);
        new_data_avail = false;
        return Status::OK;
    }
    // nothing to do when non-blocking and flag not set
    HAL_Delay(1);
    return Status::FREE;
}


/**
 * @brief Setup interrupt
 * 
 * @param sync  sync data source in imu 
 * @param pin   interrupt EXTI pin for imu INT1
 * @return LSM6DSR::Status 
 */
LSM6DSR::Status LSM6DSR::set_read_trigger_interrupt(Dev sync, uint16_t pin) {
    uint8_t data = 0;
    if (sync == Dev::ACCEL) {
        data = BIT(0);
    }else {
        data = BIT(1);
    }
    CHECK_ERR_HAL(write_reg(Reg::INT1_CTRL, data, 0, true));
    int_pin = pin;
    // force read to clear INT status of imu
    double garbage[7];
    read(garbage);
    return Status::OK;
}


/**
 * @brief Handler gpio EXTI interrupt
 * 
 * @param pin interrupted pin
 * @return LSM6DSR::Status 
 */
LSM6DSR::Status LSM6DSR::gpio_interrupt_handler(uint16_t pin) {
    if (pin == int_pin) {
        new_data_avail = true;
        return Status::OK;
    }
    return Status::FREE;
}




/**
 * @brief Write register
 * 
 * @param reg   register
 * @param data  data to write
 * @param mask  mask for write, 0 when not use
 * @param check check data after write
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef LSM6DSR::write_reg(Reg reg, uint8_t data, uint8_t mask, bool check) {
    HAL_StatusTypeDef ret;
    // Prepare buffer data
    uint8_t tx_data[2] = {static_cast<uint8_t>(reg), data};
    // Mask available
    if (mask != 0) {
        read_reg(reg, tx_data + 1);
        tx_data[1] &= ~mask;
        tx_data[1] |= (data & mask);
    }
    // Transmit addr and data
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    ret = HAL_SPI_Transmit(hspi, tx_data, 2, 10);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    // Transmit failed
    if (ret != HAL_OK) {
        return ret;
    }
    // Check
    if (check) { 
        uint8_t read_buffer = 0;
        read_reg(reg, &read_buffer);
        if (read_buffer != tx_data[1]) {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}


/**
 * @brief Read register
 * 
 * @param reg   register
 * @param data  data output
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef LSM6DSR::read_reg(Reg reg, uint8_t data[1]) {
    return read_regs(reg, data, 1);
}


/**
 * @brief Read multiple register
 * 
 * @param reg   register
 * @param data  data output
 * @param size  read data size
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef LSM6DSR::read_regs(Reg reg, uint8_t *data, size_t size) {
    HAL_StatusTypeDef ret;
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    // Prepare tx data
    uint8_t tx_data = static_cast<uint8_t>(reg) | 0x80; // Set read flag
    // Transmit addr
    ret = HAL_SPI_Transmit(hspi, &tx_data, 1, 10);
    // Transmit failed
    if (ret != HAL_OK) {
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
        return ret;
    }
    // Receive data
    ret = HAL_SPI_Receive(hspi, data, size, 10);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return ret;
}


/**
 * @brief Convert status from HAL status
 * 
 * @param status HAL status
 * @return LSM6DSR::Status 
 */
LSM6DSR::Status LSM6DSR::status_from_hal(HAL_StatusTypeDef status) {
    switch(status) {
        case HAL_OK:
            return Status::OK;
        case HAL_ERROR:
            return Status::ERROR;
        case HAL_TIMEOUT:
            return Status::TIMEOUT;
        default:
            return Status::UNDEFINE;
    }
}


/**
 * @brief Parse data from raw
 * 
 * @param input     raw data in [Temp, Gyro XYZ, Accel XYZ], byte order [L, H]
 * @param output    output data in [Temp degC, Gyro XYZ rad/s, Accel XYZ m/s]
 */
void LSM6DSR::parse_data(uint8_t input[14], double output[7]) {
    // Temperature
    int16_t *temp = reinterpret_cast<int16_t*>(input + 0);
    output[0] = *temp * (1.0 / 256) + 25;
    // Gyro
    double factor = 0.0;
    switch(current_gyro_scale) {
        case Scale::SCALE_2G_250DPS:
            factor = 8.75 * MDEG_TO_RAD;
            break;
        case Scale::SCALE_16G_500DPS:
            factor = 17.5 * MDEG_TO_RAD;
            break;
        case Scale::SCALE_4G_1000DPS:
            factor = 35 * MDEG_TO_RAD;
            break;
        case Scale::SCALE_8G_2000DPS:
            factor = 70 * MDEG_TO_RAD;
            break;
        default:
            break;
    }
    int16_t *gyro = reinterpret_cast<int16_t*>(input + 2);
    output[1] = gyro[0] * factor;
    output[2] = gyro[1] * factor;
    output[3] = gyro[2] * factor;
    // Accel
    factor = 0.0;
    switch(current_accel_scale) {
        case Scale::SCALE_2G_250DPS:
            factor = 0.061 * MG_TO_MS2;
            break;
        case Scale::SCALE_4G_1000DPS:
            factor = 0.122 * MG_TO_MS2;
            break;
        case Scale::SCALE_8G_2000DPS:
            factor = 0.244 * MG_TO_MS2;
            break;
        case Scale::SCALE_16G_500DPS:
            factor = 0.488 * MG_TO_MS2;
            break;
        default:
            break;
    }
    int16_t *accel = reinterpret_cast<int16_t*>(input + 8);
    output[4] = accel[0] * factor;
    output[5] = accel[1] * factor;
    output[6] = accel[2] * factor;
}