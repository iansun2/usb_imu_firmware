#ifndef __LSM6DSR_HPP__
#define __LSM6DSR_HPP__

#include "spi.h"
#include "gpio.h"


class LSM6DSR{
public:
    enum class Status{
        OK,
        ERROR,
        TIMEOUT,
        FREE,
        UNDEFINE,
    };

    enum class Dev{
        ACCEL,
        GYRO,
    };

    enum class DataRate{
        PWR_DOWN    = 0,
        D12_5HZ,
        D26HZ,
        D52HZ,
        D104HZ,
        D208HZ,
        D416HZ,
        D833HZ,
        D1666HZ,
        D3333HZ,
        D6666HZ
    };

    enum class Scale{
        SCALE_2G_250DPS     = 0,
        SCALE_16G_500DPS,
        SCALE_4G_1000DPS,
        SCALE_8G_2000DPS
    };

    LSM6DSR(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
    ~LSM6DSR();

    Status init();

    Status set_data_rate(Dev dev, DataRate data_rate);

    Status set_scale(Dev dev, Scale scale);

    Status read(double data[7], bool blocking = true);

    Status set_read_trigger_interrupt(Dev sync, uint16_t pin);

    Status gpio_interrupt_handler(uint16_t pin);
    

    enum class Reg{
        PIN_CTRL      = 0x02,
        FIFO_CTRL1    = 0x07,
        FIFO_CTRL2    = 0x08,
        FIFO_CTRL3    = 0x09,
        FIFO_CTRL4    = 0x0A,
        INT1_CTRL     = 0x0D,
        INT2_CTRL     = 0x0E,
        WHO_AM_I      = 0x0F,
        CTRL1_XL      = 0x10,
        CTRL2_G       = 0x11,
        CTRL3_C       = 0x12,
        CTRL4_C       = 0x13,
        CTRL5_C       = 0x14,
        CTRL6_C       = 0x15,
        CTRL7_G       = 0x16,
        CTRL8_XL      = 0x17,
        CTRL9_XL      = 0x18,
        CTRL10_C      = 0x19,
        STATUS_REG    = 0x1E,
        OUT_TEMP_L    = 0x20,
        OUT_TEMP_H    = 0x21,
        OUTX_L_G      = 0x22,
        OUTX_H_G      = 0x23,
        OUTY_L_G      = 0x24,
        OUTY_H_G      = 0x25,
        OUTZ_L_G      = 0x26,
        OUTZ_H_G      = 0x27,
        OUTX_L_XL     = 0x28,
        OUTX_H_XL     = 0x29,
        OUTY_L_XL     = 0x2A,
        OUTY_H_XL     = 0x2B,
        OUTZ_L_XL     = 0x2C,
        OUTZ_H_XL     = 0x2D,
        FIFO_STATUS1  = 0x3A,
        FIFO_STATUS2  = 0x3B,
    };

private:
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin, int_pin;
    Scale current_accel_scale, current_gyro_scale;
    volatile bool new_data_avail; 
    uint32_t last_read_time;

    HAL_StatusTypeDef write_reg(Reg reg, uint8_t data, uint8_t mask, bool check);
    HAL_StatusTypeDef read_reg(Reg reg, uint8_t data[1]);
    HAL_StatusTypeDef read_regs(Reg reg, uint8_t *data, size_t size);

    Status status_from_hal(HAL_StatusTypeDef status);
    void parse_data(uint8_t input[14], double output[7]);
};





#endif // __LSM6DSR_HPP