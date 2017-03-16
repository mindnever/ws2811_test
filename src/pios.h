#ifndef _PIOS_H_
#define _PIOS_H_

#include "stm32f10x_conf.h"
#include <stdint.h>
#include <pios_delay.h>

#define PIOS_DEBUG_Assert(x) assert_param(x)

#define PIOS_IRQ_PRIO_LOW        12              // lower than RTOS
#define PIOS_IRQ_PRIO_MID        8               // higher than RTOS
#define PIOS_IRQ_PRIO_HIGH       5               // for SPI, ADC, I2C etc...
#define PIOS_IRQ_PRIO_HIGHEST    4               // for USART etc...

struct stm32_gpio {
    GPIO_TypeDef     *gpio;
    GPIO_InitTypeDef init;
    uint8_t pin_source;
};

#endif /* _PIOS_H_ */
