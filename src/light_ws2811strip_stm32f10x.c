/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <pios.h>

#ifdef LED_STRIP

#include "color.h"
#include "light_ws2811strip.h"

struct pios_tim_channel {
    TIM_TypeDef *timer;
    uint8_t timer_chan;
    
    struct stm32_gpio pin;
    uint32_t    remap;

    
    uint16_t timer_dma_source;

    DMA_Channel_TypeDef *dma_chan;
    
    uint32_t dma_tcif;
    
    uint8_t dma_irqn;
    
};

#define _CONCAT5(a,b,c,d,e) a##b##c##d##e
#define _EVAL5(a,b,c,d,e) _CONCAT5(a,b,c,d,e)

#define _CONCAT4(a,b,c,d) a##b##c##d
#define _EVAL4(a,b,c,d) _CONCAT4(a,b,c,d)


#define TIM_SERVO_CHANNEL_CONFIG(_timer, _channel, _gpio, _pin) \
{                                                     \
    .timer = _timer,                                  \
    .timer_chan = TIM_Channel_##_channel,             \
    .pin   = {                                        \
        .gpio = GPIO##_gpio,                          \
        .init = {                                     \
            .GPIO_Pin   = GPIO_Pin_##_pin,            \
            .GPIO_Speed = GPIO_Speed_50MHz,            \
            .GPIO_Mode  = GPIO_Mode_AF_PP,               \
        },                                            \
        .pin_source     = GPIO_PinSource##_pin,       \
    },                                                \
    .remap = GPIO_REMAP_##_timer##_CH##_channel##_GPIO##_gpio, \
    .timer_dma_source = TIM_DMA_CC##_channel, \
    .dma_chan = _EVAL4(DMA, _timer##_CH##_channel##_DMA_INSTANCE, _Channel, _timer##_CH##_channel##_DMA_CHANNEL), \
    .dma_tcif = _EVAL4(DMA, _timer##_CH##_channel##_DMA_INSTANCE, _IT_TC,  _timer##_CH##_channel##_DMA_CHANNEL), \
    .dma_irqn = _EVAL5(DMA, _timer##_CH##_channel##_DMA_INSTANCE, _Channel,  _timer##_CH##_channel##_DMA_CHANNEL, _IRQn), \
}

#define DMA0_Channel0 0
#define DMA0_IT_TC0 0
#define DMA0_Channel0_IRQn 0

#define GPIO_REMAP_TIM1_CH1_GPIOA 0

#define TIM1_CH1_DMA_INSTANCE 1
#define TIM1_CH1_DMA_CHANNEL  2

//#define TIM1_CH1_DMA_INSTANCE 0
//#define TIM1_CH1_DMA_CHANNEL  0

static void WS2811_IRQHandler(uintptr_t context);

struct pios_tim_channel ws2811_config = TIM_SERVO_CHANNEL_CONFIG(TIM1, 1, A, 8);

bool ws2811Initialised;

typedef bool (* pios_dma_irqhandler_t)(uint32_t dma_id, uintptr_t context);

typedef struct pios_dma {
    uint32_t it_tc_flag;
    uint32_t it_ht_flag;
    uint32_t it_er_flag;

    pios_dma_irqhandler_t tc_irq_handler;
    uintptr_t tc_irq_context;
    
    pios_dma_irqhandler_t ht_irq_handler;
    uintptr_t ht_irq_context;
    
    pios_dma_irqhandler_t er_irq_handler;
    uintptr_t er_irq_context;

} pios_dma_t;


typedef struct pios_dma_request {
    struct pios_dma_request *next;
    DMA_InitTypeDef init;
} pios_dma_request_t;

static pios_dma_t *dma1_channel1; /* owner */
static pios_dma_t *dma1_channel2; /* owner */
static pios_dma_t *dma1_channel3; /* owner */
static pios_dma_t *dma1_channel4; /* owner */
static pios_dma_t *dma1_channel5; /* owner */
static pios_dma_t *dma1_channel6; /* owner */
static pios_dma_t *dma1_channel7; /* owner */



void PIOS_DMA_IRQHandler(pios_dma_t *dma)
{
    // check IT status
    //if( DMA_GetITStatus(dma->))
    
    WS2811_IRQHandler(0);
    // clear IT pending
}

void DMA1_Channel1_IRQHandler()
{
    PIOS_DMA_IRQHandler(dma1_channel1);
}
void DMA1_Channel2_IRQHandler()
{
    PIOS_DMA_IRQHandler(dma1_channel2);
}
void DMA1_Channel3_IRQHandler()
{
    PIOS_DMA_IRQHandler(dma1_channel3);
}
void DMA1_Channel4_IRQHandler()
{
    PIOS_DMA_IRQHandler(dma1_channel4);
}
void DMA1_Channel5_IRQHandler()
{
    PIOS_DMA_IRQHandler(dma1_channel5);
}
void DMA1_Channel6_IRQHandler()
{
    PIOS_DMA_IRQHandler(dma1_channel6);
}
void DMA1_Channel7_IRQHandler()
{
    PIOS_DMA_IRQHandler(dma1_channel7);
}



static void WS2811_IRQHandler(uintptr_t context)
{
    if( DMA_GetITStatus(ws2811_config.dma_tcif) ) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(ws2811_config.dma_chan, DISABLE);
        
        DMA_ClearITPendingBit(ws2811_config.dma_tcif);
    }
}

extern uint32_t SystemFrequency;

void ws2811LedStripHardwareInit()
{
    if(!ws2811_config.dma_chan) {
        return;
    }

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    GPIO_Init(ws2811_config.pin.gpio, &ws2811_config.pin.init);
    if(ws2811_config.remap) {
        GPIO_PinRemapConfig(ws2811_config.remap, ENABLE);
    }
    
    /* Compute the prescaler value */
    uint16_t prescalerValue = (uint16_t) (SystemFrequency / WS2811_TIMER_HZ) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = WS2811_TIMER_PERIOD; // 800kHz
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ws2811_config.timer, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    switch(ws2811_config.timer_chan) {
        case TIM_Channel_1:
            TIM_OC1Init(ws2811_config.timer, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(ws2811_config.timer, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(ws2811_config.timer, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(ws2811_config.timer, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(ws2811_config.timer, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(ws2811_config.timer, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(ws2811_config.timer, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(ws2811_config.timer, TIM_OCPreload_Enable);
            break;
    }
    

    TIM_CtrlPWMOutputs(ws2811_config.timer, ENABLE);

    /* configure DMA */
    // NVIC setup here
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQChannel = ws2811_config.dma_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_LOW;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    
    DMA_DeInit(ws2811_config.dma_chan);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ws2811_config.timer->CCR1 + ws2811_config.timer_chan;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = WS2811_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(ws2811_config.dma_chan, &DMA_InitStructure);

    /* CC1 DMA Request enable */
    TIM_DMACmd(ws2811_config.timer, ws2811_config.timer_dma_source, ENABLE);

    DMA_ITConfig(ws2811_config.dma_chan, DMA_IT_TC, ENABLE);

    ws2811Initialised = 1;
}

void ws2811LedStripDMAEnable(void)
{
    if (!ws2811Initialised)
        return;

    DMA_SetCurrDataCounter(ws2811_config.dma_chan, WS2811_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(ws2811_config.timer, 0);
    TIM_Cmd(ws2811_config.timer, ENABLE);

    
    
    DMA_Cmd(ws2811_config.dma_chan, ENABLE);
}

#endif
