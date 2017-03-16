
#include <pios.h>

#include <light_ws2811strip.h>

struct stm32_gpio led = {
    .gpio = GPIOC,
    .init = {
        .GPIO_Pin = GPIO_Pin_13,
        .GPIO_Speed = GPIO_Speed_2MHz,
        .GPIO_Mode = GPIO_Mode_Out_PP
    }
};

void Setup_RCC()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void Setup_GPIO()
{

    
    GPIO_Init(led.gpio, &led.init);
}


void safeSetLedHsv(uint16_t index, const hsvColor_t *color)
{
    if(index < WS2811_LED_STRIP_LENGTH) {
        setLedHsv(index, color);
    }
}

void safeGetLedHsv(uint16_t index, hsvColor_t *color)
{
    if(index < WS2811_LED_STRIP_LENGTH) {
        getLedHsv(index, color);
    } else {
        color->v = 0;
    }
}

void main()
{
    Setup_RCC();
    Setup_GPIO();

    PIOS_DELAY_Init();
    
    ws2811LedStripInit();
    
    hsvColor_t color = {
        .h = 0,
        .s = 255,
        .v = 0,
    };

#define NUM_SPRITES 4

    
    hsvColor_t sprite[NUM_SPRITES] = {
        {
            .h = 0,
            .s = 0,
            .v = 255,
        },
        {
            .h = 120,
            .s = 0,
            .v = 255,
        },
        {
            .h = 240,
            .s = 0,
            .v = 255,
        },
        {
            .h = 60,
            .s = 0,
            .v = 255,
        },
    };

    
    float seg[NUM_SPRITES] = { 0, 20, 50, 80};
    float seg_inc[NUM_SPRITES] = { 0.05, -0.1, 0.02, 0.07 };
    int explosion_i[NUM_SPRITES];
    float explosion_v[NUM_SPRITES];

    bool collision[NUM_SPRITES];
    
    while(1) {
        GPIO_WriteBit(led.gpio, led.init.GPIO_Pin, Bit_SET);
        PIOS_DELAY_WaitmS(1);
        GPIO_WriteBit(led.gpio, led.init.GPIO_Pin, Bit_RESET);
        PIOS_DELAY_WaitmS(1);
        
        ++(color.h);
        if(color.h > 359) {
            color.h = 0;
        }
        
        setStripColor(&color);

        for(int i = 0; i < NUM_SPRITES; ++i) {
            collision[i] = 0;

            for(int j = 0; j < NUM_SPRITES; ++j) {
                if((i != j) && ((int)seg[i] == (int)seg[j])) {
                    // colision
                    collision[i] = 1;
                    explosion_v[i] = 255;
                    explosion_i[i] = seg[i];
                }
            }
        }

        // trails
        for(int i = 0; i < NUM_SPRITES; ++i) {
            hsvColor_t tmp = sprite[i];
            
            int offset_inc = seg_inc[i] < 0 ? 1 : -1;
            int offset = seg[i];
            
            for(int j = 0; j < 10; ++j) {
                tmp.v /= 2;
                safeSetLedHsv(offset, &tmp);
                offset += offset_inc;
            }
        }


        for(int i = 0; i < NUM_SPRITES; ++i) {
            if(explosion_v[i] > 0) {
                explosion_v[i] -= 1;
            }
            
            hsvColor_t tmp;
            
            safeGetLedHsv(explosion_i[i], &tmp);

            uint8_t v = explosion_v[i];
            
            tmp.v = v;
            
            safeSetLedHsv(explosion_i[i], &tmp);
            
            for(int j = 1; j < 7; ++j) {
                safeGetLedHsv(explosion_i[i] - j , &tmp);
                if(v > tmp.v) {
                    tmp.v = v;
                    tmp.s = 255;
                    safeSetLedHsv(explosion_i[i] - j, &tmp);
                }

                safeGetLedHsv(explosion_i[i] - j , &tmp);
                if(v > tmp.v) {
                    tmp.v = v;
                    tmp.s = 255;
                    safeSetLedHsv(explosion_i[i] + j, &tmp);
                }
                
                v /= 2;
            }
        }

        
        for(int i = 0; i < NUM_SPRITES; ++i) {
            setLedHsv(seg[i], &sprite[i]);
        }

        
        for(int i = 0; i < NUM_SPRITES; ++i) {
            if(collision[i]) {
                seg_inc[i] = 0.0 - seg_inc[i];
                seg[i] += (seg_inc[i] < 0) ? -1 : 1;
            }
            
            seg[i] += seg_inc[i];
            
            if(seg[i] >= WS2811_LED_STRIP_LENGTH) {
                seg[i] = WS2811_LED_STRIP_LENGTH - 1;
                seg_inc[i] = 0.0 - seg_inc[i];
            } else if(seg[i] < 0) {
                seg[i] = 0;
                seg_inc[i] = 0.0 - seg_inc[i];
            }
        }

        ws2811UpdateStrip();

    }
}
