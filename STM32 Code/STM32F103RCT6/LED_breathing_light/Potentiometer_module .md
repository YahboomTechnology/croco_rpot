# Potentiometer module application: breathing lamp

## Hardware wiring

![1](..\LED_breathing_light\1.jpg)

| Potentiometer module | STM32F103RCT6 |
| :------------------: | :-----------: |
|         VCC          |    5V/3.3V    |
|         OUT          |      PB4      |
|          NC          |               |
|         GND          |      GND      |

## Brief principle

Potentiometer (adjustable resistor) has three lead terminals and a resistance element whose resistance value can be adjusted according to a certain law of change;

It is usually composed of a resistor and a rotating or sliding system, that is, a moving contact moves on the resistor to obtain part of the voltage output;

According to the different voltages output of the rotary potentiometer, the degree of LED lighting can be controlled to achieve the effect of breathing lamps.

### main.c

```
#include "stm32f10x.h"
#include "LED.h"

int main(void)
{
    LED_Init();//LED初始化(PB4)
    
    while(1)
    {
    }
}
```

### LED.c

```
#include "LED.h"

void LED_Init(void)//LED初始化(PB4)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Enable GPIOB and AFIO clocks */
    /* 使能GPIOB和功能复用IO时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 
    
    /* JTAG-DP Disabled and SW-DP Enabled */
    /* 禁用JTAG 启用SWD */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    /* Configure PB4 in intput floating mode */
    /* 配置PB4 浮空输入模式 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
```

### LED.h

```
#ifndef __LED_H__
#define __LED_H__

#include "stm32f10x.h"

void LED_Init(void);//LED初始化(PB4)

#endif
```

## Phenomenon

After downloading the program, press the Reset key once, and the downloaded program will run.

At this time, slowly rotate the potentiometer, and the on-board LED light will change the degree of on-board light on and off.