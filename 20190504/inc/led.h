#ifndef _led_H
#define _led_H

#include "headfile.h"
#include "common.h"

//定义模块号
typedef enum
{
    LED0=0,
    LED1=1,
    LED2=2,
    LED3=3,
    LED4=4,
    LED5=5,
    LEDALL=6,//全部四个   
} LEDn_e;

typedef enum
{
    ON=0,  //亮
    OFF=1, //灭
    RVS=2, //反转  
}LEDs_e;

void LED_init(void);
void LED_Ctrl(LEDn_e ledno, LEDs_e sta);

#endif

