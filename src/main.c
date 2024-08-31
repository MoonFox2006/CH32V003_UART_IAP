/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/11/21
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
/*
 *@Note
IAP upgrade routine:
Support serial port for FLASH burning

1. Use the IAP download tool to realize the download PA0 floating (default pull-up input)
2. After downloading the APP, connect PA2 to ground (low level input), and press the
reset button to run the APP program.
3. use WCH-LinkUtility.exe download to BOOT(adr-0x1FFFF000)

*/

#include "debug.h"
#include "string.h"
#include "iap.h"

void IAP_2_APP(void) {
    RCC->APB2PCENR &= ~(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1);
    RCC_ClearFlag();
    SystemReset_StartMode(Start_Mode_USER);
    NVIC_SystemReset();
}

int main(void) {
//    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1;

    if(GPIO_Check()) { // PD7 pressed for 500 ms.
        USART1_Cfg();

        while(1) {
            if(! (USART1->STATR & USART_STATR_RXNE)) { // ?
                if (UART_Rx_Deal())
                    break;
            }
        }
    }

    IAP_2_APP();
    while(1) {}
}
