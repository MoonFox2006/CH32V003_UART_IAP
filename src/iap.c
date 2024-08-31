/********************************** (C) COPYRIGHT  *******************************
 * File Name          : iap.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/11/21
 * Description        : IAP
 *******************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "iap.h"
#include "string.h"
#include "flash.h"
#include "core_riscv.h"

/******************************************************************************/

#define CHECK_DUR   500 // 500 ms.

#define FLASH_Base      0x08000000
#define USBD_DATA_SIZE  64

u32 Program_addr = FLASH_Base;
u32 Verity_addr = FLASH_Base;
u8 Verity_Star_flag = 0;
u8 Fast_Program_Buf[128];
u16 CodeLen = 0;
u8 End_Flag = 0;
u8 EP2_Rx_Buffer[USBD_DATA_SIZE];
#define isp_cmd_t   ((isp_cmd*)EP2_Rx_Buffer)

/*********************************************************************
 * @fn      USART1_Cfg
 *
 * @brief   GPIOD-USART1 init
 *
 * @return
 */
void USART1_Cfg() {
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1;

    GPIOD->CFGLR = 0x44944444;

    USART1->BRR = 0xD0; /* Set 115200 baudrate */
    USART1->CTLR2 = 0;
    USART1->CTLR3 = 0;
    USART1->CTLR1 = USART_CTLR1_UE | USART_CTLR1_TE | USART_CTLR1_RE;
}

/*********************************************************************
 * @fn      RecData_Deal
 *
 * @brief   UART-USB
 *
 * @return  ERR_ERROR - ERROR
 *          ERR_SCUESS - SCUESS
 *          ERR_End - End
 */
u8 RecData_Deal(void) {
    u8 i, s, Lenth;

    Lenth = isp_cmd_t->Len;
    switch (isp_cmd_t->Cmd) {
        case CMD_IAP_ERASE:
            FLASH_Unlock_Fast();
            FLASH_EraseAllPages();
            s = ERR_SCUESS;
            break;
        case CMD_IAP_PROM:
            for (i = 0; i < Lenth; i++) {
                Fast_Program_Buf[CodeLen + i] = isp_cmd_t->data[i];
            }
            CodeLen += Lenth;
            if (CodeLen >= 64) {
                CH32_IAP_Program(Program_addr, (u32*) Fast_Program_Buf);
                CodeLen -= 64;
                for (i = 0; i < CodeLen; i++) {
                    Fast_Program_Buf[i] = Fast_Program_Buf[64 + i];
                }
                Program_addr += 0x40;
            }
            s = ERR_SCUESS;
            break;
        case CMD_IAP_VERIFY:
            if (Verity_Star_flag == 0) {
                Verity_Star_flag = 1;
                for (i = 0; i < (64 - CodeLen); i++) {
                    Fast_Program_Buf[CodeLen + i] = 0xFF;
                }
                CH32_IAP_Program(Program_addr, (u32*) Fast_Program_Buf);
                CodeLen = 0;
            }
            s = ERR_SCUESS;
            for (i = 0; i < Lenth; i++) {
                if (isp_cmd_t->data[i] != *(u8*) (Verity_addr + i)) {
                    s = ERR_ERROR;
                    break;
                }
            }
            Verity_addr += Lenth;
            break;
        case CMD_IAP_END:
            Verity_Star_flag = 0;
            End_Flag = 1;
            Program_addr = FLASH_Base;
            Verity_addr = FLASH_Base;
            s = ERR_End;
            break;
        default:
            s = ERR_ERROR;
            break;
    }
    return s;
}

/*********************************************************************
 * @fn      GPIO_Cfg
 *
 * @brief   GPIOD init
 *
 * @return  none
 */
void GPIO_Cfg(void) {
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;

    GPIOD->CFGLR &= ~0x70000000;
    GPIOD->CFGLR |= 0x80000000;
    GPIOD->BSHR = 1 << 7;
}

/*********************************************************************
 * @fn      GPIO_Check
 *
 * @brief   Check PD7 state
 *
 * @return  1 - IAP
 *          0 - APP
 */
u8 GPIO_Check(void) {
    u8 i = 0;

    GPIO_Cfg();
    SysTick->SR = 0;
    SysTick->CMP = (24000000UL / 8000) * CHECK_DUR;
    SysTick->CNT = 0;
    SysTick->CTLR = 1 << 0;

    while(! (SysTick->SR & (1 << 0))) {
        i = ((GPIOD->INDR & GPIO_Pin_7) == 0);
        if (! i)
            break;
    }
    SysTick->CTLR = 0;
/*
    if(! (GPIOD->INDR & GPIO_Pin_7))
        i = (uint8_t)Bit_SET;
    else
        i = (uint8_t)Bit_RESET;

    if (i == 0) {
        Delay_Ms(100);
        return 0;
    }

    return 1;
*/
    return i;
}

void UART1_SendMultiyData(const u8* pbuf, u8 num) {
    while(num--) {
        while(! (USART1->STATR & USART_STATR_TC)) {}
        USART1->DATAR = *pbuf++;
    }
}

void UART1_SendData(u8 data) {
    while(! (USART1->STATR & USART_STATR_TC)) {}
    USART1->DATAR = data;
}

/*********************************************************************
 * @fn      Uart1_Rx
 *
 * @brief   Uart1 receive date
 *
 * @return  none
 */
u8 Uart1_Rx(void) {
    while(! (USART1->STATR & USART_STATR_RXNE)) {}
    return USART1->DATAR;
}

/*********************************************************************
 * @fn      UART_Rx_Deal
 *
 * @brief   UART Rx data deal
 *
 * @return  1 if END receiver, otherwise 0
 */
u8 UART_Rx_Deal(void) {
    u8 i, s;
    u8 Data_add = 0;
    u8 ret = 0;

    if (Uart1_Rx() == Uart_Sync_Head1) {
        if (Uart1_Rx() == Uart_Sync_Head2) {
            isp_cmd_t->Cmd = Uart1_Rx();
            Data_add += isp_cmd_t->Cmd;
            isp_cmd_t->Len = Uart1_Rx();
            Data_add += isp_cmd_t->Len;
            isp_cmd_t->Rev[0] = Uart1_Rx();
            Data_add += isp_cmd_t->Rev[0];
            isp_cmd_t->Rev[1] = Uart1_Rx();
            Data_add += isp_cmd_t->Rev[1];

            if ((isp_cmd_t->Cmd == CMD_IAP_PROM) || (isp_cmd_t->Cmd == CMD_IAP_VERIFY)) {
                for (i = 0; i < isp_cmd_t->Len; i++) {
                    isp_cmd_t->data[i] = Uart1_Rx();
                    Data_add += isp_cmd_t->data[i];
                }
            }

            if (Uart1_Rx() == Data_add) {
                s = RecData_Deal();

                if (s != ERR_End) {
                    UART1_SendData(0x00);
                    if (s == ERR_ERROR) {
                        UART1_SendData(0x01);
                    } else {
                        UART1_SendData(0x00);
                    }
                } else {
//                    if (GPIOD->INDR & GPIO_Pin_7) // PD7 released
                        ret = 1;
                }
            }
        }
    }
    return ret;
}
