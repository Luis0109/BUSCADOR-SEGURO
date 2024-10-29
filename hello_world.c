/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "stdint.h"
#include "fsl_power.h"
#include "stdio.h"
#include "UART_Init.h"
#include "UART_2Init.h"
#include "ctimer_driver.h"
#define PWM_FREQ 1000
#define DELAY 0x1500
#define IBUS_LENGTH 32 // Longitud de la trama IBUS
#define TRAMA_SIZE 32



extern USART_Type *Usart;
extern USART_Type *Usart2;
extern uint8_t Datos;
extern uint8_t Datos2;
uint8_t Mensaje[] = ("\r\nUSART\r\n");

uint32_t trama[TRAMA_SIZE];
uint32_t trama1[TRAMA_SIZE];

void loop(uint8_t Datosx)
{
	        			for(int i=0;i<32;i++)
	        			{
	        				if (i == 31)
	        				{
	        				trama[i] = Datos2;
	        				} else
	        				{
	        				 trama[i] = trama[i + 1];
	        				}
	        			}

	        			 if (trama[0] == 0x20 && trama[1] == 0x40 && trama[28] == 0xDC && trama[29] == 0x05)
	        		     {
	        				 if(trama[2] <= 99)
	        				 {
	        					// PRINTF("DRIVE");
	        					 //PRINTF("%d", trama[2]);
	        					 vfnUpdate_PWM(0,2);
	        					 vfnUpdate_PWM( trama[2],0);
	        				 }else if (trama[2] >= 100 && trama[2] <= 200)
	        				 {
	        					 trama1[2]= 200- trama[2];
	        					 //PRINTF("%d", trama1[2]);
	        					 vfnUpdate_PWM( trama1[2],2);
	        					 vfnUpdate_PWM(0,0);

	        				 }else if(trama[4] <= 99)
	        				 {
	        					// PRINTF("DRIVE");
	        					 //PRINTF("%d", trama[4]);
	        					 vfnUpdate_PWM( trama[4],2);
	        					 vfnUpdate_PWM( trama[4],0);
	        				 }else if (trama[4] >= 100 && trama[4] <= 200)
	        				 {
	        					 trama1[4]= 200- trama[4];
	        					 //PRINTF("%d", trama1[4]);
	        					 vfnUpdate_PWM( trama1[4],1);

	        				 }else
	        				 {
	        					 //PRINTF("%d", 0);
	        					 vfnUpdate_PWM( 0,2);
	        					 vfnUpdate_PWM( 0,1);
	        					 vfnUpdate_PWM( 0,0);
	        				 }
	        					/*for(int x=0;x<32;x++)
	        					{
	        						PRINTF("%d", trama[x]);
	        					if (x < 31)
	        					{
	        					  PRINTF(", ");
	        					}else
	        					{
	        						PRINTF(" \r\n");
	        					}

	        			 	 	 }*/

	        		       trama[0] = 0;
	        		       trama[1] = 0;
	        		       trama[28] = 0;
	        		       trama[29] = 0;

	        		     }
}



int main(void)
{

	char ch;
    //POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    //LOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    //BOARD_InitBootPins();
    //BOARD_InitDebugConsole();

    PRINTF("PRUEBA USART");

    vfUART_Init();
    vfUART2_Init();
    vfTX_Usart(Mensaje,(sizeof(Mensaje)/sizeof(Mensaje[0])));

    vfnInitCTimerAndPins();
    vfnSetUpPWM_Freq(PWM_FREQ,2);
    vfnSetUpPWM_Freq(PWM_FREQ,1);
    vfnSetUpPWM_Freq(PWM_FREQ,0);

    CTIMER_StartTimer();

    uint32_t wDelay = DELAY;

    while (1)
        {
    		if(Datos2)
    		{
    			loop(Datos2);
    			Datos2=0;
    		}

        }
}
