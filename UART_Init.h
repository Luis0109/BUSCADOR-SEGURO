/*
 * UART_Init.h
 *
 *  Created on: 7 mar. 2024
 *      Author: gerardolopezfrayre
 */

void vfUART_Init();
void vfTX_Usart(uint8_t *TxData, uint32_t wsize);
void FLEXCOMM0_IRQHandler();
