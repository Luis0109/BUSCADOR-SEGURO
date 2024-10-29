/*
 * UART_2Init.h
 *
 *  Created on: 16 abr. 2024
 *      Author: gerardolopezfrayre
 */

#ifndef UART_2INIT_H_
#define UART_2INIT_H_

void vfUART2_Init();
void vfTX_Usart_2(uint8_t *TxData, uint32_t wsize);
void FLEXCOMM2_IRQHandler();
void loop(uint8_t Datosx);

#endif /* UART_2INIT_H_ */
