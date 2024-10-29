/*
 * UART_Init.c
 *
 *  Created on: 7 mar. 2024
 *      Author: gerardolopezfrayre
 */

#include "board.h"
#include "UART_Init.h"

USART_Type *Usart = USART0;
uint8_t Datos= 0x00;

void vfUART_Init()
{
	SYSCON_Type *rSyscon = SYSCON;//rsyscon
	IOCON_Type *rIocon = IOCON;//pinsetup
	FLEXCOMM_Type *rFlexcom = FLEXCOMM0; //flexclock
	NVIC_Type *rInterrupt = NVIC;


	rSyscon->FCCLKSEL.FCCLKSEL0 = SYSCON_FCCLKSEL0_SEL(2);

	//ENCENDER EL IOCON
	rSyscon->AHBCLKCTRL.AHBCLKCTRL0 |= SYSCON_AHBCLKCTRL0_IOCON_MASK;

	rIocon->PIO[0][29] = IOCON_PIO_FUNC(1) | IOCON_PIO_DIGIMODE_MASK;
	rIocon->PIO[0][30] = IOCON_PIO_FUNC(1) | IOCON_PIO_DIGIMODE_MASK;

	rSyscon->AHBCLKCTRL.AHBCLKCTRL1 |= SYSCON_AHBCLKCTRL1_FC0_MASK;


	rSyscon->PRESETCTRLSET[1] = SYSCON_PRESETCTRL1_FC0_RST_MASK;
	rSyscon->PRESETCTRLCLR[1] = SYSCON_PRESETCTRL1_FC0_RST_MASK;

	rFlexcom->PSELID = FLEXCOMM_PSELID_PERSEL(1);

	Usart->FIFOCFG |= USART_FIFOCFG_EMPTYTX_MASK | USART_FIFOCFG_ENABLETX_MASK |
					  USART_FIFOCFG_EMPTYRX_MASK | USART_FIFOCFG_ENABLERX_MASK;

	Usart->FIFOTRIG = USART_FIFOTRIG_TXLVLENA(1) | USART_FIFOTRIG_RXLVLENA(1);


	Usart->CFG = USART_CFG_ENABLE_MASK | USART_CFG_DATALEN(1);


	Usart->BRG =0x14;

	Usart->OSR = 0x04;

	rInterrupt->ISER[0] = 1<<14;

	Usart->FIFOINTENSET = USART_FIFOINTENSET_RXERR_MASK | USART_FIFOINTENSET_RXLVL_MASK ;

}

void vfTX_Usart(uint8_t *TxData, uint32_t wsize)
{
	if (0U == (Usart->FIFOCFG & USART_FIFOCFG_ENABLETX_MASK ))
	{
		return;
	}

	while (wsize--)
	{
		if(Usart->FIFOSTAT & USART_FIFOSTAT_TXNOTFULL_MASK)
		{
			Usart->FIFOWR = *TxData;
			TxData ++;
		}
	}
}

void FLEXCOMM0_IRQHandler()
{
	if((USART_FIFOSTAT_RXNOTEMPTY_MASK | USART_FIFOSTAT_RXERR_MASK) & Usart->FIFOSTAT)
	{
		Datos = Usart->FIFORD;
	}
}