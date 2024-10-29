/*
 * UART_2Init.c
 *
 *  Created on: 16 abr. 2024
 *      Author: gerardolopezfrayre
 */


#include "board.h"
#include "UART_2Init.h"


USART_Type *Usart2 = USART2;
//uint8_t Datos2= 0x00;
uint8_t Datos2;




void vfUART2_Init()
{
	SYSCON_Type *rSyscon = SYSCON;//rsyscon
	IOCON_Type *rIocon = IOCON;//pinsetup
	FLEXCOMM_Type *rFlexcom = FLEXCOMM2; //flexclock
	NVIC_Type *rInterrupt = NVIC;


	rSyscon->FCCLKSEL.FCCLKSEL2 = SYSCON_FCCLKSEL2_SEL(2);

	//ENCENDER EL IOCON
	rSyscon->AHBCLKCTRL.AHBCLKCTRL0 |= SYSCON_AHBCLKCTRL0_IOCON_MASK;

	rIocon->PIO[0][27] = IOCON_PIO_FUNC(1) | IOCON_PIO_DIGIMODE_MASK;
	rIocon->PIO[1][24] = IOCON_PIO_FUNC(1) | IOCON_PIO_DIGIMODE_MASK;

	rSyscon->AHBCLKCTRL.AHBCLKCTRL1 |= SYSCON_AHBCLKCTRL1_FC2_MASK;


	rSyscon->PRESETCTRLSET[1] = SYSCON_PRESETCTRL1_FC2_RST_MASK;
	rSyscon->PRESETCTRLCLR[1] = SYSCON_PRESETCTRL1_FC2_RST_MASK;

	rFlexcom->PSELID = FLEXCOMM_PSELID_PERSEL(1);

	Usart2->FIFOCFG |= USART_FIFOCFG_EMPTYTX_MASK | USART_FIFOCFG_ENABLETX_MASK |
					  USART_FIFOCFG_EMPTYRX_MASK | USART_FIFOCFG_ENABLERX_MASK;

	Usart2->FIFOTRIG = USART_FIFOTRIG_TXLVLENA(1) | USART_FIFOTRIG_RXLVLENA(1);


	Usart2->CFG = USART_CFG_ENABLE_MASK | USART_CFG_DATALEN(1);


	Usart2->BRG =0x14;

	Usart2->OSR = 0x04;

	rInterrupt->ISER[0] = 1<<16;

	Usart2->FIFOINTENSET = USART_FIFOINTENSET_RXERR_MASK | USART_FIFOINTENSET_RXLVL_MASK ;

}

void vfTX_Usart_2(uint8_t *TxData, uint32_t wsize)
{
	if (0U == (Usart2->FIFOCFG & USART_FIFOCFG_ENABLETX_MASK ))
	{
		return;
	}

	while (wsize--)
	{
		if(Usart2->FIFOSTAT & USART_FIFOSTAT_TXNOTFULL_MASK)
		{
			Usart2->FIFOWR = *TxData;
			TxData ++;
		}
	}
}

void FLEXCOMM2_IRQHandler()
{
	if((USART_FIFOSTAT_RXNOTEMPTY_MASK | USART_FIFOSTAT_RXERR_MASK) & Usart2->FIFOSTAT)
	{
		Datos2 = Usart2->FIFORD;
	}
}


