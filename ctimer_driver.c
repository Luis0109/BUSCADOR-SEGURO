/*
 * ctimer_driver.c
 *
 *  Created on: Feb 18, 2024
 *      Author: Mario Castaneda
 */

#include "ctimer_driver.h"

/* CTimer0 counter uses the AHB clock, some CTimer1 modules use the Aysnc clock */
#define CTIMER_CLK_FREQ 96000000
#define CTIMER          CTIMER2         /* Timer 2 */
#define CTIMER_MAT_OUT  kCTIMER_Match_1 /* Match output 1 */
#define CTIMER_MAT_PWM_PERIOD_CHANNEL kCTIMER_Match_3
/*******************************************************************************
* Code
******************************************************************************/
CTIMER_Type *rCTimer2 = CTIMER2;
CTIMER_Type *rCTimer3 = CTIMER3;
volatile uint32_t g_pwmPeriod   = 0U;
volatile uint32_t g_pulsePeriod = 0U;
/*******************************************************************************
 * vfnInitCTimer
 * DESCRIPTION:Init The PORT 1 for the OKDO RGB Led
 * RETURNS:
 * void
 ******************************************************************************/
void vfnInitCTimerAndPins()
{
	/*Init SYSCOM IOCON clock*/
	SYSCON_Type *rSyscon = SYSCON;
	IOCON_Type *rPinSetup = IOCON;

	/* 	CTimer 2 clock source select register
	 * 	0 Main clock.
	 * 	PLL0 clock.
	 * 	2 No clock.
	 * 	3 FRO 96 MHz clock.<----------------Select
	 *  4 FRO 1 MHz clock.
	 *  5 MCLK clock.
	 *	6 Oscillator 32kHz clock.
	 *	7 No clock.
	 * */

	/*Table 69. CTimer 2 clock source select (CTIMERCLKSEL2, offset = 0x274)*/
	rSyscon->SYSTICKCLKSELX[5] = SYSCON_CTIMERCLKSEL2_SEL(3);
	//rSyscon->SYSTICKCLKSELX[5] = SYSCON_CTIMERCLKSEL3_SEL(3);

	/*Enable IOCON clock*/
	rSyscon->AHBCLKCTRL.AHBCLKCTRL0 |= SYSCON_AHBCLKCTRL0_IOCON_MASK;

	/*CT2MAT1*/
	/*Table 340. I/O control registers FUNC 3*/
	rPinSetup->PIO[PWM_PIN][4] |= IOCON_PIO_FUNC(3) | IOCON_PIO_DIGIMODE_MASK;
	rPinSetup->PIO[PWM_PIN][5] |= IOCON_PIO_FUNC(3) | IOCON_PIO_DIGIMODE_MASK;
	rPinSetup->PIO[PWM_PIN][6] |= IOCON_PIO_FUNC(3) | IOCON_PIO_DIGIMODE_MASK;
	rPinSetup->PIO[PWM_PIN][7] |= IOCON_PIO_FUNC(3) | IOCON_PIO_DIGIMODE_MASK;
	//rPinSetup->PIO[0][23] |= IOCON_PIO_FUNC(3) | IOCON_PIO_DIGIMODE_MASK;

	rSyscon->AHBCLKCTRL.AHBCLKCTRL1 |= SYSCON_AHBCLKCTRL1_TIMER2_MASK;
	//rSyscon->AHBCLKCTRL.AHBCLKCTRL2 |= SYSCON_AHBCLKCTRL2_TIMER3_MASK ;

	//rSyscon->AHBCLKCTRLSET[PWM_PIN] |= SYSCON_AHBCLKCTRL1_TIMER2_MASK | SYSCON_AHBCLKCTRL1_FC0_MASK;
	/*Reset*/
	//rSyscon->PRESETCTRLSET[PWM_PIN] |= SYSCON_PRESETCTRL1_TIMER2_RST_MASK;
	rSyscon->PRESETCTRLCLR[PWM_PIN] |= SYSCON_PRESETCTRL1_TIMER2_RST_MASK;
	//rSyscon->PRESETCTRLCLR[2] |= SYSCON_PRESETCTRL2_TIMER3_RST_MASK;

	rCTimer2->CTCR = CTIMER_CTCR_CTMODE(0) | CTIMER_CTCR_CINSEL(0);
	//rCTimer3->CTCR = CTIMER_CTCR_CTMODE(0) | CTIMER_CTCR_CINSEL(0);
    /* Setup the timer prescale value */
	rCTimer2->PR = CTIMER_PR_PRVAL(0);
	//rCTimer3->PR = CTIMER_PR_PRVAL(0);
}

void vfnSetUpPWM_Freq(uint32_t pwmFreqHz,uint8_t bChannel)
{
	//Period Match 4799 = /*96 000 000/1 000*/
    g_pwmPeriod = (CTIMER_CLK_FREQ / pwmFreqHz) - 1U;
    // PWMEN1
	rCTimer2->PWMC |= CTIMER_PWMC_PWMEN2_MASK;
	rCTimer2->PWMC |= CTIMER_PWMC_PWMEN0_MASK;
	rCTimer2->PWMC |= CTIMER_PWMC_PWMEN1_MASK;
	//rCTimer3->PWMC |= CTIMER_PWMC_PWMEN3_MASK;

	rCTimer2->MCR |= CTIMER_MCR_MR3R_MASK;
	//rCTimer3->MCR |= CTIMER_MCR_MR3R_MASK;

	rCTimer2->MR[3] = g_pwmPeriod;
	//rCTimer3->MR[3] = g_pwmPeriod;

	rCTimer2->IR = CTIMER_IR_MR2INT_MASK;
	rCTimer2->IR = CTIMER_IR_MR1INT_MASK;
	rCTimer2->IR = CTIMER_IR_MR0INT_MASK;
	//rCTimer3->IR = CTIMER_IR_MR3INT_MASK;

	/*kCTIMER_Match_1*/
	rCTimer2->MR[bChannel] = g_pwmPeriod >> 1;
	//rCTimer3->MR[bChannel] = g_pwmPeriod >> 1;
}


void vfnUpdate_PWM(uint16_t dutyCyclePercent, uint8_t bChannel)
{
	/*kCTIMER_Match_1*/
    g_pulsePeriod = (g_pwmPeriod + 1U) * (100 - dutyCyclePercent) / 100;
	rCTimer2->MR[bChannel] = g_pulsePeriod;
	//rCTimer3->MR[bChannel] = g_pulsePeriod;
}

void CTIMER_StartTimer()//(CTIMER_Type *base)
{
	rCTimer2->TCR |= CTIMER_TCR_CEN_MASK;
	//rCTimer3->TCR |= CTIMER_TCR_CEN_MASK;
}


