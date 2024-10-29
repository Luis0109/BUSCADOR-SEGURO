/*
 * ctimer_driver.h
 *
 *  Created on: Feb 18, 2024
 *      Author: Mario Castaneda
 */

#ifndef CTIMER_DRIVER_H_
#define CTIMER_DRIVER_H_

#include "board.h"

#define PWM_PIN		0x01



void vfnInitCTimerAndPins();
void vfnSetUpPWM_Freq(uint32_t pwmFreqHz,uint8_t bChannel);
void vfnUpdate_PWM(uint16_t dutyCyclePercent, uint8_t bChannel);
void CTIMER_StartTimer();
void CTIMER_StopTimer();

#endif /* CTIMER_DRIVER_H_ */
