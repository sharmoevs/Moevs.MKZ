#ifndef __PID_REGULATOR_H__
#define __PID_REGULATOR_H__

#include "MDR32Fx.h"
#include "pidFilter.h"


void pid_init();                        // Инициализация ПИД-регулятора
void pid_setNewKoef(float newP, float newI, float newD, float newN, float newT);
void pid_reset();
void pid_setEnable(uint8_t enable);     // включить/выключить ПИД-регулятор
float pid_nextCode(float presetCode, int16_t dusCode);






#endif //__PID_REGULATOR_H__