#ifndef __FLASH_USER_DATA_H__
#define __FLASH_USER_DATA_H__

#include "MDR32Fx.h"
#include "flash1986ve9x.h"


#define DEVICE_DESCRIPTION_MAX_LEN              256


#define USER_DATA_BASE_FLASH_ADDR       GET_FLASH1986VE9x_PAGE_ADDR(27)         // базовый адрес во флешь-памяти для пользовательских настроек
#define ZERO_LEVEL_OFFSET_ADDR          (USER_DATA_BASE_FLASH_ADDR + 0)         // смешщение фазового нуля двигателей
#define PID_KOEF_P_ADDR                 (USER_DATA_BASE_FLASH_ADDR + 4)         // значениe коэффициента P PID-регулятора
#define PID_KOEF_I_ADDR                 (USER_DATA_BASE_FLASH_ADDR + 8)         // значениe коэффициента I PID-регулятора
#define PID_KOEF_D_ADDR                 (USER_DATA_BASE_FLASH_ADDR + 12)        // значениe коэффициента D PID-регулятора
#define PID_TDISKR_ADDR                 (USER_DATA_BASE_FLASH_ADDR + 16)        // значениe времени дискретизации
#define DUS_KORR_KOER_ADDR              (USER_DATA_BASE_FLASH_ADDR + 20)        // калибровочный коэф. ДУС
#define PID_KOEF_N_ADDR                 (USER_DATA_BASE_FLASH_ADDR + 28)        // значениe коэффициента N PID-регулятора

#define DUS_TEMP_CALIB_KOEF_START_ADDR  (USER_DATA_BASE_FLASH_ADDR + 200)       // температурная поправка к ДУС


void readUserDataFromFlash();
void saveUserDataInFlash();


#endif //__FLASH_USER_DATA_H__