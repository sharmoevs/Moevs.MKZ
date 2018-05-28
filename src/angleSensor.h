#ifndef __ANGLE_SERVICE_H__
#define __ANGLE_SERVICE_H__

#include "MDR32Fx.h"


//#define CONVERT_ANGLE16_TO_FLOAT(code32)    (float)(360.0*code32/0xFFFF)
//#define CONVERT_FLOAT_TO_ANGLE16(angleF)    (uint16_t)(angleF*0xFFFF/360)



void angleService_rxAngle(uint8_t *buf, uint8_t len);
int16_t angle_convertAbs16BitCodeTo16BitCode(uint16_t angle);
uint8_t angle_getAverageVelocity(float *averageVelocity);       // средняя скорость по углу
int32_t angle_getMinimalAngleRange(uint32_t startAngle, uint32_t stopAngle);





#define MAX_SYSANGLE_CODE 0x3FFFF
#define USYSANGLE_TO_FLOAT(code32) (float)(code32*360.0/MAX_SYSANGLE_CODE)
#define USYSANGLE_TO_CODE(angleF)  (uint32_t)(angleF*MAX_SYSANGLE_CODE/360.0)



uint32_t angle_deductCorrection(uint32_t angle, uint32_t deltaFi, uint32_t code360);
uint32_t angle_addCorrection(uint32_t angle, uint32_t deltaFi, uint32_t code360);

uint32_t angle_convertCmdS2SysU(int32_t sCmdAngle, uint32_t scaleCorrection, uint32_t code360);





#endif//__ANGLE_SERVICE_H__