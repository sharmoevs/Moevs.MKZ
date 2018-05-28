#ifndef __DUS_H__
#define __DUS_H__

#include "MDR32Fx.h"
#include "dusFilter.h"



//#define VEL_CALIBRATION_AVERAGE     128 



uint16_t dus_getNextSample();
//void     dus_calibrateZeroVelocity();
float    dus_convertCodeToFloatVelocity(uint16_t code);


#endif //__DUS_H__