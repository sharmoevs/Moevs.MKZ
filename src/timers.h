#ifndef __TIMERS__
#define __TIMERS__

#include "MDR32Fx.h"


//void timer1_init(uint16_t us);
void timer1_pwm_init();
void setPwmFillingF(uint16_t upr);
void sysTimer_init(uint16_t us);
void delayTimer_init();
void timer_setEnable(MDR_TIMER_TypeDef *timer, uint8_t enable);


__ramfunc void delay_tics(uint32_t tics);
__ramfunc void delay_us(uint16_t us);
__ramfunc void delay_ms(uint16_t ms);

__ramfunc uint8_t elapsed(uint32_t *var, const uint32_t delay_ms);



#endif //__TIMERS__