#ifndef __MKPINOUT_H__
#define __MKPINOUT_H__

// Управление переферией на плате формирования углового положения по тангажу


#define AH_HI()                         (MDR_PORTA->RXTX |= 1<<2)
#define AH_LO()                         (MDR_PORTA->RXTX &= ~(1<<2))
                                          
#define AL_HI()                         (MDR_PORTA->RXTX |= 1<<3)
#define AL_LO()                         (MDR_PORTA->RXTX &= ~(1<<3))
                                          
#define BH_HI()                         (MDR_PORTA->RXTX |= 1<<4)
#define BH_LO()                         (MDR_PORTA->RXTX &= ~(1<<4))
                                          
#define BL_HI()                         (MDR_PORTA->RXTX |= 1<<5)
#define BL_LO()                         (MDR_PORTA->RXTX &= ~(1<<5))

#define CH_HI()                         (MDR_PORTA->RXTX |= 1<<6)
#define CH_LO()                         (MDR_PORTA->RXTX &= ~(1<<6))
          
#define CL_HI()                         (MDR_PORTA->RXTX |= 1<<7)
#define CL_LO()                         (MDR_PORTA->RXTX &= ~(1<<7))



#endif // __MKPINOUT_H__
