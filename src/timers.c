#include "timers.h"
#include "settings.h"

uint32_t _tmpppp[10];
volatile uint32_t system_time = 0;   // системное время
uint16_t PWM_MAX_FILLING = TIM_PWM_SET_MAX_FILLING(60);

/*
// Инициализация Timer1
void timer1_init(uint16_t us)
{
    MDR_RST_CLK->PER_CLOCK |= 1<<14;                              // тактирование TIMER2    
    MDR_RST_CLK->TIM_CLOCK |= 0x3<<0 |                            // HCLCK/8 - такктирование от 10MHz
                              1<<24;                              // разрешение тактирование таймера
            
    MDR_TIMER1->CNT = 0;
    MDR_TIMER1->PSG = 10-1;                                    // 1000 kHz
    MDR_TIMER1->ARR = us;                                         // раз в msc миллисекунд при f=80MHz
    
    MDR_TIMER1->IE |= TIMER_IE_CNT_ARR_EVENT_IE;  
    NVIC_EnableIRQ(Timer1_IRQn);                 
}
*/

// Инициализация таймера в режиме ШИМ
void timer1_pwm_init()
{
    MDR_RST_CLK->PER_CLOCK |= 1<<14;                              // тактирование TIMER2    
    MDR_RST_CLK->TIM_CLOCK |= 0x2<<0 |                            // HCLCK/4 - такктирование от 20MHz при F=80
                              1<<24;                              // разрешение тактирование таймера

    MDR_TIMER1->CH1_CNTRL = 0<<0 |      // канал работает в режиме ШИМ
                            6<<9;       // 1, если DIR= 0 (счет вверх), CNT<CCR, иначе 0
    MDR_TIMER1->CH1_CNTRL1 = 1<<0 |     // всегда на OE выдается 1, канал всегда работает на выход
                             2<<2;      // на выход выдается сигнал REF    
    MDR_TIMER1->CNT = 0;
    MDR_TIMER1->ARR = TIM_PWM_ARR_REG;
    setPwmFillingF(0);

    MDR_TIMER1->CNTRL |= 1<<0;         // запустить таймер
}

// Установить заполнение ШИМ
void setPwmFillingF(uint16_t upr)
{
   uint16_t ccr1;
   /*
   if(upr > TIM_PWM_60_PERCENTAGE_CCR_REG) ccr1 = TIM_PWM_60_PERCENTAGE_CCR_REG;   
   else ccr1 = upr;  
   uint16_t tmp = TIM_PWM_60_PERCENTAGE_CCR_REG;
   tmp++;
   */
   if(upr > PWM_MAX_FILLING) ccr1 = PWM_MAX_FILLING;   
   else ccr1 = upr;  
   
   MDR_TIMER1->CCR1 = ccr1;
}


// Включить/Выключить таймер
void timer_setEnable(MDR_TIMER_TypeDef *timer, uint8_t enable)
{
    if(enable) timer->CNTRL |= TIMER_CNTRL_CNT_EN;
    else timer->CNTRL &= ~(TIMER_CNTRL_CNT_EN);
}

// ================================ System timer ===============================
// Инициализация TIMER_2 в качестве системного таймера. T = 1c
void sysTimer_init(uint16_t us)
{      
    MDR_RST_CLK->PER_CLOCK |= 1<<15;                              // тактирование TIMER2    
    MDR_RST_CLK->TIM_CLOCK |= 0x3<<8 |                            // HCLCK/8 - такктирование от 10MHz
                              1<<25;                              // разрешение тактирование таймера
    
    MDR_TIMER2->CNT = 1;
    MDR_TIMER2->PSG = 10-1;                                       // 1000000 Hz
    MDR_TIMER2->ARR = us-1;                                         // T = 1мкс при f=80MHz    
    MDR_TIMER2->IE = TIMER_IE_CNT_ARR_EVENT_IE;                   // прерывание при CNT == ARR       
    MDR_TIMER2->STATUS = 0;
    NVIC_EnableIRQ(Timer2_IRQn);
    MDR_TIMER2->CNTRL |= TIMER_CNTRL_CNT_EN;                      // разрешение работы таймера          
}


// ================================== Delay ====================================
// Миллиссекундная и микросекундная задержка на основе таймера общего назначеня
// Инициализация Timer3. Т = 1 мкс
void delayTimer_init()
{      
   MDR_RST_CLK->PER_CLOCK |= 1<<16;                              // тактирование TIMER2    
   MDR_RST_CLK->TIM_CLOCK |= 0x3<<16 |                           // HCLCK/8 - такктирование от 10MHz
                             1<<26;                              // разрешение тактирование таймера
            
   MDR_TIMER3->CNT = 0;
   MDR_TIMER3->PSG = 10-1;                                       // 1000000 kHz
   MDR_TIMER3->ARR = 0xFFFF;                                     // раз в msc миллисекунд при f=80MHz
    
   timer_setEnable(MDR_TIMER3, 1);
}

// Задержка в итерациях. При HCLK=80МГц, Т=12.5нс
void delay_tics(uint32_t tics)
{ 
   for(volatile uint32_t i=0; i<tics; i++);
}

// Задержка на mcs микросекунд
__ramfunc void delay_us(uint16_t us)
{    
   // TimFreq = 10МГц
   MDR_TIMER3->PSG = 10-1;              // 1 MHz  T=1мкс
   MDR_TIMER3->CNT = 0;    
   while(MDR_TIMER3->CNT < us);
}

// Задержка на ms миллисекунд
__ramfunc void delay_ms(uint16_t ms)
{
   // TimFreq = 10МГц
   MDR_TIMER3->PSG = 10000-1;           // 1000 Hz  T=1мс
   MDR_TIMER3->CNT = 0;
   while(MDR_TIMER3->CNT < ms);
}


// Возвращает 1, если прошло delay_ms от момента *var, относительно системного времени
__ramfunc uint8_t elapsed(uint32_t *var, const uint32_t delay_ms) 
{
   if ((system_time - *var) >= delay_ms) 
   {
      *var = system_time; // сброс
      return 1;
   }
   else return 0;
}

