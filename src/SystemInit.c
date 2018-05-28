#include <stdio.h>
#include "MDR32Fx.h"
#include "settings.h"
#include "mkpinout.h"
#include "timers.h"
#include "can.h"
#include "dac.h"
#include "adc.h"
#include "engineControl.h"
#include "canMonitor.h"
#include "canMonitorText.h"
#include "GkControlMode.h"

void rcc_init();
void ports_init();

void SystemInit()
{
   SCB->VTOR = 0x08000000;             // смещение таблицы векторов
   SCB->CCR |= 1<<3 | 1<<4;            // UNALIGN_TRP, DIV_O_TRP - исключение при делении на ноль
   
   rcc_init();                         // настройка тактирования ядра   
   delayTimer_init();                  // инициализация таймера для задержек (мкс, мс)
   ports_init();                       // инициализация портов
   can2_init();                        // инициализация can2
   dac_init();                         // ЦАП
   adc_init();                         // АЦП
   timer1_pwm_init();                  // таймер ШИМ
}

// Инициализация тактирования
void rcc_init()
{
    MDR_RST_CLK->CPU_CLOCK = 0;  

    MDR_RST_CLK->HS_CONTROL |= RST_CLK_HS_CONTROL_HSE_ON;                       // Включить внешний генератор
    while(!(MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_HSE_RDY));         // Ждем 
    MDR_RST_CLK->CPU_CLOCK |= 0x2;                                              // CPU_C1 = HSE

    MDR_RST_CLK->PLL_CONTROL = (CPU_PLL_KOEF<<8);                               // CPU PLL         
    MDR_RST_CLK->PLL_CONTROL |= (1<<2);                                         // CPU PLL On
    while(!(MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY));     // waiting cpu pll ready
    MDR_RST_CLK->CPU_CLOCK |= 1<<2;                                             // CPU_C2 = PLLCPUo

    MDR_RST_CLK->CPU_CLOCK &= ~(1<<7);                                          // CPU_C3 = CPU_C2
    MDR_RST_CLK->CPU_CLOCK |= 1<<8;                                             // HCLK = CPU_C3

    MDR_EEPROM->CMD = 0x3<<3;         // 3 такта паузы при работе на частотах до 100МГц

    MDR_RST_CLK->PER_CLOCK = 1<<3 |   // EEPROM_CNTRL
                             1<<4;    // RST_CLK
}

// Инициализация портов
void ports_init()
{  
    MDR_RST_CLK->PER_CLOCK |= 1<<21 |     // PORTA
                              1<<22 |     // PORTB
                              1<<23 |     // PORTC
                              1<<24 |     // PORTD
                              1<<25 |     // PORTE
                              1<<29;      // PORTF
    
    // Управление ключами, PWM
    MDR_PORTA->FUNC = 0x2<<2;   // timerPWM
    MDR_PORTA->OE = 1<<1 | 1<<2| 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7;
    MDR_PORTA->ANALOG = 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7;
    MDR_PORTA->PWR = 0x3<<2 | 0x3<<4 | 0x3<<6 | 0x3<<8 | 0x3<<10 | 0x3<<12 | 0x3<<14;
    MDR_PORTA->RXTX = 0;
    engine_disableSpin();
    
    // Can2
    MDR_PORTF->FUNC = 0x3<<4  |       // CAN2 RX
                      0x3<<6;         // CAN2 TX
    MDR_PORTF->OE = 0<<2 |        // CAN2 RX  in 
                    1<<3;         // CAN2 TX  out
    MDR_PORTF->ANALOG = 1<<2 | 1<<3;
    MDR_PORTF->PWR = 0x3<<4 | 0x3<<6;
    
    
    // АЦП
    MDR_PORTD->FUNC = 0;
    MDR_PORTD->OE = 0;
    MDR_PORTD->ANALOG = 0;
    MDR_PORTD->PWR = 0x3<<4 | 0x3<<6;
    
    // ЦАП
    MDR_PORTE->FUNC = 0;
    MDR_PORTE->OE = 1<<0;
    MDR_PORTE->ANALOG = 0;
    MDR_PORTE->PWR = 0x3<<0;
}





//==============================================================================
//==============================================================================
//========================== Обработчик исключений =============================
//==============================================================================
//==============================================================================

void Hard_fault_handler_c(unsigned int* hardfault_args);

void HardFault_Handler()
{
   engine_disableSpin();
   
   uint32_t contr_reg = __get_CONTROL();
   if(contr_reg&2)
   {
      asm("MRS R0, PSP");
   }
   else
   {
      asm("MRS R0, MSP");
   }
   asm("B    (Hard_fault_handler_c)");                    //top of stack is in R0. It is passed to C-function.
}

void Hard_fault_handler_c(unsigned int* hardfault_args)
{
   unsigned int stacked_r0 = ((unsigned long) hardfault_args[0]);
   unsigned int stacked_r1 = ((unsigned long) hardfault_args[1]);
   unsigned int stacked_r2 = ((unsigned long) hardfault_args[2]);
   unsigned int stacked_r3 = ((unsigned long) hardfault_args[3]);
   unsigned int stacked_r12 = ((unsigned long) hardfault_args[4]);
   unsigned int stacked_lr = ((unsigned long) hardfault_args[5]);
   unsigned int stacked_pc = ((unsigned long) hardfault_args[6]);
   unsigned int stacked_psr = ((unsigned long) hardfault_args[7]);

   extern GKControlMode_t gk_controlMode;
   gk_controlMode = GkMode_HardFault;
   uint8_t msg[8];
   canMonitor_fillBufferWithCurrentState(msg);
   
   while(1)
   {
     can_putPackage(MDR_CAN2, CAN_MONITOR_TX_BUF, CANID_MONITOR_TX, msg, sizeof(msg));
     while(CAN_TX_REQ(MDR_CAN2, CAN_MONITOR_TX_BUF));
     
     canMonitor_blockingPrintf(MDR_CAN2, CAN_DBG_TEXT_MESSAGE_TX_BUF, 
                               "HARDFAULT!\n"
                               "R0  = %.8x\n"
                               "R1  = %.8x\n"
                               "R2  = %.8x\n"
                               "R3  = %.8x\n"
                               "R12 = %.8x\n"
                               "LR  = %.8x\n"
                               "PC  = %.8x\n"
                               "PSR = %.8x\n\n",  stacked_r0, stacked_r1, stacked_r2,
                                                  stacked_r3, stacked_r12, stacked_lr,
                                                  stacked_pc, stacked_psr);
     delay_ms(5*1000);
   }
}













