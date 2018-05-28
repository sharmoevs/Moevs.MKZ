#include <cstdlib>
#include <stdio.h>
#include "MDR32Fx.h"
#include "timers.h"
#include "global.h"
#include "canterminal.h"
#include "loaders.h"
#include "flashUserData.h"
#include "dus.h"
#include "dusCalibration.h"
#include "pidRegulator.h"
#include "cangkcontrol.h"
#include "angleSensor.h"
#include "gkControlMode.h"
#include "debugingInfo.h"
#include "canMonitor.h"
#include "canMonitorText.h"
#include "canFrameQueue.h"

extern uint16_t currentTangageAngleCode;
extern uint16_t zeroLevelAngleCodeOffset;
extern CanSoftwareBuffer_t *canmonitorSoftBuffer;


void dbgSendCanTestMessage();

void main()
{
  loaders_init();               // инициализация загрузчиков и включение CAN-прерываний
  readUserDataFromFlash();      // чтение сохраненных настроек во флеши
  pid_init();                   // ПИД-регулятор
  pidFilter_Init();             // инициализация фильтра PID-регулятора  
#ifdef USE_DYNAMIC_CAN_QUEUE
  canText_init(MDR_CAN2);       // инициализация отладки
  canMonitor_init();            // инициализация
#endif
  sysTimer_init(500);           // системный таймер (500мкс)
  NVIC_EnableIRQ(CAN2_IRQn);    // разрешение прерываний can. Чтобы после перепрограммирования загрузчики были инициализированы
  

  uint32_t dbgTimer = 0;
  while(1)
  {
#ifdef USE_DYNAMIC_CAN_QUEUE
    canMonitorText_server();              // сервер тектовых сообщений
    canSwBuffer_service(canmonitorSoftBuffer);
#endif
        
    if(elapsed(&dbgTimer, 10))
    {
      canGkControl_sendCurrentState();
      debugingInfoService();                // вывод отладочной инфы
    }
  }
}

// ДУС и управление двигателями
void Timer2_IRQHandler(void)
{
  MDR_TIMER2->STATUS &= ~TIMER_STATUS_CNT_ARR_EVENT;                            // сбросить флаг прерывания
  MDR_TIMER2->CNT = 0;
  
  static uint8_t numOfIrq = 0;
  if(++numOfIrq == 2)
  {
    numOfIrq = 0;
    system_time++;
  }
  
  dus_getNextSample();          // значение ДУС
  dus_calibrate();              // калибровка ДУС
  gk_checkSpeedProtection();    // защита по скорости
  gk_moveNext();                // управление двигателями
  
  
  // Отправка текущих данных
  static uint32_t timer = 0;
  if(elapsed(&timer, 1))
  {
    canMonitor_sendAngle();             // текущий угол
    canMonitor_sendCourseVelocity();    // угловая скорость
  }  
    
  // Отладка. Загрузка шины CAN
  ////dbgSendCanTestMessage();
}


// Отладка, загрузить шину CAN
void dbgSendCanTestMessage()
{
  static uint8_t frame1[] = {5,6,7,8};
  static uint8_t frame2[] = {0x9,0xa,0xb,0xc};
  static uint8_t CAN_ID1 = 0xbb;
  static uint8_t CAN_ID2 = 0x19;
    
  can2_putDataInBuf(CAN_TEST1_TX_BUF, CAN_ID1, frame1, sizeof(frame1));
  can2_putDataInBuf(CAN_TEST2_TX_BUF, CAN_ID2, frame2, sizeof(frame2));
}
