#include "debugingInfo.h"
#include "timers.h"
#include "global.h"
#include "canmonitor.h"


#define DEBUGINFO_SEND_PERIOD           3

extern int16_t dusTemperature;
int16_t frontSemisphereTemp;    // температура в передней полусфере
int16_t backSemisphereTemp;     // температура в задней полусфере
int16_t ustrSoprTemp;           // температура на плате упрпавления
uint8_t printCurrentTempEnable = 0;   // выводить текущую температуру в терминал


#define MAX_DBG_TEMP    80
#define MIN_DBG_TEMP    -80  
uint8_t dbg_useTestDusTemp = 0; // использовать отладочную инфу ДУС
int16_t dbg_dusTemp = 20;       // текущая температура ДУС ДЛЯ ОТЛАДКИ
int dbg_temperatureChangePeriod = 3000;      // время, через которое будет увеличиваться температура
uint8_t dbg_tempChangemode = 0;              // режим изменения тестовой прошивки

    

uint8_t debugInfo_globalEnable = 0;     // общее разрешение отправки отладочной информации
uint8_t debugInfo_velocityEnable = 0;   // передача скорости
uint8_t debugInfo_uprEnable = 0;        // передача управляющего сигнала
uint8_t debugInfo_angleEnable = 0;      // передача угла
uint8_t debugInfo_testValue1Enable = 0; // передача тестового значения 1

void dbg_temperatureService();

void debugingInfoService()
{
  static uint32_t timer = 0;
  
  dbg_temperatureService(); // температурный сервер
  
  if(!debugInfo_globalEnable) return;
  if(!elapsed(&timer, DEBUGINFO_SEND_PERIOD)) return;
  /*
  if(debugInfo_velocityEnable)
  {
    canMonitor_sendCourseVelocity();    // угловая скорость    
  }
  */
  if(debugInfo_uprEnable)
  {
    canMonitor_sendCourseUpr();         // рассогласование    
  }
  /*
  if(debugInfo_angleEnable)
  {
    canMonitor_sendAngle();             // текущий угол    
  }
  */
  if(debugInfo_testValue1Enable)        // тестовое значение 1
  {
    /*
    extern float lastDiffOut;
    extern int16_t dusAmplitude;
    canMonitor_sendTestValue1(testArValue);
    */
  }
}


// Температурный сервер
void dbg_temperatureService()
{
  static uint32_t changeTempTime = 0;
  static uint32_t temperatureLogTime = 0;
  static uint8_t dirUp = 1;
  
  if(printCurrentTempEnable && elapsed(&temperatureLogTime, 1000))
  {
    canMonitor_printf("Текущая температура %d", dusTemperature);
  }
  
  
  if(dbg_temperatureChangePeriod <= 0) return;
  if(system_time - changeTempTime < dbg_temperatureChangePeriod) return;  
  changeTempTime = system_time;
  
  switch(dbg_tempChangemode)
  {
    case 0: break;// не изменять значение
            
    case 1: // увеличивать
      dbg_dusTemp++;   
      if(dbg_dusTemp > MAX_DBG_TEMP) dbg_dusTemp = MIN_DBG_TEMP;
      break;
      
    case 2: 
      dbg_dusTemp--;   
      if(dbg_dusTemp < MIN_DBG_TEMP) dbg_dusTemp = MAX_DBG_TEMP;
      break;
      
    case 3:
      dbg_dusTemp = dirUp ? dbg_dusTemp+1 : dbg_dusTemp-1;      
      if(dirUp && dbg_dusTemp >= MAX_DBG_TEMP) dirUp = 0;
      if(!dirUp && dbg_dusTemp <= MIN_DBG_TEMP) dirUp = 1;
      break;
  }
  
  if(dbg_useTestDusTemp)
  {
    dusTemperature = dbg_dusTemp;
  }
}
      

// Парсинг данных от устройства сопряжения
void debugingInfo_rxFrameHandler(uint8_t *buf, uint8_t len)
{
  switch(buf[0])
  {
    case 10: // климатические параметры
    {
      if(len != 5) return;
      frontSemisphereTemp = (int8_t)buf[1];  // температура в передней полусфере
      backSemisphereTemp  = (int8_t)buf[2];  // температура в задней полусфере
      ustrSoprTemp = (int8_t)buf[3];         // температура на плате упрпавления
   
      
      if(!dbg_useTestDusTemp)
      {
        dusTemperature = backSemisphereTemp;
      }
    }
    break;
  }
}

