#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "canmonitor.h"
#include "global.h"
#include "flashUserData.h"
#include "dus.h"
#include "angleSensor.h"
#include "gkControlMode.h"
#include "pidRegulator.h"
#include "timers.h"
#include "mkpinout.h"
#include "settings.h"
#include "canGkControl.h"
#include "dusCalibration.h"
#include "canFrameQueue.h"
#include "dataQueue.h"

extern uint32_t g_sysAngle360;
extern uint8_t dbg_dusCorrectionEnable;
extern int16_t dusCodeAmmend;
extern float koef_P;
extern float koef_I;
extern float koef_D;
extern float koef_N;
extern float tDiskr;
extern uint32_t g_tangageHardwareZeroAngleCorrection; // ноль аппаратной шкалы тангажного контура
extern uint32_t g_tangageLogicalZeroAngleCorrection;  // ноль логической шкалы тангажного контура
extern int16_t dusTemperature;



CanSoftwareBuffer_t *canmonitorSoftBuffer;

float newPID_koefP;
float newPID_koefI;
float newPID_koefD;
float newPID_koefN;
float newPID_tDiskr;
float tudaSudaVelocityF;

       
uint8_t extCmdBuf[256];             // буфер под расширенную команду
uint8_t newDeviceDescription[DEVICE_DESCRIPTION_MAX_LEN];
uint8_t newDeviceDescriptionLen = 0;



void _canMonitor_sendListOfCommands();
void canMonitor_processDbgHEX(uint8_t *pBuf, uint8_t len);      // обработать отладочную бинарную команду
void canMonitor_processDbgTEXT(uint8_t *pBuf, uint8_t len);     // обработать отладочную тектовую команду
uint8_t cmpStr(uint8_t* pCmd, uint8_t* pData, uint8_t len);
uint8_t continueWith(uint8_t* pCmd, uint8_t* pData, char **end);

// Инициализация
void canMonitor_init()
{
  canmonitorSoftBuffer = canSwBuffer_create(MDR_CAN2, CANMONITOR_QUEUE_CAPACITY, CAN_MONITOR_TX_BUF);
}

// Принят пакет от монитора
void canMonitor_rxFrameHandler(uint8_t *buf, uint8_t len)
{
  switch(buf[0])
  {
     // Запрос состояния 
     case CANMONITOR_STATE_REQUEST:             
       canMonitor_sendStateWord();              // отправить состояние
       canMonitor_sendWorkingTime();            // отправить время наработки
       break;
     
    // Принять расширенную команду
    case CANMONITOR_EXTENDED_CMD:
       canMonitor_rxExtCmd(buf, len);
       break;
       
       
     // Запрос коэффициентов ПИД-регулятора
     case CANMONITOR_PID_KOEF_REQUEST:
       if(len != 1) return;
       canMonitor_sendPIDKoef();                   // коэф. ПИД-регулятора
       break;
    
     // Обновлеие коэффициента P ПИД-регулятора
     case CANMONITOR_PID_KOEF_SET_P:
       {
         if(len != 5) return;
         uint32_t val = (buf[4]<<24 | buf[3]<<16 | buf[2]<<8 | buf[1]);
         newPID_koefP = *((float*)(&val));
       }
       break;
       
     // Обновлеие коэффициента I ПИД-регулятора  
     case CANMONITOR_PID_KOEF_SET_I:
       {
         if(len != 5) return;
         uint32_t val = (buf[4]<<24 | buf[3]<<16 | buf[2]<<8 | buf[1]);
         newPID_koefI = *((float*)(&val));
       }
       break;
     
     // Обновлеие коэффициента D ПИД-регулятора
     case CANMONITOR_PID_KOEF_SET_D:
       {
         if(len != 5) return;
         uint32_t val = (buf[4]<<24 | buf[3]<<16 | buf[2]<<8 | buf[1]);
         newPID_koefD = *((float*)(&val));
       }
       break;
       
     // Обновлеие коэффициента N ПИД-регулятора
     case CANMONITOR_PID_KOEF_SET_N:
       {
         if(len != 5) return;
         uint32_t val = (buf[4]<<24 | buf[3]<<16 | buf[2]<<8 | buf[1]);
         newPID_koefN = *((float*)(&val));
       }
       break;      
     
     // Обновлеие частоты дискретизации ПИД-регулятора
     case CANMONITOR_SET_T_DISKR:
       {
         if(len != 5) return;
         uint32_t val = (buf[4]<<24 | buf[3]<<16 | buf[2]<<8 | buf[1]);
         newPID_tDiskr = *((float*)(&val));

         pid_setNewKoef(newPID_koefP, newPID_koefI, newPID_koefD, newPID_koefN, newPID_tDiskr);
         canMonitor_sendPIDKoef();      // отправить коэф. ПИД-регулятора
       }
       break;

     // Сохранить ПИД-коэффициенты во флешь-памяти контроллера
     case CANMONITOR_SAVE_PID_KOEF_IN_FLASH:
       if(len != 1) return;
       saveUserDataInFlash();
       break;
         
     // Запустить калибровку по скорости
     case CANMONITOR_START_VELOCITY_CALIBRATION:
       {
         gk_setModeVelocityCalibration();
       }
       break;
       
    // Установить режим работы туда-сюда ВУС
    case CANMONITOR_SET_MODE_TUDA_SUDA_VUS:
        if(len!=7) return;
        canMonitor_setModeTudaSudaVUS(buf, len);
        break;
        
    // Установить режим работы туда-сюда АР
    case CANMONITOR_SET_MODE_TUDA_SUDA_AR:
        if(len!=7) return;
        canMonitor_setModeTudaSudaAR(buf, len);
        break;
                   
    // Задать вывод отладочной инфы
    case CANMONITOR_SET_OUTPUT_DEBUGING_INFO:
      {
        if(len!=5) return;
        canMonitor_setOutputDebugingInfo(buf, len);
      }
      break;
    
    // Выключить двигатели
    case CANMONITOR_ENGINE_SWITCH_OFF:
      {
        if(len!=1) return;
        gk_setModeEngineOff();
      }    
      break;          
      
    // Калибровка ДУС по температуре
    case CANMONITOR_SET_MODE_DUS_TEMP_CALIBRATION:
      if(len != 1) return;
      gk_setModeDusTemperatureCalibration();
      break;
  }
}

// Отправить данные CAN-монитору
void canMonitor_send(uint8_t *pData, uint8_t len)
{
#ifndef USE_DYNAMIC_CAN_QUEUE
   can2_send_packet(CAN_MONITOR_TX_BUF, CANID_MONITOR_TX, pData, len);  
#else
   canSwBuffer_enqueue(canmonitorSoftBuffer, pData, len, CANID_MONITOR_TX);
#endif
}


// Отправить слово состояния (текущий режим работы)
void canMonitor_sendStateWord()
{  
  uint8_t msg[8];
  canMonitor_fillBufferWithCurrentState(msg);
  canMonitor_send(msg, sizeof(msg));
}

// Заполнить буфер данными текущего состояния
void canMonitor_fillBufferWithCurrentState(uint8_t *msg)
{
  extern GKControlMode_t gk_controlMode;
  extern int16_t cmdTangageVelocityVUS;
  extern int16_t cmdTangageAngleAR;
  extern int16_t cmdTangageVelocityVUO;
  extern int16_t cmdTangageAngleVUO;
  extern uint16_t modeTudaSudaVUSPeriod;
  extern FuncTestStage_t functionalTestStage;
  extern FuncTestErrorCode_t funcTestErrorCode;
  extern int16_t funcTestArAngle128;
  extern int16_t constantSpeedCode;
  
  msg[0] = CANMONITOR_STATE;
  msg[1] = (uint8_t)gk_controlMode;
  if(gk_controlMode == GkMode_VUS)
  {
    msg[2] = (uint8_t)(cmdTangageVelocityVUS>>8);
    msg[3] = (uint8_t)(cmdTangageVelocityVUS);
  }
  else if(gk_controlMode == GkMode_AR)
  {
    msg[2] = (uint8_t)(cmdTangageAngleAR>>8);
    msg[3] = (uint8_t)(cmdTangageAngleAR);
  }
  else if(gk_controlMode == GkMode_VUO)
  {
    msg[2] = (uint8_t)(cmdTangageAngleVUO>>8);
    msg[3] = (uint8_t)(cmdTangageAngleVUO);
    msg[4] = (uint8_t)(cmdTangageVelocityVUO>>8);
    msg[5] = (uint8_t)(cmdTangageVelocityVUO);
  }
  else if(gk_controlMode == GkMode_TudaSudaVUS)
  {    
    uint8_t *p = (uint8_t*)&tudaSudaVelocityF;
    msg[2] = p[0]; 
    msg[3] = p[1];
    msg[4] = p[2];
    msg[5] = p[3];
    msg[6] = (uint8_t)(modeTudaSudaVUSPeriod>>8);
    msg[7] = (uint8_t)(modeTudaSudaVUSPeriod);
  }
  else if(gk_controlMode == GkMode_SelfControl)
  {
    msg[2] = functionalTestStage; // стадия самоконтроля 
    msg[3] = (uint8_t)(funcTestErrorCode>>8);
    msg[4] = (uint8_t) funcTestErrorCode;
    if(functionalTestStage == FUNCTEST_AR_TEST)
    {
       msg[5] = (uint8_t)(funcTestArAngle128>>8);
       msg[6] = (uint8_t)(funcTestArAngle128);       
    }
    else if(functionalTestStage == FUNCTEST_VUS_TEST)
    {
       msg[5] = (uint8_t)(constantSpeedCode>>8);
       msg[6] = (uint8_t)(constantSpeedCode);       
    }
  }
}

// Отправить время наработки
void canMonitor_sendWorkingTime()
{ 
  extern volatile uint32_t system_time;          // системное время, мс
  uint32_t currentSystemTime = system_time;
  
  uint8_t msg[5];
  msg[0] = CANMONITOR_WORKING_TIME_MS;
  msg[1] = currentSystemTime>>24; 
  msg[2] = currentSystemTime>>16;
  msg[3] = currentSystemTime>>8;
  msg[4] = currentSystemTime;
  canMonitor_send(msg, sizeof(msg));
}


// Принять расширенную команду
void canMonitor_rxExtCmd(uint8_t *pData, uint8_t len)
{  
   static uint16_t index=0;
   uint8_t rxDone = 0;          // завершен прием команды
   uint8_t needAnswer = 0;      // необхо
   
   switch(pData[1] & 0xf)       // проверка типа операции
   {
     case EXT_CMD_START: 
       index = 0;       
       break;
     
     case EXT_CMD_CONT: break;
     
     case EXT_CMD_END:
       rxDone = 1;       
       if(pData[1] & 1<<7) needAnswer = 1;
       break;
       
     default: return;
   }
   
   for(uint8_t i=2; i<len; i++) // первые 2 байта служебные
   {
      if(index == sizeof(extCmdBuf)) break; // защита от переполнения буфера
      extCmdBuf[index++] = pData[i];
   }   
   if(rxDone)
   {        
      // ========== произвести обработку расширенной команды ==========
      if(needAnswer)
      {
        
      }
      else
      {
        
      }
      
      if(extCmdBuf[0] == CANMONITOR_EXT_CMD_STRING_ID) // принята строка
      {
        canMonitor_processDbgTEXT(&extCmdBuf[1], index-1);
      }
      else // приняты набор байт
      {
        canMonitor_processDbgHEX(extCmdBuf, index);
      }
      
      index = 0;
   }
}


// Отладочная команда
void canMonitor_processDbgHEX(uint8_t *pBuf, uint8_t len)
{
  /*
   if((pBuf[0]==2) && (len==1))
   {
       gk_setModeEngineOff();        
       delay_ms(10);
       setPwmFillingF(TIM_PWM_60_PERCENTAGE_CCR_REG/4);
       AH_HI();
       BL_HI();
   }
   else if((pBuf[0]==3) && (len==1))
   {
      extern uint32_t g_engineZeroAngleCorrection;
      extern void saveUserDataInFlash();
      
      g_engineZeroAngleCorrection = g_sysAngle360;
      saveUserDataInFlash();      
   }
   */
  
   // отладка функционального теста
   if((pBuf[0]==0xe) && (len==2))
   {
     extern uint8_t dbg_funcTestCrashed;
     dbg_funcTestCrashed = pBuf[1];
   }
}


// Обработать отладочную тектовую команду
void canMonitor_processDbgTEXT(uint8_t *pBuf, uint8_t len)
{
  pBuf[len] = 0;        // затираем в буфере последний байт, что бы корректно конвертировать число
  char *ptr;            // указатель для смещения по строке
  char *end;            // указатель для функции strtol
  if(cmpStr(DBG_CMD___PING, pBuf, len))
  {
    canMonitor_printf("Ping: Тангажный контур...");
  }
  else if(cmpStr(DBG_CMD___RESET_MK, pBuf, len)) // сброс микроконтроллера
  {
    SYSTEM_RESET();
    while(1);
  }
  if(cmpStr(DBG_CMD___GET_DUS_CORR, pBuf, len)) // прочитать текущую поправку к ДУС
  {
     extern DusCalibrationCoefDestination_t dusCalibCoefDestination;
     char *corrEnableStr = dbg_dusCorrectionEnable ? "Коррекция ДУС включена" : "Коррекия ДУС ВЫКЛЮЧЕНА";
     switch(dusCalibCoefDestination)
     {
       case DUS_CALIB_COEF__RAM:        canMonitor_printf("%s. Поправка к ДУС (ОЗУ) = %d",   corrEnableStr, dusCodeAmmend); break;
       case DUS_CALIB_COEF__FLASH:      canMonitor_printf("%s. Поправка к ДУС (FLASH) = %d", corrEnableStr, dusCodeAmmend); break;
       case DUS_CALIB_COEF__NONE:       canMonitor_printf("%s. Для текущего температурного диапазона нет данных. Поправка к ДУС = %d", corrEnableStr, dusCodeAmmend); break;
       default:                         canMonitor_printf("%s. Ошибка! Неизвестно откуда взявшаяся поправка к ДУС %d", corrEnableStr, dusCodeAmmend ); break;
     }
  }
  if(continueWith(DBG_CMD___SET_DUS_CORR, pBuf, &ptr)) // изменить текущую поправку к ДУС 
  {
     int value = strtol(ptr, &end, 10);
     if(ptr == end) return;
     dusCodeAmmend = (int16_t)value;
     canMonitor_printf("Поправка к ДУС = %d", dusCodeAmmend);     
  }
  if(continueWith(DBG_CMD___SET_DUS_CORR_ENABLE, pBuf, &ptr)) // разрешение поправки ДУС
  {
     int value = strtol(ptr, &end, 10);
     if(ptr == end) return;
     dbg_dusCorrectionEnable = (uint8_t)value;
     if(!dbg_dusCorrectionEnable) dusCodeAmmend = 0;
     canMonitor_printf("%s", dbg_dusCorrectionEnable ? "Коррекция ДУС включена" : "Коррекия ДУС ВЫКЛЮЧЕНА");     
  }

  if(cmpStr(DBG_CMD___GET_SCALES, pBuf, len)) // отправка поправок к аппаратной и логической шкалам тангажного контура
  {     
     canMonitor_printf("\nTangage hardware %.3f" 
                           "\nTangage logical %.3f",
                           USYSANGLE_TO_FLOAT(g_tangageHardwareZeroAngleCorrection), 
                           USYSANGLE_TO_FLOAT(g_tangageLogicalZeroAngleCorrection));
     return;
  }
//  if(continueWith("set max pwm filling", pBuf, &ptr)) // установить максимальное заполнение шим
//  {
//    extern uint16_t PWM_MAX_FILLING;
//    int value = strtol(ptr, &end, 10);
//    if(ptr == end) return;
//    if(value > 60) value = 60;
//    
//    PWM_MAX_FILLING = TIM_PWM_SET_MAX_FILLING(value);
//    canMonitor_printf("max PWM filling = %d%%(%d)", value, PWM_MAX_FILLING);
//  }
  
  if(cmpStr(DBG_CMD___GET_TEMP, pBuf, len))
  {
     extern int16_t dbg_dusTemp;
     extern uint8_t dbg_useTestDusTemp;
     extern int16_t frontSemisphereTemp;  // температура в передней полусфере
     extern int16_t backSemisphereTemp;   // температура в задней полусфере
     extern int16_t ustrSoprTemp;         // температура на плате упрпавления
    
     canMonitor_printf("\nТемпература в передней полусфере %d\n"
                        "Температура в задней полусфере %d\n"
                        "Температура на плате управления %d\n"
                        "Температура ДУС%s %d",                        
                          frontSemisphereTemp,  backSemisphereTemp,  ustrSoprTemp,  
                          dbg_useTestDusTemp ? " (тестовая):" : ":", dusTemperature);     
  }
  // Установить отладочную температуру
  if(continueWith(DBG_CMD___SET_DBG_TEMP, pBuf, &ptr))
  {    
    extern int16_t dbg_dusTemp;
    extern uint8_t dbg_useTestDusTemp;
    extern int dbg_temperatureChangePeriod;
    extern uint8_t dbg_tempChangemode;
    
    int value = strtol(ptr, &end, 10);
    if(ptr == end) return;
    dbg_useTestDusTemp = value;   
    
    if(continueWith(" value ", (uint8_t*)end, &ptr))
    {
       value = strtol(ptr, &end, 10);
       if(ptr != end)
       {
         dbg_dusTemp = value;
       }
       ptr = end;
    }
    if(continueWith(" period ", (uint8_t*)end, &ptr))
    {
       value = strtol(ptr, &end, 10);
       if(ptr != end)
       {
         dbg_temperatureChangePeriod = value;
       }
       ptr = end;
    }
    if(continueWith(" mode ", (uint8_t*)end, &ptr))
    {
       switch(*ptr)
       {
         case '0': dbg_tempChangemode = 0; break;
         case '+': dbg_tempChangemode = 1; break;
         case '-': dbg_tempChangemode = 2; break; 
         case '|': dbg_tempChangemode = 3; break;
       }
    }
    if(!dbg_useTestDusTemp) canMonitor_printf("Использоватение тестовой температуры ДУС отключено");
    else canMonitor_printf("Тестовая температура ДУС %d. Интервал изменения %d. Режим %d", dbg_dusTemp, dbg_temperatureChangePeriod, dbg_tempChangemode);
  }
  
  // Запросить текущую отладочную температуру
  if(cmpStr(DBG_CMD___GET_DBG_TEMP, pBuf, len))
  {
    extern int16_t dbg_dusTemp;
    extern uint8_t dbg_useTestDusTemp;
    extern int dbg_temperatureChangePeriod;
    extern uint8_t dbg_tempChangemode;
    
    if(!dbg_useTestDusTemp)canMonitor_printf("Использоватение тестовой температуры ДУС отключено");
    else canMonitor_printf("Тестовая температура ДУС %d. Интервал изменения %d. Режим %d", dbg_dusTemp, dbg_temperatureChangePeriod, dbg_tempChangemode);
  }
  
  
  // Калибровочные коэффициенты ДУС 
  if(continueWith(DBG_CMD___GET_CALIBRATION_COEF, pBuf,  &ptr))
  {
      extern DusTempCorrection_t dusTemperatureCorrections[DUS_TEMPCALIB_RANGES_COUNT];
      extern DusTempCorrection_t dusDefaultTemperatureCorrections[DUS_TEMPCALIB_RANGES_COUNT];
      DusTempCorrection_t *pCorrectionArray;
      uint32_t elapsedTime;
      int16_t startTemp;
      int16_t endTemp;
      int i;
              
      uint8_t *description;
      if(continueWith(" default", (uint8_t*)ptr, &ptr)) 
      {
        description = "Калибровочные коэффициенты ПО УМОЛЧАНИЮ:";
        pCorrectionArray = dusDefaultTemperatureCorrections;
      }
      else 
      {
        description = "Калибровочные коэффициенты:";
        pCorrectionArray = dusTemperatureCorrections;
      }      
      canMonitor_printf(description);
             
      startTemp = DUS_TEMPCALIB_MIN_TEMP;
      elapsedTime = system_time - pCorrectionArray[0].time;
      if(pCorrectionArray[0].available) 
      {
        if(pCorrectionArray[0].time == 0) canMonitor_printf("   [<%3d°]    k = %d",                startTemp, pCorrectionArray[0].correction);
        else                              canMonitor_printf("   [<%3d°]    k = %d  (%dмс назад)",  startTemp, pCorrectionArray[0].correction, elapsedTime);
      }
      else                                canMonitor_printf("   [<%3d°]    нет данных ",           startTemp);
      
      for(i=1; i<DUS_TEMPCALIB_RANGES_COUNT-1; i++)
      {
        startTemp = DUS_TEMPCALIB_MIN_TEMP + (i-1)*DUS_TEMPCALIB_TEMPERATURE_STEP;
        endTemp = startTemp + DUS_TEMPCALIB_TEMPERATURE_STEP;
        elapsedTime = system_time - pCorrectionArray[i].time;
        if(pCorrectionArray[i].available)
        {
          if(pCorrectionArray[i].time == 0) canMonitor_printf("[%3d° - %3d°] k = %d",               startTemp, endTemp, pCorrectionArray[i].correction);
          else                              canMonitor_printf("[%3d° - %3d°] k = %d  (%dмс назад)", startTemp, endTemp, pCorrectionArray[i].correction, elapsedTime);
        }
        else                                canMonitor_printf("[%3d° - %3d°] нет данных",           startTemp, endTemp);
      }    
      startTemp = DUS_TEMPCALIB_MIN_TEMP + (i-1)*DUS_TEMPCALIB_TEMPERATURE_STEP;
      elapsedTime = system_time - pCorrectionArray[i].time;
      if(pCorrectionArray[i].available) 
      {
        if(pCorrectionArray[i].time == 0) canMonitor_printf("   [>%3d°]    k = %d",               startTemp, pCorrectionArray[i].correction);
        else                              canMonitor_printf("   [>%3d°]    k = %d  (%dмс назад)", startTemp, pCorrectionArray[i].correction, elapsedTime);
      }      
      else                                canMonitor_printf("   [>%3d°]    нет данных ",          startTemp);    
  }
  
  
    // Динамическая память
  if(cmpStr(DBG_CMD___DYNAMIC_RAM_STATISTIC, pBuf, len))
  {
    extern DataQueue_t canTextQueue;
         
    uint16_t monitorMaxMsg = canmonitorSoftBuffer->bdg_maxCount;
    uint16_t monitorQueueCount = canmonitorSoftBuffer->queue->count;
    uint16_t monitorQueueCapacity = canmonitorSoftBuffer->queue->capacity;

    uint16_t textQueueMaxMsg =  canTextQueue.dbg_maxCount;
    uint16_t textQueueCount = canTextQueue.count;
    uint16_t textQueueCapacity = CAN_QUEUE_TEXT_SIZE;
    
    canMonitor_printf("\nmonitor: %d/%d. Max %d"
                      "\ntext:    %d/%d. Max %d",
                      monitorQueueCount, monitorQueueCapacity, monitorMaxMsg,
                      textQueueCount,    textQueueCapacity,    textQueueMaxMsg);
  }
    
  // Отправить список команд
  if(cmpStr(DBG_CMD___HELP, pBuf, len))
  {
     _canMonitor_sendListOfCommands();
  }  
  
  // логгирование калибровки в арретире
  if(continueWith(DBG_CMD___SET_ENABLE_AR_CALIB_LOG, pBuf, &ptr))
  {
    int value = strtol(ptr, &end, 10);
    if(ptr == end) return;
    extern uint8_t enableDusArreiterCalibrationLog;
    enableDusArreiterCalibrationLog = value;
    canMonitor_printf("Разрешение логгирование калибровки ДУС в арретире = %d", enableDusArreiterCalibrationLog);
  }
  
  // Разрешение вывода текущей температуры
  if(continueWith(DBG_CMD___SET_ENABLE_PRINT_CURRENT_TEMP, pBuf, &ptr))
  {
    int value = strtol(ptr, &end, 10);
    if(ptr == end) return;
    extern uint8_t printCurrentTempEnable;
    printCurrentTempEnable = value;
    canMonitor_printf("Разрешение вывода текущей температуры = %d", printCurrentTempEnable);
  }
}

// Отправить список команд
void _canMonitor_sendListOfCommands()
{
   canMonitor_printf("===== LIST OF COMMANDS =====");
   canMonitor_printf("%s", DBG_CMD___PING);
   canMonitor_printf("%s", DBG_CMD___RESET_MK);
   canMonitor_printf("%s", DBG_CMD___GET_DUS_CORR);
   canMonitor_printf("%s", DBG_CMD___SET_DUS_CORR);
   canMonitor_printf("%s", DBG_CMD___SET_DUS_CORR_ENABLE);
   canMonitor_printf("%s", DBG_CMD___GET_SCALES);
   canMonitor_printf("%s", DBG_CMD___GET_CALIBRATION_COEF);
   canMonitor_printf("%s", DBG_CMD___GET_CALIBRATION_COEF_DEFAULT);
   canMonitor_printf("%s", DBG_CMD___GET_TEMP);
   canMonitor_printf("%s", DBG_CMD___GET_DBG_TEMP);
   canMonitor_printf("%s", DBG_CMD___SET_DBG_TEMP__PATTERN);
   canMonitor_printf("%s", DBG_CMD___DYNAMIC_RAM_STATISTIC);
   canMonitor_printf("%s", DBG_CMD___SET_ENABLE_AR_CALIB_LOG);
   canMonitor_printf("%s", DBG_CMD___SET_ENABLE_PRINT_CURRENT_TEMP);
}


// Сравнить принятое сообщение со строкой. Возвращает 1, если строки равны
uint8_t cmpStr(uint8_t* pCmd, uint8_t* pData, uint8_t len)
{
    int i=0;
    while(pCmd[i] != 0)
    {
       if(pCmd[i] == pData[i])
       {
          i++;
          continue;
       }
       else return 0;
    }
    if(i != len) return 0;      // проверка длины
    else return 1;
}

// Проверить, что принятое сообщение начинается с pCmd[]
// Через **end возвращается указатель на первый символ после pCmd
uint8_t continueWith(uint8_t* pCmd, uint8_t* pData, char **end)
{
    int i=0;
    while(pCmd[i] != 0)
    {
       if(pCmd[i] == pData[i])
       {
          i++;
          continue;
       }
       else return 0;
    }
    *end = (char*)pData+i;
    return 1;
}
       



// Отправить ответ на расширенную команду 
void canMonitor_sendExtHex(uint8_t *pBuf, uint8_t len)
{
   uint8_t msg[8];
   const uint8_t MAX_DATA_BYTES = 6;
   uint8_t needToSend = 0;          // сколько байт осталось отправить
   uint8_t sendedBytes = 0;         // кол-во отправленных байт данных
   uint8_t dataBytesInFrame = 0;    // кол-во байт в передаваемом пакете
   uint8_t indx = 0;                // индекс

   msg[0] = CANMONITOR_EXTENDED_ANSWER;

   
   while(sendedBytes != len)
   {
      needToSend = len - sendedBytes;
      if(needToSend <= MAX_DATA_BYTES) // если оставшиеся данные влезают в один пакет - то он последний
      {
          msg[1] = EXT_CMD_END;
      }
      else
      {
          if(sendedBytes == 0) msg[1] = EXT_CMD_START;
          else msg[1] = EXT_CMD_CONT;
      }

      dataBytesInFrame = needToSend <= MAX_DATA_BYTES ? needToSend : MAX_DATA_BYTES;
      sendedBytes += dataBytesInFrame;

      for(uint8_t i=0; i<dataBytesInFrame; i++)
      {
         msg[i+2] = pBuf[indx++];
      }
      canMonitor_send(msg, dataBytesInFrame+2);  
   }
}

#ifndef USE_DYNAMIC_CAN_QUEUE
// Отправить строку
void canMonitor_printf(uint8_t *p, ...)
{
  uint8_t buf[256];
  buf[0] = CANMONITOR_EXT_CMD_STRING_ID;
  
  va_list args;
  va_start(args, p);
  //int len = vsprintf((char*)&buf[1], (char*)p, args);
  int len = vsnprintf((char*)&buf[1], sizeof(buf), (char*)p, args);
  va_end(args);
 
  if(len<0) return;  
  canMonitor_sendExtHex(buf, len+1);
}
#endif//USE_DYNAMIC_CAN_QUEUE






// Отправить текущую курсовую скорость
void canMonitor_sendCourseVelocity()
{
  extern uint16_t dusCode;
  extern int16_t dusAmplitude;
  extern float averageVelocityFromAngle;
  
  int16_t _dusAmpl = dusAmplitude;
  float aveVel = averageVelocityFromAngle;
  uint8_t *p = (uint8_t*)&aveVel;  
  uint8_t msg[7];
  msg[0] = CANMONITOR_COURSE_VELOCITY;
  msg[1] = (_dusAmpl>>8);
  msg[2] = (uint8_t)(_dusAmpl);  
  msg[3] = p[0]; 
  msg[4] = p[1];
  msg[5] = p[2];
  msg[6] = p[3];
  canMonitor_send(msg, sizeof(msg));
}

// Отправить значение управляющего сигнала
void canMonitor_sendCourseUpr()
{
  extern float lastPidUprValue;
  uint8_t *p = (uint8_t*)&lastPidUprValue;  
  
  uint8_t msg[5];
  msg[0] = CANMONITOR_COURSE_UPR;
  msg[1] = p[0]; 
  msg[2] = p[1];
  msg[3] = p[2];
  msg[4] = p[3];
  canMonitor_send(msg, sizeof(msg));
}

// Отправить значения коэффициентов ПИД-регулятора
void canMonitor_sendPIDKoef()
{  
  uint8_t msg[5];  
  uint8_t *p = (uint8_t*)&koef_P;
  
  msg[0] = CANMONITOR_COURSE_GET_P;
  msg[1] = p[0]; 
  msg[2] = p[1];
  msg[3] = p[2];
  msg[4] = p[3];
  canMonitor_send(msg, sizeof(msg));
  
  p = (uint8_t*)&koef_I;
  msg[0] = CANMONITOR_COURSE_GET_I;
  msg[1] = p[0]; 
  msg[2] = p[1];
  msg[3] = p[2];
  msg[4] = p[3];
  canMonitor_send(msg, sizeof(msg));
  
  p = (uint8_t*)&koef_D;
  msg[0] = CANMONITOR_COURSE_GET_D;
  msg[1] = p[0]; 
  msg[2] = p[1];
  msg[3] = p[2];
  msg[4] = p[3];
  canMonitor_send(msg, sizeof(msg));
 
  p = (uint8_t*)&koef_N;
  msg[0] = CANMONITOR_COURSE_GET_N;
  msg[1] = p[0]; 
  msg[2] = p[1];
  msg[3] = p[2];
  msg[4] = p[3];
  canMonitor_send(msg, sizeof(msg));  
  
  p = (uint8_t*)&tDiskr;
  msg[0] = CANMONITOR_COURSE_GET_T;
  msg[1] = p[0]; 
  msg[2] = p[1];
  msg[3] = p[2];
  msg[4] = p[3];
  canMonitor_send(msg, sizeof(msg));
}


// Отпрвить значение угла
void canMonitor_sendAngle()
{
  uint32_t angle = g_sysAngle360;
  uint8_t msg[5];
  msg[0] = CANMONITOR_SEND_CURRENT_ANGLE;
  msg[1] = angle>>24; 
  msg[2] = angle>>16;
  msg[3] = angle>>8;
  msg[4] = angle;
  canMonitor_send(msg, sizeof(msg));
}

// Отпрвить тестовое значение 1
void canMonitor_sendTestValue1(float value)
{
  uint8_t *p = (uint8_t*)&value;
  uint8_t msg[5];
  msg[0] = CANMONITOR_SEND_TEST_VALUE_1;
  msg[1] = p[0]; 
  msg[2] = p[1];
  msg[3] = p[2];
  msg[4] = p[3];
  canMonitor_send(msg, sizeof(msg));
}

// Установить режим работы туда-сюда
void canMonitor_setModeTudaSudaVUS(uint8_t *buf, uint8_t len)
{
  uint16_t period = buf[6]<<8 | buf[5];
  uint32_t velI = (buf[4]<<24 | buf[3]<<16 | buf[2]<<8 | buf[1]);
  tudaSudaVelocityF = *((float*)(&velI));
  int16_t vel32 = (int16_t)(tudaSudaVelocityF*32);   // с шагом 1/32 градуса/сек
  
  gk_setModeTudaSudaVUS(period, vel32);
}

// Установить режим работы туда-сюда АР
void canMonitor_setModeTudaSudaAR(uint8_t *buf, uint8_t len)
{
  uint16_t period = buf[1]<<8 | buf[2];
  int16_t cmdAngle128_1 = (int16_t)(buf[3]<<8 | buf[4]);
  int16_t cmdAngle128_2 = (int16_t)(buf[5]<<8 | buf[6]);
    
  uint32_t angle1 = angle_convertCmdS2SysU(cmdAngle128_1, g_tangageHardwareZeroAngleCorrection, MAX_SYSANGLE_CODE);
  uint32_t angle2 = angle_convertCmdS2SysU(cmdAngle128_2, g_tangageHardwareZeroAngleCorrection, MAX_SYSANGLE_CODE);
    
  gk_setModeTudaSudaAR(period, angle1, angle2);
}


// Задать вывод отладочной информации
void canMonitor_setOutputDebugingInfo(uint8_t *buf, uint8_t len)
{
  extern uint8_t debugInfo_globalEnable;     // общее разрешение отправки отладочной информации
  extern uint8_t debugInfo_velocityEnable;   // передача скорости
  extern uint8_t debugInfo_uprEnable;        // передача управляющего сигнала
  extern uint8_t debugInfo_angleEnable;      // передача угла
  extern uint8_t debugInfo_testValue1Enable; // передача тестового значения 1

  uint32_t word = buf[4]<<24 | buf[3]<<16 | buf[2]<<8 | buf[1];
  
  debugInfo_globalEnable = word & 1;
  debugInfo_velocityEnable = (word>>1) & 1;
  debugInfo_uprEnable = (word>>2) & 1;
  debugInfo_angleEnable = (word>>3) & 1;
  debugInfo_testValue1Enable = (word>>4) & 1;
}

// Отправить сохраненное значение для вывода его в текстовое поле и тд.
void canMonitor_sendDbgSavedValue(float value)
{
  uint8_t *p = (uint8_t*)&value;
  uint8_t msg[5];
  msg[0] = CANMONITOR_DBG_SEND_SAVED_VALUE;
  msg[1] = p[0]; 
  msg[2] = p[1];
  msg[3] = p[2];
  msg[4] = p[3];
  canMonitor_send(msg, sizeof(msg));
}
