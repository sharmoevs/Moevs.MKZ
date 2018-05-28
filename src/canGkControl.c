#include "canGkControl.h"
#include "can.h"
#include "global.h"
#include "angleSensor.h"
#include "canMonitor.h"


// Для монитора состояния
int16_t cmdTangageVelocityVUS = 0; // заданная скорость в режиме ВУС с шагом 1/128 градуса
int16_t cmdTangageAngleAR = 0;     // заданный угол в режиме АР с шагом 1/128
int16_t cmdTangageVelocityVUO = 0; // заданная скорость в режиме ВУО с шагом 1/128
int16_t cmdTangageAngleVUO = 0;    // заданный угол в режиме ВУО с шагом 1/128
int16_t funcTestArAngle128;        // угол на который сейчас встает ГК при самоконтроле

// Калибровка ДУС по включению питания
//uint8_t startUpCalibrationAngleReceived = 0; // был получен стартовый угол калибровки
//uint32_t startUpCalibrationAngle;            // стартовый угол на который нужно встать

// тест шины CAN
uint8_t canBusTest_enable = 0;
uint32_t canBusTest_expectValue = 0;
uint32_t canBusTest_errorCount = 0;

extern GKControlMode_t          gk_controlMode;
extern FuncTestStage_t          functionalTestStage;
extern FuncTestErrorCode_t      funcTestErrorCode;
extern uint32_t                 arretierRequiredAngleU32;
extern uint32_t                 g_tangageHardwareZeroAngleCorrection; // ноль аппаратной шкалы тангажного контура
extern uint32_t                 g_tangageLogicalZeroAngleCorrection;  // ноль логической шкалы тангажного контура



void setStartUpCalibrationAngle(uint32_t angle);
void canGkControl_send(uint8_t *pData, uint8_t len);
void canGkControl_setSelfControlSubMode(uint8_t *buf, uint8_t len);   // установить режим самоконтроля
void canBusTest_check(uint8_t *pBuf, uint8_t len);


// Отправить данные по CAN
void canGkControl_send(uint8_t *pData, uint8_t len)
{
   ////can2_send_packet(CAN_GK_CONTROL_TX_BUF, CANID_GK_CONTROL_TX, pData, len);  
  can2_putDataInBuf(CAN_GK_CONTROL_TX_BUF, CANID_GK_CONTROL_TX, pData, len);  
}



// Принять сообщение
void canGkControl_rxMsg(uint8_t* buf, uint8_t len)
{
  switch(buf[0])
  {
      case TANGAGE_CTRL_SETMODE_VUS:              // ВУС
      {
        if(len!=3) return;
        cmdTangageVelocityVUS = (int16_t)((buf[1]<<8) | buf[2]);
        int16_t velocityCode32 = cmdTangageVelocityVUS>>2;
        gk_setModeVUS(velocityCode32);
      }
      break;

      case TANGAGE_CTRL_SETMODE_AR:               // Арретир
      {
        if(len!=6) return;
        uint32_t angleU = (uint32_t)((buf[1]<<16) | (buf[2]<<8) | buf[3]);
        cmdTangageAngleAR = (int16_t)((buf[4]<<8) | buf[5]);
        gk_setModeAR(angleU);
      }
      break;

    case TANGAGE_CTRL_SETMODE_ENGINE_OFF:         // Останов
      {
        if(len!=1) return;
        gk_setModeEngineOff();
      }
      break;
      
    case TANGAGE_CTRL_SETMODE_DISABLE_ENGINE:     // запрет управлениями двигателями
        if(len!=1) return;
        gk_setModeDisableEngineAtStartup();
      break;
      
    case TANGAGE_CTRL_SETMODE_TP:                 // транспортное положение
      {
        if(len!=5) return;
        uint32_t angleU = (uint32_t)((buf[1]<<24) | (buf[2]<<16) | (buf[3]<<8) | buf[4]);
        gk_setModeTP(angleU);
      }
      break;
      
    case TANGAGE_CTRL_SETMODE_VUO:                // ВУО
      {
        if(len!=8) return;
        uint32_t angleU = (uint32_t)((buf[1]<<16 | buf[2]<<8 | buf[3]));
        cmdTangageAngleVUO = (int16_t)((buf[4]<<8) | buf[5]);
        cmdTangageVelocityVUO = (int16_t)(buf[6]<<8 | buf[7]);
        int16_t velocityCode32 = cmdTangageVelocityVUO>>2;
        gk_setModeVOU(angleU, velocityCode32);
      }
      break;
      
    case TANGAGE_CTRL_ANGLE_CORRECTIONS:
      {
        if(len!=7) return;
        g_tangageHardwareZeroAngleCorrection = (uint32_t)((buf[1]<<16 | buf[2]<<8 | buf[3]));
        g_tangageLogicalZeroAngleCorrection = (uint32_t)((buf[4]<<16 | buf[5]<<8 | buf[6]));
        //setStartUpCalibrationAngle(angleU);
      }
      break;
    
      
    case TANGAGE_CTRL_SETSUBMODE_SELFCONTROL:      // Самоконтроль
      {
        canGkControl_setSelfControlSubMode(buf, len);
      }
      break;
      
    case TANGAGE_CTRL_CAN_BUS_TEST:  // тест шины CAN 
      if((buf[1] == 1) && (len == 2)) 
      {
        canBusTest_start();
      }
      else if((buf[1] == 0) && (len == 2)) 
      {
        canBusTest_stop();
      }
      else 
      {
        canBusTest_check(buf, len); 
      }
      break;
  }
}



// Отправить угловую скорость
void canGkControl_sendCurrentState()
{
  extern int16_t dusAmplitude;
  extern GKControlMode_t gk_controlMode;
  extern int16_t dusTemperature;
  
  int16_t _dusAmplitude = dusAmplitude;
  uint8_t buf[5];
  buf[0] = (uint8_t)(_dusAmplitude>>8);
  buf[1] = (uint8_t)_dusAmplitude;
  buf[2] = (uint8_t)gk_controlMode;
  buf[3] = (uint8_t)(dusTemperature>>8);
  buf[4] = (uint8_t)dusTemperature;
  canGkControl_send(buf, sizeof(buf));
}

// Установить режим самоконтроля
void canGkControl_setSelfControlSubMode(uint8_t *buf, uint8_t len)
{
  FuncTestStage_t stage = (FuncTestStage_t)buf[1];
  //uint32_t angleU = (uint32_t)((buf[2]<<16 | buf[3]<<8 | buf[4]));
  //int16_t velocity32 = (int16_t)(buf[5]<<8 | buf[6]);
  //uint8_t numOfAngles = buf[7];
  
  switch(stage)
  {
    case FUNCTEST_INIT:      // инициализация
        if(len != 2) return;
        gk_setConstVelocityCode(CONVERT_VELOCITY_TO_CODE32(1));
        gk_setModeSelfControl();
      break;
      
    case FUNCTEST_VUS_TEST:      // ВУС
        {
          if(len != 5) return;
          if(gk_controlMode != GkMode_SelfControl) return;
          
          functionalTestStage = stage;
          int16_t velocity32 = (int16_t)(buf[3]<<8 | buf[4]);
          gk_setConstVelocityCode(velocity32);
        }
      break;
      
    case FUNCTEST_AR_TEST:        // АР    
        {
          if(len != 8) return;
          if(gk_controlMode != GkMode_SelfControl) return;
          
          functionalTestStage = stage;
          arretierRequiredAngleU32 = (uint32_t)((buf[3]<<16 | buf[4]<<8 | buf[5]));
          funcTestArAngle128 = (int16_t)(buf[6]<<8 | buf[7]);
        }
      break;
      
      case FUNCTEST_COMPLETE_SUCCESS: // Функциональный тест завершен успешно
          if(len != 4) return;
          if(gk_controlMode != GkMode_SelfControl) return;
          //gk_setModeVUS(0);   // ======================================================== ОТЛАДКА!!!                                                  
          functionalTestStage = stage;
      break;
      
      case FUNCTEST_COMPLETE_WITH_ERROR: // Функциональный тест завершен с ошибкой
          if(len != 4) return;
          if(gk_controlMode != GkMode_SelfControl) return;
          functionalTestStage = stage;
          funcTestErrorCode = (FuncTestErrorCode_t)(buf[2]<<8 | buf[3]);
          gk_setConstVelocityCode(0);
      break;
  }
}
/*
// Установить начальный угол калибрвоки
void setStartUpCalibrationAngle(uint32_t angle)
{
  if(startUpCalibrationAngleReceived) return;
  startUpCalibrationAngleReceived = 1; // был получен стартовый угол калибровки
  startUpCalibrationAngle = angle;           // стартовый угол на который нужно встать
  canMonitor_sendString("received start up angle");
}
*/

// Начать тест шины CAN
void canBusTest_start()
{
  canBusTest_enable = 1;
  canBusTest_expectValue = 0;
  canBusTest_errorCount = 0;
  canMonitor_printf("CANTEST started");
}

// Завершить тест шины CAN
void canBusTest_stop()
{
  canBusTest_enable = 0;
  canMonitor_printf("CANTEST stopped. Errors: %d. Last val %d", canBusTest_errorCount, canBusTest_expectValue);
}

// Тест шины CAN
void canBusTest_check(uint8_t *pBuf, uint8_t len)
{
  static uint32_t lastReportTime = 0; // время последней отправки сообщения об ошибке (отсылаем редко, что бы не забивать шину)
  if(!canBusTest_enable) return;
  
  if(len != 5) 
  {
    canMonitor_printf("CANERR: incorrect frameLen");
    return;
  }
  uint32_t rxValue = (uint32_t)(pBuf[1]<<24 | pBuf[2]<<16 | pBuf[3]<<8 | pBuf[4]);
  
  if(canBusTest_expectValue != rxValue)
  {
    canBusTest_errorCount++;
    if(system_time - lastReportTime >= 1000)
    {
      canMonitor_printf("CANERR: rx: %x, expect: %x, errs: %d", rxValue, canBusTest_expectValue, canBusTest_errorCount);
      lastReportTime = system_time;
    }
    
    canBusTest_expectValue = rxValue+1;
  }
  else 
  {
    canBusTest_expectValue++;
  }
}
