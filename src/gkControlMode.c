#include "gkControlMode.h"
#include "global.h"
#include "engineControl.h"
#include "pidRegulator.h"
#include "timers.h"
#include "canMonitor.h"
#include "angleSensor.h"



//#define DBG_START_IN_ENGINE_OFF_MODE        // ДЛЯ ОТЛАДКИ - запуститься с выключенными двигателями


extern int16_t dusAmplitude;
extern uint32_t g_sysAngle360;
extern uint32_t g_engineZeroAngleCorrection;
extern uint32_t g_tangageHardwareZeroAngleCorrection; // ноль аппаратной шкалы тангажного контура
extern uint32_t g_tangageLogicalZeroAngleCorrection;  // ноль логической шкалы тангажного контура

#ifdef DBG_START_IN_ENGINE_OFF_MODE
  GKControlMode_t gk_controlMode = GkMode_EngineOffByCommand;    // режим управления ГК
#else
  GKControlMode_t gk_controlMode = GkMode_Initialize;    // режим управления ГК
#endif

int16_t constantSpeedCode = 0;          // код АЦП, соответствующей желаемой постоянной скорости вращения

float     arretier_AngleToVelKoef = ((ARRETIR_VELOCITY*32.0)/AR_THRESHOLD); // коэф. преобразования угла в скорость 640 - код скорости 20гр/сек, 1456 - код 2градусов с датчика угла
uint32_t  arretierRequiredAngleU32 = 0;  // арретир, в 16разрядной СС для управления по углу


uint8_t  dusSpeedCalibrationStage;      // стадия калибровки по скорости
double   newDusCalibrationKoef;         // новый коэф. пропорциональности ДУС

uint16_t modeTudaSudaVUSPeriod;            // время вращение в одну сторону в режиме Туда-Сюда
int16_t  modeTudaSudaVUSVelocityCode;      // код скорости вращения в режиме Туда-Сюда. Шаг 1/32 градуса

uint16_t modeTudaSudaARPeriod;
uint32_t modeTudaSudaARAngle1;
uint32_t modeTudaSudaARAngle2;

float    averageVelocityFromAngle;      // средняя сокрость, рассчитанная по углу

VUOStage_t vuoStage;                    // стадия ВУО
FuncTestStage_t functionalTestStage;    // стадия функционального теста
FuncTestErrorCode_t funcTestErrorCode;  // код ошибки функционального теста, для определения какой тест выдал ошибку
uint8_t dbg_funcTestCrashed = 0;                // ОТЛАДКА - поломка функционального теста


// Управление
void _gk_nextInitialize();
void _gk_nextAR();
void _gk_nextVUS();
void _gk_nextVelocityCalibration();
void _gk_nextTudaSudaVUS();
void _gk_nextTudaSudaAR();
void _gk_nextVUO();
void _gk_nextSelfControl();

uint8_t  _gk_canChangeMode();           // возможность сменить режим работы
void     _gk_spinVus(int16_t desiredDusCode);
void     gk_PidRegulatorSpin(float upr);
void     setPwmFromUprCode(float uprCode);


void gk_moveNext()
{
  switch(gk_controlMode)
  {  
    case GkMode_Initialize:             _gk_nextInitialize(); break;            // Инициализация
    case GkMode_VUS:                    _gk_nextVUS(); break;                   // ВУС      
    case GkMode_AR:                     _gk_nextAR(); break;                    // арретир
    case GkMode_VelocityCalibration:    _gk_nextVelocityCalibration(); break;   // калибровка
    case GkMode_TudaSudaVUS:            _gk_nextTudaSudaVUS(); break;           // туда-сюда ВУС
    case GkMode_TudaSudaAR:             _gk_nextTudaSudaAR(); break;            // туда-сюда АР
    case GkMode_TP:                     _gk_nextAR(); break;                    // транспортное положение                            
    case GkMode_VUO:                    _gk_nextVUO(); break;                   // ВУО
    case GkMode_SelfControl:            _gk_nextSelfControl(); break;           // самоконтроль
    case GkMode_DUSTemperatureCalibration: _gk_nextAR(); break;                 // калибровка по температуре
    
    case GkMode_EngineDisabled:
    case GkMode_EngineOffBySpeedProtection:
    case GkMode_EngineOffByCommand:
      break;
  }
}

// Защита по скорости
void gk_checkSpeedProtection()
{
  static uint8_t protectionWorked = 0;
  static uint8_t numOfThresholdExceeded = 0;    // количество превышений порога
  if(!angle_getAverageVelocity(&averageVelocityFromAngle)) return;
  if(protectionWorked) return;
  
  // Получено новое значение средней скорости по углу
  if(ABS(averageVelocityFromAngle) >= VELOCITY_PROTECTION_THRESHOLD)
  {
     numOfThresholdExceeded++;
     if(numOfThresholdExceeded >= 3)
     {
       protectionWorked = 1;
       gk_setModeEngineOffBySpeedProtection();       
     }
  }
  else 
  {
    if(numOfThresholdExceeded != 0) numOfThresholdExceeded--;
  }
}

// =============================================================================
// =============================================================================
// ============================ Установка режима работы ========================
// =============================================================================
// =============================================================================

// Возвращает 1, если можно сменить режим работы, в противном случае 0
uint8_t _gk_canChangeMode()
{
  if(gk_controlMode == GkMode_EngineDisabled || 
     gk_controlMode == GkMode_EngineOffBySpeedProtection)
  {
    return 0;
  }
  
  return 1;
}

// Установить запрет управления двигателями по включению питания
void gk_setModeDisableEngineAtStartup()
{
  gk_controlMode = GkMode_EngineDisabled;
  pid_setEnable(0);
  engine_disableSpin();
}

// Выключить двигатели после срабатывания защиты по скорости
void gk_setModeEngineOffBySpeedProtection()
{
  extern int8_t prevSectorCW;
  extern int8_t prevSectorCCW;  
  gk_controlMode = GkMode_EngineOffBySpeedProtection;
  pid_setEnable(0);
  engine_disableSpin();
  prevSectorCW = -1;
  prevSectorCCW = -1;
}

// Встать на определенный угол
void gk_setModeAR(uint32_t angleU)                                          //*****
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  gk_controlMode = GkMode_AR;
  arretierRequiredAngleU32 = angleU;
  __enable_irq();
  pid_setEnable(1);
}

// Установить режим управления по угловым скоростям.
// velocityCode32 - угловая скорость с шагом 1/32 градуса
void gk_setModeVUS(int16_t velocityCode32)
{
  if(!_gk_canChangeMode()) return;

  __disable_irq();
  gk_controlMode = GkMode_VUS;
  gk_setConstVelocityCode(velocityCode32);
  __enable_irq();
  pid_setEnable(1);
}


// Установить режим калибровки по угловым скоростям
void gk_setModeVelocityCalibration()
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  gk_controlMode = GkMode_VelocityCalibration;
  dusSpeedCalibrationStage = 0;
  __enable_irq();
  pid_setEnable(1);
}

// Движение туда-сюда ВУС
void gk_setModeTudaSudaVUS(uint16_t period, int16_t velocityCode32)
{  
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  gk_controlMode = GkMode_TudaSudaVUS;  
  modeTudaSudaVUSPeriod = period;
  modeTudaSudaVUSVelocityCode = velocityCode32;
  pid_setEnable(1);
  __enable_irq();
}

// Движение туда-сюда АРРЕТИР
// angle1, angle2 - углы арретирования
void gk_setModeTudaSudaAR(uint16_t period, uint32_t angle1, uint32_t angle2)
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  gk_controlMode = GkMode_TudaSudaAR;
  modeTudaSudaARPeriod = period;
  modeTudaSudaARAngle1 = angle1;
  modeTudaSudaARAngle2 = angle2;
  pid_setEnable(1);
  __enable_irq();
}

// Выключение двигателей
void gk_setModeEngineOff()
{  
  if(!_gk_canChangeMode()) return;
  
  extern int8_t prevSectorCW;
  extern int8_t prevSectorCCW;
  
  gk_controlMode = GkMode_EngineOffByCommand;
  pid_setEnable(0);
  engine_disableSpin();
  prevSectorCW = -1;
  prevSectorCCW = -1;
}

// Транспортное положение
void gk_setModeTP(uint32_t angleU)
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  gk_controlMode = GkMode_TP;
  arretierRequiredAngleU32 = angleU;
  __enable_irq();
  pid_setEnable(1);
}

// ВУО - внешнее управление ориентацией
// velocityCode32 - угловая скорость с шагом 1/32 градуса
void gk_setModeVOU(uint32_t angleU, int16_t velocityCode32)
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  vuoStage = VUO_INIT;    // Инициализация режима ВУО
  gk_controlMode = GkMode_VUO;
  arretierRequiredAngleU32 = angleU;
  gk_setConstVelocityCode(velocityCode32);
  __enable_irq();
  pid_setEnable(1);
}

// Самоконтроль
void gk_setModeSelfControl()
{
  if(!_gk_canChangeMode()) return;
  
  __disable_irq();
  functionalTestStage = FUNCTEST_INIT;    // Инициализация режима Самоконтроль
  gk_controlMode = GkMode_SelfControl;  
  __enable_irq();
  pid_setEnable(1);
}

// Калибровка ДУС по температуре
void gk_setModeDusTemperatureCalibration()
{
  if(!_gk_canChangeMode()) return;
    
  __disable_irq();
  gk_controlMode = GkMode_DUSTemperatureCalibration;
  arretierRequiredAngleU32 = g_sysAngle360;
  __enable_irq();
  pid_setEnable(1);
}

// =============================================================================
// =============================================================================
// ================================= Режимы работы =============================
// =============================================================================
// =============================================================================

// Инициализация
void _gk_nextInitialize()
{
  static uint8_t stage = 0;    
  switch(stage)
  {
    case 0:
      {
        if(system_time < ENGINE_STARTUP_DELAY) return;        
        arretierRequiredAngleU32 = angle_addCorrection(0, g_tangageHardwareZeroAngleCorrection, MAX_SYSANGLE_CODE);
        stage = 1;
      }
      break;
      
    case 1:
        _gk_nextAR();  
      break;
  }  
}

// Арретир
void _gk_nextAR()                                                               // ***********
{
  uint32_t startAngle = g_sysAngle360;
  uint32_t stopAngle = arretierRequiredAngleU32;
  int32_t deltaFi = angle_getMinimalAngleRange(startAngle, stopAngle);

  if((deltaFi>=0) && (deltaFi>AR_THRESHOLD)) deltaFi = AR_THRESHOLD;
  if((deltaFi<0) && (deltaFi<-AR_THRESHOLD)) deltaFi = -AR_THRESHOLD;
    
  float x = (float)deltaFi * arretier_AngleToVelKoef;
  float upr = pid_nextCode(x, dusAmplitude);
  gk_PidRegulatorSpin(upr);  // управление движением
    
  // отладка
  startAngle++;
  stopAngle++;
  deltaFi++;
  upr++;
}

// ВУС
void _gk_nextVUS()
{
  _gk_spinVus(constantSpeedCode);
}

// Калибровка скорости
void _gk_nextVelocityCalibration()
{
  static uint32_t startCalibrationTime; // время начала калибровки
  static uint32_t startTurnTime;        // время начала круга
  static uint32_t startAngle;
  static uint8_t numOfCircle = 0;
    
  switch(dusSpeedCalibrationStage)
  {
    case 0:     // Начало калибровки
      {
        gk_setConstVelocityCode(CONVERT_VELOCITY_TO_CODE32(GK_VEL_CALIBRATION_SPEED));
        startCalibrationTime = system_time;
        numOfCircle = 1;
        dusSpeedCalibrationStage = 1;
      }
      break;
      
      
    case 1:     // первые несколько секунд вращения, что бы набрать скорость
      {
        if(elapsed(&startCalibrationTime, 3000))
        {
          dusSpeedCalibrationStage = 2;
        }
        else
        {
           _gk_spinVus(constantSpeedCode);
        }
      }
      break;
      
  case 2:     // засечка угла, начало нового круга
      {
        if(numOfCircle == 1) // первый круг
        {
          startCalibrationTime = system_time;
          startAngle = g_sysAngle360;
        } 
        
        startTurnTime =  system_time;
        dusSpeedCalibrationStage = 3;
        canMonitor_printf("Turn #%d started", numOfCircle);
      }
      break;
   
      
    case 3:     // первые несколько секунд КАЛИБРОВКИ, что бы шар ушел с начального угла
      {
        if((system_time - startTurnTime) >= 2000)
        {
          dusSpeedCalibrationStage = 4;          
        }
        _gk_spinVus(constantSpeedCode);
      }
      break;
      
      
    case 4:     // ожидание завершения оборота
      {
        uint32_t delta = (startAngle>g_sysAngle360) ? (startAngle-g_sysAngle360) : (g_sysAngle360-startAngle);
        if(delta <= USYSANGLE_TO_CODE(0.3))
        {
          canMonitor_printf("Turn #%d completed. Time=%dms", numOfCircle, (system_time - startTurnTime));
                  
          if(numOfCircle == GK_CALIB_NUM_OF_CIRCLES)
          {
            dusSpeedCalibrationStage = 5;
          }
          else 
          {
            dusSpeedCalibrationStage = 2;       // ожидание круга         
            numOfCircle++;
          }
        }
        _gk_spinVus(constantSpeedCode);
      }
      break;
      
    case 5:     // завершение калибровки, подсчет всего
      {
         extern double dusCalibrationKoef;
         uint32_t endCalibrationTime = system_time;
         uint32_t elapsedTime = endCalibrationTime - startCalibrationTime;
       
         float t1_ms = GK_TOTAL_CALIB_TIME*1000;
         float t2_ms = elapsedTime;
         float k = (t1_ms/t2_ms);
         newDusCalibrationKoef = dusCalibrationKoef*k;
         canMonitor_printf("Done. elapsed time = %d. K=%f\n", elapsedTime, k);
         
         dusSpeedCalibrationStage = 6;
         gk_setConstVelocityCode(CONVERT_VELOCITY_TO_CODE32(0));
         
         // Сохранить новый коэффициент
         dusCalibrationKoef = newDusCalibrationKoef;
         extern void saveUserDataInFlash();
         saveUserDataInFlash();
         
         endCalibrationTime+=0;
         t1_ms+=0;
         t2_ms+=0;
      }
      break;
            
    case 6:     // удерживать нулевую скорость после калибровки
      {
        _gk_spinVus(constantSpeedCode);
      }
      break;
  }
}

// Движение туда-обратно
void _gk_nextTudaSudaVUS()
{
  static uint32_t startTime = 0;
  if(elapsed(&startTime, modeTudaSudaVUSPeriod))
  {
    modeTudaSudaVUSVelocityCode = -modeTudaSudaVUSVelocityCode;
  }
  _gk_spinVus(modeTudaSudaVUSVelocityCode); 
}

// Движение туда-сюда АР
void _gk_nextTudaSudaAR()
{
  static uint32_t startTime = 0;
  static uint8_t firstAngle=0; 
  if(elapsed(&startTime, modeTudaSudaARPeriod))
  {
    arretierRequiredAngleU32 = firstAngle ? modeTudaSudaARAngle1 : modeTudaSudaARAngle2;
    firstAngle = !firstAngle;
  }
  _gk_nextAR();     
}



// ВУО
void _gk_nextVUO()
{
  static uint32_t arTime;       // время вхождение в арретир
  static uint8_t arretered;     // вошел в арретир
   
#define ARRETIRED_TIME  200
  static uint32_t AR_DELTA_FI = USYSANGLE_TO_CODE(3.0/128.0);       // количество отсчетов 1/128 градуса
  
  switch(vuoStage)
  {
    case VUO_INIT:
      {
        arretered = 0;
        vuoStage = VUO_AR;
      }
      break;
      
    case VUO_AR:        // Арретирование
      {
        _gk_nextAR();
        
        uint32_t startAngle = g_sysAngle360;
        uint32_t stopAngle = arretierRequiredAngleU32;
        int32_t deltaFi = angle_getMinimalAngleRange(startAngle, stopAngle);
        int32_t deltaFiAbs = ABS(deltaFi);
        if(deltaFiAbs <= AR_DELTA_FI)
        {
          if(!arretered)
          {
            arretered = 1;
            arTime = system_time;
          }
          if(elapsed(&arTime, ARRETIRED_TIME))
          {
            vuoStage = VUO_VUS;
          }
        }
        else arretered = 0;
        deltaFi++;
        deltaFiAbs++;
      }
      break;
              
    case VUO_VUS:       // ВУС
      {
        _gk_nextVUS();
      }
      break;
  }
}



// Самоконтроль
void _gk_nextSelfControl()
{
  if(dbg_funcTestCrashed) return;
  
  switch(functionalTestStage)
  {
    case FUNCTEST_INIT:                 _gk_nextVUS();  break;
    case FUNCTEST_VUS_TEST:             _gk_nextVUS();  break;
    case FUNCTEST_AR_TEST:              _gk_nextAR();   break;
    case FUNCTEST_COMPLETE_WITH_ERROR:  _gk_nextVUS();  break;
    case FUNCTEST_COMPLETE_SUCCESS:     _gk_nextAR();   break;
  }
}


// =============================================================================
// =============================================================================
// =============================== PID-регулятор ===============================
// =============================================================================
// =============================================================================

// Вращение в заданной постоянной скоростью скоростью
void _gk_spinVus(int16_t desiredDusCode)
{
  extern int16_t dusAmplitude;
  float upr = pid_nextCode(desiredDusCode, dusAmplitude);
  gk_PidRegulatorSpin(upr);
}



// Задать постоянную скорость вращения ГК
void gk_setConstVelocityCode(int16_t velocityCode32)
{ 
  constantSpeedCode = velocityCode32;
}

// Управление движением
void gk_PidRegulatorSpin(float upr)
{
  uint8_t clockwise = (upr>0) ? 1 : 0; // 1-вращение по часовой стрелке, 0-вращение против часовой стрелке 
  setPwmFromUprCode(upr);
  
  if(clockwise) engine_spinCW();
  else engine_spinCCW();
}

// Установить ШИМ
void setPwmFromUprCode(float uprCode)
{
  float absUpr = uprCode<0 ? -uprCode : uprCode;
/*
  static const uint32_t threshold60percentage = 2797;
  if(absUpr > threshold60percentage) absUpr = threshold60percentage;
  float pwm = (absUpr*60)/threshold60percentage;
  setPwmFillingF((int16_t)pwm);
*/
  setPwmFillingF((uint16_t)absUpr);
}


