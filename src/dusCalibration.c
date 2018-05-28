#include "dusCalibration.h"
#include "dus.h"
#include "timers.h"
#include "gkControlMode.h"
#include "angleSensor.h"
#include "global.h"
#include "canmonitor.h"
#include "flashUserData.h"


extern uint8_t          gs_preparationNow;
extern GKControlMode_t  gk_controlMode;
extern uint16_t         dusFilteredCode;
extern uint32_t         g_sysAngle360;
extern uint32_t         arretierRequiredAngleU32;

uint8_t dbg_dusCorrectionEnable = 1;    // включить поправку к ДУС
int16_t  dusCodeAmmend = 0;             // попкравка к АЦП ДУС
uint32_t g_arCalibratedAngle = 0;       // угол в котором откалибровались в последний раз
DusTempCorrection_t dusTemperatureCorrections[DUS_TEMPCALIB_RANGES_COUNT];
DusTempCorrection_t dusDefaultTemperatureCorrections[DUS_TEMPCALIB_RANGES_COUNT];
DusCalibrationCoefDestination_t dusCalibCoefDestination;    // откуда получена текущая поправка к ДУС
int16_t dusTemperature = 0;
uint8_t enableDusTemperatureCalibrationLog = 1;         // логгирование калибровки ДУС по температуре
uint8_t enableDusArreiterCalibrationLog = 0;            // логгирование калибровки ДУС в арретире

ArCalibration_t arCalibration;

void _dus_startupCalibration();
void _dus_arretierCalibration();
void _dus_temperatureCalibration();
void _dus_averageInArretire();
uint8_t _getNumOfTemperatureRange(int16_t temperature);
void _setCalibrationCoeff(DusTempCorrection_t *pArray, int16_t temperature, int16_t correction);
int16_t _getCalibrationCoeff(int16_t temperature, DusCalibrationCoefDestination_t *destination);








// Калибровка ДУС
void dus_calibrate()
{
  static GKControlMode_t previousGkControlMode;
  static GKControlMode_t currentGkControlMode;
  static uint32_t dusCorrectionTime = 0;        // время калибровки ДУС
  
  currentGkControlMode = gk_controlMode;
  uint8_t modeChanged = (currentGkControlMode != previousGkControlMode);
  previousGkControlMode = currentGkControlMode;
  
  switch(currentGkControlMode)
  {
    // Инициализация по включению питания
    case GkMode_Initialize:
      if(modeChanged) dusCalibration_init(DUS_STARTUP_CALIBRATION_SAMPLES, 5000);
      _dus_startupCalibration();  
      break;
    
    // Калибровка в арретире
    case GkMode_AR:
    case GkMode_TP: 
      if(modeChanged) dusCalibration_init(ARRETIER_CALIBRATION_SAMPLES, 5000);
      _dus_arretierCalibration(); 
      break;
      
    // Калибровка по температуре
    case GkMode_DUSTemperatureCalibration:
      if(modeChanged) dusCalibration_init(TEMPERATURE_CALIBRATION_SAMPLES, 0);
      _dus_temperatureCalibration();
      break;
  }
  
  if(dbg_dusCorrectionEnable && elapsed(&dusCorrectionTime, 1000))
  {
    dusCodeAmmend = _getCalibrationCoeff(dusTemperature, &dusCalibCoefDestination);
  }
}



// Калибровка ДУС по включению питания
void _dus_startupCalibration()
{   
  static uint8_t  tangageCalibrationDone = 0;  // калибровка курсового контура завершена
  static uint8_t  stage = 0;
  
  switch(stage)
  {
    case 0:
        if(system_time < ENGINE_STARTUP_DELAY) return;
        //calibrationStartTime = system_time;
        stage = 1;
      break;
      
    case 1:
      _dus_averageInArretire();
      if(arCalibration.done)
      {
        arCalibration.done = 0;
        tangageCalibrationDone = 1;
        stage = 2;
        _setCalibrationCoeff(dusTemperatureCorrections, dusTemperature, arCalibration.dusCorrection);
      }
      break;
      
    case 2: break;
  }
  
  if(tangageCalibrationDone)
  {
    gk_controlMode = GkMode_AR; // включить арретирование
  }
}

// Калибровка ДУС в арретире
void _dus_arretierCalibration()
{
  if(g_arCalibratedAngle != arretierRequiredAngleU32)   // если изменился угол арретирования - начать калибровку заново
  {
    g_arCalibratedAngle = arretierRequiredAngleU32;     // калибровка ведется для этого угла
    arCalibration.stage = AR_CALIB_STATE__GRAB_ANGLE;
  }
  
  _dus_averageInArretire();
  if(arCalibration.done)
  {
     arCalibration.done = 0;
    _setCalibrationCoeff(dusTemperatureCorrections, dusTemperature, arCalibration.dusCorrection);
  }
}

// Калибровка ДУС по температуре
void _dus_temperatureCalibration()
{
  // Запускаем калибровку, только если температура в заданных пределах +- x град.
  // Сохраняем значение в ту же ячейку только через N минут.
  
  // Проверяем, что находимся в допустиомом калибровочном диапазоне
  uint8_t canCalibrateInCurrentRange;
  int16_t lowTemp;       // нижняя граница диапазона
  int16_t highTemp;      // верхняя граница диапазона
  
  uint8_t iRange =  _getNumOfTemperatureRange(dusTemperature);
  DusTempCorrection_t *dusCorr = &dusDefaultTemperatureCorrections[iRange];
  if(iRange == 0) // температура ниже минимальной
  {
    canCalibrateInCurrentRange = 1;
    lowTemp = -100;
    highTemp = DUS_TEMPCALIB_MIN_TEMP;
  }
  else if(iRange >= (DUS_TEMPCALIB_RANGES_COUNT-1)) // температура  выше максимальной
  {
    canCalibrateInCurrentRange = 1;   
    lowTemp = DUS_TEMPCALIB_MAX_TEMP;
    highTemp = 100; 
  }
  else
  {
    lowTemp = GET_LOW_TEMPERATURE(iRange);        // нижняя граница диапазона
    highTemp = GET_HIGH_TEMPERATURE(iRange);      // верхняя граница диапазона  
    int16_t averageTemp = (lowTemp + highTemp)/2;         // среднее значение температуры в текущем температурном диапазоне
    
    canCalibrateInCurrentRange = ((dusTemperature >= (averageTemp - DUS_TEMPCALIB_AVER_TEMP_OFFSET)) &&
                                  (dusTemperature <= (averageTemp + DUS_TEMPCALIB_AVER_TEMP_OFFSET)));    
  }  
  if(!canCalibrateInCurrentRange) 
  {
    arCalibration.stage = AR_CALIB_STATE__GRAB_ANGLE; // сбросить текущую калибровку если выскочили за диапазон
    return;
  }
      
  
  // Проверяем, что последняя калибровка в этом диапазоне производилась давно
  if((dusCorr->time != 0) && ((system_time - dusCorr->time) < DUS_TEMPCALIB_CALIBRATION_INTERVAL))          // если этот диапазон был недавно откалиброван
  {   
    return;
  }
  
  
  // Ожидание завершения калибровки
  _dus_averageInArretire();
  
  if(enableDusTemperatureCalibrationLog)
  {
    if((arCalibration.stage==AR_CALIB_STATE__ACCUM_SAMPLES) && (arCalibration.iSample == 1))
    {
      canMonitor_printf("КАЛИБРОВКА ПО ТЕМПЕРАТУРЕ: началась калибровка диапазона [%d - %d]. Текущая температура %d", lowTemp, highTemp, dusTemperature);
    }
  }
  
  if(arCalibration.done)
  {
    arCalibration.done = 0;
    
    dusCorr->available = 1;
    dusCorr->correction = arCalibration.dusCorrection;
    dusCorr->time = system_time;
    
    saveUserDataInFlash(); // сохранить калибровочный коэффициент
    if(enableDusTemperatureCalibrationLog)
    {
      canMonitor_printf("КАЛИБРОВКА ПО ТЕМПЕРАТУРЕ: диапазон [%d - %d] откалиброван. Отсчетов %d, k = %d", lowTemp, highTemp, arCalibration.totalSamplesCount, arCalibration.dusCorrection);
    }
  }
}

// Усреднение отсчетов с ДУС в арретире
void _dus_averageInArretire()
{
  uint32_t currentAngle = g_sysAngle360;
  switch(arCalibration.stage)
  {
    case AR_CALIB_STATE__GRAB_ANGLE:  // захватить текущий угол
      {
        arCalibration.repereAngle = currentAngle;
        arCalibration.numOfAr = 0;
        arCalibration.startOperationTime = system_time;
        arCalibration.stage = AR_CALIB_STATE__CHECK_ANGLE;
      }
      break;
      
    case AR_CALIB_STATE__CHECK_ANGLE:     // проверить, что не ушли с захваченного угла
      {
        if(system_time - arCalibration.startOperationTime < 500) break; // не прошло Х мс
       
        int32_t deltaFi = angle_getMinimalAngleRange(currentAngle, arCalibration.repereAngle);
        uint32_t absDeltaFi = ABS(deltaFi);        
        if(absDeltaFi <= ARRETIER_CALIB_MAX_DEVIATION) // не ушли с той точки
        {
          arCalibration.startOperationTime = system_time;
          if(++arCalibration.numOfAr == 3) 
          {
            arCalibration.iSample = 0;
            arCalibration.dusAccumulator = 0;
            arCalibration.stage = AR_CALIB_STATE__ACCUM_SAMPLES;
            if(enableDusArreiterCalibrationLog)
            {
              canMonitor_printf("КАЛИБРОВКА В АРРЕТИРЕ: угол захвачен");
            }
          }
        }
        else 
        {
          arCalibration.stage = AR_CALIB_STATE__GRAB_ANGLE;            
          if(enableDusArreiterCalibrationLog)
          {
             canMonitor_printf("КАЛИБРОВКА В АРРЕТИРЕ: поиск угла");
          }
        }        
      }
      break;
      
    case AR_CALIB_STATE__ACCUM_SAMPLES:// накопить отсчеты
      {
          int32_t deltaFi = angle_getMinimalAngleRange(currentAngle, arCalibration.repereAngle);
          uint32_t absDeltaFi = ABS(deltaFi);        
          if(absDeltaFi > ARRETIER_CALIB_MAX_DEVIATION) //  ушли с той точки
          {
            arCalibration.stage = AR_CALIB_STATE__GRAB_ANGLE;
            return;
          }
                    
          arCalibration.dusAccumulator += dusFilteredCode;
          arCalibration.iSample++;
            
          if(arCalibration.iSample == arCalibration.totalSamplesCount)
          {
            uint16_t averVal = arCalibration.dusAccumulator/arCalibration.totalSamplesCount;
            arCalibration.dusCorrection = 0x7FF - averVal;
            arCalibration.done = 1;
            arCalibration.stage = AR_CALIB_STATE__WAIT_NEXT_CALIB;
            arCalibration.startOperationTime = system_time;
            if(enableDusArreiterCalibrationLog)
            {
               canMonitor_printf("КАЛИБРОВКА В АРРЕТИРЕ: завершена. K = %d", arCalibration.dusCorrection);
            }
          }
      }
      break;
      
    case AR_CALIB_STATE__WAIT_NEXT_CALIB: // ожидание следующей калибровки
      {
         if(system_time - arCalibration.startOperationTime < arCalibration.calibrationInterval) return;
         arCalibration.stage = AR_CALIB_STATE__GRAB_ANGLE;
      }
      break;    
  }
}




// Возвращает номер температурного диапазона
uint8_t _getNumOfTemperatureRange(int16_t temperature)
{
  uint8_t finded = 0;
  uint8_t iRange = 0;
  // Определение индекса
  while(!finded)
  {
    int16_t threshold = DUS_TEMPCALIB_MIN_TEMP + (iRange*DUS_TEMPCALIB_TEMPERATURE_STEP);
    if(temperature <= threshold)
    {
      finded = 1;
      if(iRange >= DUS_TEMPCALIB_RANGES_COUNT) iRange = DUS_TEMPCALIB_RANGES_COUNT - 1;
    }
    else iRange++;
  }
  return iRange;
}

// Установить температурную поправку к ДУС при текущей температуре в соответствующий слот
void _setCalibrationCoeff(DusTempCorrection_t *pArray, int16_t temperature, int16_t correction)
{
  uint8_t iRange =  _getNumOfTemperatureRange(temperature);
  DusTempCorrection_t *dusCorr = &pArray[iRange];
    
  dusCorr->correction = correction;
  dusCorr->time = system_time;
  dusCorr->available = 1;
}

// Получить температурную поправку к ДУС при текущей температуре
int16_t _getCalibrationCoeff(int16_t temperature, DusCalibrationCoefDestination_t *destination)
{
  uint8_t iRange =  _getNumOfTemperatureRange(temperature);
  
  if(dusTemperatureCorrections[iRange].available)               
  {
    *destination = DUS_CALIB_COEF__RAM;
    return dusTemperatureCorrections[iRange].correction;
  }
  else if(dusDefaultTemperatureCorrections[iRange].available)
  {
    *destination = DUS_CALIB_COEF__FLASH;
    return dusDefaultTemperatureCorrections[iRange].correction;
  }
  else 
  {
    *destination = DUS_CALIB_COEF__NONE;
    return 0;
  }
}

// Инициализация структуры калибровки ДУС в арретире
void dusCalibration_init(uint16_t totalSamplesCount, uint32_t calibrationInterval)
{
    arCalibration.totalSamplesCount = totalSamplesCount;
    arCalibration.calibrationInterval = calibrationInterval;
    
    arCalibration.stage = AR_CALIB_STATE__WAIT_NEXT_CALIB;
    //if(system_time >= calibrationInterval-100) arCalibration.startOperationTime = system_time - (calibrationInterval-100);
    //else arCalibration.startOperationTime = system_time;
    arCalibration.startOperationTime = system_time;
    arCalibration.done = 0;          
    arCalibration.iSample = 0;
    arCalibration.dusAccumulator = 0;
}
































































/*
// Калибровка ДУС
void dus_calibrate()
{
  switch(gk_controlMode)
  {
    case GkMode_Initialize: _dus_startupCalibration(); break; // Инициализация по включению питания
    
    case GkMode_AR:         
    case GkMode_TP:
      _dus_arretierCalibration(); 
      break;
  }
}


// Калибровка по включению питания
void _dus_startupCalibration()
{   
  static uint8_t  stage = 0;            // стадия калибровки
  static uint32_t arrieteredTime;       // время выхода в арретир
  static uint8_t  arrietered = 0;       // флаг выхода в арретир
  static uint32_t dusAccumulator = 0;   // аккумулятор
  static uint32_t sampleCount = 0;      // счетчик
  static uint8_t  tangageCalibrationDone = 0;  // калибровка курсового контура завершена
  static uint32_t dbg_maxDeltaFi = 0;
  ////static uint32_t previousAngle = 0;
  
  uint32_t currentAngle = g_sysAngle360;   
  
  switch(stage)
  {
    case 0: // инициализация
      {
        if(system_time < ENGINE_STARTUP_DELAY) return;
        stage = 1;
      }
      break;
        
    case 1: // ожидание выхода в арретир
      {
        int32_t deltaFi = angle_getMinimalAngleRange(currentAngle, arretierRequiredAngleU32);
        uint32_t absDeltaFi = ABS(deltaFi);
        if(dbg_maxDeltaFi < absDeltaFi) dbg_maxDeltaFi = absDeltaFi; // отладка
        
        if(absDeltaFi <= STARTUP_CALIB_MAX_DEVIATION)
        {
          if(!arrietered)// не в арретире
          {
            arrietered = 1;
            arrieteredTime = system_time;
          }
          else // в арретире
          {
            if(system_time - arrieteredTime < 2000) break;
            sampleCount = 0;
            dusAccumulator = 0;
            stage = 2;
          }
        }
        else 
        {
          arrietered = 0;
        }        
      }
      break;
              
      case 2: // калибровка
        {
          dusAccumulator += dusFilteredCode;
          sampleCount++;
            
          if(sampleCount == DUS_STARTUP_CALIBRATION_SAMPLES)
          {
             uint16_t averVal = dusAccumulator/DUS_STARTUP_CALIBRATION_SAMPLES;
             dusCodeAmmend = 0x7FF - averVal;
             tangageCalibrationDone = 1;
             stage = 3;
          }
        }
        break;
        
      case 3:
      
      break;
  }
  ////previousAngle = currentAngle;
  
  if(tangageCalibrationDone)
  {
    gk_controlMode = GkMode_AR; // включить арретирование
  }
}


// Калибровка по включению питания
void _dus_arretierCalibration()
{
  // После калибровки по включению питания мы уже откаллибровались в арретире. Поэтому ждем новую команду
  static uint8_t  stage = 4;            // стадия калибровки
  static uint32_t startTime = 0;        // таймер
  static uint32_t dusAccumulator = 0;   // аккумулятор
  static uint32_t sampleCount = 0;      // счетчик  
  static uint32_t reperAngle = 0;       // угол от которого ведем отсчет вхождение в арретир
  static uint16_t numOfAr = 0;
  
  if(g_arCalibratedAngle != arretierRequiredAngleU32) stage = 0;   // если изменился угол арретирования - начать калибровку заново
  
  uint32_t currentAngle = g_sysAngle360;  
  switch(stage)
  {
    case 0: // инициализация
      {
        g_arCalibratedAngle = arretierRequiredAngleU32;    // калибровка ведется для этого угла
        stage = 1;
#ifdef DBG_ENABLE_AR_CALIBRATION_LOG
        canMonitor_sendString("calib: started");
#endif
      }
      break;
      
    case 1: // запомнить угол
      {
        reperAngle = currentAngle;
        startTime = system_time;
        numOfAr = 0;
        stage = 2;
      }
      break;
          
    case 2: // проверить не ушли ли с этого угла через 500мс
      {
        if(system_time - startTime < 500) break; // не прошло 500мс
       
        int32_t deltaFi = angle_getMinimalAngleRange(currentAngle, reperAngle);
        uint32_t absDeltaFi = ABS(deltaFi);        
        if(absDeltaFi <= ARRETIER_CALIB_MAX_DEVIATION) // не ушли с той точки
        {
          startTime = system_time;
          if(++numOfAr == 3) 
          {
            sampleCount = 0;
            dusAccumulator = 0;
            stage = 3;
#ifdef DBG_ENABLE_AR_CALIBRATION_LOG
            canMonitor_sendString("calib: arretiered");
#endif
          }
        }
        else 
        {
#ifdef DBG_ENABLE_AR_CALIBRATION_LOG
          canMonitor_sendString("calib: angle search");
#endif
          stage = 1;
        }        
      }
      break;
              
      case 3: // калибровка
        {
          int32_t deltaFi = angle_getMinimalAngleRange(currentAngle, reperAngle);
          uint32_t absDeltaFi = ABS(deltaFi);        
          if(absDeltaFi > ARRETIER_CALIB_MAX_DEVIATION) //  ушли с той точки
          {
            stage = 1;
            return;
          }
          
          dusAccumulator += dusFilteredCode;
          sampleCount++;
            
          if(sampleCount == ARRETIER_CALIBRATION_SAMPLES)
          {
             uint16_t averVal = dusAccumulator/ARRETIER_CALIBRATION_SAMPLES;
             dusCodeAmmend = 0x7FF - averVal; 
             stage = 4;          
             startTime = system_time;
#ifdef DBG_ENABLE_AR_CALIBRATION_LOG
             canMonitor_sendString("calib: done. Delta = %d\n", dusCodeAmmend);
#endif
          }
        }
        break;
        
      case 4:
        if(system_time - startTime < 5000) return;
        stage = 1;        
      break;
  }
}

*/