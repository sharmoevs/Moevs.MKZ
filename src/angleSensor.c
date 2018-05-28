#include "angleSensor.h"
#include "can.h"
#include "canTerminal.h"
#include "global.h"
#include "timers.h"


uint32_t g_sysAngle360;                 // угол с датчика угла (0..360)
uint32_t g_engineControlAngle360;       // угол от 0..360 градусов для управления двигателями (с поправкой на ноль фазового двигателя)
uint32_t g_engineZeroAngleCorrection;   // поправка к датчику угла для определения фазового нуля двигателей

uint32_t g_tangageHardwareZeroAngleCorrection = 0x1000; // ноль аппаратной шкалы тангажного контура (значение не 0, т.к. в тестовом шаре датчик угла сбоит на нуле)
uint32_t g_tangageLogicalZeroAngleCorrection = 0;  // ноль логической шкалы тангажного контура

void _angle_saveCurentSample();
uint32_t angle_convertCmdU2SysU(uint32_t cmdAngle, uint32_t code360);
uint32_t _angle_convertS2U(int32_t angleS, uint32_t code360);

// Принять угол
void angleService_rxAngle(uint8_t *buf, uint8_t len)
{
   if(len != 3) return;
   g_sysAngle360 =  buf[0]<<16 | buf[1]<<8 | buf[2];
   g_engineControlAngle360 = angle_deductCorrection(g_sysAngle360, g_engineZeroAngleCorrection, MAX_SYSANGLE_CODE);
}



// Рассчет средней скорости
// Возвращает 1, если была рассчитана средняя скорость, в противном случае 0
uint8_t angle_getAverageVelocity(float *averageVelocity)
{
  static uint32_t lastSampleTime = 0;     // время последнего отсчета
  static uint32_t angle1 = 0;
  static uint32_t angle2 = 0; 
  static uint8_t  stage = 0;
    
  if(!elapsed(&lastSampleTime, AVERAGE_VELOCITY_INTERVAL)) return 0;  
  switch(stage)
  {
    case 0:     // инициализация
      stage = 1;
      angle1 = g_sysAngle360;
      return 0;
      
    case 1:
      {
        angle2 = g_sysAngle360;
        float deltaFi = USYSANGLE_TO_FLOAT(angle2) - USYSANGLE_TO_FLOAT(angle1);      
        *averageVelocity = deltaFi/((float)AVERAGE_VELOCITY_INTERVAL/1000.0);
        angle1 = angle2;
        return 1;
      }
    default: return 0;
  }
}

// Получает минимальное расстояние между двумя углами.
// Пример:
// startAngle = 110, stopAngle = 210
// Расстояние по часовой стрелке: 100 градусов
// Расстояние против часово стрелке: -260 градусов
// Возвращаемое значение 100
// Углы в аппаратной системе координат!
int32_t angle_getMinimalAngleRange(uint32_t startAngle, uint32_t stopAngle)
{
  int32_t rangeCW, rangeCCW;
  if(stopAngle>startAngle) 
  {
    rangeCW = stopAngle - startAngle;
    rangeCCW = rangeCW - USYSANGLE_TO_CODE(360);       // со знаком минус (против часовой стрелки)
  }
  else 
  {
    rangeCCW = stopAngle - startAngle;  // со знаком минус
    rangeCW = USYSANGLE_TO_CODE(360) + rangeCCW;
  }
  int32_t absCW = ABS(rangeCW);
  int32_t absCCW = ABS(rangeCCW);  
  int32_t deltaFi = (absCW<absCCW) ? rangeCW : rangeCCW;    // выбор кратчайшего расстояния
  return deltaFi;
}

// Вычесть из угла поправку deltaFi
uint32_t angle_deductCorrection(uint32_t angle, uint32_t deltaFi, uint32_t code360)
{
  uint32_t res = (angle >= deltaFi) ? 
                 (angle - deltaFi) :
                 (code360 - (deltaFi - angle));
  return res;
}
// Прибавить к углу поправку deltaFi
uint32_t angle_addCorrection(uint32_t angle, uint32_t deltaFi, uint32_t code360)
{
  uint32_t res = (angle + deltaFi<=code360) ? 
                 (angle + deltaFi) :
                 (deltaFi + angle - code360);
  return res;
}

// Преобразовать угол с ценой деления 1/128 градуса в угол с ценой деления 360/MAX_SYSANGLE_CODE
uint32_t angle_convertCmdU2SysU(uint32_t cmdAngle, uint32_t code360)
{
  float ANGLE_STEP = (float)code360/(float)0xB400;
  uint32_t res = (uint32_t)((float)cmdAngle*ANGLE_STEP); // угол от 0..360
  return res;
}

// Преобразовать знаковый -180..180 в беззнаковый 0..360 
// angleS - код угла
// code360 - код угла, соответствующий 360 градусам (он будет отличаться в зависимости от шкалы)
uint32_t _angle_convertS2U(int32_t angleS, uint32_t code360)
{
  uint32_t res = (angleS<0) ? (code360 + angleS) : angleS;
  return res;
}


// Преобразовать знаковый командный угол с ценой деления 1/128 (в аппаратной или логической шкале)
// в системный угол с учетом поправки на шкалу
uint32_t angle_convertCmdS2SysU(int32_t sCmdAngle, uint32_t scaleCorrection, uint32_t code360)
{
  uint32_t uCmdAngle = _angle_convertS2U(sCmdAngle, 360*128);                   // беззнаковый 1/128 апаратн/лог
  uint32_t uExtAngle = angle_convertCmdU2SysU(uCmdAngle, code360);              // беззнаковый 360/7ffff апаратн/лог
  uint32_t uSysAngle = angle_addCorrection(uExtAngle, scaleCorrection, code360);

  uCmdAngle++;  uExtAngle++;  uSysAngle++; uSysAngle--;
  return uSysAngle;
}





















/*

//uint32_t tangageAngleCode16;
//uint32_t hardwareTangageAngleCode;      // код в аппаратной шкале, без поправок на фазовый ноль
//uint16_t _angle_shiftScaleToZero(uint16_t angle);




// Поправка к фазовому нулю двигателей
uint16_t _angle_shiftScaleToZero(uint16_t angle)
{
  extern uint16_t zeroLevelAngleCodeOffset;        // поправка 
  if(angle > zeroLevelAngleCodeOffset)
    return angle - zeroLevelAngleCodeOffset;
  else 
    return 0xFFFF - (zeroLevelAngleCodeOffset - angle);  
}
*/
/*
// angle  - код с шагом 1/0xFFFF градуса в диапазоне 0..360град
// return - код с шагом 1/128 градуса в диапазоне -180..180град
int16_t angle_convertAbs16BitCodeTo16BitCode(uint16_t angle)
{
   static float ANGLE_STEP = (float)0xFFFF/(float)0xB400;
   int32_t result = (int32_t)((float)angle/ANGLE_STEP);       // угол от 0..360 
   if(result >= 180*128) result = result - 360*128;
   return result;
}
*/
/*
// Получить угол без поправки на нулевое смещение
float angle_getAngleWithZeroCorrection(float cmdAngle)
{
  extern float zeroLevelAngleOffsetF;
  float angle = cmdAngle-zeroLevelAngleOffsetF;
  if(angle>=180) angle-=360;
  else if(angle<=-180) angle+=360;
  return angle;
}*/



