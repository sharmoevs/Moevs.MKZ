#include "global.h"



// Возвращает колчисечтво одиничных бит в слове
uint32_t getBitsInWord(uint32_t x)
{
   x = (x & 0x55555555) + ((x>>1) & 0x55555555);
   x = (x & 0x33333333) + ((x>>2) & 0x33333333);
   x = (x & 0x0F0F0F0F) + ((x>>4) & 0x0F0F0F0F);
   x = (x & 0x00FF00FF) + ((x>>8) & 0x00FF00FF);
   x = (x & 0x0000FFFF) + ((x>>16) & 0x0000FFFF);
   return x;
}

// Преобразовать угол из шкалы 0°-360° в (-180)°-180°
float convertAngleToHardwareScale(float angle)
{
  if(angle < 180) return angle;
  else return angle - 360;
}
