#ifndef __CAN_GK_CONTROL_H__
#define __CAN_GK_CONTROL_H__


#include "MDR32Fx.h"
#include "gkControlMode.h"



#define TANGAGE_CTRL_SETMODE_VUS                0x01          // установить режим ВУС
#define TANGAGE_CTRL_SETMODE_AR                 0x02          // установить режим АР
#define TANGAGE_CTRL_SETMODE_ENGINE_OFF         0x03          // останов
#define TANGAGE_CTRL_SETMODE_DISABLE_ENGINE     0x04          // запрет управлениями двигателями
#define TANGAGE_CTRL_SETMODE_TP                 0x05          // транспортное положениеы
#define TANGAGE_CTRL_SETMODE_VUO                0x06          // ВУО
#define TANGAGE_CTRL_SETSUBMODE_SELFCONTROL     0x07          // подрежим самоконтроля
#define TANGAGE_CTRL_ANGLE_CORRECTIONS          0x08          // аппаратная и логическая шкалы
#define TANGAGE_CTRL_CAN_BUS_TEST               0xDB          // тест шины CAN






void canGkControl_rxMsg(uint8_t* buf, uint8_t len);
void canGkControl_sendCurrentState();

void canBusTest_start();
void canBusTest_stop();



#endif //__CAN_GK_CONTROL_H__