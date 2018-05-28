#ifndef __CANMONITOR_H__
#define __CANMONITOR_H__

#include "MDR32Fx.h"
#include "canmonitorText.h"
#include "can.h"

#define CANMONITOR_QUEUE_CAPACITY               50


// Идентификаторы команд
#define CANMONITOR_STATE_REQUEST                0x01    // запрос состояния
#define CANMONITOR_EXTENDED_CMD                 0xF0    // расширенная команда
#define CANMONITOR_PID_KOEF_REQUEST             0x02    // запрос коэффициентов ПИД-регулятора
#define CANMONITOR_PID_KOEF_SET_P               0x03
#define CANMONITOR_PID_KOEF_SET_I               0x04
#define CANMONITOR_PID_KOEF_SET_D               0x05
#define CANMONITOR_PID_KOEF_SET_N               0x51
#define CANMONITOR_SET_T_DISKR                  0x06
#define CANMONITOR_SAVE_PID_KOEF_IN_FLASH       0x07
#define CANMONITOR_START_VELOCITY_CALIBRATION   0x08
#define CANMONITOR_SET_MODE_TUDA_SUDA_VUS       0x09
#define CANMONITOR_SET_OUTPUT_DEBUGING_INFO     0x0A
#define CANMONITOR_ENGINE_SWITCH_OFF            0x0B    // выключить двигатели
#define CANMONITOR_SET_MODE_TUDA_SUDA_AR        0x0F 
#define CANMONITOR_SET_MODE_DUS_TEMP_CALIBRATION 0x12

// Служебные команды
#define CANMONITOR_DEBUG                        0xDB    // отладочная команда
#define CANMONITOR_EXT_CMD_STRING_ID            0xBB    // идентификатор того, что  данные в расширенной команде нужно представлять как строку


// Идентификаторы ответов
#define CANMONITOR_STATE                        0x0E    // состояние ГК
#define CANMONITOR_WORKING_TIME_MS              0x50    // время в работы в мс от подачи питания
#define CANMONITOR_EXTENDED_ANSWER              0xF0    // расширенная команда
#define CANMONITOR_COURSE_VELOCITY              0x51    // угловая скорость
#define CANMONITOR_COURSE_UPR                   0x52    // управляющее воздействие
#define CANMONITOR_COURSE_GET_P                 0x53    // P коэф.
#define CANMONITOR_COURSE_GET_I                 0x54    // I коэф.
#define CANMONITOR_COURSE_GET_D                 0x55    // D коэф.
#define CANMONITOR_COURSE_GET_N                 0x59    // N коэф.
#define CANMONITOR_COURSE_GET_T                 0x56    // T дискр.
#define CANMONITOR_SEND_CURRENT_ANGLE           0x0A    // Текущий угол
#define CANMONITOR_SEND_TEST_VALUE_1            0x0B
#define CANMONITOR_DBG_SEND_SAVED_VALUE         0x0C    // отправить сохраненные значения

// Расширенная команда
#define EXT_CMD_START                           0x00
#define EXT_CMD_CONT                            0x01
#define EXT_CMD_END                             0x02


// Текстовые команды для отладки и контроля
#define DBG_CMD___HELP                          ("help")                // список команд
#define DBG_CMD___PING                          ("ping")                // проверка связи
#define DBG_CMD___RESET_MK                      ("reset tangage mk")
#define DBG_CMD___GET_DUS_CORR                  ("get dus corr")        // прочитать текущую поправку к ДУС
#define DBG_CMD___SET_DUS_CORR                  ("set dus corr ")       // прочитать текущую поправку к ДУС
#define DBG_CMD___SET_DUS_CORR_ENABLE           ("set enable dus corr ")// прочитать текущую поправку к ДУС
#define DBG_CMD___GET_SCALES                    ("get scales")          // текущие поправки
#define DBG_CMD___GET_TEMP                      ("get temp")            // текущая температура
#define DBG_CMD___GET_DBG_TEMP                  ("get dbg temp")        // режим и значение отладочной температуры
#define DBG_CMD___SET_DBG_TEMP                  ("set dbg temp enable ")        // установить температуру
#define DBG_CMD___SET_DBG_TEMP__PATTERN         ("set dbg temp enable 1 value +30 period 1000 mode +")        // установить температуру
#define DBG_CMD___GET_CALIBRATION_COEF          ("get dus coef")        // коэффициенты калибровки ДУС по температуре
#define DBG_CMD___GET_CALIBRATION_COEF_DEFAULT  ("get dus coef default")   // !!!! реализуется по другому
#define DBG_CMD___DYNAMIC_RAM_STATISTIC         ("dram")                // статистика использования памяти
#define DBG_CMD___SET_ENABLE_AR_CALIB_LOG       ("set enable calibration log ")                            // вкл/выкл логгирование калибровки в арретире
#define DBG_CMD___SET_ENABLE_PRINT_CURRENT_TEMP ("set enable print current temp ")






void canMonitor_init();
void canMonitor_rxFrameHandler(uint8_t *buf, uint8_t len);      // Принят пакет от монитора
void canMonitor_send(uint8_t *pData, uint8_t len);              // Отправить данные CAN-монитору
void onCanMonitorTransmitDone();



void canMonitor_sendStateWord();        // отправить слово состояния
void canMonitor_fillBufferWithCurrentState(uint8_t *msg);
void canMonitor_sendWorkingTime();      // время наработки
void canMonitor_rxExtCmd(uint8_t *pData, uint8_t len);      // принять расширенную команду
void canMonitor_sendExtHex(uint8_t *pBuf, uint8_t len);        // отправить ответ на расширенную команду
void canMonitor_printf(uint8_t *p, ...);



void canMonitor_sendCourseVelocity();
void canMonitor_sendCourseUpr();
void canMonitor_sendPIDKoef();
void canMonitor_sendAngle();
void canMonitor_sendTestValue1(float value);
void canMonitor_setModeTudaSudaVUS(uint8_t *buf, uint8_t len);
void canMonitor_setModeTudaSudaAR(uint8_t *buf, uint8_t len);
void canMonitor_setOutputDebugingInfo(uint8_t *buf, uint8_t len);
void canMonitor_sendDbgSavedValue(float value);


#endif //__CANMONITOR_H__
