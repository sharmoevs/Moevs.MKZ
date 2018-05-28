#ifndef __CAN_H__
#define __CAN_H__

#include "MDR32Fx.h"


#define CAN_TX_TIMEOUT_MS               20                                       // таймаут на передачу пакета

#define CAN_TX_REQ(can, bufNum)         (can->BUF_CON[bufNum] & 1<<5)           // запрос на отправку пакета, 0 - пакет отправлен, 1 - передача не завершена
#define CAN_TX_REQ_CLR(can, bufNum)     (can->BUF_CON[bufNum] &= ~(1<<5))       // снять запрос на отправку пакета
#define CAN_ERROR_OVER(can)             (can->STATUS & 1<<2)                    // ошибок больше, чем ERR_MAX


#define CAN_STANDART_ID_MASK               0x7FF<<18                            // маска стандартного ID


// Передающие буферы
#define CAN_LDR_TX_BUF                     0            // передающий буфер для перепрограммировния
#define CAN_GK_CONTROL_TX_BUF              1            // передающий буфер для управления
#define CAN_MONITOR_TX_BUF                 2            // передающий буфер для монитора состояния камеры
#define CAN_DBG_TEXT_MESSAGE_TX_BUF        3             // текстовые отладочные сообщения 
//#define CAN_TERM_TX_BUF                    4            // передающий буфер для терминала
#define CAN_TEST1_TX_BUF                   5            // тест1
#define CAN_TEST2_TX_BUF                   6            // тест2

// Приемные буферы
#define CAN_LDR_RX_BUF                     15             // приемный буфер для перепрограммирования
//#define CAN_TERM_RX_BUF                    16             // приемный буфер для терминала 
#define CAN_MONITOR_RX_BUF                 17             // приемный буфер для монитора состояния камеры 
#define CAN_GK_CONTROL_RX_BUF              18             // приемный буфер для управления ГК
#define CAN_ANGLE_RX_BUF                   19             // приемный буфер для угла с АЦПВТ      // Индекс буфера для передачи текстовых сообщений
#define CAN_USTR_SOPR_BUF                  20             // приемный буфер для данных от устройства сопряжения






// CAN-Идентификаторы 
#define CANID_LDR_REQ                      0x01             // перепрошивка - id=1
#define CANID_LDR_ANSWER                   0x02             // девайсы отвечают сообщениями с id=2 при перепрограммировании

#define CANID_TERM_RX                      0x04             // id сообщений от терминала 
#define CANID_TERM_TX                      0x03             // id сообщений терминалу

#define CANID_MONITOR_RX                   0x81             // id сообщений от монитора состояний
#define CANID_MONITOR_TX                   0x71             // id сообщения монитору состояний

#define CANID_ANGLE_RX                     0x07             // id сообщений от АЦПВТ

#define CANID_GK_CONTROL_RX                0x09             // id сообщений от монитора состояний
#define CANID_GK_CONTROL_TX                0x08             // id сообщения монитору состояний

#define CANID_DBG_TEXT_TX                  0x91          // id сообщений текстовой отладки

#define CANID_USRT_SOPR_DATA_RX            0x24


// Маски id
#define CAN_RX_FILTER(id)                  (id<<18)
/*
#define CAN_RX_FILTER_PROGRAM              (CANID_LDR_REQ<<18)                  // команды перепрошивки
#define CAN_RX_FILTER_TERMINAL             (CANID_TERM_RX<<18)                  // сообщения от терминала
#define CAN_RX_FILTER_MONITOR              (CANID_MONITOR_RX<<18)               // сообщения от монитора
#define CAN_RX_FILTER_ANGLE                (CANID_ANGLE_RX<<18)                 // сообщения от монитора
#define CAN_RX_FILTER_GK_CONTROL           (CANID_GK_CONTROL_RX<<18)            // сообщения от курсового контура
*/




typedef struct 
{
  uint8_t data[8];
} canBuffer_t;



void can2_init();
__ramfunc uint8_t can2_rx_new_packet(uint8_t can_buf_indx, uint8_t *buf, uint8_t *len);
__ramfunc uint8_t can2_send_packet(uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len);
void can_putPackage(MDR_CAN_TypeDef *can, uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len);
__ramfunc void can2_putDataInBuf(uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len);



#endif //__CAN_H__