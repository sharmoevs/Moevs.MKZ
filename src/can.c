#include "can.h"
#include "canmonitor.h"
#include "loaders.h"
#include "canterminal.h"
#include "timers.h"
#include "angleSensor.h"
#include "canGkControl.h"


extern volatile uint32_t system_time;
uint32_t transfer_time = 0;           // для таймаута - время начала передачи пакета
 
void _can2_initRxBuffer(uint8_t numOfBuf, uint32_t canFilterID);
void _can2_initTxBuffer(uint8_t numOfBuf, uint8_t enableIrq);

// Инициализация can1
void can2_init()
{
     // Разрешение тактирование блока CAN и предделитель частоты 
   MDR_RST_CLK->PER_CLOCK |= 1<<1;             // тактирование CAN1    
   MDR_RST_CLK->CAN_CLOCK = 0<<0 |             // делитель тактовой частоты CAN1 (0-0, 1-2, 2-4 ect)
                            1<<25;             // разрешение тактовой частоты на CAN2
   
      // Настройка скорости шины CAN
   // SYNC_SEG = 1        SEG1 = 6xTQ     SEG2 = 8xTQ     PSEG = 5xTQ 
   MDR_CAN2->BITTMNG = 1<<27 |                 // трехкратное семплирование с мажоритарным контролем
                       2<<25 |                 // размер фазы SJW - это максимальное значение, на которое происходит подстройка приема и передачи при работе на шине CAN. Приемник подстраивается на значение ошибки, но не более чем SJW.
                       5<<22 |                 // размер фазы SEG2 - это время, используемое для сокращения битового интервала при подстройке.
                       5<<19 |                 // размер фазы SEG1 - это время, используемое для увеличения битового интервала при подстройке.
                       6<<16 |                 // размер фазы PSEG - это время, компенсирующее задержку распространения сигналов в шине CAN
                       3<<0;                   // BRP - предделитель системной частоты CLK = PCLK/(BRP + 1);  TQ(us) = (BRP + 1)/CLK(MHz)
   
   // буферы, работающие на прием
   _can2_initRxBuffer(CAN_LDR_RX_BUF,           CAN_RX_FILTER(CANID_LDR_REQ));           // загрузчик
   //_can2_initRxBuffer(CAN_TERM_RX_BUF,          CAN_RX_FILTER(CANID_TERM_RX));           // терминал
   _can2_initRxBuffer(CAN_MONITOR_RX_BUF,       CAN_RX_FILTER(CANID_MONITOR_RX));        // монитор
   _can2_initRxBuffer(CAN_ANGLE_RX_BUF,         CAN_RX_FILTER(CANID_ANGLE_RX));          // угол
   _can2_initRxBuffer(CAN_GK_CONTROL_RX_BUF,    CAN_RX_FILTER(CANID_GK_CONTROL_RX));     // команды
   _can2_initRxBuffer(CAN_USTR_SOPR_BUF,        CAN_RX_FILTER(CANID_USRT_SOPR_DATA_RX)); // устройство сопряжения
 
   
   // буферы, работающие на передачу
   _can2_initTxBuffer(CAN_LDR_TX_BUF, 0);               // загрузчик
   //_can2_initTxBuffer(CAN_TERM_TX_BUF, 0);              // терминал
   _can2_initTxBuffer(CAN_MONITOR_TX_BUF, 0);           // монитор
   _can2_initTxBuffer(CAN_GK_CONTROL_TX_BUF, 0);        // загрузчик
   _can2_initTxBuffer(CAN_LDR_TX_BUF, 0);               // команды
   _can2_initTxBuffer(CAN_DBG_TEXT_MESSAGE_TX_BUF, 0);  // буфер для текста
   _can2_initTxBuffer(CAN_TEST1_TX_BUF, 0);             // тест 1
   _can2_initTxBuffer(CAN_TEST2_TX_BUF, 0);             // тест 2
   
   /*   
   // Приемный буфер - для перепрошивки
   MDR_CAN2->BUF_CON[CAN_LDR_RX_BUF] = 1<<1 |                                   // буфер работает на прием
                                       1<<0;                                    // разрешение работы буфера
   MDR_CAN2->CAN_BUF_FILTER[CAN_LDR_RX_BUF].MASK = CAN_STANDART_ID_MASK;        // ID & CAN_BUF_FILTER[x].MASK == CAN_BUF_FILTER[x].FILTER
   MDR_CAN2->CAN_BUF_FILTER[CAN_LDR_RX_BUF].FILTER = CAN_RX_FILTER_PROGRAM;
   MDR_CAN2->INT_RX |= 1<<CAN_LDR_RX_BUF;                                       // разрешение прерывания первого буфера
  
   // Приемный буфер - для терминала
   MDR_CAN2->BUF_CON[CAN_TERM_RX_BUF] = 1<<1 |                                  // буфер работает на прием
                                        1<<0;                                   // разрешение работы буфера
   MDR_CAN2->CAN_BUF_FILTER[CAN_TERM_RX_BUF].MASK = CAN_STANDART_ID_MASK;       // ID & CAN_BUF_FILTER[x].MASK == CAN_BUF_FILTER[x].FILTER
   MDR_CAN2->CAN_BUF_FILTER[CAN_TERM_RX_BUF].FILTER = CAN_RX_FILTER_TERMINAL;
   MDR_CAN2->INT_RX |= 1<<CAN_TERM_RX_BUF;                                      // разрешение прерывания первого буфера
 
   // Приемный буфер - для монитора
   MDR_CAN2->BUF_CON[CAN_MONITOR_RX_BUF] = 1<<1 |                                  // буфер работает на прием
                                        1<<0;                                   // разрешение работы буфера
   MDR_CAN2->CAN_BUF_FILTER[CAN_MONITOR_RX_BUF].MASK = CAN_STANDART_ID_MASK;       // ID & CAN_BUF_FILTER[x].MASK == CAN_BUF_FILTER[x].FILTER
   MDR_CAN2->CAN_BUF_FILTER[CAN_MONITOR_RX_BUF].FILTER = CAN_RX_FILTER_MONITOR;
   MDR_CAN2->INT_RX |= 1<<CAN_MONITOR_RX_BUF;                                      // разрешение прерывания первого буфера
 
   // Приемный буфер - для угла
   MDR_CAN2->BUF_CON[CAN_ANGLE_RX_BUF] = 1<<1 |                                  // буфер работает на прием
                                        1<<0;                                   // разрешение работы буфера
   MDR_CAN2->CAN_BUF_FILTER[CAN_ANGLE_RX_BUF].MASK = CAN_STANDART_ID_MASK;       // ID & CAN_BUF_FILTER[x].MASK == CAN_BUF_FILTER[x].FILTER
   MDR_CAN2->CAN_BUF_FILTER[CAN_ANGLE_RX_BUF].FILTER = CAN_RX_FILTER_ANGLE;
   MDR_CAN2->INT_RX |= 1<<CAN_ANGLE_RX_BUF;                                      // разрешение прерывания первого буфера
 
    // Приемный буфер - для управления
   MDR_CAN2->BUF_CON[CAN_GK_CONTROL_RX_BUF] = 1<<1 |                                  // буфер работает на прием
                                        1<<0;                                   // разрешение работы буфера
   MDR_CAN2->CAN_BUF_FILTER[CAN_GK_CONTROL_RX_BUF].MASK = CAN_STANDART_ID_MASK;       // ID & CAN_BUF_FILTER[x].MASK == CAN_BUF_FILTER[x].FILTER
   MDR_CAN2->CAN_BUF_FILTER[CAN_GK_CONTROL_RX_BUF].FILTER = CAN_RX_FILTER_GK_CONTROL;
   MDR_CAN2->INT_RX |= 1<<CAN_GK_CONTROL_RX_BUF;                                      // разрешение прерывания первого буфера
 
   
   
   
   // Передающий буфер - для перепрошивки
   MDR_CAN2->BUF_CON[CAN_LDR_TX_BUF] = 0<<1 |                                   // буфер работает на передачу
                                       1<<0;                                    // разрешение работы буфера

    // Передающий буфер - для терминала
   MDR_CAN2->BUF_CON[CAN_TERM_TX_BUF] = 0<<1 |                                  // буфер работает на передачу
                                        1<<0;                                   // разрешение работы буфера

   // Передающий буфер - для монитора
   MDR_CAN2->BUF_CON[CAN_MONITOR_TX_BUF] = 0<<1 |                               // буфер работает на передачу
                                           1<<0;                                // разрешение работы буфера

   // Передающий буфер - для управление (текущее состояние)
   MDR_CAN2->BUF_CON[CAN_GK_CONTROL_TX_BUF] = 0<<1 |                            // буфер работает на передачу
                                           1<<0;                                // разрешение работы буфера
   
   // Тестовые буферы
   // Передающий буфер - тест
   MDR_CAN2->BUF_CON[CAN_TEST1_TX_BUF] = 0<<1 |                                  // буфер работает на передачу
                                           1<<0;                                // разрешение работы буфера
   // Передающий буфер - тест
   MDR_CAN2->BUF_CON[CAN_TEST2_TX_BUF] = 0<<1 |                                  // буфер работает на передачу
                                           1<<0;                                // разрешение работы буфера
   */
   
   
   // Настройка прерываний 
   MDR_CAN2->INT_EN = //1<<4 |                                                    // разрешение прерывания по превышению TEC или REC допустимого значения в ERROR_MAX
                      //1<<3 |                                                    // разрешение прерывания по возникновению ошибки
                      //1<<2 |                                                    // разрешение прерывания по возможности передачи
                      1<<1 |                                                    // разрешениe прерывания по приему сообщений
                      1<<0;                                                     // глобальное разрешение прерывания блока CAN   
  
   MDR_CAN2->CONTROL = 1<<0;                                                    // разрешение работы контроллера CAN     

   NVIC_SetPriority(CAN2_IRQn, 1);                                              // снизить приоритет прерывания CAN, что бы он не забивал системный таймер
}

// Инициализация приемного буфера
void _can2_initRxBuffer(uint8_t numOfBuf, uint32_t canFilterID)
{
   // Приемный буфер - для монитора
   MDR_CAN2->BUF_CON[numOfBuf] = 1<<1 |                               // буфер работает на прием
                                 1<<0;                                // разрешение работы буфера
   MDR_CAN2->CAN_BUF_FILTER[numOfBuf].MASK = CAN_STANDART_ID_MASK;    // ID & CAN_BUF_FILTER[x].MASK == CAN_BUF_FILTER[x].FILTER
   MDR_CAN2->CAN_BUF_FILTER[numOfBuf].FILTER = canFilterID;
   MDR_CAN2->INT_RX |= 1<<numOfBuf;                                   // разрешение прерывания первого буфера
}

// Инициализация передающего буфера
void _can2_initTxBuffer(uint8_t numOfBuf, uint8_t enableIrq)
{
   MDR_CAN2->BUF_CON[numOfBuf] = 0<<1 |                               // буфер работает на передачу
                                 1<<0;                                // разрешение работы буфера
   if(enableIrq) MDR_CAN2->INT_TX |= 1<<numOfBuf;                                   // разрешение прерывания первого буфера
}


// Принять новый пакет
__ramfunc uint8_t can2_rx_new_packet(uint8_t can_buf_indx, uint8_t *buf, uint8_t *len)
{
   if(MDR_CAN2->BUF_CON[can_buf_indx] & 1<<6)
   {
      ((uint32_t*)buf)[0] = MDR_CAN2->CAN_BUF[can_buf_indx].DATAL;
      ((uint32_t*)buf)[1] = MDR_CAN2->CAN_BUF[can_buf_indx].DATAH;
     
      MDR_CAN2->BUF_CON[can_buf_indx] &= ~(1<<6);
      *len = MDR_CAN2->CAN_BUF[can_buf_indx].DLC&0xF;
      return 1;
   }
   else return 0;
}


// Передать пакет. Возвращает 1, если передача завершилась успешно, 0 - в противном случае
__ramfunc uint8_t can2_send_packet(uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len)
{ 
    transfer_time = system_time;
    while(CAN_TX_REQ(MDR_CAN2, can_buf_indx)) // ожидание освобождения буфера передатчика
    {
      if(elapsed(&transfer_time, CAN_TX_TIMEOUT_MS)) // истекло время ожидания 
      {
         CAN_TX_REQ_CLR(MDR_CAN2, can_buf_indx);                                // снять запрос на передачу пакета
         return 0;
      }
    }
    
    //while(CAN_TX_REQ(MDR_CAN1, can_buf_indx));  
    // Ожидание, пока завершится передача пакета или превысится лимит ошибок передачи
    //while(CAN_TX_REQ(MDR_CAN1, can_buf_indx) && !CAN_ERROR_OVER(MDR_CAN1));
    
    if(len>8) len=8;
    MDR_CAN2->CAN_BUF[can_buf_indx].ID = msgId<<18;
    MDR_CAN2->CAN_BUF[can_buf_indx].DLC = len | 1<<11;    
    MDR_CAN2->CAN_BUF[can_buf_indx].DATAL = *((uint32_t*)pData);
    MDR_CAN2->CAN_BUF[can_buf_indx].DATAH = *((uint32_t*)pData+1);    
    MDR_CAN2->BUF_CON[can_buf_indx] |= 1<<5;              // запрос на отправку  
           
    return 1;
}

// Записать пакет в аппаратный буфер передатчика
void can_putPackage(MDR_CAN_TypeDef *can, uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len)
{
    if(len>8) len=8;
    can->CAN_BUF[can_buf_indx].ID = msgId<<18;
    can->CAN_BUF[can_buf_indx].DLC = len | 1<<11;    
    can->CAN_BUF[can_buf_indx].DATAL = *((uint32_t*)pData);
    can->CAN_BUF[can_buf_indx].DATAH = *((uint32_t*)pData+1);
    can->BUF_CON[can_buf_indx] |= 1<<5;              // запрос на отправку
}

// Положить данные в программный буфер
__ramfunc void can2_putDataInBuf(uint8_t can_buf_indx, uint16_t msgId, uint8_t *pData, uint8_t len)
{  
    if(CAN_TX_REQ(MDR_CAN2, can_buf_indx)) return;     // аппаратный буфер не пуст
    if(len>8) len=8;
    MDR_CAN2->CAN_BUF[can_buf_indx].ID = msgId<<18;
    MDR_CAN2->CAN_BUF[can_buf_indx].DLC = len | 1<<11;    
    MDR_CAN2->CAN_BUF[can_buf_indx].DATAL = *((uint32_t*)pData);
    MDR_CAN2->CAN_BUF[can_buf_indx].DATAH = *((uint32_t*)pData+1);    
    MDR_CAN2->BUF_CON[can_buf_indx] |= 1<<5;              // запрос на отправку
}

// Обработчик прерываний CAN1
void CAN2_IRQHandler()
{ 
// Завершена передача 
   //if((MDR_CAN2->BUF_CON[CAN_MONITOR_TX_BUF] & (1<<5)) == 0) // аппаратный буфер пуст
   //{
   // onCanMonitorTransmitDone();
   //}
  
// Принят новый пакет
    uint8_t rxBuf[8];
    uint8_t len;
    if(can2_rx_new_packet(CAN_LDR_RX_BUF, rxBuf, &len))         // перепрошивка
    {
       process_canloader_frame(rxBuf, len);
    }  
    
    ////if(can2_rx_new_packet(CAN_TERM_RX_BUF, rxBuf, &len))        // терминал
    ////{
    ////   term_receive_msg(rxBuf, len);
    ////}
  
    if(can2_rx_new_packet(CAN_MONITOR_RX_BUF, rxBuf, &len))     // монитор
    {
       canMonitor_rxFrameHandler(rxBuf, len);
    }
    
    if(can2_rx_new_packet(CAN_ANGLE_RX_BUF, rxBuf, &len))     // угол
    {
       angleService_rxAngle(rxBuf, len);
    }
    
    if(can2_rx_new_packet(CAN_GK_CONTROL_RX_BUF, rxBuf, &len))     // управление
    {
       canGkControl_rxMsg(rxBuf, len);
    }   
         
    if(can2_rx_new_packet(CAN_USTR_SOPR_BUF, rxBuf, &len))     // управление
    {
       extern void debugingInfo_rxFrameHandler(uint8_t *buf, uint8_t len);
       debugingInfo_rxFrameHandler(rxBuf, len);
    }   
    
}