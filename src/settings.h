#ifndef __SETTINGS__
#define __SETTINGS__


#define CPU_CLOCK_MHZ                   80000000                                // частота CPU в МГц
#define EXT_GENERATOR_FREQ              16000000                                // частота внешнего генератора
#define CPU_PLL_KOEF                    ((CPU_CLOCK_MHZ/EXT_GENERATOR_FREQ)-1)  // коэф. умножения PLL


#define SELFLOADER_DEFAULT_NAME        ("MK Z")
#define SELFLOADER_DEFAULT_ID          0x82


// Частота ШИМ
#define TIM_PWM_CLK                     20000000        // частота тактирования таймера ШИМ
#define PWM_FREQ                        15000           // частота ШИМ
#define TIM_PWM_ARR_REG                 ((TIM_PWM_CLK/PWM_FREQ)-1)
#define TIM_PWM_60_PERCENTAGE_CCR_REG   (uint16_t)((60*TIM_PWM_ARR_REG)/100)
#define TIM_PWM_SET_MAX_FILLING(percentage)    (uint16_t)((percentage*TIM_PWM_ARR_REG)/100)     

// Защита по скорости
#define AVERAGE_VELOCITY_INTERVAL       100             // период усреднения
#define VELOCITY_PROTECTION_THRESHOLD   50.0            // защита по скорости

// Установка в Арретир
#define ARRETIR_VELOCITY                35.0                                    // скорость переброса в режиме АР
#define AR_THRESHOLD                    USYSANGLE_TO_CODE(2)                    // ограничение рассогласования по углу в схеме управления!

// Калибровка ДУС
#define ENGINE_STARTUP_DELAY            2000                                    // задержка включения двигателей после подачи питания
#define DUS_STARTUP_CALIBRATION_SAMPLES 5000                                    // количество отсчетов для усреднения ДУС
#define ARRETIER_CALIB_MAX_DEVIATION    USYSANGLE_TO_CODE(10.0/128.0)           // максимальное отклонение от арретира при калибровке в арретире
#define ARRETIER_CALIBRATION_SAMPLES    2000                                    // количество отсчетов для усреднения ДУС
#define TEMPERATURE_CALIBRATION_SAMPLES 5000                                    // количество отсчетов для усреднения ДУС при калибровке по температуре


// Использовать CAN с очередью и динамической памятью
#define USE_DYNAMIC_CAN_QUEUE


// Фильтр ДУС
#define DUS_FILTER_NONE
//#define DUS_FILTER_F4kHz_B100Hz
//#define DUS_FILTER_F4kHz_B200Hz
//#define DUS_FILTER_F4kHz_B300Hz
//#define DUS_FILTER_F4kHz_B500Hz
//#define DUS_FILTER_F2kHz_B200Hz_500Hz_20dB_80dB
//#define DUS_FILTER_F2kHz_B200Hz_SPECIFY_ORDER_2
//#define DUS_FILTER_F2kHz_B200Hz_500Hz_1dB_60dB                  // *
//#define DUS_FILTER_F2kHz_B500Hz_700Hz_1dB_60dB
//#define DUS_PID_FILTER_F2kHz_B200Hz_700Hz_2dB_26dB


// Фильтр ПИД
#define PID_FILTER_NONE
//#define PID_FILTER_F2kHz_B200Hz_500Hz_20dB_80dB
//#define PID_FILTER_F2kHz_B200Hz_SPECIFY_ORDER_2
//#define PID_FILTER_F2kHz_B200Hz_500Hz_1dB_60dB                //*
//#define PID_FILTER_F2kHz_B500Hz_700Hz_1dB_60dB
//#define PID_FILTER_F2kHz_B200Hz_700Hz_2dB_26dB







//#if (DUS_SAMPLING_FREQUENCY != 2000) && (DUS_SAMPLING_FREQUENCY != 4000)
//  #error "Incorrect DUS_SAMPLING_FREQUENCY value" 
//#endif









#endif //__SETTINGS__