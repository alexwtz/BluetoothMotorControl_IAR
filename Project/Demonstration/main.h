/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery_lis302dl.h"
#include <stdio.h>
#include "MPU6050.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
//Bluetooth cstes
#define BT_BAUD 9200
#define MAX_STRLEN 20 // this is the maximum string length of our string in characters

#define CONNECTED_LENGTH 20

//On = 2ms Off=1ms
#define SPEED_100 60
#define SPEED_MIDDLE 30
#define SPEED_0 27

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
//Init
void init_LIS302DL();
void init_USART1(uint32_t baudrate);
void initPA15();
void  initPB0();
void initPA14();
void init_LED(void);
void TIM3_Config();
void TIM1_Config();
void init_BT_serial();

//Config
void PWM1_Config(int period);
void PWM3_Config(int period);
void PWM_SetDC(uint16_t channel,uint16_t dutycycle);
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
void setPA15(int val);
void setPA15On();
void setPB0(int val);
void togglePA15();
void setPA14Off();
void UARTSend(const unsigned char * pucBuffer, unsigned long ulCount);
int8_t getSpeed(int8_t motorId,int8_t horizon);
uint32_t LIS302DL_TIMEOUT_UserCallback(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
