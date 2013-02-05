/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery_lis302dl.h"
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define BT_BAUD 28800
#define MAX_STRLEN 20 // this is the maximum string length of our string in characters

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
void init_LIS302DL();
void init_USART1(uint32_t baudrate);
void initPA15();
void setPA15On();
void togglePA15();
void initPA14();
void setPA14Off();

void TIM_Config_PWM(void);
void PWM_Config(int period);
void PWM_SetDC(uint16_t channel,uint16_t dutycycle);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
