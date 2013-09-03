/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include "MPU6050.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
//Bluetooth cstes
#define BT_BAUD 9200
#define MAX_STRLEN 20 // this is the maximum string length of our string in characters

#define CONNECTED_LENGTH 20

//On = 2ms Off=1ms
#define SPEED_100 6000
#define SPEED_MIDDLE 3000
#define SPEED_0 2700

#define LX 0xFF
#define LY 0xFE
#define RX 0xFD
#define RY 0xFC
#define ASK_POS 0xFB
#define STOP_POS 0xFA

/* Exported macro ------------------------------------------------------------*/
static float interval = 100.0f / ((float) (SPEED_100 - SPEED_0));
/* Exported functions ------------------------------------------------------- */
//Init
void init_USART1(uint32_t baudrate);
void initPA15();
void  initPB0();
void initPA14();
void init_LED(void);
void TIM3_Config();
void TIM6_Config();
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
int8_t getSpeed(float value);

float pid_calculate(float target, float actual, uint8_t key);
float kalman_calculate(float acc, float gyro, uint8_t looptime, uint8_t key);
uint8_t parseFloat(float f);
void Calibrate_Gyros();
void Get_Accel_Angles(float *xangle,float *yangle);
void Zero_Sensors();
void getAngles(float *a,float *b);
void analyseString(uint8_t cnt);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
