#include "main.h"

/* Private variables ---------------------------------------------------------*/
//UART connection  
static __IO uint32_t TimingDelay;
static u8 sendPosition = 0;
volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string

//Command variables
static uint8_t lx,ly,rx,ry;
static int8_t motorCor[4];
bool init = true;

//LIS302DL accelerometer
LIS302DL_InitTypeDef  LIS302DL_InitStruct;
LIS302DL_FilterConfigTypeDef LIS302DL_FilterStruct;  
__IO int8_t X_Offset, Y_Offset, Z_Offset  = 0x00;
uint8_t Buffer[6];

//PWM
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;

/* Method definition ---------------------------------------------------------*/

int main(void) {
  
  u8 loop = 1;
  u8 i = 0;
  
  //Variables initialization
  motorCor[0] = 0;
  motorCor[1] = 0;
  motorCor[2] = 0;
  motorCor[3] = 0;
    
  //Initialisation of the MCU
  init_LED();
  initPB0();
  initPA15();
  init_USART1(BT_BAUD);
  init_LIS302DL();
  
  //PWM config (motor control)  
  TIM1_Config();
  TIM3_Config();
  PWM1_Config(100);
  PWM3_Config(100);
    
  while(loop){
    //Update motor control
    PWM_SetDC(1,getSpeed(1));//PE9 | PC6//ON 2ms
    PWM_SetDC(2,10+getSpeed(2));//PE11 | PC 7
    PWM_SetDC(3,20+getSpeed(3));//PE13
    PWM_SetDC(4,30+getSpeed(4));//PE14
    
    //Read and print the accelerometer values
    LIS302DL_Read(Buffer, LIS302DL_OUT_X_ADDR, 6);
    printf("%d, %d ,%d\n",Buffer[0],Buffer[2],Buffer[4]);
  
    if(sendPosition){
    //Send the command
    unsigned char cmd[] = "C1";
    UARTSend(cmd,sizeof(cmd));
    
    //Send data through the bluetooth communication
    UARTSend(Buffer, sizeof(Buffer));
    }
    //Wait some time befor ending the loop
    if(i++%2)GPIOD->BSRRL = 0x1000; // this sets LED1 (green)
    else GPIOD->BSRRH = 0x1000;
    GPIOD->BSRRH = 0x8000;
    
    Delay(1000);
  }
  
    /* Disable SPI1 used to drive the MEMS accelerometre */
    SPI_Cmd(LIS302DL_SPI, DISABLE);
  
    /* Disable the UART connection */
    USART_Cmd(USART1, DISABLE);
}

/*- Normal method ------------------------------------------------------------*/

/**
*Method used to get the correct PWM frequency according to the command
*@return PWM speed
*/
int getSpeed(int motorId){
  if(init){
    return SPEED_MIDDLE;
  }else{
  float dif = (float)(SPEED_100- SPEED_0);
  float interval = 100.0f/dif;
  return (int8_t)(motorCor[motorId-1]+SPEED_0+(((float)ly)/interval));
  }
}

/**
* Method that send a string to the UART.
* @param *pcBuffer buffers to be printed.
*@param ulCount the buffer's length
*/
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
  //
  // Loop while there are more characters to send.
  //
  while(ulCount--)
  {
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USART1, (uint8_t) *pucBuffer++);
    /* Loop until the end of transmission */
    
  }
  
}

void setPB0(int val){
  if(val)GPIOB->BSRRL = GPIO_Pin_0;
    else GPIOB->BSRRH = GPIO_Pin_0;
}

void setPA15(int val){
    if(val)GPIOA->BSRRL = GPIO_Pin_15;
    else GPIOA->BSRRH = GPIO_Pin_15;
}

void setPA15On(){
  // GPIO port bit set/reset low register,  Address offset: 0x18      */
  GPIOA->BSRRL = GPIO_Pin_15;
}


void togglePA15(){
  //  GPIO port output data register,        Address offset: 0x14      */
  GPIOA->ODR ^= GPIO_Pin_15;
}

/**
* @brief  MEMS accelerometre management of the timeout situation.
*/
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
    while (1);
}

/*- Initialisation methods ---------------------------------------------------*/

/**
*@brief This method is the one provided by STMicroelectronic with the discoverycard demo. It is used to configure the LIS302DL accelerometer embedded on the discovery card.
*/
void init_LIS302DL(){
/* MEMS configuration */
LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
LIS302DL_InitStruct.Axes_Enable = LIS302DL_XYZ_ENABLE;
LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
LIS302DL_Init(&LIS302DL_InitStruct);

/* Required delay for the MEMS Accelerometre: Turn-on time = 3/Output data Rate 
= 3/100 = 30ms */
Delay(30);

/* MEMS High Pass Filter configuration */
LIS302DL_FilterStruct.HighPassFilter_Data_Selection = LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER;
LIS302DL_FilterStruct.HighPassFilter_CutOff_Frequency = LIS302DL_HIGHPASSFILTER_LEVEL_1;
LIS302DL_FilterStruct.HighPassFilter_Interrupt = LIS302DL_HIGHPASSFILTERINTERRUPT_1_2;
LIS302DL_FilterConfig(&LIS302DL_FilterStruct);
}

/* This funcion initializes the USART1 peripheral
*
* Arguments: baudrate --> the baudrate at which the USART is
* 						   supposed to operate
*/
void init_USART1(uint32_t baudrate){
  
  /* This is a concept that has to do with the libraries provided by ST
  * to make development easier the have made up something similar to
  * classes, called TypeDefs, which actually just define the common
  * parameters that every peripheral needs to work correctly
  *
  * They make our life easier because we don't have to mess around with
  * the low level stuff of setting bits in the correct registers
  */
  GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
  USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
  NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
  
  /* enable APB2 peripheral clock for USART1
  * note that only USART1 and USART6 are connected to APB2
  * the other USARTs are connected to APB1
  */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  /* enable the peripheral clock for the pins used by
  * USART1, PB6 for TX and PB7 for RX
  */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* This sequence sets up the TX and RX pins
  * so they work correctly with the USART1 peripheral
  */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;			// this activates the pullup resistors on the IO pins
  
  GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
  
  /* The RX and TX pins are now connected to their AF
  * so that the USART1 can take over control of the
  * pins
  */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
  
  /* Now the USART_InitStruct is used to define the
  * properties of USART1
  */
  USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
  USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
  USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
  USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
  
  
  /* Here the USART1 receive interrupt is enabled
  * and the interrupt controller is configured
  * to jump to the USART1_IRQHandler() function
  * if the USART1 receive interrupt occurs
  */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
  
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
  NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff
  
  // finally this enables the complete USART1 peripheral
  USART_Cmd(USART1, ENABLE);
}

/*- Interruption handler -----------------------------------------------------*/

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){
  
  // check if the USART1 receive interrupt flag was set
  if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
    
    init = false;
    static uint8_t cnt = 0; // this counter is used to determine the string length
    char t = USART1->DR; // the character from the USART1 data register is saved in t
        
    /* check if the received character is not the LF character (used to determine end of string)
    * or the if the maximum string length has been been reached
    */
    if( (t != 'n') && (cnt < MAX_STRLEN) ){
      received_string[cnt] = t;
      if(cnt == 4){
        if(received_string[cnt-4]=='L'){
          if(received_string[cnt-3]=='X'){
            lx = 100*(received_string[cnt-2]-48)+10*(received_string[cnt-1]-48)+received_string[cnt]-48;
          }else if(received_string[cnt-3]=='Y'){
            ly = 100*(received_string[cnt-2]-48)+10*(received_string[cnt-1]-48)+received_string[cnt]-48;
          }
        }else if(received_string[cnt-4]=='R'){
          if(received_string[cnt-3]=='X'){
            rx = 100*(received_string[cnt-2]-48)+10*(received_string[cnt-1]-48)+received_string[cnt]-48;
          }else if(received_string[cnt-3]=='Y'){
            ry = 100*(received_string[cnt-2]-48)+10*(received_string[cnt-1]-48)+received_string[cnt]-48;
          }
        }else if(received_string[cnt-4]=='M'){
          if(received_string[cnt-3]=='1'){
            motorCor[0] = (received_string[cnt-2]=='N'?-1:1)*10*(received_string[cnt-1]-48)+received_string[cnt]-48;
          }else if(received_string[cnt-3]=='2'){
            motorCor[1] = (received_string[cnt-2]=='N'?-1:1)*10*(received_string[cnt-1]-48)+received_string[cnt]-48;
          }else if(received_string[cnt-3]=='3'){
            motorCor[2] = (received_string[cnt-2]=='N'?-1:1)*10*(received_string[cnt-1]-48)+received_string[cnt]-48;
          }else if(received_string[cnt-3]=='4'){
            motorCor[3] = (received_string[cnt-2]=='N'?-1:1)*+10*(received_string[cnt-1]-48)+received_string[cnt]-48;
          }
        }
      }
      
      if(received_string[cnt-1]=='C' && t == '1')sendPosition = 1;
      if(received_string[cnt-1]=='C' && t == '0')sendPosition = 0;
      if(cnt%2)GPIOD->BSRRL = 0x8000; // this sets LED4
      else GPIOD->BSRRH = 0x8000; // this sets LED4
      cnt++;
      
    }
    else{ // otherwise reset the character counter
      cnt = 0;
    }
    
  }
}

void initPB0(){
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void initPA15(){
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TIM3_Config()
{
  GPIO_InitTypeDef GPIO_InitStructure;
 
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 
  /* GPIOC, GPIOB and GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);
   
  /* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
   
  /* GPIOB Configuration:  TIM3 CH3 (PB0) and TIM3 CH4 (PB1) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    /* Connect TIM3 pins to AF2 */ 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
}

  void PWM_SetDC(uint16_t channel,uint16_t dutycycle)
{
  if (channel == 1)
  {
    TIM3->CCR1 = dutycycle;
    TIM1->CCR1 = dutycycle;

  }
  else if (channel == 2)
  {
    TIM3->CCR2 = dutycycle;
    TIM1->CCR2 = dutycycle;
  }
  else if (channel == 3)
  {
    TIM3->CCR3 = dutycycle;
    TIM1->CCR3 = dutycycle;
  }
  else if (channel == 4)
  {
    TIM3->CCR4 = dutycycle;
    TIM1->CCR4 = dutycycle;
  }
}

/*- Timing methods -----------------------------------------------------------*/

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
*@brief Method used to wait a certain amount of time
*@param nCount the time you want to wait
*/
void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

/**
  * @brief  Configure the TIM1 Pins.
  * @param  None
  * @retval None
  */
void TIM1_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA and GPIOB clocks enable */
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE);

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
                         
  /* GPIOA Configuration: Channel 1 to 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  //  /* Connect TIM pins to AF1 */
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
}

void PWM1_Config(int period)
{

  /* -----------------------------------------------------------------------
  1/ Generate 3 complementary PWM signals with 3 different duty cycles:
    
    In this example TIM1 input clock (TIM1CLK) is set to 2 * APB2 clock (PCLK2), 
    since APB2 prescaler is different from 1 (APB2 Prescaler = 2, see system_stm32f4xx.c file).
      TIM1CLK = 2 * PCLK2  
      PCLK2 = HCLK / 2 
      => TIM1CLK = 2*(HCLK / 2) = HCLK = SystemCoreClock
         
    To get TIM1 counter clock at 168 MHz, the prescaler is computed as follows:
       Prescaler = (TIM1CLK / TIM1 counter clock) - 1
       Prescaler = (SystemCoreClock / 168 MHz) - 1 = 0
  
    The objective is to generate PWM signal at 17.57 KHz:
    - TIM1_Period = (SystemCoreClock / 17570) - 1

    To get TIM1 output clock at 17.57 KHz, the period (ARR) is computed as follows:
       ARR = (TIM1 counter clock / TIM1 output clock) - 1
           = 9561
    
  The Three Duty cycles are computed as the following description: 

    TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR)* 100 = 50%
    TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR)* 100 = 25%
    TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR)* 100 = 12.5%
    
    The Timer pulse is calculated as follows:
      - TIM1_CCRx = (DutyCycle * TIM1_ARR)/ 100
    
  2/ Insert a dead time equal to (11/SystemCoreClock) ns

  3/ Configure the break feature, active at High level, and using the automatic 
     output enable feature

  4/ Use the Locking parameters level1.
  
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */  
  
  /* Time Base configuration */
    uint16_t PrescalerValue = 0;
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 16000) - 1;
  
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1to 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  
  TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  
  TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);

  /* Automatic Output enable, Break, dead time and lock configuration*/
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
  TIM_BDTRInitStructure.TIM_DeadTime = 11;
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PWM3_Config(int period)
{
  uint16_t PrescalerValue = 0;
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /4) / 16000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

/* This funcion shows how to initialize
 * the GPIO pins on GPIOD and how to configure
 * them as inputs and outputs
 */
void init_LED(void){

	/* This TypeDef is a structure defined in the
	 * ST's library and it contains all the properties
	 * the corresponding peripheral has, such as output mode,
	 * pullup / pulldown resistors etc.
	 *
	 * These structures are defined for every peripheral so
	 * every peripheral has it's own TypeDef. The good news is
	 * they always work the same so once you've got a hang
	 * of it you can initialize any peripheral.
	 *
	 * The properties of the periperals can be found in the corresponding
	 * header file e.g. stm32f4xx_gpio.h and the source file stm32f4xx_gpio.c
	 */
	GPIO_InitTypeDef GPIO_InitStruct;

	/* This enables the peripheral clock to the GPIOD IO module
	 * Every peripheral's clock has to be enabled
	 *
	 * The STM32F4 Discovery's User Manual and the STM32F407VGT6's
	 * datasheet contain the information which peripheral clock has to be used.
	 *
	 * It is also mentioned at the beginning of the peripheral library's
	 * source file, e.g. stm32f4xx_gpio.c
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* In this block of instructions all the properties
	 * of the peripheral, the GPIO port in this case,
	 * are filled with actual information and then
	 * given to the Init function which takes care of
	 * the low level stuff (setting the correct bits in the
	 * peripheral's control register)
	 *
	 *
	 * The LEDs on the STM324F Discovery are connected to the
	 * pins PD12 thru PD15
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12; // we want to configure all LED GPIO pins
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOD, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.
}
