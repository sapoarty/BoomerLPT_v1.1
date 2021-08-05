/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t i=0;
uint32_t n=0;
uint32_t battery=0;
int16_t resultADC[3200]={0};
uint16_t SerialNum=123;
uint8_t comRecv;
uint8_t argumentRecv;
uint16_t adcConvertCounter;
uint16_t batCounter=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

void compOn();
void compOff();
void boomer2kVOn();
void boomer2kVOffBoomOn();
void boomer2kVOffBoomOff();
void resetQuartzCounter();
void setRangeOfMeasure(int range);
void setFilterFreq1kHz();
void setFilterFreq2kHz();
void startADCConversion();
void start3200ConversADC(int range);
void getADCResult();

void setPortRD(int decByte);
void setPortRC(int decByte);
void setNresetPortRC();
int getPortRS();
void delayMicroseconds();

/* USER CODE BEGIN PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	setPortRD(0);
	setPortRC(1);
	boomer2kVOffBoomOff();
	HAL_UART_Receive_IT(&huart2, &comRecv, 1);

//	for(i=0;i<3200;i++)
//	{
//		resultADC[i]=i;
//	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

//		startADCConversion();
		//	  HAL_Delay(1000);
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,1);
//		delayMicroseconds(100);
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,0);
//		delayMicroseconds(50);
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc1);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1800;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA11 PA12 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	argumentRecv=(comRecv&240)>>4;
	comRecv=comRecv&15;
	if (comRecv==0)//������ ���������
	{
		HAL_UART_Transmit_IT(&huart2,(uint8_t *)&SerialNum,2);
	}
	if (comRecv==1)//��������� �����
	{
		compOn();
	}
	if (comRecv==2)//������ �������
	{
		batCounter=1000;
		HAL_ADC_Start_IT(&hadc1);
	}
	if (comRecv==3)//���������� �����
	{
		compOff();
	}
	if (comRecv==4)//���. 2��
	{
		boomer2kVOn();
	}
	if (comRecv==5)//����. 2�� � ���������� ������
	{
		boomer2kVOffBoomOn();
	}
	if (comRecv==6)//����. 2�� � ����������� ������
	{
		boomer2kVOffBoomOff();
	}
	if (comRecv==7)//����� ������
	{
		resetQuartzCounter();
	}
	if (comRecv==8)//������ �������� ���������
	{
		setRangeOfMeasure(argumentRecv);
	}
	if (comRecv==9)//������ 1 ���
	{
		setFilterFreq1kHz();//������ 1 ���
	}
	if (comRecv==10)//������ 2 ���
	{
		setFilterFreq2kHz();//������ 2 ���
	}
	if (comRecv==11)//����� �������������� ���
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
		startADCConversion();//����� �������������� ���
	}
	if (comRecv==12)//������ �������� ���������
		{
			start3200ConversADC(argumentRecv);
		}

	if (comRecv==13)//����
		{
		HAL_TIM_Base_Stop_IT(&htim3);
		compOn();
		boomer2kVOffBoomOn();
		}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
	HAL_UART_Receive_IT(&huart2, &comRecv, 1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (comRecv==2)///������ �������
	{
		if (batCounter)
		{
			batCounter--;
			HAL_ADC_Stop_IT(&hadc1);
			battery = battery + HAL_ADC_GetValue(&hadc1);
			delayMicroseconds(10);
			HAL_ADC_Start_IT(&hadc1);
		}
		else
		{
			battery=battery/100;
			HAL_UART_Transmit_IT(&huart2,(uint8_t *)&battery,2);
		}
		}
}


void start3200ConversADC(int range){
	setRangeOfMeasure(range);
	compOff();//2
	boomer2kVOffBoomOff();//5
	delayMicroseconds(100);
	boomer2kVOffBoomOn();//4
	delayMicroseconds(200);
	boomer2kVOffBoomOff();//5
	resetQuartzCounter();//6
	adcConvertCounter=3200;
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
	HAL_TIM_Base_Start_IT(&htim3);
//	while(adcConvertCounter)
//	{
//		adcConvertCounter--;
//		delayMicroseconds(23);
//		startADCConversion();
//	}
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
//	boomer2kVOffBoomOff();
//	boomer2kVOn();
//	compOn();
//	HAL_UART_Transmit_IT(&huart2,(uint8_t *)&resultADC,6400);
}

void compOn(){//1
	setPortRD(16);
	setNresetPortRC();
}

void compOff(){//2
	setPortRD(17);
	setNresetPortRC();
}

void boomer2kVOn(){//3
	setPortRD(43);
	setNresetPortRC();
}

void boomer2kVOffBoomOn(){//4
	setPortRD(40);
	setNresetPortRC();
}

void boomer2kVOffBoomOff(){//5
	setPortRD(41);
	setNresetPortRC();
}

void resetQuartzCounter(){//6
	setPortRD(56);
	setNresetPortRC();
}

void setRangeOfMeasure(int range){//7
	setPortRD(range);
	setNresetPortRC();
}

void setFilterFreq1kHz(){//8
	setPortRD(24);
	setNresetPortRC();
}

void setFilterFreq2kHz(){//9
	setPortRD(25);
	setNresetPortRC();
}

void startADCConversion(){//10
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
	setPortRD(48);
	setNresetPortRC();
	setPortRD(0);
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)){

}
	getADCResult();
}

void setPortRD(int decByte){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,(decByte&32)>>5);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,(decByte&16)>>4);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,(decByte&8)>>3);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,(decByte&4)>>2);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,(decByte&2)>>1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,(decByte&1));
}

void setPortRC(int decByte){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,decByte);
}

void setNresetPortRC(){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,1);
//	delayMicroseconds(1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,0);
	HAL_UART_Receive_IT(&huart2, &comRecv, 1);
}


void delayMicroseconds(int microseconds){
	microseconds=microseconds*5.8;
	while(microseconds>0){
		microseconds--;
	}
}


void getADCResult(){
		resultADC[adcConvertCounter]=0;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,0);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,0);
		resultADC[adcConvertCounter]|=(GPIOA->IDR & 0x1E00)>>9;

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,0);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,1);
		resultADC[adcConvertCounter]|=(GPIOA->IDR & 0x1E00)>>5;

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,0);
		resultADC[adcConvertCounter]|=(GPIOA->IDR & 0x1E00)>>1;

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,1);
		resultADC[adcConvertCounter]|=(GPIOA->IDR & 0x1E00)<<3;
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
