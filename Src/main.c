
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "sd_hal_mpu6050.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* Para enviar como string, defina SENDNUMBER como 0. Para enviar um buffer de bytes, defina como 1*/
#define SENDNUMBER 1

/* Define endereços dos dados (measureID) de acordo com o protocolo FTCAN2.0*/
#define TPS_ADDR 0x0002
#define RPM_ADDR 0x0084
#define BAT_V_ADDR 0x0012
#define TPM_MOT_ADDR 0x0008
#define OILP_ADDR 0x000A
#define SPEEDFR_ADDR 0x001A
#define FUELP_ADDR 0x000C
#define MAP_ADDR 0x0004
#define DRSFRPM 0x0020

/* Define endereços das temperaturas do Freio*/

/*Variaveis globais dos dados*/
uint16_t RPM = 0,  SPEEDFR = 0, TIMERCOUNT=0, POSVOL = 0, PFREIOT = 0, PFREIOD =0, TEMPPDU = 0,TPS, ECT, BAT, OILP,FUELP, CORRENTE=0, SUSP = 0,TEMPBREAK_1=0, TEMPBREAK_2=0;
float  MAP;
uint8_t BOMBA, VENT, SPARKC, BEACON = 0;

CanTxMsgTypeDef TxTemp;
CanTxMsgTypeDef TxSpeed;
CanTxMsgTypeDef adc1;

/* UART */
char tx_buffer[100];
char tx_buffer2[80];
char tx_buffer3[80];
char tx_buffer4[100];
uint8_t pack3_cnt = 0, pack2_cnt = 0, PACKNO;

uint8_t buff1[15], buff2[20], buff3[15] ,buff4[30];

/*I2c acelerometro*/
SD_MPU6050 mpu1;
char acel_result[100];
int16_t g_x, g_y, g_z, a_x, a_y, a_z;

/* Recepcao de pacotes CAN */
uint32_t canID =0, callbackCnt = 0;
uint8_t  datafieldID;
uint16_t payloadLength, payloadCnt = 0;
uint8_t segPackNo = 0;
uint8_t payloadData[80];
uint8_t canData[8];

/* ADC */
uint32_t adc_buffer[600], ADC[6]= {0,0,0,0,0,0};

/* Extensometros */
uint8_t dados_ext[8], RECEIVED_ETX = 0;
uint8_t EXT_ARRAY[8];
unsigned long EXT_DATA[8] = {0,0,0,0,0,0,0,0};
uint8_t CANTX = 0, CANRX = 0;


/*  vent = B4
  bomba = A15
beacon = a8
sensor de corrente = b0
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* Declara funcoes proprias do usuario */
static void CAN_Config(void);
void TR_CAN_Transmit( CanTxMsgTypeDef *MessageTX, uint8_t Data_size, uint8_t Data_array[]);
void UserMsgConfig(void);
void getMeasure(uint8_t address, uint16_t value);
void sendFrame(void);

/*Chama essa função quando completa a conversao dos canais ADC*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	int i;
		for(i=300;i<=594;i+=6){
			ADC[0] = adc_buffer[i]+ ADC[0];
			ADC[1] = adc_buffer[i+1]+ ADC[1];
			ADC[2] = adc_buffer[i+2]+ ADC[2];
			ADC[3] = adc_buffer[i+3]+ ADC[3];
			ADC[4] = adc_buffer[i+4]+ ADC[4];
			ADC[5] = adc_buffer[i+5]+ ADC[5];
		}
	ADC[0] = ADC[0]/100;
	ADC[1] = ADC[1]/100;
	ADC[2] = ADC[2]/100;
	ADC[3] = ADC[3]/100;
	ADC[4] = ADC[4]/100;
	ADC[5] = ADC[5]/100;


	PFREIOT = ADC[0];
	PFREIOD = ADC[1];
	POSVOL = ADC[2];
	TEMPPDU = ADC[3];
	CORRENTE = ADC[4];
	SUSP = ADC[5];
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	int i;
	for(i=0;i<=294;i+=6){
		ADC[0] = adc_buffer[i]+ ADC[0];
		ADC[1] = adc_buffer[i+1]+ ADC[1];
		ADC[2] = adc_buffer[i+2]+ ADC[2];
		ADC[3] = adc_buffer[i+3]+ ADC[3];
		ADC[4] = adc_buffer[i+4]+ ADC[4];
		ADC[5] = adc_buffer[i+5]+ ADC[5];

	}
	//ADC[0] = ADC[0]/5;
	//ADC[1] = ADC[1]/5;
	//ADC[2] = ADC[2]/5;
	//ADC[3] = ADC[3]/5;
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

   SD_MPU6050_Result result;
   char mpu_ok[20] = {"MPU WORK FINE\n\r"};
   char mpu_not[20] = {"MPU NOT WORKING\n"};
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /*Delay para esperar FT500 estabelecer comunicacao com WB-O2*/
  HAL_Delay(4000);

  /* Funcoes de inicialicao das mensagens CAN */
  CAN_Config();
  UserMsgConfig();

  /*Acelerometro*/
  SD_MPU6050_Init (&hi2c2, &mpu1, SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G, SD_MPU6050_Gyroscope_250s);

  /* Inicializa ADC no modo circular, carregando resultados no buffer adc_buffer*/
  HAL_ADC_Start_DMA(&hadc1, adc_buffer, 600);

  /* Sinaliza para timer começar a contagem */
  HAL_TIM_Base_Start_IT(&htim3);

  /* Chama função de recepção de frames CAN  */
  if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
    {
      /* Reception Error */
     Error_Handler();
    }

  /*LED C13 pisca para sinalizar que o progrma começou*/
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Pisca LED para sinalizar que há um programa rodando */
	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	//  result = SD_MPU6050_Init (&hi2c2, &mpu1, SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G, SD_MPU6050_Gyroscope_250s);
	//  HAL_Delay(100);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


	 	 //SD_MPU6050_ReadAccelerometer(&hi2c2,&mpu1);
	 //	 a_x = mpu1.Accelerometer_X;
	 	// a_y = mpu1.Accelerometer_Y;
	 //	 a_z = mpu1.Accelerometer_Z;
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_5TQ;
  hcan.Init.BS2 = CAN_BS2_6TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 36000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void sendFrame(void){
	hcan.pTxMsg->Data[0] = TIMERCOUNT>>8;
	hcan.pTxMsg->Data[1] = TIMERCOUNT;
	if (HAL_CAN_Transmit_IT(&hcan) != HAL_OK) {
			   /* Transmition Error */
				// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			   //Error_Handler();
	}
}

/* Recebe endereço do measureID do dado recebido pelo barramento e seu valor e atualiza a variavel global correspondente*/
void getMeasure(uint8_t address, uint16_t value){

	switch(address){
	case TPS_ADDR:
		//TPS = value*0.1;
		TPS = value;
		break;
	case RPM_ADDR:
		RPM = value;
		break;
	case  OILP_ADDR:
		//OILP = 0.001*value;
		OILP = value;
		break;
	case  MAP_ADDR:
		MAP = 0.001*value;
		break;
	case  FUELP_ADDR:
//		FUELP = 0.001*value;
		FUELP = value;
		break;
	case SPEEDFR_ADDR:
		SPEEDFR = value;
		break;
	case BAT_V_ADDR:
//		BAT = 0.01*value;
		BAT = value;
		break;
	case TPM_MOT_ADDR:
		//ECT = 0.1*value;
		ECT = value;
		break;
	case	DRSFRPM:
		RPM = value;
	break;
	default:
		break;
	}
}

	/* Essa funcao configura cada tipo de mensagem (para transmissao) com sua ID e tamanho de dados (DLC)
	 * As mensagens foram criadas em private variables
	 * Nesse codigo, utiliza-se o modo extendido
	 */
void UserMsgConfig(void){

	/* Mensagem 1 -> TxTemp */
	TxTemp.StdId = 0x321;
	TxTemp.ExtId = 0x321;
	TxTemp.RTR = CAN_RTR_DATA;
	TxTemp.IDE = CAN_ID_EXT;
	TxTemp.DLC = 4;

	/* Mensagem 2 -> TxSpeed */
	TxSpeed.StdId = 0x125;
	TxSpeed.ExtId = 0x125;
	TxSpeed.RTR = CAN_RTR_DATA;
	TxSpeed.IDE = CAN_ID_EXT;
	TxSpeed.DLC = 2;

	/* Mensagem 3 -> adc1 */
	adc1.StdId = 0x125;
	adc1.ExtId = 0x3244;
	adc1.RTR = CAN_RTR_DATA;
	adc1.IDE = CAN_ID_EXT;
	adc1.DLC = 2;
}


	/* Entra como parametro o endereco da mensagem (tipo CanTxMsgTypeDef) ,
	 * quantos bytes a serem enviados e o array de dados.
	 * Envia ao barramento por meio de HAL_CAN_Transmit_IT(&hcan)
	 */
void TR_CAN_Transmit( CanTxMsgTypeDef *MessageTX, uint8_t Data_size, uint8_t Data_array[]){

	 /*##-1- Configure the CAN peripheral #######################################
	  * Associa a estrutura de mensagem a ser transmitida ao campo pTxMsg do Can Handle hcan
	  */
	hcan.pTxMsg = MessageTX;
	hcan.pTxMsg->DLC = Data_size;

	int i;
	for(i=0;i<Data_size;i++){
		hcan.pTxMsg->Data[i] = Data_array[i];
	}

	/*transmite frame*/
	if (HAL_CAN_Transmit_IT(&hcan) != HAL_OK){
	/* Transmition Error */
		Error_Handler();
	}
}


static void CAN_Config(void)
{
  CAN_FilterConfTypeDef  sFilterConfig;
 static CanTxMsgTypeDef        TxMessage;     //--
 static CanRxMsgTypeDef        RxMessage;     //-- codigo original

  /*##-1- Configure the CAN peripheral #######################################*/
   hcan.pTxMsg = &TxMessage;     //-- codigo original
   hcan.pRxMsg = &RxMessage;	  // --



  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Configure Transmission process #####################################*/
  hcan.pTxMsg->StdId = 0x321;
  hcan.pTxMsg->ExtId = 0x301<<19;
  hcan.pTxMsg->RTR = CAN_RTR_DATA;
  /*configurando como extended id (29 bits)*/
  hcan.pTxMsg->IDE = CAN_ID_EXT;
  hcan.pTxMsg->DLC = 2;
}


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{

	CANRX = 1;

	/* LED 13 muda de estado toda vez que é chamada a funcão callback, ou seja, toda vez que recebe um pacote CAN */
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	uint8_t i, dataLength;

	/* Armazena a quantidade de bytes do frame na variavel dataLength e a ID em canID */
	dataLength = hcan->pRxMsg->DLC;
	canID = hcan->pRxMsg->ExtId;

	/* Armazena dados recebidos em array canData*/
	for (i=0;i<dataLength;i++){
		canData[i] = hcan->pRxMsg->Data[i];
	}

	/*Se o ProductID for relativo a FT500 (0x280)*/
   if ((((hcan->pRxMsg->ExtId) >> 19) == 0x280) && (hcan->pRxMsg->IDE == CAN_ID_EXT)){

	datafieldID = (canID>>11)& 0x7;

	if (datafieldID != 0x2){
		//while(1){}
	}

	/* Testa se pacote é do tipo Single Packet*/
	if (canData[0] == 0xff){
	}

	/* Testa se pacote é do tipo segmented packet */
	/* Testa se é o primeiro pacote do segmented packet (valor 0 no primeiro byte)*/
	if ( canData[0] == 0){

		/* Zera variável que conta o indice da payload */
		payloadCnt = 0;

		/* Seta segPackNo como 1, indicando que o proximo pacote a ser recebido tem valor do primeiro byte como 1 */
		segPackNo = 1;

		/* Armazena tamanho da payload na variavel payloadLength. Tamanho da payload sao o segundo e
		 * terceiros bytes do primeiro pacote */
		payloadLength = ((canData[1] & 0x7) << 8) | canData[2];

		/* Nos 5 bytes restantes, armazena payload no array */
		for (i=3;i<dataLength;i++){
			payloadData[payloadCnt] = canData[i];
			payloadCnt++;
		}
	}

	/* Caso o pacote tenha o primeiro byte diferente de 0 */
	else{
		/* Caso seja o pacote com o número certo */
		if(canData[0] == segPackNo){
			segPackNo++;
			/* Armazena os dados recebidos na variavel payloadData, começando do byte 1 e indo até o ultimo byte recebido*/
			for (i=1;i<dataLength;i++){
				payloadData[payloadCnt] = canData[i];

				/*Dados sao enviados de 4 em 4 bytes, sendo os 2 primeiro relativos ao endereço especifico de cada um
				  e os 2 ultimos o dado
				  Faz divisao por 4 e pega o resto.
				*/
				if(payloadCnt % 4 == 3){
					uint8_t add = payloadData[payloadCnt-2];

					/*Junta os 2 bytes correspondentes ao valor do dado*/
					uint16_t value = (payloadData[payloadCnt-1]<<8)|payloadData[payloadCnt];

					getMeasure(add, value);
				}

				/* Chegou no final da payload */
				if((payloadCnt == payloadLength-1)){
					break;
				}

				payloadCnt++;
			}
		}
	}
  }

   /* Pacote da placa dos extensometros  */
  else if ((((hcan->pRxMsg->ExtId) >> 19) == 0x300) && (hcan->pRxMsg->IDE == CAN_ID_EXT)){

	RECEIVED_ETX = 1;
	for(i=0;i<8;i++){
		EXT_ARRAY[i] = canData[i];
	}

	switch(EXT_ARRAY[0]){
	case 0xA:
	EXT_DATA[0] = (EXT_ARRAY[1]<<16) | (EXT_ARRAY[2]<<8) | EXT_ARRAY[3];
	EXT_DATA[1] = (EXT_ARRAY[5]<<16) | (EXT_ARRAY[6]<<8) | EXT_ARRAY[7];
	break;
	case 0xB:
	EXT_DATA[2] = (EXT_ARRAY[1]<<16) | (EXT_ARRAY[2]<<8) | EXT_ARRAY[3];
	EXT_DATA[3] = (EXT_ARRAY[5]<<16) | (EXT_ARRAY[6]<<8) | EXT_ARRAY[7];
	break;
	case 0xC:
	EXT_DATA[4] = (EXT_ARRAY[1]<<16) | (EXT_ARRAY[2]<<8) | EXT_ARRAY[3];
	EXT_DATA[5] = (EXT_ARRAY[5]<<16) | (EXT_ARRAY[6]<<8) | EXT_ARRAY[7];
	break;
	case 0xD:
	EXT_DATA[6] = (EXT_ARRAY[1]<<16) | (EXT_ARRAY[2]<<8) | EXT_ARRAY[3];
	EXT_DATA[7] = (EXT_ARRAY[5]<<16) | (EXT_ARRAY[6]<<8) | EXT_ARRAY[7];
	break;

	}

  }

  else if ((((hcan->pRxMsg->ExtId) >> 19) == 0x302) && (hcan->pRxMsg->IDE == CAN_ID_EXT)){
	  TEMPBREAK_1 = ((canData[1]<<8)|canData[2]);
	  TEMPBREAK_2 = ((canData[4]<<8)|canData[5]);
  }
  callbackCnt++; //Contador de quantos callbacks foram chamados

  /* Chama funcao para receber novo pacote */
  if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
  {
    Error_Handler();
  }
  CANRX = 0;

}

/* Quando UART termina de enviar, gera uma interrupção que chama esse callback */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

}

/*
 * Timer 3 -> clock vem de APB1 = 72Mhz
 * Prescaler de 36000 -> 72Mhz/36k = 2000Hz frequencia do timer. Prescaler divide o clock do timer
 * Counter Period = 49
 * Frequencia de interrupts = 2000/( 49 + 1 ) = 50Hz
 * Assim como o nome diz, HAL_TIM_PeriodElapsedCallback é o callback da interrupção gerada quando a contagem do timer
 * chega no limite, o couter period. Nesse caso Counter Period foi setado no Cube como 49, pois começando do 0, isso
 * corresponde a 50 periodos do timer. Dessa forma, com uma frequencia do timer configurada para 2000Hz, temos
 * uma interrupção a cada 20ms
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	TIMERCOUNT++;
	SPARKC = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	BEACON = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	BEACON=1;

	if(SENDNUMBER == 0){



		/*Mensagem para interface, pacote 1 (de 50Hz) com ext sem acelerometro*/
		//sprintf(tx_buffer, "%d %d %d %lu %lu %lu %lu %d\n",1, SPEEDFR, SPARKC, EXT_DATA[0], EXT_DATA[1], EXT_DATA[2], EXT_DATA[3],TIMERCOUNT);

		/*Mensagem para interface, pacote de 50Hz com acelerometro e ext*/
		//sprintf(tx_buffer, "%d %d %d %d %d %d %lu %lu %lu %lu %lu %lu %lu %lu %d\n", 1, a_x, a_y, a_z, SPEEDFR, SPARKC,EXT_DATA[0], EXT_DATA[1], EXT_DATA[2], EXT_DATA[3],0,0,0,0, TIMERCOUNT);


		/*Mensagem para interface, pacote de 50Hz so com acelerometro*/

		SUSP = 0;
		sprintf(tx_buffer, "%d %d %d %d %d %d %d %d\n", 1, a_x, a_y, a_z, SPEEDFR, SPARKC,SUSP, TIMERCOUNT);/*Mensagem para interface novo jeito, pacote de 50Hz*/

			//assemblePackage(1);


		/* Transmite pacote de 50Hz (pacote 1) e espera UART estar liberado p/ transmitir */


		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}

		HAL_UART_Transmit_DMA(&huart1, tx_buffer, strlen(tx_buffer));
		//HAL_UART_Transmit_DMA(&huart1, buff1, 14);
		if((pack2_cnt ==0)&& (RECEIVED_ETX == 1)){
				/*Mensagem para interface pacote so com ext (pacote 4)*/
				sprintf(tx_buffer4, "%d %lu %lu %lu %lu %lu %lu %lu %lu %d \n",  4,EXT_DATA[0], EXT_DATA[1], EXT_DATA[2], EXT_DATA[3],EXT_DATA[4],EXT_DATA[5],EXT_DATA[6],EXT_DATA[7], TIMERCOUNT+1);

				/* Transmissao do pacote 4. Espera UART estar liberado p/ transmitir */
				while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
				HAL_UART_Transmit_DMA(&huart1, tx_buffer4, strlen(tx_buffer4));
		}
		/* Define frequencias dos pacotes 2 e 3*/

		if(pack2_cnt == 1){
			/*Carrega o buffer do pacote 2*/


			sprintf(tx_buffer2, "%d %d %d %d %d %d %d %d %d %d\n", 2, OILP, FUELP, TPS, PFREIOT, PFREIOD,POSVOL, BEACON,CORRENTE, TIMERCOUNT);

			/* Transmissao do pacote 2. Espera UART estar liberado p/ transmitir */
			while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
			HAL_UART_Transmit_DMA(&huart1, tx_buffer2, strlen(tx_buffer2));

			/*Reseta counter do pacote 2*/
			pack2_cnt = -1;
			if(pack3_cnt == 19){

				VENT = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
				BOMBA = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

				/* Carrega de uma vez o buffer do pacote 3 */
				TEMPPDU = 0;
				sprintf(tx_buffer3, "%d %d %d %d %d %d %d %d %d\n", 3, ECT,  BAT, BOMBA, VENT, TEMPPDU, TEMPBREAK_1, TEMPBREAK_2, TIMERCOUNT);
				//BAT = 2;
				/* Transmissao do pacote 3. Espera UART estar liberado p/ transmitir */
				while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
				HAL_UART_Transmit_DMA(&huart1, tx_buffer3, strlen(tx_buffer3));
				pack3_cnt = -1;
			}
		}
	}
	else{

		/*Mensagem para interface, pacote de 50Hz so com acelerometro*/

		assemblePackage(1);

		/* Transmite pacote de 50Hz (pacote 1) e espera UART estar liberado p/ transmitir */
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}

		HAL_UART_Transmit_DMA(&huart1, buff1, 15);
		if((pack2_cnt ==0)&& (RECEIVED_ETX == 1)){
			/*Mensagem para interface pacote so com ext (pacote 4)*/
			assemblePackage(4);

			/* Transmissao do pacote 4. Espera UART estar liberado p/ transmitir */
			while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
			HAL_UART_Transmit_DMA(&huart1, buff4, 30);
		}
		/* Define frequencias dos pacotes 2 e 3.
		 * Ja carrega de uma vez o buffer do pacote 2 e 4*/

		assemblePackage(2);

		if(pack2_cnt == 1){
			/* Transmissao do pacote 2. Espera UART estar liberado p/ transmitir */

			while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
			HAL_UART_Transmit_DMA(&huart1, buff2, 20);
			pack2_cnt = -1;
			if(pack3_cnt == 19){

				VENT = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
				BOMBA = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

				/* Carrega de uma vez o buffer do pacote 3 */
				assemblePackage(3);

				/* Transmissao do pacote 3. Espera UART estar liberado p/ transmitir */
				while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
				HAL_UART_Transmit_DMA(&huart1, buff3, 15);
				pack3_cnt = -1;
			}
		}
	}
	/* Funcoes do acelerometro */
	SD_MPU6050_ReadAccelerometer(&hi2c2,&mpu1);
	a_x = mpu1.Accelerometer_X;
	a_y = mpu1.Accelerometer_Y;
	a_z = mpu1.Accelerometer_Z;
	pack2_cnt++;
	pack3_cnt++;
}

void assemblePackage(uint8_t n){

	switch(n){

	case 1:
		buff1[0]=1;
		buff1[1]= 5;
		buff1[2]=a_x >> 8;
		buff1[3]= a_x;
		buff1[4]=a_y >> 8;
		buff1[5]= a_y;
		buff1[6]=a_z >> 8;
		buff1[7]= a_z;
		buff1[8] = SPEEDFR;
		buff1[9] = (SUSP>>8) | (SPARKC<<7);
		buff1[10] = SUSP;   //0 a 4095
		buff1[11] = TIMERCOUNT>>8;
		buff1[12] = TIMERCOUNT;
		buff1[13] = 9;
		buff1[14] = '\n';
		break;
	case 2:
		buff2[0]=2;
		buff2[1]=5;
		buff2[2]=OILP>>8;
		buff2[3]=OILP ;
		buff2[4]= FUELP>>8;
		buff2[5]= FUELP;
		buff2[6]=TPS>>8; // de 0 a 1000, 10bits
		buff2[7]= TPS;
		buff2[8]= PFREIOT>>8;
		buff2[9]=PFREIOT;
		buff2[10]= PFREIOD>>8;
		buff2[11] = PFREIOD;
		buff2[12] = POSVOL>>8;
		buff2[13] = POSVOL;
		buff2[14] = (CORRENTE>>8) | (BEACON<<7);
		buff2[15] = CORRENTE;
		buff2[16] = TIMERCOUNT>>8;
		buff2[17] = TIMERCOUNT;
		buff2[18] = 9;
		buff2[19] = '\n';
			break;
	case 3:
		buff3[0]=3;
		buff3[1]=ECT>>8;
		buff3[2]=ECT ;
		buff3[3]= BAT>>8; //max1500
		buff3[4]= BAT;
		buff3[5]= (TEMPPDU>>8) | (BOMBA<<7) | (VENT<<5); //B0V0
		buff3[6]=TEMPPDU;
		buff3[7]=TEMPBREAK_1>>8;
		buff3[8]=TEMPBREAK_1 ;
		buff3[9]=TEMPBREAK_2>>8;
		buff3[10]=TEMPBREAK_2 ;
		buff3[11] = TIMERCOUNT>>8;
		buff3[12] = TIMERCOUNT;
		buff3[13] = 9;
		buff3[14] = '\n';
			break;
	case 4:
		buff4[0] = 4;
		buff4[1] = 10;
		buff4[2] = EXT_DATA[0] >> 16;
		buff4[3] = EXT_DATA[0] >> 8;
 	    buff4[4] = EXT_DATA[0];
 	    buff4[5] = EXT_DATA[1] >> 16;
 	    buff4[6] = EXT_DATA[1] >> 8;
 	    buff4[7] = EXT_DATA[1];
 	    buff4[8] = EXT_DATA[2] >> 16;
 	    buff4[9] = EXT_DATA[2] >> 8;
 	    buff4[10] = EXT_DATA[2];
 	    buff4[11] = EXT_DATA[3] >> 16;
 	    buff4[12] = EXT_DATA[3] >> 8;
 	    buff4[13] = EXT_DATA[3];
 	    buff4[14] = EXT_DATA[4] >> 16;
 	    buff4[15] = EXT_DATA[4] >> 8;
 	    buff4[16] = EXT_DATA[4];
 	    buff4[17] = EXT_DATA[5] >> 16;
 	    buff4[18] = EXT_DATA[5] >> 8;
 	    buff4[19] = EXT_DATA[5];
 	    buff4[20] = EXT_DATA[6] >> 16;
 	    buff4[21] = EXT_DATA[6] >> 8;
 	    buff4[22] = EXT_DATA[6];
 	    buff4[23] = EXT_DATA[7] >> 16;
 	    buff4[24] = EXT_DATA[7] >> 8;
 	    buff4[25] = EXT_DATA[7];
		buff4[26] = TIMERCOUNT>>8;
		buff4[27] = TIMERCOUNT;
		buff4[28] = 9;
		buff4[29] = '\n';

			break;
	default:
			break;

	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	  break;

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
