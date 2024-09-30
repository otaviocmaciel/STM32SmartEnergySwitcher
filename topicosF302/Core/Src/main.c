/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

OPAMP_HandleTypeDef hopamp2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef enum {ACSINE_CHK, BAT_CHK, PANEL_CHK}StateMachine_CHECK;
StateMachine_CHECK State_CHECK;

#define LEN_BUFFER 1000
volatile uint16_t adcBuffer[LEN_BUFFER];
volatile int BATBuffer, PANELBuffer;
volatile uint8_t DMA_FLAG;
volatile uint8_t CONV_START;
volatile uint16_t VMAX;
volatile uint8_t REG_STATUS = 0x0;
int ACSINE_VMIN = 3350;
int ACSINE_VMAX = 3860;
int BAT_VMIN = 2000;
int PANEL_VMIN = 2500;

typedef enum{AC, BAT, PANEL, NC}StateMachine_POWER;
StateMachine_POWER State_POWER = NC;

typedef enum{PRESET_1, PRESET_2, PRESET_3, PRESET_4, PRESET_5, PRESET_6}StateMachine_PRESET;
StateMachine_PRESET State_PRESET;
#define AC_AVAILABLE     (REG_STATUS & (1 << 0))  // Verifica se o bit 0 indica AC disponível
#define BAT_AVAILABLE    (REG_STATUS & (1 << 1))  // Verifica se o bit 1 indica Bateria disponível
#define PANEL_AVAILABLE  (REG_STATUS & (1 << 2))  // Verifica se o bit 2 indica Painel disponível

volatile int SW_Time;

typedef enum{IDLE, DBC, PRES}StateMachine_BUTTON;
StateMachine_BUTTON State_BUTTON;
#define READ_BUTTON (GPIOC->IDR &(1 << 13))
volatile uint8_t BUTTON_Time;
uint8_t POWER_Control;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//volatile uint16_t index_buffer = 0;
char rxBuffer[50];
//char uartBuffer[10000];
//int length = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int preset;
int vmin_ac, vmax_ac, vmin_bat;
int dados;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		// Usa sscanf para extrair os dados formatados da string
		  dados = sscanf(rxBuffer, "%d-%d-%d-%d", &preset, &vmin_ac, &vmax_ac, &vmin_bat);
		  if(dados == 4){
			  State_PRESET = preset;
			  ACSINE_VMIN = vmin_ac;
			  ACSINE_VMAX = vmax_ac;
			  BAT_VMIN = vmin_bat;
		  }
		// Agora as variáveis preset, vmin_ac, vmax_ac e vmin_bat contêm os valores
		// Reinicia a recepção de 3 bytes pela UART em modo de interrupção
		}
		HAL_UART_Receive_IT(&huart2, rxBuffer, 19);
	}

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rxBuffer[0] == '#' && rxBuffer[1] == 'B' && rxBuffer[2] == '.') {
            // Converter os valores do vetor para uma string com índices
            int length = 0;
            for (int i = 0; i < LEN_BUFFER; i++) {
                length += sprintf(&uartBuffer[length], "%d\r\n", adcBuffer[i]);
            }

            // Transmitir a string via UART
            HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, length, HAL_MAX_DELAY);
        }
        length = 0;
        // Reinicia a recepção de 3 bytes pela UART em modo de interrupção
        HAL_UART_Receive_IT(&huart2, rxBuffer, 3);
    }
}
*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_OPAMP2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_ADC_Start_DMA (&hadc1, (uint32_t*)adcBuffer, LEN_BUFFER); // Inicia o DMA.
  HAL_UART_Receive_IT(&huart2, rxBuffer, 19);
  GPIOB->MODER |= (0b11 << 28);
  GPIOA->MODER |= (0b11 << 14);

  GPIOA->ODR |= (1 << 10); // Configura os pinos como 1
  GPIOB->ODR |= (1 << 3);  //pois estao em dreno aberto
  GPIOB->ODR |= (1 << 5);
  GPIOA->OSPEEDR |= (0b11 << 20); // PA10 em 50MHz
  GPIOA->MODER |= (1 << 20); // PA10 como saida
  GPIOA->OTYPER |= (1 << 10); // PA10 Dreno Aberto

  GPIOB->OSPEEDR |= (0b11 << 6); // PB3 em 50MHz
  GPIOB->MODER &= ~(1 << 7);
  GPIOB->MODER |= (1 << 6); // PB3 como saida
  GPIOB->OTYPER |= (1 << 3); // PB3 dreno aberto

  GPIOB->OSPEEDR |= (0b11 << 10); //PB5 em 50MHz
  GPIOB->MODER |= (1 << 10); // PB5 como saida
  GPIOB->OTYPER |= (1 << 5); // PB5 Dreno Aberto

  GPIOB->OSPEEDR |= (0b11 << 26); // Configura a saida do LED
  GPIOB->MODER |= (1 << 26); // Coloca saida como Push-Pull

  GPIOC->MODER &= ~(0b11 << 26);
  GPIOC->PUPDR &= ~(0b11 << 26);
  GPIOC->PUPDR |= (1 << 26);
  GPIOC->ODR |= (1 << 13);

  HAL_TIM_Base_Start(&htim2);
  NVIC_EnableIRQ(TIM2_IRQn);
  TIM2->DIER |= (1 << 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Pagina 438 Datasheet
	   * Entradas do VIP do OPAMP2:
	   * PB0 (VP2 -> 10) - Entrada AC
	   * PA7 (VP0 -> 11) - Entrada Bateria
	   * PB14 (VP3-> 01) - Entrada Painel
	   * Configuração do OPAMP2 feita pelo: OPAMP2_CSR
	   */
	  switch(State_CHECK){
	  case ACSINE_CHK: // PB0 (VP2 -> 10) - Entrada AC
		  if(CONV_START){
			  CONV_START = 0;
			  OPAMP2->CSR |= (1 << 3);
			  OPAMP2->CSR &= ~(1 << 2);
			  DMA_FLAG=0;
			  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, LEN_BUFFER); // Inicia uma nova amostragem
		  }
		  if(DMA_FLAG==2){ // O FLAG de controle deve ser igual a 2 porque a ISR é chamada na metade e no fim do buffer.
			  DMA_FLAG = 0;
			  //VMAX=0;
			  int VMAX_temp = 0;
			  for(int x = 0; x < LEN_BUFFER; x++){
				  if(VMAX_temp < adcBuffer[x]) VMAX_temp = adcBuffer[x];
			  }
			  if(VMAX_temp > ACSINE_VMIN && VMAX_temp < ACSINE_VMAX){
				  REG_STATUS |= (1 << 0);
				  GPIOB->ODR |= (1 << 13); // Acende o LED se o status for 1
			  }
			  else {
				  REG_STATUS &= ~(1 << 0);
				  GPIOB->ODR &= ~(1 << 13); // Apaga o LED se o status for 0
				  SW_Time = 5000;
			  }
			  VMAX = VMAX_temp;
			  State_CHECK=BAT_CHK;
			  HAL_ADC_Stop_DMA(&hadc1);
			  //ADC1->CR2 &= ~ADC1_CR2_DMA;
			  CONV_START=1; //Configura o flag para iniciar a conversão do prox estado.
		  }
		  break;
	  case BAT_CHK: // PA7 (VP0 -> 11) - Entrada Bateria
		  if(CONV_START){
		  	  CONV_START = 0;
		  	  OPAMP2->CSR |= (1 << 2);
		  	  OPAMP2->CSR |= (1 << 3);
		  	  HAL_ADC_Start(&hadc1);
		  	  HAL_ADC_PollForConversion(&hadc1, 100); // poll for conversion
		  	  BATBuffer = HAL_ADC_GetValue(&hadc1); // get the adc value
		  	  if(BATBuffer > BAT_VMIN) REG_STATUS |= (1 << 1);
		  	  else REG_STATUS &= ~(1 << 1);
		  	  State_CHECK = PANEL_CHK;
		  	  CONV_START = 1;
		  }
		  break;
	  case PANEL_CHK: // PB14 (VP3-> 01) - Entrada Painel
		  if(CONV_START){
		      CONV_START = 0;
		      OPAMP2->CSR &= ~(1 << 3);
		      OPAMP2->CSR |= (1 << 2);
		      HAL_ADC_PollForConversion(&hadc1, 100); // poll for conversion
		      PANELBuffer = HAL_ADC_GetValue(&hadc1); // get the adc value
		      if(PANELBuffer > PANEL_VMIN) REG_STATUS |= (1 << 2);
		      else REG_STATUS &= ~(1 << 2);
		      HAL_ADC_Stop(&hadc1);
		      State_CHECK = ACSINE_CHK;
		      CONV_START = 1;
		  }
		  break;
	  }

	  switch(State_POWER){
	  case AC:
		  // Pino PA10
		  GPIOB->ODR |= (1 << 3);
		  GPIOB->ODR |= (1 << 5);
		  GPIOA->ODR &= ~(1 << 10);
		  break;
	  case BAT:
		  // Pino PB3
		  GPIOB->ODR |= (1 << 5);
		  GPIOA->ODR |= (1 << 10);
		  GPIOB->ODR &= ~(1 << 3);
		  break;
	  case PANEL:
		  // Pino PB5
		  GPIOB->ODR |= (1 << 3);
		  GPIOA->ODR |= (1 << 10);
		  GPIOB->ODR &= ~(1 << 5);
		  break;
	  case NC:
		  GPIOA->ODR |= (1 << 10);
		  GPIOB->ODR |= (1 << 3);
		  GPIOB->ODR |= (1 << 5);
		  break;
	  }
	  if(POWER_Control){
	  switch (State_PRESET) {
	      case PRESET_1:  // AC > BAT > PANEL > NC
	          if (AC_AVAILABLE && SW_Time == 0) {
	              State_POWER = AC;
	          } else if (BAT_AVAILABLE) {
	              State_POWER = BAT;
	          } else if (PANEL_AVAILABLE) {
	              State_POWER = PANEL;
	          } else {
	              State_POWER = NC;  // Nenhuma fonte disponível
	          }
	          break;

	      case PRESET_2:  // AC > PANEL > BAT
	          if (AC_AVAILABLE && SW_Time == 0) {
	              State_POWER = AC;
	          } else if (PANEL_AVAILABLE) {
	              State_POWER = PANEL;
	          } else if (BAT_AVAILABLE) {
	              State_POWER = BAT;
	          } else {
	              State_POWER = NC;  // Nenhuma fonte disponível
	          }
	          break;

	      case PRESET_3:  // BAT > AC > PANEL
	          if (BAT_AVAILABLE) {
	              State_POWER = BAT;
	          } else if (AC_AVAILABLE && SW_Time == 0) {
	              State_POWER = AC;
	          } else if (PANEL_AVAILABLE) {
	              State_POWER = PANEL;
	          } else {
	              State_POWER = NC;  // Nenhuma fonte disponível
	          }
	          break;

	      case PRESET_4:  // BAT > PANEL > AC
	          if (BAT_AVAILABLE) {
	              State_POWER = BAT;
	          } else if (PANEL_AVAILABLE) {
	              State_POWER = PANEL;
	          } else if (AC_AVAILABLE && SW_Time == 0) {
	              State_POWER = AC;
	          } else {
	              State_POWER = NC;  // Nenhuma fonte disponível
	          }
	          break;

	      case PRESET_5:  // PANEL > AC > BAT
	          if (PANEL_AVAILABLE) {
	              State_POWER = PANEL;
	          } else if (AC_AVAILABLE && SW_Time == 0) {
	              State_POWER = AC;
	          } else if (BAT_AVAILABLE) {
	              State_POWER = BAT;
	          } else {
	              State_POWER = NC;  // Nenhuma fonte disponível
	          }
	          break;

	      case PRESET_6:  // PANEL > BAT > AC
	          if (PANEL_AVAILABLE) {
	              State_POWER = PANEL;
	          } else if (BAT_AVAILABLE) {
	              State_POWER = BAT;
	          } else if (AC_AVAILABLE && SW_Time == 0) {
	              State_POWER = AC;
	          } else {
	              State_POWER = NC;  // Nenhuma fonte disponível
	          }
	          break;
	  }
	  }else{
		  State_POWER=NC;
	  }

	  switch(State_BUTTON){
	  case IDLE:
		  if(READ_BUTTON==0){
			  BUTTON_Time = 40;
			  State_BUTTON = DBC;
		  }
		  break;
	  case DBC:
		  if(BUTTON_Time == 0){
			  if(READ_BUTTON == 0){
				  State_BUTTON = PRES;
				  POWER_Control = !POWER_Control;
			  }else{
				  State_BUTTON = IDLE;
			  }
		  }
		  break;
	  case PRES:
		  if(READ_BUTTON){
			  State_BUTTON = IDLE;
		  }
		  break;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
