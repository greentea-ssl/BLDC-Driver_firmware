/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "pwm.h"
#include "parameters.h"
#include "ACR.h"
#include "ASR.h"
#include "APR.h"
#include "encoder.h"
#include "CurrentSensor.h"
#include "drv8323.h"
#include "canCom.h"
#include "sin_t.h"

#include "debugDump.h"

#include "../waveSamplerLib/waveSampler.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define  PRINT_HEX(x)  printf(#x " = %04x\n", (x))





#define _APR_ENABLE_		1



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */



/********** ACR DUMP DEBUG **********/

#if _ACR_DUMP_


float Id_dump[ACR_DUMP_STEPS] = {0.0f};
float Iq_dump[ACR_DUMP_STEPS] = {0.0f};
float Id_ref_dump[ACR_DUMP_STEPS] = {0.0f};
float Iq_ref_dump[ACR_DUMP_STEPS] = {0.0f};
float Vd_ref_dump[ACR_DUMP_STEPS] = {0.0f};
float Vq_ref_dump[ACR_DUMP_STEPS] = {0.0f};


int ACR_dump_count = 0;

#endif


/********** ASR DUMP DEBUG **********/

#if _ASR_DUMP_

#define ASR_DUMP_STEPS		2000


float omega_dump[ASR_DUMP_STEPS] = {0.0f};
float omega_ref_dump[ASR_DUMP_STEPS] = {0.0f};
float torque_ref_dump[ASR_DUMP_STEPS] = {0.0f};

int ASR_dump_count = 0;

#endif



/********** WaveSampler **********/

WaveSampler_TypeDef hWave;


/********** Timeout Control **********/


volatile uint8_t timeoutEnable = 1;
volatile uint32_t timeoutCount = 0;

// 1: timeout
volatile uint8_t timeoutState = 0;



/********** LED Control **********/

volatile uint32_t LED_blink_count = 0;
volatile uint32_t LED_blink_state = 0;
volatile uint32_t LED_blink_t_us = 0;
volatile uint32_t LED_blink_times = 0;
volatile uint32_t LED_blink_Ton_us = 50000;
volatile uint32_t LED_blink_Toff_us = 200000;
volatile uint32_t LED_blink_T_wait_us = 1000000;
volatile uint32_t LED_blink_Ts_us = 100;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void LED_blink();

inline static int32_t UartPrintf(UART_HandleTypeDef *huart, char *format, ...);

int32_t printFloat(float val);


#if 1

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch)
{
	//HAL_UART_Transmit(&huart2, &ch, 1, 1);
}

#endif


#if 0
int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for(DataIdx=0; DataIdx<len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}
#endif

#if 1
extern void initialise_monitor_handles(void);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


	int count = 0;

	uint8_t p_ch, ch;

	/********** for ASR ***********/



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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  //initialise_monitor_handles();


  DRV_Init();



  //UartPrintf(&huart2, "Hello world\n");


  WaveSampler_Init(&hWave, &huart2);

/*
  hWave.variableAddr[0] = &mainCS.Iu;
  hWave.variableAddr[1] = &mainCS.Iv;
  hWave.variableAddr[2] = &mainCS.Iw;
  hWave.variableAddr[3] = &mainEncoder.theta_re;
  */
  /*
  hWave.variableAddr[0] = &mainASR.omega_ref;
  hWave.variableAddr[1] = &mainASR.omega;
  hWave.variableAddr[2] = &mainACR.Iq_ref;
  hWave.variableAddr[3] = &mainACR.Iq;
	*/

/*
  hWave.variableAddr[0] = &mainEncoder.theta;
  hWave.variableAddr[1] = &mainEncoder.theta_re;
  hWave.variableAddr[2] = &mainCS.Vdc;
  hWave.variableAddr[3] = &mainASR.omega_ref;
  hWave.variableAddr[4] = &mainCS.Iu;
  hWave.variableAddr[5] = &mainCS.Iv;
  hWave.variableAddr[6] = &mainCS.Iw;
  hWave.variableAddr[7] = &amp_u;
  */



  hWave.variableAddr[0] = &mainEncoder.theta;
  hWave.variableAddr[1] = &mainEncoder.omega;
  hWave.variableAddr[2] = &mainACR.Id_ref;
  hWave.variableAddr[3] = &mainACR.Id;
  hWave.variableAddr[4] = &mainACR.Iq_ref;
  hWave.variableAddr[5] = &mainACR.Iq;
  hWave.variableAddr[6] = &amp_u;
  hWave.variableAddr[7] = &amp_v;



#if DEBUG_PRINT_ENABLE

  printf("Hello\n");

#endif


  // Gate Enable
  HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_SET);


  //printf("Hello SPI Gate Driver\n");


  DRV_ReadData(&drv8323, ADDR_OCP_Control);

  drv8323.Reg.OCP_Control.DEAD_TIME = 0b01; // Dead Time : 100ns
  drv8323.Reg.OCP_Control.OCP_MODE = 0b00; // Overcurrentcausesa latchedfault
  drv8323.Reg.OCP_Control.OCP_DEG = 0b11; // Deglitch Time of 8us
  drv8323.Reg.OCP_Control.VDS_LVL = 0b1001; // VDS = 0.75V -> ID = 75A
  //drv8323.Reg.OCP_Control.VDS_LVL = 0b1111; // VDS = 0.75V -> ID = 75A

  DRV_WriteData(&drv8323, ADDR_OCP_Control);


  DRV_ReadData(&drv8323, ADDR_CSA_Control);

  drv8323.Reg.CSA_Control.SEN_LVL = 0b11;	// Vsense = 0.5V -> 50A
  drv8323.Reg.CSA_Control.CSA_GAIN = 0b01;	// Amplifier Gain = 10V/V

  DRV_WriteData(&drv8323, ADDR_CSA_Control);


#if DEBUG_PRINT_ENABLE

  PRINT_HEX(drv8323.Reg.FaultStatus1.word);
  PRINT_HEX(drv8323.Reg.FaultStatus2.word);
  PRINT_HEX(drv8323.Reg.DriverControl.word);
  PRINT_HEX(drv8323.Reg.GateDrive_HS.word);
  PRINT_HEX(drv8323.Reg.GateDrive_LS.word);
  PRINT_HEX(drv8323.Reg.OCP_Control.word);
  PRINT_HEX(drv8323.Reg.CSA_Control.word);

#endif

  // Current Sensing Auto Offset Calibration
  HAL_GPIO_WritePin(OP_CAL_GPIO_Port, OP_CAL_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(OP_CAL_GPIO_Port, OP_CAL_Pin, GPIO_PIN_RESET);


  /******** DEBUG ********/


  DRV_ReadData(&drv8323, ADDR_CSA_Control);


#if DEBUG_PRINT_ENABLE
  PRINT_HEX(drv8323.Reg.CSA_Control.word);
#endif

  HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(DB2_GPIO_Port, DB2_Pin, GPIO_PIN_RESET);


  p_ch = getChannel();


  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);		HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	HAL_Delay(100);


  ch = getChannel();

  LED_blink_times = ch;

  CAN_Init();


  Encoder_Init();


  HAL_Delay(100);

  CurrentSensor_Init();


  ACR_Init();

  ASR_Init();

  APR_Init();

  PWM_Init();

  CurrentSensor_Start(&mainCS);


  // Offset calibration
#if 1
  float sum_uvw[3] = {0, 0, 0};
  for(count = 0; count < 1000; count++)
  {
	  HAL_Delay(1);
	  sum_uvw[0] += mainCS.V_Iu;
	  sum_uvw[1] += mainCS.V_Iv;
	  sum_uvw[2] += mainCS.V_Iw;
  }
  mainCS.Init.V_Iu_offset += sum_uvw[0] / 1000.0;
  mainCS.Init.V_Iv_offset += sum_uvw[1] / 1000.0;
  mainCS.Init.V_Iw_offset += sum_uvw[2] / 1000.0;
#endif


  HAL_Delay(1);

  ACR_Start(&mainACR);

  timeoutEnable = 0;

  setZeroEncoder((p_ch != ch)? 1: 0);

  ASR_Start(&mainASR);

  timeoutEnable = 1;

  //APR_Start(&mainAPR);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //APR_Refresh(&mainAPR);

	  //ASR_Refresh(&mainASR);


#if DUMP_ENABLE

	  if(DumpCount >= DUMP_STEPS)
	  {
		  break;
	  }

#endif


  }


  mainACR.Id_ref = 0.0f;
  mainACR.Iq_ref = 0.0f;

  HAL_Delay(10);

  // Gate Disable
  HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_RESET);

  HAL_Delay(10);

  stopPWM(&htim8);

  HAL_Delay(10);


  Dump_Print();

  /*
	printf("t, Id, Iq, Id_ref, Iq_ref, Vd_ref, Vq_ref, theta_re, omega\n");

	for(count = 0; count < DUMP_STEPS; count++)
	{

		printf("%.4e, ", count * DUMP_CYCLETIME);

		printf("%.4f, ", record[count].Id);
		printf("%.4f, ", record[count].Iq);
		printf("%.4f, ", record[count].Id_ref);
		printf("%.4f, ", record[count].Iq_ref);
		printf("%.4f, ", record[count].Vd_ref);
		printf("%.4f, ", record[count].Vq_ref);
		printf("%.4f, ", record[count].theta_re);
		printf("%.4f, ", record[count].omega);

		printf("\n");

	}
*/



  while(1);

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 320;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_FALLING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
  sConfigInjected.InjectedRank = 4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = 8000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 7800;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DB1_Pin|OP_CAL_Pin|GATE_EN_Pin 
                          |SPI3_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_NSS_Pin|DB2_Pin|DB0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DB1_Pin OP_CAL_Pin GATE_EN_Pin 
                           SPI3_NSS_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DB1_Pin|OP_CAL_Pin|GATE_EN_Pin 
                          |SPI3_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_NSS_Pin DB2_Pin DB0_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin|DB2_Pin|DB0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CH_b0_Pin CH_b1_Pin CH_b2_Pin */
  GPIO_InitStruct.Pin = CH_b0_Pin|CH_b1_Pin|CH_b2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BR_FLT_Pin */
  GPIO_InitStruct.Pin = BR_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BR_FLT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

#if 1


void HAL_ADCEx_InjectedConvCpltCallback (ADC_HandleTypeDef * hadc)
//void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc)
{

	static float Vgam_ref;
	static float Vdel_ref;

	static float phase = 0.0;

	static float cos_phase;
	static float sin_phase;


	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);



#if 1

	Encoder_Refresh(&mainEncoder);

	CurrentSensor_Refresh(&mainCS, sector_SVM);


	/*
	if(mainASR.enable == 1)
	{

		Vgam_ref = mainASR.omega_ref / 300.0 * 20.0;
		Vdel_ref = 0.0;

		cos_phase = sin_table2[(int)((phase * 0.3183f + 0.5f) * 5000.0f)];
		sin_phase = sin_table2[(int)(phase * 1591.54943f)];

		phase += mainACR.Init.cycleTime * mainASR.omega_ref * POLE_PAIRS;
		if(phase < 0.0) phase += 2 * M_PI;
		else if(phase >= 2 * M_PI) phase -= 2 * M_PI;

		setSVM_dq(&htim8, Vgam_ref, Vdel_ref, cos_phase, sin_phase);

	}
	*/


#if 1
	mainASR.launchFlg = 1;
	ASR_Refresh(&mainASR);

	ACR_Refresh(&mainACR);

	//ASR_prescaler(&mainASR);

	//APR_prescaler(&mainAPR);




#endif


	Encoder_Request(&mainEncoder);

	WaveSampler_Sampling(&hWave);

#endif


#if 1

	if(timeoutEnable == 1)
	{
		// timeout control
		if(timeoutCount < TIMEOUT_MS * TIMEOUT_BASE_FREQ / 1000)
		{
			timeoutCount += 1;
		}
		else
		{
			//stopPWM(&htim8);
			// Gate Disable
			HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_RESET);
			timeoutCount = 0;
			timeoutState = 1;
		}
	}

#if DUMP_ENABLE

	Dump_Refresh();

#endif

#endif

	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);



	LED_blink();

}

#endif


void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{



	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);



	if(htim->Instance == TIM8 && __HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
	{




	}


	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);



}



inline void timeoutReset()
{
	timeoutCount = 0;
	if(timeoutState == 1)
	{
		timeoutState = 0;
		ASR_Reset(&mainASR);
		ACR_Reset(&mainACR);
		//startPWM(&htim8);

		// Gate Disable
		HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_SET);
	}
}

/*
volatile uint32_t LED_blink_count = 0;
volatile uint32_t LED_blink_state = 0;
volatile uint32_t LED_blink_t_us = 0;
volatile uint32_t LED_blink_times = 1;
volatile uint32_t LED_blink_Ton_us = 100000;
volatile uint32_t LED_blink_Toff_us = 100000;
volatile uint32_t LED_blink_T_wait_us = 1000000;
volatile uint32_t LED_blink_Ts_us = 100;
 */
inline void LED_blink()
{

	switch(LED_blink_state)
	{
	case 0: // OFF WAIT
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		if(LED_blink_t_us >= LED_blink_T_wait_us)
		{
			if(LED_blink_times > 0)
			{
				LED_blink_state = 1;
				LED_blink_count = 0;
			}

			LED_blink_t_us = 0;
		}
		break;

	case 1: // ON
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		if(LED_blink_t_us >= LED_blink_Ton_us)
		{
			LED_blink_count += 1;
			LED_blink_state = 2;
			LED_blink_t_us = 0;
		}
		break;

	case 2: // OFF
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		if(LED_blink_t_us >= LED_blink_Toff_us)
		{
			if(LED_blink_count < LED_blink_times)
				LED_blink_state = 1;
			else
				LED_blink_state = 0;

			LED_blink_t_us = 0;
		}
		break;

	default:

		break;
	}

	LED_blink_t_us += LED_blink_Ts_us;

}


void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart)
{

	WaveSampler_TxCplt(&hWave, huart);

}


void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart)
{

	WaveSampler_RxCplt(&hWave, huart);

}



inline static int32_t UartPrintf(UART_HandleTypeDef *huart, char *format, ...){
	int32_t TransStrLength;
	char TransStr[1024];

	va_list args;
	va_start(args, format);
	TransStrLength = vsprintf(TransStr, format, args);
	va_end(args);

	HAL_UART_Transmit(huart, (uint8_t*)TransStr, TransStrLength, 100);

	return TransStrLength;
}


int32_t printFloat(float val)
{
	int i;
	int charCount = 0;
	int first, temp;
	const int decNum = 4;
	int i_val;

	if(val < 0.0f)
	{
		i_val = val * pow(10.0, decNum) - 0.5;
		putchar('-');
		i_val *= -1;
		charCount += 1;
	}
	else
	{
		i_val = val * pow(10.0, decNum) + 0.5;
	}

	first = 0;
	for(i = 9; i >= 0; i--)
	{
		temp = (int)(i_val * pow(10.0, -1.0 * i)) % 10;

		if(i == decNum - 1)
		{
			if(first == 0)
			{
				first = 1;
				putchar('0');
			}
			putchar('.');
		}

		if(first == 1 || temp != 0)
		{
			putchar('0' + temp);
			charCount += 1;
			first = 1;
		}

	}

	return charCount;

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
