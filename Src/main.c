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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "sin_t.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


// ACR variables dump enable

#define _ACR_DUMP_			0

#define _ASR_DUMP_			0



#define _ACR_ENABLE_		1

#define _ASR_ENABLE_		1

#define _APR_ENABLE_		0



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/********** ACR DUMP DEBUG **********/

#if _ACR_DUMP_

#define ACR_DUMP_STEPS		400


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




/********** for Magnetic Rotary Encoder **********/


// Magnet Poles
#define POLES	22

// RealRotorAngle = MeasuredAngle + this
float theta_offset = 0.0f;

// RealElecAngle = MeasuredElecAngle + this
float theta_re_offset = -3.0723f;

// Encoder Resolution
#define ENCODER_RESOL 16384


// Sensing Data
volatile uint8_t spi2txBuf[2] = {0};
volatile uint8_t spi2rxBuf[2] = {0};

uint32_t angle_raw = 0;

// Rotor rotation angle
volatile float theta = 0.0f;

// Rotor electrical position
volatile float theta_re = 0.0f;


volatile float cos_theta_re = 1.0;
volatile float sin_theta_re = 0.0;


volatile float p_theta = 0.0f;

volatile float omega = 0.0f;



/********** Forced commutation **********/

volatile uint8_t forced_commute_state = 0;

volatile float forced_theta = 0.0f;

volatile float _forced_theta_re = 0.0f;
volatile float forced_theta_re = 0.0f;

#define FORCED_COMMUTE_STEPS	2000

volatile uint32_t forced_commute_count = 0;

const float forced_I_gamma_ref = 5.0f;
const float forced_I_delta_ref = 0.0f;


volatile float sensed_theta_error;
volatile float sensed_theta_error_sum = 0.0f;


volatile float sensedTheta_f[FORCED_COMMUTE_STEPS] = {0.0f};
volatile float sensedTheta_b[FORCED_COMMUTE_STEPS] = {0.0f};



#define _FC_DUMP_	0



/********** for ADC **********/


const float Vref_AD = 3.3f;

const int32_t AD_Range = 4096;


volatile uint32_t AD_Iu = 0;
volatile uint32_t AD_Iv = 0;
volatile uint32_t AD_Iw = 0;

int32_t ADI_Offset = 2048;

volatile float V_Iu = 0.0f;
volatile float V_Iv = 0.0f;
volatile float V_Iw = 0.0f;


float V_Iu_offset = -0.0445f;
float V_Iv_offset = -0.0275f;
float V_Iw_offset = -0.0325f;


const float Gain_currentSense = -10.0f; // 1 / ( R * OPAmpGain) [A / V]


// Current Moving Average Filter
#define N_MAF_I		2



volatile float Vdc = 20.0f;


/********** for PWM Output **********/


#define PWM_RESOL	8000.0f

volatile float Vd_ref = 0.0f;
volatile float Vq_ref = 0.0f;


volatile int sector_SVM = 0;


volatile float amp_u = 0.0;
volatile float amp_v = 0.0;
volatile float amp_w = 0.0;



// reference vectors for SVM
const float refVector[6+1][2] = {
	{ 1.000,  0.000},
	{ 0.500,  0.866},
	{-0.500,  0.866},
	{-1.000,  0.000},
	{-0.500, -0.866},
	{ 0.500, -0.866},
	{ 1.000,  0.000},
};






/********** for ACR (Auto Current Regulator) **********/


float Kp_ACR = 0.3;
float Ki_ACR = 300.0;

const float ACR_cycleTime = 100E-6;


volatile float Id = 0.0;
volatile float Iq = 0.0;


volatile float Iu = 0.0;
volatile float Iv = 0.0;
volatile float Iw = 0.0;


float Id_limit = 30.0f;
float Iq_limit = 30.0f;


volatile float Id_ref = 0.0f;
volatile float Iq_ref = 0.0f;


volatile float Id_error = 0.0f;
volatile float Iq_error = 0.0f;

volatile float Id_error_integ = 0.0f;
volatile float Iq_error_integ = 0.0f;




/********** for ASR (Auto Speed Regulator) **********/


float Kp_ASR = 0.3;
float Ki_ASR = 20.0;

int ASR_flg = 0;
int ASR_prescalerCount = 0;
const int ASR_prescale = 10;


const float ASR_cycleTime = 1E-3;

float omega_limit = 1000000.0;

volatile float omega_ref = 0.0f;


volatile float omega_error = 0.0f;

volatile float omega_error_integ = 0.0f;


#define 	KV	370.0

#define 	KE	(60.0f / (KV * 2 * M_PI))

#define 	KT	(POLES / 2 * KE)


float torque_ref = 0.0;


/********** APR **********/

float Kp_APR = 50.0f;
float Kd_APR = 0.02f;


const float APR_cycleTime = 1E-3;

volatile float theta_ref = 0.0f;


volatile float theta_error = 0.0f;

volatile float theta_error_diff = 0.0f;




/********** CAN **********/

CAN_FilterTypeDef sFilterConfig;


CAN_RxHeaderTypeDef can1RxHeader;
uint8_t can1RxData[8];
uint8_t can1RxFlg = 0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


inline static int32_t UartPrintf(UART_HandleTypeDef *huart, char *format, ...);

int32_t printFloat(float val);

inline static void setPWM(const float *duty);

inline static void setSVM(float ampl, float phase);

inline static void setSVM_dq();


inline static void currentControl(void);


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch)
{
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}


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


	/********** for ASR ***********/

	int ASR_steps = 0;

	float d_theta;

	float _omega_ref;

	float omega_error_integ_temp1 = 0.0f;
	float omega_error_integ_temp2 = 0.0f;

	float p_theta_error = 0.0f;


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
  MX_CAN1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */


  /********** CAN Setting **********/


  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  if(HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	  Error_Handler();
  }

  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
	  Error_Handler();
  }





  UartPrintf(&huart2, "Hello world\n");


  // Gate Enable
  HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_SET);


  // Current Sensing Auto Offset Calibration
  HAL_GPIO_WritePin(OP_CAL_GPIO_Port, OP_CAL_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(OP_CAL_GPIO_Port, OP_CAL_Pin, GPIO_PIN_RESET);


  /******** DEBUG ********/

  HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(DB2_GPIO_Port, DB2_Pin, GPIO_PIN_RESET);



  // ADC Starting
  HAL_ADC_Start_DMA(&hadc1, &AD_Iu, 1);
  HAL_ADC_Start_DMA(&hadc2, &AD_Iv, 1);
  HAL_ADC_Start_DMA(&hadc3, &AD_Iw, 1);



  // Setting Timer Interrupts
  /*
  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC1);
  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC2);
  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC3);
  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC4);
  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_COM);
  __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_BREAK);*/
  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);



  // 3phase PWM Starting
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_3);



  // SPI Interrupt Setting
  __HAL_SPI_ENABLE_IT(&hspi2, SPI_IT_TXE | SPI_IT_RXNE);







  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  if(ASR_flg == 1)
	  {
		  HAL_GPIO_TogglePin(DB2_GPIO_Port, DB2_Pin);


		  if(forced_commute_state > 0)
		  {

			  switch(forced_commute_state)
			  {
			  case 1:
				  if(forced_commute_count < 500)
					  forced_commute_count += 1;
				  else
				  {
					  forced_commute_count = 0;
					  forced_commute_state = 2;
				  }
				  break;

			  case 2:
				  if(forced_commute_count < FORCED_COMMUTE_STEPS)
				  {
#if _FC_DUMP_
					  sensedTheta_f[forced_commute_count] = theta;
#endif
					  sensed_theta_error = forced_theta - theta;
					  if(sensed_theta_error < -M_PI)		sensed_theta_error += 2.0f * M_PI;
					  else if(sensed_theta_error > M_PI)	sensed_theta_error -= 2.0f * M_PI;
					  sensed_theta_error_sum += sensed_theta_error;
					  forced_theta = forced_commute_count * 2.0f * M_PI / FORCED_COMMUTE_STEPS;
					  forced_commute_count += 1;
				  }
				  else
				  {
					  forced_commute_count = 0;
					  forced_commute_state = 3;
					  break;
				  }
				  break;

			  case 3:
				  if(forced_commute_count < 500)
					  forced_commute_count += 1;
				  else
				  {
					  forced_commute_count = 0;
					  forced_commute_state = 4;
				  }
				  break;

			  case 4:
				  if(forced_commute_count < FORCED_COMMUTE_STEPS)
				  {
#if _FC_DUMP_
					  sensedTheta_b[FORCED_COMMUTE_STEPS - forced_commute_count - 1] = theta;
#endif
					  sensed_theta_error = forced_theta - theta;
					  if(sensed_theta_error < -M_PI)		sensed_theta_error += 2.0f * M_PI;
					  else if(sensed_theta_error > M_PI)	sensed_theta_error -= 2.0f * M_PI;
					  sensed_theta_error_sum += sensed_theta_error;
					  forced_theta = (FORCED_COMMUTE_STEPS - forced_commute_count - 1) * 2.0f * M_PI / FORCED_COMMUTE_STEPS;
					  forced_commute_count += 1;
				  }
				  else
				  {
					  theta_re_offset = fmod(sensed_theta_error_sum * 0.5f / FORCED_COMMUTE_STEPS * POLES / 2, 2.0f * M_PI);
					  if(theta_re_offset < -M_PI)		theta_re_offset += 2.0f * M_PI;
					  else if(theta_re_offset > M_PI)	theta_re_offset -= 2.0f * M_PI;
					  forced_commute_count = 0;
					  forced_commute_state = 0;
					  break;
				  }
				  break;

			  default:

				  break;
			  }




		  }
#if _FC_DUMP_
		  else
		  {
			  break;
		  }

#endif



#if _ACR_DUMP_

		  if(Iq_ref <= 0.0f)
			  Iq_ref = 5.0f;
		  else
			  Iq_ref = -5.0f;

		  if(ACR_dump_count >= ACR_DUMP_STEPS)
			  break;

#endif


#if _ASR_DUMP_

		  if(ASR_dump_count % 1000 < 500)
		  {
			  omega_ref = 50.0f;
		  }
		  else
		  {
			  omega_ref = -50.0f;
		  }

		  if(ASR_dump_count >= ASR_DUMP_STEPS)
			  break;

#endif



		  if(ASR_steps <= 0)
		  {
			  d_theta = 0.0f;
		  }
		  else
		  {
			  d_theta = theta - p_theta;
		  }
		  ASR_steps += 1;

		  p_theta = theta;

		  if(d_theta < - M_PI)		d_theta += 2 * M_PI;
		  else if(d_theta > M_PI)	d_theta -= 2 * M_PI;

		  omega = omega * 0.5 + 0.5 * d_theta / ASR_cycleTime;


		  /********** APR (Auto Position Regulator) **********/

#if _APR_ENABLE_

		  theta_error = theta_ref - theta;

		  if(theta_error < - M_PI)		theta_error += 2 * M_PI;
		  else if(theta_error > M_PI)	theta_error -= 2 * M_PI;

		  theta_error_diff = (theta_error - p_theta_error) / APR_cycleTime;

		  p_theta_error = theta_error;

		  omega_ref = Kp_APR * theta_error + Kd_APR * theta_error_diff;

#endif

		  /********** ASR (Auto Speed Regulator) **********/

#if _ASR_ENABLE_


		  if(omega_ref < -omega_limit)		_omega_ref = -omega_limit;
		  else if(omega_ref > omega_limit)	_omega_ref = omega_limit;
		  else								_omega_ref = omega_ref;

		  omega_error = _omega_ref - omega;

		  // integral
		  omega_error_integ_temp1 = omega_error + omega_error_integ_temp2;
		  if(omega_error_integ_temp1 < -6.0 / ASR_cycleTime)
		  {
			  omega_error_integ_temp1 = -6.0 / ASR_cycleTime;
		  }
		  else if(omega_error_integ_temp1 > 6.0 / ASR_cycleTime)
		  {
			  omega_error_integ_temp1 = 6.0 / ASR_cycleTime;
		  }
		  omega_error_integ = ASR_cycleTime * 0.5f * (omega_error_integ_temp1 + omega_error_integ_temp2);
		  omega_error_integ_temp2 = omega_error_integ_temp1;


		  torque_ref = Kp_ASR * omega_error + Ki_ASR * omega_error_integ;

		  Id_ref = 0.0f;
		  Iq_ref = KT * torque_ref;

#endif


#if _ASR_DUMP_

		  if(ASR_dump_count < ASR_DUMP_STEPS)
		  {
			  omega_dump[ASR_dump_count] = omega;
			  omega_ref_dump[ASR_dump_count] = omega_ref;
			  torque_ref_dump[ASR_dump_count] = torque_ref;
			  ASR_dump_count += 1;
		  }

#endif



		  /********** end of ASR **********/


		  ASR_flg = 0;

	  }




  }


  Id_ref = 0.0f;
  Iq_ref = 0.0f;

  HAL_Delay(10);

  // Gate Disable
  HAL_GPIO_WritePin(GATE_EN_GPIO_Port, GATE_EN_Pin, GPIO_PIN_RESET);

  HAL_Delay(10);

  // 3phase PWM Stopping
  HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_3);

  HAL_Delay(10);

#if _FC_DUMP_

  printFloat(theta_re_offset);
  printf("\n");

  printf("forcedTheta[rad], measuredTheta[rad]\n");

  for(count = 0; count < FORCED_COMMUTE_STEPS; count++)
  {
	  printFloat(count * 2.0f * M_PI / FORCED_COMMUTE_STEPS);
	  printf(", ");
	  printFloat(sensedTheta_f[count]);
	  printf(", ");
	  printFloat(sensedTheta_b[count]);
	  printf("\n");
  }


#endif



#if _ACR_DUMP_

  printf("time[s], Id[A], Iq[A], Id*[A], Iq*[A], Vd*[V], Vq*[V]\n");

  for(count = 0; count < ACR_DUMP_STEPS; count++)
  {

	  printFloat(count * ACR_cycleTime);
	  printf(", ");

	  printFloat(Id_dump[count]);
	  printf(", ");
	  printFloat(Iq_dump[count]);
	  printf(", ");

	  printFloat(Id_ref_dump[count]);
	  printf(", ");
	  printFloat(Iq_ref_dump[count]);
	  printf(", ");

	  printFloat(Vd_ref_dump[count]);
	  printf(", ");
	  printFloat(Vq_ref_dump[count]);
	  printf("\n");

  }

#endif

#if _ASR_DUMP_

  printf("time[s], Ï‰[rad/s], ?????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?*[rad/s], Torque*[Nãƒ»m]\n");

  for(count = 0; count < ASR_DUMP_STEPS; count++)
  {
	  printFloat(count * ASR_cycleTime);
	  printf(", ");

	  printFloat(omega_dump[count]);
	  printf(", ");

	  printFloat(omega_ref_dump[count]);
	  printf(", ");

	  printFloat(torque_ref_dump[count]);
	  printf("\n");

  }

#endif




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

/* USER CODE BEGIN 4 */




void HAL_CAN_TxMailbox0CompleteCallback (CAN_HandleTypeDef * hcan)
{
	if(hcan->Instance == CAN1)
	{
	}
	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

}

void HAL_CAN_TxMailbox1CompleteCallback (CAN_HandleTypeDef * hcan)
{
	if(hcan->Instance == CAN1)
	{
	}
	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

}

void HAL_CAN_TxMailbox2CompleteCallback (CAN_HandleTypeDef * hcan)
{
	if(hcan->Instance == CAN1)
	{
	}
	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	union _rcdata{
		struct{
			float fval;
		};
		struct{
			uint8_t byte[4];
		};
	}controlRef;


	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1RxHeader, can1RxData);

	can1RxFlg = 1;

#if _ASR_ENABLE_ && !_APR_ENABLE_
	if(can1RxHeader.StdId == 0x004 && can1RxHeader.DLC == 0x4)
	{
		controlRef.byte[0] = can1RxData[0];
		controlRef.byte[1] = can1RxData[1];
		controlRef.byte[2] = can1RxData[2];
		controlRef.byte[3] = can1RxData[3];

		omega_ref = controlRef.fval;
	}
#endif

#if _APR_ENABLE_
	if(can1RxHeader.StdId == 0x008 && can1RxHeader.DLC == 0x4)
	{
		controlRef.byte[0] = can1RxData[0];
		controlRef.byte[1] = can1RxData[1];
		controlRef.byte[2] = can1RxData[2];
		controlRef.byte[3] = can1RxData[3];

		theta_ref = controlRef.fval;
	}
#endif


	HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_SET);

}





void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{

	if(htim->Instance == TIM8)
	{

		if(!__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
		{

			currentControl();

		}


	}

}


#if 0

// SPI RX CallBack function
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef *hspi)
{


}

#endif




inline static void currentControl(void)
{

	static float _theta;
	static float _theta_re;

	static float _Id_ref;
	static float _Iq_ref;



	// Moving Average Filter

	static int32_t pos_MAF_I = 0;

	static int32_t AD_Iu_buf[N_MAF_I] = {0};
	static int32_t AD_Iv_buf[N_MAF_I] = {0};
	static int32_t AD_Iw_buf[N_MAF_I] = {0};

	static int32_t AD_Iu_MAF = 0;
	static int32_t AD_Iv_MAF = 0;
	static int32_t AD_Iw_MAF = 0;


	static float Id_error_integ_temp1 = 0.0f;
	static float Id_error_integ_temp2 = 0.0f;
	static float Iq_error_integ_temp1 = 0.0f;
	static float Iq_error_integ_temp2 = 0.0f;


	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_SET);


	// Read ADC
	/*
	AD_Iu = HAL_ADC_GetValue(&hadc1);
	AD_Iv = HAL_ADC_GetValue(&hadc2);
	AD_Iw = HAL_ADC_GetValue(&hadc3);
	*/

	HAL_ADC_Start_DMA(&hadc1, &AD_Iu, 1);
	HAL_ADC_Start_DMA(&hadc2, &AD_Iv, 1);
	HAL_ADC_Start_DMA(&hadc3, &AD_Iw, 1);


	// Reading RX Data from SPI Encoder
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	angle_raw = (spi2rxBuf[1] & 0x3f) << 8 | spi2rxBuf[0];

	_theta = (float)angle_raw / (float)ENCODER_RESOL * 2.0f * M_PI + theta_offset;

	if(_theta < 0.0f)			theta = _theta + 2 * M_PI;
	else if(_theta >= 2 * M_PI)	theta = _theta - 2 * M_PI;
	else						theta = _theta;


	// calculate sin(theta_re), cos(theta_re)
	if(forced_commute_state > 0)
	{

		_forced_theta_re = fmodf(forced_theta * POLES / 2, 2.0f * M_PI);

		if(_forced_theta_re < 0.0f)				forced_theta_re = _forced_theta_re + 2 * M_PI;
		else if(_forced_theta_re >= 2 * M_PI)	forced_theta_re = _forced_theta_re - 2 * M_PI;
		else									forced_theta_re = _forced_theta_re;

		cos_theta_re = sin_table2[(int)((forced_theta_re * 0.3183f + 0.5f) * 5000.0f)];
		sin_theta_re = sin_table2[(int)(forced_theta_re * 1591.54943f)];
	}
	else
	{

		_theta_re = fmodf((float)angle_raw / (float)ENCODER_RESOL * 2.0f * M_PI * POLES / 2, 2.0f * M_PI) + theta_re_offset;

		if(_theta_re < 0.0f)			theta_re = _theta_re + 2 * M_PI;
		else if(_theta_re >= 2 * M_PI)	theta_re = _theta_re - 2 * M_PI;
		else							theta_re = _theta_re;

		cos_theta_re = sin_table2[(int)((theta_re * 0.3183f + 0.5f) * 5000.0f)];
		sin_theta_re = sin_table2[(int)(theta_re * 1591.54943f)];
	}



	AD_Iu_buf[pos_MAF_I] = (int32_t)AD_Iu - ADI_Offset;
	AD_Iv_buf[pos_MAF_I] = (int32_t)AD_Iv - ADI_Offset;
	AD_Iw_buf[pos_MAF_I] = (int32_t)AD_Iw - ADI_Offset;


	AD_Iu_MAF += AD_Iu_buf[pos_MAF_I];
	AD_Iv_MAF += AD_Iv_buf[pos_MAF_I];
	AD_Iw_MAF += AD_Iw_buf[pos_MAF_I];

	// Writing position Update
	pos_MAF_I += 1;
	if(pos_MAF_I >= N_MAF_I)
	{
		pos_MAF_I = 0;
	}

	V_Iu = (float)AD_Iu_MAF / (N_MAF_I * AD_Range) * Vref_AD + V_Iu_offset;
	V_Iv = (float)AD_Iv_MAF / (N_MAF_I * AD_Range) * Vref_AD + V_Iv_offset;
	V_Iw = (float)AD_Iw_MAF / (N_MAF_I * AD_Range) * Vref_AD + V_Iw_offset;

	AD_Iu_MAF -= AD_Iu_buf[pos_MAF_I];
	AD_Iv_MAF -= AD_Iv_buf[pos_MAF_I];
	AD_Iw_MAF -= AD_Iw_buf[pos_MAF_I];



	/*
	V_Iu = V_Iu * 0.9 + 0.1 * ((float)(ADI_Offset - (int32_t)AD_Iu) / AD_Range * Vref_AD - V_Iu_offset);
	V_Iv = V_Iv * 0.9 + 0.1 * ((float)(ADI_Offset - (int32_t)AD_Iv) / AD_Range * Vref_AD - V_Iv_offset);
	V_Iw = V_Iw * 0.9 + 0.1 * ((float)(ADI_Offset - (int32_t)AD_Iw) / AD_Range * Vref_AD - V_Iw_offset);
	*/


	switch(sector_SVM)
	{
	case 0: case 5:
		Iv = V_Iv * Gain_currentSense;
		Iw = V_Iw * Gain_currentSense;
		Iu = - Iv - Iw;
		break;

	case 1: case 2:
		Iw = V_Iw * Gain_currentSense;
		Iu = V_Iu * Gain_currentSense;
		Iv = - Iw - Iu;
		break;

	case 3: case 4:
		Iu = V_Iu * Gain_currentSense;
		Iv = V_Iv * Gain_currentSense;
		Iw = - Iu - Iv;
		break;
	}

	Id = 0.8165f * (Iu * cos_theta_re + Iv * (-0.5f * cos_theta_re + 0.855f * sin_theta_re) + Iw * (-0.5f * cos_theta_re - 0.855f * sin_theta_re));
	Iq = 0.8165f * (-Iu * sin_theta_re + Iv * (0.5f * sin_theta_re + 0.855f * cos_theta_re) + Iw * (0.5f * sin_theta_re - 0.855f * cos_theta_re));


	if(theta_re < M_PI)
		HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_SET);


	/********** ACR (Auto Current Regulator) **********/

#if _ACR_ENABLE_

	if(Id_ref < -Id_limit)		_Id_ref = -Id_limit;
	else if(Id_ref > Id_limit)	_Id_ref = Id_limit;
	else						_Id_ref = Id_ref;

	if(Iq_ref < -Iq_limit)		_Iq_ref = -Iq_limit;
	else if(Iq_ref > Iq_limit)	_Iq_ref = Iq_limit;
	else						_Iq_ref = Iq_ref;

	if(forced_commute_state > 0)
	{
		Id_error = forced_I_gamma_ref - Id;
		Iq_error = forced_I_delta_ref - Iq;
	}
	else
	{
		Id_error = _Id_ref - Id;
		Iq_error = _Iq_ref - Iq;
	}


	// integral
	Id_error_integ_temp1 = Id_error + Id_error_integ_temp2;
	if(Id_error_integ_temp1 < -1000000.0) Id_error_integ_temp1 = -1000000.0;
	else if(Id_error_integ_temp1 > 1000000.0) Id_error_integ_temp1 = 1000000.0;
	Id_error_integ = ACR_cycleTime * 0.5f * (Id_error_integ_temp1 + Id_error_integ_temp2);
	Id_error_integ_temp2 = Id_error_integ_temp1;

	Iq_error_integ_temp1 = Iq_error + Iq_error_integ_temp2;
	if(Iq_error_integ_temp1 < -1000000.0) Iq_error_integ_temp1 = -1000000.0;
	else if(Iq_error_integ_temp1 > 1000000.0) Iq_error_integ_temp1 = 1000000.0;
	Iq_error_integ = ACR_cycleTime * 0.5f * (Iq_error_integ_temp1 + Iq_error_integ_temp2);
	Iq_error_integ_temp2 = Iq_error_integ_temp1;


	Vd_ref = Kp_ACR * Id_error + Ki_ACR * Id_error_integ;
	Vq_ref = Kp_ACR * Iq_error + Ki_ACR * Iq_error_integ;

#endif

	/********* end of ACR **********/


	setSVM_dq();


#if _ACR_DUMP_

	if(ACR_dump_count < ACR_DUMP_STEPS)
	{
		Id_dump[ACR_dump_count] = Id;
		Iq_dump[ACR_dump_count] = Iq;
		Id_ref_dump[ACR_dump_count] = Id_ref;
		Iq_ref_dump[ACR_dump_count] = Iq_ref;
		Vd_ref_dump[ACR_dump_count] = Vd_ref;
		Vq_ref_dump[ACR_dump_count] = Vq_ref;
		ACR_dump_count += 1;
	}

#endif


	// Reading Encoder for next sampling
	spi2txBuf[0] = 0xff;
	spi2txBuf[1] = 0xff;
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(&hspi2, spi2txBuf, spi2rxBuf, 1);


	// Auto Speed Regulator launching
	ASR_prescalerCount += 1;
	if(ASR_prescalerCount >= ASR_prescale)
	{
		ASR_flg = 1;
		ASR_prescalerCount = 0;
	}



	HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);

	return;
}


inline static void setSVM_dq()
{

	static float cross0 = 0.0;
	static float cross1 = 0.0;
	static float duty[3] = {0};
	static float x1, y1, x2, y2;
	static float x, y;
	static float vect1, vect2;




	x = Vd_ref * cos_theta_re - Vq_ref * sin_theta_re;
	y = Vd_ref * sin_theta_re + Vq_ref * cos_theta_re;

	cross0 = refVector[0][0] * y - refVector[0][1] * x;
	cross1 = refVector[1][0] * y - refVector[1][1] * x;

	if(cross0 >= 0)
	{
		if(cross1 <= 0)				sector_SVM = 0;
		else if(cross0 >= cross1)	sector_SVM = 1;
		else						sector_SVM = 2;
	}
	else
	{
		if(cross1 >= 0)				sector_SVM = 3;
		else if(cross0 <= cross1)	sector_SVM = 4;
		else						sector_SVM = 5;
	}

	x1 = refVector[sector_SVM][0];
	y1 = refVector[sector_SVM][1];
	x2 = refVector[sector_SVM + 1][0];
	y2 = refVector[sector_SVM + 1][1];

	vect1 = (y2 * x - x2 * y) / ((x1 * y2 - y1 * x2) * Vdc);
	vect2 = (-y1 * x + x1 * y) / ((x1 * y2 - y1 * x2) * Vdc);

	switch(sector_SVM)
	{
	case 0: duty[2] = (1.0 - vect1 - vect2) * 0.5f; 	duty[1] = duty[2] + vect2; 	duty[0] = duty[1] + vect1;  break;
	case 1: duty[2] = (1.0 - vect1 - vect2) * 0.5f; 	duty[0] = duty[2] + vect1; 	duty[1] = duty[0] + vect2; 	break;
	case 2: duty[0] = (1.0 - vect1 - vect2) * 0.5f; 	duty[2] = duty[0] + vect2; 	duty[1] = duty[2] + vect1; 	break;
	case 3: duty[0] = (1.0 - vect1 - vect2) * 0.5f; 	duty[1] = duty[0] + vect1; 	duty[2] = duty[1] + vect2; 	break;
	case 4: duty[1] = (1.0 - vect1 - vect2) * 0.5f; 	duty[0] = duty[1] + vect2; 	duty[2] = duty[0] + vect1; 	break;
	case 5: duty[1] = (1.0 - vect1 - vect2) * 0.5f; 	duty[2] = duty[1] + vect1; 	duty[0] = duty[2] + vect2; 	break;
	}


	if(duty[0] < -1.0f) duty[0] = -1.0f; else if (duty[0] > 1.0f) duty[0] = 1.0f;
	if(duty[1] < -1.0f) duty[1] = -1.0f; else if (duty[1] > 1.0f) duty[1] = 1.0f;
	if(duty[2] < -1.0f) duty[2] = -1.0f; else if (duty[2] > 1.0f) duty[2] = 1.0f;

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_RESOL * (1.0f - (amp_u = duty[0])));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PWM_RESOL * (1.0f - (amp_v = duty[1])));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, PWM_RESOL * (1.0f - (amp_w = duty[2])));


	return;
}


inline static void setSVM(float ampl, float phase){
	//float rate[3] = {0};
	//float _duty[3] = {0};
	static float duty[3] = {0};
	static float x1, y1, x2, y2;
	static float x, y;
	static float vect1, vect2;

	// reference vectors for SVM
	const float refVector[6+1][2] = {
			{ 1.000,  0.000},
			{ 0.500,  0.866},
			{-0.500,  0.866},
			{-1.000,  0.000},
			{-0.500, -0.866},
			{ 0.500, -0.866},
			{ 1.000,  0.000},
	};

	// Select two from 6 vector
	int sector = (int)(phase * 0.95493f);



	x = sin_table2[(int)((phase * 0.3183f + 0.5f) * 5000.0f)];
	y = sin_table2[(int)(phase * 1591.54943f)];


	x1 = refVector[sector][0];
	y1 = refVector[sector][1];
	x2 = refVector[sector + 1][0];
	y2 = refVector[sector + 1][1];


	vect1 = ampl / (x1 * y2 - y1 * x2) * (y2 * x - x2 * y);
	vect2 = ampl / (x1 * y2 - y1 * x2) * (-y1 * x + x1 * y);

	switch(sector)
	{
	case 0: duty[2] = (1.0 - vect1 - vect2) * 0.5f; 	duty[1] = duty[2] + vect2; 	duty[0] = duty[1] + vect1;  break;
	case 1: duty[2] = (1.0 - vect1 - vect2) * 0.5f; 	duty[0] = duty[2] + vect1; 	duty[1] = duty[0] + vect2; 	break;
	case 2: duty[0] = (1.0 - vect1 - vect2) * 0.5f; 	duty[2] = duty[0] + vect2; 	duty[1] = duty[2] + vect1; 	break;
	case 3: duty[0] = (1.0 - vect1 - vect2) * 0.5f; 	duty[1] = duty[0] + vect1; 	duty[2] = duty[1] + vect2; 	break;
	case 4: duty[1] = (1.0 - vect1 - vect2) * 0.5f; 	duty[0] = duty[1] + vect2; 	duty[2] = duty[0] + vect1; 	break;
	case 5: duty[1] = (1.0 - vect1 - vect2) * 0.5f; 	duty[2] = duty[1] + vect1; 	duty[0] = duty[2] + vect2; 	break;
	}


	if(duty[0] < -1.0f) duty[0] = -1.0f; else if (duty[0] > 1.0f) duty[0] = 1.0f;
	if(duty[1] < -1.0f) duty[1] = -1.0f; else if (duty[1] > 1.0f) duty[1] = 1.0f;
	if(duty[2] < -1.0f) duty[2] = -1.0f; else if (duty[2] > 1.0f) duty[2] = 1.0f;

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 4200.0f * (1.0f - (amp_u = duty[0])));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 4200.0f * (1.0f - (amp_v = duty[1])));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 4200.0f * (1.0f - (amp_w = duty[2])));


/*
	x = sin_t(phase + M_PI * 0.5);
	y = sin_t(phase);

	float x1 = refVector[sector][0];
	float y1 = refVector[sector][1];
	float x2 = refVector[sector + 1][0];
	float y2 = refVector[sector + 1][1];

	float vect1 = ampl / (x1 * y2 - y1 * x2) * (y2 * x - x2 * y);
	float vect2 = ampl / (x1 * y2 - y1 * x2) * (-y1 * x + x1 * y);

	if(sector%2 != 0){
		rate[1] = vect2;
		rate[2] = vect1;
	}
	else{
		rate[1] = vect1;
		rate[2] = vect2;
	}

	rate[0] = 1.0 - rate[1] - rate[2];


	_duty[0]   = rate[0] * 0.5 + rate[1] + rate[2];
	_duty[1]   = rate[0] * 0.5 + rate[2];
	_duty[2]   = rate[0] / 2.0;

	switch(sector){
		case 0: duty[0] = _duty[0]; duty[1] = _duty[1]; duty[2] = _duty[2]; break;
		case 1: duty[0] = _duty[1]; duty[1] = _duty[0]; duty[2] = _duty[2]; break;
		case 2: duty[0] = _duty[2]; duty[1] = _duty[0]; duty[2] = _duty[1]; break;
		case 3: duty[0] = _duty[2]; duty[1] = _duty[1]; duty[2] = _duty[0]; break;
		case 4: duty[0] = _duty[1]; duty[1] = _duty[2]; duty[2] = _duty[0]; break;
		case 5: duty[0] = _duty[0]; duty[1] = _duty[2]; duty[2] = _duty[1]; break;
	}

	if(duty[0] < -1.0f) duty[0] = -1.0; else if (duty[0] > 1.0) duty[0] = 1.0;
	if(duty[1] < -1.0f) duty[1] = -1.0; else if (duty[1] > 1.0) duty[1] = 1.0;
	if(duty[2] < -1.0f) duty[2] = -1.0; else if (duty[2] > 1.0) duty[2] = 1.0;

	setPWM(duty);

*/

	return;

}

inline static void setPWM(const float *duty){

	if(duty[0] <= 1.0 && duty[0] >= 0.0)__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 4200 * (1.0 - (amp_u = duty[0])));
	if(duty[1] <= 1.0 && duty[1] >= 0.0)__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 4200 * (1.0 - (amp_v = duty[1])));
	if(duty[2] <= 1.0 && duty[2] >= 0.0)__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 4200 * (1.0 - (amp_w = duty[2])));

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
