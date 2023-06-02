/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FRONT_SENSOR_POS 65u
#define LEFT_MAX_SENSOR_POS 180u
#define RIGHT_MAX_SENSOR_POS 0u
#define GET_LEFT_STATUS 1u
#define GET_BOTH_STATUS 0u

typedef struct{
	GPIO_TypeDef *port;
	uint16_t pin;
} LED_pins;

typedef enum{
    INIT,
    IDLE,
    SAFE_DRIVE,
    OBSTACLE_DETECTED,
	TURN_EVENT,
    WALL_DETECTED,
    RESET_POSITION,

} board_process_t;

typedef enum{
	STOP,
	FORWARD,
	BACKWARD,
	TURN_RIGHT,
	TURN_LEFT

} drive_state_t;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t ulSonicRead_Start = 0;
uint32_t ulSonicRead_Stop = 0;
float fSonicRead_Value = 0.0;
uint8_t message[50] = {0};

uint32_t tick_start = 0;
uint32_t tick_start_2 = 0;

uint8_t frequency_coeff = 0;

board_process_t boardProcess = INIT;
drive_state_t	activeDriveProcess = 0;
drive_state_t	previousDriveProcess = 0;

uint8_t safe_forward = 0u;
uint8_t safe_right = 0u;
uint8_t safe_left =  0u;
uint8_t direction =  0u;

const LED_pins LED_array[6] = {
		{GREEN_3_GPIO_Port, GREEN_3_Pin},
		{GREEN_4_GPIO_Port, GREEN_4_Pin},
		{YELLOW_5_GPIO_Port, YELLOW_5_Pin},
		{YELLOW_6_GPIO_Port, YELLOW_6_Pin},
		{RED_7_GPIO_Port, RED_7_Pin},
		{RED_8_GPIO_Port, RED_8_Pin},
};

float valueArray[6] = {60.0, 50.0, 40.0, 30.0, 20.0, 10.0};


float way_table[] = {0, 0, 0, 0};
float max = 0;
uint8_t test_cnt = 0u;

int i = 0;
int n = 0;
int Index = 0;
uint32_t max_time = 1000;
int speedValue = 312;
uint8_t turn_event_handler = 0;

//TODO: add port #DONE!#, pin #DONE!# & scale values #DONE!# arrays

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}

void printDebugMessage(){

	memset(message, 0, sizeof message);
	sprintf((char *)message, "%.1f cm.\n", fSonicRead_Value);
	printf((char *)message);

}

void readSonicSensor(){
	ulSonicRead_Start = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
	ulSonicRead_Stop = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
	fSonicRead_Value = (float)((ulSonicRead_Stop - ulSonicRead_Start))/ 58.0;
}

uint8_t vSignalDistanceValue(){

	unsigned long int array_size = sizeof(LED_array) / sizeof(LED_array[0]);
	bool flag = false;
	uint8_t close_obstacle = 0u;

	for (int i = 0; i < array_size; i++){
		if (fSonicRead_Value < valueArray[i]){
			flag = true; // HELP flag for buzzer control. IF car is in parking mode (at least one led is on), then flag will have been activated

			HAL_GPIO_WritePin(LED_array[i].port, LED_array[i].pin, GPIO_PIN_SET);
			frequency_coeff = i+1;
			if(LED_array[i].port == YELLOW_5_GPIO_Port){
				close_obstacle=1u;
			}
		}
		else {
			HAL_GPIO_WritePin(LED_array[i].port, LED_array[i].pin, GPIO_PIN_RESET);
		}
	}

	if (flag){ // If flag is on (parking mode), then the sound can be heard in different frequency
		if (HAL_GetTick() - tick_start_2 > (int) 1000/frequency_coeff){
			HAL_GPIO_TogglePin(Buzzer_pin_GPIO_Port, Buzzer_pin_Pin);
			tick_start_2 = HAL_GetTick();
		}
	}else {
		HAL_GPIO_WritePin(Buzzer_pin_GPIO_Port, Buzzer_pin_Pin, GPIO_PIN_RESET);
	}
	return close_obstacle;

}


void vDriveForward(){
	HAL_GPIO_WritePin(RIGHT_WHEEL_PC4_GPIO_Port, RIGHT_WHEEL_PC4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RIGHT_WHEEL_PC5_GPIO_Port, RIGHT_WHEEL_PC5_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LEFT_WHEEL_PB0_GPIO_Port, LEFT_WHEEL_PB0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEFT_WHEEL_PB1_GPIO_Port, LEFT_WHEEL_PB1_Pin, GPIO_PIN_RESET);

}
void vDriveBackward(){
	HAL_GPIO_WritePin(RIGHT_WHEEL_PC4_GPIO_Port, RIGHT_WHEEL_PC4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RIGHT_WHEEL_PC5_GPIO_Port, RIGHT_WHEEL_PC5_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(LEFT_WHEEL_PB0_GPIO_Port, LEFT_WHEEL_PB0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEFT_WHEEL_PB1_GPIO_Port, LEFT_WHEEL_PB1_Pin, GPIO_PIN_SET);
}

void vStopAcceleration(){
	HAL_GPIO_WritePin(RIGHT_WHEEL_PC4_GPIO_Port, RIGHT_WHEEL_PC4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RIGHT_WHEEL_PC5_GPIO_Port, RIGHT_WHEEL_PC5_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LEFT_WHEEL_PB0_GPIO_Port, LEFT_WHEEL_PB0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEFT_WHEEL_PB1_GPIO_Port, LEFT_WHEEL_PB1_Pin, GPIO_PIN_RESET);
}

void vTurnLeft(){

	HAL_GPIO_WritePin(RIGHT_WHEEL_PC4_GPIO_Port, RIGHT_WHEEL_PC4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RIGHT_WHEEL_PC5_GPIO_Port, RIGHT_WHEEL_PC5_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LEFT_WHEEL_PB0_GPIO_Port, LEFT_WHEEL_PB0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEFT_WHEEL_PB1_GPIO_Port, LEFT_WHEEL_PB1_Pin, GPIO_PIN_RESET);

}

void vTurnRight(){

	HAL_GPIO_WritePin(RIGHT_WHEEL_PC4_GPIO_Port, RIGHT_WHEEL_PC4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RIGHT_WHEEL_PC5_GPIO_Port, RIGHT_WHEEL_PC5_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LEFT_WHEEL_PB0_GPIO_Port, LEFT_WHEEL_PB0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEFT_WHEEL_PB1_GPIO_Port, LEFT_WHEEL_PB1_Pin, GPIO_PIN_RESET);

}


void vDriveProc(){

	if(activeDriveProcess != previousDriveProcess){
		previousDriveProcess = activeDriveProcess;
		switch (activeDriveProcess) {
			case FORWARD:
				vDriveForward();
				break;
			case BACKWARD:
				vDriveBackward();
				break;
			case TURN_RIGHT:
				vTurnRight();
				break;
			case TURN_LEFT:
				vTurnLeft();
				break;
			case STOP:
				vStopAcceleration();
				break;
			default:
				vStopAcceleration();
				break;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == USER_BUTTON_Pin){
        boardProcess = SAFE_DRIVE;
    }

}


void vSetScannerPosition(const uint8_t SET_POS){
	servo_set_angle(SET_POS);
}

uint8_t usScanWithSensor(const uint8_t POSITION){
	vSetScannerPosition(POSITION);
	readSonicSensor();
	if(vSignalDistanceValue()){
		return 1u;

	} else {
		return 0u;

	}
}



uint8_t usScanForTurn(uint8_t turn_flag){


	switch (turn_flag) {
		case GET_BOTH_STATUS:
			if(!usScanWithSensor(RIGHT_MAX_SENSOR_POS)){
				return 3u;

			} else if(!usScanWithSensor(LEFT_MAX_SENSOR_POS)){
				return 4u;

			} else{
				return 0u;

			}
			break;

		case GET_LEFT_STATUS:
			if(!usScanWithSensor(LEFT_MAX_SENSOR_POS)){
				return 4u;

			} else{
				return 0u;

			}
			break;
		default:
			return 0u;
			break;
	}

}

uint8_t usScanForward(){

	if(usScanWithSensor(FRONT_SENSOR_POS)){
		return 1u;
	} else{
		return 0u;

	}
}

void vCar_Main(){
	switch (boardProcess) {
		case INIT:
			if(activeDriveProcess >= 5){
				activeDriveProcess = 0;
				vSetScannerPosition(FRONT_SENSOR_POS);
				boardProcess = IDLE;
			} else {
				activeDriveProcess++;
			}
			vDriveProc();
			break;

		case IDLE:
			activeDriveProcess = STOP;
			vDriveProc();
			break;

		case SAFE_DRIVE:
			test_cnt++;
			if(usScanForward())
			{
				boardProcess = OBSTACLE_DETECTED;
				activeDriveProcess = STOP;

			} else{
				activeDriveProcess = FORWARD;

			}
			vDriveProc();
			break;

		case OBSTACLE_DETECTED:
			activeDriveProcess = STOP;
			vDriveProc();
			activeDriveProcess = usScanForTurn(GET_BOTH_STATUS);
			vDriveProc();
			boardProcess = TURN_EVENT;
			break;
		case TURN_EVENT:
			if(turn_event_handler ==4){
				activeDriveProcess = STOP;
				vDriveProc();
				turn_event_handler++;

			} else if(turn_event_handler >= 5 && turn_event_handler < 8){
				activeDriveProcess = FORWARD;
				vDriveProc();
				turn_event_handler++;

			} else if(turn_event_handler == 8){
				activeDriveProcess = STOP;
				vDriveProc();
				activeDriveProcess = usScanForTurn(GET_LEFT_STATUS);
				if(activeDriveProcess != TURN_LEFT){
					activeDriveProcess = FORWARD;
				} else{
					turn_event_handler++;
				}
				vDriveProc();

			} else if(turn_event_handler == 12){

				activeDriveProcess = STOP;
				vDriveProc();
				activeDriveProcess = usScanForTurn(GET_LEFT_STATUS);
				if(activeDriveProcess != TURN_LEFT){
					activeDriveProcess = FORWARD;
				} else{
					turn_event_handler++;
				}
				vDriveProc();

			}else if(turn_event_handler == 16){

				activeDriveProcess = STOP;
				vDriveProc();
				activeDriveProcess = usScanForTurn(GET_BOTH_STATUS);
				if(activeDriveProcess != TURN_RIGHT){
					activeDriveProcess = FORWARD;
				} else{
					boardProcess = SAFE_DRIVE;
					turn_event_handler = 0;
				}
				vDriveProc();

			}
			else{
			turn_event_handler++;
			}
			break;
		default:
			break;
	}

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t callbackHandler = 0;
	static   int speedValue = 312;

	if (htim == &htim15) {


		if(callbackHandler % 20u == 0u){
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, speedValue);
			vCar_Main();
		}
		if(callbackHandler >= 40){
			callbackHandler = 0;
		}

	}
}





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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  servo_init(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim15);



  while (1)
  {

/*
	  if ( (HAL_GetTick() - time_tick) > max_time )
	  {
		  time_tick = HAL_GetTick();
		  servo_set_angle(angle_table[i]);
		  readSonicSensor();

		  way_table[i] = fSonicRead_Value;
		  i = i + 1;
		  if(i >= 4)
		  {
			  n = n + 1;
			  i = 0;

			  if(n >= 3)
			  {

				  for (i = 0; i < 5; i++)		// Finding max value from the scope of viewing
				  {
					  if (max < way_table[i])
					  {
						  max = way_table[i];
						  Index = i;
					  }
				  }

				  servo_set_angle(angle_table[Index]);		// Setting servo in position of the max value of the sensor
				  HAL_Delay(10000);
				  n = 0;
			  }


		  }

	  }
	  	  */


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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM15
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1151-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 625-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 3000-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 6000-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7200-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 2500-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GREEN_3_Pin|GREEN_4_Pin|YELLOW_5_Pin|RED_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RIGHT_WHEEL_PC4_Pin|RIGHT_WHEEL_PC5_Pin|RED_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEFT_WHEEL_PB0_Pin|LEFT_WHEEL_PB1_Pin|Buzzer_pin_Pin|YELLOW_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_3_Pin GREEN_4_Pin YELLOW_5_Pin RED_8_Pin */
  GPIO_InitStruct.Pin = GREEN_3_Pin|GREEN_4_Pin|YELLOW_5_Pin|RED_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT_WHEEL_PC4_Pin RIGHT_WHEEL_PC5_Pin RED_7_Pin */
  GPIO_InitStruct.Pin = RIGHT_WHEEL_PC4_Pin|RIGHT_WHEEL_PC5_Pin|RED_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_WHEEL_PB0_Pin LEFT_WHEEL_PB1_Pin Buzzer_pin_Pin YELLOW_6_Pin */
  GPIO_InitStruct.Pin = LEFT_WHEEL_PB0_Pin|LEFT_WHEEL_PB1_Pin|Buzzer_pin_Pin|YELLOW_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
