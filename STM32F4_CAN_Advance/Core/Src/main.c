/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t Empty;

#define ReadDatabyIdentifier 0x22
#define WriteDatabyIdentifier 0x2e
#define HexaMessage 0
#define HelloMessage 1
#define RequestMessage 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
CAN_TxHeaderTypeDef Node1_TxHeader;
CAN_RxHeaderTypeDef Node1_RxHeader;

uint8_t Node1_TxData[8];
uint8_t Node1_RxData[8];

uint32_t Node1_TxMailbox;

CAN_TxHeaderTypeDef Node2_TxHeader;
CAN_RxHeaderTypeDef Node2_RxHeader;

uint8_t Node2_TxData[8];
uint8_t Node2_RxData[8];

uint32_t Node2_TxMailbox;

uint8_t Node2_Data[6] = "Hello!";

CAN_TxHeaderTypeDef Request_TxHeader;
uint8_t Request_TxData;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN_Tester_Task */
osThreadId_t CAN_Tester_TaskHandle;
const osThreadAttr_t CAN_Tester_Task_attributes = {
  .name = "CAN_Tester_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN_ECU_Task */
osThreadId_t CAN_ECU_TaskHandle;
const osThreadAttr_t CAN_ECU_Task_attributes = {
  .name = "CAN_ECU_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void CAN_Tester(void *argument);
void CAN_ECU(void *argument);

/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Node1_RxHeader, Node1_RxData);
  if (Node1_RxHeader.DLC == 8)
  {
    printf("Node1 get data: %s\r\n", Node1_RxData);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &Node2_RxHeader, Node2_RxData);
  if (Node2_RxHeader.DLC == 8)
  {
	  if(Node2_RxData[0] == ReadDatabyIdentifier)
	  {
		  if(Node2_RxData[7] == HelloMessage)
		  {
			printf("%s", Node2_Data);
		  }
		  else
		{
			for(int i=0; i< 6; i++)
			{
				printf("%x ", Node2_Data[i]);
			}
		}
		printf("\r\n");
	  }

	  if(Node2_RxData[0] == WriteDatabyIdentifier)
	  {
		  for(int i=1; i< 7; i++)
		  {
			  Node2_Data[i - 1] = Node2_RxData[i];
		  }
	  }
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	UNUSED(htim);
	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	Node1_TxData[0] = ReadDatabyIdentifier;
	HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData,
				  &Node1_TxMailbox);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == LEFT_Pin)
  {
	  Node1_TxData[0] = WriteDatabyIdentifier;
	  Node1_TxData[1] = 0xaa;
	  Node1_TxData[2] = 0xaa;
	  Node1_TxData[3] = 0xaa;
	  Node1_TxData[4] = 0xaa;
	  Node1_TxData[5] = 0xaa;
	  Node1_TxData[6] = 0xaa;
	  Node1_TxData[7] = HexaMessage;
	  HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData,
	  				  &Node1_TxMailbox);
  }else if(GPIO_Pin == RIGHT_Pin)
  {
	  Node1_TxData[0] = WriteDatabyIdentifier;
	  Node1_TxData[1] = 0xff;
	  Node1_TxData[2] = 0xff;
	  Node1_TxData[3] = 0xff;
	  Node1_TxData[4] = 0xff;
	  Node1_TxData[5] = 0xff;
	  Node1_TxData[6] = 0xff;
	  Node1_TxData[7] = HexaMessage;
	  HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData,
					  &Node1_TxMailbox);
  }else if(GPIO_Pin == PRESS_Pin)
  {
	  Node1_TxData[0] = WriteDatabyIdentifier;
	  Node1_TxData[1] = 0x00;
	  Node1_TxData[2] = 0x00;
	  Node1_TxData[3] = 0x00;
	  Node1_TxData[4] = 0x00;
	  Node1_TxData[5] = 0x00;
	  Node1_TxData[6] = 0x00;
	  Node1_TxData[7] = HexaMessage;
	  HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData,
					  &Node1_TxMailbox);
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);

    // Activate the notification
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    // Activate the notification
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

    Node1_TxHeader.DLC = 8;  // data length
    Node1_TxHeader.IDE = CAN_ID_STD;
    Node1_TxHeader.RTR = CAN_RTR_DATA;
    Node1_TxHeader.StdId = 0x555;  // ID

    Node1_TxData[0] = 0x00;
    Node1_TxData[1] = 'H';
    Node1_TxData[2] = 'e';
    Node1_TxData[3] = 'l';
    Node1_TxData[4] = 'l';
    Node1_TxData[5] = 'o';
    Node1_TxData[6] = '!';
    Node1_TxData[7] = HelloMessage;

    Node2_TxHeader.DLC = 8;  // data length
    Node2_TxHeader.IDE = CAN_ID_STD;
    Node2_TxHeader.RTR = CAN_RTR_DATA;
    Node2_TxHeader.StdId = 0x2AA;  // ID

    Node2_TxData[0] = 'N';
    Node2_TxData[1] = '0';
    Node2_TxData[2] = '0';
    Node2_TxData[3] = 'b';
    Node2_TxData[4] = 'B';
    Node2_TxData[5] = 'y';
    Node2_TxData[6] = 'e';
    Node2_TxData[7] = '!';

    Request_TxHeader.DLC = 1;  // data length
	Request_TxHeader.IDE = CAN_ID_STD;
	Request_TxHeader.RTR = CAN_RTR_DATA;
	Request_TxHeader.StdId = 0x555;  // ID

	Request_TxData = 0x00;

	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CAN_Tester_Task */
  CAN_Tester_TaskHandle = osThreadNew(CAN_Tester, NULL, &CAN_Tester_Task_attributes);

  /* creation of CAN_ECU_Task */
  CAN_ECU_TaskHandle = osThreadNew(CAN_ECU, NULL, &CAN_ECU_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED0_Pin);
//	      HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData,
//	          &Node1_TxMailbox);
//	      HAL_CAN_AddTxMessage(&hcan2, &Node2_TxHeader, Node2_TxData,
//	          &Node2_TxMailbox);
	      HAL_Delay(500);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
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
  	CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 18; // which filter bank to use from the assigned ones
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig.FilterIdHigh = 0x2AA << 5;
    canfilterconfig.FilterIdLow = 0;
    canfilterconfig.FilterMaskIdHigh = 0x2AA << 5;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 20; // how many filters to assign to the CAN1 (master can)

    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10; // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  canfilterconfig.FilterIdHigh = 0x555 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x555 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0; // doesn't matter in single can controllers

  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);
  /* USER CODE END CAN2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PRESS_Pin LEFT_Pin RIGHT_Pin */
  GPIO_InitStruct.Pin = PRESS_Pin|LEFT_Pin|RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY0_Pin KEY1_Pin */
  GPIO_InitStruct.Pin = KEY0_Pin|KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 100);
  return ch;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED0_Pin);
//	HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData,
//	  &Node1_TxMailbox);
//	HAL_CAN_AddTxMessage(&hcan2, &Node2_TxHeader, Node2_TxData,
//	  &Node2_TxMailbox);

	HAL_Delay(500);
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_CAN_Tester */
/**
* @brief Function implementing the CAN_Tester_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Tester */
void CAN_Tester(void *argument)
{
  /* USER CODE BEGIN CAN_Tester */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_Tester */
}

/* USER CODE BEGIN Header_CAN_ECU */
/**
* @brief Function implementing the CAN_ECU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_ECU */
void CAN_ECU(void *argument)
{
  /* USER CODE BEGIN CAN_ECU */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_ECU */
}

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
