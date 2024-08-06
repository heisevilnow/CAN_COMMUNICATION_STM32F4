//CAN2

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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef Scanfilter;

CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;

uint32_t TxMailbox;

uint8_t TxData[8];
uint8_t RxData[8];
uint8_t flag;
uint8_t flag_display;
uint8_t check[8];

uint8_t message3[100] = "Board 3 pressed\n";

uint8_t message_request[100] = "Board 2 sends request to ";


uint8_t choose12[] = "Choose LED 12\n";

uint8_t choose13[] = "Choose LED 13\n";

uint8_t turnon[]= "Turn ON\n";
uint8_t turnoff[]= "Turn OFF\n";

uint8_t message1[100] = "Board 1 pressed\n";

uint8_t message2[100] = "Board 2 pressed\n";

uint8_t message4[100] = "Board 4 pressed\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
        flag=0x01;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  if(GPIO_Pin==GPIO_PIN_0)
  {
      TxData[0]=0x8E;
      TxData[1]=0xFF;
	/* Send data*/
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)!= HAL_OK)
    {
            Error_Handler();
    }
    HAL_UART_Transmit_IT(&huart1, message2, sizeof(message2));
  }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==huart1.Instance)
  {
    HAL_UART_Receive_IT(&huart1, check, 3);
    flag_display = 0x01;
    
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, check, &TxMailbox)!= HAL_OK)
    { 
            Error_Handler();
    }
    
    if(check[0]==0x8E)
    {
      switch (check[1])
      {
      case 0x01: 
        if(check[2]==0x01)
          HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
        else if(check[2]==0x00)
          HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
        break;
       case 0x02: 
        if(check[2]==0x01)
          HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
        else if(check[2]==0x00)
          HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);      
        break;
      }
    }
  }
}

void NotifyLed(){
    switch (RxData[1])
    {
    case 0x01: 
       HAL_Delay(200);
       HAL_UART_Transmit_IT(&huart1, choose12, sizeof(choose12));
       HAL_Delay(200);
      if(RxData[2]==0x01)
      {
        HAL_UART_Transmit_IT(&huart1, turnon, sizeof(turnon));
      }
      else if(RxData[2]==0x00)
      {
        HAL_UART_Transmit_IT(&huart1, turnoff, sizeof(turnoff));
      }
      break;
     case 0x02: 
      HAL_Delay(200);
      HAL_UART_Transmit_IT(&huart1, choose13, sizeof(choose13));
      HAL_Delay(200);
      if(RxData[2]==0x01)
      {
        HAL_UART_Transmit_IT(&huart1, turnon, sizeof(turnon));
      }
      else if(RxData[2]==0x00)
      {
        HAL_UART_Transmit_IT(&huart1, turnoff, sizeof(turnoff));
      }
      break;
    }
}


void ActiveLed(){
  switch (RxData[1])
    {
    case 0x01: 
       HAL_Delay(200);
       HAL_UART_Transmit_IT(&huart1, choose12, sizeof(choose12));
       HAL_Delay(200);
      if(RxData[2]==0x01)
      {
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
        HAL_UART_Transmit_IT(&huart1, turnon, sizeof(turnon));
      }
      else if(RxData[2]==0x00)
      {
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);  
        HAL_UART_Transmit_IT(&huart1, turnoff, sizeof(turnoff));
      }
      break;
     case 0x02: 
      HAL_Delay(200);
      HAL_UART_Transmit_IT(&huart1, choose13, sizeof(choose13));
      HAL_Delay(200);
      if(RxData[2]==0x01)
      {
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);

        HAL_UART_Transmit_IT(&huart1, turnon, sizeof(turnon));
      }
      else if(RxData[2]==0x00)
      {
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);   

        HAL_UART_Transmit_IT(&huart1, turnoff, sizeof(turnoff));
      }
      break;
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
                
                TxHeader.StdId=0x8E;
                TxHeader.DLC = 8;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.RTR = CAN_RTR_DATA;
                
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  

  
  /* USER CODE END 2 */
  HAL_UART_Receive_IT(&huart1, check, 3);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
    if(flag_display==0x01)
    {
      HAL_UART_Transmit_IT(&huart1, message_request, sizeof(message_request));
      HAL_Delay(200);
      if(check[0]==0x7E)
      {
        uint8_t arr[]= "Board 1\n";
        HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));

      }
      else if(check[0]==0x8E)
      {
        uint8_t arr[]= "Board 2\n";
        HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));

      }
      else if(check[0]==0x9E)
      {
        uint8_t arr[]= "Board 3\n";
        HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));
      }
      else if(check[0]==0x6E)
      {
        uint8_t arr[]= "Board 4\n";
        HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));
      }
      
      HAL_Delay(100);
      
      if(check[1]==0x01)
      {
        HAL_UART_Transmit_IT(&huart1, choose12, sizeof(choose12));
      }
      else if(check[1]==0x02)
      {
        HAL_UART_Transmit_IT(&huart1, choose13, sizeof(choose13));
      }
      
      HAL_Delay(100);
      
      if(check[2]==0x01)
      {
        HAL_UART_Transmit_IT(&huart1, turnon, sizeof(turnon));
      }
      else if(check[2]==0x00)
      {

        HAL_UART_Transmit_IT(&huart1, turnoff, sizeof(turnoff));
      }
      
      flag_display=0;
    }
    if(flag==0x01)
    {
      if(RxData[1]!= 0xFF){
          if(RxHeader.StdId == 0x8E)
          {
            uint8_t arr[]= "Board 2 sends request to ";
            HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));
          }
          else if(RxHeader.StdId == 0x9E)
          {
            uint8_t arr[]= "Board 3 sends request to ";
            HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));
          }
          else if(RxHeader.StdId == 0x7E)
          {
            uint8_t arr[]= "Board 1 sends request to ";
            HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));
          }
          else if(RxHeader.StdId == 0x6E)
          {
            uint8_t arr[]= "Board 4 sends request to ";
            HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));
          }
          HAL_Delay(200);
          
          if(RxData[0]==0x7E)
          {
            uint8_t arr[]= "Board 1\n";
            HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));
            NotifyLed();
          }
          else if(RxData[0]==0x8E)
          {
            uint8_t arr[]= "Board 2\n";
            HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));
            ActiveLed();
            
          }
          else if(RxData[0]==0x6E)
          {
            uint8_t arr[]= "Board 4\n";
            HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));
            NotifyLed();
          }
          else if(RxData[0]==0x9E)
          {
            uint8_t arr[]= "Board 3\n";
            HAL_UART_Transmit_IT(&huart1, arr, sizeof(arr));
            NotifyLed();
          }
          
          HAL_Delay(100);
      }
      else{
        switch (RxData[0])
        {
        case 0x7E:
          if(RxData[1]==0xFF)
          {
            HAL_UART_Transmit_IT(&huart1, message1, sizeof(message1));
          }
          break;  
        case 0x9E:
          if(RxData[1]==0xFF)
          {
            HAL_UART_Transmit_IT(&huart1, message3, sizeof(message3));
          }
          break; 
        case 0x6E:
          if(RxData[1]==0xFF)
          {
            HAL_UART_Transmit_IT(&huart1, message4, sizeof(message4));
          }
          break; 
        }
      }
        flag=0;
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
CAN_FilterTypeDef canfilterconfig;              //Std = 0x11

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;  
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0000;                //0x15<<5
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
canfilterconfig.SlaveStartFilterBank = 20;  

	if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
