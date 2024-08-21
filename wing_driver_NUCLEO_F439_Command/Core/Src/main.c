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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "retarget.h"
#include "usbh_cdc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MTDATA2 0x36					//XCODE for Device reading
#define EULERANGLES 0x2030				//XCODE for Euler angles reading
#define ACCELERATION 0x4020				//XCODE for Acceleration angles reading
#define RATEOFTURN 0x8020				//XCODE for RateofTurn angles reading
#define LATLON 0x5040					//XCODE for Latitude and Longitude reading
#define l_F 0.65						//Length of Wing, Mast-Fore
#define l_A 0.65						//Length of Wing, Mast-Aft
#define l_B 1.25						//Length of Wing, Mast-Bord
#define l_T 1.25						//Length of Wing, Mast-Tribord
#define l_M 1.2							//Length of Mast
#define RX_USB_BUFF_SIZE   64  			//Length of Received data from USB
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
uint8_t UART_rxBuffer[54] = {0};							//Used to store latest reading from UART
uint8_t UART_rx7Buffer[16] = {0};							//Used to store latest reading from UART

float euler[3];                                             //Used to store latest EulerAngle reading
float acc[3];                                               //Used to store latest Acceleration reading
float rot[3];                                               //Used to store latest RateOfTurn reading
float latlon[2];											//Used to store latest Latitude and Longitude reading
float alt;													//Used to store latest Altitude reading

CAN_TxHeaderTypeDef   TxHeader;								//Used to send CAN Header
CAN_RxHeaderTypeDef   RxHeader;								//Used to receive CAN Header
CAN_FilterTypeDef sFilterConfig;							//Used to filter CAN messages
uint8_t               TxData[8];							//Used to store latest sent data for CAN
uint8_t               RxData[8];							//Used to store latest reading from CAN
uint32_t              TxMailbox;							//Used to store latest mailbox from CAN


float deltaL_F = 0;											//Used to store the difference of string for the Fore strings
float deltaL_A = 0;											//Used to store the difference of string for the Aft strings
float deltaL_B = 0;											//Used to store the difference of string for the Babord strings
float deltaL_T = 0;											//Used to store the difference of string for the Tribord strings

float deltaL_1 = 0;											//Used to store the difference of string for the L1 string
float deltaL_2 = 0;											//Used to store the difference of string for the L2 string
float deltaL_3 = 0;											//Used to store the difference of string for the L3 string
float deltaL_4 = 0;											//Used to store the difference of string for the L4 string

float roll;													//Used to store the roll value
float pitch;												//Used to store the pitch value
float rotX;													//Used to store the rotational velocity in the X axis value
float rotY;													//Used to store the rotational velocity in the Y axis value

float alpha_ref;												//Used to store the reference of the roll
float beta_ref;													//Used to store the reference of the pitch

float e_alpha;												//Used to store the error in the roll orientation
float e_beta;												//Used to store the error in the pitch orientation

float V_2;													//Used to store the measured velocity of P2 in the wing
float V_3;													//Used to store the measured velocity of P3 in the wing

float I_1;													//Used to store the reference of the current in L1 motor
float I_4;													//Used to store the reference of the current in L4 motor

int rudder_angle = 0;										//Value of the angle of the rudder in degrees from -30 to 30 (TODO)

int speed_motor_boat = 0;									//Value of the speed of the motor of the boat in percentage from -100 to 100

int flagWingMode, rudder_ref, flagBoatPowerSwitch, boatThrottle, flagSpeedMode, flagBoatBrake = 0;

uint8_t x =100;

extern USBH_HandleTypeDef hUsbHostFS;
extern ApplicationTypeDef Appli_state;
USBH_StatusTypeDef usbresult;
uint8_t CDC_RX_Buffer[RX_USB_BUFF_SIZE];
uint8_t buffer[RX_USB_BUFF_SIZE];

typedef enum {
  CDC_STATE_IDLE = 0,
  CDC_SEND,
  CDC_RECEIVE,
}CDC_StateTypedef;

CDC_StateTypedef CDC_STATE = CDC_RECEIVE;

uint32_t awa = 0;
uint32_t aws = 0;

uint32_t t0 = 0;
uint32_t timeEllapsed;

uint32_t tx_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART7_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void parseMTData2(uint8_t* data, uint8_t datalength);
void dataswapendian(uint8_t* data, uint8_t length);
void config_CAN(void);
void CDC_HANDLE (void);
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
	volatile int i = 10000000 *2/5;
	while (i--);
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
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_HOST_Init();
  MX_CAN1_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  config_CAN();															//Configures the CAN1 peripheral
  HAL_CAN_Start(&hcan1);												//Starts the CAN1 peripheral
  HAL_UART_Receive_DMA (&huart6, UART_rxBuffer, 54);					//Assigns the reception of DMA to the memory space to UART_rxBuffer
  HAL_UART_Receive_DMA (&huart7, UART_rx7Buffer, 16);					//Assigns the reception of DMA to the memory space to UART_rxBuffer
  RetargetInit(&huart3);
  printf("\n----------------------------------------------------------------\n\r");

  t0 = HAL_GetTick();
  /*Test values*/
  TxData[0] = 0;
  TxData[1] = 0x00;
  flagBoatBrake = 1;
  flagSpeedMode = 3;
  flagBoatPowerSwitch = 1;
  flagWingMode = 1;

  rudder_ref = 15;
  boatThrottle = 50;

  alpha_ref = 0;															//roll in degrees
  beta_ref = 0;																//pitch in degrees.
  I_1 = 5;																	//Reference current in motor 1 in Amperes
  I_4 = 5;																	//Reference current in motor 4 in Amperes




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    /*Calculate the error in orientation*/
    e_alpha = alpha_ref - roll;
    e_beta = beta_ref - pitch;

	/* Obtaining the linearized differences in rope */
	deltaL_F = l_F*(e_beta*3.14159/180);
	deltaL_A = -l_A*(e_beta*3.14159/180);
	deltaL_B = -l_B*(e_alpha*3.14159/180);
	deltaL_T = l_T*(e_alpha*3.14159/180);

	deltaL_1 = deltaL_F + deltaL_T;
	deltaL_2 = deltaL_A + deltaL_T;
	deltaL_3 = deltaL_A + deltaL_B;
	deltaL_4 = deltaL_F + deltaL_B;

	V_2 = rotX*(-l_T) - rotY*(-l_A);										//Obtains the vel. of P2 with respect of the wing (+ goes up - goes down)
	V_3 = rotX*(l_B) - rotY*(-l_A);											//Obtains the vel. of P3 with respect of the wing (+ goes up - goes down)

	if (deltaL_2>0) {
		I_4 = 15;
	}
	else {
		I_4 = 1;
	}

	if (deltaL_3>0) {
		I_1 = 15;
	}
	else {
		I_1 = 1;
	}
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (&(I_1)), &TxMailbox);
	TxHeader.StdId=0x002;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (&(I_4)), &TxMailbox);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
	TxHeader.StdId=0x003;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (&(deltaL_2)), &TxMailbox);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
	TxHeader.StdId=0x004;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (&(deltaL_3)), &TxMailbox);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
	TxHeader.StdId=0x005;
	TxData[3] = rudder_ref;
	TxData[2] = 0;
	TxData[1] = 0;
	TxData[0] = 0;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (TxData), &TxMailbox);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
	TxHeader.StdId=0x006;
	TxData[3] = boatThrottle;
	TxData[2] = 0;
	TxData[1] = 0;
	TxData[0] = 0;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (TxData), &TxMailbox);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
	TxHeader.StdId=0x007;
	TxData[3] = flagWingMode;
	TxData[2] = flagBoatPowerSwitch;
	TxData[1] = flagSpeedMode;
	TxData[0] = flagBoatBrake;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (TxData), &TxMailbox);
//	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
//	TxHeader.StdId=0x005;
//	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (&(V_2)), &TxMailbox);
//	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
//	TxHeader.StdId=0x006;
//	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (&(V_3)), &TxMailbox);
	TxHeader.StdId=0x001;


	if (Appli_state == APPLICATION_READY)
	{
		CDC_HANDLE();
		for (int i = 0;  i<RX_USB_BUFF_SIZE; i++)
		{
			if (buffer[i] == 0x01 && buffer[i+1] == 0x04 && buffer[i+2] == 0x05 && buffer[i+8] == 0x81)
			{
				aws = (( (uint32_t) (buffer[i+4]))<<8U | ( (uint32_t) buffer[i+3])) * 50000 / 65535;
				awa = (( (uint32_t) (buffer[i+6]))<<8U | ( (uint32_t) buffer[i+5]));
				awa = awa * (360.0 / 65536);
				timeEllapsed = HAL_GetTick() - t0;
				printf("t(ms): %lu ; awa(deg): %u; aws(m/s): %u.%u; roll: %d; pitch: %d \n\r", timeEllapsed, (unsigned int)(awa), (unsigned int)aws/100, (unsigned int)aws%100, (int)roll, (int) pitch);
			}
		}
	}
	else
	{
		timeEllapsed = HAL_GetTick() - t0;
		printf("t(ms): %lu ; roll: %d.%u; pitch: %d.%d ;L2(mm) %d ;L3(mm) %d ;I1(A) %d; I4(A) %d \n\r", timeEllapsed, (int)roll,(unsigned int)(roll*100)%100 , (int) pitch,(unsigned int)(pitch*100)%100, (int)(deltaL_1*1000), (int)(deltaL_2*1000), (int)I_1, (int)I_4);

	}
//	  HAL_Delay (1000);												//Arbitrary delay


	/*Test routine*/
	//HAL_UART_Transmit(&huart7, (uint8_t)(x), sizeof (uint8_t), 10);
//	timeEllapsed = HAL_GetTick() - t0;
//	printf("t(ms): %lu ;roll: %d; pitch: %d; L2(mm): %d;L3(mm): %d ; I1(A): %d;I4(A): %d\n\r", timeEllapsed,(int)roll, (int) pitch, (int) (deltaL_2*1000), (int) (deltaL_3*1000),(int) I_1,(int) I_4);
	//HAL_UART_Transmit(&huart7, (int8_t)(&(deltaL_2)), sizeof (int8_t), 10);
	HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hcan1.Init.Prescaler = 24;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
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

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 9600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CTS_GPIO_Port, CTS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CTS_Pin */
  GPIO_InitStruct.Pin = CTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CTS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void config_CAN(void)
{
	/*Defining the Header of the TX message*/
	  TxHeader.DLC=4;
	  TxHeader.IDE=CAN_ID_STD;
	  TxHeader.RTR=CAN_RTR_DATA;
	  TxHeader.StdId=0x001;

	/*Configuring the RX filter*/
	  sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
	  sFilterConfig.FilterIdHigh=0x245<<5;
	  sFilterConfig.FilterIdLow=0;
	  sFilterConfig.FilterMaskIdHigh=0;
	  sFilterConfig.FilterMaskIdLow=0;
	  sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterActivation=ENABLE;
	  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

	  /*Activating the notification to receive a message*/
	  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	parseMTData2(UART_rxBuffer+2,51);										//Parses the data received from the MT device
	roll = euler[0];
	pitch =  euler[1];
	rotX = rot[0];
	rotY = rot[1];

	if (UART_rx7Buffer[0] == 0x01)
	{
		flagWingMode = (int) UART_rx7Buffer[1];
	}
	if (UART_rx7Buffer[2] == 0x02)
	{
		beta_ref = (signed char) UART_rx7Buffer[3];
	}
	if (UART_rx7Buffer[4] == 0x03)
	{
		alpha_ref = (signed char) UART_rx7Buffer[5];
	}
	if (UART_rx7Buffer[6] == 0x04)
	{
		rudder_ref = (signed char) UART_rx7Buffer[7];
	}
	if (UART_rx7Buffer[8] == 0x05)
	{
		flagBoatPowerSwitch = (int) UART_rx7Buffer[9];
	}
	if (UART_rx7Buffer[10] == 0x06)
	{
		boatThrottle = (signed char) UART_rx7Buffer[11];
	}
	if (UART_rx7Buffer[12] == 0x07)
	{
		flagSpeedMode = (int) UART_rx7Buffer[13];
	}
	if (UART_rx7Buffer[14] == 0x08)
	{
		flagBoatBrake = (int) UART_rx7Buffer[15];
	}


//	if((signed char)UART_rx7Buffer[0]>-60 && (signed char)UART_rx7Buffer[0]<60){
//		alpha_ref = (signed char)UART_rx7Buffer[0];
//	}
//	if((signed char)UART_rx7Buffer[0]>-60 && (signed char)UART_rx7Buffer[0]<60){
//		beta_ref = (signed char)UART_rx7Buffer[1];
//	}
//	HAL_UART_Transmit(&huart3, (&(pitch)), sizeof (float), 10);
//	HAL_UART_Transmit(&huart3, (&(rotY)), sizeof (float), 10);
//	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (&(pitch)), &TxMailbox);
//	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (&(rotY)), &TxMailbox);
	HAL_UART_Receive_DMA(&huart6, UART_rxBuffer, 54);						//Waits for a new message to be received
}

void parseMTData2(uint8_t* data, uint8_t datalength) {
  if (datalength < 2)                                                           //Reached the end of the MTData2 message
    return;

  uint8_t XDI = data[0] ;                                                       //Xsens Data Identifier
  if (XDI == (uint8_t)MTDATA2) {                                         //Start of the MTData2 message
    uint8_t length = data[1];
    parseMTData2(data + 2, length);
  } else {
    uint8_t length = data[2];

    switch (((uint16_t)data[1] | ((uint16_t)data[0] << 8)) & (uint16_t)0xFFFF) { //Extract the 2-byte Xsens Data Identifier
      case ((uint16_t)EULERANGLES):
        dataswapendian(data + 3, sizeof(float) * 3);
        memcpy(euler, data + 3, sizeof(float) * 3);
        break;
      case ((uint16_t)ACCELERATION):
        dataswapendian(data + 3, sizeof(float) * 3);
        memcpy(acc, data + 3, sizeof(float) * 3);
        break;
      case ((uint16_t)RATEOFTURN):
        dataswapendian(data + 3, sizeof(float) * 3);
        memcpy(rot, data + 3, sizeof(float) * 3);
        break;
      case ((uint16_t)LATLON):
        dataswapendian(data + 3, sizeof(float) * 2);
        memcpy(latlon, data + 3, sizeof(float) * 2);
        break;
      default:
        break;
    }
    parseMTData2(data + length + 3, datalength - length - 3);                     //Move onto next data element within MTData2 packet
  }
}

void dataswapendian(uint8_t* data, uint8_t length) {                         		 //Swap the endianness of the data such that the float value can be printed
  uint8_t cpy[length];                                                              //Create a copy of the data
  memcpy(cpy, data, length);                                                        //Create a copy of the data
  for (int i = 0; i < length / 4; i++) {
    for (int j = 0; j < 4; j++) {
      data[j + i * 4] = cpy[3 - j + i * 4];                                         //Within each 4-byte (32-bit) float, reverse the order of the individual bytes
    }
  }
}

void CDC_HANDLE (void)
{
	USBH_CDC_Stop(&hUsbHostFS);
	usbresult = USBH_CDC_Receive(&hUsbHostFS, (uint8_t *) CDC_RX_Buffer, RX_USB_BUFF_SIZE);
	HAL_Delay (100);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{

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
