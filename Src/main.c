/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/*******************************************************************************
* Author: Benjamin Scholtz
* Contact: bscholtz.bds@gmail.com
* Purpose: Mechatronic Engineering Undergrad Thesis: Baleka - Robotic Hopping Leg
* Tools: STM32CubeMX, FreeRTOS, HAL
*******************************************************************************/

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <complex.h>    /* Standard Library of Complex Numbers */

#include "CRC.h"
#include "arrayFunctions.h"
#include <stm32f4xx_hal_uart.h>
#include <stm32f4xx_hal_usart.h>

//https://github.com/PetteriAimonen/Baselibc
//#include "memccpy.c"
//#include "memcmp.c"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId defaultTaskHandle;
osThreadId TXPCHandle;
osThreadId RXPCHandle;
osThreadId HeartbeatHandle;
osThreadId TXMotor1Handle;
osThreadId TXMotor2Handle;
osThreadId RXMotor1Handle;
osThreadId RXMotor2Handle;
osThreadId ControllerHandle;
osMessageQId ProcessQM1Handle;
osMessageQId ProcessQM2Handle;
osMessageQId ProcessQPCHandle;
osMessageQId TransmitM1QHandle;
osMessageQId TransmitM2QHandle;
osMessageQId ICommandM1QHandle;
osMessageQId ICommandM2QHandle;
osMessageQId PCommandM1QHandle;
osMessageQId PCommandM2QHandle;
osMessageQId CommandM1QHandle;
osMessageQId CommandM2QHandle;
osMessageQId ControllerQHandle;
osMessageQId ControlM1QHandle;
osMessageQId ControlM2QHandle;
osMessageQId ProcessQControlHandle;
osSemaphoreId M1Handle;
osSemaphoreId M2Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//Communication Timing
uint8_t Ts = 25; //Sampling time in 1/_X_ ms
uint8_t Td = 3;
//NB: #define configTICK_RATE_HZ ((TickType_t)_X_000) in FreeRTOSConfig

uint8_t RXBufPC[50];
uint8_t RXBufM1[50];
uint8_t RXBufM2[50];

#define PI 3.141592653f

//Motor UART connections
#define M1_UART huart3
#define M2_UART huart2

//Packet Op-Codes
#define KILL_BRIDGE 0
#define WRITE_ENABLE 1
#define BRIDGE_ENABLE 2
#define CURRENT_COMMAND 20
#define POSITION_COMMAND 22
#define ZERO_POSITION 8
#define GAIN_SET 9
#define GAIN_CHANGE_M1 10
#define GAIN_CHANGE_M2 13
#define CONFIG_SET 16

#define CONTROL_CURRENT_M1 40
#define CONTROL_CURRENT_M2 41

#define START_CONTROL 30

////////////////////////////////////////////////////////////////////////

uint8_t KILL_BRIDGE_DATA[2] = {0x01, 0x00};
uint8_t WRITE_ENABLE_DATA[2] = {0x0F, 0x00};
uint8_t BRIDGE_ENABLE_DATA[2] = {0x00, 0x00};
uint8_t GAIN_SET_0_DATA[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t GAIN_SET_1_DATA[4] = {0x08, 0x00, 0x00, 0x00};
uint8_t ZERO_POSITION_DATA[4] = {0x08, 0x00, 0x00, 0x00};
uint8_t CONFIG_0_SET_DATA[2] = {0x00, 0x00};
uint8_t CONFIG_1_SET_DATA[2] = {0x01, 0x00};

////////////////////////////////////////////////////////////////////////
//TX Packet to PC

//'packed' makes sure compiler won't insert any gaps!
struct __attribute__((__packed__)) TXPacketStruct {
        uint8_t START[2];

        uint8_t M1C[2];
        uint8_t M1P[4];
        uint8_t M1V[4];

        uint8_t M2C[2];
        uint8_t M2P[4];
        uint8_t M2V[4];

        // uint8_t ACCX[2];
        // uint8_t ACCY[2];
        // uint8_t ACCZ[2];
        // uint8_t GYRX[2];
        // uint8_t GYRY[2];
        // uint8_t GYRZ[2];
        // uint8_t TEMP;
        uint8_t MISC[16];

        uint8_t StatBIT_1 : 1;
        uint8_t StatBIT_2 : 1;
        uint8_t StatBIT_3 : 1;
        uint8_t StatBIT_4 : 1;
        uint8_t StatBIT_5 : 1;
        uint8_t StatBIT_6 : 1;
        uint8_t StatBIT_7 : 1;
        uint8_t StatBIT_8 : 1;

        uint8_t CRCCheck[2];

        uint8_t STOP[2];
};

struct TXPacketStruct PCPacket;
//Transmit pointer PCPacketPTR with sizeof(PCPacket)
uint8_t *PCPacketPTR = (uint8_t*)&PCPacket;

////////////////////////////////////////////////////////////////////////
//RX Packet from PC

struct __attribute__((__packed__)) RXPacketStruct {
        uint8_t START[2];

        uint8_t OPCODE;

        uint8_t M1C[4];
        uint8_t M2C[4];

        uint8_t M1P[4];
        uint8_t M2P[4];

        uint8_t StatBIT_1 : 1;
        uint8_t StatBIT_2 : 1;
        uint8_t StatBIT_3 : 1;
        uint8_t StatBIT_4 : 1;
        uint8_t StatBIT_5 : 1;
        uint8_t StatBIT_6 : 1;
        uint8_t StatBIT_7 : 1;
        uint8_t StatBIT_8 : 1;

        uint8_t CRCCheck[2];

        uint8_t STOP[2];
};

struct RXPacketStruct RXPacket;
uint8_t *RXPacketPTR = (uint8_t*)&RXPacket;

uint8_t RX_DATA_VALID = 0;

////////////////////////////////////////////////////////////////////////
//Driver Command Compilation

struct __attribute__((__packed__)) BaseCommandStruct {
        uint8_t START[2];
        uint8_t CB;
        uint8_t INDOFF[2];
        uint8_t LEN;
        uint8_t CRC1[2];
        uint8_t DATA[4];
        uint8_t CRC2[2];
};

uint8_t SNIP;

struct BaseCommandStruct BaseCommand[50];
struct BaseCommandStruct* BaseCommandPTR;

//Used for converting from word to byte array etc.
union {
        uint32_t WORD;
        uint16_t HALFWORD;
        uint8_t BYTE[4];
} WORDtoBYTEBase;

uint32_t CALC_CRCBase;

////////////////////////////////////////////////////////////////////////
//Controller

struct /*__attribute__((__packed__))*/ ControlPacketStruct {
        uint8_t M1C[2];
        uint8_t M1P[4];
        uint8_t M1V[4];

        uint8_t M2C[2];
        uint8_t M2P[4];
        uint8_t M2V[4];

        uint8_t ACCX[2];
        uint8_t ACCY[2];
        uint8_t ACCZ[2];
        uint8_t GYRX[2];
        uint8_t GYRY[2];
        uint8_t GYRZ[2];
        uint8_t TEMP;
};

struct ControlPacketStruct ControlPacket;

struct __attribute__((__packed__)) ControlLogStruct {
	float f_r;
	float f_theta;
	float I_cmd_0;
	float I_cmd_1;
};

struct ControlLogStruct ControlLogPacket;
uint8_t *ControlLogPacketPTR = (uint8_t*)&ControlLogPacket;

uint8_t START = 0;

union {
        float FLOAT;
        int32_t INT32;
        int16_t INT16;
        uint8_t BYTE[4];
} F2BM1;

union {
        float FLOAT;
        int32_t INT32;
        int16_t INT16;
        uint8_t BYTE[4];
} F2BM2;

////////////////////////////////////////////////////////////////////////
//Binary Semaphores

SemaphoreHandle_t PCRXHandle;
SemaphoreHandle_t PCTXHandle;

SemaphoreHandle_t TXMotorM1Handle;
SemaphoreHandle_t TXMotorM2Handle;
SemaphoreHandle_t RXMotorM1Handle;
SemaphoreHandle_t RXMotorM2Handle;

SemaphoreHandle_t ControlM1Handle;
SemaphoreHandle_t ControlM2Handle;

////////////////////////////////////////////////////////////////////////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void const * argument);
void StartTXPC(void const * argument);
void StartRXPC(void const * argument);
void StartHeartbeat(void const * argument);
void StartTXMotor1(void const * argument);
void StartTXMotor2(void const * argument);
void StartRXMotor1(void const * argument);
void StartRXMotor2(void const * argument);
void StartController(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void SetupBinarySemaphores(void);

//Motor driver packet compilation function
int32_t swap_int32( int32_t val );
void BaseCommandCompile(uint8_t n, uint8_t SeqBits, uint8_t ComBits, uint8_t INDOFF1, uint8_t INDOFF2, uint8_t *DATA, uint8_t LEN, uint8_t SNIP);
//void ControlBaseCommandCompile(uint8_t n, uint8_t SeqBits, uint8_t ComBits, uint8_t INDOFF1, uint8_t INDOFF2, uint8_t *DATA, uint8_t LEN, uint8_t SNIP_LEN);

//Motor driver DMA commands
void TransmitM1_DMA(uint8_t *data, uint8_t size);
void ReceiveM1_DMA(uint8_t *data, uint8_t size);
void TransmitM2_DMA(uint8_t *data, uint8_t size);
void ReceiveM2_DMA(uint8_t *data, uint8_t size);

//Call-back functions
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart);
void DMA_XFER_CPLT_Callback(DMA_HandleTypeDef *_hdma);

void HAL_UART_EndDMA_RX(UART_HandleTypeDef *huart);

//Kinematics: SI Units and Radians
float *ForwardKinematics(float phi1, float phi2);
float *InverseKinematics(float r, float theta);

/* USER CODE END PFP */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();

  /* USER CODE BEGIN 2 */
        initCRC(0); //iNemo CRC False
        initCRC(1); //Driver CRC XModem
        SetupBinarySemaphores();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
        /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of M1 */
  osSemaphoreDef(M1);
  M1Handle = osSemaphoreCreate(osSemaphore(M1), 1);

  /* definition and creation of M2 */
  osSemaphoreDef(M2);
  M2Handle = osSemaphoreCreate(osSemaphore(M2), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
        /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
        /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TXPC */
  osThreadDef(TXPC, StartTXPC, osPriorityRealtime, 0, 128);
  TXPCHandle = osThreadCreate(osThread(TXPC), NULL);

  /* definition and creation of RXPC */
  osThreadDef(RXPC, StartRXPC, osPriorityRealtime, 0, 128);
  RXPCHandle = osThreadCreate(osThread(RXPC), NULL);

  /* definition and creation of Heartbeat */
  osThreadDef(Heartbeat, StartHeartbeat, osPriorityRealtime, 0, 128);
  HeartbeatHandle = osThreadCreate(osThread(Heartbeat), NULL);

  /* definition and creation of TXMotor1 */
  osThreadDef(TXMotor1, StartTXMotor1, osPriorityRealtime, 0, 128);
  TXMotor1Handle = osThreadCreate(osThread(TXMotor1), NULL);

  /* definition and creation of TXMotor2 */
  osThreadDef(TXMotor2, StartTXMotor2, osPriorityRealtime, 0, 128);
  TXMotor2Handle = osThreadCreate(osThread(TXMotor2), NULL);

  /* definition and creation of RXMotor1 */
  osThreadDef(RXMotor1, StartRXMotor1, osPriorityRealtime, 0, 128);
  RXMotor1Handle = osThreadCreate(osThread(RXMotor1), NULL);

  /* definition and creation of RXMotor2 */
  osThreadDef(RXMotor2, StartRXMotor2, osPriorityRealtime, 0, 128);
  RXMotor2Handle = osThreadCreate(osThread(RXMotor2), NULL);

  /* definition and creation of Controller */
  osThreadDef(Controller, StartController, osPriorityRealtime, 0, 500);
  ControllerHandle = osThreadCreate(osThread(Controller), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
        /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of ProcessQM1 */
  osMessageQDef(ProcessQM1, 1, uint32_t);
  ProcessQM1Handle = osMessageCreate(osMessageQ(ProcessQM1), NULL);

  /* definition and creation of ProcessQM2 */
  osMessageQDef(ProcessQM2, 1, uint32_t);
  ProcessQM2Handle = osMessageCreate(osMessageQ(ProcessQM2), NULL);

  /* definition and creation of ProcessQPC */
  osMessageQDef(ProcessQPC, 1, uint32_t);
  ProcessQPCHandle = osMessageCreate(osMessageQ(ProcessQPC), NULL);

  /* definition and creation of TransmitM1Q */
  osMessageQDef(TransmitM1Q, 3, uint32_t);
  TransmitM1QHandle = osMessageCreate(osMessageQ(TransmitM1Q), NULL);

  /* definition and creation of TransmitM2Q */
  osMessageQDef(TransmitM2Q, 3, uint32_t);
  TransmitM2QHandle = osMessageCreate(osMessageQ(TransmitM2Q), NULL);

  /* definition and creation of ICommandM1Q */
  osMessageQDef(ICommandM1Q, 1, uint32_t);
  ICommandM1QHandle = osMessageCreate(osMessageQ(ICommandM1Q), NULL);

  /* definition and creation of ICommandM2Q */
  osMessageQDef(ICommandM2Q, 1, uint32_t);
  ICommandM2QHandle = osMessageCreate(osMessageQ(ICommandM2Q), NULL);

  /* definition and creation of PCommandM1Q */
  osMessageQDef(PCommandM1Q, 1, uint32_t);
  PCommandM1QHandle = osMessageCreate(osMessageQ(PCommandM1Q), NULL);

  /* definition and creation of PCommandM2Q */
  osMessageQDef(PCommandM2Q, 1, uint32_t);
  PCommandM2QHandle = osMessageCreate(osMessageQ(PCommandM2Q), NULL);

  /* definition and creation of CommandM1Q */
  osMessageQDef(CommandM1Q, 3, uint32_t);
  CommandM1QHandle = osMessageCreate(osMessageQ(CommandM1Q), NULL);

  /* definition and creation of CommandM2Q */
  osMessageQDef(CommandM2Q, 3, uint32_t);
  CommandM2QHandle = osMessageCreate(osMessageQ(CommandM2Q), NULL);

  /* definition and creation of ControllerQ */
  osMessageQDef(ControllerQ, 1, uint32_t);
  ControllerQHandle = osMessageCreate(osMessageQ(ControllerQ), NULL);

  /* definition and creation of ControlM1Q */
  osMessageQDef(ControlM1Q, 1, uint32_t);
  ControlM1QHandle = osMessageCreate(osMessageQ(ControlM1Q), NULL);

  /* definition and creation of ControlM2Q */
  osMessageQDef(ControlM2Q, 1, uint32_t);
  ControlM2QHandle = osMessageCreate(osMessageQ(ControlM2Q), NULL);

  /* definition and creation of ProcessQControl */
  osMessageQDef(ProcessQControl, 1, uint32_t);
  ProcessQControlHandle = osMessageCreate(osMessageQ(ProcessQControl), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
        /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
        while (1)
        {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

        }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 500000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
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

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
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

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void SetupBinarySemaphores(void){
        PCRXHandle = xSemaphoreCreateBinary();
        PCTXHandle = xSemaphoreCreateBinary();

        TXMotorM1Handle = xSemaphoreCreateBinary();
        TXMotorM2Handle = xSemaphoreCreateBinary();
        RXMotorM1Handle = xSemaphoreCreateBinary();
        RXMotorM2Handle = xSemaphoreCreateBinary();

        ControlM1Handle = xSemaphoreCreateBinary();
        ControlM2Handle = xSemaphoreCreateBinary();
}

//! Byte swap int
int32_t swap_int32( int32_t val )
{
        val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF );
        return (val << 16) | ((val >> 16) & 0xFFFF);
}

void BaseCommandCompile(uint8_t n, uint8_t SeqBits, uint8_t ComBits, uint8_t INDOFF1, uint8_t INDOFF2, uint8_t *DATA, uint8_t LEN, uint8_t SNIP_LEN){
        memset(&BaseCommand[n], 0, sizeof(BaseCommand[0]));

        //ComBits = 0x02 for set and 0x00 for read
        //SeqBits = 0bXXXX according to op-code

        BaseCommand[n].START[0] = 0xA5;
        BaseCommand[n].START[1] = 0x3F;
        BaseCommand[n].CB = (SeqBits<<2 | ComBits);
        BaseCommand[n].INDOFF[0] = INDOFF1;
        BaseCommand[n].INDOFF[1] = INDOFF2;
        BaseCommand[n].LEN = LEN;
        CALC_CRCBase = crcCalc(BaseCommand[n].START, 0, 6, 1);
        WORDtoBYTEBase.HALFWORD = CALC_CRCBase;
        BaseCommand[n].CRC1[0] = WORDtoBYTEBase.BYTE[1];
        BaseCommand[n].CRC1[1] = WORDtoBYTEBase.BYTE[0];

        if(DATA != NULL) {
                for(int i = 0; i<LEN*2; i++) {
                        BaseCommand[n].DATA[i] = DATA[i];
                }
                CALC_CRCBase = crcCalc(BaseCommand[n].DATA, 0, LEN*2, 1);
                WORDtoBYTEBase.HALFWORD = CALC_CRCBase;
                BaseCommand[n].CRC2[0] = WORDtoBYTEBase.BYTE[1];
                BaseCommand[n].CRC2[1] = WORDtoBYTEBase.BYTE[0];
        }

        SNIP = SNIP_LEN;

        memcpy(&BaseCommand[n].DATA[4-SNIP], BaseCommand[n].CRC2, 2);

        if(SNIP==2) {
                memset(&BaseCommand[n].CRC2, 0, 2);
        }
        if(SNIP==4) {
                memset(&BaseCommand[n].DATA, 0, 4);
                memset(&BaseCommand[n].CRC2, 0, 2);
        }
}

//void ControlBaseCommandCompile(uint8_t n, uint8_t CB, uint8_t INDOFF1, uint8_t INDOFF2, uint8_t *DATA, uint8_t LEN, uint8_t SNIP_LEN){
//        memset(&ControlBaseCommand[n], 0, sizeof(ControlBaseCommand[0]));
//
//        ControlBaseCommand[n].START[0] = 0xA5;
//        ControlBaseCommand[n].START[1] = 0x3F;
//        ControlBaseCommand[n].CB = CB;
//        ControlBaseCommand[n].INDOFF[0] = INDOFF1;
//        ControlBaseCommand[n].INDOFF[1] = INDOFF2;
//        ControlBaseCommand[n].LEN = LEN;
//        CALC_CRCBase = crcCalc(ControlBaseCommand[n].START, 0, 6, 1);
//        WORDtoBYTEBase.HALFWORD = CALC_CRCBase;
//        ControlBaseCommand[n].CRC1[0] = WORDtoBYTEBase.BYTE[1];
//        ControlBaseCommand[n].CRC1[1] = WORDtoBYTEBase.BYTE[0];
//
//        if(DATA != NULL) {
//                for(int i = 0; i<LEN*2; i++) {
//                        ControlBaseCommand[n].DATA[i] = DATA[i];
//                }
//                CALC_CRCBase = crcCalc(ControlBaseCommand[n].DATA, 0, LEN*2, 1);
//                WORDtoBYTEBase.HALFWORD = CALC_CRCBase;
//                ControlBaseCommand[n].CRC2[0] = WORDtoBYTEBase.BYTE[1];
//                ControlBaseCommand[n].CRC2[1] = WORDtoBYTEBase.BYTE[0];
//        }
//
//        SNIP = SNIP_LEN;
//
//        memcpy(&ControlBaseCommand[n].DATA[4-SNIP], ControlBaseCommand[n].CRC2, 2);
//
//        if(SNIP==2) {
//                memset(&ControlBaseCommand[n].CRC2, 0, 2);
//        }
//        if(SNIP==4) {
//                memset(&ControlBaseCommand[n].DATA, 0, 4);
//                memset(&ControlBaseCommand[n].CRC2, 0, 2);
//        }
//}

void TransmitM1_DMA(uint8_t *data, uint8_t size){
        /* Start the transmission - an interrupt is generated when the transmission
           is complete. */
        //if(HAL_UART_Transmit_DMA(&huart2, data, size) != HAL_OK) { Error_Handler(); }
        HAL_UART_Transmit_DMA(&M1_UART, data, size);
}

void ReceiveM1_DMA(uint8_t *data, uint8_t size){
        HAL_UART_Receive_DMA(&M1_UART, data, size);
        //HAL_UART_Receive(&huart2, data, size, 5);
}

void TransmitM2_DMA(uint8_t *data, uint8_t size){
        /* Start the transmission - an interrupt is generated when the transmission
           is complete. */
        //if(HAL_UART_Transmit_DMA(&huart3, data, size) != HAL_OK) { Error_Handler(); }
        HAL_UART_Transmit_DMA(&M2_UART, data, size);
}

void ReceiveM2_DMA(uint8_t *data, uint8_t size){
        HAL_UART_Receive_DMA(&M2_UART, data, size);
        //HAL_UART_Receive(&huart3, data, size, 5);
}

//Select Call-backs functions called after Transfer complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
        __NOP();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;

        if(huart->Instance == UART4) {
                xSemaphoreGiveFromISR( PCRXHandle, &xHigherPriorityTaskWoken );
        }

//        if(huart->Instance == USART2 && __HAL_USART_GET_FLAG(huart, USART_FLAG_IDLE)) {
//                __HAL_USART_CLEAR_IDLEFLAG(huart);
//                xSemaphoreGiveFromISR( RXMotorM1Handle, &xHigherPriorityTaskWoken );
//        }
//
//        if(huart->Instance == USART3 && __HAL_USART_GET_FLAG(huart, USART_FLAG_IDLE)) {
//                __HAL_USART_CLEAR_IDLEFLAG(huart);
//                xSemaphoreGiveFromISR( RXMotorM2Handle, &xHigherPriorityTaskWoken );
//        }

        /* If xHigherPriorityTaskWoken was set to true you
           we should yield.  The actual macro used here is
           port specific. portYIELD_FROM_ISR */
        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

//http://www.riuson.com/blog/post/stm32-hal-uart-dma-rx-variable-length
//void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart){
//        BaseType_t xHigherPriorityTaskWoken;
//        xHigherPriorityTaskWoken = pdFALSE;
//
//        if(huart->Instance == UART4 && __HAL_USART_GET_FLAG(huart, USART_FLAG_IDLE)) {
//                __HAL_USART_CLEAR_IDLEFLAG(huart);
//                xSemaphoreGiveFromISR( PCRXHandle, &xHigherPriorityTaskWoken );
//        }
//
////        if(huart->Instance == USART2 && __HAL_USART_GET_FLAG(huart, USART_FLAG_IDLE)) {
////                __HAL_USART_CLEAR_IDLEFLAG(huart);
////                xSemaphoreGiveFromISR( RXMotorM1Handle, &xHigherPriorityTaskWoken );
////        }
////
////        if(huart->Instance == USART3 && __HAL_USART_GET_FLAG(huart, USART_FLAG_IDLE)) {
////                __HAL_USART_CLEAR_IDLEFLAG(huart);
////                xSemaphoreGiveFromISR( RXMotorM2Handle, &xHigherPriorityTaskWoken );
////        }
//
//        /* If xHigherPriorityTaskWoken was set to true you
//           we should yield.  The actual macro used here is
//           port specific. portYIELD_FROM_ISR */
//        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
//}

void DMA_XFER_CPLT_Callback(DMA_HandleTypeDef *_hdma){
        __NOP();
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
//    BaseType_t xHigherPriorityTaskWoken;
//    xHigherPriorityTaskWoken = pdFALSE;
//
//    if(_hdma == &hdma_uart4_rx) {
//            xSemaphoreGiveFromISR( PCRXHandle, &xHigherPriorityTaskWoken );
//    }
//
////        if(huart->Instance == USART2 && __HAL_USART_GET_FLAG(huart, USART_FLAG_IDLE)) {
////                __HAL_USART_CLEAR_IDLEFLAG(huart);
////                xSemaphoreGiveFromISR( RXMotorM1Handle, &xHigherPriorityTaskWoken );
////        }
////
////        if(huart->Instance == USART3 && __HAL_USART_GET_FLAG(huart, USART_FLAG_IDLE)) {
////                __HAL_USART_CLEAR_IDLEFLAG(huart);
////                xSemaphoreGiveFromISR( RXMotorM2Handle, &xHigherPriorityTaskWoken );
////        }
//
//    /* If xHigherPriorityTaskWoken was set to true you
//       we should yield.  The actual macro used here is
//       port specific. portYIELD_FROM_ISR */
//    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
//    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void HAL_UART_EndDMA_RX(UART_HandleTypeDef *huart){
/* Stop UART DMA Rx request if ongoing */
        uint32_t dmarequest = 0x00U;
        dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR);
        if((huart->RxState == HAL_UART_STATE_BUSY_RX) && dmarequest)
        {
                CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
                /* Abort the UART DMA Rx channel */
                if(huart->hdmarx != NULL)
                {
                        HAL_DMA_Abort(huart->hdmarx);
                }
                /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
                CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
                CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
                /* At end of Rx process, restore huart->RxState to Ready */
                huart->RxState = HAL_UART_STATE_READY;
        }
}

float *ForwardKinematics(float phi1, float phi2){
        //function [r,theta] = fcn(phi1,phi2)

        uint8_t valid = 1;
        static float ret[2];

        //phi1 = (phi1*2*PI)/360.0; //To radians
        //phi2 = (phi2*2*PI)/360.0;

        static float l1 = 0.15; //length of upper linkage in m (measured from center of joint of 5 cm diameter)
        static float l2 = 0.3; //length of lower linkage in m (measured from center of joint of 5 cm diameter)

        ret[0] = fabs(-l1*cosf((phi1 + phi2)/2.0) + sqrt(pow(l2,2) - pow(l1,2)*pow(sinf((phi1 + phi2)/2.0),2))); //r
        ret[1] = (phi1 - phi2)/2.0; //theta

        //ret[1] = (ret[1]*360)/(2.0*PI); //To degrees

        if(phi1*360/(2*PI) > 185 || phi1*360/(2*PI) < 25){ //162 90
          valid = 0;
        }

        if(phi2*360/(2*PI) > 185 || phi2*360/(2*PI) < 25){
          valid = 0;
        }

        if(valid) {
                return ret;
        }
        else{
                return NULL;
        }
}

float *InverseKinematics(float r, float theta){
        //function [phi1,phi2] = fcn(r,theta)

        uint8_t valid = 1;
        static float ret[2];

        theta = (theta*2*PI)/360.0;

        static float l1 = 0.15; //length of upper linkage in m (measured from center of joint of 5 cm diameter)
        static float l2 = 0.3; //length of lower linkage in m (measured from center of joint of 5 cm diameter)

        if (r == 0) {r = 0.000001; }

        //float complex cmp1;
        float cmp1 = (pow(r,2) + pow(l1,2) - pow(l2,2))/(2.0*r*l1);

        //float complex cmp2;
        float cmp2 = (pow(r,2) + pow(l1,2) - pow(l2,2))/(2.0*r*l1);

        ret[0] = fabs(PI - acosf(cmp1) + theta); //phi1
        ret[1] = fabs(PI - acosf(cmp2) - theta); //phi2

        ret[0] = (ret[0]*360)/(2.0*PI);
        ret[1] = (ret[1]*360)/(2.0*PI);

        if(valid) {
                return ret;
        }
        else{
                return NULL;
        }
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
        vTaskSuspend( NULL );
        /* Infinite loop */
        for(;; )
        {
                vTaskDelay(500);
        }
  /* USER CODE END 5 */ 
}

/* StartTXPC function */
void StartTXPC(void const * argument)
{
  /* USER CODE BEGIN StartTXPC */

        memset(PCPacketPTR, 0, sizeof(PCPacket));

        PCPacket.START[0] = 0x7E;
        PCPacket.START[1] = 0x5B;

        PCPacket.STOP[0] = 0x5D;
        PCPacket.STOP[1] = 0x7E;

        uint32_t CALC_CRC;

        union {
                uint32_t WORD;
                uint16_t HALFWORD;
                uint8_t BYTE[4];
        } WORDtoBYTE;

        uint8_t *pxRxedMessage;

        uint8_t CurrentPCPacket[sizeof(PCPacket)];

        /* Infinite loop */
        for(;; )
        {
                xSemaphoreTake( PCTXHandle,  portMAX_DELAY );

                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);

                if(xQueueReceive( ProcessQM1Handle, &pxRxedMessage, 0 )) {
                        memcpy(PCPacket.M1C, pxRxedMessage, 10);
                }

                if(xQueueReceive( ProcessQM2Handle, &pxRxedMessage, 0 )) {
                        memcpy(PCPacket.M2C, pxRxedMessage, 10);
                }

                if(xQueueReceive( ProcessQControlHandle, &pxRxedMessage, 0 )) {
                        memcpy(PCPacket.MISC, pxRxedMessage, 13);
                }

                CALC_CRC = crcCalc(&PCPacket.M1C, 0, 37, 0);
                WORDtoBYTE.HALFWORD = CALC_CRC;
                PCPacket.CRCCheck[0] = WORDtoBYTE.BYTE[1];
                PCPacket.CRCCheck[1] = WORDtoBYTE.BYTE[0];

                memcpy(CurrentPCPacket, PCPacketPTR, sizeof(PCPacket));

//                /* Disable TXEIE and TCIE interrupts */
//                CLEAR_BIT(huart4.Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
//                /* At end of Tx process, restore huart->gState to Ready */
//                huart4.gState = HAL_UART_STATE_READY;

                HAL_UART_Transmit_DMA(&huart4, CurrentPCPacket, sizeof(PCPacket)); //TODO

                PCPacket.StatBIT_1 = 0;
                PCPacket.StatBIT_2 = 0;
                PCPacket.StatBIT_3 = 0;
                PCPacket.StatBIT_4 = 0;
                PCPacket.StatBIT_5 = 0;
                PCPacket.StatBIT_6 = 0;
                PCPacket.StatBIT_7 = 0;
                PCPacket.StatBIT_8 = 0;

                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
        }
  /* USER CODE END StartTXPC */
}

/* StartRXPC function */
void StartRXPC(void const * argument)
{
  /* USER CODE BEGIN StartRXPC */

        //From PC

        uint8_t RXPacketLen;
        RXPacketLen = 24;

        RXPacket.START[0] = 0x7E;
        RXPacket.START[1] = 0x5B;

        RXPacket.STOP[0] = 0x5D;
        RXPacket.STOP[1] = 0x7E;


        int8_t START_INDEX = 0;
        uint32_t CALC_CRC;

        union {
                uint32_t WORD;
                uint16_t HALFWORD;
                uint8_t BYTE[4];
        } WORDtoBYTE;

        uint8_t rcvdCount;

        HAL_Delay(1000);
        //__HAL_DMA_ENABLE_IT(&hdma_uart4_rx, DMA_IT_TC);
        //HAL_DMA_RegisterCallback(&hdma_uart4_rx, HAL_DMA_XFER_CPLT_CB_ID, DMA_XFER_CPLT_Callback);

//                __HAL_USART_CLEAR_IDLEFLAG(&huart4);
//                __HAL_USART_ENABLE_IT(&huart4, USART_IT_IDLE);
//
//                __HAL_USART_CLEAR_IDLEFLAG(&huart3);
//                __HAL_USART_ENABLE_IT(&huart3, USART_IT_IDLE);

        /* Infinite loop */
        for(;; )
        {
                HAL_UART_Receive_DMA(&huart4, RXBufPC, sizeof(RXPacket));
                //HAL_DMA_Start(&hdma_uart4_rx, (uint32_t)(&(UART4->DR)), (uint32_t)(&RXBufPC), sizeof(RXPacket));
                if(xSemaphoreTake( PCRXHandle, portMAX_DELAY ) == pdTRUE) {

                        rcvdCount = sizeof(RXPacket);
                        //rcvdCount = sizeof(RXBufPC) - huart4.hdmarx->Instance->NDTR;
                        //HAL_UART_EndDMA_RX(&huart4);

                        //xQueueReset(TransmitM1QHandle);
                        //xQueueReset(TransmitM2QHandle);

                        START_INDEX = findBytes(RXBufPC, rcvdCount, RXPacket.START, 2, 1);
                        if(START_INDEX>=0) {

                                memcpy(RXPacketPTR, &RXBufPC[START_INDEX], RXPacketLen);
                                RX_DATA_VALID = 0;

                                WORDtoBYTE.BYTE[1] = RXPacket.CRCCheck[0];
                                WORDtoBYTE.BYTE[0] = RXPacket.CRCCheck[1];
                                CALC_CRC = crcCalc(&RXPacket.OPCODE, 0, 18, 0); //Check entire data CRC

                                if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                        RX_DATA_VALID = 1;
                                        START = 0;
                                        switch(RXPacket.OPCODE) {
                                        case KILL_BRIDGE:
                                                START = 0;
                                                BaseCommandCompile(RXPacket.OPCODE, 0b0001, 0x02, 0x01, 0x00, KILL_BRIDGE_DATA, 1, 2);
                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                                xQueueSendToBack( CommandM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                xQueueSendToBack( CommandM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                break;
                                        case WRITE_ENABLE:
                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                                BaseCommandCompile(RXPacket.OPCODE, 0b0010, 0x02, 0x07, 0x00, WRITE_ENABLE_DATA, 1, 2);
                                                xQueueSendToBack( CommandM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                xQueueSendToBack( CommandM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                break;
                                        case BRIDGE_ENABLE:
                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                                BaseCommandCompile(RXPacket.OPCODE, 0b0100, 0x02, 0x01, 0x00, BRIDGE_ENABLE_DATA, 1, 2);
                                                xQueueSendToBack( CommandM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                xQueueSendToBack( CommandM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                break;
                                        case CURRENT_COMMAND:
                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                                BaseCommandCompile(RXPacket.OPCODE, 0b0011, 0x02, 0x45, 0x02, RXPacket.M1C, 2, 0);
                                                xQueueOverwrite( ICommandM1QHandle, &BaseCommandPTR);

                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE+1];
                                                BaseCommandCompile(RXPacket.OPCODE+1, 0b0011, 0x02, 0x45, 0x02, RXPacket.M2C, 2, 0);
                                                xQueueOverwrite( ICommandM2QHandle, &BaseCommandPTR);
                                                break;
                                        case POSITION_COMMAND:
                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                                BaseCommandCompile(RXPacket.OPCODE, 0b1010, 0x02, 0x45, 0x00, RXPacket.M1P, 2, 0);
                                                xQueueOverwrite( PCommandM1QHandle, &BaseCommandPTR);

                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE+1];
                                                BaseCommandCompile(RXPacket.OPCODE+1, 0b1010, 0x02, 0x45, 0x00, RXPacket.M2P, 2, 0);
                                                xQueueOverwrite( PCommandM2QHandle, &BaseCommandPTR);
                                                break;
                                        case ZERO_POSITION:
                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                                BaseCommandCompile(RXPacket.OPCODE, 0b0000, 0x02, 0x01, 0x00, ZERO_POSITION_DATA, 2, 0);
                                                xQueueSendToBack( CommandM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                xQueueSendToBack( CommandM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                break;
                                        case GAIN_SET:
                                                if(RXPacket.StatBIT_1 == 0) {
                                                        BaseCommandCompile(RXPacket.OPCODE, 0b0000, 0x02, 0x01, 0x01, GAIN_SET_0_DATA, 2, 0);
                                                }
                                                else{
                                                        BaseCommandCompile(RXPacket.OPCODE, 0b0000, 0x02, 0x01, 0x01, GAIN_SET_1_DATA, 2, 0);
                                                }
                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                                xQueueSendToBack( CommandM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                xQueueSendToBack( CommandM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                break;
                                        case GAIN_CHANGE_M1:
                                                //PID Position Loop
                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                                BaseCommandCompile(RXPacket.OPCODE, 0b0000, 0x02, 0x38, 0x00, RXPacket.M1C, 2, 0);
                                                xQueueSendToBack( CommandM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE+1];
                                                BaseCommandCompile(RXPacket.OPCODE+1, 0b0000, 0x02, 0x38, 0x02, RXPacket.M1P, 2, 0);
                                                xQueueSendToBack( CommandM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE+2];
                                                BaseCommandCompile(RXPacket.OPCODE+2, 0b0000, 0x02, 0x38, 0x04, RXPacket.M2C, 2, 0);
                                                xQueueSendToBack( CommandM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                break;
                                        case GAIN_CHANGE_M2:
                                                //PID Position Loop
                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                                BaseCommandCompile(RXPacket.OPCODE, 0b0000, 0x02, 0x38, 0x00, RXPacket.M1C, 2, 0);
                                                xQueueSendToBack( CommandM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE+1];
                                                BaseCommandCompile(RXPacket.OPCODE+1, 0b0000, 0x02, 0x38, 0x02, RXPacket.M1P, 2, 0);
                                                xQueueSendToBack( CommandM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE+2];
                                                BaseCommandCompile(RXPacket.OPCODE+2, 0b0000, 0x02, 0x38, 0x04, RXPacket.M2C, 2, 0);
                                                xQueueSendToBack( CommandM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                break;
                                        case CONFIG_SET:
                                                if(RXPacket.StatBIT_2 == 0) {
                                                        BaseCommandCompile(RXPacket.OPCODE, 0b1001, 0x02, 0xD1, 0x00, CONFIG_0_SET_DATA, 1, 2);
                                                }
                                                else{
                                                        BaseCommandCompile(RXPacket.OPCODE, 0b1001, 0x02, 0xD1, 0x00, CONFIG_1_SET_DATA, 1, 2);
                                                }
                                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                                xQueueSendToBack( CommandM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                xQueueSendToBack( CommandM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                                break;
                                        case START_CONTROL:
                                        		if(RXPacket.StatBIT_3 == 1){
                                        			START = 1;
                                        		}
                                        		else{
                                        			START = 0;
                                        		}
                                                break;
                                        default:
                                                break;
                                        }
                                }
                        }

                }
        }
  /* USER CODE END StartRXPC */
}

/* StartHeartbeat function */
void StartHeartbeat(void const * argument)
{
  /* USER CODE BEGIN StartHeartbeat */


        /* Infinite loop */
        for(;; )
        {
                //Read Current
                BaseCommandCompile(5, 0b1100, 0x01, 0x10, 0x03, NULL, 1, 4);
                BaseCommandPTR = &BaseCommand[5];
                xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                //Read Position
                BaseCommandCompile(6, 0b1111, 0x01, 0x12, 0x00, NULL, 2, 4);
                BaseCommandPTR = &BaseCommand[6];
                xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                //Read Velocity
                BaseCommandCompile(7, 0b0101, 0x01, 0x11, 0x02, NULL, 2, 4);
                BaseCommandPTR = &BaseCommand[7];
                xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                xSemaphoreGive(TXMotorM1Handle);
                xSemaphoreGive(TXMotorM2Handle);

                vTaskDelay(Ts);

                xSemaphoreGive(PCTXHandle);

//                while(uxQueueMessagesWaiting( TransmitM1QHandle )
//                    || uxQueueMessagesWaiting( TransmitM2QHandle )
//                    || uxQueueMessagesWaiting( CommandM1QHandle )
//                    || uxQueueMessagesWaiting( CommandM2QHandle )
//                    || uxQueueMessagesWaiting( ICommandM1QHandle )
//                    || uxQueueMessagesWaiting( ICommandM2QHandle )
//                    || uxQueueMessagesWaiting( PCommandM1QHandle )
//                    || uxQueueMessagesWaiting( PCommandM2QHandle ));
        }
  /* USER CODE END StartHeartbeat */
}

/* StartTXMotor1 function */
void StartTXMotor1(void const * argument)
{
  /* USER CODE BEGIN StartTXMotor1 */
        uint8_t *pxRxedMessage;
        /* Infinite loop */
        for(;; )
        {
                xSemaphoreTake( TXMotorM1Handle, portMAX_DELAY );
                __HAL_UART_FLUSH_DRREGISTER(&M1_UART);
                memset(RXBufM1, 0, sizeof(RXBufM1));
                HAL_UART_Receive_DMA(&M1_UART, RXBufM1, sizeof(RXBufM1));

                while(uxQueueMessagesWaiting( CommandM1QHandle )) {
                        xQueueReceive( CommandM1QHandle, &( pxRxedMessage ), portMAX_DELAY);
                        TransmitM1_DMA(pxRxedMessage, sizeof(BaseCommand[0]));
                        vTaskDelay(Td);
                }

                if(xQueueReceive( ICommandM1QHandle, &( pxRxedMessage ), 0)) {
                        TransmitM1_DMA(pxRxedMessage, sizeof(BaseCommand[0]));
                        vTaskDelay(Td);
                }

                else if (xQueueReceive( PCommandM1QHandle, &( pxRxedMessage ), 0)) {
                        TransmitM1_DMA(pxRxedMessage, sizeof(BaseCommand[0]));
                        vTaskDelay(Td);
                }

                while(uxQueueMessagesWaiting( TransmitM1QHandle )) {
                        // Receive a message on the created queue. Block 5.
                        xQueueReceive( TransmitM1QHandle, &( pxRxedMessage ), portMAX_DELAY);
                        TransmitM1_DMA(pxRxedMessage, sizeof(BaseCommand[0])); //TODO sizing??
                        //while(huart2.gState != HAL_UART_STATE_READY);
                        vTaskDelay(Td);
                        if(uxQueueMessagesWaiting( TransmitM2QHandle )>0) {vTaskDelay(Td); }
                        else{xSemaphoreGive( RXMotorM1Handle ); }
                }
        }
  /* USER CODE END StartTXMotor1 */
}

/* StartTXMotor2 function */
void StartTXMotor2(void const * argument)
{
  /* USER CODE BEGIN StartTXMotor2 */
        uint8_t *pxRxedMessage;
        TimeOut_t xTimeOut;
        /* Infinite loop */
        for(;; )
        {
                xSemaphoreTake( TXMotorM2Handle, portMAX_DELAY );
                __HAL_UART_FLUSH_DRREGISTER(&M2_UART);
                memset(RXBufM2, 0, sizeof(RXBufM2));
                HAL_UART_Receive_DMA(&M2_UART, RXBufM2, sizeof(RXBufM2));

                while(uxQueueMessagesWaiting( CommandM2QHandle )) {
                        xQueueReceive( CommandM2QHandle, &( pxRxedMessage ), portMAX_DELAY);
                        TransmitM2_DMA(pxRxedMessage, sizeof(BaseCommand[0]));
                        vTaskDelay(Td);
                }

                if(xQueueReceive( ICommandM2QHandle, &( pxRxedMessage ), 0)) {
                        TransmitM2_DMA(pxRxedMessage, sizeof(BaseCommand[0]));
                        vTaskDelay(Td);
                }

                else if(xQueueReceive( PCommandM2QHandle, &( pxRxedMessage ), 0)) {
                        TransmitM2_DMA(pxRxedMessage, sizeof(BaseCommand[0]));
                        vTaskDelay(Td);
                }

                while(uxQueueMessagesWaiting( TransmitM2QHandle )) {
                        // Receive a message on the created queue. Block 5.
                        xQueueReceive( TransmitM2QHandle, &( pxRxedMessage ), portMAX_DELAY);
                        TransmitM2_DMA(pxRxedMessage, sizeof(BaseCommand[0]));
                        vTaskDelay(Td);
                        if(uxQueueMessagesWaiting( TransmitM2QHandle )>0) {vTaskDelay(Td); }
                        else{xSemaphoreGive( RXMotorM2Handle ); }
//                        HAL_UART_DMAResume(&huart3);
//                        vTaskSetTimeOutState( &xTimeOut );
//                        while(!__HAL_USART_GET_FLAG(&huart3, USART_FLAG_IDLE) && !xTaskCheckForTimeOut( &xTimeOut, 5 ));
//                        __HAL_USART_CLEAR_IDLEFLAG(&huart3);
//                        HAL_UART_DMAPause(&huart3);
//                        while(huart3.gState != HAL_UART_STATE_READY);

//                        while(huart3.gState != HAL_UART_STATE_READY);
//                        do{
//                            volatile uint32_t tmpreg = 0x00U;
//                            tmpreg = (&huart3)->Instance->SR;
//                            ((void)(tmpreg));
//                          } while(0);
                }

        }
  /* USER CODE END StartTXMotor2 */
}

/* StartRXMotor1 function */
void StartRXMotor1(void const * argument)
{
  /* USER CODE BEGIN StartRXMotor1 */

        uint8_t PCBuf[10] = {0};
        uint8_t *PCBufPTR;


        uint8_t OPCODE;
        uint8_t START_BYTE[2] = {0xA5, 0xFF};
        uint8_t START_SIZE = 2;
        uint8_t DATA_SIZE;
        uint8_t START_INDEX;
        uint32_t CALC_CRC;

        uint8_t INDEX_SIZE = 0;
        uint8_t INDEX[5] = {0};

        union {
                uint32_t WORD;
                uint16_t HALFWORD;
                uint8_t BYTE[4];
        } WORDtoBYTE;

        struct __attribute__((__packed__)) CURRENTStruct {
                uint8_t HEAD[8];
                uint8_t DATA[2];
                uint8_t CRC2[2];
        };

        struct __attribute__((__packed__)) POSITIONStruct {
                uint8_t HEAD[8];
                uint8_t DATA[4];
                uint8_t CRC2[2];
        };

        struct __attribute__((__packed__)) VELOCITYStruct {
                uint8_t HEAD[8];
                uint8_t DATA[4];
                uint8_t CRC2[2];
        };

        struct CURRENTStruct CURRENTrx;
        struct POSITIONStruct POSITIONrx;
        struct VELOCITYStruct VELOCITYrx;
        uint8_t *CURRENTrxPTR = (uint8_t*)&CURRENTrx;
        uint8_t *POSITIONrxPTR = (uint8_t*)&POSITIONrx;
        uint8_t *VELOCITYrxPTR = (uint8_t*)&VELOCITYrx;

        uint8_t rcvdCount;
        /* Infinite loop */
        for(;; )
        {
                //vTaskSuspend(NULL);
                xSemaphoreTake( RXMotorM1Handle, portMAX_DELAY );
                START_BYTE[0] = 0xA5;
                START_BYTE[1] = 0xFF;

                rcvdCount = sizeof(RXBufM1) - M1_UART.hdmarx->Instance->NDTR;
                HAL_UART_EndDMA_RX(&M1_UART);
                //__HAL_UART_FLUSH_DRREGISTER(&huart2);

                if(rcvdCount>0) {

                        INDEX_SIZE = findMultipleBytes(RXBufM1, rcvdCount, START_BYTE, START_SIZE, INDEX, sizeof(INDEX));

                        for(int i=0; i<INDEX_SIZE; i++) {
                                OPCODE = (RXBufM1[INDEX[i]+2] & 0b00111100)>>2;
                                START_INDEX = INDEX[i];
                                switch(OPCODE)
                                {
                                case 0b0011: //Current_Set
                                        break;
                                case 0b1100: //Current_Data
                                        DATA_SIZE = 2;
                                        memcpy(CURRENTrxPTR, &RXBufM1[START_INDEX], rcvdCount);
                                        WORDtoBYTE.BYTE[1] = CURRENTrx.CRC2[0];
                                        WORDtoBYTE.BYTE[0] = CURRENTrx.CRC2[1];
                                        CALC_CRC = crcCalc(CURRENTrx.DATA, 0, DATA_SIZE, 1);
                                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                                appendBytes(PCBuf, 10, 0, CURRENTrx.DATA, 0, DATA_SIZE);
                                                PCPacket.StatBIT_1 = 1;
                                        }
                                        break;
                                case 0b1111: //Position_Data
                                        DATA_SIZE = 4;
                                        memcpy(POSITIONrxPTR, &RXBufM1[START_INDEX], rcvdCount);
                                        WORDtoBYTE.BYTE[1] = POSITIONrx.CRC2[0];
                                        WORDtoBYTE.BYTE[0] = POSITIONrx.CRC2[1];
                                        CALC_CRC = crcCalc(POSITIONrx.DATA, 0, DATA_SIZE, 1);
                                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                                appendBytes(PCBuf, 10, 2, POSITIONrx.DATA, 0, DATA_SIZE);
                                                PCPacket.StatBIT_2 = 1;
                                        }
                                        break;
                                case 0b0101: //Velocity_Data
                                        DATA_SIZE = 4;
                                        memcpy(VELOCITYrxPTR, &RXBufM1[START_INDEX], rcvdCount);
                                        WORDtoBYTE.BYTE[1] = VELOCITYrx.CRC2[0];
                                        WORDtoBYTE.BYTE[0] = VELOCITYrx.CRC2[1];
                                        CALC_CRC = crcCalc(VELOCITYrx.DATA, 0, DATA_SIZE, 1);
                                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                                appendBytes(PCBuf, 10, 2 + 4, VELOCITYrx.DATA, 0, DATA_SIZE);
                                                PCPacket.StatBIT_3 = 1;
                                        }
                                        break;
                                default:
                                        break;
                                }
                        }
                }


                PCBufPTR = &PCBuf;
                if(PCPacket.StatBIT_1 && PCPacket.StatBIT_2 && PCPacket.StatBIT_3) {
                        xQueueOverwrite(ProcessQM1Handle, &PCBufPTR);
                }
                if(PCPacket.StatBIT_1 && PCPacket.StatBIT_2 && PCPacket.StatBIT_3) {
                        xQueueOverwrite(ControlM1QHandle, &PCBufPTR);
                }


                //Rx A5 Rx FF Rx CMD -> Check bits 2-5 for opcode -> Move on to specific Rx
                //worst case TX blocked for 5ms and new transmission takes place, reply should be picked up
                // Command Byte (byte 3 of packet) Opcode: xx xx:
                // TX->Reply
                // Set Current (0)
                // 00xx xx10->00xx xx00
                // 0000 1110->0000 1100
                // 0x0E->0x0C
                // Read Current (2)
                // 00xx xx01->00xx xx10
                // 0011 0001->0011 0010
                // 0x31->0x32
                // Read Velocity (4)
                // 00xx xx01->00xx xx10
                // 0011 1101->0011 1110
                // 0x3D->0x3E
                // Read Position (4)
                // 00xx (2)xx(1) 01->00xx xx10
                // 0001 0101->0001 0110
                // 0x15->0x16
        }
  /* USER CODE END StartRXMotor1 */
}

/* StartRXMotor2 function */
void StartRXMotor2(void const * argument)
{
  /* USER CODE BEGIN StartRXMotor2 */

        uint8_t PCBuf[10] = {0};
        uint8_t *PCBufPTR;

        uint8_t OPCODE;
        uint8_t START_BYTE[2] = {0xA5, 0xFF};
        uint8_t START_SIZE = 2;
        uint8_t DATA_SIZE;
        uint8_t START_INDEX;
        uint32_t CALC_CRC;

        uint8_t INDEX_SIZE = 0;
        uint8_t INDEX[5] = {0};

        union {
                uint32_t WORD;
                uint16_t HALFWORD;
                uint8_t BYTE[4];
        } WORDtoBYTE;

        struct __attribute__((__packed__)) CURRENTStruct {
                uint8_t HEAD[8];
                uint8_t DATA[2];
                uint8_t CRC2[2];
        };

        struct __attribute__((__packed__)) POSITIONStruct {
                uint8_t HEAD[8];
                uint8_t DATA[4];
                uint8_t CRC2[2];
        };

        struct __attribute__((__packed__)) VELOCITYStruct {
                uint8_t HEAD[8];
                uint8_t DATA[4];
                uint8_t CRC2[2];
        };

        struct CURRENTStruct CURRENTrx;
        struct POSITIONStruct POSITIONrx;
        struct VELOCITYStruct VELOCITYrx;
        uint8_t *CURRENTrxPTR = (uint8_t*)&CURRENTrx;
        uint8_t *POSITIONrxPTR = (uint8_t*)&POSITIONrx;
        uint8_t *VELOCITYrxPTR = (uint8_t*)&VELOCITYrx;

        uint8_t rcvdCount;
        /* Infinite loop */
        for(;; )
        {
                //vTaskSuspend(NULL);

                xSemaphoreTake( RXMotorM2Handle, portMAX_DELAY );
                START_BYTE[0] = 0xA5;
                START_BYTE[1] = 0xFF;

                rcvdCount = sizeof(RXBufM2) - M2_UART.hdmarx->Instance->NDTR;
                HAL_UART_EndDMA_RX(&M2_UART);
                //__HAL_UART_FLUSH_DRREGISTER(&huart3);

                if(rcvdCount>0) {

                        INDEX_SIZE = findMultipleBytes(RXBufM2, rcvdCount, START_BYTE, START_SIZE, INDEX, sizeof(INDEX));

                        for(int i=0; i<INDEX_SIZE; i++) {
                                OPCODE = (RXBufM2[INDEX[i]+2] & 0b00111100)>>2;
                                START_INDEX = INDEX[i];
                                switch(OPCODE)
                                {
                                case 0b0011: //Current_Set
                                        break;
                                case 0b1100: //Current_Data
                                        DATA_SIZE = 2;
                                        memcpy(CURRENTrxPTR, &RXBufM2[START_INDEX], rcvdCount);
                                        WORDtoBYTE.BYTE[1] = CURRENTrx.CRC2[0];
                                        WORDtoBYTE.BYTE[0] = CURRENTrx.CRC2[1];
                                        CALC_CRC = crcCalc(CURRENTrx.DATA, 0, DATA_SIZE, 1);
                                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                                appendBytes(PCBuf, 10, 0, CURRENTrx.DATA, 0, DATA_SIZE);
                                                PCPacket.StatBIT_4 = 1;
                                                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
                                        }
                                        break;
                                case 0b1111: //Position_Data
                                        DATA_SIZE = 4;
                                        memcpy(POSITIONrxPTR, &RXBufM2[START_INDEX], rcvdCount);
                                        WORDtoBYTE.BYTE[1] = POSITIONrx.CRC2[0];
                                        WORDtoBYTE.BYTE[0] = POSITIONrx.CRC2[1];
                                        CALC_CRC = crcCalc(POSITIONrx.DATA, 0, DATA_SIZE, 1);
                                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                                appendBytes(PCBuf, 10, 2, POSITIONrx.DATA, 0, DATA_SIZE);
                                                PCPacket.StatBIT_5 = 1;
                                                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
                                        }
                                        break;
                                case 0b0101: //Velocity_Data
                                        DATA_SIZE = 4;
                                        memcpy(VELOCITYrxPTR, &RXBufM2[START_INDEX], rcvdCount);
                                        WORDtoBYTE.BYTE[1] = VELOCITYrx.CRC2[0];
                                        WORDtoBYTE.BYTE[0] = VELOCITYrx.CRC2[1];
                                        CALC_CRC = crcCalc(VELOCITYrx.DATA, 0, DATA_SIZE, 1);
                                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                                appendBytes(PCBuf, 10, 2 + 4, VELOCITYrx.DATA, 0, DATA_SIZE);
                                                PCPacket.StatBIT_6 = 1;
                                                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
                                        }
                                        break;
                                default:
                                        break;
                                }
                        }

                }

                PCBufPTR = &PCBuf;
                if(PCPacket.StatBIT_4 && PCPacket.StatBIT_5 && PCPacket.StatBIT_6) {
                        xQueueOverwrite(ProcessQM2Handle, &PCBufPTR);
                        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
                }
                if(PCPacket.StatBIT_4 && PCPacket.StatBIT_5 && PCPacket.StatBIT_6) {
                        xQueueOverwrite(ControlM2QHandle, &PCBufPTR);
                }

        }
  /* USER CODE END StartRXMotor2 */
}

/* StartController function */
void StartController(void const * argument)
{
  /* USER CODE BEGIN StartController */

        union {
                float FLOAT;
                int32_t INT32;
                int16_t INT16;
                uint8_t BYTE[4];
        } F2B;

        float *ret;

        float M1C = 0;
        float M1P = 0;
        float M1V = 0;
        float M2C = 0;
        float M2P = 0;
        float M2V = 0;

        float l1 = 0.15;
        float l2 = 0.3;

        float r_fbk = 0;
        float r_cmd = 0.3;
        float r_d_fbk = 0;
        float r_d_cmd = 0;

        float theta_fbk = 0;
        float theta_cmd = 0;
        float theta_d_fbk = 0;
        float theta_d_cmd = 0;

        float phi1 = 0;
        float dphi1 = 0;
        float phi1_cmd = 0;
        float phi2 = 0;
        float dphi2 = 0;
        float phi2_cmd = 0;

        float dphi1_cmd = 0;
        float dphi2_cmd = 0;

        //Virtual compliance control
        float JT[2][2] = {0};
        float F[2] = {0};
        float Tau[2] = {0};

        float ks_theta = 0;
        float kd_theta = 0;

        float ks_r = 200;
        float kd_r = 30;

        float f_r = 0;
        float f_theta = 0;

        float Ki = 0.119;

        float I_cmd[2] = {0};
        float I_fbk[2] = {0};

        uint8_t valid = 1;

        uint8_t *pxRxedMessage;

        uint8_t PCBuf[13] = {0};
        uint8_t *PCBufPTR;

        /* Infinite loop */
        for(;; )
        {
                //xSemaphoreTake( ControlM1Handle, portMAX_DELAY );
                //xSemaphoreTake( ControlM2Handle, portMAX_DELAY );

                if(uxQueueMessagesWaiting(ControlM1QHandle) && uxQueueMessagesWaiting(ControlM2QHandle)) {

                        //Data from Drivers
                        if(xQueueReceive(ControlM1QHandle, &pxRxedMessage, 0 )) {
                                memcpy(ControlPacket.M1C, pxRxedMessage, 10);
                        }
                        if(xQueueReceive(ControlM2QHandle, &pxRxedMessage, 0 )) {
                                memcpy(ControlPacket.M2C, pxRxedMessage, 10);
                        }

                        if(START==1) {
                                valid = 1;
                                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);

                                memcpy(F2B.BYTE, ControlPacket.M1C, 2);
                                I_fbk[0] = F2B.INT16/(pow(2.0,13)/60.0);

                                memcpy(F2B.BYTE, ControlPacket.M1P, 4);
                                phi1 = (F2B.INT32/(4*250.0) - 1)*(-180.0)*(2*PI/360.0);

                                memcpy(F2B.BYTE, ControlPacket.M1V, 4);
                                dphi1 = (F2B.INT32/(pow(2.0,17)/20000.0))*(1/2000.0)*60.0*(2*PI/60.0);

                                memcpy(F2B.BYTE, ControlPacket.M2C, 2);
                                I_fbk[1] = F2B.INT16/(pow(2.0,13)/60.0);

                                memcpy(F2B.BYTE, ControlPacket.M2P, 4);
                                phi2 = (F2B.INT32/(4*250.0) + 1)*180.0*(2*PI/360.0);

                                memcpy(F2B.BYTE, ControlPacket.M2V, 4);
                                dphi2 = (F2B.INT32/(pow(2.0,17)/20000.0))*(1/2000.0)*60.0*(2*PI/60.0);

                                //Forward kinematic mapping
                                ret = ForwardKinematics(phi1, phi2);
                                if(ret == NULL) {
                                	valid = 0;
                                    START = 0;
                                    BaseCommandCompile(RXPacket.OPCODE, 0b0001, 0x02, 0x01, 0x00, KILL_BRIDGE_DATA, 1, 2);
                                    BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                    xQueueSendToBack( CommandM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                    xQueueSendToBack( CommandM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                }
                                r_fbk = ret[0];
                                theta_fbk = ret[1];

                                //Velocity mapping
                                r_d_fbk = dphi1*((3*sinf(phi1/2.0 + phi2/2.0))/40 - (9*cosf(phi1/2.0 + phi2/2.0)*sinf(phi1/2.0 + phi2/2.0))/(800*pow(9/100.0 - (9*pow(sinf(phi1/2.0 + phi2/2.0),2)/400.0),0.5)))
                                          + dphi2*((3*sinf(phi1/2.0 + phi2/2))/40 - (9*cosf(phi1/2.0 + phi2/2.0)*sinf(phi1/2.0 + phi2/2.0))/(800*pow(9/100.0 - (9*pow(sinf(phi1/2.0 + phi2/2.0),2)/400.0),0.5)));
                                theta_d_fbk = dphi1/2.0 - dphi2/2.0;

                                //Data from PC
                                //RXPacketPTR; RXPacket;

                                //Control
                                JT[0][0] = (3*sinf(phi1/2.0 + phi2/2.0))/40.0 - (9*cosf(phi2/2.0 + phi2/2.0)*sinf(phi1/2.0 + phi2/2.0))/(800*pow((9/100.0 - (9*pow(sinf(phi1/2.0 + phi2/2.0),2))/400.0),0.5));
                                JT[0][1] = 0.5;
                                JT[1][0] = (3*sinf(phi1/2.0 + phi2/2.0))/40.0 - (9*cosf(phi2/2.0 + phi2/2.0)*sinf(phi1/2.0 + phi2/2.0))/(800*pow((9/100.0 - (9*pow(sinf(phi1/2.0 + phi2/2.0),2))/400.0),0.5));
                                JT[1][1] = -0.5;

                                //Virtual spring dampener
                                f_r = ks_r*(r_fbk - r_cmd) + kd_r*(r_d_fbk - r_d_cmd);
                                f_theta = ks_theta*(theta_fbk - theta_cmd) + kd_theta*(theta_d_fbk - theta_d_cmd);

                                F[0] = f_r;
                                F[1] = f_theta;

                                Tau[0] = JT[0][0]*F[0] + JT[0][1]*F[1]; //
                                Tau[1] = JT[1][0]*F[0] + JT[1][1]*F[1]; //

                                //Motor 1 Control
                                I_cmd[0] = (1/Ki)*Tau[0];
                                //I_cmd[0] = 0.5*(f_r - 2.421)/1.127;
                                F2BM1.INT32 = I_cmd[0]*(pow(2.0,15)/60.0);
                                swap_int32( F2BM1.INT32 );
                                BaseCommandPTR = &BaseCommand[CONTROL_CURRENT_M1];
                                BaseCommandCompile(CONTROL_CURRENT_M1, 0b0011, 0x02, 0x45, 0x02, F2BM1.BYTE, 2, 0);
                                if(I_cmd[0] > 30 || I_cmd[0]<-30) {valid = 0; }
                                if(valid) {xQueueOverwrite( ICommandM1QHandle, &BaseCommandPTR); }

                                //Motor 2 Control
                                I_cmd[1] = -(1/Ki)*Tau[1];
                                //I_cmd[1] = 0.5*-(f_r - 2.421)/1.127;
                                F2BM2.INT32 = I_cmd[1]*(pow(2.0,15)/60.0);
                                swap_int32( F2BM2.INT32 );
                                BaseCommandPTR = &BaseCommand[CONTROL_CURRENT_M2];
                                BaseCommandCompile(CONTROL_CURRENT_M2, 0b0011, 0x02, 0x45, 0x02, F2BM2.BYTE, 2, 0);
                                if(I_cmd[1] > 30 || I_cmd[1]<-30) {valid = 0; }
                                if(valid) {xQueueOverwrite( ICommandM2QHandle, &BaseCommandPTR); }

                                //PC Logging
                                ControlLogPacket.I_cmd_0 = I_cmd[0];
                                ControlLogPacket.I_cmd_1 = I_cmd[1];
                                ControlLogPacket.f_r = f_r;
                                ControlLogPacket.f_theta = f_theta;
                                xQueueOverwrite(ProcessQControlHandle, &ControlLogPacketPTR);

                                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
                                xSemaphoreGive( TXMotorM1Handle );
                                xSemaphoreGive( TXMotorM2Handle );
                        }
                }

                //vTaskDelay(Ts);
        }
  /* USER CODE END StartController */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
        /* User can add his own implementation to report the HAL error return state */
        while(1)
        {
        }
  /* USER CODE END Error_Handler */ 
}

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
