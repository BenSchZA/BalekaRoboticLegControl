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
osMessageQId ProcessQM1Handle;
osMessageQId ProcessQM2Handle;
osMessageQId TransmitQHandle;
osMessageQId ProcessQiNemoHandle;
osMessageQId ProcessQPCHandle;
osMessageQId TransmitM1QHandle;
osMessageQId TransmitM2QHandle;
osMessageQId CurrentM1Handle;
osMessageQId CurrentM2Handle;
osSemaphoreId M1Handle;
osSemaphoreId M2Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t Ts = 5; //Sampling time in ms

uint8_t RXBufPC[50];
uint8_t RXBufM1[50];
uint8_t RXBufM2[50];

/* WHAT ARE ALL THE USARTS AND TIMERS USED FOR AND WHICH PINS DO THEY USE?
 *
 * USART1 = COMMS between the PC and STM	---		PB6 (TX)	 PB7 (RX)
 * USART2 = COMMS to Motor Controller 1		---		PA2 (TX)	 PA3 (RX)
 * USART3 = COMMS to Motor Controller 2		---		PC10 (TX)    PC11 (RX)
 * USART6 = COMMS to iNEMO					---		PC6 (TX)	 PC7 (RX)
 *
 * TIM1 = Timer for Center Boom Encoder		---		PE9 (A)	     PE11 (B)
 * TIM2 = Timer for Pitch Encoder			---		PA15 (A)	 PB3 (B)
 * TIM3 = Used for 5ms interrupts
 * TIM4 = Timer for Linear Encoder			---		PD12 (A)     PD13 (B)
 */

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

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

//Select Call-backs functions called after Transfer complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

//Heartbeat ##################################################################
#define TASK_TXM1        ( 1 << 0 )
#define TASK_TXM2        ( 1 << 1 )
#define TASK_RXM1        ( 1 << 2 )
#define TASK_RXM2        ( 1 << 3 )
#define TASK_HEARTBEAT   ( 1 << 4 )

////Message IDs
#define ID_KILL 0
#define ID_WRITE 1
#define ID_BRIDGE 2
#define ID_CURRENT_SET 3
#define ID_CURRENT_DATA 4
#define ID_POSITION_DATA 5
#define ID_VELOCITY_DATA 6

#define ALL_SYNC_BITS ( TASK_HEARTBEAT | TASK_TXM1 | TASK_TXM2 )

/* Declare a variable to hold the created event group. */
EventGroupHandle_t xEventSyncDriver;

////////////////////////////////////////////////////////////////////////

//Motor Packets
//'packed' makes sure compiler won't insert any gaps!
struct __attribute__((__packed__)) HeartPacketStruct {
        uint8_t KILL[14];
        uint8_t WRITE[14];
        uint8_t BRIDGE[14];
        uint8_t CURRENT_SET[14];
        uint8_t CURRENT_DATA[14];
        uint8_t POSITION_DATA[14];
        uint8_t VELOCITY_DATA[14];
};

//Same size packets allows access
struct HeartPacketStruct MotorPacket;
//Transmit pointer PCPacketPTR with sizeof(PCPacket)
uint8_t *MotorPacketPTR = (uint8_t*)&MotorPacket;

uint8_t KILL[12] = {0xA5, 0x3F, 0x06, 0x01, 0x00, 0x01, 0xCB, 0xB6, 0x01, 0x00, 0x33, 0x31};
uint8_t WRITE[12] = {0xA5, 0x3F, 0x0A, 0x07, 0x00, 0x01, 0x36, 0x24, 0x0F, 0x00, 0x10, 0x3E};
uint8_t BRIDGE[12] = {0xA5, 0x3F, 0x12, 0x01, 0x00, 0x01, 0x1A, 0xE0, 0x00, 0x00, 0x00, 0x00};
uint8_t CURRENT_SET[14] = {0xA5, 0x3F, 0x0E, 0x45, 0x00, 0x02, 0xBF, 0x7B, 0x48 /*Current[3]*/, 0x01 /*Current[2]*/, 0x00 /*Current[1]*/, 0x00 /*Current[0]*/, 0xDC, 0x6F}; //TODO Set Current 0
uint8_t CURRENT_DATA[8] = {0xA5, 0x3F, 0x31, 0x10, 0x03, 0x01, 0x97, 0x72};
uint8_t POSITION_DATA[8] = {0xA5, 0x3F, 0x3D, 0x12, 0x00, 0x02, 0xD3, 0x10};
uint8_t VELOCITY_DATA[8] = {0xA5, 0x3F, 0x15, 0x11, 0x02, 0x02, 0x5E, 0xAF};

uint8_t GAIN_0[12] = {0xA5, 0x3F, 0x02, 0x01, 0x01, 0x02, 0x02, 0x15, 0x00, 0x00, 0x00};
uint8_t GAIN_1[12] = {0xA5, 0x3F, 0x02, 0x01, 0x01, 0x02, 0x02, 0x15, 0x01, 0x10, 0x21};
uint8_t ZERO_SET[12] = {0xA5, 0x3F, 0x02, 0x01, 0x00, 0x02, 0x31, 0x24, 0x01, 0x10, 0x21};

uint8_t *MotorPacketArray[7] = {KILL,WRITE,BRIDGE,CURRENT_SET,CURRENT_DATA,POSITION_DATA,VELOCITY_DATA};

struct __attribute__((__packed__)) MotorPacketSize {
        uint8_t KILL;
        uint8_t WRITE;
        uint8_t BRIDGE;
        uint8_t CURRENT_SET;
        uint8_t CURRENT_DATA;
        uint8_t POSITION_DATA;
        uint8_t VELOCITY_DATA;
};

struct MotorPacketSize MotorPacketSize;
//Transmit pointer PCPacketPTR with sizeof(PCPacket)
uint8_t *MotorPacketSizePTR = (uint8_t*)&MotorPacketSize;

uint8_t KILL_L = 12;
uint8_t WRITE_L = 12;
uint8_t BRIDGE_L = 12;
uint8_t CURRENT_SET_L = 14;
uint8_t CURRENT_DATA_L = 8;
uint8_t POSITION_DATA_L = 8;
uint8_t VELOCITY_DATA_L = 8;

////////////////////////////////////////////////////////////////////////

//Message from the "heart"
struct HeartMessage
{
        uint8_t ucMessageID;
        uint8_t *ucData;
};

struct HeartMessage xHeartM1;
struct HeartMessage xHeartM2;

////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////

struct __attribute__((__packed__)) CurrentCommandStruct {
        uint8_t START[2];
        uint8_t CB;
        uint8_t INDOFF[2];
        uint8_t LEN;
        uint8_t CRC1[2];
        uint8_t DATA[4];
        uint8_t CRC2[2];
};

struct CurrentCommandStruct CurrentCommandM1;
struct CurrentCommandStruct CurrentCommandM2;
uint8_t *CurrentCommandM1PTR = (uint8_t*)&CurrentCommandM1;
uint8_t *CurrentCommandM2PTR = (uint8_t*)&CurrentCommandM2;

////////////////////////////////////////////////////////////////////////

struct __attribute__((__packed__)) PositionCommandStruct {
        uint8_t START[2];
        uint8_t CB;
        uint8_t INDOFF[2];
        uint8_t LEN;
        uint8_t CRC1[2];
        uint8_t DATA[4];
        uint8_t CRC2[2];
};

struct PositionCommandStruct PositionCommandM1;
struct PositionCommandStruct PositionCommandM2;
uint8_t *PositionCommandM1PTR = (uint8_t*)&PositionCommandM1;
uint8_t *PositionCommandM2PTR = (uint8_t*)&PositionCommandM2;

////////////////////////////////////////////////////////////////////////
//(aligned(1),
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

struct BaseCommandStruct BaseCommand[30];
struct BaseCommandStruct* BaseCommandPTR;

union {
        uint32_t WORD;
        uint16_t HALFWORD;
        uint8_t BYTE[4];
} WORDtoBYTEBase;

uint32_t CALC_CRCBase;

////////////////////////////////////////////////////////////////////////


SemaphoreHandle_t xSemaphoreM1 = NULL;
SemaphoreHandle_t xSemaphoreM2 = NULL;

SemaphoreHandle_t PCRXHandle;

SemaphoreHandle_t TXMotorM1Handle;
SemaphoreHandle_t TXMotorM2Handle;
SemaphoreHandle_t RXMotorM1Handle;
SemaphoreHandle_t RXMotorM2Handle;


//#############################################################################

//Motor functions
void SetupMotors(void);
void SetupArrays(void);
void SetupBinarySemaphores(void);

void BaseCommandCompile(uint8_t n, uint8_t CB, uint8_t INDOFF1, uint8_t INDOFF2, uint8_t *DATA, uint8_t LEN, uint8_t SNIP);
void ClearBuf(uint8_t buf[], uint8_t size); //Set buffer contents to 0
void SetBuf(uint8_t buf[], uint8_t set[], uint8_t size); //Set buf[] to set[]
void SetBytes(uint8_t buf[], uint8_t pos1, uint8_t val1, uint8_t pos2, uint8_t val2, uint8_t pos3, uint8_t val3, uint8_t pos4, uint8_t val4); //For setting current commands, set pos to NULL to omit
void TransmitM1_DMA(uint8_t *data, uint8_t size);
void ReceiveM1_DMA(uint8_t *data, uint8_t size);
void TransmitM2_DMA(uint8_t *data, uint8_t size);
void ReceiveM2_DMA(uint8_t *data, uint8_t size);

void SetupCircularBuffers(void);
void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart);
void HAL_UART_EndDMA_RX(UART_HandleTypeDef *huart);

uint8_t count;

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
        SetupArrays();
        SetupMotors(); //NOTE! For some reason calling this function after SetupBinarySemaphores stops UART from initialising properly...
        SetupBinarySemaphores();
        SetupCircularBuffers();
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
        osThreadDef(defaultTask, StartDefaultTask, osPriorityRealtime, 0, 128);
        defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

        /* definition and creation of TXPC */
        osThreadDef(TXPC, StartTXPC, osPriorityRealtime, 0, 128);
        TXPCHandle = osThreadCreate(osThread(TXPC), NULL);

        /* definition and creation of RXPC */
        osThreadDef(RXPC, StartRXPC, osPriorityRealtime, 0, 300);
        RXPCHandle = osThreadCreate(osThread(RXPC), NULL);

        /* definition and creation of Heartbeat */
        osThreadDef(Heartbeat, StartHeartbeat, osPriorityRealtime, 0, 128);
        HeartbeatHandle = osThreadCreate(osThread(Heartbeat), NULL);

        /* definition and creation of TXMotor1 */
        osThreadDef(TXMotor1, StartTXMotor1, osPriorityAboveNormal, 0, 128);
        TXMotor1Handle = osThreadCreate(osThread(TXMotor1), NULL);

        /* definition and creation of TXMotor2 */
        osThreadDef(TXMotor2, StartTXMotor2, osPriorityAboveNormal, 0, 128);
        TXMotor2Handle = osThreadCreate(osThread(TXMotor2), NULL);

        /* definition and creation of RXMotor1 */
        osThreadDef(RXMotor1, StartRXMotor1, osPriorityHigh, 0, 128);
        RXMotor1Handle = osThreadCreate(osThread(RXMotor1), NULL);

        /* definition and creation of RXMotor2 */
        osThreadDef(RXMotor2, StartRXMotor2, osPriorityHigh, 0, 128);
        RXMotor2Handle = osThreadCreate(osThread(RXMotor2), NULL);

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

        /* definition and creation of TransmitQ */
        osMessageQDef(TransmitQ, 20, uint32_t);
        TransmitQHandle = osMessageCreate(osMessageQ(TransmitQ), NULL);

        /* definition and creation of ProcessQiNemo */
        osMessageQDef(ProcessQiNemo, 1, uint32_t);
        ProcessQiNemoHandle = osMessageCreate(osMessageQ(ProcessQiNemo), NULL);

        /* definition and creation of ProcessQPC */
        osMessageQDef(ProcessQPC, 1, uint32_t);
        ProcessQPCHandle = osMessageCreate(osMessageQ(ProcessQPC), NULL);

        /* definition and creation of TransmitM1Q */
        osMessageQDef(TransmitM1Q, 10, uint32_t);
        TransmitM1QHandle = osMessageCreate(osMessageQ(TransmitM1Q), NULL);

        /* definition and creation of TransmitM2Q */
        osMessageQDef(TransmitM2Q, 10, uint32_t);
        TransmitM2QHandle = osMessageCreate(osMessageQ(TransmitM2Q), NULL);

        /* definition and creation of CurrentM1 */
        osMessageQDef(CurrentM1, 1, uint32_t);
        CurrentM1Handle = osMessageCreate(osMessageQ(CurrentM1), NULL);

        /* definition and creation of CurrentM2 */
        osMessageQDef(CurrentM2, 1, uint32_t);
        CurrentM2Handle = osMessageCreate(osMessageQ(CurrentM2), NULL);

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
        huart4.Init.BaudRate = 115200;
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

void SetupDMA(UART_HandleTypeDef *huart){
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

void SetupMotors(void){
        //Initialize Motors
//        HAL_Delay(1000);
//        TransmitM1_DMA(WRITE, sizeof(WRITE));
//        while(huart2.gState != HAL_UART_STATE_READY);
//        HAL_Delay(100);
//        TransmitM2_DMA(WRITE, sizeof(WRITE));
//        while(huart3.gState != HAL_UART_STATE_READY);
//        HAL_Delay(100);
//        TransmitM1_DMA(BRIDGE, sizeof(BRIDGE));
//        while(huart2.gState != HAL_UART_STATE_READY);
//        HAL_Delay(100);
//        TransmitM2_DMA(BRIDGE, sizeof(BRIDGE));
//        while(huart3.gState != HAL_UART_STATE_READY);
//        HAL_Delay(100);
}

void SetupBinarySemaphores(void){
        PCRXHandle = xSemaphoreCreateBinary();

        TXMotorM1Handle = xSemaphoreCreateBinary();
        TXMotorM2Handle = xSemaphoreCreateBinary();
        RXMotorM1Handle = xSemaphoreCreateBinary();
        RXMotorM2Handle = xSemaphoreCreateBinary();
}

void SetupArrays(void){
        //Initialize arrays
        memcpy(MotorPacket.KILL, KILL, sizeof(KILL));
        memcpy(MotorPacket.WRITE, WRITE, sizeof(WRITE));
        memcpy(MotorPacket.BRIDGE, BRIDGE, sizeof(BRIDGE));
        memcpy(MotorPacket.CURRENT_SET, CURRENT_SET, sizeof(CURRENT_SET));
        memcpy(MotorPacket.CURRENT_DATA, CURRENT_DATA, sizeof(CURRENT_DATA));
        memcpy(MotorPacket.POSITION_DATA, POSITION_DATA, sizeof(POSITION_DATA));
        memcpy(MotorPacket.VELOCITY_DATA, VELOCITY_DATA, sizeof(VELOCITY_DATA));

        MotorPacketSize.KILL = KILL_L;
        MotorPacketSize.WRITE = WRITE_L;
        MotorPacketSize.BRIDGE = BRIDGE_L;
        MotorPacketSize.CURRENT_SET = CURRENT_SET_L;
        MotorPacketSize.CURRENT_DATA = CURRENT_DATA_L;
        MotorPacketSize.POSITION_DATA = POSITION_DATA_L;
        MotorPacketSize.VELOCITY_DATA = VELOCITY_DATA_L;
}

void BaseCommandCompile(uint8_t n, uint8_t CB, uint8_t INDOFF1, uint8_t INDOFF2, uint8_t *DATA, uint8_t LEN, uint8_t SNIP_LEN){
        memset(&BaseCommand[n], 0, sizeof(BaseCommand[0]));

        BaseCommand[n].START[0] = 0xA5;
        BaseCommand[n].START[1] = 0x3F;
        BaseCommand[n].CB = CB;
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
}

void ClearBuf(uint8_t buf[], uint8_t size){
        uint8_t i;
        for (i = 0; i < size; i++) {
                buf[i] = 0;
        }
}

void SetBuf(uint8_t buf[], uint8_t set[], uint8_t size){
        //ClearBuf(buf, 14);
        uint8_t i;
        for (i = 0; i < size; i++) {
                buf[i] = set[i];
        }
}

void SetBytes(uint8_t buf[], uint8_t pos1, uint8_t val1, uint8_t pos2, uint8_t val2, uint8_t pos3, uint8_t val3, uint8_t pos4, uint8_t val4){
        if (pos1 != -1) {buf[pos1] = buf[val1]; }
        if (pos2 != -1) {buf[pos2] = buf[val2]; }
        if (pos3 != -1) {buf[pos3] = buf[val3]; }
        if (pos4 != -1) {buf[pos4] = buf[val4]; }
}

void TransmitM1_DMA(uint8_t *data, uint8_t size){
        /* Start the transmission - an interrupt is generated when the transmission
           is complete. */
        //if(HAL_UART_Transmit_DMA(&huart2, data, size) != HAL_OK) { Error_Handler(); }
        HAL_UART_Transmit_DMA(&huart2, data, size);
}

void ReceiveM1_DMA(uint8_t *data, uint8_t size){
        HAL_UART_Receive_DMA(&huart2, data, size);
        //HAL_UART_Receive(&huart2, data, size, 5);
}

void TransmitM2_DMA(uint8_t *data, uint8_t size){
        /* Start the transmission - an interrupt is generated when the transmission
           is complete. */
        //if(HAL_UART_Transmit_DMA(&huart3, data, size) != HAL_OK) { Error_Handler(); }
        HAL_UART_Transmit_DMA(&huart3, data, size);
}

void ReceiveM2_DMA(uint8_t *data, uint8_t size){
        HAL_UART_Receive_DMA(&huart3, data, size);
        //HAL_UART_Receive(&huart3, data, size, 5);
}

void SetupCircularBuffers(void){

        //HAL_UART_Receive_DMA(&huart2, RXBufM1, 50);
        HAL_Delay(100);
        //HAL_UART_Receive_DMA(&huart3, RXBufM2, 50);
        HAL_Delay(100);

}

//Select Call-backs functions called after Transfer complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
        __NOP();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
        __NOP();
}

//http://www.riuson.com/blog/post/stm32-hal-uart-dma-rx-variable-length
void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart){
        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;

        if(huart->Instance == UART4) {
                __HAL_USART_CLEAR_IDLEFLAG(huart);
                xSemaphoreGiveFromISR( PCRXHandle, &xHigherPriorityTaskWoken );
        }

//        if(huart->Instance == USART2) {
//                __HAL_USART_CLEAR_IDLEFLAG(huart);
//                xSemaphoreGiveFromISR( RXMotorM1Handle, &xHigherPriorityTaskWoken );
//        }
//
//        if(huart->Instance == USART3) {
//                __HAL_USART_CLEAR_IDLEFLAG(huart);
//                xSemaphoreGiveFromISR( RXMotorM2Handle, &xHigherPriorityTaskWoken );
//        }

        /* If xHigherPriorityTaskWoken was set to true you
           we should yield.  The actual macro used here is
           port specific. portYIELD_FROM_ISR */
        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
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

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

        /* USER CODE BEGIN 5 */
        //vTaskSuspend( NULL );
        /* Infinite loop */
        for(;; )
        {
                osDelay(500);
        }
        /* USER CODE END 5 */
}

/* StartTXPC function */
void StartTXPC(void const * argument)
{
        /* USER CODE BEGIN StartTXPC */

        //Packet Protocol ############################################################
        //'packet' makes sure compiler won't insert any gaps!
        //To PC
        struct __attribute__((__packed__)) TXPacketStruct {
                uint8_t START[2];

                uint8_t LENGTH;

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
                uint8_t StatBIT_1 : 1;
                uint8_t StatBIT_2 : 1;
                uint8_t StatBIT_3 : 1;
                uint8_t StatBIT_4 : 1;
                uint8_t StatBIT_5 : 1;
                uint8_t StatBIT_6 : 1;
                uint8_t StatBIT_7 : 1;
                uint8_t StatBIT_8 : 1;

                uint8_t CRCCheck;

                uint8_t STOP[2];
        };

        struct TXPacketStruct PCPacket;
        //Transmit pointer PCPacketPTR with sizeof(PCPacket)
        uint8_t *PCPacketPTR = (uint8_t*)&PCPacket;

        memset(PCPacketPTR, 0, sizeof(PCPacket));

        PCPacket.START[0] = 0x7E;
        PCPacket.START[1] = 0x5B;

        PCPacket.STOP[0] = 0x5B;
        PCPacket.STOP[1] = 0x7E;

        //Example Usage
        //PCPacket.ACCX[0] = 0xAA;
        //PCPacket.STOP[0] = 0xAA;
        //PCPacket.STOP[1] = 0xAA;
        //PCPacket.StatBIT_2 = 1;
        //PCPacket.StatBIT_8 = 1;
        //sizeof(PCPacket);
        //PCPacketPTR[n];

        uint8_t *pxRxedM1Message;
        uint8_t *pxRxedM2Message;

        /* Infinite loop */
        for(;; )
        {
            		memset(&PCPacket.M1C, 0, 34);

                	if(xQueueReceive( ProcessQM1Handle, &pxRxedM1Message, portMAX_DELAY )){
                	memcpy(PCPacket.M1C, pxRxedM1Message, 10);
                	}
                	if(xQueueReceive( ProcessQM2Handle, &pxRxedM2Message, portMAX_DELAY )){
                        memcpy(PCPacket.M2C, pxRxedM2Message, 10);
                	}
                	HAL_UART_Transmit_DMA(&huart4, PCPacketPTR, sizeof(PCPacket)); //TODO
                	while(huart4.gState != HAL_UART_STATE_READY);
                	osDelay(5);
                        //PCPacket.CRCCheck = ; //TODO

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


        uint8_t START_INDEX = 0;
        uint8_t START_SIZE = sizeof(RXPacket.START);
        uint8_t STOP_INDEX = 0;
        uint8_t STOP_SIZE = sizeof(RXPacket.STOP);
        uint8_t *DATA_SIZE;
        uint8_t *EXTRACT_DATA;
        uint8_t *EXTRACT_CRC;
        uint8_t CRC_SIZE = sizeof(RXPacket.CRCCheck);
        uint32_t CALC_CRC;

        ////////////////////////////////////////////////////////////////////////

        uint8_t KILL_BRIDGE_DATA[2] = {0x01, 0x00};
        uint8_t WRITE_ENABLE_DATA[2] = {0x0F, 0x00};
        uint8_t BRIDGE_ENABLE_DATA[2] = {0x00, 0x00};
        uint8_t GAIN_SET_0_DATA[4] = {0x00, 0x00, 0x00, 0x00};
        uint8_t GAIN_SET_1_DATA[4] = {0x08, 0x00, 0x00, 0x00};
        uint8_t ZERO_POSITION_DATA[4] = {0x08, 0x00, 0x00, 0x00};

        ////////////////////////////////////////////////////////////////////////

        union {
                uint32_t WORD;
                uint16_t HALFWORD;
                uint8_t BYTE[4];
        } WORDtoBYTE;

        //uint8_t temp[24] = {0x7E,0x5B,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB5,0xE8,0x5D,0x7E};

        uint8_t rcvdCount;
        __HAL_USART_CLEAR_IDLEFLAG(&huart4);
        __HAL_USART_ENABLE_IT(&huart4, USART_IT_IDLE);
        HAL_Delay(100);
        /* Infinite loop */
        for(;; )
        {
                HAL_UART_Receive_DMA(&huart4, RXBufPC, 50);
                xSemaphoreTake( PCRXHandle, portMAX_DELAY );

                rcvdCount = sizeof(RXBufPC) - huart4.hdmarx->Instance->NDTR;
                HAL_UART_EndDMA_RX(&huart4);

                if(findBytes(RXBufPC, rcvdCount, RXPacket.START, 2, 1) > 0){
                START_INDEX = findBytes(RXBufPC, rcvdCount, RXPacket.START, 2, 1);
                memcpy(RXPacketPTR, &RXBufPC[START_INDEX], RXPacketLen);

                WORDtoBYTE.BYTE[1] = RXPacket.CRCCheck[0];
                WORDtoBYTE.BYTE[0] = RXPacket.CRCCheck[1];
                CALC_CRC = crcCalc(&RXPacket.OPCODE, 0, 18, 0); //Check entire data CRC

                if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                        switch(RXPacket.OPCODE) {
                        case 0: //Kill Bridge
                                BaseCommandCompile(RXPacket.OPCODE, 0x06, 0x01, 0x00, KILL_BRIDGE_DATA, 1, 2);
                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                break;
                        case 1: //Write Enable
                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                BaseCommandCompile(RXPacket.OPCODE, 0x0A, 0x07, 0x00, WRITE_ENABLE_DATA, 1, 2);
                                xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                break;
                        case 2: //Enable Bridge
                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                BaseCommandCompile(RXPacket.OPCODE, 0x12, 0x01, 0x00, BRIDGE_ENABLE_DATA, 1, 2);
                                xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                break;
                        case 20: //Current Command
                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                BaseCommandCompile(RXPacket.OPCODE, 0x0E, 0x45, 0x00, RXPacket.M1C, 2, 0);
                                xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE+1];
                                BaseCommandCompile(RXPacket.OPCODE+1, 0x0E, 0x45, 0x00, RXPacket.M2C, 2, 0);
                                xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                break;
                        case 22: //Position Command
                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                BaseCommandCompile(RXPacket.OPCODE, 0x2A, 0x45, 0x00, RXPacket.M1P, 2, 0);
                                xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE+1];
                                BaseCommandCompile(RXPacket.OPCODE+1, 0x2A, 0x45, 0x00, RXPacket.M2P, 2, 0);
                                xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                break;
                        case 8: //Zero Position
                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                BaseCommandCompile(RXPacket.OPCODE, 0x02, 0x01, 0x00, ZERO_POSITION_DATA, 2, 0);
                                xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                break;
                        case 9: //Gain Set
                                if(RXPacket.StatBIT_1 == 0) {
                                        BaseCommandCompile(RXPacket.OPCODE, 0x02, 0x01, 0x01, GAIN_SET_0_DATA, 2, 0);
                                }
                                else{
                                        BaseCommandCompile(RXPacket.OPCODE, 0x02, 0x01, 0x01, GAIN_SET_1_DATA, 2, 0);
                                }
                                BaseCommandPTR = &BaseCommand[RXPacket.OPCODE];
                                xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                                break;
                        default:
                                break;
                        }
                }
                }

                        //Read Current
                        BaseCommandCompile(5, 0x31, 0x10, 0x03, NULL, 1, 4);
                        BaseCommandPTR = &BaseCommand[5];
                        xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                        xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                        //Read Position
                        BaseCommandCompile(6, 0x3D, 0x12, 0x00, NULL, 2, 4);
                        BaseCommandPTR = &BaseCommand[6];
                        xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                        xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                        //Read Velocity
                        BaseCommandCompile(7, 0x15, 0x11, 0x02, NULL, 2, 4);
                        BaseCommandPTR = &BaseCommand[7];
                        xQueueSendToBack( TransmitM1QHandle, &BaseCommandPTR, ( TickType_t ) 5);
                        xQueueSendToBack( TransmitM2QHandle, &BaseCommandPTR, ( TickType_t ) 5);

                        xSemaphoreGive( TXMotorM1Handle );
                        xSemaphoreGive( TXMotorM2Handle );


        }
        /* USER CODE END StartRXPC */
}

/* StartHeartbeat function */
void StartHeartbeat(void const * argument)
{
        /* USER CODE BEGIN StartHeartbeat */
        //http://www.freertos.org/xEventGroupSync.html
        //For reference: ALL_SYNC_BITS ( TASK_HEARTBEAT | TASK_TXM1 | TASK_TXM2 | TASK_RXM1 | TASK_RXM2 )
        //osMessageQId TransmitM1QHandle;
        //osMessageQId TransmitM2QHandle;
        //Message from the "heart"
        //	struct HeartMessage
        //	 {
        //		  uint8_t ucMessageID;
        //	    uint8_t ucMessageLen;
        //	    uint8_t ucData[ucMessageLen];
        //	 };
        //	struct HeartMessage xHeartM1;
        //	struct HeartMessage xHeartM2;
        //
        //#define KILL (1 << 0)
        //#define WRITE (1 << 1)
        //#define BRIDGE (1 << 2)
        //#define CURRENT_SET (1 << 3)
        //#define CURRENT_DATA (1 << 4)
        //#define POSITION_DATA (1 << 5)
        //#define VELOCITY_DATA (1 << 6)

        EventBits_t uxReturn;
        //TickType_t xTicksToWait = 5 / portTICK_PERIOD_MS;
        TickType_t xTicksToWait = portMAX_DELAY;

        uint8_t *pxMessage;

        uint8_t ID = 4;

        /* Infinite loop */
        for(;; )
        {
                vTaskSuspend(NULL);
                /* Perform task functionality here. */
                pxMessage = MotorPacketArray[ID];

                xQueueSendToBack( TransmitM1QHandle, &pxMessage, ( TickType_t ) 5);
                xQueueSendToBack( TransmitM2QHandle, &pxMessage, ( TickType_t ) 5);

                ID++;
                if(ID==7)
                {
                        ID = 4;         //Start with motor current read
                        osDelay(5);
                }
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
                HAL_UART_Receive_DMA(&huart2, RXBufM1, 50);
                while(uxQueueMessagesWaiting( TransmitM1QHandle )) {
                        // Receive a message on the created queue. Block 5.
                        xQueueReceive( TransmitM1QHandle, &( pxRxedMessage ), portMAX_DELAY);
                        TransmitM1_DMA(pxRxedMessage, sizeof(BaseCommand[0]) - SNIP - (SNIP==4)*2); //TODO sizing??
                        while(huart2.gState != HAL_UART_STATE_READY);
                        osDelay(1);
                }
                osDelay(1);
                HAL_UART_EndDMA_RX(&huart2);
                xSemaphoreGive( RXMotorM1Handle );
        }
        /* USER CODE END StartTXMotor1 */
}

/* StartTXMotor2 function */
void StartTXMotor2(void const * argument)
{
        /* USER CODE BEGIN StartTXMotor2 */
        uint8_t *pxRxedMessage;
        /* Infinite loop */
        for(;; )
        {
                xSemaphoreTake( TXMotorM2Handle, portMAX_DELAY );
                HAL_UART_Receive_DMA(&huart3, RXBufM2, 50);
                while(uxQueueMessagesWaiting( TransmitM2QHandle )) {
                        // Receive a message on the created queue. Block 5.
                        xQueueReceive( TransmitM2QHandle, &( pxRxedMessage ), portMAX_DELAY);
                        TransmitM2_DMA(pxRxedMessage, sizeof(BaseCommand[0]) - SNIP - (SNIP==4)*2);
                        //xSemaphoreGive( RXMotorM2Handle );
                        while(huart3.gState != HAL_UART_STATE_READY);
                        osDelay(1);
                }
                osDelay(1);
                HAL_UART_EndDMA_RX(&huart3);
                xSemaphoreGive( RXMotorM2Handle );
        }
        /* USER CODE END StartTXMotor2 */
}

/* StartRXMotor1 function */
void StartRXMotor1(void const * argument)
{
        /* USER CODE BEGIN StartRXMotor1 */
        uint8_t OPCODE;
        uint8_t BUFFER_TO_CHECK[50];
        uint8_t BUFF_SIZE;
        uint8_t START_BYTE[2] = {0xA5, 0xFF};
        uint8_t START_SIZE = 2;
        uint8_t DATA_SIZE;
        uint8_t CRC_SIZE = 2;
        uint8_t START_INDEX;
        uint8_t *EXTRACT_CRC;
        uint32_t CALC_CRC;

        uint8_t INDEX_SIZE = 5;
        uint8_t INDEX[5] = {0,0,0,0,0};

        uint8_t i;
        union {
                uint32_t WORD;
                uint16_t HALFWORD;
                uint8_t BYTE[4];
        } WORDtoBYTE;

        uint8_t PCBufM1[10];
        uint8_t Data1 = 0;
        uint8_t Data2 = 0;
        uint8_t Data3 = 0;

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

                rcvdCount = sizeof(RXBufM1) - huart2.hdmarx->Instance->NDTR;
                findMultipleBytes(RXBufM1, rcvdCount, START_BYTE, 2, INDEX);

                for(int i=0; i<INDEX_SIZE; i++){
                OPCODE = (RXBufM1[INDEX[i]+2] & 0b00111100)>>2;
                START_INDEX = INDEX[i];
                switch(OPCODE)
                {
                case 0b0011:                 //Current_Set
                        break;
                case 0b1100:                 //Current_Data
                        BUFF_SIZE = 12;
                        DATA_SIZE = 2;
                        memcpy(CURRENTrxPTR, &RXBufM1[START_INDEX], rcvdCount);
                        WORDtoBYTE.BYTE[1] = CURRENTrx.CRC2[0];
                        WORDtoBYTE.BYTE[0] = CURRENTrx.CRC2[1];
                        CALC_CRC = crcCalc(CURRENTrx.DATA, 0, DATA_SIZE, 1);
                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                appendBytes(PCBufM1, 10, 0, CURRENTrx.DATA, 0, DATA_SIZE);
                                Data1 = 1;
                        }
                        break;
                case 0b1111:                 //Position_Data
                        BUFF_SIZE = 14;
                        DATA_SIZE = 4;
                        memcpy(POSITIONrxPTR, &RXBufM1[START_INDEX], rcvdCount);
                        WORDtoBYTE.BYTE[1] = POSITIONrx.CRC2[0];
                        WORDtoBYTE.BYTE[0] = POSITIONrx.CRC2[1];
                        CALC_CRC = crcCalc(POSITIONrx.DATA, 0, DATA_SIZE, 1);
                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                appendBytes(PCBufM1, 10, 2, POSITIONrx.DATA, 0, DATA_SIZE);
                                Data2 = 1;
                        }
                        break;
                case 0b0101:                 //Velocity_Data
                        BUFF_SIZE = 14;
                        DATA_SIZE = 4;
                        memcpy(VELOCITYrxPTR, &RXBufM1[START_INDEX], rcvdCount);
                        WORDtoBYTE.BYTE[1] = VELOCITYrx.CRC2[0];
                        WORDtoBYTE.BYTE[0] = VELOCITYrx.CRC2[1];
                        CALC_CRC = crcCalc(VELOCITYrx.DATA, 0, DATA_SIZE, 1);
                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                appendBytes(PCBufM1, 10, 2 + 4, VELOCITYrx.DATA, 0, DATA_SIZE);
                                Data3 = 1;
                        }
                        break;
                default:
                        break;
                }
                }

                if(Data1 || Data2 || Data3) {
                        xQueueSend( ProcessQM1Handle, &PCBufM1, ( TickType_t ) 5 );
                        Data1 = 0;
                        Data2 = 0;
                        Data3 = 0;
                        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
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
        uint8_t OPCODE;
        uint8_t BUFFER_TO_CHECK[50];
        uint8_t BUFF_SIZE;
        uint8_t START_BYTE[2] = {0xA5, 0xFF};
        uint8_t START_SIZE = 2;
        uint8_t DATA_SIZE;
        uint8_t CRC_SIZE = 2;
        uint8_t START_INDEX;
        uint8_t *EXTRACT_CRC;
        uint32_t CALC_CRC;

        uint8_t INDEX_SIZE = 5;
        uint8_t INDEX[5] = {0,0,0,0,0};

        uint8_t i;
        union {
                uint32_t WORD;
                uint16_t HALFWORD;
                uint8_t BYTE[4];
        } WORDtoBYTE;

        uint8_t PCBufM2[10];
        uint8_t Data1 = 0;
        uint8_t Data2 = 0;
        uint8_t Data3 = 0;

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

				rcvdCount = sizeof(RXBufM2) - huart3.hdmarx->Instance->NDTR;
				findMultipleBytes(RXBufM2, rcvdCount, START_BYTE, 2, INDEX);

				for(int i=0; i<INDEX_SIZE; i++){
				OPCODE = (RXBufM2[INDEX[i]+2] & 0b00111100)>>2;
				START_INDEX = INDEX[i];
                switch(OPCODE)
                {
                case 0b0011:                 //Current_Set
                        break;
                case 0b1100:                 //Current_Data
                        BUFF_SIZE = 12;
                        DATA_SIZE = 2;
                        memcpy(CURRENTrxPTR, &RXBufM2[START_INDEX], rcvdCount);
                        WORDtoBYTE.BYTE[1] = CURRENTrx.CRC2[0];
                        WORDtoBYTE.BYTE[0] = CURRENTrx.CRC2[1];
                        CALC_CRC = crcCalc(CURRENTrx.DATA, 0, DATA_SIZE, 1);
                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                appendBytes(PCBufM2, 10, 0, CURRENTrx.DATA, 0, DATA_SIZE);
                                Data1 = 1;
                        }
                        break;
                case 0b1111:                 //Position_Data
                        BUFF_SIZE = 14;
                        DATA_SIZE = 4;
                        memcpy(POSITIONrxPTR, &RXBufM2[START_INDEX], rcvdCount);
                        WORDtoBYTE.BYTE[1] = POSITIONrx.CRC2[0];
                        WORDtoBYTE.BYTE[0] = POSITIONrx.CRC2[1];
                        CALC_CRC = crcCalc(POSITIONrx.DATA, 0, DATA_SIZE, 1);
                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                appendBytes(PCBufM2, 10, 2, POSITIONrx.DATA, 0, DATA_SIZE);
                                Data2 = 1;
                        }
                        break;
                case 0b0101:                 //Velocity_Data
                        BUFF_SIZE = 14;
                        DATA_SIZE = 4;
                        memcpy(VELOCITYrxPTR, &RXBufM2[START_INDEX], rcvdCount);
                        WORDtoBYTE.BYTE[1] = VELOCITYrx.CRC2[0];
                        WORDtoBYTE.BYTE[0] = VELOCITYrx.CRC2[1];
                        CALC_CRC = crcCalc(VELOCITYrx.DATA, 0, DATA_SIZE, 1);
                        if(WORDtoBYTE.HALFWORD==CALC_CRC) {
                                appendBytes(PCBufM2, 10, 2 + 4, VELOCITYrx.DATA, 0, DATA_SIZE);
                                Data3 = 1;
                        }
                        break;
                default:
                        break;
                }
				}

                if(Data1 || Data2 || Data3) {
                        xQueueSend( ProcessQM2Handle, &PCBufM2, ( TickType_t ) 5 );
                        Data1 = 0;
                        Data2 = 0;
                        Data3 = 0;
                }
        }
        /* USER CODE END StartRXMotor2 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
        if (htim->Instance == TIM1) {
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
