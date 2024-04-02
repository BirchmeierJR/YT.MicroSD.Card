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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
	//@todo JRB Lenovo f_mkfs() fails at MULTI BLOCK WRITE when tmpWorkBuffer[] is increased to 1024 bytes. Hmmm?
	int8_t 	tmpWorkBuffer[512]; 		/* NOTE: Only used in f_mkfs() & f_checkdisk(); Work area, must be at least sector size or larger (larger is better for process time) */
	FATFS 	ramDiskFatFs;  				/* File system object for RAM disk logical drive */
	FIL 	MyFile;        				/* File object */
	TCHAR	volLabel[12];				// Fixed length of 11 + NULL terminator
} ramDiskItems_ST;
static ramDiskItems_ST rdi;				// **** CAUTION **** this is a large item, don't allocate on the stack.


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
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|MicroSD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin MicroSD_CS_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|MicroSD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// BEGIN: printf(): stdout override __io_putchar() to direct stdout to ITM => STM32CubeICE 'SWV ITM Data Console'
//
// The documentation for this is a bit scarce from a STM's perspective. I found much more information
// on an ARM site that discusses the SWV/ITM capabilities. It seems impressive, but I'm not sure how
// much the CubeIDE supports.
//
// Debug Exception & Monitor Control Register Base Address; see RM0368 section 23.10 "Core debug"
//
// NOTE:
// (a) It appears that my original clock frequency is required for this to work. When I switch
// from 84 MHz => 2 MHz ... printf() appears to stop working. Upon returning to 84 MHz, it starts
// working again.
// (b) Not sure of printf() is thread safe or not.

	#define DEMCR				*((volatile uint32_t*) 0xE000EDFCU)

	// ITM Register Address
	#define ITM_STIMULUS_PORT0	*((volatile uint32_t*) 0xE0000000)
	#define ITM_TRACE_EN		*((volatile uint32_t*) 0xE0000E00)

	int __io_putchar(int ch)
	{
		uint32_t	cnt=0;
		// Enable TRCENA
		DEMCR |= (1 << 24);
		// Enable Stimulus Port0
		ITM_TRACE_EN |= (1<<0);
		// Read FIFO Status in bit [0];
		while (!(ITM_STIMULUS_PORT0 & 1) && (cnt++ < 1000));
		// Write to ITM Stimulus Port0
		ITM_STIMULUS_PORT0 = ch;
		return (cnt < 1000) ? 1 : 0;
	}

// END: printf(): stdout override __io_putchar() to direct stdout to ITM => STM32CubeICE 'SWV ITM Data Console'


static FRESULT _mountMicroSDDrive(const TCHAR* path)
{
	// Once successfully mounted, the drive seems to work forever. However, it does not
	// always succeed on the first attempt after power up. I wonder if it is related to my
	// clock hi/low and edge settings?
	//
	// Until this is resolved, we'll allow up to to 10x tries for this operation.
	FRESULT	res;
	res = f_mount(&rdi.ramDiskFatFs, path, 0);
	if (FR_OK != res)
	{
		printf("%s.%d: Mount Failure (%d)\n", __FUNCTION__, __LINE__, res);
		for (;;)
		{
			osDelay(1000);
		}
	}
	else
	{


		if (FR_OK != res)
		{
			printf("%s.%d: Drive: '%s' f_getlabel() failed\n", __FUNCTION__, __LINE__, USERPath);
		}
	}
	if (FR_OK != res)
	{
		printf("%s.%d: Mount Failure (%d)\n", __FUNCTION__, __LINE__, res);
	}
	return res;
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

	FRESULT 	res;                            /* FatFs function common result code */
	uint32_t 	byteswritten, bytesread;       /* File write/read counts */
	uint8_t 	wtext[] 	= "Hello World!";
	FIL*		pFileHandle	= &rdi.MyFile;

	uint8_t	fDbgEn = 1;
	disk_ioctl(0, CTRL_DEBUG, &fDbgEn);	// Enable debug messages

	// Mount and make initial file system image.
	// NOTE:
	// - The use of 'FM_SFD' option is required for small file system to work, else the f_mkfs() fails.
	// - See user_diskio.c/h for low-level driver, which gets installed via function 'MX_FATFS_Init()'.
	//		The driver installation is complete prior to this function's invocation.
	res = _mountMicroSDDrive("0:/");

	// If mount fails, format drive ... CAUTION ... only for testing purposes. I'd guess many products
	// would not support a format option, leaving that to a PC or similar machine.
	if (FR_OK != res)
	{
		printf("%s.%d: Call f_mkfs()\n", __FUNCTION__, __LINE__);
		#if (_FATFS	== 68300)
		res = f_mkfs ((TCHAR const*)USERPath, FM_ANY | FM_SFD, 0, rdi.tmpWorkBuffer, sizeof(rdi.tmpWorkBuffer));	/* Create a FAT volume */
		#elif (FF_DEFINED	== 80286)
		MKFS_PARM	mkfs_parm =
		{
				.fmt	= FM_ANY | FM_SFD,
				.n_fat	= 2,
				.align	= 0,
				.n_root	= 1024,
				.au_size= 0,
		};
		res = f_mkfs ((TCHAR const*)USERPath, &mkfs_parm, rdi.tmpWorkBuffer, sizeof(rdi.tmpWorkBuffer));	/* Create a FAT volume */
		#endif
		if (FR_OK != res)
		{
			printf("%s.%d: f_mkfs() failed: result=%d\n", __FUNCTION__, __LINE__, res);
			for (;;)
			{
				osDelay(10000);
			}
		}
		printf("%s.%d: f_mkfs() Success\n", __FUNCTION__, __LINE__);
	}


	// Label the drive
	res = f_setlabel ("STM32F401");							/* Set volume label */
	if (FR_OK != res)
	{
		printf("%s.%d: f_setlabel() failed: result=%d\n", __FUNCTION__, __LINE__, res);
		for (;;)
		{
			osDelay(1000);
		}
	}

	// Get the label
	if (FR_OK == f_getlabel((TCHAR const*)USERPath, rdi.volLabel, NULL))
	{
		printf("%s.%d: Drive: '%s' is labeled '%s'\n", __FUNCTION__, __LINE__, USERPath, rdi.volLabel);
	}
	printf("%s%d: Drive '%s' Set Label\n", __FUNCTION__, __LINE__, USERPath);

	/* Open an pre-existing file, it if exists and print its contents */
	if(f_open(pFileHandle, "HWorld1.TXT", FA_READ) == FR_OK)
	{
		static uint32_t	loopCnt	= 0;
		res = f_read(pFileHandle, rdi.tmpWorkBuffer, sizeof(rdi.tmpWorkBuffer), (void *)&bytesread);
		if (bytesread > 0)
		{
			printf("%s.%d: %s %ld\n", __FUNCTION__, __LINE__, rdi.tmpWorkBuffer, loopCnt++);
		}
		/* Close the open text file */
		f_close(pFileHandle);
	}


	/* Create a file, same one just tried to open above, and write 'hello world' text */
	if(f_open(pFileHandle, "HWorld1.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
	{
		/* Write data to the text file */
		res = f_write(pFileHandle, wtext, sizeof(wtext), (void *)&byteswritten);
		f_sync(pFileHandle);
		f_close(pFileHandle);				/* Close the open text file */
	}


  /* Infinite loop */
  for(;;)
  {
    osDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END 5 */
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
