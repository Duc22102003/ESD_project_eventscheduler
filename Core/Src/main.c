/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include<time.h>
#include <SON_CDC_Logger.h>
#include <stdbool.h>
#include <SON_I2C_MPU6050.h>
#include <dht22.h>
#include "usbd_cdc_if.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float temp;
bool status1 = false;
bool status2 = false;
char usb_rx_buffer[64]; // Bộ đệm nhận USB
#define NUM_TASKS 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    uint32_t earliest_deadline;
    uint32_t end_cycle;
    uint32_t deadline;
    uint32_t period;
} TaskState;

TaskState tasks[NUM_TASKS] = {
    {0, 0, 100, 300},  // Temp task
    {0, 0, 50, 150},   // Acc task
};



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* Definitions for TempTask */
osThreadId_t TempTaskHandle;
const osThreadAttr_t TempTask_attributes = {
  .name = "TempTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for AccTask */
osThreadId_t AccTaskHandle;
const osThreadAttr_t AccTask_attributes = {
  .name = "AccTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* USER CODE BEGIN PV */


void CDC_ReceiveCallback(uint8_t* Buf, uint32_t Len) {
    // Copy dữ liệu nhận vào bộ đệm và thêm ký tự kết thúc chuỗi
    memcpy(usb_rx_buffer, Buf, Len);
    usb_rx_buffer[Len] = '\0'; // Đảm bảo chuỗi kết thúc
     if (strcmp(usb_rx_buffer, "start") == 0) {
            status1 = true; // Bắt đầu hệ thống
            status2 = true;

        } else if (strcmp(usb_rx_buffer, "stop") == 0) {
            status1 = false; // Dừng hệ thống
            status2 = false;
        }


}

uint32_t getCurrentTimeMs() {
    return HAL_GetTick();
}

void updateTaskDeadlines(uint32_t current_time) {
    for (int i = 0; i < NUM_TASKS; i++) {
        if (current_time >= tasks[i].end_cycle) {
            tasks[i].end_cycle += tasks[i].period;
            tasks[i].earliest_deadline = tasks[i].end_cycle - tasks[i].deadline;
        }
    }
}

int getNextTask() {
    int next_task = -1;
    uint32_t earliest_deadline = UINT32_MAX;

    for (int i = 0; i < NUM_TASKS; i++) {
        if (tasks[i].earliest_deadline < earliest_deadline) {
            earliest_deadline = tasks[i].earliest_deadline;
            next_task = i;
        }
    }
    return next_task;
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void GetTempTask(void *argument);
void GetAccTask(void *argument);

/* USER CODE BEGIN PFP */
void runEDF() {
    uint32_t current_time;
    while (1) {
        if (!status1) {
            osDelay(100); // Tạm dừng nếu hệ thống không hoạt động
            continue;
        }

        current_time = getCurrentTimeMs();
        updateTaskDeadlines(current_time);
        printf("Current Time: %lu ms\n", current_time); // In thời gian hiện tại
        for (int i = 0; i < NUM_TASKS; i++) {
                    printf("Task %d -> Deadline: %lu ms, End Cycle: %lu ms, Period: %lu ms\n",
                        i, tasks[i].earliest_deadline, tasks[i].end_cycle, tasks[i].period);
                }
        int next_task = getNextTask();
        if (next_task == 0) {
        	 printf("Next Task: GetTempTask\n"); // In ra thông báo về task tiếp theo
            GetTempTask(NULL);
        } else if (next_task == 1) {
        	 printf("Next Task: GetAccTask\n"); // In ra thông báo về task tiếp theo
            GetAccTask(NULL);
        }

        osDelay(1000); // Tránh sử dụng quá nhiều CPU
    }
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  dht22_init();
 MPU6050_Init();

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
  /* creation of TempTask */
  TempTaskHandle = osThreadNew(GetTempTask, NULL, &TempTask_attributes);

  /* creation of AccTask */
  AccTaskHandle = osThreadNew(GetAccTask, NULL, &AccTask_attributes);

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

  runEDF(); // Chạy EDF scheduler

      while (1) {}


       // Chạy bộ lập lịch để xử lý sự kiện đã thêm





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_GetTempTask */
/**
  * @brief  Function implementing the TempTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_GetTempTask */
void GetTempTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if (status1) {
		  DHT22_Get_Temp(&temp);

		  	  printf("t=%d\n", (int) temp);
	  }

    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_GetAccTask */
/**
* @brief Function implementing the AccTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GetAccTask */
void GetAccTask(void *argument)
{
  /* USER CODE BEGIN GetAccTask */
  /* Infinite loop */
  for(;;)
  {

	  if (status2) {
		  MPU6050_Read_Accel();
		    //	  intPart = (int)floorf(Ax * 100);
	  	  printf("ax=%c%d.%d\n", (Ax < 0) ? '-': '+',(int) Ax, abs( ((int)(Ax * 10000)) % 10000)  );
	  }

    osDelay(1000);
  }
  /* USER CODE END GetAccTask */
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
