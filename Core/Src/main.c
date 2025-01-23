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
typedef enum {
    TASK_1_EVENT,
    TASK_2_EVENT
} Event;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float temp;
bool status = false;
char usb_rx_buffer[64]; // Bộ đệm nhận USB
float Ax;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_EVENTS 100
Event eventQueue[MAX_EVENTS];
int front = -1, rear = -1;

// Hàm kiểm tra hàng đợi rỗng
bool isQueueEmpty() {
    return front == -1;
}

// Hàm kiểm tra hàng đợi đầy
bool isQueueFull() {
    return (rear + 1) % MAX_EVENTS == front;
}

// Thêm sự kiện vào hàng đợi
bool enqueue(Event event) {
    if (isQueueFull()) {
        printf("Event queue is full!\n");
        return false;
    }
    if (isQueueEmpty()) {
        front = rear = 0;
    } else {
        rear = (rear + 1) % MAX_EVENTS;
    }
    eventQueue[rear] = event;
    return true;
}

// Lấy sự kiện ra khỏi hàng đợi
Event dequeue() {
    if (isQueueEmpty()) {
        printf("Event queue is empty!\n");
        return -1; // Giá trị lỗi
    }
    Event event = eventQueue[front];
    if (front == rear) { // Nếu chỉ còn một phần tử
        front = rear = -1;
    } else {
        front = (front + 1) % MAX_EVENTS;
    }
    return event;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
void handleTask1() {
    printf("Task 1: Reading temperature...\n");

    DHT22_Get_Temp(&temp);

   	  	  printf("t=%f\n", temp);
}

void handleTask2() {
    printf("Task 2: Reading acceleration...\n");

    MPU6050_Read_Accel();
    	  //	  intPart = (int)floorf(Ax * 100);
    	  	  printf("Ax=%f\n", Ax);
}
void eventScheduler() {
    while (1) {
    	if (!status) {
    	            printf("Waiting for 'start' command...\n");
    	            HAL_Delay(1000); // Chờ 1 giây trước khi kiểm tra lại
    	            continue; // Quay lại đầu vòng lặp
    	        }

       if (isQueueEmpty()) {
            printf("No events, system sleeping...\n");
            // Giả lập chế độ "ngủ" bằng cách chờ 100ms
            HAL_Delay(1000);
        } else {
            Event event = dequeue(); // Lấy sự kiện ra khỏi hàng đợi

            // Xử lý sự kiện tương ứng
            switch (event) {
                case TASK_1_EVENT:
                	printf("Add event 1 to queue\n");
                    handleTask1();
                    printf("Remove t1 from queue.\n");
                    break;
                case TASK_2_EVENT:
                	printf("Add event 2 to queue\n");
                    handleTask2();

                    printf("Remove t2 from queue.\n");
                    break;
                default:
                    printf("Unknown event!\n");
                    break;
            }
        }

}}
void CDC_ReceiveCallback(uint8_t* Buf, uint32_t Len) {
    // Copy dữ liệu nhận vào bộ đệm và thêm ký tự kết thúc chuỗi
    memcpy(usb_rx_buffer, Buf, Len);
    usb_rx_buffer[Len] = '\0'; // Đảm bảo chuỗi kết thúc
     if (strcmp(usb_rx_buffer, "start") == 0) {
            status = true; // Bắt đầu hệ thống

        } else if (strcmp(usb_rx_buffer, "stop") == 0) {
            status = false; // Dừng hệ thống

        }
    // Kiểm tra nội dung và thêm sự kiện hoặc thay đổi trạng thái hệ thống
        else if (strcmp(usb_rx_buffer, "event1") == 0) {
        if (status) enqueue(TASK_1_EVENT); // Chỉ thêm nếu hệ thống đang chạy
    } else if (strcmp(usb_rx_buffer, "event2") == 0) {
        if (status) enqueue(TASK_2_EVENT); // Chỉ thêm nếu hệ thống đang chạy
    }
}





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  dht22_init();
 MPU6050_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 printf("Event-Triggered Scheduler Starting...\n");



       // Chạy bộ lập lịch để xử lý sự kiện đã thêm
       eventScheduler();




   return 0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
