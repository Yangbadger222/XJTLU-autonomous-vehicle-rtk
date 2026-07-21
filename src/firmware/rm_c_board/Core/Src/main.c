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
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "headfile.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFFER_SIZE 19  // 根据需要设置缓冲区大小 
char input_buffer[BUFFER_SIZE];

uint8_t rx_char;
uint16_t buffer_index = 0;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


extern int stop_flag; // KEY Button
extern FusionAhrs ahrs;
extern int control_mode;
extern int brake_flag;

int control_mode = 1;  // 0: 手柄控制, 1: 串口控制

// volatile int new_serial_data_received = 0;

int SERIAL_PERIOD_MS = 10; // 100HZ

float Vcx = 0;   //   m/s 
float Wc = 0;    //   rad/s 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Serial_Output();
void Serial_Input(); // 函数声明
void Serial_Control();

uint8_t DMA_RX_Buffer[DMA_RX_BUF_SIZE];
uint8_t UART1_RX_Buffer[UART_RX_BUF_SIZE];
volatile uint16_t UART1_RX_Size = 0;
volatile uint8_t new_serial_data_received = 0;

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

  // USART1_DMA_Init();  // 启动 DMA 串口接收

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  Init_all();
  clrStruct();
  
  TaskAdd(Serial_Control, 100); // 100Hz
  
  TaskAdd(Serial_Output, SERIAL_PERIOD_MS); // 100Hz Serial
 

  TaskAdd(Speed_set, 5); // 200Hz for wheel PID
  TaskAdd(IMU_update, IMUdeltaTime * 1000); // 00Hz for updating IMU and print delta time // TaskPrintDeltaTime( TaskAdd(IMU_update, 1) );
  
  TaskAdd(Joystick_motor_control, 10); // 50Hz
 
//  TaskAdd(ParseGpsBuffer, 100); // 10Hz

  // (1) Start DMA reception
  HAL_UART_Receive_DMA(&huart1, DMA_RX_Buffer, DMA_RX_BUF_SIZE);

  // (2) Enable the idle interrupt (IDLE interrupt)
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // led_green_start();

    if(stop_flag)  // Stop Driving when stop_flag is true
    {
      Emergency_Stop_Output();
      led_red_start();
      HAL_Delay(5);
    }
    else
    {
        TaskRun();

        if (new_serial_data_received) {
          new_serial_data_received = 0;
          Serial_Input((char*)UART1_RX_Buffer);
          usart_printf("Received: %s\r\n", UART1_RX_Buffer);
      }
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
  RCC_OscInitStruct.PLL.PLLM = 6;
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

/* USER CODE BEGIN 4 */

extern float gyro[], accel[];
extern FusionOffset offset;
extern FusionVector gyroscope;
extern FusionVector accelerometer;
extern FusionVector magnetometer;
float real_vc;
float real_w;    //clockwise stands for minus
float x, y, z; // local pose

// void USART1_DMA_Init(void)
//   {
//       // 启动 DMA 接收


//       HAL_UART_Receive_DMA(&huart1, input_buffer, BUFFER_SIZE);
//   }


void Serial_Output(){
  const FusionQuaternion Q = FusionAhrsGetQuaternion(&ahrs);
  const FusionEuler euler = FusionQuaternionToEuler(Q);
  const FusionVector LinearAcc = FusionAhrsGetLinearAcceleration(&ahrs);
  MOTORrpm2vw(motor_chassis[0].speed_rpm,-motor_chassis[2].speed_rpm,&real_vc,&real_w);
  float yaw = FusionDegreesToRadians(euler.angle.yaw), pitch = FusionDegreesToRadians(euler.angle.pitch), roll = FusionDegreesToRadians(euler.angle.roll);
  float cos_yaw = cos(yaw), sin_yaw = sin(yaw);
  float cos_pitch = cos(pitch), sin_pitch = sin(pitch);
  float cos_roll = cos(roll), sin_roll = sin(roll);
  float delta_s = real_vc * (float)SERIAL_PERIOD_MS / 1000.0f;
  // pose integration
  x += (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) * delta_s;
  y += (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) * delta_s;
  z += (cos_pitch * sin_roll) * delta_s;
  
  // usart_printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",	
  //             (int)(x * 10000), (int)(y * 10000), (int)(z * 10000), // pose
  //             (int)(Q.element.x * 10000), (int)(Q.element.y * 10000), (int)(Q.element.z * 10000), (int)(Q.element.w * 10000), // orientation
  //             (int)(0.0f * 10000), (int)(real_vc * 10000), (int)(0.0f * 10000), // linear acceleration
  //             (int)(FusionDegreesToRadians(gyroscope.axis.x) * 10000), 
  //             (int)(FusionDegreesToRadians(gyroscope.axis.y) * 10000), 
  //             (int)(FusionDegreesToRadians(gyroscope.axis.z) * 10000));

    usart_printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                      x, y, z, // Position coordinates
                      Q.element.x,
                      Q.element.y,
                      Q.element.z,
                      Q.element.w, // Attitude (quaternion)
                      0.0f, real_vc, 0.0f, // Linear velocity
                      FusionDegreesToRadians(gyroscope.axis.x),
                      FusionDegreesToRadians(gyroscope.axis.y),
                      FusionDegreesToRadians(gyroscope.axis.z), // Angular velocity

                      magnetometer.axis.x,
                      magnetometer.axis.y,
                      magnetometer.axis.z); // Magnetometer data

  //  LongLat2XY(Convert_to_degrees(Save_Data.longitude),Convert_to_degrees(Save_Data.latitude),&gps_X,&gps_Y);
//  LongLat2XY(120.742925,31.268221,&gps_X0,&gps_Y0);
  
  // Print to VOFA+
//  usart_printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
//      gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z,
//      acc_x_avg, acc_y_avg, acc_z_avg,
//      magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z,
//      euler.angle.roll, euler.angle.pitch, euler.angle.yaw
//  );
  
//usart_printf("%lf,%lf,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",  
//      Convert_to_degrees(Save_Data.latitude),
//      Convert_to_degrees(Save_Data.longitude),
//      motor_chassis[0].speed_rpm,
//      motor_chassis[1].speed_rpm,
//      -motor_chassis[2].speed_rpm,
//      -motor_chassis[3].speed_rpm,
//      real_vc,
//      real_w,
//      gyro[2],
//      euler.angle.roll,
//      euler.angle.pitch,
//      euler.angle.yaw,
//      LinearAcc.axis.x,
//      gps_X-gps_X0,
//      gps_Y-gps_Y0
//      );
      

  // usart_printf("%0.3f,%0.3f,%0.3f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
//   PrintGpsBuffer();
}

#define MAX_RETRIES 1  // 最大重试次数
#define RETRY_TIMEOUT_MS 3  // 每次重试的超时时间，单位毫秒

char failed_input_buffer[BUFFER_SIZE];  // 用于存储解析失败的输入数据
char record_input_buffer[BUFFER_SIZE];
// void Serial_Input(const char* input_data)
// {
//     if (control_mode == 1)
//     {
//         float temp_vcx, temp_wc;
//         int retries = 0;  // 重试次数
//         // 尝试解析输入数据
//         while (retries < MAX_RETRIES) {
//             if (sscanf(input_data, "vcx=%f,wc=%f", &temp_vcx, &temp_wc) == 2) {
//                 // 如果解析成功，更新Vcx和Wc
//                 Vcx = temp_vcx;
//                 Wc = temp_wc;
//                 return;  // 解析成功，退出函数
//             } else {
//                 // 如果解析失败，增加重试次数并等待

//                 retries++;

//                 // 将失败的输入数据存入失败缓冲区，以便监视
//                 snprintf(failed_input_buffer, sizeof(failed_input_buffer), "Failed input: %s", input_data);
                
//                 // 可选择在串口打印失败的输入数据，方便调试
//                 // usart_printf("Failed to parse input after %d retries: %s\n", retries, failed_input_buffer);

//                 HAL_Delay(RETRY_TIMEOUT_MS);  // 等待指定时间后重新尝试
//             }
//         }
//         led_green_start();
//         // 如果重试次数用尽，表示解析失败
//         usart_printf("Failed to parse input after %d retries: %s\n", retries, input_data);
//     }
// }


// void Serial_Input(const char* input_data)
// {
//     if (control_mode == 1)
//     {   
//         led_green_start();
//         float temp_vcx, temp_wc;
//         int retries = 0;  // 重试次数

//         // 尝试解析输入数据
//         while (retries < MAX_RETRIES) {
//             if (sscanf(input_data, "vcx=%f,wc=%f", &temp_vcx, &temp_wc) == 2) {
//                 // 如果解析成功，更新 Vcx 和 Wc
//                 Vcx = temp_vcx;
//                 Wc = temp_wc;
//                 return;  // 解析成功，退出函数
//             } else {
//                 retries++;
//                 HAL_Delay(RETRY_TIMEOUT_MS);  // 等待指定时间后重新尝试
//             }
//         }
//         // 如果解析失败，打印错误信息
//         snprintf(failed_input_buffer, sizeof(failed_input_buffer), "Failed input: %s", input_data);
//         usart_printf("Failed to parse input after %d retries: %s\n", retries, input_data);
//     }
// }

void Serial_Input(const char* input_data)
{
    if (control_mode == 1)
    {   
        // led_green_start();
        float temp_vcx, temp_wc;
        snprintf(record_input_buffer, sizeof(record_input_buffer), "%s", input_data);

        // Attempt to parse the input data
        if (sscanf(input_data, "vcx=%f,wc=%f\n", &temp_vcx, &temp_wc) == 2) {
            // If parsing is successful, update Vcx and Wc
            Vcx = temp_vcx;
            Wc = temp_wc;
            
            // Parsing succeeded, exit the function
        } 
        else {      // If parsing fails, print an error message
            // led_red_start();
            snprintf(failed_input_buffer, sizeof(failed_input_buffer), "Failed input: %s", input_data);
        }
    }
}


// void Serial_Control(void)
// {
//     if (new_serial_data_received) {
//         // 解析串口输入的数据，更新 Vcx 和 Wc
//         Serial_Input(input_buffer);
        
//         // 重置标志位
//         new_serial_data_received = 0;
//         memset(input_buffer, 0, BUFFER_SIZE);  // 清空接收缓冲区
//     }
// }


void Serial_Control(void)
{
    // if (new_serial_data_received) {
    //     new_serial_data_received = 0;

    //     // 这里的 UART1_RX_Buffer[] 就是本次数据
    //     UART1_RX_Buffer[UART1_RX_Size] = '\0';
    //     Serial_Input((char*)UART1_RX_Buffer);

    //     // 你也可在这里 memset(UART1_RX_Buffer, 0, UART_RX_BUF_SIZE);
    // }
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
