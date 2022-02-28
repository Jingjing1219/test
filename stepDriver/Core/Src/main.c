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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define DRIVER1_POW_EN_PORT GPIOB
#define DRIVER1_POW_EN_PIN GPIO_PIN_13

#define DRIVER1_A1_UP_PORT GPIOA
#define DRIVER1_A1_UP_PIN GPIO_PIN_11
#define DRIVER1_A1_DOWN_PORT GPIOA
#define DRIVER1_A1_DOWN_PIN GPIO_PIN_10

#define DRIVER1_A2_UP_PORT GPIOA
#define DRIVER1_A2_UP_PIN GPIO_PIN_9
#define DRIVER1_A2_DOWN_PORT GPIOA
#define DRIVER1_A2_DOWN_PIN GPIO_PIN_8

#define DRIVER1_B1_UP_PORT GPIOB
#define DRIVER1_B1_UP_PIN GPIO_PIN_1
#define DRIVER1_B1_DOWN_PORT GPIOB
#define DRIVER1_B1_DOWN_PIN GPIO_PIN_0

#define DRIVER1_B2_UP_PORT GPIOA
#define DRIVER1_B2_UP_PIN GPIO_PIN_7
#define DRIVER1_B2_DOWN_PORT GPIOA
#define DRIVER1_B2_DOWN_PIN GPIO_PIN_6

#define DRIVER2_POW_EN_PORT GPIOC
#define DRIVER2_POW_EN_PIN GPIO_PIN_13

#define DRIVER2_A1_UP_PORT GPIOB
#define DRIVER2_A1_UP_PIN GPIO_PIN_9
#define DRIVER2_A1_DOWN_PORT GPIOB
#define DRIVER2_A1_DOWN_PIN GPIO_PIN_8

#define DRIVER2_A2_UP_PORT GPIOB
#define DRIVER2_A2_UP_PIN GPIO_PIN_7
#define DRIVER2_A2_DOWN_PORT GPIOB
#define DRIVER2_A2_DOWN_PIN GPIO_PIN_6

#define DRIVER2_B1_UP_PORT GPIOA
#define DRIVER2_B1_UP_PIN GPIO_PIN_3
#define DRIVER2_B1_DOWN_PORT GPIOA
#define DRIVER2_B1_DOWN_PIN GPIO_PIN_2

#define DRIVER2_B2_UP_PORT GPIOB
#define DRIVER2_B2_UP_PIN GPIO_PIN_3
#define DRIVER2_B2_DOWN_PORT GPIOA
#define DRIVER2_B2_DOWN_PIN GPIO_PIN_15

#define HALL_DETEC1_PORT GPIOB
#define HALL_DETEC1_PIN GPIO_PIN_14
#define HALL_DETEC2_PORT GPIOB
#define HALL_DETEC2_PIN GPIO_PIN_15
#define HALL_DETEC3_PORT GPIOA
#define HALL_DETEC3_PIN GPIO_PIN_5
#define HALL_DETEC4_PORT GPIOA
#define HALL_DETEC4_PIN GPIO_PIN_4

#define ADC_REPEAT_TIMES    50
#define ADC_CHANNELS        2

#define MOTOR_BUFFER_SIZE   13
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t ADC_Result[ADC_REPEAT_TIMES][ADC_CHANNELS];
float ADC_Value[ADC_CHANNELS];

uint16_t ADC_value[2][100];
uint8_t i;
uint32_t adc1, adc2;
uint16_t adc[2];

struct msg
{
  uint8_t version;
  uint8_t data_from;
  uint8_t data[MOTOR_BUFFER_SIZE];
};

uint8_t RxBuffer;
static struct msg step_motor_buf;
uint8_t rec_flag, address_flag, version_flag, para_flag, set_flag;

//received motor set data
struct motor_set_data
{
  uint16_t rpm;
  uint8_t height;
  uint8_t dir;
};
struct motor_set_data knift_motor, lidar_motor;

uint8_t stepmoto_check_high_knife = 1, stepmoto_check_low_knife = 1;
uint8_t stepmoto_check_high_lidar = 1, stepmoto_check_low_lidar = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t check_sum(const void *data, uint16_t length)
{
  const uint8_t *buf = data;
  uint8_t retval = 0;
  
  while(length)
  {
    retval += *buf++;
    --length;
  }
  return retval;
}

void setMotor_four_reverse(void)
{
  static uint8_t i=0;
  
  switch(i)
  {
    case 3:
    {
      // A1=1;B1=0;A2=0;B2=0; 
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }
    case 2:
    {
      //A1=0;B1=1;A2=0;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      
      break;
    }
    case 1:
    {
      //A1=0;B1=0;A2=1;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_SET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }
    case 0:
    {
      //A1=0;B1=0;A2=0;B2=1;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_SET);
      
      break;
    }
    
  }
  i++;
  if(i>3)
    i=0;
  

}

void setMotor_four(void)
{
  static uint8_t i=0;
  
  switch(i)
  {
    case 0:
    {
      // A1=1;B1=0;A2=0;B2=0; 
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }
    case 1:
    {
      //A1=0;B1=1;A2=0;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      
      break;
    }
    case 2:
    {
      //A1=0;B1=0;A2=1;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_SET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }
    case 3:
    {
      //A1=0;B1=0;A2=0;B2=1;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_SET);
      
      break;
    }
  }
  i++;
  if(i>3)
    i=0;
  
}

void setMotor_eight(void)
{
  static uint8_t i = 0;
  switch(i)
  {
    case 0:
    {
      // A1=1;B1=0;A2=0;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      
      break;
    }

    case 1:
    {
      // A1=1;B1=1;A2=0;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }

    case 2:
    {
      // A1=0;B1=1;A2=0;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      
      break;
    }

    case 3:
    {
      // A1=0;B1=1;A2=1;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_SET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      
      break;
    }
    case 4:
    {
      // A1=0;B1=0;A2=1;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_SET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }

    case 5:
    {
      // A1=0;B1=0;A2=1;B2=1;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_SET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_SET);
      break;
    }

    case 6:
    {
      // A1=0;B1=0;A2=0;B2=1; 
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_SET);
      break;
    }

    case 7:
    {
      // A1=1;B1=0;A2=0;B2=1;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_SET);
      break;
    }

  }
  i++;
  if(i==8)
    i=0;
}

void setMotor_eight_reverse(void)
{
  static uint8_t i = 0;
  switch(i)
  {
    case 7:
    {
      // A1=1;B1=0;A2=0;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      
      break;
    }

    case 6:
    {
      // A1=1;B1=1;A2=0;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }

    case 5:
    {
      // A1=0;B1=1;A2=0;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      
      break;
    }

    case 4:
    {
      // A1=0;B1=1;A2=1;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_SET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      
      break;
    }
    case 3:
    {
      // A1=0;B1=0;A2=1;B2=0;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_SET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }

    case 2:
    {
      // A1=0;B1=0;A2=1;B2=1;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_SET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_SET);
      break;
    }

    case 1:
    {
      // A1=0;B1=0;A2=0;B2=1; 
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_SET);
      break;
    }

    case 0:
    {
      // A1=1;B1=0;A2=0;B2=1;
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_SET);
      break;
    }
  }
  i++;
  if(i==8)
    i=0;
}

void setMotor_four_reverse2(void)
{
  static uint8_t i=0;
  
  switch(i)
  {
    case 3:
    {
      // A1=1;B1=0;A2=0;B2=0; 
      HAL_GPIO_WritePin(DRIVER2_A1_UP_PORT, DRIVER2_A1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER2_A1_DOWN_PORT, DRIVER2_A1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER2_B1_UP_PORT, DRIVER2_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B1_DOWN_PORT, DRIVER2_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_A2_UP_PORT, DRIVER2_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A2_DOWN_PORT, DRIVER2_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_B2_UP_PORT, DRIVER2_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B2_DOWN_PORT, DRIVER2_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }
    case 2:
    {
      //A1=0;B1=1;A2=0;B2=0;
      HAL_GPIO_WritePin(DRIVER2_A1_UP_PORT, DRIVER2_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A1_DOWN_PORT, DRIVER2_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_B1_UP_PORT, DRIVER2_B1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER2_B1_DOWN_PORT, DRIVER2_B1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER2_A2_UP_PORT, DRIVER2_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A2_DOWN_PORT, DRIVER2_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER2_B2_UP_PORT, DRIVER2_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B2_DOWN_PORT, DRIVER2_B2_DOWN_PIN, GPIO_PIN_RESET);
      
      break;
    }
    case 1:
    {
      //A1=0;B1=0;A2=1;B2=0;
      HAL_GPIO_WritePin(DRIVER2_A1_UP_PORT, DRIVER2_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A1_DOWN_PORT, DRIVER2_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_B1_UP_PORT, DRIVER2_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B1_DOWN_PORT, DRIVER2_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_A2_UP_PORT, DRIVER2_A2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER2_A2_DOWN_PORT, DRIVER2_A2_DOWN_PIN, GPIO_PIN_SET);
      
      
      HAL_GPIO_WritePin(DRIVER2_B2_UP_PORT, DRIVER2_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B2_DOWN_PORT, DRIVER2_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }
    case 0:
    {
      //A1=0;B1=0;A2=0;B2=1;
      HAL_GPIO_WritePin(DRIVER2_A1_UP_PORT, DRIVER2_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A1_DOWN_PORT, DRIVER2_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_B1_UP_PORT, DRIVER2_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B1_DOWN_PORT, DRIVER2_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_A2_UP_PORT, DRIVER2_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A2_DOWN_PORT, DRIVER2_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER2_B2_UP_PORT, DRIVER2_B2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER2_B2_DOWN_PORT, DRIVER2_B2_DOWN_PIN, GPIO_PIN_SET);
      
      break;
    }
    
  }
  i++;
  if(i>3)
    i=0;
  

}

void setMotor_four2(void)
{
  static uint8_t i=0;
  
  switch(i)
  {
    case 0:
    {
      // A1=1;B1=0;A2=0;B2=0; 
      HAL_GPIO_WritePin(DRIVER2_A1_UP_PORT, DRIVER2_A1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER2_A1_DOWN_PORT, DRIVER2_A1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER2_B1_UP_PORT, DRIVER2_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B1_DOWN_PORT, DRIVER2_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_A2_UP_PORT, DRIVER2_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A2_DOWN_PORT, DRIVER2_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_B2_UP_PORT, DRIVER2_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B2_DOWN_PORT, DRIVER2_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }
    case 1:
    {
      //A1=0;B1=1;A2=0;B2=0;
      HAL_GPIO_WritePin(DRIVER2_A1_UP_PORT, DRIVER2_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A1_DOWN_PORT, DRIVER2_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_B1_UP_PORT, DRIVER2_B1_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER2_B1_DOWN_PORT, DRIVER2_B1_DOWN_PIN, GPIO_PIN_SET);
      
      HAL_GPIO_WritePin(DRIVER2_A2_UP_PORT, DRIVER2_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A2_DOWN_PORT, DRIVER2_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER2_B2_UP_PORT, DRIVER2_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B2_DOWN_PORT, DRIVER2_B2_DOWN_PIN, GPIO_PIN_RESET);
      
      break;
    }
    case 2:
    {
      //A1=0;B1=0;A2=1;B2=0;
      HAL_GPIO_WritePin(DRIVER2_A1_UP_PORT, DRIVER2_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A1_DOWN_PORT, DRIVER2_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_B1_UP_PORT, DRIVER2_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B1_DOWN_PORT, DRIVER2_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_A2_UP_PORT, DRIVER2_A2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER2_A2_DOWN_PORT, DRIVER2_A2_DOWN_PIN, GPIO_PIN_SET);
      
      
      HAL_GPIO_WritePin(DRIVER2_B2_UP_PORT, DRIVER2_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B2_DOWN_PORT, DRIVER2_B2_DOWN_PIN, GPIO_PIN_RESET);
      break;
    }
    case 3:
    {
      //A1=0;B1=0;A2=0;B2=1;
      HAL_GPIO_WritePin(DRIVER2_A1_UP_PORT, DRIVER2_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A1_DOWN_PORT, DRIVER2_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_B1_UP_PORT, DRIVER2_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_B1_DOWN_PORT, DRIVER2_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER2_A2_UP_PORT, DRIVER2_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER2_A2_DOWN_PORT, DRIVER2_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER2_B2_UP_PORT, DRIVER2_B2_UP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DRIVER2_B2_DOWN_PORT, DRIVER2_B2_DOWN_PIN, GPIO_PIN_SET);
      
      break;
    }
  }
  i++;
  if(i>3)
    i=0;
  
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
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(4000);
  
  HAL_GPIO_WritePin(DRIVER1_POW_EN_PORT, DRIVER1_POW_EN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRIVER2_POW_EN_PORT, DRIVER2_POW_EN_PIN, GPIO_PIN_RESET);
  
  HAL_Delay(2000);
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim2);
  
  //uart
  HAL_Delay(1000);
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&RxBuffer, 1);
  
   //set_flag = 1;
    
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //adc
     uint16_t Count = 20;
     adc[0] = 0;
     adc[1] = 0;
     while(Count)
     {
       for(i = 0; i < 2; i++)
       {
         HAL_ADC_Start(&hadc1);
       
         HAL_ADC_PollForConversion(&hadc1, 0xffff); //等待ADC转换完成
         
         if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
         {
            adc[i] += HAL_ADC_GetValue(&hadc1); //voltage
         }
       }
     
       HAL_ADC_Stop(&hadc1);
       Count--;
      
       HAL_Delay(5);
     }
     ADC_Value[0] = adc[0]/20 * 3.3f / 4096.0f;
     ADC_Value[1] = adc[1]/20 * 3.3f / 4096.0f;
     HAL_Delay(1000);
    
    //Hall detect
    stepmoto_check_high_knife = !HAL_GPIO_ReadPin(HALL_DETEC1_PORT, HALL_DETEC1_PIN);
    stepmoto_check_low_knife = !HAL_GPIO_ReadPin(HALL_DETEC2_PORT, HALL_DETEC2_PIN);
    stepmoto_check_high_lidar = !HAL_GPIO_ReadPin(HALL_DETEC3_PORT, HALL_DETEC3_PIN);
    stepmoto_check_low_lidar = !HAL_GPIO_ReadPin(HALL_DETEC4_PORT, HALL_DETEC4_PIN);
    
   
    
     
    //uart parser
    if(rec_flag)
    {
      rec_flag = 0;
      if(step_motor_buf.data[7] == check_sum((const void*)&step_motor_buf.data[2], 5))
      {
        if(step_motor_buf.data[1] == 0x01) //knife motor
        {
          if(step_motor_buf.data[2] == 0xC2) //receive direction, height, velocity
          {
            para_flag = 1;
            set_flag = 1;
            knift_motor.rpm = (int16_t)((step_motor_buf.data[3] << 8) | step_motor_buf.data[4]);
            knift_motor.dir = step_motor_buf.data[5];
            knift_motor.height = step_motor_buf.data[6];
          }
          else if(step_motor_buf.data[2] == 0xC0) //address
          {
            address_flag = 1;
          }
          else if(step_motor_buf.data[2] == 0xC1) //require version
          {
            version_flag = 1;
          }
        }
        else if(step_motor_buf.data[1] == 0x01) //lidar motor
        {
          
        }
      }
      
      
      //uart send
      if(address_flag == 1)
      {
        address_flag = 0;
        uint8_t buf_knife[9] = {0x55, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x01, 0x00, 0xAA};
        buf_knife[7] = check_sum(&buf_knife[2], 5);
        
        HAL_UART_Transmit_IT(&huart3, &buf_knife[0], sizeof(buf_knife));
      }
      if(version_flag == 1) //发送版本号
      {
        version_flag = 0;
        uint8_t buf_knife_version[9] = {0x55, 0x00, 0xC1, 0x00, 0x01, 0x00, 0x01, 0x00, 0xAA};
        buf_knife_version[7] = check_sum(&buf_knife_version[2], 5);
        
        HAL_UART_Transmit_IT(&huart3, &buf_knife_version[0], sizeof(buf_knife_version));
      }
      if(para_flag == 1)
      {
        para_flag = 0;
        //发送电流值 状态值
        uint8_t buf_knife_state[9] = {0x55, 0x00, 0xC2, 0x00, 0x01, 0x00, 0x01, 0x00, 0xAA};
        buf_knife_state[5] = ADC_Value[0]; //电流值
        buf_knife_state[6] = 0x01;
        buf_knife_state[7] = check_sum(&buf_knife_state[2], 5);
        
        HAL_UART_Transmit_IT(&huart3, &buf_knife_state[0], sizeof(buf_knife_state));
      }
    }
    
    //限流
    if(ADC_Value[0] > 5 || ADC_Value[1] > 5)
    {
      HAL_GPIO_WritePin(DRIVER1_POW_EN_PORT, DRIVER1_POW_EN_PIN, GPIO_PIN_RESET);
          
      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
      
      
      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
      
      set_flag = 0;
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 800-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_3
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA6 PA7
                           PA8 PA9 PA10 PA11
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB13 PB3
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_3
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint32_t flag_reverse = 0;
uint32_t cnt = 0;
uint8_t up_flag = 0; 

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ // 0.8ms
  if(TIM2 == htim->Instance)
  {
//    //正转15 stop 反转15 stop 
//    cnt++;
//    if(cnt < 2000);
//    else if(cnt == 2000)
//    { //open
//      HAL_GPIO_WritePin(DRIVER1_POW_EN_PORT, DRIVER1_POW_EN_PIN, GPIO_PIN_SET);
//    }
//    else if(cnt < 5000)
//    {
//      setMotor_eight();
//    }
//    else if(cnt == 5000)
//    { //close
//      HAL_GPIO_WritePin(DRIVER1_POW_EN_PORT, DRIVER1_POW_EN_PIN, GPIO_PIN_RESET);
//      
//      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
//      
//      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
//      
//      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
//      
//      
//      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
//    }
//    else if(cnt < 7000);
//    else if(cnt == 7000)
//    { //open
//      HAL_GPIO_WritePin(DRIVER1_POW_EN_PORT, DRIVER1_POW_EN_PIN, GPIO_PIN_SET);
//    }
//    else if(cnt <= 10000)
//    {
//      setMotor_eight_reverse();
//    }
//    else if(cnt > 10000)
//    { //close
//      HAL_GPIO_WritePin(DRIVER1_POW_EN_PORT, DRIVER1_POW_EN_PIN, GPIO_PIN_RESET);
//      
//      HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
//      
//      HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
//      
//      HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
//      
//      
//      HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
//      
//      cnt=0;
//    }

    if(set_flag == 1) //接收到标志 开始调节步进电机
    {
      if(up_flag == 0)
      {
        //向上到最高位
        cnt++;
        HAL_GPIO_WritePin(DRIVER1_POW_EN_PORT, DRIVER1_POW_EN_PIN, GPIO_PIN_SET);
        if(cnt < 2000 && stepmoto_check_high_knife == 0x01)
        {
          setMotor_four();
        }
        else
        {
          up_flag = 1;
          HAL_GPIO_WritePin(DRIVER1_POW_EN_PORT, DRIVER1_POW_EN_PIN, GPIO_PIN_RESET);
          
          HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
          
          HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
          
          HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
          
          
          HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
          
          cnt = 0;
        }
      }
      else
      {
        //延时一段时间再调节
        cnt++;
        if(cnt < 2000);
        else if(cnt ==2000)
        {
          //open EN 
          HAL_GPIO_WritePin(DRIVER1_POW_EN_PORT, DRIVER1_POW_EN_PIN, GPIO_PIN_SET);
        }
        else if(cnt < 4000 && stepmoto_check_low_knife == 0x01) //圈数
        {
          setMotor_four_reverse();
        }
        else
        { //调节结束
          up_flag = 0; 
          set_flag = 0;
          HAL_GPIO_WritePin(DRIVER1_POW_EN_PORT, DRIVER1_POW_EN_PIN, GPIO_PIN_RESET);
          
          HAL_GPIO_WritePin(DRIVER1_A1_UP_PORT, DRIVER1_A1_UP_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DRIVER1_A1_DOWN_PORT, DRIVER1_A1_DOWN_PIN, GPIO_PIN_RESET);
          
          HAL_GPIO_WritePin(DRIVER1_B1_UP_PORT, DRIVER1_B1_UP_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DRIVER1_B1_DOWN_PORT, DRIVER1_B1_DOWN_PIN, GPIO_PIN_RESET);
          
          HAL_GPIO_WritePin(DRIVER1_A2_UP_PORT, DRIVER1_A2_UP_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DRIVER1_A2_DOWN_PORT, DRIVER1_A2_DOWN_PIN, GPIO_PIN_RESET);
          
          
          HAL_GPIO_WritePin(DRIVER1_B2_UP_PORT, DRIVER1_B2_UP_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DRIVER1_B2_DOWN_PORT, DRIVER1_B2_DOWN_PIN, GPIO_PIN_RESET);
          
          cnt = 0;
        }
      }
    }
  }
}

//步进电机串口中断回调函数
void step_motor_receive_hanlder(void)
{
  static uint8_t num = 0;
  static uint8_t command_type;
  switch(num)
  {
    case 0:
      if(RxBuffer == 0x55)
      {
        step_motor_buf.data[num] = RxBuffer;
        num++;
      }
      break;
    case 1:
    {
      if(RxBuffer >= 0x01 && RxBuffer <= 0x02)
      {
        step_motor_buf.data[num] = RxBuffer;
        num++;
        step_motor_buf.data_from = RxBuffer;
        break;
      }
      num = 0;
      break;
    }
    case 2:
    {
      command_type = RxBuffer;
      if(command_type >= 0xC0 && command_type <= 0xC3)
      {
        step_motor_buf.data[num] = RxBuffer;
        num++;
        break;
      }
      num = 0;
      break;
    }
    case 8:
    {
      if(RxBuffer == 0xAA) //结束位
      {
        step_motor_buf.data[num] = RxBuffer;
        rec_flag = 1;
      }
      num = 0;
    }
    default:
      step_motor_buf.data[num] = RxBuffer;
      num++;
      break;
  }
  
  //HAL_UART_Receive_IT(&huart3, (uint8_t *)&RxBuffer, 1); //???
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /****************** uart3:  *********************/
  if(huart->Instance == USART3)
  {
    step_motor_receive_hanlder();
   // HAL_UART_Transmit(&huart3, (uint8_t *)&RxBuffer, 10, 0xFFFF);
  }
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

