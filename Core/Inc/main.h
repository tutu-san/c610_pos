/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LIM_2_Pin GPIO_PIN_3
#define LIM_2_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_5
#define LED_G_GPIO_Port GPIOE
#define LIM_1_Pin GPIO_PIN_6
#define LIM_1_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define VALVE_1_Pin GPIO_PIN_0
#define VALVE_1_GPIO_Port GPIOF
#define VALVE_2_Pin GPIO_PIN_1
#define VALVE_2_GPIO_Port GPIOF
#define VALVE_3_Pin GPIO_PIN_2
#define VALVE_3_GPIO_Port GPIOF
#define VALVE_4_Pin GPIO_PIN_3
#define VALVE_4_GPIO_Port GPIOF
#define LED_B_Pin GPIO_PIN_6
#define LED_B_GPIO_Port GPIOF
#define LIM_4_Pin GPIO_PIN_7
#define LIM_4_GPIO_Port GPIOF
#define LIM_3_Pin GPIO_PIN_8
#define LIM_3_GPIO_Port GPIOF
#define LIM_5_Pin GPIO_PIN_9
#define LIM_5_GPIO_Port GPIOF
#define LED_1_Pin GPIO_PIN_3
#define LED_1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define RE1_A_Pin GPIO_PIN_9
#define RE1_A_GPIO_Port GPIOE
#define RE1_B_Pin GPIO_PIN_11
#define RE1_B_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define RE4_A_Pin GPIO_PIN_12
#define RE4_A_GPIO_Port GPIOD
#define RE4_B_Pin GPIO_PIN_13
#define RE4_B_GPIO_Port GPIOD
#define TWE_IN_Pin GPIO_PIN_2
#define TWE_IN_GPIO_Port GPIOG
#define TWE_OUT_Pin GPIO_PIN_3
#define TWE_OUT_GPIO_Port GPIOG
#define USB_OTG_FS_OVCR_Pin GPIO_PIN_7
#define USB_OTG_FS_OVCR_GPIO_Port GPIOG
#define RE3_B_Pin GPIO_PIN_7
#define RE3_B_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_8
#define LED_R_GPIO_Port GPIOC
#define RE2_A_Pin GPIO_PIN_15
#define RE2_A_GPIO_Port GPIOA
#define LIM6_Pin GPIO_PIN_14
#define LIM6_GPIO_Port GPIOG
#define RE2_B_Pin GPIO_PIN_3
#define RE2_B_GPIO_Port GPIOB
#define RE3_A_Pin GPIO_PIN_4
#define RE3_A_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
