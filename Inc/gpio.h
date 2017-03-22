/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define LED_NUM 4
#define LA	    0x01
#define LB      0x02
#define LC      0x04
#define LD      0x08
//
#define E_READY 		   0
#define E_CALI			   1
#define E_BAT_LOW		   2
#define E_CALI_FAIL	   3
#define E_LOST_RC 	   4
#define E_AUTO_LANDED  5
#define E_BatChg       6


typedef union{
	uint8_t byte;
	struct 
	{
			uint8_t A	:1;
		  uint8_t B	:1;
			uint8_t C	:1;
		  uint8_t D	:1;
			uint8_t reserved	:4;
	}bits;
}LEDBuf_t;

typedef struct Led_tt
{
uint8_t event;
uint8_t state;
uint16_t cnt;
}LED_t;


#define LedA_on    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11, GPIO_PIN_SET)
#define LedA_off   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11, GPIO_PIN_RESET)

#define LedB_on    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_SET)
#define LedB_off   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_RESET)

#define LedC_on    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_SET)
#define LedC_off   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_RESET)

#define LedD_on    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3, GPIO_PIN_SET)
#define LedD_off   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3, GPIO_PIN_RESET)

#define LEDA_troggle HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11)
#define LEDB_troggle HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8)
#define LEDC_troggle HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1)

extern LED_t LEDCtrl;
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
void LEDFSM(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
