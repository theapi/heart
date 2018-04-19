/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "tim.h"
#include "main.h"
#include <stdbool.h>

#define EV_PRESSED_BIT ( 1 << 0 )
/* Pattern bits in the group event, allows for 7 patterns. */
#define EV_PATTERN_BIT_1 ( 1 << 1 )
#define EV_PATTERN_BIT_2 ( 1 << 2 )
#define EV_PATTERN_BIT_3 ( 1 << 3 )
#define EV_PATTERN_CHANGE_BIT ( 1 << 4 )
/* The above bits that define the pattern */
#define PATTERN_BIT_MASK 0b0001110

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId buttonHandle;
osThreadId patternManagerHandle;
osThreadId pattern1Handle;
osThreadId pattern2Handle;
osThreadId pattern3Handle;

/* USER CODE BEGIN Variables */

/* Declare a variable to hold the handle of the created event group. */
EventGroupHandle_t xEventGroupHandle;


  /* Sine wave table */
  const uint8_t sine_wave[256] = {
    0x80, 0x83, 0x86, 0x89, 0x8C, 0x90, 0x93, 0x96,
    0x99, 0x9C, 0x9F, 0xA2, 0xA5, 0xA8, 0xAB, 0xAE,
    0xB1, 0xB3, 0xB6, 0xB9, 0xBC, 0xBF, 0xC1, 0xC4,
    0xC7, 0xC9, 0xCC, 0xCE, 0xD1, 0xD3, 0xD5, 0xD8,
    0xDA, 0xDC, 0xDE, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8,
    0xEA, 0xEB, 0xED, 0xEF, 0xF0, 0xF1, 0xF3, 0xF4,
    0xF5, 0xF6, 0xF8, 0xF9, 0xFA, 0xFA, 0xFB, 0xFC,
    0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFD,
    0xFD, 0xFC, 0xFB, 0xFA, 0xFA, 0xF9, 0xF8, 0xF6,
    0xF5, 0xF4, 0xF3, 0xF1, 0xF0, 0xEF, 0xED, 0xEB,
    0xEA, 0xE8, 0xE6, 0xE4, 0xE2, 0xE0, 0xDE, 0xDC,
    0xDA, 0xD8, 0xD5, 0xD3, 0xD1, 0xCE, 0xCC, 0xC9,
    0xC7, 0xC4, 0xC1, 0xBF, 0xBC, 0xB9, 0xB6, 0xB3,
    0xB1, 0xAE, 0xAB, 0xA8, 0xA5, 0xA2, 0x9F, 0x9C,
    0x99, 0x96, 0x93, 0x90, 0x8C, 0x89, 0x86, 0x83,
    0x80, 0x7D, 0x7A, 0x77, 0x74, 0x70, 0x6D, 0x6A,
    0x67, 0x64, 0x61, 0x5E, 0x5B, 0x58, 0x55, 0x52,
    0x4F, 0x4D, 0x4A, 0x47, 0x44, 0x41, 0x3F, 0x3C,
    0x39, 0x37, 0x34, 0x32, 0x2F, 0x2D, 0x2B, 0x28,
    0x26, 0x24, 0x22, 0x20, 0x1E, 0x1C, 0x1A, 0x18,
    0x16, 0x15, 0x13, 0x11, 0x10, 0x0F, 0x0D, 0x0C,
    0x0B, 0x0A, 0x08, 0x07, 0x06, 0x06, 0x05, 0x04,
    0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x03,
    0x03, 0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x0A,
    0x0B, 0x0C, 0x0D, 0x0F, 0x10, 0x11, 0x13, 0x15,
    0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22, 0x24,
    0x26, 0x28, 0x2B, 0x2D, 0x2F, 0x32, 0x34, 0x37,
    0x39, 0x3C, 0x3F, 0x41, 0x44, 0x47, 0x4A, 0x4D,
    0x4F, 0x52, 0x55, 0x58, 0x5B, 0x5E, 0x61, 0x64,
    0x67, 0x6A, 0x6D, 0x70, 0x74, 0x77, 0x7A, 0x7D
  };

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void buttonTask(void const * argument);
void patternManagerTask(void const * argument);
void pattern1Task(void const * argument);
void pattern2Task(void const * argument);
void pattern3Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
bool buttonPressed(void);

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* Attempt to create the event group. */
  xEventGroupHandle = xEventGroupCreate();
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of button */
  osThreadDef(button, buttonTask, osPriorityNormal, 0, 64);
  buttonHandle = osThreadCreate(osThread(button), NULL);

  /* definition and creation of patternManager */
  osThreadDef(patternManager, patternManagerTask, osPriorityAboveNormal, 0, 64);
  patternManagerHandle = osThreadCreate(osThread(patternManager), NULL);

  /* definition and creation of pattern1 */
  osThreadDef(pattern1, pattern1Task, osPriorityNormal, 0, 64);
  pattern1Handle = osThreadCreate(osThread(pattern1), NULL);

  /* definition and creation of pattern2 */
  osThreadDef(pattern2, pattern2Task, osPriorityNormal, 0, 64);
  pattern2Handle = osThreadCreate(osThread(pattern2), NULL);

  /* definition and creation of pattern3 */
  osThreadDef(pattern3, pattern3Task, osPriorityNormal, 0, 64);
  pattern3Handle = osThreadCreate(osThread(pattern3), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* buttonTask function */
void buttonTask(void const * argument)
{

  /* USER CODE BEGIN buttonTask */

  /* Count the button presses up to the number of possible tasks. */
  uint8_t count = 0;
  bool pressed = 0;
  bool button_state = 0;

  for(;;) {
    /* Set / unset the event bit depending on the state of the button */
    pressed = buttonPressed();
    /* Only act on an edge */
    if (button_state != pressed) {
      button_state = pressed;
      if (!pressed) {
        /* Clear the pressed bit */
        xEventGroupClearBits(xEventGroupHandle, EV_PRESSED_BIT);
      } else {
        count++;
        if (count > 2) {
         count = 0;
        }

        /* Clear all pattern bits */
        xEventGroupClearBits(xEventGroupHandle, EV_PATTERN_BIT_3 | EV_PATTERN_BIT_2 | EV_PATTERN_BIT_1);
        switch (count) {
          case 1:
            xEventGroupSetBits(xEventGroupHandle, EV_PATTERN_CHANGE_BIT | EV_PATTERN_BIT_2 | EV_PRESSED_BIT);
            break;
          case 2:
            xEventGroupSetBits(xEventGroupHandle, EV_PATTERN_CHANGE_BIT | EV_PATTERN_BIT_2 | EV_PATTERN_BIT_1 | EV_PRESSED_BIT);
            break;
          default:
            /* Use the default */
            xEventGroupSetBits(xEventGroupHandle, EV_PATTERN_CHANGE_BIT | EV_PATTERN_BIT_1 | EV_PRESSED_BIT);
            break;
        }
      }
    }

    osDelay(20);
  }
  /* USER CODE END buttonTask */
}

/* patternManagerTask function */
void patternManagerTask(void const * argument)
{
  /* USER CODE BEGIN patternManagerTask */

  TaskHandle_t current_pattern = pattern1Handle;

  /* Infinite loop */
  for(;;) {
    /* Wait for the change pattern bit, then clear it */
    EventBits_t bits = xEventGroupWaitBits(
            xEventGroupHandle,
            EV_PATTERN_CHANGE_BIT,
            pdTRUE,
            pdTRUE,
            portMAX_DELAY );

    /* Show when a pattern change is happening */
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);

    /* Stop the current pattern */
    vTaskSuspend( current_pattern );

    /* Current pattern based on the number of button presses */
    uint8_t pattern = (bits & PATTERN_BIT_MASK) >> 1;
    switch (pattern) {
      case 2:
        current_pattern = pattern2Handle;
        vTaskResume( pattern2Handle );
        break;
      case 3:
        current_pattern = pattern3Handle;
        vTaskResume( pattern3Handle );
        break;
      default:
        current_pattern = pattern1Handle;
        vTaskResume( pattern1Handle );
        break;
    }
  }
  /* USER CODE END patternManagerTask */
}

/* pattern1Task function */
void pattern1Task(void const * argument)
{
  /* USER CODE BEGIN pattern1Task */
  /* Index of the sine table */
  uint8_t sine_index = 0;
  uint8_t sine_index_b = 127;

  /*
   * Fade all the leds
   */
  for(;;) {
    // PA1     ------> TIM2_CH2
    /* The bottom led */
    htim2.Instance->CCR2 = sine_wave[sine_index];
    // PA2     ------> TIM21_CH1
    /* The centre led */
    htim21.Instance->CCR1 = sine_wave[sine_index_b];
    // PA3     ------> TIM21_CH2
    /* Middle left */
    htim21.Instance->CCR2 = sine_wave[sine_index];
    // PA5     ------> TIM2_CH1
    /* Middle right */
    htim2.Instance->CCR1 = sine_wave[sine_index];
    // PA6     ------> TIM22_CH1
    /* Top left */
    htim22.Instance->CCR1 = sine_wave[sine_index];
    // PA7     ------> TIM22_CH2
    /* Top right */
    htim22.Instance->CCR2 = sine_wave[sine_index];

    // Increment and let overflow back to 0.
    sine_index++;
    sine_index_b++;

    /* Fade */
    osDelay(20);
  }
  /* USER CODE END pattern1Task */
}

/* pattern2Task function */
void pattern2Task(void const * argument)
{
  /* USER CODE BEGIN pattern2Task */
  /* Index of the sine table */
  uint8_t sine_index_1 = 0;
  uint8_t sine_index_2 = 42;
  uint8_t sine_index_3 = 84;
  uint8_t sine_index_4 = 126;
  uint8_t sine_index_5 = 168;

  /* Wait until called */
  vTaskSuspend( NULL );

  /*
   * Fade all the leds slowly
   */
  for(;;) {
    // PA1     ------> TIM2_CH2
    /* The bottom led */
    htim2.Instance->CCR2 = sine_wave[sine_index_1];
    // PA2     ------> TIM21_CH1
    /* The centre led */
    htim21.Instance->CCR1 = 255;
    // PA3     ------> TIM21_CH2
    /* Middle left */
    htim21.Instance->CCR2 = sine_wave[sine_index_5];
    // PA5     ------> TIM2_CH1
    /* Middle right */
    htim2.Instance->CCR1 = sine_wave[sine_index_2];
    // PA6     ------> TIM22_CH1
    /* Top left */
    htim22.Instance->CCR1 = sine_wave[sine_index_4];
    // PA7     ------> TIM22_CH2
    /* Top right */
    htim22.Instance->CCR2 = sine_wave[sine_index_3];

    // Increment and let overflow back to 0.
    //sine_index++;
    sine_index_1++;
    sine_index_2++;
    sine_index_3++;
    sine_index_4++;
    sine_index_5++;

    osDelay(15);

  }
  /* USER CODE END pattern2Task */
}

/* pattern3Task function */
void pattern3Task(void const * argument)
{
  /* USER CODE BEGIN pattern3Task */
  /* Wait until called */
  vTaskSuspend( NULL );

  /* Solid on */
  for(;;) {
    // PA1     ------> TIM2_CH2
    htim2.Instance->CCR2 = 127;
    // PA2     ------> TIM21_CH1
    htim21.Instance->CCR1 = 127;
    // PA3     ------> TIM21_CH2
    htim21.Instance->CCR2 = 127;
    // PA5     ------> TIM2_CH1
    htim2.Instance->CCR1 = 127;
    // PA6     ------> TIM22_CH1
    htim22.Instance->CCR1 = 127;
    // PA7     ------> TIM22_CH2
    htim22.Instance->CCR2 = 127;

    /* No need to come back, nothing changes */
    osDelay(10000);
  }
  /* USER CODE END pattern3Task */
}

/* USER CODE BEGIN Application */

/**
 * State machine to monitor whether the buton is pressed.
 */
bool buttonPressed(void) {
  enum button_state {BST_RELEASED, BST_MAYBE_PRESSED, BST_PRESSED, BST_MAYBE_RELEASED};

  static bool pressed = false;
  static enum button_state state = BST_RELEASED;

  /* Button pulled high when not pressed */
  GPIO_PinState button = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

  /* Debounce */
  switch (state) {
    case BST_RELEASED:
      if (button == GPIO_PIN_RESET) {
        state = BST_MAYBE_PRESSED;
      }
      break;
    case BST_MAYBE_PRESSED:
      if (button == GPIO_PIN_RESET) {
        state = BST_PRESSED;
        pressed = true;
      } else {
        /* Not pressed, just a bounce */
        state = BST_RELEASED;
      }
      break;
    case BST_PRESSED:
      if (button == GPIO_PIN_SET) {
        state = BST_MAYBE_RELEASED;
      }
      break;
    case BST_MAYBE_RELEASED:
      if (button == GPIO_PIN_SET) {
        state = BST_RELEASED;
        pressed = false;
      } else {
        /* Just a bounce */
        state = BST_PRESSED;
      }
      break;
  }

  return pressed;
}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
