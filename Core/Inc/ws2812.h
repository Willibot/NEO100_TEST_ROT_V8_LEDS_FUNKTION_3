/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ws2812.h
  * @brief   This file provides code for the WS2812 driver implementation.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Ihr Name.
  * Licensed under the same terms as the original code (AS-IS).
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WS2812_H__
#define __WS2812_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define LED_CFG_COUNT               12
#define LED_CFG_LEDS_PER_DMA_IRQ    1
#define LED_CFG_BYTES_PER_LED       3
#define LED_RESET_PRE_MIN_LED_CYCLES    2
#define LED_RESET_POST_MIN_LED_CYCLES   2

#define DMA_BUFF_ELE_HALF_LEN   (LED_CFG_LEDS_PER_DMA_IRQ * LED_CFG_BYTES_PER_LED * 8)
#define DMA_BUFF_ELE_LEN        (2 * DMA_BUFF_ELE_HALF_LEN)
#define DMA_BUFF_ELE_HALF_SIZEOF    (DMA_BUFF_ELE_HALF_LEN * sizeof(uint32_t))
#define DMA_BUFF_ELE_SIZEOF     (DMA_BUFF_ELE_LEN * sizeof(uint32_t))
#define DMA_BUFF_ELE_LED_LEN    (LED_CFG_BYTES_PER_LED * 8)
#define DMA_BUFF_ELE_LED_SIZEOF     (DMA_BUFF_ELE_LED_LEN * sizeof(uint32_t))
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
extern uint8_t leds_color_data[LED_CFG_BYTES_PER_LED * LED_CFG_COUNT];
extern uint32_t dma_buffer[2 * LED_CFG_LEDS_PER_DMA_IRQ * LED_CFG_BYTES_PER_LED * 8];
extern volatile uint8_t is_updating;
extern volatile uint32_t led_cycles_cnt;
extern volatile uint8_t brightness;
extern volatile uint32_t color_counter;
extern float fade_step;
extern float fade_value;
extern volatile uint8_t cycle_count;
extern volatile uint8_t in_circle_mode;
extern volatile uint8_t green_led_index;
extern volatile uint8_t interrupt_triggered; // Flag für Interrupt an PA1
extern volatile uint32_t interrupt_flash_timer; // Timer für blauen Flash

void led_fill_led_pwm_data(size_t ledx, uint32_t* ptr);
void led_update_sequence(uint8_t tc);
uint8_t led_start_transfer(void);
float quad_calc(float t);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H__ */
