#include "ws2812.h"

extern TIM_HandleTypeDef htim3;

uint16_t pwmData[LED_DATA_SIZE];

void WS2812_PrepareData(void)
{
  uint8_t brightness = 4;  // weich glimmend
  uint8_t red = brightness, green = 0, blue = 0;

  int idx = 0;
  for (int i = 0; i < LED_COUNT; i++) {
    uint8_t color[3] = {green, red, blue};

    for (int c = 0; c < 3; c++) {
      for (int b = 7; b >= 0; b--) {
        if (color[c] & (1 << b))
          pwmData[idx++] = 52;  // 0.75 µs
        else
          pwmData[idx++] = 26;  // 0.375 µs
      }
    }
  }

  for (int i = 0; i < 40; i++) pwmData[idx++] = 0; // Reset
}

void WS2812_Send(void)
{
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t *)pwmData, LED_DATA_SIZE);
}
