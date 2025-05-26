/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Hauptprogramm für STM32G031G8U6 WS2812-Treiber
  * @description    : Steuert 12 WS2812-LEDs an PA7 (TIM3_CH2) mit sanftem Blinken
  * @author         : Angepasst für STM32G031G8U6 mit HAL
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Ihr Name.
  * Lizenziert unter denselben Bedingungen wie der Originalcode (AS-IS).
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_hal.h"
#include "dma.h"
#include "tim.h"
#include "ws2812.h"
#include "gpio.h" // Für MX_GPIO_Init

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stddef.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_CIRCLE_MODE = 0,
    STATE_FLASH_BLUE = 1
} State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RGB_CYCLE_DURATION_MS 9000  // 9 Sekunden für den RGB-Zyklus
#define COLOR_CHANGE_INTERVAL_MS 1000 // Farbwechsel alle 1 Sekunde
#define FADING_DURATION_MS 5000     // Fading-Dauer über 5 Sekunden pro Zyklus
#define FLASH_BLUE_DURATION_MS 7000 // 7000 ms für blauen Flash
#define CIRCLE_UPDATE_INTERVAL_MS 67 // Aktualisierungsintervall für den Kreis-Modus
#define FADING_INTERVAL_MS 10       // Fading-Intervall
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
uint8_t leds_color_data[LED_CFG_BYTES_PER_LED * LED_CFG_COUNT]; // LED-Farbpuffer (R,G,B)
uint32_t dma_buffer[2 * LED_CFG_LEDS_PER_DMA_IRQ * LED_CFG_BYTES_PER_LED * 8]; // DMA-Puffer
volatile uint8_t is_updating = 0;    // Übertragung läuft
volatile uint32_t led_cycles_cnt;    // Zählt übertragene LED-Zyklen
volatile uint8_t brightness = 0xFF;  // Helligkeit (0-255)
volatile uint32_t color_counter = 0; // Farbzyklus-Zähler (0: Rot, 1: Grün, 2: Blau)
float fade_value = 0.0f;            // Fading-Wert (0.0-255.0)
float fade_step = 0.0f;             // Fading-Schritt (wird berechnet)
volatile uint8_t cycle_count = 0;    // Zählt die Farbzyklen (nach 3 Zyklen Moduswechsel)
volatile uint8_t in_circle_mode = 0; // Flag für Kreis-Modus
volatile uint8_t red_led_index = 0;  // Index der roten LED im Kreis-Modus
volatile uint8_t interrupt_triggered = 0; // Flag für Interrupt an PA1
volatile uint32_t interrupt_flash_timer = 0; // Timer für blauen Flash

// FSM Variablen
volatile int current_state = STATE_CIRCLE_MODE;
volatile int previous_state = STATE_CIRCLE_MODE;
uint32_t state_timer = 0; // Timer für Zustandsdauer
uint32_t rgb_cycle_timer = 0; // Timer für RGB-Zyklus
uint32_t circle_mode_timer = 0; // Timer für Kreis-Modus
uint32_t last_update_time = 0; // Für Timing-Kontrolle
uint32_t last_fading_time = 0; // Für Fading-Timing
uint32_t last_circle_update_time = 0; // Für Kreis-Modus-Timing
uint32_t last_flash_start_time = 0; // Für Timing des blauen Flashes
uint32_t last_rgb_cycle_start_time = 0; // Für Timing des RGB-Zyklus
uint32_t last_color_change_time = 0; // Für Timing des Farbwechsels
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void handle_state(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Berechnet quadratische Kurve für sanftes Fading
  * @param t: Eingabewert [0,1]
  * @return Wert [0,1]
  */
float quad_calc(float t) {
    return t * t * t * t;
}

/**
  * @brief Bereitet PWM-Daten für WS2812 vor
  * @param ledx: LED-Index
  * @param ptr: Ausgabearray für PWM-Werte
  */
void led_fill_led_pwm_data(size_t ledx, uint32_t* ptr) {
    extern TIM_HandleTypeDef htim3;
    uint32_t arr = htim3.Init.Period + 1;
    uint32_t pulse_high = (3 * arr / 4) - 1; // Hoch für logische 1 (~68%)
    uint32_t pulse_low = (1 * arr / 4) - 1;  // Hoch für logische 0 (~32%)

    if (ledx < LED_CFG_COUNT) {
        uint32_t r, g, b;
        // Im Kreis-Modus oder Flash-Modus keine Helligkeitsanpassung für die aktive LED
        if ((in_circle_mode && ledx == red_led_index) || (current_state == STATE_FLASH_BLUE)) {
            r = leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 0]; // Volle Helligkeit
            g = leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 1];
            b = leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 2];
        } else {
            r = (uint8_t)(((uint32_t)leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 0] * brightness) / 0xFF);
            g = (uint8_t)(((uint32_t)leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 1] * brightness) / 0xFF);
            b = (uint8_t)(((uint32_t)leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 2] * brightness) / 0xFF);
        }
        for (size_t i = 0; i < 8; i++) {
            ptr[i] = (g & (1 << (7 - i))) ? pulse_high : pulse_low;      // Grün
            ptr[8 + i] = (r & (1 << (7 - i))) ? pulse_high : pulse_low;  // Rot
            ptr[16 + i] = (b & (1 << (7 - i))) ? pulse_high : pulse_low; // Blau
        }
    }
}

/**
  * @brief Aktualisiert WS2812-Daten bei DMA-HT/TC-Ereignissen
  * @param tc: 1 für Transfer Complete, 0 für Half Transfer
  */
void led_update_sequence(uint8_t tc) {
    tc = tc ? 1 : 0;
    if (!is_updating) {
        return;
    }

    led_cycles_cnt += LED_CFG_LEDS_PER_DMA_IRQ;

    if (led_cycles_cnt < LED_RESET_PRE_MIN_LED_CYCLES) {
#if LED_CFG_LEDS_PER_DMA_IRQ > 1
        if ((led_cycles_cnt + LED_CFG_LEDS_PER_DMA_IRQ) > LED_RESET_PRE_MIN_LED_CYCLES) {
            uint32_t index = LED_RESET_PRE_MIN_LED_CYCLES - led_cycles_cnt;
            for (uint32_t i = 0; index < LED_CFG_LEDS_PER_DMA_IRQ && i < LED_CFG_COUNT; ++index, ++i) {
                led_fill_led_pwm_data(i, &dma_buffer[tc * DMA_BUFF_ELE_HALF_LEN + (index % LED_CFG_LEDS_PER_DMA_IRQ) * DMA_BUFF_ELE_LED_LEN]);
            }
        }
#endif
    } else if (led_cycles_cnt < (LED_RESET_PRE_MIN_LED_CYCLES + LED_CFG_COUNT)) {
        uint32_t next_led = led_cycles_cnt - LED_RESET_PRE_MIN_LED_CYCLES;
#if LED_CFG_LEDS_PER_DMA_IRQ == 1
        led_fill_led_pwm_data(next_led, &dma_buffer[tc * DMA_BUFF_ELE_HALF_LEN]);
#endif
#if LED_CFG_LEDS_PER_DMA_IRQ > 1
        uint32_t counter = 0;
        for (; counter < LED_CFG_LEDS_PER_DMA_IRQ && next_led < LED_CFG_COUNT; ++counter, ++next_led) {
            led_fill_led_pwm_data(next_led, &dma_buffer[tc * DMA_BUFF_ELE_HALF_LEN + counter * DMA_BUFF_ELE_LED_LEN]);
        }
        if (counter < LED_CFG_LEDS_PER_DMA_IRQ) {
            memset(&dma_buffer[tc * DMA_BUFF_ELE_HALF_LEN + counter * DMA_BUFF_ELE_LED_SIZEOF], 0x00,
                   (LED_CFG_LEDS_PER_DMA_IRQ - counter) * DMA_BUFF_ELE_LED_SIZEOF);
        }
#endif
    } else if (led_cycles_cnt < (LED_RESET_PRE_MIN_LED_CYCLES + LED_CFG_COUNT + LED_RESET_POST_MIN_LED_CYCLES + LED_CFG_LEDS_PER_DMA_IRQ)) {
        if (led_cycles_cnt < (LED_RESET_PRE_MIN_LED_CYCLES + LED_CFG_COUNT + 2 * LED_CFG_LEDS_PER_DMA_IRQ)) {
            memset(&dma_buffer[tc * DMA_BUFF_ELE_HALF_LEN], 0x00, DMA_BUFF_ELE_HALF_SIZEOF);
        }
    } else {
        HAL_DMA_Abort_IT(&hdma_tim3_ch2);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        is_updating = 0;
    }
}

/**
  * @brief Startet die WS2812-Datenübertragung
  * @return 1 wenn gestartet, 0 wenn bereits läuft
  */
uint8_t led_start_transfer(void) {
    extern TIM_HandleTypeDef htim3;
    if (is_updating) {
        return 0;
    }

    is_updating = 1;
    led_cycles_cnt = LED_CFG_LEDS_PER_DMA_IRQ;

    memset(dma_buffer, 0x00, sizeof(dma_buffer));
    for (uint32_t i = 0, index = LED_RESET_PRE_MIN_LED_CYCLES; index < 2 * LED_CFG_LEDS_PER_DMA_IRQ; ++index, ++i) {
        led_fill_led_pwm_data(i, &dma_buffer[index * DMA_BUFF_ELE_LED_LEN]);
    }

    // Deinitialisiere und initialisiere den DMA-Kanal neu
    HAL_DMA_DeInit(&hdma_tim3_ch2);
    if (HAL_DMA_Init(&hdma_tim3_ch2) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_Delay(100);
        while (1);
    }

    // Überprüfe den DMA-Zustand vor dem Start
    HAL_DMA_StateTypeDef dma_state = HAL_DMA_GetState(&hdma_tim3_ch2);
    if (dma_state != HAL_DMA_STATE_READY) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_Delay(100);
        while (1);
    }

    // Überprüfe die NVIC-Einstellungen
    if (!NVIC_GetEnableIRQ(DMA1_Channel1_IRQn)) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(100);
        while (1);
    }

    // Überprüfe die Timer-Initialisierung
    if (htim3.Instance == NULL) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        while (1);
    }

    // Starte den Timer mit DMA
    htim3.Instance->DIER |= TIM_DIER_CC2DE; // Explizit aktivieren
    if (HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, dma_buffer, DMA_BUFF_ELE_LEN) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(100);
        while (1);
    }

    return 1;
}

/* USER CODE END 0 */

/**
  * @brief  Der Einstiegspunkt der Anwendung.
  * @retval int
  */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init(); // GPIO-Initialisierung für PA1
    MX_DMA_Init();
    MX_TIM3_Init();

    // SysTick-Konfiguration (1 ms Interrupt)
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SysTick_IRQn);

    while (1) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_update_time >= 1) { // 1 ms Schleifenintervall
            handle_state();
            last_update_time = current_time;
        }
    }
}

/**
  * @brief Systemtakt-Konfiguration
  * @retval Keine
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

/**
  * @brief  Diese Funktion wird bei Fehlern ausgeführt.
  * @retval Keine
  */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

/**
  * @brief  Behandelt die Finite State Machine (FSM) für die LED-Steuerung
  * @retval Keine
  */
void handle_state(void) {
    static uint32_t last_circle_update_time = 0;
    static uint32_t last_flash_start_time = 0;

    uint32_t current_time = HAL_GetTick();

    // INTERRUPT-HANDLING VOR DEM SWITCH!
    if (interrupt_triggered && current_state != STATE_FLASH_BLUE) {
        previous_state = current_state;
        current_state = STATE_FLASH_BLUE;
        last_flash_start_time = HAL_GetTick(); // Sofort setzen!
        interrupt_triggered = 0;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // Flash-Blue aktiv
    }

    // Debug: Zeige aktuellen State an PA8
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (current_state == STATE_FLASH_BLUE) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    switch (current_state) {
        case STATE_CIRCLE_MODE:
            // Kreis-Modus: Rosa LED wandert alle 67 ms
            if (current_time - last_circle_update_time >= CIRCLE_UPDATE_INTERVAL_MS) {
                if (is_updating) {
                    HAL_DMA_Abort_IT(&hdma_tim3_ch2);
                    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
                    is_updating = 0;
                }

                // Setze alle LEDs auf Schwarz (aus)
                for (size_t i = 0; i < LED_CFG_COUNT; ++i) {
                    leds_color_data[i * LED_CFG_BYTES_PER_LED + 0] = 0x00; // Rot
                    leds_color_data[i * LED_CFG_BYTES_PER_LED + 1] = 0x00; // Grün
                    leds_color_data[i * LED_CFG_BYTES_PER_LED + 2] = 0x00; // Blau
                }

                // Setze die aktuelle Position der rosa LED (R=255, G=0, B=128)
                leds_color_data[red_led_index * LED_CFG_BYTES_PER_LED + 0] = 0xFF; // Rot
                leds_color_data[red_led_index * LED_CFG_BYTES_PER_LED + 1] = 0x00; // Grün
                leds_color_data[red_led_index * LED_CFG_BYTES_PER_LED + 2] = 0x80; // Blau (128)

                // Im Kreis-Modus die LED weiterbewegen
                red_led_index = (red_led_index + 1) % LED_CFG_COUNT;
                led_start_transfer();

                last_circle_update_time = current_time;
            }
            break;

        case STATE_FLASH_BLUE:
            if (is_updating) {
                HAL_DMA_Abort_IT(&hdma_tim3_ch2);
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
                is_updating = 0;
            }
            // Setze alle LEDs auf Blau
            for (size_t i = 0; i < LED_CFG_COUNT; ++i) {
                leds_color_data[i * LED_CFG_BYTES_PER_LED + 0] = 0x00;
                leds_color_data[i * LED_CFG_BYTES_PER_LED + 1] = 0x00;
                leds_color_data[i * LED_CFG_BYTES_PER_LED + 2] = 0xFF;
            }
            led_start_transfer();
            HAL_Delay(1000);

            if (HAL_GetTick() - last_flash_start_time >= FLASH_BLUE_DURATION_MS) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // Flash-Blue beendet
                current_state = previous_state;
            }
            break;
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        led_update_sequence(1);
    }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        led_update_sequence(0);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_1 && current_state != STATE_FLASH_BLUE) {
        interrupt_triggered = 1;
        interrupt_flash_timer = 0;
    }
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12); // Sollte bei jedem Interrupt toggeln

    if (interrupt_triggered) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11); // Sollte toggeln, wenn das Flag erkannt wird
    }
}
/* USER CODE END 4 */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Meldet den Namen der Quelldatei und die Zeilennummer des Fehlers.
  * @param  file: Zeiger auf den Namen der Quelldatei
  * @param  line: Zeilennummer des Fehlers
  * @retval Keine
  */
void assert_failed(uint8_t *file, uint32_t line) {
    while (1) {
    }
}
#endif /* USE_FULL_ASSERT */
