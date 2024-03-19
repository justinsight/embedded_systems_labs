/**
 * @file main.c
 *
 * @brief Main program body
 *
 * @author Justin Newkirk
 * @date   2/12/2024
 * @class  Embedded Systems Design
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

// Main Program ================================================================

int main(void) {

    // MCU Configuration--------------------------------------------------------
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();
    
    // Initialize the LEDs
    initialize_leds();

    // LAB 6 Demonstration ------------------------------------------------------

    // WARNING: Determine if we need to attach the GPIO to the RCC

    // Configure GPIO PA1 as an analog input.

    SET_BIT(GPIOA->MODER, GPIO_MODER_MODER0);   // Set to Analog mode: 0x3
    CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR0); // Ensure no pull-up, no pull-down: 0x0

    // Enable the ADC1 Peripheral in the RCC.

    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);

    // Configure ADC for 8-bit resolution, continuous conversion mode, and ensure hardware trigger is disabled.

    SET_BIT(ADC1->CFGR1, ADC_CFGR1_RES_1); // Set resolution to 8-bit
    SET_BIT(ADC1->CFGR1, ADC_CFGR1_CONT);  // Continuous conversion mode.
    CLEAR_BIT(ADC1->CFGR1, ADC_CFGR1_EXTEN); // Ensure hardware trigger is disabled.

    // Select/enable the input pin’s channel for ADC conversion.

    SET_BIT(ADC1->CHSELR, ADC_CHSELR_CHSEL1);  // Set Channel 1 for conversion and connecting PA1 to ADC1.

    // Perform self-calibration, enable, and wait for ADC to be ready. Then, start the ADC.

    SET_BIT(ADC1->CR, ADC_CR_ADCAL); // Start the calibration process.

    while(READ_BIT(ADC1->CR, ADC_CR_ADCAL)) { } // Wait for the calibration to complete.

    SET_BIT(ADC1->CR, ADC_CR_ADEN); // Enable the ADC.

    while(!READ_BIT(ADC1->ISR, ADC_ISR_ADRDY)) { } // Wait for the ADC to be ready.

    SET_BIT(ADC1->CR, ADC_CR_ADSTART); // Start the ADC.

    while (1) {

        HAL_Delay(10); // Delay for 10ms.

        // Read the ADC data register and turn on/off the LEDs based on the value.

        const uint8_t adc_value = READ_REG(ADC1->DR);

        adc_value > 0   ? LED_ON(COLOR_RED)    : LED_OFF(COLOR_RED);
        adc_value > 64  ? LED_ON(COLOR_GREEN)  : LED_OFF(COLOR_GREEN);
        adc_value > 128 ? LED_ON(COLOR_BLUE)   : LED_OFF(COLOR_BLUE);
        adc_value > 192 ? LED_ON(COLOR_ORANGE) : LED_OFF(COLOR_ORANGE);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
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
