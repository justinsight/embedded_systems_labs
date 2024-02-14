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

#define MAX_COUNT           10000

int main(void) {

    // MCU Configuration--------------------------------------------------------
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();
    
    // Initialize the LEDs
    initialize_leds();

    // Initialize the User Push Button
    initialize_user_button();

    // LAB 2 Demonstration ------------------------------------------------------

    // Configure EXT1

    SET_BIT(EXTI->IMR,   EXTI_IMR_MR0);      // Enable interrupt for EXTI0
    SET_BIT(EXTI->RTSR,  EXTI_RTSR_TR0);     // Enable rising edge trigger for EXTI0

    // Enable the SYSCFG peripheral and connect RCC

    SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SYSCFGRST);

    // Set the EXTI0 to use the PA0 GPIO pin.

    SET_BIT(SYSCFG->EXTICR[ SYSCFG_EXTICR1_EXTI0 ], SYSCFG_EXTICR1_EXTI0_PA);

    // Enable the EXTI0 interrupt in the NVIC and set priority to 1 (High)

    NVIC_SetPriority(EXTI0_1_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_1_IRQn);


    LED_ON(COLOR_GREEN);

    // Enter infinite loop

    bool toggle = false;

    while(1) {

        if((toggle = !toggle)) {

            LED_ON(COLOR_RED);
        } else {

            LED_OFF(COLOR_RED);
        }

        HAL_Delay(500);
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
