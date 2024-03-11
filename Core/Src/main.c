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
#define MAX_ARR             10000

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

    // LAB 3 Demonstration ------------------------------------------------------

    // 3.1: User Timer Interrupts ----------------------------------------------

    // Enable the TIM2 Peripheral with the RCC
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);

    // Set TIM2 UEV to 4 Hz
    WRITE_REG(TIM2->PSC, 99); // Prescaler -> 99
    WRITE_REG(TIM2->ARR, 20000); // Auto-reload -> 20,000

    // Enable the TIM2 Update Interrupt
    SET_BIT(TIM2->DIER, TIM_DIER_UIE);

    // Enable the TIM2 Counter (ie, turn it on)
    SET_BIT(TIM2->CR1, TIM_CR1_CEN);

    // Enable the TIM2 interrupt in the NVIC and set priority to 1 (High)
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);

    // 3.2: Configure Timer Channels to PWM Mode -------------------------------

    // Enable the TIM3 Peripheral with the RCC
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);

    // Set TIM2 UEV to 800 Hz
    WRITE_REG(TIM3->PSC, 0);       // Prescaler -> 0;
    WRITE_REG(TIM3->ARR, MAX_ARR); // Auto-reload -> 10000

    // Set the Compare/Capture channels to outputs

    CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_CC1S); // Set to output - Values are 00
    CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_CC2S); // Set to output - Values are 00

    SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M);                      // PWM Mode 2 - For Channel 1
    SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2); // PWM Mode 1 - For Channel 2

    // Enable output compare preload for both channels
    SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1PE);
    SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2PE);

    SET_BIT(TIM3->CCER, TIM_CCER_CC1E); // Enable the output for channel 1
    SET_BIT(TIM3->CCER, TIM_CCER_CC2E); // Enable the output for channel 2

    // Set the Capture/Compare values for the channels at 20% the ARR value.
    WRITE_REG(TIM3->CCR1, MAX_ARR);
    WRITE_REG(TIM3->CCR2, MAX_ARR);

    // 3.3: Configuring Pin Alternate Functions --------------

    // Set alternate function for PC6 and PC7 to TIM3_CH1

    SET_BIT(GPIOC->MODER, GPIO_MODER_MODER6_1); // Set PC6 to alternate function mode
    SET_BIT(GPIOC->MODER, GPIO_MODER_MODER7_1); // Set PC7 to alternate function mode

    WRITE_REG(*GPIOC->AFR, GPIO_AFRL_AFRL0); // Set the alternate function for PC6 and PC7 (pg 46)

    // Enable the TIM3 Counter (ie, turn it on)

    SET_BIT(TIM3->CR1, TIM_CR1_CEN);

    // Enter infinite loop

    LED_ON(COLOR_GREEN);

    volatile bool count_direction = false; // False means we count down, true to count up.
    volatile uint16_t CCR_value   = MAX_ARR;

    const uint8_t prescaler = 255;

    while(1) {

        CCR_value = 0;

        // Change the Capture and Compare values for the channels up and down.
        if(count_direction = !count_direction){
            for(CCR_value = 0; CCR_value <= MAX_ARR; CCR_value += 10){

                // Busy loop to slow down the PWM change.
                for(volatile uint8_t i = 0; i < prescaler; i++) { }

                WRITE_REG(TIM3->CCR1, CCR_value);
                WRITE_REG(TIM3->CCR2, CCR_value);
            }

        } else {

            for(CCR_value = MAX_ARR; CCR_value > 0; CCR_value -= 10){

                // Busy loop to slow down the PWM change.
                for(volatile uint8_t i = 0; i < prescaler; i++) { }

                WRITE_REG(TIM3->CCR1, CCR_value);
                WRITE_REG(TIM3->CCR2, CCR_value);
            }

        }
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
