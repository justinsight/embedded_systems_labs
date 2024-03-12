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


/**
 * @brief This function will send a character over the UART3 peripheral.
 *
 * @param character The character to send.
 */
static void _uart_send_char(const char character) {

    while (!READ_BIT(USART3->ISR, USART_ISR_TXE)) { } // Wait for the Transmit Data Register to be empty.

    WRITE_REG(USART3->TDR, character); // Write the character to the Transmit Data Register.
}

/**
 * @brief This function will send a string over the UART3 peripheral.
 *
 * @param string The string to send.
 * @note The string must be null terminated.
 */
static void _uart_send_string(const char* string) {

    char character;

    while((character = *string++) != '\0')
        _uart_send_char(character);
}


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

    // LAB 4 Demonstration ------------------------------------------------------

    // Connect UART3 to the RCC

    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);

    // Set the Alternate Pin function for PC4 and PC5 for USART3_TX and USART3_RX respectively.

    CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER4_0); // Set the PC4 to alternate function mode.
    CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODER5_0); // Set the PC5 to alternate function mode.

    SET_BIT(GPIOC->MODER, GPIO_MODER_MODER4_1); // Set the PC4 to alternate function mode.
    SET_BIT(GPIOC->MODER, GPIO_MODER_MODER5_1); // Set the PC5 to alternate function mode.

    // Set the alternate function to AF1 for PC4 and PC5.

    SET_BIT(GPIOC->AFR[0], (0x1 << GPIO_AFRL_AFSEL4_Pos)); // Set the PC4 to AF1. ( USART3_TX )
    SET_BIT(GPIOC->AFR[0], (0x1 << GPIO_AFRL_AFSEL5_Pos)); // Set the PC5 to AF1. ( USART3_RX )

    // Set USART3 to 115200 Baud Rate.
    const uint32_t BRR = HAL_RCC_GetHCLKFreq() / 115200;

    WRITE_REG(USART3->BRR, BRR); // 115200 Baud Rate

    // Enable the Transmitter and Receiver for USART3.

    SET_BIT(USART3->CR1, USART_CR1_TE); // Enable the Transmitter
    SET_BIT(USART3->CR1, USART_CR1_RE); // Enable the Receiver

    // Enable the USART3

    SET_BIT(USART3->CR1, USART_CR1_UE); // Enable the UART3 Peripheral

    LED_ON(COLOR_GREEN);

    while(1) {

        // Send Hello World through the UART3 Peripheral.
        _uart_send_string("Hello World!\n\r");
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
