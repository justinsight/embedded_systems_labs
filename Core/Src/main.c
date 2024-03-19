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

// Simple Helper Functions =====================================================

static void blink_led_forever(const uint32_t color) {

    uint32_t counter = 0;
    bool     toggle  = false;

    while(1)
        if(!(counter++ % 10000))
            (toggle = !toggle) ? LED_ON(color) : LED_OFF(color);
}

// UART Functions ===============================================================

/**
 * @brief This function will send a character over the UART3 peripheral.
 *
 * @param character The character to send.
 */
static void _uart_send_char(const char character) {

    while (!READ_BIT(USART3->ISR, USART_ISR_TXE)) { } // Wait for the Transmit Data Register to be empty.

    WRITE_REG(USART3->TDR, character); // Write the character to the Transmit Data Register.
}

static char _uart_receive_char( void ) {

    while(!READ_BIT(USART3->ISR, USART_ISR_RXNE)) { } // Wait until the Receive Data Register is not empty.

    READ_REG(USART3->RDR); // Read the character from the Receive Data Register.
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

// For UART reception and flagging
static bool data_received = false;
static char data;

/**
 * @brief This function will handle the USART3 and USART4 interrupts.
 *
 * For our particular case, we will be using USART3 exclusively,
 * so we will not be checking for USART4 interrupts.
 */
void USART3_4_IRQHandler( void ) {

    const bool read_register_full = READ_BIT(USART3->ISR, USART_ISR_RXNE);

    if (read_register_full) {

        if(data_received)
            return;

        data_received = true;

        data = READ_REG(USART3->RDR);
    }
}

// I2C Functions ===============================================================

static void i2c_initialization_error(const uint32_t error_color) {

    // Stop the transaction.
    SET_BIT(I2C2->CR2, I2C_CR2_STOP);

    uint32_t counter = 0;
    bool     toggle  = false;

    while(1)
        if(!(counter++ % 10000))
            (toggle = !toggle) ? LED_ON(error_color) : LED_OFF(error_color);
}

static void _i2c_readwrite_byte(const uint8_t byte, const uint32_t target_register, const bool read_or_write) {

    // Setup the first write transaction to the gyroscope to request the particular register.

    CLEAR_BIT(I2C2->CR2, I2C_CR2_RD_WRN); // Set the transfer direction to write initially.

    SET_BIT(I2C2->CR2, (0x69 << 1));

    CLEAR_BIT(I2C2->CR2, I2C_CR2_NBYTES);
    SET_BIT(I2C2->CR2, (1 << I2C_CR2_NBYTES_Pos));

    SET_BIT(I2C2->CR2, I2C_CR2_START);

    while(!READ_BIT(I2C2->ISR, I2C_ISR_TXIS) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { }

    if(READ_BIT(I2C2->ISR, I2C_ISR_NACKF))
        i2c_initialization_error(COLOR_ORANGE);

    SET_BIT(I2C2->TXDR, target_register); // Set the target address.

    while(!READ_BIT(I2C2->ISR, I2C_ISR_TC)) { } // Wait for the transaction to complete.

    // Keeping all the values the same in CR2, we'll send either a read or write request in RD_WRN.

    if(read_or_write) {

        SET_BIT(I2C2->CR2, I2C_CR2_RD_WRN); // Set the transfer direction to read.

        SET_BIT(I2C2->CR2, I2C_CR2_START);  // Start again for restart condition (only in read mode).

        while(!READ_BIT(I2C2->ISR, I2C_ISR_RXNE) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { } // WARNING: May have to reset the TXIS flag if it is set.

        if(READ_BIT(I2C2->ISR, I2C_ISR_NACKF))
            i2c_initialization_error(COLOR_ORANGE);

        // Read the data from the RXDR register.

        const uint8_t data = READ_REG(I2C2->RXDR);

    } else {

        while(!READ_BIT(I2C2->ISR, I2C_ISR_TXIS) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { }
    }



    while(!READ_BIT(I2C2->ISR, I2C_ISR_RXNE) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { } // WARNING: May have to reset the TXIS flag if it is set.

    if(READ_BIT(I2C2->ISR, I2C_ISR_NACKF))
        i2c_initialization_error(COLOR_BLUE);

    while(!READ_BIT(I2C2->ISR, I2C_ISR_TC)) { } // Wait for the transaction to complete.

    // Set STOP bit to end the transaction.

    SET_BIT(I2C2->CR2, I2C_CR2_STOP);
    // Either read or write the target register.

    SET_BIT(I2C2->CR2, read_or_write); // Set read or write mode.

}

static void initialize_i2c2( void ) {

    // Enable the I2C2 Peripheral in the RCC

    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);

    // Set PB11 into alternate function mode and with open-drain output type.

    SET_BIT(GPIOB->MODER, GPIO_MODER_MODER11_1); // Set the PB11 to alternate function mode.
    SET_BIT(GPIOB->MODER, GPIO_MODER_MODER13_1); // Set the PB13 to alternate function mode.
    CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER15); // Set the PB13 to alternate function mode.

    SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_11);   // Set PB11 to open-drain output type.
    SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_13);   // Set PB13 to open-drain output type.

    SET_BIT(GPIOB->AFR[1], (0x1 << GPIO_AFRH_AFSEL11_Pos)); // Set the PB11 to AF1. ( I2C2_SDA )
    SET_BIT(GPIOB->AFR[1], (0x5 << GPIO_AFRH_AFSEL13_Pos)); // Set the PB13 to AF5. ( I2C2_SCL )

    // Set pins PB11 and PB13 to have pull-up resistors.

    SET_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR11_0);
    SET_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPDR13_0);

    // Set PB14 and PC0 to output mode, push-pull output, and initialize with high output.

    SET_BIT(GPIOB->MODER, GPIO_MODER_MODER14_0); // Set the PB14 to general purpose output mode.
    SET_BIT(GPIOC->MODER, GPIO_MODER_MODER0_0);  // Set the PC0 to general purpose output mode.

    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_14); // Set the PB14 to push-pull output type.
    CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_0);  // Set the PC0 to push-pull output type.

    SET_BIT(GPIOB->ODR, GPIO_ODR_14);            // Set the PB14 to high output.
    SET_BIT(GPIOC->ODR, GPIO_ODR_0);             // Set the PC0 to high output.

    // Set the I2C2 Peripheral to 100kHz

    SET_BIT(I2C2->TIMINGR, (0x1  << I2C_TIMINGR_PRESC_Pos));  // Set the prescaler to 1.
    SET_BIT(I2C2->TIMINGR, (0x13 << I2C_TIMINGR_SCLL_Pos));   // Set the SCL low period to 19.
    SET_BIT(I2C2->TIMINGR, (0xF  << I2C_TIMINGR_SCLH_Pos));   // Set the SCL high period to 15.
    SET_BIT(I2C2->TIMINGR, (0x2  << I2C_TIMINGR_SDADEL_Pos)); // Set the SDA hold time to 2.
    SET_BIT(I2C2->TIMINGR, (0x4  << I2C_TIMINGR_SCLDEL_Pos)); // Set the SCL hold time to 4.

    // Enable the I2C2 Peripheral

    SET_BIT(I2C2->CR1, I2C_CR1_PE);

    // Set up the transaction parameters
    // Set the slave address to the Gyroscope to 0x69

    SET_BIT(I2C2->CR2, (0x69 << 1));               // In 7 bit address mode, we have to write to bits [7:1], not [6:0].
    SET_BIT(I2C2->CR2, (1 << I2C_CR2_NBYTES_Pos)); // Set the number of bytes to transfer to 1.
    CLEAR_BIT(I2C2->CR2, I2C_CR2_RD_WRN);          // Set the transfer direction to write.

    SET_BIT(I2C2->CR2, I2C_CR2_START); // Start the transaction.

    // Wait for either the TXIS or NACKF flag to be set.

    while(!READ_BIT(I2C2->ISR, I2C_ISR_TXIS) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { } // WARNING: May have to reset the TXIS flag if it is set.

    // If the NACKF flag is set, then the slave did not acknowledge the address.

    if(READ_BIT(I2C2->ISR, I2C_ISR_NACKF))
        i2c_initialization_error(COLOR_RED);

    // Request the WHO_AM_I register from the Gyroscope and wait for the transaction to complete.

    WRITE_REG(I2C2->TXDR, 0xF); // Address of the WHO_AM_I register on the gyroscope.

    while(!READ_BIT(I2C2->ISR, I2C_ISR_TC)) { } // Wait for the transaction to complete.

    // Keeping all the values the same in CR2, we'll send a read request in RD_WRN

    SET_BIT(I2C2->CR2, I2C_CR2_RD_WRN); // Set the transfer direction to read.
    SET_BIT(I2C2->CR2, I2C_CR2_START);  // Start again for restart condition.

    while(!READ_BIT(I2C2->ISR, I2C_ISR_RXNE) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { } // WARNING: May have to reset the TXIS flag if it is set.

    if(READ_BIT(I2C2->ISR, I2C_ISR_NACKF))
        i2c_initialization_error(COLOR_BLUE);

    while(!READ_BIT(I2C2->ISR, I2C_ISR_TC)) { } // Wait for the transaction to complete.

    // Set STOP bit to end the transaction.

    SET_BIT(I2C2->CR2, I2C_CR2_STOP);

    // Read the data from the RXDR register.

    if(READ_REG(I2C2->RXDR) != 0xD3) // Determine if the value returned in the WHO_AM_I register is what we're expecting.
        i2c_initialization_error(COLOR_ORANGE);
}

// Main Program ================================================================


// Change threshold to 30000

#define THRESHOLD 5000

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

    // LAB 5 Demonstration ------------------------------------------------------

    // Connect GPIOC and GPIOB to the RCC
    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);

    // Initialize the I2C2 Peripheral and verify by requesting and checking the WHO_AM_I register from the gyroscope.
    initialize_i2c2();

    // Enable the X and Y sensing axes, as well as PD, in the CTRL_REG1 register

    CLEAR_BIT(I2C2->ISR, I2C_ISR_STOPF); // Clear the STOPF flag.

    const uint8_t CTRL_REG1  = 0x20;
    uint8_t GYRO_CONFIGURE_PACKAGE = 0;

    GYRO_CONFIGURE_PACKAGE |= (1 << 0); // Enable the Y axis.
    GYRO_CONFIGURE_PACKAGE |= (1 << 1); // Enable the X axis.
    GYRO_CONFIGURE_PACKAGE |= (1 << 3); // Enable Normal/Sleep mode.

    CLEAR_BIT(I2C2->CR2, I2C_CR2_NBYTES);
    SET_BIT(I2C2->CR2, (2 << I2C_CR2_NBYTES_Pos));
    CLEAR_BIT(I2C2->CR2, I2C_CR2_RD_WRN); // Write mode

    SET_BIT(I2C2->CR2, I2C_CR2_START); // Start the transaction.

    while(!READ_BIT(I2C2->ISR, I2C_ISR_TXIS) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { }

    WRITE_REG(I2C2->TXDR, CTRL_REG1); // Address of the CTRL_REG1 register on the gyroscope.

    while(!READ_BIT(I2C2->ISR, I2C_ISR_TXIS) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { }

    WRITE_REG(I2C2->TXDR, GYRO_CONFIGURE_PACKAGE); // Write the value to the CTRL_REG1 register, enabling the X and Y axes.

    while(!READ_BIT(I2C2->ISR, I2C_ISR_TC)) { } // Wait for the transaction to complete.

    SET_BIT(I2C2->CR2, I2C_CR2_STOP);

    // With our gyroscope configured, we can now read the X and Y axis data.

    while(1) {

    	HAL_Delay(100); // Delay for 100 milliseconds

        static int16_t x_axis_data = 0;
        static int16_t y_axis_data = 0;

        // Constantly read from both the X and Y axis registers.

        const uint8_t OUT_X_L = 0xA8; // We will start at this address and count up to 0x2B. This will give us the X and Y axes data.

        CLEAR_BIT(I2C2->CR2, I2C_CR2_NBYTES);
        SET_BIT(I2C2->CR2, (1 << I2C_CR2_NBYTES_Pos));
        CLEAR_BIT(I2C2->CR2, I2C_CR2_RD_WRN); // Write mode

        SET_BIT(I2C2->CR2, I2C_CR2_START); // Start the transaction.

        while(!READ_BIT(I2C2->ISR, I2C_ISR_TXIS) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { }

        WRITE_REG(I2C2->TXDR, OUT_X_L); // Address of the OUT_X_L register on the gyroscope.

        while(!READ_BIT(I2C2->ISR, I2C_ISR_TC)) { } // Wait for the transaction to complete.

        CLEAR_BIT(I2C2->CR2, I2C_CR2_NBYTES);
        SET_BIT(I2C2->CR2, (4 << I2C_CR2_NBYTES_Pos));
        SET_BIT(I2C2->CR2, I2C_CR2_RD_WRN); // Read mode

        SET_BIT(I2C2->CR2, I2C_CR2_START); // Restart the transaction.

        while(!READ_BIT(I2C2->ISR, I2C_ISR_RXNE) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { }

        x_axis_data = READ_REG(I2C2->RXDR);

        while(!READ_BIT(I2C2->ISR, I2C_ISR_RXNE) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { }

        x_axis_data |= READ_REG(I2C2->RXDR) << 8;

        while(!READ_BIT(I2C2->ISR, I2C_ISR_RXNE) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { }

        y_axis_data = READ_REG(I2C2->RXDR);

        while(!READ_BIT(I2C2->ISR, I2C_ISR_RXNE) && !READ_BIT(I2C2->ISR, I2C_ISR_NACKF)) { }

        y_axis_data |= READ_REG(I2C2->RXDR) << 8;

        while(!READ_BIT(I2C2->ISR, I2C_ISR_TC)) { } // Wait for the transaction to complete.
        
        SET_BIT(I2C2->CR2, I2C_CR2_STOP);

        // Change the onboard LEDs based on the X and Y axis data. If threshold is exceeded.
        if(x_axis_data > THRESHOLD || x_axis_data <= -THRESHOLD) {
            if(x_axis_data > 0) {

                LED_ON(COLOR_GREEN);
                LED_OFF(COLOR_ORANGE);
            } else {

                LED_OFF(COLOR_GREEN);
                LED_ON(COLOR_ORANGE);
            }
        }

        if(y_axis_data > THRESHOLD || y_axis_data <= -THRESHOLD) {
            if (y_axis_data > 0) {

                LED_ON(COLOR_RED);
                LED_OFF(COLOR_BLUE);
            } else {

                LED_OFF(COLOR_RED);
                LED_ON(COLOR_BLUE);
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
