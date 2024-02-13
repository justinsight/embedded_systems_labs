/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  *
  *
  *@author Justin Newkirk
  *@class  Embedded Systems Design
  *@date   February 12, 2024
  *
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f072xb.h"
#include "stm32f0xx.h"
#include <stdbool.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/


// NOTE: PC6 Red, PC7 Blue

// Defines ------------------------------------------------------------------------

#define LED_ON(COLOR)  		SET_BIT(GPIOC->BSRR, COLOR);
#define LED_OFF(COLOR) 		SET_BIT(GPIOC->BRR, COLOR);

#define COLOR_RED			GPIO_BSRR_BS_6
#define COLOR_BLUE			GPIO_BSRR_BS_7
#define COLOR_ORANGE		GPIO_BSRR_BS_8
#define COLOR_GREEN			GPIO_BSRR_BS_9

#define MAX_COUNT 			10000

#define HELD_HIGH 			0xFFFFFFFF
#define TRANSITION_HIGH		0x7FFFFFFF
#define HELD_LOW			0x00000000


// Function Demos --------------------------------------------------------------------

static void toggle_group(const bool group, const bool current_state){

	const uint32_t led_A = group ? COLOR_ORANGE : COLOR_RED;
	const uint32_t led_B = group ? COLOR_GREEN  : COLOR_BLUE;

	if(current_state){


		LED_ON(led_A);
		LED_OFF(led_B);

		return;
	}

	LED_OFF(led_A);
	LED_ON(led_B);
}

static void group_off(const bool group){

	const uint32_t led_A = group ? COLOR_ORANGE : COLOR_RED;
	const uint32_t led_B = group ? COLOR_GREEN  : COLOR_BLUE;

	LED_OFF(led_A);
	LED_OFF(led_B);
}



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  // LED Settings ----------------------------------------------------------------

  // Turn on the RCC for AHB bus and GPIOC
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);

  // Set the Pins PC8 and PC9, GPIOC as general input output.
  SET_BIT(GPIOC->MODER, GPIO_MODER_MODER6_0);
  SET_BIT(GPIOC->MODER, GPIO_MODER_MODER7_0);
  SET_BIT(GPIOC->MODER, GPIO_MODER_MODER8_0);
  SET_BIT(GPIOC->MODER, GPIO_MODER_MODER9_0);

  // Set the LEDs to push/pull output type.
  CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_6);
  CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_7);
  CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_8);
  CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_9);

  // Set the LED's GPIO speed to slow.
  CLEAR_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEEDR6_0);
  CLEAR_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEEDR7_0);
  CLEAR_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEEDR8_0);
  CLEAR_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEEDR9_0);

  // Set the Pull up and pull down resistors to use neither of them.
  CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR6);
  CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR7);
  CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR8);
  CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR9);

  // Push Button Settings -------------------------------------------------------

  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

  // Set the Push Button Speed to low speed.
  CLEAR_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR0_0);

  // Set the GPIO to have a pull down resistor.
  SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR0_1);

	// Toggle Variable

	bool toggle = false;

	// Current Group
	bool group = false;

	// Duration Counter
	uint32_t counter = 0;

	// Debounce Detector

	uint32_t debouncer = 0;
	uint32_t debouncer_state = HELD_LOW;

	while(1){

		// See if its time to toggle colors.
		if(++counter % MAX_COUNT == 0)
			toggle_group(group, (toggle = !toggle));

		// Determine if our button's been pressed.
		debouncer <<= 1;
		debouncer |=  READ_BIT(GPIOA->IDR, GPIO_IDR_0);

		switch(debouncer){

		case HELD_HIGH:

			debouncer_state = HELD_HIGH;

			break;
		case TRANSITION_HIGH:

			if(debouncer_state != HELD_LOW)
				continue;

			debouncer_state = HELD_HIGH;

			group_off(group);

			group = !group;

			break;
		case HELD_LOW:

			debouncer_state = HELD_LOW;

			break;
		}
	}





}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
