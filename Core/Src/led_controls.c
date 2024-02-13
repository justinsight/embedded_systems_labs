/**
 * @brief Implementation of the LED controls.
 *
 * @author Justin Newkirk
 * @date 2/13/2024
 */

// Includes =====================================================================

#include "../Inc/led_controls.h"

// Initialization Functions =====================================================

void initialize_leds( void ) {

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
}

// Function Definitions =========================================================

void toggle_group(const bool group, const bool current_state){

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

void group_off(const bool group){

    const uint32_t led_A = group ? COLOR_ORANGE : COLOR_RED;
    const uint32_t led_B = group ? COLOR_GREEN  : COLOR_BLUE;

    LED_OFF(led_A);
    LED_OFF(led_B);
}
