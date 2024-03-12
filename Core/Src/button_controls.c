/**
 * @brief Implementation of the LED controls.
 *
 * @author Justin Newkirk
 * @date 2/13/2024
 */

// Includes =====================================================================

#include "../Inc/button_controls.h"
#include "../Inc/led_controls.h"

// Global Variables =============================================================

// Variable used to detect a button press.

static uint32_t debouncer       = HELD_LOW;
static uint32_t debouncer_state = HELD_LOW;

// Implementation Functions =====================================================

void initialize_user_button( void ) {

    // Attach the RCC to the GPIOA bus.
    //SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

    // Set the Push Button Speed to low speed.
    //CLEAR_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR0_0);

    // Set the GPIO to have a pull down resistor.
    //SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR0_1);
}

// Function Implementations =====================================================

#define MAX_DELAY_COUNT 1500000

void EXTI0_1_IRQHandler( void ) {

	LED_ON(COLOR_ORANGE);
	LED_OFF(COLOR_GREEN);

    // Add Long Delay
    for(volatile uint32_t i = 0; i < MAX_DELAY_COUNT; i++) { }

	LED_OFF(COLOR_ORANGE);
	LED_ON(COLOR_GREEN);

    // Clear the EXTI0 pending bit.

    SET_BIT(EXTI->PR, EXTI_PR_PR0);
}

bool user_button_pressed_GPIO_variant( void ) {

    debouncer <<= 1;
    debouncer |= READ_BIT(GPIOA->IDR, GPIO_IDR_0);

    switch (debouncer) {

        case HELD_HIGH:

            debouncer_state = HELD_HIGH;

            break;
        case TRANSITION_HIGH:

            if (debouncer_state != HELD_LOW)
                return false;

            debouncer_state = HELD_HIGH;

            return true;

            break;
        case HELD_LOW:

            debouncer_state = HELD_LOW;

            break;
    }

    return false;
}
