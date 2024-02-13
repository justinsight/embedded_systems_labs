/**
 * This file contains common LED controls for the STM32 Curiosity Board
 * we're using for Embedded Systems Design.
 *
 * @author Justin Newkirk
 * @date 2/13/2024
 */

#ifndef LAB1_EMBEDDED_SYSTEMS_LED_CONTROLS_H
#define LAB1_EMBEDDED_SYSTEMS_LED_CONTROLS_H

#include "stm32f072xb.h"
#include "stm32f0xx.h"
#include <stdbool.h>

// Macros =======================================================================

#define LED_ON(COLOR)  		SET_BIT(GPIOC->BSRR, COLOR);
#define LED_OFF(COLOR) 		SET_BIT(GPIOC->BRR, COLOR);

#define COLOR_RED			GPIO_BSRR_BS_6
#define COLOR_BLUE			GPIO_BSRR_BS_7
#define COLOR_ORANGE		GPIO_BSRR_BS_8
#define COLOR_GREEN			GPIO_BSRR_BS_9

// LED Initializations ==========================================================

/**
 * @brief Will initialize the LEDs for use.
 *
 */
void initialize_leds( void );

// Helper Functions =============================================================

/**
 * @brief Toggles the group of LEDs based on the current state.
 *
 * Origin: LAB 1
 *
 * Group 0: Red and Blue
 * Group 1: Orange and Green
 *
 * @param group True if the group is 0, false if the group is 1.
 * @param current_state True for the first state, false for the second (doesn't really matter).
 */
void toggle_group(const bool group, const bool current_state);

/**
 * @brief Turns off the group of LEDs.
 *
 * Group 0: Red and Blue
 * Group 1: Orange and Green
 *
 * @param group True to turn off group 0, false to turn off group 1.
 *
 */
void group_off(const bool group);



#endif //LAB1_EMBEDDED_SYSTEMS_LED_CONTROLS_H
