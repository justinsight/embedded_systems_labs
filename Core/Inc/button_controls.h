/**
 * Contains the controls for the USER button on the STM32 Curiosity Board.
 *
 * @author Justin Newkirk
 * @date 2/13/2024
 */

#ifndef LAB1_EMBEDDED_SYSTEMS_BUTTON_CONTROLS_H
#define LAB1_EMBEDDED_SYSTEMS_BUTTON_CONTROLS_H

// Includes =====================================================================

#include "stm32f072xb.h"
#include "stm32f0xx.h"
#include <stdbool.h>

// Macros =======================================================================

#define HELD_HIGH           0xFFFFFFFF
#define TRANSITION_HIGH     0x7FFFFFFF
#define HELD_LOW            0x00000000

// Implementation Functions =====================================================

/**
 * @brief Will initialize the USER button for use.
 *
 */
void initialize_user_button( void );

// Helper Functions =============================================================

/**
 * @brief This function handles the User Button press interrupt.
 *
 * This will handle the interrupt from the core on the EXTI0_1 lines,
 * of which the EXTI0 line is connected to the user button.
 *
 * Origin: LAB 2
 *
 * @note If we want to see other, pregenerated interrupt handlers, we can look
 * at the stm32f0xx_it.c file.
 */
void EXTI0_1_IRQHandler( void );

/**
 * @brief This will detect whether the button has been pressed or not.
 *
 * This will be used to detect whether the button has been pressed or not.
 * Will maintain a static variable to keep track of the button's state
 * and implement a de-bouncer to prevent multiple, incorrect readings.
 *
 * Origin: LAB 1
 *
 * @return True if the button has been pressed, false otherwise.
 *
 * @note This function must be continuously called to detect the button press.
 */
bool user_button_pressed_GPIO_variant( void );


#endif //LAB1_EMBEDDED_SYSTEMS_BUTTON_CONTROLS_H
