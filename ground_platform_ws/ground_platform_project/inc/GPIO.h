/**
 * @file GPIO.h
 * @brief Header file for the GPIO driver.
 *
 * This file contains the function definitions for the GPIO driver.
 * It interfaces with the following:
 *  - User buttons and LEDs of the TI MSP432 LaunchPad
 *
 * @author Aaron Nanas, Michael Granberry, Abdullah Hendy
 *
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>
#include "msp.h"

// Constant definitions for the built-in red LED
extern const uint8_t RED_LED_OFF;
extern const uint8_t RED_LED_ON;

// Constant definitions for the RGB LED colors
extern const uint8_t RGB_LED_OFF;
extern const uint8_t RGB_LED_RED;
extern const uint8_t RGB_LED_GREEN;
extern const uint8_t RGB_LED_YELLOW;
extern const uint8_t RGB_LED_BLUE;
extern const uint8_t RGB_LED_PINK;
extern const uint8_t RGB_LED_SKY_BLUE;
extern const uint8_t RGB_LED_WHITE;

// Constant definitions for the PMOD 8LD module
extern const uint8_t PMOD_8LD_ALL_OFF;
extern const uint8_t PMOD_8LD_ALL_ON;
extern const uint8_t PMOD_8LD_0_3_ON;
extern const uint8_t PMOD_8LD_4_7_ON;

/**
 * @brief The LED1_Init function initializes the built-in red LED (P1.0).
 *
 * This function initializes the built-in red LED located at pin P1.0
 * and configures it as a GPIO pin. It sets the direction of the pin as output.
 *
 * @param None
 *
 * @return None
 */
void LED1_Init();

/**
 * @brief The LED1_Output function sets the output of the built-in red LED and returns the status.
 *
 * This function sets the output of the built-in red LED based on the value of the input, led_value.
 * A bitwise AND operation (& 0xFE) is performed to mask the first bit (LSB) of the output register
 * to preserve the state of other pins connected to Port 1 while keeping the LED pin unaffected.
 * Then, a bitwise OR operation is performed with led_value to set the LED pin to the desired state
 * specified by led_value.
 *
 * @param led_value An 8-bit unsigned integer that determines the output of the built-in red LED. To turn off
 *                  the LED, set led_value to 0. Otherwise, setting led_value to 1 turns on the LED.
 *
 * @return Indicates the status of the LED.
 *         - 0: LED Off
 *         - 1: LED On
 */
uint8_t LED1_Output(uint8_t led_value);

/**
 * @brief The LED2_Init function initializes the RGB LED (P2.0 - P2.2).
 *
 * This function initializes the following RGB LED, configures the pins as GPIO pins with high drive strength,
 * and sets the direction of the pins as output. The RGB LED is off by default upon initialization.
 *  - RGBLED_RED      (P2.0)
 *  - RGBLED_GREEN    (P2.1)
 *  - RGBLED_BLUE     (P2.2)
 *
 * @param None
 *
 * @return None
 */
void RGB_Init();

/**
 * @brief The LED2_Output function sets the output of the RGB LED and returns the status.
 *
 * This function sets the output of the RGB LED based on the value of the input, led_value.
 * A bitwise AND operation (& 0xF8) is performed to mask the lower three bits of the output register
 * to preserve the state of other pins connected to Port 2 while keeping the RGB LED pin unaffected.
 * Then, a bitwise OR operation is performed with led_value to set the RGB LED pin to the desired state
 * specified by led_value.
 *
 * @param led_value An 8-bit unsigned integer that determines the output of the RGB LED. To turn off
 *                  the RGB LED, set led_value to 0. The following values determine the color of the RGB LED:
 *
 *  Color       LED(s)   led_value
 *  Off         ---         0x00
 *  Red         R--         0x01
 *  Green       -G-         0x02
 *  Yellow      RG-         0x03
 *  Blue        --B         0x04
 *  Pink        R-B         0x05
 *  Sky Blue    -GB         0x06
 *  White       RGB         0x07
 *
 * @return Indicates the status of the RGB LED.
 *          - 0: RGB LED Off
 *          - 1: RGB LED On
 */
uint8_t RGB_Output(uint8_t led_value);

#endif /* GPIO_H_ */
