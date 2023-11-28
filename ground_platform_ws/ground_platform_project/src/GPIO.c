/**
 * @file GPIO.c
 * @brief Source code for the GPIO LEDs driver.
 *
 * This file contains the function definitions for the GPIO driver.
 * It interfaces with the following:
 *  - LEDs of the TI MSP432 LaunchPad
 *
 * @author Aaron Nanas, Michael Granberry, Abdullah Hendy
 *
 */

#include "../inc/GPIO.h"
#include "../inc/Clock.h"

// Constant definitions for the built-in red LED
const uint8_t RED_LED_OFF           =   0x00;
const uint8_t RED_LED_ON            =   0x01;

// Constant definitions for the RGB LED colors
const uint8_t RGB_LED_OFF           =   0x00;
const uint8_t RGB_LED_RED           =   0x01;
const uint8_t RGB_LED_GREEN         =   0x02;
const uint8_t RGB_LED_YELLOW        =   0x03;
const uint8_t RGB_LED_BLUE          =   0x04;
const uint8_t RGB_LED_PINK          =   0x05;
const uint8_t RGB_LED_SKY_BLUE      =   0x06;
const uint8_t RGB_LED_WHITE         =   0x07;

void LED1_Init()
{
    P1->SEL0 &= ~0x01;
    P1->SEL1 &= ~0x01;
    P1->DIR |= 0x01;
    P1->OUT &= ~0x01;
}

uint8_t LED1_Output(uint8_t led_value)
{
    P1->OUT = (P1->OUT & 0xFE) | led_value;
    return ((P1->OUT != 0) ? 1 : 0);
}

void RGB_Init()
{
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DS |= 0x07;
    P2->DIR |= 0x07;
    P2->OUT &= ~0x07;
}

uint8_t RGB_Output(uint8_t led_value)
{
    P2->OUT = (P2->OUT & 0xF8) | led_value;
    return ((P2->OUT != 0) ? 1 : 0);
}

