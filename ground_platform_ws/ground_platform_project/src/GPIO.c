/**
 * @file GPIO.c
 * @brief Source code for the GPIO driver.
 *
 * This file contains the function definitions for the GPIO driver.
 * It interfaces with the following:
 *  - User buttons and LEDs of the TI MSP432 LaunchPad
 *  - PMOD SWT (4 Slide Switches)
 *  - PMOD 8LD (8 LEDs)
 *
 * To verify the pinout of the user buttons and LEDs, refer to the MSP432P401R SimpleLink Microcontroller LaunchPad Development Kit User's Guide
 * Link: https://docs.rs-online.com/3934/A700000006811369.pdf
 *
 * For more information regarding the PMODs used in this lab, visit the following links:
 *  - PMOD SWT: https://digilent.com/reference/pmod/pmodswt/reference-manual
 *  - PMOD 8LD: https://digilent.com/reference/pmod/pmod8ld/reference-manual
 *
 * @note The user buttons, located at P1.1 and P1.4, are configured with negative logic
 * as the default setting. When the buttons are pressed, they connect to GND. Refer to the
 * schematic found in the MSP432P401R LaunchPad User's Guide.
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

// Constant definitions for the PMOD 8LD module
const uint8_t PMOD_8LD_ALL_OFF      =   0x00;
const uint8_t PMOD_8LD_ALL_ON       =   0xFF;
const uint8_t PMOD_8LD_0_3_ON       =   0x0F;
const uint8_t PMOD_8LD_4_7_ON       =   0xF0;

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

void LED2_Toggle(uint8_t led_value)
{
    P2->OUT ^= led_value;
}

void Buttons_Init()
{
    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;
    P1->DIR |= ~0x12;
    P1->REN |= 0x12;
    P1->OUT |= 0x12;
}

uint8_t Get_Buttons_Status()
{
    uint8_t button_status = (P1->IN & 0x12);
    return button_status;
}

void PMOD_8LD_Init()
{
    P9->SEL0 &= ~0xFF;
    P9->SEL1 &= ~0xFF;
    P9->DS |= 0xFF;
    P9->DIR |= 0xFF;
    P9->OUT &= ~0xFF;
}

uint8_t PMOD_8LD_Output(uint8_t led_value)
{
    P9->OUT = led_value;
    uint8_t PMOD_8LD_value = P9->OUT;
    return PMOD_8LD_value;
}

void PMOD_SWT_Init()
{
    P10->SEL0 &= ~0xF;
    P10->SEL1 &= ~0xF;
    P10->DIR &= ~0xF;
}

uint8_t PMOD_SWT_Status()
{
    uint8_t switch_status = P10->IN & 0xF;
    return switch_status;
}

void LED_Pattern_1(uint8_t button_status)
{
    switch(button_status)
    {
        // Button 1 and Button 2 are pressed
        case 0x00:
        {
            LED1_Output(RED_LED_ON);
            RGB_Output(RGB_LED_RED);
            PMOD_8LD_Output(PMOD_8LD_ALL_ON);
            break;
        }

        // Button 1 is pressed
        // Button 2 is not pressed
        case 0x10:
        {
            LED1_Output(RED_LED_ON);
            RGB_Output(RGB_LED_OFF);
            PMOD_8LD_Output(PMOD_8LD_0_3_ON);
            break;
        }

        // Button 1 is not pressed
        // Button 2 is pressed
        case 0x02:
        {
            LED1_Output(RED_LED_OFF);
            RGB_Output(RGB_LED_GREEN);
            PMOD_8LD_Output(PMOD_8LD_4_7_ON);
            break;
        }

        // Button 1 and Button 2 are not pressed
        case 0x12:
        {
            LED1_Output(RED_LED_OFF);
            RGB_Output(RGB_LED_OFF);
            PMOD_8LD_Output(PMOD_8LD_ALL_OFF);
            break;
        }
    }
}

void LED_Pattern_2()
{
    LED1_Output(RED_LED_ON);
    RGB_Output(RGB_LED_RED);

    for (uint8_t led_count = 0; led_count <= 0xFF; led_count++)
    {
        PMOD_8LD_Output(led_count);
        Clock_Delay1ms(100);
        uint8_t switch_status = PMOD_SWT_Status();
        if (switch_status != 0x01)
        {
            break;
        }
    }
}

void LED_Controller(uint8_t button_status, uint8_t switch_status)
{
    switch(switch_status)
    {
        case 0x00:
        {
            LED_Pattern_1(button_status);
        }
        break;

        case 0x01:
        {
            LED_Pattern_2();
        }
        break;

        default:
        {
            LED_Pattern_1(button_status);
        }
    }
}

void P8_Init()
{
    P8->SEL0 &= ~0xE1;
    P8->SEL1 &= ~0xE1;
    P8->DIR |= 0xE1;
    P8->OUT &= ~0xE1;
}
