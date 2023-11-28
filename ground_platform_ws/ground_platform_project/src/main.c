/**
 * @file main.c
 * @brief Source code of the final demo. 
 *
 * This file contains the driver initializations as well the main state machine.
 * The main state machine is always listening until an ASCII character is received.
 * The state machine changes states: based on the received ASCII character, each character-specific task is executed.
 *
 * @author Michael Granberry, Abdullah Hendy
 *
 */

#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/GPIO.h"
#include "../inc/EUSCI_A0_UART.h"
#include "../inc/EUSCI_A2_UART.h"
#include "../inc/Timer_A0_PWM.h"
#include "../inc/Timer_A1_Interrupt.h"
#include "../inc/Timer_A3_Capture.h"
#include "../inc/Motor.h"
#include "../inc/LPF.h"
#include "../inc/Analog_Distance_Sensor.h"
#include "../inc/SysTick_Interrupt.h"

// Controller
//#define DEBUG_ACTIVE    1

// Initialize constant distance values (in mm)
#define TOO_CLOSE_DISTANCE_GREEN  150
#define TOO_CLOSE_DISTANCE_YELLOW 300
#define TOO_CLOSE_DISTANCE_RED 150
#define DESIRED_DISTANCE    500

// Initialize constant PWM duty cycle values for the motors
#define PWM_NOMINAL         2500
#define PWM_SWING           1000
#define PWM_MIN             (PWM_NOMINAL - PWM_SWING)
#define PWM_MAX             (PWM_NOMINAL + PWM_SWING)

// Mian FSM ------------------------------------------------
enum State {
    IDLE,
    GREEN,
    RED,
    YELLOW,
    ERROR
};
typedef enum State state;
state  curr_state = IDLE;

// Yellow state ---------------------------------------------
enum Yellow_State
{
    Y_STRAIGHT,
    Y_AVOID
};
typedef enum Yellow_State yellow_state;
yellow_state yellow_curr_state = Y_STRAIGHT;

enum Drive_Mode
{
    FOLLOW,
    AVOID,
    TURN_LEFT,
    TURN_RIGHT,
    NO_TURN
};
uint16_t yellow_avoid_counter = 0;
int8_t turn_flag = -1;

// Red state -------------------------------------------------
enum Red_State
{
    R_STRAIGHT,
    R_BACKWARDS,
    R_FIGURE_8,
    R_COMPLETE
};
typedef enum Red_State red_state;
red_state red_curr_state = R_STRAIGHT;

// Declare global variables used to store filtered distance values from the Analog Distance Sensor
uint32_t Filtered_Distance_Left;
uint32_t Filtered_Distance_Center;
uint32_t Filtered_Distance_Right;

// Declare global variables used to store converted distance values from the Analog Distance Sensor
int32_t Converted_Distance_Left;
int32_t Converted_Distance_Center;
int32_t Converted_Distance_Right;

// Declare global variable used to store the amount of error
int32_t Error;

// Proportional Controller Gain
int32_t Kp = 4;

// Initialize set point to 250 mm
int32_t Set_Point = 250;

// Declare global variables used to update PWM duty cycle values for the motors
uint16_t Duty_Cycle_Left;
uint16_t Duty_Cycle_Right;

// Command Flags
int complete_flag = 0;

void FSM(void);
void Stop_WatchDog(void);
void DetermineAction(uint8_t avoid_follow_flag);
void Action(uint8_t avoid_follow_flag);
void Green_Task();
void Yellow_Task();
void Red_Task();
void Listening(uint8_t received_data);
void Sample_Analog_Distance_Sensor();
void Timer_A1_Periodic_Task(void);



int main(void)
{
    // Declare variables for Analog Distance Sensor
    uint32_t Raw_A17;
    uint32_t Raw_A14;
    uint32_t Raw_A16;

    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Stop watch dog
    Stop_WatchDog();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED
    LED1_Init();

    // Init RGB
    RGB_Init();

    // Init all UARTs
    EUSCI_A0_UART_Init();
    EUSCI_A2_UART_Init();

    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();

    // Initialize Timer A1 periodic interrupt with a rate of 2 kHz
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Init Motor
    Motor_Init();

    // Initialize motor duty cycle values
    Duty_Cycle_Left  = PWM_NOMINAL;
    Duty_Cycle_Right = PWM_NOMINAL;

    // Initialize the Analog Distance Sensor using ADC14
    Analog_Distance_Sensor_Init();

    // Start conversion of Analog Distance Sensor raw values
    Analog_Distance_Sensor_Start_Conversion(&Raw_A17, &Raw_A14, &Raw_A16);

    // Initialize low-pass filters for the Analog Distance Sensor
    LPF_Init(Raw_A17, 64);
    LPF_Init2(Raw_A14, 64);
    LPF_Init3(Raw_A16, 64);

    // SysTic Init
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Enable the interrupts used by Timer A1 and other modules
    EnableInterrupts();

    while(1){
        FSM();
//        printf("red state: %d\r\n", red_curr_state);
//        printf("Left: %d mm | Center: %d mm | Right: %d mm\n", Converted_Distance_Left, Converted_Distance_Center, Converted_Distance_Right);
#ifdef DEBUG_ACTIVE
        printf("state: %d\r\n", curr_state);
        printf("yellow state: %d\r\n", yellow_curr_state);
        printf("red state: %d\r\n", red_curr_state);
        printf("turn_flag: %d\r\n", turn_flag);
        printf("Left: %d mm | Center: %d mm | Right: %d mm\n", Converted_Distance_Left, Converted_Distance_Center, Converted_Distance_Right);
        Clock_Delay1ms(100);
#endif
    }
}

void FSM(void) {
    switch (curr_state) {
        case IDLE:
            yellow_curr_state = Y_STRAIGHT;
            complete_flag = 0;
            yellow_avoid_counter = 0;
            RGB_Output(RGB_LED_WHITE);
            Motor_Stop();
            Listening(EUSCI_A2_UART_InChar());
//            Listening('r'); // Testing: Hard code any value: 'g', 'y', 'r', 'p', or anything else.
            break;

        case GREEN:
            if (!complete_flag) {
                RGB_Output(RGB_LED_GREEN);
                Green_Task();
            } else {
                curr_state = IDLE;
            }
            break;

        case YELLOW:
            if (!complete_flag) {
                RGB_Output(RGB_LED_YELLOW);
                Yellow_Task();
            } else {
                curr_state = IDLE;
            }
            break;

        case RED:
            if (!complete_flag) {
                RGB_Output(RGB_LED_RED);
                Red_Task();
            } else {
                curr_state = IDLE;
            }
            break;

        default:
            RGB_Output(RGB_LED_PINK);
            Motor_Stop();
            break;
    }
}

void Stop_WatchDog(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
}

void Listening(uint8_t received_data){
    if (received_data == 'g') {
        curr_state = GREEN;

    } else if  (received_data == 'y'){
        curr_state = YELLOW;

    } else if  (received_data == 'r'){
        curr_state = RED;

    } else if  (received_data == 'p'){
        curr_state = IDLE;

    } else {
        curr_state = IDLE;
    }
}

// Go straight and detect cone (object follower-ish)
void Green_Task()
{
    if ((Converted_Distance_Left < TOO_CLOSE_DISTANCE_GREEN) || (Converted_Distance_Center < TOO_CLOSE_DISTANCE_GREEN) || (Converted_Distance_Right < TOO_CLOSE_DISTANCE_GREEN)) {
        complete_flag = 1;
    } else {
        Action(FOLLOW);
    }
}

// Drive and avoid
void Yellow_Task() {
    if (yellow_curr_state == Y_STRAIGHT)
    {
        if ((Converted_Distance_Left < TOO_CLOSE_DISTANCE_YELLOW) || (Converted_Distance_Center < TOO_CLOSE_DISTANCE_YELLOW) || (Converted_Distance_Right < TOO_CLOSE_DISTANCE_YELLOW))
        {
            yellow_curr_state = Y_AVOID;
            yellow_avoid_counter = 0;
        }
        else
        {
            DetermineAction(FOLLOW);
        }
    } else if (yellow_curr_state == Y_AVOID) {
        DetermineAction(AVOID);
    }
}

// Rotate in place
void Red_Task() {

    switch (red_curr_state) {
        case R_STRAIGHT:
            if ((Converted_Distance_Left < TOO_CLOSE_DISTANCE_RED) || (Converted_Distance_Center < TOO_CLOSE_DISTANCE_RED) || (Converted_Distance_Right < TOO_CLOSE_DISTANCE_RED)) {
                red_curr_state = R_BACKWARDS;
            } else {
                Action(FOLLOW);
            }
            break;

        case R_BACKWARDS:
#ifndef DEBUG_ACTIVE
            Motor_Backward(4000,4000);
#endif
            Clock_Delay1ms(5000);
#ifndef DEBUG_ACTIVE
            Motor_Stop();
#endif
            Clock_Delay1ms(500);
            red_curr_state = R_FIGURE_8;
            break;

        case R_FIGURE_8:
    // rotate 90 degrees
#ifndef DEBUG_ACTIVE
            Motor_Left(3000, 3000);
#endif

            Clock_Delay1ms(700);

    // semi circle
#ifndef DEBUG_ACTIVE
            Motor_Forward(5000, 3000);
#endif

            Clock_Delay1ms(2700);

   // full circle
#ifndef DEBUG_ACTIVE
             Motor_Forward(3000, 5000);
#endif
//
             Clock_Delay1ms(5600);
//
   // semi circle
#ifndef DEBUG_ACTIVE
             Motor_Forward(5000, 3000);
#endif
//
             Clock_Delay1ms(2700);

             // rotate 90 degrees
         #ifndef DEBUG_ACTIVE
                     Motor_Right(3000, 3000);
         #endif

                     Clock_Delay1ms(700);
            red_curr_state = R_COMPLETE;
            break;

        case R_COMPLETE:
            // complete flag
            complete_flag = 1;
            break;
    }
}

void DetermineAction(uint8_t avoid_follow_flag) {
    if ((Converted_Distance_Center < Converted_Distance_Left) || (Converted_Distance_Center < Converted_Distance_Right)) {
        if(turn_flag == TURN_RIGHT) {
#ifndef DEBUG_ACTIVE
            Motor_Right(3000, 3000);
#endif
        } else if (turn_flag == TURN_LEFT){
#ifndef DEBUG_ACTIVE
            Motor_Left(3000, 3000);
#endif
        } else if (turn_flag == NO_TURN){
#ifndef DEBUG_ACTIVE
            Motor_Left(3000, 3000);
#endif
        }
    } else  {
        Action(avoid_follow_flag);
    }
}

void Action(uint8_t avoid_follow_flag) {

    // Check if both the left and right distance sensor readings are greater than the desired distance
    if ((Converted_Distance_Left > DESIRED_DISTANCE) && (Converted_Distance_Right > DESIRED_DISTANCE))
    {
        // Calculate the set point as the average of the left and right sensor distance readings
        Set_Point = (Converted_Distance_Left + Converted_Distance_Right) / 2;
    }
    else
    {
        // If at least one distance sensor reading is below the desired distance, assign the set point to the desired distance
        Set_Point = DESIRED_DISTANCE;
    }

    // Calculate the error based on the sensor readings
    if (Converted_Distance_Left < Converted_Distance_Right)
    {
        Error = Converted_Distance_Left - Set_Point;
    }
    else
    {
        Error = Set_Point - Converted_Distance_Right;
    }

    if (avoid_follow_flag == FOLLOW) {
        // Calculate the new duty cycle for the right motor based on the error and proportional constant (Kp)
        Duty_Cycle_Right = PWM_NOMINAL - (Kp * Error);

        // Calculate the new duty cycle for the left motor based on the error and proportional constant (Kp)
        Duty_Cycle_Left = PWM_NOMINAL + (Kp * Error);

    } else if (avoid_follow_flag == AVOID){
        // Calculate the new duty cycle for the right motor based on the error and proportional constant (Kp)
        Duty_Cycle_Right = PWM_NOMINAL + (Kp * Error);

        // Calculate the new duty cycle for the left motor based on the error and proportional constant (Kp)
        Duty_Cycle_Left = PWM_NOMINAL - (Kp * Error);
    }

    // Ensure that the duty cycle for the right motor does not go below the minimum PWM value
    if (Duty_Cycle_Right < PWM_MIN)
        Duty_Cycle_Right = PWM_MIN;

    // Ensure that the duty cycle for the right motor does not exceed the maximum PWM value
    if (Duty_Cycle_Right > PWM_MAX)
        Duty_Cycle_Right = PWM_MAX;

    // Ensure that the duty cycle for the left motor does not go below the minimum PWM value
    if (Duty_Cycle_Left < PWM_MIN)
        Duty_Cycle_Left = PWM_MIN;

    // Ensure that the duty cycle for the left motor does not exceed the maximum PWM value
    if (Duty_Cycle_Left > PWM_MAX)
        Duty_Cycle_Left = PWM_MAX;

#ifndef DEBUG_ACTIVE
    // Apply the updated PWM duty cycle values to the motors
    Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);

#endif
    // Determine previous turn
    // Related to DetermineAction();
    // Determines robots rotation
    if (Duty_Cycle_Left > Duty_Cycle_Right) { // if right
        turn_flag = TURN_RIGHT;
    } else if (Duty_Cycle_Left < Duty_Cycle_Right) { // if left
        turn_flag = TURN_LEFT;
    } else {
        turn_flag = NO_TURN;
    }
}

void SysTick_Handler(void)
{
    if (yellow_curr_state == Y_AVOID) {
        if (yellow_avoid_counter < 2000) {
            yellow_avoid_counter++;
        } else {
            complete_flag = 1;
            yellow_avoid_counter = 0;
        }
    }
}


void Sample_Analog_Distance_Sensor()
{
    // Declare variables for Analog Distance Sensor
    uint32_t Raw_A17;
    uint32_t Raw_A14;
    uint32_t Raw_A16;

    // Start conversion of Analog Distance Sensor raw values
    Analog_Distance_Sensor_Start_Conversion(&Raw_A17, &Raw_A14, &Raw_A16);

    // Apply low-pass filter to raw values
    Filtered_Distance_Right = LPF_Calc(Raw_A17);
    Filtered_Distance_Center = LPF_Calc2(Raw_A14);
    Filtered_Distance_Left = LPF_Calc3(Raw_A16);

    // Convert filtered distance values using calibration formula
    Converted_Distance_Left = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Left);
    Converted_Distance_Center = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Center);
    Converted_Distance_Right = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Right);
}


void Timer_A1_Periodic_Task(void)
{
    Sample_Analog_Distance_Sensor();
}
