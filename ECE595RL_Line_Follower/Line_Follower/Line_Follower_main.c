/**
 * @file Line_Follower_main.c
 * @brief Main source code for the Line_Follower program.
 *
 * This file contains the main entry point for the Line_Follower program.
 * The main controller demonstrates a Line Follower robot without using a specific algorithm.
 *
 * It interfaces the following peripherals using GPIO to demonstrate line following:
 *  - 8-Channel QTRX Sensor Array module
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (1 kHz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author Aaron Nanas
 *
 */

#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/GPIO.h"
#include "../inc/SysTick_Interrupt.h"
#include "../inc/EUSCI_A0_UART.h"
#include "../inc/Timer_A0_PWM.h"
#include "../inc/Timer_A1_Interrupt.h"
#include "../inc/Timer_A3_Capture.h"
#include "../inc/Motor.h"
#include "../inc/Tachometer.h"
#include "../inc/LPF.h"
#include "../inc/Analog_Distance_Sensor.h"
#include "../inc/Reflectance_Sensor.h"

#define CONTROLLER_1 1
//#define CONTROLLER_2 1

// Initialize constant PWM duty cycle values for the motors
#define PWM_NOMINAL         2500
#define PWM_SWING           1000
#define PWM_MIN             (PWM_NOMINAL - PWM_SWING)
#define PWM_MAX             (PWM_NOMINAL + PWM_SWING)

// Declare global variables used to update PWM duty cycle values for the motors
uint16_t Duty_Cycle_Left;
uint16_t Duty_Cycle_Right;

// Declare global variable used to store line sensor raw data
uint8_t Line_Sensor_Data;

// Declare global variable used to store line sensor position
int32_t Line_Sensor_Position;

// Global variable counter to keep track of the number of SysTick interrupts
// that have occurred. It increments in SysTick_Handler on each interrupt event.
uint32_t SysTick_counter = 0;

// Define the states for the Line Follower FSM
typedef enum
{
    CENTER  = 0,
    LEFT    = 1,
    RIGHT   = 2
} Line_Follower_State;

// Initialize the current state to CENTER
Line_Follower_State current_state = CENTER;

/**
 * @brief Implements the finite state machine (FSM) for a simple Line Follower robot.
 *
 * This function represents the FSM for the Line Follower robot. The robot's behavior is determined
 * by the current state, and it performs the following actions based on the state:
 * - CENTER: Moves forward and changes the RGB LED's color to green
 * - LEFT: Turns left and changes the RGB LED's color to blue
 * - RIGHT: Turns right and changes the RGB LED's color to yellow
 *
 * @return None
 */
void Line_Follower_FSM_1()
{
    switch(current_state)
    {
        case CENTER:
        {
            LED2_Output(RGB_LED_GREEN);
            Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }

        case LEFT:
        {
            LED2_Output(RGB_LED_BLUE);
            Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }

        case RIGHT:
        {
            LED2_Output(RGB_LED_YELLOW);
            Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }
    }
}

/**
 * @brief Implements the control logic for the Line Follower robot for a line-following robot based on sensor readings.
 *
 * This function is responsible for controlling the behavior of the Line Follower robot based on the values read from
 * from the 8-Channel QTRX Sensor Array module. It performs the following steps:
 *
 *  1. Increments the SysTick_counter by 1 every time the SysTick periodic interrupt occurs.
 *  2. Starts the process of reading the reflectance sensor array every 10 ms.
 *  3. Finishes reading the reflectance sensor array after 1 ms.
 *  4. Determines the position of the robot relative to the center of the line using Reflectance_Sensor_Position().
 *  5. Updates the current state of the robot based on the calculated position and predefined thresholds.
 *      - CENTER: The robot is at the center of the line (or very close to it).
 *      - RIGHT: The robot is on the left side of the line, and it needs to steer to the right to move back to the center.
 *      - LEFT: The robot is on the right side of the line, and it needs to steer to the left to move back to the center.
 *
 * @return None
 */
void Line_Follower_Controller_1()
{
    // Increment SysTick_counter by 1 every time the SysTick periodic interrupt occurs
    SysTick_counter = SysTick_counter + 1;

    // Start the process of reading the reflectance sensor array every 10 ms (i.e. 11, 21, 31, ...)
    if ((SysTick_counter % 10) == 1)
    {
        Reflectance_Sensor_Start();
    }

    // Finish reading the reflectance sensor sensor array after 1 ms (i.e. 12, 22, 32, ...)
    if ((SysTick_counter % 10) == 2)
    {
        Line_Sensor_Data = Reflectance_Sensor_End();
        Line_Sensor_Position = Reflectance_Sensor_Position(Line_Sensor_Data);

        // Check if the robot is at the center of the line (or very close to it)
        // Assign current_state to CENTER so that the robot will keep moving forward at the center
        if (Line_Sensor_Position > -47 && Line_Sensor_Position < 47)
        {
            current_state = CENTER;
        }

        // Check if the robot is on the left side of the line
        // Assign current_state to RIGHT to steer the robot to the right in order to move back to the center
        else if (Line_Sensor_Position >= 47 && Line_Sensor_Position < 332)
        {
            current_state = RIGHT;
            if (Line_Sensor_Position > -47 && Line_Sensor_Position < 47)
            {
                current_state = CENTER;
            }
        }

        // Check if the robot is on the right side of the line
        // Assign current_state to LEFT to steer the robot to the left in order to move back to the center
        else if (Line_Sensor_Position <= -47 && Line_Sensor_Position > -332)
        {
            current_state = LEFT;
            if (Line_Sensor_Position > -47 && Line_Sensor_Position < 47)
            {
                current_state = CENTER;
            }
        }

        // Otherwise, the robot will keep turning right at other positions (e.g. dead end)
        else
        {
            current_state = RIGHT;
        }
    }
}

/**
 * @brief
 *
 *
 * @return None
 */
void Line_Follower_Controller_2()
{

}


/**
 * @brief This function is the handler for the SysTick periodic interrupt with a rate of 1 kHz.
 *
 * The SysTick_Handler generates a periodic interrupt that calls a specific controller function based on the selected
 * active configuration. Only one of the options can be defined at a time: CONTROLLER_1 or CONTROLLER_2.
 *
 * @return None
 */
void SysTick_Handler(void)
{
#if defined CONTROLLER_1

    Line_Follower_Controller_1();

#elif defined CONTROLLER_2
    #if defined CONTROLLER_1
        #error "Only CONTROLLER_1 or CONTROLLER_2 can be active at the same time."
    #endif

    // Your function for Task 1 goes here (Line_Follower_Controller_2)
    Line_Follower_Controller_2();

#else
    #error "Define either one of the options: CONTROLLER_1 or CONTROLLER_2."
#endif
}

/**
 * @brief User-defined function executed by Timer A1 using a periodic interrupt at a rate of 1 kHz.
 *
 *
 * @return None
 */
void Timer_A1_Periodic_Task(void)
{
#if defined CONTROLLER_1

    Line_Follower_FSM_1();

#elif defined CONTROLLER_2
    #if defined CONTROLLER_1
        #error "Only CONTROLLER_1 or CONTROLLER_2 can be active at the same time."
    #endif

    // Your function for Task 1 goes here (Line_Follower_FSM_2)

#else
    #error "Define either one of the options: CONTROLLER_1 or CONTROLLER_2."
#endif
}

int main(void)
{
    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED
    LED1_Init();
    LED2_Init();

    // Initialize the buttons
    Buttons_Init();

    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();

    // Initialize Timer A1 periodic interrupt with a rate of 1 kHz
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Initialize the tachometers
    Tachometer_Init();

    // Initialize the motors
    Motor_Init();

    // Initialize the 8-Channel QTRX Reflectance Sensor Array module
    Reflectance_Sensor_Init();

    // Initialize motor duty cycle values
    Duty_Cycle_Left  = PWM_NOMINAL;
    Duty_Cycle_Right = PWM_NOMINAL;

    // Initialize SysTick periodic interrupt with a rate of 1 kHz
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Enable the interrupts used by Timer A1 and other modules
    EnableInterrupts();

    while(1)
    {

    }
}
