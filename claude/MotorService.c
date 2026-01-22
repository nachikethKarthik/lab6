/****************************************************************************
 Module
   MotorService.c

 Revision
   1.0.0

 Description
   This service implements PWM motor control using OC4 and Timer2.
   Motor speed is controlled by a potentiometer read via ADC at 10Hz.

 Notes
   - PWM output on RB13 (pin 24) via OC4
   - Potentiometer input on AN0/RA0 (pin 2)
   - Direction control on RB2 (1A) and RB3 (2A)
   - PWM frequency approximately 200Hz

 History
 When           Who     What/Why
 -------------- ---     --------
 01/21/26       Student  Lab 6 Part 1
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MotorService.h"
#include "PIC32_AD_Lib.h"

#include <xc.h>
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/
// PWM Configuration
// PBCLK = 20 MHz, Target PWM freq = 200 Hz
// With prescaler 1:4: PWM Period = (PR2 + 1) * 4 / 20MHz
// For 200 Hz: Period = 5ms = (PR2 + 1) * 200ns
// PR2 + 1 = 5ms / 200ns = 25000, so PR2 = 24999
#define PWM_PERIOD          24999
#define PWM_PRESCALE        0b010       // 1:4 prescaler

// ADC update rate (10 Hz = 100ms)
#define ADC_UPDATE_TIME     100

// Timer assignment (must match ES_Configure.h)
#define MOTOR_TIMER         0

// Direction pin definitions
#define DIR_1A_TRIS         TRISBbits.TRISB2
#define DIR_2A_TRIS         TRISBbits.TRISB3
#define DIR_1A              LATBbits.LATB2
#define DIR_2A              LATBbits.LATB3

/*---------------------------- Module Functions ---------------------------*/
static void InitPWM(void);
static void InitDirectionPins(void);
static void SetDutyCycle(uint32_t dutyCycle);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitMotorService

 Parameters
     uint8_t Priority - the priority of this service

 Returns
     bool - true if successful initialization

 Description
     Initializes PWM, ADC, direction pins, and starts the periodic timer
****************************************************************************/
bool InitMotorService(uint8_t Priority)
{
    ES_Event_t ThisEvent;
    
    MyPriority = Priority;
    
    // Initialize direction control pins
    InitDirectionPins();
    
    // Initialize ADC for potentiometer on AN0
    ADC_ConfigAutoScan(BIT0HI);
    
    // Initialize PWM using OC4 and Timer2
    InitPWM();
    
    // Start the periodic ADC read timer (10 Hz)
    ES_Timer_InitTimer(MOTOR_TIMER, ADC_UPDATE_TIME);
    
    // Post the initial transition event
    ThisEvent.EventType = ES_INIT;
    if (ES_PostToService(MyPriority, ThisEvent) == true)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/****************************************************************************
 Function
     PostMotorService

 Parameters
     ES_Event_t ThisEvent - the event to post to the queue

 Returns
     bool - true if successful, false otherwise
****************************************************************************/
bool PostMotorService(ES_Event_t ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
     RunMotorService

 Parameters
     ES_Event_t ThisEvent - the event to process

 Returns
     ES_Event_t - ES_NO_EVENT if no error

 Description
     Handles timeout events to read ADC and update PWM duty cycle
****************************************************************************/
ES_Event_t RunMotorService(ES_Event_t ThisEvent)
{
    ES_Event_t ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT;
    
    switch (ThisEvent.EventType)
    {
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == MOTOR_TIMER)
            {
                // Read potentiometer value from ADC
                uint32_t adcResult[1];
                ADC_MultiRead(adcResult);
                
                // Scale ADC value (0-1023) to duty cycle (0-PWM_PERIOD)
                uint32_t newDutyCycle = (adcResult[0] * PWM_PERIOD) / 1023;
                
                // Update PWM duty cycle
                SetDutyCycle(newDutyCycle);
                
                // Restart timer for next reading
                ES_Timer_InitTimer(MOTOR_TIMER, ADC_UPDATE_TIME);
            }
            break;
            
        case ES_INIT:
            // Initialization complete - nothing special needed
            break;
            
        default:
            break;
    }
    
    return ReturnEvent;
}

/***************************************************************************
 Private Functions
 ***************************************************************************/

/****************************************************************************
 Function
     InitDirectionPins

 Description
     Configures RB2 and RB3 as digital outputs for motor direction control
****************************************************************************/
static void InitDirectionPins(void)
{
    // Disable analog function on RB2 and RB3 (they are AN4 and AN5)
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    
    // Set RB2 and RB3 as outputs
    DIR_1A_TRIS = 0;
    DIR_2A_TRIS = 0;
    
    // Set initial direction (forward: 1A=1, 2A=0)
    DIR_1A = 1;
    DIR_2A = 0;
}

/****************************************************************************
 Function
     InitPWM

 Description
     Configures Timer2 and OC4 for PWM output on RB13 at ~200Hz
****************************************************************************/
static void InitPWM(void)
{
    // ===== Timer2 Setup (PWM time base) =====
    
    // Step 1: Disable timer during setup
    T2CON = 0x0000;
    
    // Step 2: Select internal PBCLK source
    T2CONbits.TCS = 0;
    
    // Step 3: Select prescaler (1:4)
    T2CONbits.TCKPS = PWM_PRESCALE;
    
    // Step 4: Clear timer register
    TMR2 = 0;
    
    // Step 5: Load period register for ~200Hz
    PR2 = PWM_PERIOD;
    
    // Step 6: Clear T2IF interrupt flag
    IFS0CLR = _IFS0_T2IF_MASK;
    
    // (Not using Timer2 interrupt, so skip IPC/IEC setup)
    
    // ===== Output Compare 4 Setup =====
    
    // Disable OC4 during setup
    OC4CON = 0x0000;
    
    // Set initial duty cycle (0% - motor stopped)
    OC4R = 0;
    OC4RS = 0;
    
    // Map OC4 output to RB13 (pin 24)
    // From PPS table: RPB13R value 0b0101 = OC4
    RPB13R = 0b0101;
    
    // Configure OC4 for PWM mode
    OC4CONbits.OCTSEL = 0;      // Use Timer2 as clock source
    OC4CONbits.OCM = 0b110;     // PWM mode, fault pin disabled
    
    // ===== Enable Timer2 and OC4 =====
    
    // Step 7: Enable Timer2
    T2CONbits.ON = 1;
    
    // Enable OC4
    OC4CONbits.ON = 1;
}

/****************************************************************************
 Function
     SetDutyCycle

 Parameters
     uint32_t dutyCycle - duty cycle value (0 to PWM_PERIOD)

 Description
     Updates the PWM duty cycle by writing to OC4RS (double-buffered)
****************************************************************************/
static void SetDutyCycle(uint32_t dutyCycle)
{
    // Clamp duty cycle to valid range
    if (dutyCycle > PWM_PERIOD)
    {
        dutyCycle = PWM_PERIOD;
    }
    
    // Write to OC4RS (double-buffered, updates on next period match)
    OC4RS = dutyCycle;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
