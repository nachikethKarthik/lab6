/****************************************************************************
 Module
   MotorService.c

 Revision
   2.0.0

 Description
   This service implements PWM motor control using OC4 and Timer2.
   Motor speed is controlled by a potentiometer read via ADC at 10Hz.
   
   Part 2 additions:
   - Input Capture (IC2) on RB9 to measure encoder period
   - Bar graph display on RA1, RA2, RA3, RA4, RB8, RB10, RB11, RB15
   - RPM output to terminal at 10Hz
   - Timing measurement pin on RB5

 Notes
   - PWM output on RB13 (pin 24) via OC4
   - Potentiometer input on AN0/RA0 (pin 2)
   - Direction control on RB2 (1A) and RB3 (2A)
   - Encoder input on RB9 (pin 18) via IC2
   - PWM frequency approximately 200Hz for part 1

 History
 When           Who     What/Why
 -------------- ---     --------
 01/21/2026     karthi24    Added Part 2 encoder and bar graph
 01/21/2026     karthi24    started conversion from template file
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MotorService.h"
#include "PIC32_AD_Lib.h"
#include "dbprintf.h"

#include <xc.h>
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/
// PWM Configuration
// PBCLK = 20 MHz, Target PWM freq = 200 Hz
// With prescaler 1:4: PWM Period = (PR2 + 1) * PRESCALER / 20MHz
// For 200 Hz: Period = 5ms = (PR2 + 1) * 200ns
// PR2 + 1 = 5ms / 200ns = 25000, so PR2 = 24999
#define PWM_PERIOD          24999
#define PWM_PRESCALE        0b010       // 1:4 prescaler

// ADC update rate 10 Hz
#define ADC_UPDATE_TIME     100

// Direction pin definitions
#define DIR_1A_TRIS         TRISBbits.TRISB2
#define DIR_1A_ANSEL        ANSELBbits.ANSB2
#define DIR_1A              LATBbits.LATB2

#define DIR_2A_TRIS         TRISBbits.TRISB3
#define DIR_2A_ANSEL        ANSELBbits.ANSB3
#define DIR_2A              LATBbits.LATB3

// PWM pin definition
#define PWM_PIN_TRIS        TRISBbits.TRISB13
#define PWM_PIN_ANSEL       ANSELBbits.ANSB13

// ===== Part 2 Defines =====

// Timer3 prescaler for input capture (1:8)
#define IC_TIMER_PRESCALE   0b011

// Bar graph scaling thresholds (in timer ticks)
// Tune these based on observed motor speed range
// Timer3 tick = 0.4 us with 1:8 prescaler at 20MHz PBCLK
#define MIN_PERIOD          200     // Fast speed (tune this)
#define MAX_PERIOD          3000    // Slow speed (tune this)
#define PERIOD_RANGE        (MAX_PERIOD - MIN_PERIOD)

// RPM calculation constant
// Output wheel: 512 pulses/rev * 5.9 gearbox = 3020.8 pulses/rev
// Timer tick = 0.4 us, so frequency = 2,500,000 ticks/sec
// RPM = (2,500,000 / Period) / 3020.8 * 60 = 49,656 / Period
#define RPM_NUMERATOR       49656

// Timing pin (RB5) for scope measurement
#define TIMING_PIN_TRIS     TRISBbits.TRISB5
#define TIMING_PIN          LATBbits.LATB5

// Bar graph pin definitions
// Top to bottom: RA1, RA2, RA3, RA4, RB8, RB10, RB11, RB15
#define BAR1_TRIS           TRISAbits.TRISA1
#define BAR1                LATAbits.LATA1

#define BAR2_TRIS           TRISAbits.TRISA2
#define BAR2                LATAbits.LATA2

#define BAR3_TRIS           TRISAbits.TRISA3
#define BAR3                LATAbits.LATA3

#define BAR4_TRIS           TRISAbits.TRISA4
#define BAR4                LATAbits.LATA4

#define BAR5_TRIS           TRISBbits.TRISB8
#define BAR5                LATBbits.LATB8

#define BAR6_TRIS           TRISBbits.TRISB10
#define BAR6                LATBbits.LATB10

#define BAR7_TRIS           TRISBbits.TRISB11
#define BAR7                LATBbits.LATB11

#define BAR8_TRIS           TRISBbits.TRISB15
#define BAR8_ANSEL          ANSELBbits.ANSB15   // RB15 is AN9
#define BAR8                LATBbits.LATB15

/*---------------------------- Module Functions ---------------------------*/
static void InitPWM(void);
static void InitDirectionPWMPins(void);
static void SetDutyCycle(uint32_t dutyCycle);
static void InitInputCapture(void);
static void InitBarGraph(void);
static void InitTimingPin(void);
static void UpdateBarGraph(uint32_t period);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;

// Shared with ISR (must be volatile)
static volatile uint32_t CurrentPeriod = 0;     // Latest measured period in ticks
static uint32_t LastCapture = 0;                // Previous capture value (ISR only)

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitMotorService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and Initializes PWM, ADC, direction pins, 
     input capture, bar graph, and starts the periodic timer
****************************************************************************/
bool InitMotorService(uint8_t Priority)
{
    ES_Event_t ThisEvent;

    MyPriority = Priority;

    // Initialize direction control pins
    InitDirectionPWMPins();
    
    // Initialize ADC for potentiometer on AN0
    ADC_ConfigAutoScan(BIT0HI);
    
    // Initialize PWM using OC4 and Timer2
    InitPWM();
    
    // ===== Part 2 Initialization =====
    
    // Initialize timing pin for scope measurements
    InitTimingPin();
    
    // Initialize bar graph output pins
    InitBarGraph();
    
    // Initialize input capture for encoder
    InitInputCapture();
    
    // Start the periodic timer (10 Hz for ADC and RPM display)
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
     EF_Event_t ThisEvent, the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostMotorService(ES_Event_t ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMotorService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Handles timeout events for ADC/RPM updates and new edge events for bar graph
****************************************************************************/
ES_Event_t RunMotorService(ES_Event_t ThisEvent)
{
    ES_Event_t ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT;

    switch (ThisEvent.EventType)
    {
        case ES_NEW_EDGE:
            // New encoder edge detected - update bar graph
            UpdateBarGraph(CurrentPeriod);
            break;
            
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
                
                // ===== Part 2: Print RPM to terminal =====
                // Raise timing pin before RPM calculation (Part 2.5)
                TIMING_PIN = 1;
                
                if (CurrentPeriod > 0)
                {
                    uint32_t wheelRPM = RPM_NUMERATOR / CurrentPeriod;
                    
                    // Lower timing pin after calculation (Part 2.5)
                    TIMING_PIN = 0;
                    
                    // Raise timing pin before printf (Part 2.6)
                    TIMING_PIN = 1;
                    
                    DB_printf("\rRPM: %4d  Period: %5d", wheelRPM, CurrentPeriod);
                    
                    // Lower timing pin after printf (Part 2.6)
                    TIMING_PIN = 0;
                }
                else
                {
                    TIMING_PIN = 0;
                    DB_printf("\rRPM: ----  Period: -----");
                }
                
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
 Interrupt Service Routine
 ***************************************************************************/
/****************************************************************************
 Function
     IC2_ISR

 Description
     Input Capture 2 Interrupt Service Routine
     Captures timer value on each encoder rising edge and calculates period
****************************************************************************/
void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL6SOFT) IC2_ISR(void)
{
    // Read capture buffer (this also helps clear ICBNE)
    uint32_t ThisCapture = IC2BUF;
    
    // Calculate period (unsigned subtraction handles 16-bit rollover)
    CurrentPeriod = (ThisCapture - LastCapture) & 0xFFFF;
    
    // Save for next edge
    LastCapture = ThisCapture;
    
    // Post event to service to trigger bar graph update
    ES_Event_t NewEvent;
    NewEvent.EventType = ES_NEW_EDGE;
    PostMotorService(NewEvent);
    
    // Clear interrupt flag
    IFS0CLR = _IFS0_IC2IF_MASK;
}

/***************************************************************************
 Private Functions
 ***************************************************************************/

/****************************************************************************
 Function
     InitDirectionPWMPins

 Description
     Configures RB2, RB3, and RB13 as digital outputs
****************************************************************************/
static void InitDirectionPWMPins(void)
{
    // Disable analog function on RB2, RB3, and RB13
    DIR_1A_ANSEL = 0;
    DIR_2A_ANSEL = 0;
    PWM_PIN_ANSEL = 0;
    
    // Set as outputs
    DIR_1A_TRIS = 0;
    DIR_2A_TRIS = 0;
    PWM_PIN_TRIS = 0;
    
    // Set initial direction 
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
    // ===== Timer2 Setup =====
    T2CONbits.ON = 0;
    T2CONbits.TCS = 0;
    T2CONbits.TCKPS = PWM_PRESCALE;
    TMR2 = 0;
    PR2 = PWM_PERIOD;
    T2CONbits.ON = 1;
    
    // ===== Output Compare 4 Setup =====
    OC4CONbits.ON = 0;
    OC4R = 0;
    OC4RS = 0;
    OC4CONbits.OCTSEL = 0;      // Use Timer2 as clock source
    OC4CONbits.OCM = 0b110;     // PWM mode, fault pin disabled
    RPB13R = 0b0101;            // Map OC4 to RB13
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
    if (dutyCycle > PWM_PERIOD)
    {
        dutyCycle = PWM_PERIOD;
    }
    OC4RS = dutyCycle;
}

/****************************************************************************
 Function
     InitTimingPin

 Description
     Configures RB5 as digital output for scope timing measurements
****************************************************************************/
static void InitTimingPin(void)
{
    // RB5 is not an analog pin, no ANSEL needed
    TIMING_PIN_TRIS = 0;    // Set as output
    TIMING_PIN = 0;         // Initialize low
}

/****************************************************************************
 Function
     InitBarGraph

 Description
     Configures bar graph pins as digital outputs
     Pins: RA1, RA2, RA3, RA4, RB8, RB10, RB11, RB15
****************************************************************************/
static void InitBarGraph(void)
{
    // Disable analog on RB15 (AN9)
    BAR8_ANSEL = 0;
    
    // Set all bar graph pins as outputs
    BAR1_TRIS = 0;
    BAR2_TRIS = 0;
    BAR3_TRIS = 0;
    BAR4_TRIS = 0;
    BAR5_TRIS = 0;
    BAR6_TRIS = 0;
    BAR7_TRIS = 0;
    BAR8_TRIS = 0;
    
    // Initialize all LEDs off
    BAR1 = 0;
    BAR2 = 0;
    BAR3 = 0;
    BAR4 = 0;
    BAR5 = 0;
    BAR6 = 0;
    BAR7 = 0;
    BAR8 = 0;
}

/****************************************************************************
 Function
     InitInputCapture

 Description
     Configures Timer3 (free-running) and IC2 for encoder period measurement
     IC2 input on RB9 (pin 18)
****************************************************************************/
static void InitInputCapture(void)
{
    // ===== Timer3 Setup (free-running for Input Capture) =====
    
    // Step 1: Disable timer
    T3CONbits.ON = 0;
    
    // Step 2: Select internal PBCLK source
    T3CONbits.TCS = 0;
    
    // Step 3: Select prescaler (1:8)
    T3CONbits.TCKPS = IC_TIMER_PRESCALE;
    
    // Step 4: Clear timer register
    TMR3 = 0;
    
    // Step 5: Load period register (max for free-running)
    PR3 = 0xFFFF;
    
    // Step 6: Clear T3IF interrupt flag
    IFS0CLR = _IFS0_T3IF_MASK;
    
    // Step 7: Enable Timer3
    T3CONbits.ON = 1;
    
    // ===== Input Capture 2 Setup =====
    
    // Map IC2 input to RB9
    // From PPS table: IC2R value 0b0100 = RPB9
    IC2R = 0b0100;
    
    // Disable IC2 during setup
    IC2CONbits.ON = 0;
    
    // Configure IC2
    IC2CONbits.ICTMR = 0;       // Use Timer3 as source (ICTMR=0 means Timer3)
    IC2CONbits.ICI = 0b00;      // Interrupt on every capture event
    IC2CONbits.ICM = 0b011;     // Capture on every rising edge
    
    // Configure IC2 interrupt
    IFS0CLR = _IFS0_IC2IF_MASK; // Clear interrupt flag
    IPC2bits.IC2IP = 6;         // Priority 6
    IPC2bits.IC2IS = 0;         // Subpriority 0
    IEC0SET = _IEC0_IC2IE_MASK; // Enable IC2 interrupt
    
    // Enable Input Capture module
    IC2CONbits.ON = 1;
}

/****************************************************************************
 Function
     UpdateBarGraph

 Parameters
     uint32_t period - measured period in timer ticks

 Description
     Scales period to 1-8 bars and updates the LED bar graph
     Short period (fast) = 1 bar, Long period (slow) = 8 bars
****************************************************************************/
static void UpdateBarGraph(uint32_t period)
{
    uint8_t numBars;
    
    // Raise timing pin before calculation (Part 2.3)
    TIMING_PIN = 1;
    
    // Determine number of bars to light
    if (period <= MIN_PERIOD)
    {
        numBars = 1;
    }
    else if (period >= MAX_PERIOD)
    {
        numBars = 8;
    }
    else
    {
        // Linear interpolation: 1 + 7 * (period - MIN) / RANGE
        numBars = 1 + (7 * (period - MIN_PERIOD)) / PERIOD_RANGE;
    }
    
    // Turn off all LEDs first
    BAR1 = 0;
    BAR2 = 0;
    BAR3 = 0;
    BAR4 = 0;
    BAR5 = 0;
    BAR6 = 0;
    BAR7 = 0;
    BAR8 = 0;
    
    // Light bars from top down based on numBars
    // Bar 1 (top) = RA1, Bar 8 (bottom) = RB15
    if (numBars >= 1) BAR1 = 1;
    if (numBars >= 2) BAR2 = 1;
    if (numBars >= 3) BAR3 = 1;
    if (numBars >= 4) BAR4 = 1;
    if (numBars >= 5) BAR5 = 1;
    if (numBars >= 6) BAR6 = 1;
    if (numBars >= 7) BAR7 = 1;
    if (numBars >= 8) BAR8 = 1;
    
    // Lower timing pin after writing to LEDs (Part 2.3)
    TIMING_PIN = 0;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
