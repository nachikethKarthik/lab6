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
   - PWM frequency approximately 200Hz for part 1

 History
 When           Who     What/Why
 -------------- ---     --------
 01/21/2026     karthi24    started conversion from template file
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MotorService.h"
#include "PIC32_AD_Lib.h"
#include "dbprintf.h"
#include "xc.h"
#include "sys/attribs.h"
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

//pwm pin definition
#define PWM_PIN_TRIS        TRISBbits.TRISB13
#define PWM_PIN_ANSEL       ANSELBbits.ANSB13

// Timer3 prescaler for input capture (1:8)
#define IC_TIMER_PRESCALE   0b011


// Timer3 tick = 0.4 us with 1:8 prescaler at 20MHz PBCLK
#define MIN_PERIOD          900     // Fast speed (tunable parameter)
#define MAX_PERIOD          2000    // Slow speed (tunable parameter)
#define PERIOD_RANGE        (MAX_PERIOD - MIN_PERIOD)

// RPM calculation constant (refer to tablet for proper calculations)
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
#define BAR1_ANSEL           ANSELAbits.ANSA1
#define BAR1                LATAbits.LATA1

#define BAR2_TRIS           TRISAbits.TRISA2
// A2 doesnt have an ANSEL register
#define BAR2                LATAbits.LATA2

#define BAR3_TRIS           TRISAbits.TRISA3
// A3 doesnt have an ANSEL
#define BAR3                LATAbits.LATA3

#define BAR4_TRIS           TRISAbits.TRISA4
// A4 doesnt have an ANSEL
#define BAR4                LATAbits.LATA4

#define BAR5_TRIS           TRISBbits.TRISB8
// B8 does not have an ANSEL
#define BAR5                LATBbits.LATB8

#define BAR6_TRIS           TRISBbits.TRISB10
// B10 does not have an ANSEL
#define BAR6                LATBbits.LATB10

#define BAR7_TRIS           TRISBbits.TRISB11
//B11 does not have an ANSEL
#define BAR7                LATBbits.LATB11

#define BAR8_TRIS           TRISBbits.TRISB15
#define BAR8_ANSEL          ANSELBbits.ANSB15   // RB15 is AN9
#define BAR8                LATBbits.LATB15
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void InitPWM(void);
static void InitDirectionPWMPins(void);
static void SetDutyCycle(uint32_t dutyCycle);
static void InitInputCapture(void);
static void InitBarGraphTimingPins(void);
static void UpdateBarGraph(uint16_t period);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

// Shared with ISR (these variables need to be of the volatile datatype)
static volatile uint16_t CurrentPeriod = 0;     
static volatile uint16_t LastCapture = 0;


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitMotorService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and Initializes PWM, ADC, direction pins, and starts the periodic timer
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitMotorService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  
   // Initialize direction control pins
    InitDirectionPWMPins();
    
    
    ADC_ConfigAutoScan(BIT0HI);
    
    // Initialize PWM using OC4 and Timer2
    InitPWM();
    
    // Initialize bar graph output pins
    InitBarGraphTimingPins();
    
    // Initialize input capture for encoder
    InitInputCapture();
    
    // Start the periodic ADC read timer (10 Hz)
    ES_Timer_InitTimer(MOTOR_TIMER, ADC_UPDATE_TIME);
    
  // post the initial transition event
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
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
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
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunMotorService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  switch (ThisEvent.EventType)
    {
        case ES_NEW_EDGE:
            // New encoder edge detected - update bar graph
            
//            UpdateBarGraph(CurrentPeriod);
            
            
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
                // Update bar graph 
                __builtin_disable_interrupts();
                uint16_t periodSnapshot = CurrentPeriod;
                

                // Update bar graph with the snapshot
                UpdateBarGraph(periodSnapshot);

                // Print period for debugging
//                DB_printf("Period: %d\r\n", periodSnapshot);
                
                // Raise timing pin before RPM calculation
//                TIMING_PIN = 1;
                
                if (periodSnapshot > 0)
                {
                    uint32_t wheelRPM = RPM_NUMERATOR / periodSnapshot;
                    
                    // Lower timing pin after calculation ---> step 2.5
//                    TIMING_PIN = 0;
                    
                    // Raise timing pin before printf (Part 2.6)
                    TIMING_PIN = 1;
                    
                    DB_printf("\rRPM: %d  Period: %d\n", wheelRPM, periodSnapshot);
                    
                    // Lower timing pin after printf (Part 2.6)
                    TIMING_PIN = 0;
                }
                else
                {
                    TIMING_PIN = 0;
                    DB_printf("\rRPM: ----  Period: -----\n");
                }
                // Restart timer for next reading
                
                ES_Timer_InitTimer(MOTOR_TIMER, ADC_UPDATE_TIME);
            }
            __builtin_enable_interrupts();
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
 ISR's
 ***************************************************************************/
void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL6AUTO) IC2_ISR(void){
    
    uint16_t ThisCapture;
    
    // Drain the FIFO - read ALL buffered captures, keep only the last one
    // ICBNE = 1 means buffer is not empty
    while (IC2CONbits.ICBNE) {
        ThisCapture = IC2BUF;
    }
    
    // Calculate period (unsigned subtraction handles 16-bit rollover)
    CurrentPeriod = ThisCapture - LastCapture;
    
    // Save for next edge
    LastCapture = ThisCapture;
    
    // Clear interrupt flag AFTER reading buffer
    IFS0CLR = _IFS0_IC2IF_MASK;
    
    // Post event to service to trigger bar graph update
//    DB_printf("%d\r\n",LastCapture); 
//    ES_Event_t NewEvent;
//    NewEvent.EventType = ES_NEW_EDGE;
//    PostMotorService(NewEvent);

}
/***************************************************************************
 private functions
 ***************************************************************************/
static void InitDirectionPWMPins(void)
{
    // Disable analog function on RB2 and RB3 and RB13 (they are AN4 and AN5)
    DIR_1A_ANSEL = 0;
    DIR_2A_ANSEL = 0;
    PWM_PIN_ANSEL = 0;
    
    // Set RB2 and RB3 and RB13 as outputs
    DIR_1A_TRIS = 0;
    DIR_2A_TRIS = 0;
    PWM_PIN_TRIS = 0;
    
    // Set initial direction 
    DIR_1A = 1;
    DIR_2A = 0;
}

static void InitPWM(void)
{
    // ===== Timer2 Setup =====
    
    T2CONbits.ON = 0;
   
    T2CONbits.TCS = 0;
    
    // Select prescaler (1:4)
    T2CONbits.TCKPS = PWM_PRESCALE;
    
    TMR2 = 0;
    
    PR2 = PWM_PERIOD;
    
//    IFS0CLR = _IFS0_T2IF_MASK;
    //Enable Timer2
    T2CONbits.ON = 1;
    // ===== Output Compare 4 Setup =====
    
    OC4CONbits.ON = 0;
    
    // Set initial duty cycle (0% - motor stopped)
    OC4R = 0;
    OC4RS = 0;
    
    // Map OC4 output to RB13 (pin 24)
    
    
    // Configure OC4 for PWM mode
    OC4CONbits.OCTSEL = 0;      // Use Timer2 as clock source
    OC4CONbits.OCM = 0b110;     // PWM mode, fault pin disabled
        
    RPB13R = 0b0101;
    
    
    // Enable OC4
    OC4CONbits.ON = 1;
}

static void InitBarGraphTimingPins(void)
{
    // RB5 is not an analog pin, no ANSEL needed
    TIMING_PIN_TRIS = 0;    // Set as output
    TIMING_PIN = 0;         // Initialize low
    
    // Disable analog
    BAR1_ANSEL = 0;
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

static void InitInputCapture(void)
{
    // ===== Timer3 Setup (free-running for Input Capture) =====
    
    // Disable timer
    T3CONbits.ON = 0;
    
    //Select internal PBCLK source
    T3CONbits.TCS = 0;
    
    //  Select prescaler (1:8)
    T3CONbits.TCKPS = IC_TIMER_PRESCALE;
    
    // Clear timer register
    TMR3 = 0;
    
    //  Load period register (max for free-running)
    PR3 = 0xFFFF;
    
    //Clear T3IF interrupt flag
    IFS0CLR = _IFS0_T3IF_MASK;
    
    //  Enable Timer3
    T3CONbits.ON = 1;
    
    // ===== Input Capture 2 Setup =====
    
    // Map IC2 input to RB9
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

static void UpdateBarGraph(uint16_t period)
{
    uint8_t numBars;
    
    // Raise timing pin before calculation (Part 2.3)
//    TIMING_PIN = 1;
    
    // Determine number of bars to light
    if (period <= MIN_PERIOD)
    {
        numBars = 8;  // Fast = more bars (or 1 if you prefer opposite)
    }
    else if (period >= MAX_PERIOD)
    {
        numBars = 1;  // Slow = fewer bars
    }
    else
    {
        // Linear interpolation: 8 - 7 * (period - MIN) / RANGE
        numBars = 8 - (7 * (period - MIN_PERIOD)) / PERIOD_RANGE;
    }
    
    // Update each LED directly based on numBars (no flicker)
    BAR1 = (numBars >= 1);
    BAR2 = (numBars >= 2);
    BAR3 = (numBars >= 3);
    BAR4 = (numBars >= 4);
    BAR5 = (numBars >= 5);
    BAR6 = (numBars >= 6);
    BAR7 = (numBars >= 7);
    BAR8 = (numBars >= 8);
    
    // Lower timing pin after writing to LEDs (Part 2.3)
//    TIMING_PIN = 0;
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

