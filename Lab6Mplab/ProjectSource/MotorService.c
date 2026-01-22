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
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void InitPWM(void);
static void InitDirectionPWMPins(void);
static void SetDutyCycle(uint32_t dutyCycle);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

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
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == MOTOR_TIMER)
            {
                // Read potentiometer value from ADC
                uint32_t adcResult[1];
                ADC_MultiRead(adcResult);
                
                DB_printf("%d\r\n",adcResult[0]);
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

