/****************************************************************************
 Module
   MotorService.h

 Revision
   1.0.0

 Description
   Header file for MotorService - PWM motor control with potentiometer input

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/21/26       Student  Lab 6 Part 1
****************************************************************************/

#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

/*----------------------------- Include Files -----------------------------*/
#include "ES_Types.h"
#include <stdint.h>
#include <stdbool.h>

/*----------------------------- Public Functions ---------------------------*/
bool InitMotorService(uint8_t Priority);
bool PostMotorService(ES_Event_t ThisEvent);
ES_Event_t RunMotorService(ES_Event_t ThisEvent);

#endif /* MOTOR_SERVICE_H */
