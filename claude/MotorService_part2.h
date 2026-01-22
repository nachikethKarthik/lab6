/****************************************************************************

  Header file for MotorService
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitMotorService(uint8_t Priority);
bool PostMotorService(ES_Event_t ThisEvent);
ES_Event_t RunMotorService(ES_Event_t ThisEvent);

#endif /* MOTOR_SERVICE_H */
