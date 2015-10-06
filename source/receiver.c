/*
 * receiver.c
 *
 *  Created on: Sep 26, 2015
 *      Author: Phobos_
 */

#include <receiver.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "config.h"
#include "drivers/mpu9250.h"
#if LOG
#include "flash.h"
#endif

extern void SetPIDValues(void);

/***********************************/
/* Global variables in this module */
/***********************************/

volatile int16_t RCTarget[NUMBER_OF_CHANNELS+1];
volatile bool Armed;
uint16_t ReceiverData[16];
uint32_t ReceiverState;
uint32_t OldReceiverState;
uint32_t StateCounter;

/* Map:
 * Maps an input value 'In' with range InMin-InMax
 * to an output value (returned) between OutMin and OutMax
 */

static int16_t Map(int16_t In, int16_t InMin, int16_t InMax, int16_t OutMin,
                   int16_t OutMax)
  {
    if (In <= InMin)
      return OutMin;
    if (In >= InMax)
      return OutMax;
    return (In - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
  }

/* FailSafeHandling:
 * Timer interrupt service routine for the
 * failsafe timer.
 * In case of Failsafe just disarm.
 */

void FailSafeHandling(void *arg)
  {
    (void) arg;
    RCTarget[THROTTLE_CH] = THROTTLE_MIN;
    Armed = FALSE;
    TURN_LED_OFF();
    TURN_B2_LED_OFF();
  }

/* ReceiverFSM:
 * Software Finite state machine that defines the state
 * of the system according to stick commads
 * States areaArmed, disarmed, etc...
 * Outputs channel data into RCTarget variable.
 */
void ReceiverFSM(uint16_t ReceiverData[])
{
   if (Armed && (ReceiverData[THROTTLE_CH] <= RC_MIN) && (ReceiverData[RUDDER_CH] <= RC_MIN) &&
       (ReceiverData[ELEVATOR_CH] < RC_MAX))
     ReceiverState = DISARM;
   else if (!Armed && (ReceiverData[THROTTLE_CH] <= RC_MIN) && (ReceiverData[RUDDER_CH] >= RC_MAX) &&
       (ReceiverData[ELEVATOR_CH] < RC_MAX))
     ReceiverState = ARM;
   else if (!Armed && (ReceiverData[THROTTLE_CH] <= RC_MIN) && (ReceiverData[RUDDER_CH] <= RC_MIN) &&
       (ReceiverData[ELEVATOR_CH] >= RC_MAX))
     ReceiverState = CALIBRATE;
#if LOG
   else if (!Armed && (ReceiverData[THROTTLE_CH] >= RC_MAX) && (ReceiverData[RUDDER_CH] >= RC_MAX) &&
       (ReceiverData[ELEVATOR_CH] < RC_MAX))
     ReceiverState = RECORDING;
   else if (!Armed && (ReceiverData[THROTTLE_CH] >= RC_MAX) && (ReceiverData[RUDDER_CH] <= RC_MIN) &&
       (ReceiverData[ELEVATOR_CH] < RC_MAX))
     ReceiverState = READING;
#endif
   else
     {
       ReceiverState = UNKNOWN;
       StateCounter = 0;
     }
    if (ReceiverState == OldReceiverState)
      {
        StateCounter++;
        if (StateCounter >= ARM_FRAMES)
          switch (ReceiverState)
            {
            case ARM:
              //Arm();
              SetPIDValues();
              Armed = TRUE;
              TURN_LED_ON();
              ResetPID();
              StateCounter = 0;
              break;
            case DISARM:
              Armed = FALSE;
              TURN_LED_OFF();
              StateCounter = 0;
              //Disarm();
              break;
            case CALIBRATE:
              StartCalibration();
              StateCounter = 0;
              //Calibrate();
              break;
#if LOG
            case RECORDING:
              StartRecording();
              StateCounter = 0;
              //Calibrate();
              break;
            case READING:
              StartReading();
              StateCounter = 0;
              //Calibrate();
              break;
#endif
            }
      }
    else
      {
        OldReceiverState = ReceiverState;
        StateCounter = 0;
      }

    chSysLock(); // SysLock to have mutual exclusion with MPU Thread
                 // that is using this data
    RCTarget[THROTTLE_CH] = Map(ReceiverData[THROTTLE_CH], RC_MIN, RC_MAX, THROTTLE_MIN,
                                THROTTLE_MAX);
    RCTarget[AILERON_CH] = Map(ReceiverData[AILERON_CH], RC_MIN, RC_MAX,
                               -PITCH_RATE, PITCH_RATE);
    RCTarget[ELEVATOR_CH] = Map(ReceiverData[ELEVATOR_CH], RC_MIN, RC_MAX,
                                -ROLL_RATE, ROLL_RATE);
    RCTarget[RUDDER_CH] = Map(ReceiverData[RUDDER_CH], RC_MIN, RC_MAX,
                              -YAW_RATE, YAW_RATE);
    RCTarget[AUX1_CH] = Map(ReceiverData[AUX1_CH], RC_MIN, RC_MAX, -AUX_RATE,
                            AUX_RATE);
    RCTarget[AUX2_CH] = Map(ReceiverData[AUX2_CH], RC_MIN, RC_MAX, -AUX_RATE,
                            AUX_RATE);
    RCTarget[STATUS_CH]++;// = Armed;
    chSysUnlock();
}

void ReceiverInit(void)
{
    Armed = FALSE;
    ReceiverState = UNKNOWN;
    OldReceiverState = UNKNOWN;
    StateCounter = 0;
    memset(ReceiverData, 0xff, sizeof(ReceiverData));
    memset((void *)RCTarget, 0, sizeof(RCTarget));
}
