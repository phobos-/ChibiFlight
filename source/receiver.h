/*
 * receiver.h
 *
 *  Created on: Sep 26, 2015
 *      Author: Phobos_
 */

#include <stdint.h>

#ifndef RECEIVER_H_
#define RECEIVER_H_

#define THROTTLE_CH           0
#define AILERON_CH            1
#define ROLL_CH               1
#define ELEVATOR_CH           2
#define PITCH_CH              2
#define RUDDER_CH             3
#define YAW_CH                3
#define AUX1_CH               4
#define AUX2_CH               5
#define STATUS_CH             6

#define RC_MIN                0x00B //12
#define RC_NEUTRAL            0x1F4 //500
#define RC_MAX                0x3F4 //1012


#define ARM_FRAMES            30

#define UNKNOWN                0xFFFFFFFF
#define ARM                    1
#define DISARM                 2
#define CALIBRATE              3
#define RECORDING              4
#define READING                5

#define NUMBER_OF_CHANNELS	   8

void FailSafeHandling(void *arg);
void ReceiverFSM(uint16_t ReceiverData[]);
void ReceiverInit(void);

#endif /* RECEIVER_H_ */
