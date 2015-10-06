/*
 * openlrs.h
 *
 *  Created on: Sep 26, 2015
 *      Author: Phobos_
 */

#ifndef OPENLRS_H_
#define OPENLRS_H_

#include "ch.h"

#define RFM22_DEVICE_TYPE 0x00  // R
#define RFM22_DT_MASK     0x1F
#define GPIO0_TX_GPIO1_RX TRUE

// BIND_DATA flag masks
#define TELEMETRY_OFF       0x00
#define TELEMETRY_PASSTHRU  0x08
#define TELEMETRY_FRSKY     0x10 // covers smartport if used with &
#define TELEMETRY_SMARTPORT 0x18
#define TELEMETRY_MASK      0x18
#define CHANNELS_4_4        0x01
#define CHANNELS_8          0x02
#define CHANNELS_8_4        0x03
#define CHANNELS_12         0x04
#define CHANNELS_12_4       0x05
#define CHANNELS_16         0x06
#define DIVERSITY_ENABLED   0x80

#define RF_MAGIC 		0xABCDEF

#define DATARATE 		4
#define RF_POWER 		0
#define CHANNEL_SPACING 25

#define MIN_RFM_FREQUENCY 413000000
#define MAX_RFM_FREQUENCY 463000000
#define CARRIER_FREQUENCY 435000000  // Hz  (ch 0)

#define HOPCHANNELS 	6
#define MAXHOPS			24
#define HOPLIST 		18, 36, 54, 72, 90, 108

#define FLAGS       (CHANNELS_4_4 | TELEMETRY_OFF)

#define TELEMETRY_PACKETSIZE 9

THD_FUNCTION(OpenLRSThread, arg);

#endif /* OPENLRS_H_ */
