/*
 * spektrum.h
 *
 *  Created on: Apr 25, 2015
 *      Author: Pablo
 */

#ifndef SPEKTRUM_H_
#define SPEKTRUM_H_

#include "ch.h"

#define DSM_MASK                (0x7FF)
#define DSM_CHANNELS_PER_FRAME  7
#define DSM_FRAME_LENGTH        (1+1+DSM_CHANNELS_PER_FRAME*2)
#define DSM_DSM2_RES_MASK       0x0010
#define DSM_2ND_FRAME_MASK      0x8000

THD_FUNCTION(ReceiverThread, arg);

#endif /* SPEKTRUM_H_ */
