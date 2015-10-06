/*
 * spektrum.c
 *
 *  Created on: Apr 25, 2015
 *      Author: Pablo
 */
#include <drivers/spektrum.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "config.h"
#include "receiver.h"

/***********************************/
/* Global variables in this module */
/***********************************/
extern uint16_t	ReceiverData[16];

static const SerialConfig MyConfig =
{
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};


/***********************************/
/* Routines                        */
/***********************************/

/* DecodeFrame:
 * Decodes a receiver data frame received in 'Frame'
 * Outputs the new data in 'RCTarget'
 * returns TRUE if OK, FALSE if frame error
 */

static bool DecodeFrame(uint8_t *Frame)
  {
    uint32_t ByteInFrame, i;
    uint16_t Word;
    uint8_t ChannelNumber;

    ByteInFrame = 2;
    for (i = 0; i < DSM_CHANNELS_PER_FRAME; i++)
      {
        Word = ((uint16_t) Frame[ByteInFrame] << 8)
            | Frame[ByteInFrame + 1];
        ByteInFrame += 2;
        /* skip empty channel slot */
        if (Word == 0xffff)
          continue;

        /* minimal data validation */
        if ((i > 0) && (Word & DSM_2ND_FRAME_MASK))
          {
            /* invalid frame data, ignore rest of the frame */
            return FALSE;
          }
        /* extract and save the channel value */
        ChannelNumber = (Word >> 11) & 0x0f;
        ReceiverData[ChannelNumber] = (Word & DSM_MASK);
      }
    ReceiverFSM(ReceiverData);
    return TRUE;
  }

/* ReceiverThread:
 *
 */

THD_WORKING_AREA(waReceiverThread, 1024);
THD_FUNCTION(ReceiverThread, arg)
  {
    (void) arg;
    chRegSetThreadName("SerialReceiver");

    uint8_t Frame[16];

    unsigned int ByteInFrame;
    bool GotFrame;
    systime_t LastByteTime, CurrentTime;
    msg_t ByteRead;
    virtual_timer_t FailsafeTimer;

    chVTObjectInit(&FailsafeTimer);  // virtual timer used for failsafe handling

    sdStart(&SD6, &MyConfig);  // Serial port to receiver
    LastByteTime= 0;
    ReceiverInit();

    while (TRUE)
      {
        GotFrame = FALSE;
        ByteInFrame = 0;
        while (!GotFrame)
          {
            ByteRead = chnGetTimeout(&SD6, TIME_INFINITE); // Get next byte from serial port
            CurrentTime = chVTGetSystemTime();
            Frame[ByteInFrame] = (uint8_t) ByteRead;
            if (ByteInFrame == 0)
              {
                LastByteTime = CurrentTime;
                ByteInFrame = 1;
              }
            else
              if (ST2MS((systime_t)(CurrentTime-LastByteTime)) > 5)
                { // More than 5ms from last byte: We lost some bytes. Restart frame counter.
                  Frame[0] = Frame[ByteInFrame];
                  LastByteTime = CurrentTime;
                  ByteInFrame = 1;
                }
              else
                {
                  ByteInFrame++;
                  LastByteTime = CurrentTime;
                  if (ByteInFrame == DSM_FRAME_LENGTH)
                    GotFrame = TRUE;  // Whole frame received
                }
          }
        if (DecodeFrame(Frame))  // Decode frame
          chVTSet(&FailsafeTimer, MS2ST(500), FailSafeHandling, 0); // re-start failsafe timer

      }
  }

