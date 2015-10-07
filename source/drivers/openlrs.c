/*
 * openlrs.c
 *
 *  Created on: Sep 26, 2015
 *      Author: Phobos_
 */
#include <drivers/openlrs.h>
#include <drivers/openlrs_rfm22b_regs.h>

#include "ch.h"
#include "hal.h"
#include "config.h"
#include "receiver.h"

static binary_semaphore_t RFMDataReady; /* Semaphore for the Receiver thread */
extern uint16_t    ReceiverData[16];

uint8_t hopchannels[MAXHOPS] = { HOPLIST };
const uint8_t pktsizes[] = { 0, 7, 11, 12, 16, 17, 21, 0 };
#define PACKET_SIZE            pktsizes[FLAGS & 0x07]

// Private constants
const struct rfm22_modem_regs {
  uint32_t bps;
  uint8_t  r_1c, r_1d, r_1e, r_20, r_21, r_22, r_23, r_24, r_25, r_2a, r_6e, r_6f, r_70, r_71, r_72;
} modem_params[] = {
  { 4800, 0x1a, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x1b, 0x1e, 0x27, 0x52, 0x2c, 0x23, 0x30 }, // 50000 0x00
  { 9600, 0x05, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x20, 0x24, 0x4e, 0xa5, 0x2c, 0x23, 0x30 }, // 25000 0x00
  { 19200, 0x06, 0x40, 0x0a, 0xd0, 0x00, 0x9d, 0x49, 0x00, 0x7b, 0x28, 0x9d, 0x49, 0x2c, 0x23, 0x30 }, // 25000 0x01
  { 57600, 0x05, 0x40, 0x0a, 0x45, 0x01, 0xd7, 0xdc, 0x03, 0xb8, 0x1e, 0x0e, 0xbf, 0x00, 0x23, 0x2e },
  { 125000, 0x8a, 0x40, 0x0a, 0x60, 0x01, 0x55, 0x55, 0x02, 0xad, 0x1e, 0x20, 0x00, 0x00, 0x23, 0xc8 },
};

const uint8_t OUT_FF[64] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

void RFMISR(EXTDriver *extp, expchannel_t channel)
{
  (void) extp;
  (void) channel;

  chSysLockFromISR();
  chBSemSignalI(&RFMDataReady);
  chSysUnlockFromISR();
}

/*
 * Maximum speed SPI configuration (21MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig HSSpiConfig ={
  NULL,
  GPIOA,
  15,
  0
};

static void rfm_claimBus(void){
    spiAcquireBus(&SPID3);
    spiStart(&SPID3, &HSSpiConfig);
}

static void rfm_releaseBus(void){
    spiReleaseBus(&SPID3);
}

static uint8_t rfm_claim_read(uint8_t Address){
    uint8_t RxBuffer[2];
    uint8_t TxBuffer[2];

    TxBuffer[0] = Address & 0x7F;
    TxBuffer[1] = 0xFF;
    spiAcquireBus(&SPID3);
    spiStart(&SPID3, &HSSpiConfig);
    spiSelect(&SPID3);
    spiExchange(&SPID3, 2, TxBuffer, RxBuffer);
    spiUnselect(&SPID3);
    spiReleaseBus(&SPID3);
    return RxBuffer[1];
}

static void rfm_claim_write(uint8_t Address, uint8_t Value){
    uint8_t TxBuffer[2];

    TxBuffer[0] = Address | 0x80;
    TxBuffer[1] = Value;
    spiAcquireBus(&SPID3);
    spiStart(&SPID3, &HSSpiConfig);
    spiSelect(&SPID3);
    spiSend(&SPID3, 2, TxBuffer);
    spiUnselect(&SPID3);
    spiReleaseBus(&SPID3);
}

static uint8_t rfm_read(uint8_t Address){
    uint8_t RxBuffer[2];
    uint8_t TxBuffer[2];

    TxBuffer[0] = Address & 0x7F;
    TxBuffer[1] = 0xFF;
    spiSelect(&SPID3);
    spiExchange(&SPID3, 2, TxBuffer, RxBuffer);
    spiUnselect(&SPID3);
    return RxBuffer[1];
}

static void rfm_write(uint8_t Address, uint8_t Value){
    uint8_t TxBuffer[2];

    TxBuffer[0] = Address | 0x80;
    TxBuffer[1] = Value;

    spiSelect(&SPID3);
    spiSend(&SPID3, 2, TxBuffer);
    spiUnselect(&SPID3);
}

static void rfmSetCarrierFrequency(uint32_t f)
{
    uint16_t fb, fc, hbsel;
    if (f < 480000000) {
        hbsel = 0;
        fb = f / 10000000 - 24;
        fc = (f - (fb + 24) * 10000000) * 4 / 625;
    } else {
        hbsel = 1;
        fb = f / 20000000 - 24;
        fc = (f - (fb + 24) * 20000000) * 2 / 625;
    }
    rfm_claimBus();
    rfm_write(RFM22_frequency_band_select, RFM22_fbs_sbse + (hbsel ? RFM22_fbs_hbsel : 0) + (fb & RFM22_fb_mask));
    rfm_write(RFM22_nominal_carrier_frequency1, (fc >> 8));
    rfm_write(RFM22_nominal_carrier_frequency0, (fc & 0xff));
    rfm_releaseBus();
}

static void setModemRegs(const struct rfm22_modem_regs* r)
{
    rfm_claimBus();
    rfm_write(RFM22_if_filter_bandwidth, r->r_1c);
    rfm_write(RFM22_afc_loop_gearshift_override, r->r_1d);
    rfm_write(RFM22_afc_timing_control, r->r_1e);
    rfm_write(RFM22_clk_recovery_oversampling_ratio, r->r_20);
    rfm_write(RFM22_clk_recovery_offset2, r->r_21);
    rfm_write(RFM22_clk_recovery_offset1, r->r_22);
    rfm_write(RFM22_clk_recovery_offset0, r->r_23);
    rfm_write(RFM22_clk_recovery_timing_loop_gain1, r->r_24);
    rfm_write(RFM22_clk_recovery_timing_loop_gain0, r->r_25);
    rfm_write(RFM22_afc_limiter, r->r_2a);
    rfm_write(RFM22_tx_data_rate1, r->r_6e);
    rfm_write(RFM22_tx_data_rate0, r->r_6f);
    rfm_write(RFM22_modulation_mode_control1, r->r_70);
    rfm_write(RFM22_modulation_mode_control2, r->r_71);
    rfm_write(RFM22_frequency_deviation, r->r_72);
    rfm_releaseBus();
}

static void rfm_reset(void)
{
    chThdSleepMilliseconds(5);
    rfm_claim_write(RFM22_op_and_func_ctrl1, RFM22_opfc1_swres | 0x01); // Reset register values on RFM22B
    chThdSleepMilliseconds(5);
}

static void init_rfm(void)
{
    rfm_claimBus();
    rfm_read(RFM22_interrupt_status1);   // read status, clear interrupt
    rfm_read(RFM22_interrupt_status2);
    rfm_write(RFM22_interrupt_enable2, 0x00);    // disable interrupts
    rfm_write(RFM22_op_and_func_ctrl1, RFM22_opfc1_xton); // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
    rfm_write(RFM22_xtal_osc_load_cap, RFM22_xolc_xlc_mask);   // c = 12.5p
    rfm_write(RFM22_cpu_output_clk, RFM22_coc_2MHz);
    if (GPIO0_TX_GPIO1_RX) {
        rfm_write(RFM22_gpio0_config, RFM22_gpio0_config_txstate);    // gpio0 TX State
        rfm_write(RFM22_gpio1_config, RFM22_gpio1_config_rxstate);    // gpio1 RX State
    }else{
        rfm_write(RFM22_gpio0_config, RFM22_gpio0_config_rxstate);    // gpio0 TX State
        rfm_write(RFM22_gpio1_config, RFM22_gpio1_config_txstate);    // gpio1 RX State
    }
    rfm_write(RFM22_gpio2_config, 0xfd);    // gpio 2 VDD
    rfm_write(RFM22_io_port_config, RFM22_io_port_default);    // gpio    0, 1,2 NO OTHER FUNCTION.
    rfm_releaseBus();

    setModemRegs(&modem_params[DATARATE]);

    // Packet settings
    rfm_claimBus();
    rfm_write(RFM22_data_access_control, 0x8c);    // enable packet handler, msb first, enable crc,
    rfm_write(RFM22_header_control1, 0x0f);    // no broadcast, check header bytes 3,2,1,0
    rfm_write(RFM22_header_control2, 0x42);    // 4 byte header, 2 byte synch, variable pkt size
    rfm_write(RFM22_preamble_length, 0x0a);    // 40 bit preamble
    rfm_write(RFM22_preamble_detection_ctrl1, 0x2a);    // preath = 5 (20bits), rssioff = 2
    rfm_write(RFM22_sync_word3, 0x2d);    // synchronize word 3
    rfm_write(RFM22_sync_word2, 0xd4);    // synchronize word 2
    rfm_write(RFM22_sync_word1, 0x00);    // synch word 1 (not used)
    rfm_write(RFM22_sync_word0, 0x00);    // synch word 0 (not used)

    uint32_t magic = RF_MAGIC;
    uint8_t i;
    for (i = 0; i < 4; i++) {
        rfm_write(RFM22_transmit_header3 + i, (magic >> 24) & 0xff);   // tx header
        rfm_write(RFM22_check_header3 + i, (magic >> 24) & 0xff);   // rx header
        magic = magic << 8; // advance to next byte
    }

    rfm_write(RFM22_header_enable3, 0xff);    // all the bit to be checked
    rfm_write(RFM22_header_enable2, 0xff);    // all the bit to be checked
    rfm_write(RFM22_header_enable1, 0xff);    // all the bit to be checked
    rfm_write(RFM22_header_enable0, 0xff);    // all the bit to be checked

    rfm_write(RFM22_tx_power, RF_POWER);

    rfm_write(RFM22_frequency_hopping_channel_select, 0);
    rfm_write(RFM22_frequency_hopping_step_size, CHANNEL_SPACING);   // channel spacing

    rfm_write(RFM22_frequency_offset1, 0x00);
    rfm_write(RFM22_frequency_offset2, 0x00);    // no offset
    rfm_releaseBus();

    rfmSetCarrierFrequency(CARRIER_FREQUENCY);
}

static void rfmSetChannel(uint8_t ch)
{
    uint8_t magicLSB = (RF_MAGIC & 0xff) ^ ch;
    rfm_claimBus();
    rfm_write(RFM22_frequency_hopping_channel_select, hopchannels[ch]);
    rfm_write(RFM22_transmit_header3 + 3, magicLSB);
    rfm_write(RFM22_check_header3 + 3, magicLSB);
    rfm_releaseBus();
}

static void rx_reset(void)
{
    rfm_claimBus();
    rfm_write(RFM22_op_and_func_ctrl1, RFM22_opfc1_xton);

    rfm_write(RFM22_rx_fifo_control, 36);     // threshold for rx almost full, interrupt when 1 byte received
    //clear FIFO
    rfm_write(RFM22_op_and_func_ctrl2, 0x03);
    rfm_write(RFM22_op_and_func_ctrl2, 0x00);

    rfm_write(RFM22_op_and_func_ctrl1, RFM22_opfc1_rxon | RFM22_opfc1_xton);   // to rx mode
    rfm_write(RFM22_interrupt_enable1, RFM22_ie1_enpkvalid);
    rfm_read(RFM22_interrupt_status1);   //read the Interrupt Status1 register
    rfm_read(RFM22_interrupt_status2);
    rfm_releaseBus();
}

static void to_rx_mode(void)
{
    rfm_claimBus();
    rfm_read(RFM22_interrupt_status1);
    rfm_read(RFM22_interrupt_status2);
    rfm_write(RFM22_op_and_func_ctrl1, RFM22_opfc1_xton);
    rfm_releaseBus();
    chThdSleepMilliseconds(10);
    rx_reset();
    __asm__ __volatile__("nop");
}

static void unpackChannels(uint8_t config, uint16_t PPM[], uint8_t *p)
{
    uint8_t i;
    for (i=0; i<=(config/2); i++) { // 4ch packed in 5 bytes
        PPM[0] = (((uint16_t)p[4] & 0x03) << 8) + p[0];
        PPM[1] = (((uint16_t)p[4] & 0x0c) << 6) + p[1];
        PPM[2] = (((uint16_t)p[4] & 0x30) << 4) + p[2];
        PPM[3] = (((uint16_t)p[4] & 0xc0) << 2) + p[3];
        p+=5;
        PPM+=4;
    }
    if (config & 1) { // 4ch packed in 1 byte;
        PPM[0] = (((uint16_t)p[0] >> 6) & 3) * 333 + 12;
        PPM[1] = (((uint16_t)p[0] >> 4) & 3) * 333 + 12;
        PPM[2] = (((uint16_t)p[0] >> 2) & 3) * 333 + 12;
        PPM[3] = (((uint16_t)p[0] >> 0) & 3) * 333 + 12;
    }
}

//! Apply the OpenLRS rescaling to the channels
/*static void rescaleChannels(int16_t PPM[])
{
    uint8_t i;
    for (i = 0; i < NUMBER_OF_CHANNELS; i++) {
        int16_t x = PPM[i];
        int16_t ret;

        if (x < 12) {
            ret = 808 + x * 16;
        } else if (x < 1012) {
            ret = x + 988;
        } else if (x < 1024) {
            ret = 2000 + (x - 1011) * 16;
        } else {
            ret = 2192;
        }

        PPM[i] = ret;
    }
}*/

static uint32_t getInterval(void)
{
    uint32_t ret;
    // Sending a x byte packet on bps y takes about (emperical)
    // usec = (x + 15) * 8200000 / baudrate
#define BYTES_AT_BAUD_TO_USEC(bytes, bps, div) ((uint32_t)((bytes) + (div?20:15)) * 8200000L / (uint32_t)(bps))

    ret = (BYTES_AT_BAUD_TO_USEC(PACKET_SIZE, modem_params[DATARATE].bps, FLAGS&DIVERSITY_ENABLED) + 2000);

    if (FLAGS & TELEMETRY_MASK) {
        ret += (BYTES_AT_BAUD_TO_USEC(TELEMETRY_PACKETSIZE, modem_params[DATARATE].bps, FLAGS&DIVERSITY_ENABLED) + 1000);
    }

    // round up to ms
    ret = ((ret + 999) / 1000) * 1000;

  // enable following to limit packet rate to 50Hz at most
#ifdef LIMIT_RATE_TO_50HZ
    if (ret < 20000) {
        ret = 20000;
    }
#endif

    return US2ST(ret); //in systicks
}

bool OpenLRSInit(void){
    // Before initializing everything, make sure device found
    rfm_reset();
    uint8_t device_type = rfm_claim_read(RFM22_DEVICE_TYPE) & RFM22_DT_MASK;
    if (device_type != 0x08)
        return false;

    uint8_t i;
    for (i=HOPCHANNELS;i<MAXHOPS;i++) hopchannels[i]=0;

    init_rfm();   // Configure the RFM22B's registers for normal operation
    rfmSetChannel(0); //set the first channel

    //################### RX SYNC AT STARTUP #################
    to_rx_mode();

    return true;
}

THD_WORKING_AREA(waOpenLRSThread, 1024);
THD_FUNCTION(OpenLRSThread, arg)
{
    (void) arg;
    chRegSetThreadName("OpenLRSReceiver");
    chBSemObjectInit(&RFMDataReady, TRUE); /* Semaphore initialization before use */
    //extStart(&EXTD1, &ExternalInterruptConfig); /* External interrupt setup */
    spiStart(&SPID3, &HSSpiConfig); /* Setup SPI3 */

    uint8_t numberOfLostPackets = 0;
    bool willhop;
    systime_t lastPacketTime, currentTime;
    virtual_timer_t failsafeTimer;

    uint8_t RxBuffer[64];
    uint8_t rf_channel = 0;
    uint32_t interval = getInterval();

    ReceiverInit();

    if (!OpenLRSInit())
          while (TRUE)
            {
              TOGGLE_O_LED();                  // RFM22b not answering
              chThdSleepMilliseconds(500);   // Toggle LED forever
            }

    chVTObjectInit(&failsafeTimer);  // virtual timer used for failsafe handling

    while(TRUE){

        currentTime = chVTGetSystemTime();
        willhop=false;

        if (rfm_claim_read(RFM22_gpio1_config) == 0) {     // detect the locked module and reboot
            init_rfm();
            to_rx_mode();
        }

        if(chBSemWaitTimeout(&RFMDataReady,interval * HOPCHANNELS) == MSG_OK){
            //got link

            TOGGLE_B2_LED();

            spiAcquireBus(&SPID3);
            spiStart(&SPID3, &HSSpiConfig);
            spiSelect(&SPID3);
            spiSend(&SPID3, 1, &(uint8_t){RFM22_fifo_access}); //creates temp variable and passes it's addr
            spiExchange(&SPID3, PACKET_SIZE, OUT_FF, RxBuffer);
            spiUnselect(&SPID3);
            spiReleaseBus(&SPID3);

            if ((RxBuffer[0] & 0x3e) == 0x00) { //we got proper data
                unpackChannels(FLAGS & 7, ReceiverData, RxBuffer + 1);
                //rescaleChannels(ReceiverData);

                ReceiverFSM(ReceiverData);

                numberOfLostPackets=0;
                lastPacketTime = currentTime;

                chVTSet(&failsafeTimer, MS2ST(500), FailSafeHandling, 0); // re-start failsafe timer (failsafe after 500ms)
                willhop = true;
            }

            if ((numberOfLostPackets < HOPCHANNELS) && ((currentTime - lastPacketTime) > (interval + US2ST(1000)))) {
                        // we lost packet, hop to next channel
                        TURN_B2_LED_OFF();
                        numberOfLostPackets++;
                        lastPacketTime += interval;
                        willhop=true;
                    } else if ((numberOfLostPackets >= HOPCHANNELS) && ((currentTime - lastPacketTime) > (interval * HOPCHANNELS))) {
                        // hop slowly to allow resync with TX
                        TURN_B2_LED_OFF();
                        lastPacketTime = currentTime;
                        willhop=true;
                    }
        } else {
            // Waiting for first packet, hop slowly
            lastPacketTime = currentTime;
            willhop=true;
        }

        if (willhop){
            rf_channel++;
            if ((rf_channel == MAXHOPS) || hopchannels[rf_channel] == 0) rf_channel = 0;
            rfmSetChannel(rf_channel);
            rx_reset();
        }
    }
}
