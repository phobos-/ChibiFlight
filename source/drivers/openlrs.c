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
extern volatile bool Armed;

uint8_t hopchannels[MAXHOPS] = { HOPLIST };
const uint8_t pktsizes[] = { 0, 7, 11, 12, 16, 17, 21, 0 };
#define PACKET_SIZE            pktsizes[FLAGS & 0x07]
const uint32_t packet_advance_time_us = 1500;
const uint32_t packet_timeout_us = 1000;

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

/*
static uint32_t us_ticks;
static uint32_t us_modulo;

bool DELAY_Init(void)
{
    us_ticks = 168;

    // Split this into two steps to avoid 64bit maths
    us_modulo = 0xffffffff / us_ticks;
    us_modulo += ((0xffffffff % us_ticks) + 1) / us_ticks;

    if(us_modulo > 0x80000000) return false;

    // turn on access to the DWT registers
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // enable the CPU cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    return true;
}

static uint32_t micros(void)
{
    // turn on access to the DWT registers
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    return DWT->CYCCNT / us_ticks;
}

uint32_t getUsSince(uint32_t t)
{
    return (micros() + us_modulo - t) % us_modulo;
}
*/

static uint32_t micros(void)
{
    return (uint32_t) ST2US(chVTGetSystemTimeX());
}

uint32_t getUsSince(uint32_t t)
{
    return (micros() - t);
}

static uint32_t millis(void)
{
    return (uint32_t) ST2MS(chVTGetSystemTimeX());
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

static uint8_t rfmGetRSSI(void)
{
    return rfm_claim_read(RFM22_rssi);
}

static uint8_t beaconGetRSSI(void)
{
    uint16_t rssiSUM=0;

    rfmSetCarrierFrequency(BEACON_FREQUENCY);
    rfm_claim_write(RFM22_frequency_hopping_channel_select, 0); // ch 0 to avoid offset
    chThdSleep(MS2ST(1));
    rssiSUM+=rfmGetRSSI();
    chThdSleep(MS2ST(1));
    rssiSUM+=rfmGetRSSI();
    chThdSleep(MS2ST(1));
    rssiSUM+=rfmGetRSSI();
    chThdSleep(MS2ST(1));
    rssiSUM+=rfmGetRSSI();

    return rssiSUM>>2;
}

static void beacon_tone(int16_t hz, int16_t len) //duration is now in half seconds.
{
    int16_t d = 500000 / hz; // better resolution

    TURN_B2_LED_ON();

    if (d < 1) {
        d = 1;
    }

    rfm_claimBus();

    // Set MOSI to digital out for bit banging
    palSetPadMode(GPIOC, 12, PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(GPIOC, 12);

    uint32_t raw_time = chVTGetSystemTimeX();
    int16_t cycles = (len * 500000 / d);
    int16_t i;
    for (i = 0; i < cycles; i++) {
        palSetPad(GPIOC, 12);
        chThdSleep(US2ST(d));
        palClearPad(GPIOC, 12);
        chThdSleep(US2ST(d));

        // Make sure to give other tasks time to do things
        if (ST2US(chVTGetSystemTimeX()-raw_time) > 10000) {
            chThdSleep(MS2ST(1));
            raw_time = chVTGetSystemTimeX();
        }
    }

    palSetPadMode(GPIOC, 12, PAL_MODE_ALTERNATE(6)); /* New MOSI. */

    rfm_releaseBus();

    TURN_B2_LED_OFF();
}

static void beacon_send(bool static_tone)
{
    rfm_claimBus();
    rfm_read(0x03);   // read status, clear interrupt
    rfm_read(0x04);
    rfm_write(0x06, 0x00);    // no wakeup up, lbd,
    rfm_write(0x07, RFM22_opfc1_xton);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
    rfm_write(0x09, 0x7f);  // (default) c = 12.5p
    rfm_write(0x0a, 0x05);
    rfm_write(0x0b, 0x12);    // gpio0 TX State
    rfm_write(0x0c, 0x15);    // gpio1 RX State
    rfm_write(0x0d, 0xfd);    // gpio 2 micro-controller clk output
    rfm_write(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.

    rfm_write(0x70, 0x2C);    // disable manchest

    rfm_write(0x30, 0x00);    //disable packet handling

    rfm_write(0x79, 0);  // start channel

    rfm_write(0x7a, 0x05);   // 50khz step size (10khz x value) // no hopping

    rfm_write(0x71, 0x12);   // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
    rfm_write(0x72, 0x02);   // fd (frequency deviation) 2*625Hz == 1.25kHz

    rfm_write(0x73, 0x00);
    rfm_write(0x74, 0x00);    // no offset
    rfm_releaseBus();

    rfmSetCarrierFrequency(BEACON_FREQUENCY);

    rfm_claim_write(0x6d, 0x07);   // 7 set max power 100mW

    chThdSleep(MS2ST(10));
    rfm_claim_write(0x07, (RFM22_opfc1_txon | RFM22_opfc1_xton));    // to tx mode
    chThdSleep(MS2ST(10));

    if (static_tone) {
        uint8_t i=0;
        while (i++<20) {
            beacon_tone(440, 1);
        }
    } else {
        //close encounters tune
        //  G, A, F, F(lower octave), C
        //octave 3:  392  440  349  175   261

        beacon_tone(392, 1);

        rfm_claim_write(0x6d, 0x05);   // 5 set mid power 25mW
        chThdSleep(MS2ST(10));
        beacon_tone(440,1);

        rfm_claim_write(0x6d, 0x04);   // 4 set mid power 13mW
        chThdSleep(MS2ST(10));
        beacon_tone(349, 1);

        rfm_claim_write(0x6d, 0x02);   // 2 set min power 3mW
        chThdSleep(MS2ST(10));
        beacon_tone(175,1);

        rfm_claim_write(0x6d, 0x00);   // 0 set min power 1.3mW
        chThdSleep(MS2ST(10));
        beacon_tone(261, 2);
    }
    rfm_claim_write(0x07, RFM22_opfc1_xton);
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

    return ret;
}

uint32_t lastPacketTimeUs=0;
uint32_t numberOfLostPackets=0;
uint32_t lastRSSITimeUs=0;
uint8_t lastRSSI=0;
bool willhop=false;
bool link_acquired=false;
bool received=false;
bool beacon_armed=false;
bool failsafe_active=false;
uint32_t nextBeaconTimeMs=0;
uint32_t linkLossTimeMs=0;
uint32_t beacon_rssi_avg=0;
uint16_t linkQuality=0;
uint8_t RxBuffer[64];
uint8_t rf_channel = 0;
uint32_t interval=0;

static void rx_loop(void)
{
    volatile uint32_t timeUs, timeMs;

    if (rfm_claim_read(RFM22_gpio1_config) == 0) {     // detect the locked module and reboot
        init_rfm();
        to_rx_mode();
    }

    timeUs = micros();
    timeMs = millis();

    if(received){
        //got link

        TOGGLE_B2_LED();

        spiAcquireBus(&SPID3);
        spiStart(&SPID3, &HSSpiConfig);
        spiSelect(&SPID3);
        spiSend(&SPID3, 1, &(uint8_t){RFM22_fifo_access}); //creates temp variable and passes it's addr
        spiExchange(&SPID3, PACKET_SIZE, OUT_FF, RxBuffer);
        spiUnselect(&SPID3);
        spiReleaseBus(&SPID3);

        lastPacketTimeUs = timeUs;
        numberOfLostPackets = 0;
        linkQuality <<= 1;
        linkQuality |= 1;

        if ((RxBuffer[0] & 0x3e) == 0x00) { //we got proper data
            unpackChannels(FLAGS & 7, ReceiverData, RxBuffer + 1);
            //rescaleChannels(ReceiverData);

            ReceiverFSM(ReceiverData);

        }
        /*else {
            // Not PPM data. Push into serial RX buffer.
            if ((RxBuffer[0] & 0x38) == 0x38) {
                if ((RxBuffer[0] ^ tx_buf[0]) & 0x80) {
                    // We got new data... (not retransmission)
                    tx_buf[0] ^= 0x80; // signal that we got it
                    bool rx_need_yield;
                    uint8_t data_len = RxBuffer[0] & 7;
                    if (openlrs_dev->rx_in_cb && (data_len > 0)) {
                        (openlrs_dev->rx_in_cb) (openlrs_dev->rx_in_context, &RxBuffer[1], data_len, NULL, &rx_need_yield);
                    }
                }
            }
        }*/

        // Flag to indicate ever got a link
        link_acquired |= true;
        failsafe_active = false;
        beacon_armed = false; // when receiving packets make sure beacon cannot emit

        // When telemetry is enabled we ack packets and send info about FC back
        /*if (FLAGS & TELEMETRY_MASK) {
            if ((TxBuffer[0] ^ RxBuffer[0]) & 0x40) {
                // resend last message
            } else {
                TxBuffer[0] &= 0xc0;
                TxBuffer[0] ^= 0x40; // swap sequence as we have new data

                // Check for data on serial link
                uint8_t bytes = 0;
                // Append data from the com interface if applicable.
                if (openlrs_dev->tx_out_cb) {
                    // Try to get some data to send
                    bool need_yield = false;
                    bytes = (openlrs_dev->tx_out_cb) (openlrs_dev->tx_out_context, &TxBuffer[1], 8, NULL, &need_yield);
                }

                if (bytes > 0) {
                    TxBuffer[0] |= (0x37 + bytes);
                } else {
                    // TxBuffer[0] lowest 6 bits left at 0
                    TxBuffer[1] = lastRSSI;
                    if (FlightBatteryStateHandle()) {
                        FlightBatteryStateData bat;
                        FlightBatteryStateGet(&bat);
                        // FrSky protocol normally uses 3.3V at 255 but
                        // divider from display can be set internally
                        TxBuffer[2] = (uint8_t) bat.Voltage / 25.0f * 255;
                        TxBuffer[3] = (uint8_t) bat.Current / 60.0f * 255;
                    } else {
                        TxBuffer[2] = 0; // these bytes carry analog info. package
                        TxBuffer[3] = 0; // battery here
                    }
                    TxBuffer[4] = (openlrs_dev->lastAFCCvalue >> 8);
                    TxBuffer[5] = openlrs_dev->lastAFCCvalue & 0xff;
                    TxBuffer[6] = countSetBits(linkQuality & 0x7fff);
                }
            }

            // This will block until sent
            tx_packet(openlrs_dev, tx_buf, 9);
        }*/

        // Once a packet has been processed, flip back into receiving mode
        //openlrs_dev->rf_mode = Receive;
        rx_reset();

        willhop = true;
    }

    if (link_acquired) {
        // For missing packets to be properly trigger a well timed channel hop, this method should be called fairly close (but not sooner)
        // than 1ms after the packet was expected to trigger this path
        if ((numberOfLostPackets < HOPCHANNELS) && (getUsSince(lastPacketTimeUs) > (interval + packet_timeout_us))) {
            // we lost packet, hop to next channel
            linkQuality <<= 1;
            willhop = true;
            if (numberOfLostPackets == 0) {
                linkLossTimeMs = timeMs;
                nextBeaconTimeMs = 0;
            }
            numberOfLostPackets++;
            lastPacketTimeUs += interval;
            willhop = true;
        } else if ((numberOfLostPackets >= HOPCHANNELS) && (getUsSince(lastPacketTimeUs) > (interval * HOPCHANNELS))) {
            // hop slowly to allow resync with TX
            linkQuality = 0;
            willhop = true;
            lastPacketTimeUs = timeUs;
        }

        if (numberOfLostPackets) {

            TURN_B2_LED_OFF();

            if (FAILSAFE_DELAY && !failsafe_active &&
                ((timeMs - linkLossTimeMs) > FAILSAFE_DELAY))
            {
                failsafe_active = true;
                FailSafeHandling();
                nextBeaconTimeMs = (timeMs + 1000UL * BEACON_INTERVAL) | 1; //beacon activating...
            }

            if (BEACON_FREQUENCY && nextBeaconTimeMs &&
                    ((timeMs - nextBeaconTimeMs) < 0x80000000)) {

                // Indicate that the beacon is now active so we can trigger extra ones below
                beacon_armed = true;

                // Only beacon when disarmed
                if (!Armed) {
                    beacon_send(false); // play cool tune
                    init_rfm();   // go back to normal RX
                    rx_reset();
                    nextBeaconTimeMs = (timeMs +  1000UL * BEACON_INTERVAL) | 1; // avoid 0 in time
                }
            }
        }

    } else {
        // Waiting for first packet, hop slowly
        if (getUsSince(lastPacketTimeUs) > (interval * HOPCHANNELS)) {
            lastPacketTimeUs = timeUs;
            willhop = true;
        }
    }

    if (willhop) {
        rf_channel++;
        if ((rf_channel == MAXHOPS) || hopchannels[rf_channel] == 0) rf_channel = 0;

        if (BEACON_FREQUENCY && nextBeaconTimeMs && beacon_armed) {
            // Listen for RSSI on beacon channel briefly for 'trigger'
            uint8_t brssi = beaconGetRSSI();
            if (brssi > ((beacon_rssi_avg>>2) + 20)) {
                nextBeaconTimeMs = timeMs + 1000L;
            }
            beacon_rssi_avg = (beacon_rssi_avg * 3 + brssi * 4) >> 2;

            rfmSetCarrierFrequency(BEACON_FREQUENCY);
        }

        rfmSetChannel(rf_channel);
        rx_reset();
        willhop = false;
    }
}

uint8_t OpenLRS_GetRSSI(void)
{
    if(failsafe_active)
        return 0;
    else {
        uint16_t LQ = linkQuality & 0x7fff;
        // count number of 1s in LinkQuality
        LQ  = LQ - ((LQ >> 1) & 0x5555);
        LQ  = (LQ & 0x3333) + ((LQ >> 2) & 0x3333);
        LQ  = LQ + (LQ >> 4);
        LQ &= 0x0F0F;
        LQ = (LQ * 0x0101) >> 8;

        switch(RSSI_TYPE) {
        case 0: //OPENLRS_RSSI_TYPE_COMBINED
            if ((uint8_t)LQ == 15) {
                return (uint8_t)((lastRSSI >> 1)+128);
            } else {
                return LQ * 9;
            }
        case 1: //OPENLRS_RSSI_TYPE_RSSI
            return lastRSSI;
        case 2: //OPENLRS_RSSI_TYPE_LINKQUALITY
            return (uint8_t)(LQ << 4);
        default:
            return 0;
        }
    }
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

    ReceiverInit();

    if (!OpenLRSInit()/* || !DELAY_Init()*/)
          while (TRUE)
            {
              TOGGLE_O_LED();                  // RFM22b not answering
              chThdSleepMilliseconds(500);   // Toggle LED forever
            }

    bool rssi_sampled = false;
    interval = getInterval();

    while(TRUE){
        /* This block of code determines the timing of when to call the loop method. It reaches a bit into
         * the internal state of that method to get the optimal timings. This is to keep the loop method as
         * similar as possible to the openLRSng implementation (for easier maintenance of compatibility)
         * while minimizing overhead spinning in a while loop.
         *
         * There are three reasons to go into loop:
         *  1. the ISR was triggered (packet was received)
         *  2. a little before the expected packet (to sample the RSSI while receiving packet)
         *  3. a little after expected packet (to channel hop when a packet was missing)
         */

        uint32_t delay_ms = 0;

        uint32_t time_since_packet_us = getUsSince(lastPacketTimeUs);

        if (!rssi_sampled) {
            // If we had not sampled RSSI yet, schedule a bit early to try and catch while "packet is in the air"
            uint32_t time_till_measure_rssi_us  = (interval - packet_advance_time_us) - time_since_packet_us;
            delay_ms = (time_till_measure_rssi_us + 999) / 1000;
        } else {
            // If we have sampled RSSI we want to schedule to hop when a packet has been missed
            uint32_t time_till_timeout_us  = (interval + packet_timeout_us) - time_since_packet_us;
            delay_ms = (time_till_timeout_us + 999) / 1000;
        }

        // Maximum delay based on packet time
        const uint32_t max_delay = (interval + packet_timeout_us) / 1000;
        if (delay_ms > max_delay) delay_ms = max_delay;

        if (chBSemWaitTimeout(&RFMDataReady,MS2ST(delay_ms)) == MSG_TIMEOUT) {
            received = false;

            if (!rssi_sampled) {
                // We timed out to sample RSSI
                if (numberOfLostPackets < 2) {

                    lastRSSITimeUs = lastPacketTimeUs;
                    lastRSSI = rfmGetRSSI(); // Read the RSSI value
                }
            } else {
                // We timed out because packet was missed
                rx_loop();
            }

            rssi_sampled = true;
        } else {
            // Process incoming data
            received = true;
            rx_loop();

            // When a packet has been received (processed below) indicate we need to sample a new RSSI
            rssi_sampled = false;
        }
    }
}
