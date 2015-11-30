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
//extern uint16_t    ReceiverData[16];
extern volatile bool Armed;

struct bind_data {
  uint8_t version;
  uint32_t serial_baudrate;
  uint32_t rf_frequency;
  uint32_t rf_magic;
  uint8_t rf_power;
  uint8_t rf_channel_spacing;
  uint8_t hopchannel[MAXHOPS];
  uint8_t modem_params;
  uint8_t flags;
} __attribute__((packed));

struct openlrs_dev {
  // The PPM buffer
  int16_t ppm[OPENLRS_PPM_NUM_CHANNELS];

  // Flag to indicate if link every acquired
  bool link_acquired;

  // Active bound information data
  struct bind_data bind_data;

  // Beacon settings
  uint32_t beacon_frequency;
  uint8_t beacon_delay;
  uint8_t beacon_period;

  enum RF_MODE rf_mode;
  uint32_t rf_channel;

  uint8_t rx_buf[64];
  uint8_t tx_buf[9];

  // Variables from OpenLRS for radio control
  uint8_t hopcount;
  uint32_t lastPacketTimeUs;
  uint32_t numberOfLostPackets;
  uint16_t lastAFCCvalue;
  uint32_t lastRSSITimeUs;
  bool willhop;
  uint32_t nextBeaconTimeMs;
  uint32_t linkLossTimeMs;
  uint32_t failsafeDelay;
  uint32_t beacon_rssi_avg;

  enum gpio_direction gpio_direction;

  uint8_t RSSI_Type;

  uint16_t LinkQuality;
  uint8_t LastRSSI;
  uint8_t FailsafeActive;

  bool beacon_armed;
};

/*
 * Maximum speed SPI configuration (21MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig HSSpiConfig ={
  NULL,
  GPIOA,
  15,
  0
};

static void rx_reset(void);
static void rfmSetCarrierFrequency(uint32_t f);
static uint8_t rfmGetRSSI(void);
static void to_rx_mode(void);
static void tx_packet(struct openlrs_dev *openlrs_dev, uint8_t* pkt, uint8_t size);

// SPI read/write functions
static void rfm22_claimBus(void);
static void rfm22_releaseBus(void);
static void rfm22_write_claim(uint8_t addr, uint8_t data);
static void rfm22_write(uint8_t addr,uint8_t data);
static uint8_t rfm22_read_claim(uint8_t addr);
static uint8_t rfm22_read(uint8_t addr);

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

static const uint8_t pktsizes[8] = { 0, 7, 11, 12, 16, 17, 21, 0 };

static const uint8_t OUT_FF[64] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

const uint8_t default_hop_list[] = {DEFAULT_HOPLIST};

const uint32_t packet_advance_time_us = 1500;
const uint32_t packet_timeout_us = 1000;

const struct rfm22_modem_regs bind_params =
{ 9600, 0x05, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x20, 0x24, 0x4e, 0xa5, 0x2c, 0x23, 0x30 };

//! Global device handle, required for IRQ handler
static struct openlrs_dev * g_openlrs_dev;
/*
static struct bind_data binding = {
    BINDING_VERSION,
    125000,
    DEFAULT_CARRIER_FREQUENCY,
    DEFAULT_RF_MAGIC,
    DEFAULT_RF_POWER,
    DEFAULT_CHANNEL_SPACING,
    {DEFAULT_HOPLIST},
    DEFAULT_DATARATE,
    DEFAULT_FLAGS
};*/

static struct bind_data binding = {
    BINDING_VERSION,
    125000,
    DEFAULT_CARRIER_FREQUENCY,
    0xABCDEF,
    0,
    25,
    {18, 36, 54, 72, 90, 108},
    4,
    CHANNELS_4_4 | TELEMETRY_OFF
};

/*****************************************************************************
* OpenLRS data formatting utilities
*****************************************************************************/

static uint8_t getPacketSize(struct bind_data *bd)
{
	return pktsizes[(bd->flags & 0x07)];
}

static uint32_t getInterval(struct bind_data *bd)
{
	uint32_t ret;
	// Sending a x byte packet on bps y takes about (emperical)
	// usec = (x + 15) * 8200000 / baudrate
#define BYTES_AT_BAUD_TO_USEC(bytes, bps, div) ((uint32_t)((bytes) + (div?20:15)) * 8200000L / (uint32_t)(bps))

	ret = (BYTES_AT_BAUD_TO_USEC(getPacketSize(bd), modem_params[bd->modem_params].bps, bd->flags&DIVERSITY_ENABLED) + 2000);

	if (bd->flags & TELEMETRY_MASK) {
		ret += (BYTES_AT_BAUD_TO_USEC(TELEMETRY_PACKETSIZE, modem_params[bd->modem_params].bps, bd->flags&DIVERSITY_ENABLED) + 1000);
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

static void unpackChannels(uint8_t config, int16_t PPM[], uint8_t *p)
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
static void rescaleChannels(int16_t PPM[])
{
    uint32_t i;
	for (i = 0; i < OPENLRS_PPM_NUM_CHANNELS; i++) {
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
}

static uint8_t countSetBits(uint16_t x)
{
	x  = x - ((x >> 1) & 0x5555);
	x  = (x & 0x3333) + ((x >> 2) & 0x3333);
	x  = x + (x >> 4);
	x &= 0x0F0F;
	return (x * 0x0101) >> 8;
}

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

static void delay(uint32_t time)
{
    chThdSleepMilliseconds(time);
}

/*****************************************************************************
* OpenLRS hardware access
*****************************************************************************/

#define NOP() __asm__ __volatile__("nop")

#define RF22B_PWRSTATE_POWERDOWN    0x00
#define RF22B_PWRSTATE_READY	    RFM22_opfc1_xton
#define RF22B_PWRSTATE_RX	        (RFM22_opfc1_rxon | RFM22_opfc1_xton)
#define RF22B_PWRSTATE_TX	        (RFM22_opfc1_txon | RFM22_opfc1_xton)

#define RF22B_PACKET_SENT_INTERRUPT          RFM22_ie1_enpksent
#define RF22B_RX_PACKET_RECEIVED_IRQ         RFM22_ie1_enpkvalid

static void rfmSetChannel(struct openlrs_dev *openlrs_dev, uint8_t ch)
{
	//DEBUG_PRINTF(3,"rfmSetChannel %d\r\n", ch);
	uint8_t magicLSB = (openlrs_dev->bind_data.rf_magic & 0xff) ^ ch;
	rfm22_claimBus();
	rfm22_write(RFM22_frequency_hopping_channel_select, openlrs_dev->bind_data.hopchannel[ch]);
	rfm22_write(RFM22_transmit_header3 + 3, magicLSB);
	rfm22_write(RFM22_check_header3 + 3, magicLSB);
	rfm22_releaseBus();
}

static uint8_t rfmGetRSSI(void)
{
	return rfm22_read_claim(0x26);
}

static uint16_t rfmGetAFCC(void)
{
	return (((uint16_t)rfm22_read_claim(0x2B) << 2) | ((uint16_t)rfm22_read_claim(0x2C) >> 6));
}

static void setModemRegs(const struct rfm22_modem_regs* r)
{
	//DEBUG_PRINTF(3,"setModemRegs\r\n");
	rfm22_claimBus();
	rfm22_write(RFM22_if_filter_bandwidth, r->r_1c);
	rfm22_write(RFM22_afc_loop_gearshift_override, r->r_1d);
	rfm22_write(RFM22_afc_timing_control, r->r_1e);
	rfm22_write(RFM22_clk_recovery_oversampling_ratio, r->r_20);
	rfm22_write(RFM22_clk_recovery_offset2, r->r_21);
	rfm22_write(RFM22_clk_recovery_offset1, r->r_22);
	rfm22_write(RFM22_clk_recovery_offset0, r->r_23);
	rfm22_write(RFM22_clk_recovery_timing_loop_gain1, r->r_24);
	rfm22_write(RFM22_clk_recovery_timing_loop_gain0, r->r_25);
	rfm22_write(RFM22_afc_limiter, r->r_2a);
	rfm22_write(RFM22_tx_data_rate1, r->r_6e);
	rfm22_write(RFM22_tx_data_rate0, r->r_6f);
	rfm22_write(RFM22_modulation_mode_control1, r->r_70);
	rfm22_write(RFM22_modulation_mode_control2, r->r_71);
	rfm22_write(RFM22_frequency_deviation, r->r_72);
	rfm22_releaseBus();
}

static void rfmSetCarrierFrequency(uint32_t f)
{
	//DEBUG_PRINTF(3,"rfmSetCarrierFrequency %d\r\n", f);
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
	rfm22_claimBus();
	rfm22_write(RFM22_frequency_band_select, RFM22_fbs_sbse + (hbsel ? RFM22_fbs_hbsel : 0) + (fb & RFM22_fb_mask));
	rfm22_write(RFM22_nominal_carrier_frequency1, (fc >> 8));
	rfm22_write(RFM22_nominal_carrier_frequency0, (fc & 0xff));
	rfm22_releaseBus();
}

static void init_rfm(struct openlrs_dev *openlrs_dev, uint8_t isbind)
{
	/*DEBUG_PRINTF(2,"init_rfm %d\r\n", isbind);

	if (!isbind) {
		DEBUG_PRINTF(2, "Binding settings:\r\n");
		PIOS_Thread_Sleep(10);
		DEBUG_PRINTF(2, "  version: %d\r\n", openlrs_dev->bind_data.version);
		PIOS_Thread_Sleep(10);
		DEBUG_PRINTF(2, "  serial_baudrate: %d\r\n", openlrs_dev->bind_data.serial_baudrate);
		PIOS_Thread_Sleep(10);
		DEBUG_PRINTF(2, "  rf_frequency: %d\r\n", openlrs_dev->bind_data.rf_frequency);
		PIOS_Thread_Sleep(10);
		DEBUG_PRINTF(2, "  rf_power: %d\r\n", openlrs_dev->bind_data.rf_power);
		PIOS_Thread_Sleep(10);
		DEBUG_PRINTF(2, "  rf_channel_spacing: %d\r\n", openlrs_dev->bind_data.rf_channel_spacing);
		PIOS_Thread_Sleep(10);
		DEBUG_PRINTF(2, "  modem_params: %d\r\n", openlrs_dev->bind_data.modem_params);
		PIOS_Thread_Sleep(10);
		DEBUG_PRINTF(2, "  flags: %d\r\n", openlrs_dev->bind_data.flags);
		PIOS_Thread_Sleep(10);
	}*/

	rfm22_claimBus();
	rfm22_read(RFM22_interrupt_status1);   // read status, clear interrupt
	rfm22_read(RFM22_interrupt_status2);
	rfm22_write(RFM22_interrupt_enable2, 0x00);    // disable interrupts
	rfm22_write(RFM22_op_and_func_ctrl1, RF22B_PWRSTATE_READY); // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
	rfm22_write(RFM22_xtal_osc_load_cap, 0x7f);   // c = 12.5p
	rfm22_write(RFM22_cpu_output_clk, 0x05);
	switch (openlrs_dev->gpio_direction) {
	case GPIO0_TX_GPIO1_RX:
		rfm22_write(RFM22_gpio0_config, RFM22_gpio0_config_txstate);    // gpio0 TX State
		rfm22_write(RFM22_gpio1_config, RFM22_gpio1_config_rxstate);    // gpio1 RX State
		break;
	case GPIO0_RX_GPIO1_TX:
		rfm22_write(RFM22_gpio0_config, RFM22_gpio0_config_rxstate);    // gpio0 RX State
		rfm22_write(RFM22_gpio1_config, RFM22_gpio1_config_txstate);    // gpio1 TX State
		break;
	}
	rfm22_write(RFM22_gpio2_config, 0xfd);    // gpio 2 VDD
	rfm22_write(RFM22_io_port_config, RFM22_io_port_default);    // gpio    0, 1,2 NO OTHER FUNCTION.
	rfm22_releaseBus();

	if (isbind) {
		setModemRegs(&bind_params);
	} else {
		setModemRegs(&modem_params[openlrs_dev->bind_data.modem_params]);
	}

	// Packet settings
	rfm22_claimBus();
	rfm22_write(RFM22_data_access_control, 0x8c);    // enable packet handler, msb first, enable crc,
	rfm22_write(RFM22_header_control1, 0x0f);    // no broadcast, check header bytes 3,2,1,0
	rfm22_write(RFM22_header_control2, 0x42);    // 4 byte header, 2 byte synch, variable pkt size
	rfm22_write(RFM22_preamble_length, (openlrs_dev->bind_data.flags & DIVERSITY_ENABLED)?0x14:0x0a);    // 40 bit preamble, 80 with diversity
	rfm22_write(RFM22_preamble_detection_ctrl1, 0x2a);    // preath = 5 (20bits), rssioff = 2
	rfm22_write(RFM22_sync_word3, 0x2d);    // synchronize word 3
	rfm22_write(RFM22_sync_word2, 0xd4);    // synchronize word 2
	rfm22_write(RFM22_sync_word1, 0x00);    // synch word 1 (not used)
	rfm22_write(RFM22_sync_word0, 0x00);    // synch word 0 (not used)

	uint32_t magic = isbind ? BIND_MAGIC : openlrs_dev->bind_data.rf_magic;
	uint8_t i;
	for (i = 0; i < 4; i++) {
		rfm22_write(RFM22_transmit_header3 + i, (magic >> 24) & 0xff);   // tx header
		rfm22_write(RFM22_check_header3 + i, (magic >> 24) & 0xff);   // rx header
		magic = magic << 8; // advance to next byte
	}

	rfm22_write(RFM22_header_enable3, 0xff);    // all the bit to be checked
	rfm22_write(RFM22_header_enable2, 0xff);    // all the bit to be checked
	rfm22_write(RFM22_header_enable1, 0xff);    // all the bit to be checked
	rfm22_write(RFM22_header_enable0, 0xff);    // all the bit to be checked

	if (isbind) {
		rfm22_write(RFM22_tx_power, BINDING_POWER);
	} else {
		rfm22_write(RFM22_tx_power, openlrs_dev->bind_data.rf_power);
	}

	rfm22_write(RFM22_frequency_hopping_channel_select, 0);
	rfm22_write(RFM22_frequency_hopping_step_size, openlrs_dev->bind_data.rf_channel_spacing);   // channel spacing

	rfm22_write(RFM22_frequency_offset1, 0x00);
	rfm22_write(RFM22_frequency_offset2, 0x00);    // no offset

	rfm22_releaseBus();

	rfmSetCarrierFrequency(isbind ? BINDING_FREQUENCY : openlrs_dev->bind_data.rf_frequency);
}

static void to_rx_mode(void)
{
	//DEBUG_PRINTF(3,"to_rx_mode\r\n");
	rfm22_claimBus();
	rfm22_read(RFM22_interrupt_status1);
	rfm22_read(RFM22_interrupt_status2);
	rfm22_write(RFM22_op_and_func_ctrl1, RF22B_PWRSTATE_READY);
	rfm22_releaseBus();
	delay(10);
	rx_reset();
	NOP();
}

static void clearFIFO(void)
{
	//DEBUG_PRINTF(3,"clearFIFO\r\n");
	rfm22_claimBus();
	rfm22_write(RFM22_op_and_func_ctrl2, 0x03);
	rfm22_write(RFM22_op_and_func_ctrl2, 0x00);
	rfm22_releaseBus();
}

static void rx_reset(void)
{
	//DEBUG_PRINTF(3,"rx_reset\r\n");
	rfm22_write_claim(RFM22_op_and_func_ctrl1, RF22B_PWRSTATE_READY);
	rfm22_write_claim(RFM22_rx_fifo_control, 36);	 // threshold for rx almost full, interrupt when 1 byte received
	clearFIFO();
	rfm22_claimBus();
	rfm22_write(RFM22_op_and_func_ctrl1, RF22B_PWRSTATE_RX);   // to rx mode
	rfm22_write(RFM22_interrupt_enable1, RF22B_RX_PACKET_RECEIVED_IRQ);
	rfm22_read(RFM22_interrupt_status1);   //read the Interrupt Status1 register
	rfm22_read(RFM22_interrupt_status2);
	rfm22_releaseBus();
}

// TODO: move into dev structure
uint32_t tx_start = 0;

static void tx_packet_async(struct openlrs_dev *openlrs_dev, uint8_t* pkt, uint8_t size)
{
	rfm22_claimBus();
	rfm22_write(RFM22_transmit_packet_length, size);   // total tx size
	uint8_t i;
	for (i = 0; i < size; i++) {
		rfm22_write(RFM22_fifo_access, pkt[i]);
	}

	rfm22_write(RFM22_interrupt_enable1, RF22B_PACKET_SENT_INTERRUPT);
	rfm22_read(RFM22_interrupt_status1);	  //read the Interrupt Status1 register
	rfm22_read(RFM22_interrupt_status2);
	tx_start = micros();
	rfm22_write(RFM22_op_and_func_ctrl1, RF22B_PWRSTATE_TX);	// to tx mode
	rfm22_releaseBus();

	openlrs_dev->rf_mode = Transmit;
}

static void tx_packet(struct openlrs_dev *openlrs_dev, uint8_t* pkt, uint8_t size)
{
	tx_packet_async(openlrs_dev, pkt, size);
	//PIOS_Semaphore_Take(&RFMDataReady, 25); //TODO

	if (openlrs_dev->rf_mode == Transmit) {
		//DEBUG_PRINTF(2,"OLRS ERR: tx_packet timeout\r\n");
		init_rfm(openlrs_dev, false); // reset modem
	}
}

static void beacon_tone(int16_t hz, int16_t len) //duration is now in half seconds.
{
	//DEBUG_PRINTF(2,"beacon_tone: %d %d\r\n", hz, len*2);
	int16_t d = 500000 / hz; // better resolution

	TURN_B2_LED_ON();

	if (d < 1) {
		d = 1;
	}

	rfm22_claimBus();

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

	rfm22_releaseBus();

	TURN_B2_LED_OFF();

}


static uint8_t beaconGetRSSI(struct openlrs_dev *openlrs_dev)
{
	uint16_t rssiSUM=0;

	rfmSetCarrierFrequency(openlrs_dev->beacon_frequency);
	rfm22_write_claim(RFM22_frequency_hopping_channel_select, 0); // ch 0 to avoid offset
	delay(1);
	rssiSUM+=rfmGetRSSI();
	delay(1);
	rssiSUM+=rfmGetRSSI();
	delay(1);
	rssiSUM+=rfmGetRSSI();
	delay(1);
	rssiSUM+=rfmGetRSSI();

	return rssiSUM>>2;
}

static void beacon_send(struct openlrs_dev *openlrs_dev, bool static_tone)
{
	//DEBUG_PRINTF(2,"beacon_send\r\n");
	rfm22_claimBus();
	rfm22_read(0x03);   // read status, clear interrupt
	rfm22_read(0x04);
	rfm22_write(0x06, 0x00);    // no wakeup up, lbd,
	rfm22_write(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
	rfm22_write(0x09, 0x7f);  // (default) c = 12.5p
	rfm22_write(0x0a, 0x05);
	rfm22_write(0x0b, 0x12);    // gpio0 TX State
	rfm22_write(0x0c, 0x15);    // gpio1 RX State
	rfm22_write(0x0d, 0xfd);    // gpio 2 micro-controller clk output
	rfm22_write(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.

	rfm22_write(0x70, 0x2C);    // disable manchest

	rfm22_write(0x30, 0x00);    //disable packet handling

	rfm22_write(0x79, 0);	// start channel

	rfm22_write(0x7a, 0x05);   // 50khz step size (10khz x value) // no hopping

	rfm22_write(0x71, 0x12);   // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
	rfm22_write(0x72, 0x02);   // fd (frequency deviation) 2*625Hz == 1.25kHz

	rfm22_write(0x73, 0x00);
	rfm22_write(0x74, 0x00);    // no offset
	rfm22_releaseBus();

	rfmSetCarrierFrequency(openlrs_dev->beacon_frequency);

	rfm22_write_claim(0x6d, 0x07);   // 7 set max power 100mW

	delay(10);
	rfm22_write_claim(0x07, RF22B_PWRSTATE_TX);	// to tx mode
	delay(10);

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

		rfm22_write(0x6d, 0x05);	// 5 set mid power 25mW
		delay(10);
		beacon_tone(440,1);

		rfm22_write(0x6d, 0x04);	// 4 set mid power 13mW
		delay(10);
		beacon_tone(349, 1);

		rfm22_write(0x6d, 0x02);	// 2 set min power 3mW
		delay(10);
		beacon_tone(175,1);

		rfm22_write(0x6d, 0x00);	// 0 set min power 1.3mW
		delay(10);
		beacon_tone(261, 2);
	}
	rfm22_write_claim(0x07, RF22B_PWRSTATE_READY);
}


/*****************************************************************************
* High level OpenLRS functions
*****************************************************************************/

// TODO: these should move into device structure, or deleted
// if not useful to be reported via GCS

#define ntohl(v) (				\
	(((v) & 0xFF000000) >> 24) |		\
	(((v) & 0x00FF0000) >>  8) |		\
	(((v) & 0x0000FF00) <<  8) |		\
	(((v) & 0x000000FF) << 24))

static uint8_t openlrs_bind_receive(struct openlrs_dev *openlrs_dev, uint32_t timeout)
{
	uint32_t start = millis();
	uint8_t  rxb;
	init_rfm(openlrs_dev, true);
	// TODO: move openlrs_dev->rf_mode into dev structure
	openlrs_dev->rf_mode = Receive;
	to_rx_mode();
	//DEBUG_PRINTF(2,"Waiting bind\r\n");

	uint32_t i = 0;

	while ((!timeout) || ((millis() - start) < timeout)) {
		delay(1);

		if (i++ % 100 == 0) {
			//DEBUG_PRINTF(2,"Waiting bind\r\n");
		    TOGGLE_B2_LED();
		}
		if (openlrs_dev->rf_mode == Received) {

			//DEBUG_PRINTF(2,"Got pkt\r\n");

			// TODO: CHECK + parse data packet (write command for that)
	        spiAcquireBus(&SPID3);
	        spiStart(&SPID3, &HSSpiConfig);
	        spiSelect(&SPID3);
	        spiSend(&SPID3, 1, &(uint8_t){0x7f});
	        spiExchange(&SPID3, 1, &(uint8_t){0x00}, &rxb);
			if (rxb == 'b') {
			    spiExchange(&SPID3, sizeof(struct bind_data), OUT_FF, (uint8_t*) &openlrs_dev->bind_data);
			    spiUnselect(&SPID3);
			    spiReleaseBus(&SPID3);

#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
				if(2 <= DEBUG_LEVEL && pios_com_debug_id > 0) {
					DEBUG_PRINTF(2, "Binding settings:\r\n");
					PIOS_Thread_Sleep(10);
					DEBUG_PRINTF(2, "  version: %d\r\n", openlrs_dev->bind_data.version);
					PIOS_Thread_Sleep(10);
					DEBUG_PRINTF(2, "  serial_baudrate: %d\r\n", openlrs_dev->bind_data.serial_baudrate);
					PIOS_Thread_Sleep(10);
					DEBUG_PRINTF(2, "  rf_frequency: %d\r\n", openlrs_dev->bind_data.rf_frequency);
					PIOS_Thread_Sleep(10);
					DEBUG_PRINTF(2, "  rf_power: %d\r\n", openlrs_dev->bind_data.rf_power);
					PIOS_Thread_Sleep(10);
					DEBUG_PRINTF(2, "  rf_channel_spacing: %d\r\n", openlrs_dev->bind_data.rf_channel_spacing);
					PIOS_Thread_Sleep(10);
					DEBUG_PRINTF(2, "  modem_params: %d\r\n", openlrs_dev->bind_data.modem_params);
					PIOS_Thread_Sleep(10);
					DEBUG_PRINTF(2, "  flags: %d\r\n", openlrs_dev->bind_data.flags);
					PIOS_Thread_Sleep(10);

					for (uint32_t i = 0; i < MAXHOPS; i++) {
						DEBUG_PRINTF(2, "    hop channel: %d\r\n", openlrs_dev->bind_data.hopchannel[i]);
						PIOS_Thread_Sleep(10);
					}
				}
#endif

				if (openlrs_dev->bind_data.version == BINDING_VERSION) {
					//DEBUG_PRINTF(2,"data good\r\n");
					rxb = 'B';
					tx_packet(openlrs_dev, &rxb, 1); // ACK that we got bound

					//OpenLRSData binding;
					//OpenLRSGet(&binding);
					binding.version = openlrs_dev->bind_data.version;
					binding.serial_baudrate = openlrs_dev->bind_data.serial_baudrate;
					binding.rf_frequency = openlrs_dev->bind_data.rf_frequency;
					binding.rf_magic = openlrs_dev->bind_data.rf_magic;
					binding.rf_power = openlrs_dev->bind_data.rf_power;
					binding.rf_channel_spacing = openlrs_dev->bind_data.rf_channel_spacing;
					binding.modem_params = openlrs_dev->bind_data.modem_params;
					binding.flags = openlrs_dev->bind_data.flags;
					uint32_t i;
					for (i = 0; i < OPENLRS_HOPCHANNEL_NUMELEM; i++)
						binding.hopchannel[i] = openlrs_dev->bind_data.hopchannel[i];
					//binding.beacon_frequency = openlrs_dev->beacon_frequency;
					//binding.beacon_delay = openlrs_dev->beacon_delay;
					//binding.beacon_period = openlrs_dev->beacon_period;
					//OpenLRSSet(&binding);
					//UAVObjSave(OpenLRSHandle(), 0);

					TOGGLE_B2_LED();

					return 1;
				}
			} else {
				rfm22_releaseBus();
			}

			openlrs_dev->rf_mode = Receive;
			rx_reset();
		}
	}
	return 0;
}

#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
static void printVersion(uint16_t v)
{
	char ver[8];
	ver[0] = '0' + ((v >> 8) & 0x0f);
	ver[1] = '.';
	ver[2] = '0' + ((v >> 4) & 0x0f);
  	if (v & 0x0f) {
    	ver[3] = '.';
    	ver[4] = '0' + (v & 0x0f);
    	ver[5] = '\r';
    	ver[6] = '\n';
    	ver[7] = '\0';
    } else {
    	ver[3] = '\r';
    	ver[4] = '\n';
    	ver[5] = '\0';
    }
    DEBUG_PRINTF(2, ver);
}
#endif

static void openlrs_setup(struct openlrs_dev *openlrs_dev, bool bind)
{
	//DEBUG_PRINTF(2,"OpenLRSng RX setup starting. Binding: %e\r\n", bind);
	delay(5);
#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
	printVersion(OPENLRSNG_VERSION);
#endif

	if ( bind ) {
		if (openlrs_bind_receive(openlrs_dev, 0)) {
			// TODO: save binding settings bindWriteEeprom();
			//DEBUG_PRINTF(2,"Saved bind data to EEPROM (not really yet -- TODO)\r\n");
		}
	}

	//DEBUG_PRINTF(2,"Entering normal mode\r\n");

	init_rfm(openlrs_dev, 0);   // Configure the RFM22B's registers for normal operation
	openlrs_dev->rf_channel = 0;
	rfmSetChannel(openlrs_dev, openlrs_dev->rf_channel);

	// Count hopchannels as we need it later
	openlrs_dev->hopcount = 0;
	while ((openlrs_dev->hopcount < MAXHOPS) && (openlrs_dev->bind_data.hopchannel[openlrs_dev->hopcount] != 0)) {
		openlrs_dev->hopcount++;
	}

	//################### RX SYNC AT STARTUP #################
	openlrs_dev->rf_mode = Receive;
	to_rx_mode();

	openlrs_dev->link_acquired = 0;
	openlrs_dev->lastPacketTimeUs = micros();

	//DEBUG_PRINTF(2,"OpenLRSng RX setup complete\r\n");
}

static void openlrs_rx_loop(struct openlrs_dev *openlrs_dev)
{
	uint32_t timeUs, timeMs;

	if (rfm22_read_claim(0x0C) == 0) {     // detect the locked module and reboot
		//DEBUG_PRINTF(2,"OLRS ERR: RX hang\r\n");
		init_rfm(openlrs_dev, 0);
		to_rx_mode();
	}

	timeUs = micros();
	timeMs = millis();

	//DEBUG_PRINTF(2,"Time: %d\r\n", timeUs);

	uint8_t *tx_buf = openlrs_dev->tx_buf;  // convenient variable

	if (openlrs_dev->rf_mode == Received) {

		//DEBUG_PRINTF(2,"Packet Received. Dt=%d\r\n", timeUs-openlrs_dev->lastPacketTimeUs);

		// Read the packet from RFM22b
        spiAcquireBus(&SPID3);
        spiStart(&SPID3, &HSSpiConfig);
        spiSelect(&SPID3);
        spiSend(&SPID3, 1, &(uint8_t){RFM22_fifo_access}); //creates temp variable and passes it's addr
        uint32_t packet_size = getPacketSize(&openlrs_dev->bind_data);
        spiExchange(&SPID3, packet_size, OUT_FF, openlrs_dev->rx_buf);
        spiUnselect(&SPID3);
        spiReleaseBus(&SPID3);

		openlrs_dev->lastAFCCvalue = rfmGetAFCC();

		TOGGLE_B2_LED();

		openlrs_dev->lastPacketTimeUs = timeUs;
		openlrs_dev->numberOfLostPackets = 0;
		openlrs_dev->LinkQuality <<= 1;
		openlrs_dev->LinkQuality |= 1;

		if ((openlrs_dev->rx_buf[0] & 0x3e) == 0x00) {
			// This flag indicates receiving PPM data

			unpackChannels(openlrs_dev->bind_data.flags & 7, openlrs_dev->ppm, openlrs_dev->rx_buf + 1);
			//rescaleChannels(openlrs_dev->ppm);

			// Call the PPM received callback if it's available.
			ReceiverFSM((uint16_t*)openlrs_dev->ppm);
		}
		else {
			// Not PPM data. Push into serial RX buffer.
			if ((openlrs_dev->rx_buf[0] & 0x38) == 0x38) {
				if ((openlrs_dev->rx_buf[0] ^ tx_buf[0]) & 0x80) {
					// We got new data... (not retransmission)
					tx_buf[0] ^= 0x80; // signal that we got it
					/*bool rx_need_yield;
					uint8_t data_len = openlrs_dev->rx_buf[0] & 7;
					if (openlrs_dev->rx_in_cb && (data_len > 0)) {
						(openlrs_dev->rx_in_cb) (openlrs_dev->rx_in_context, &openlrs_dev->rx_buf[1], data_len, NULL, &rx_need_yield);
					}*/
				}
			}
		}

		// Flag to indicate ever got a link
		openlrs_dev->link_acquired |= true;
		openlrs_dev->FailsafeActive = OPENLRSSTATUS_FAILSAFEACTIVE_INACTIVE;
		openlrs_dev->beacon_armed = false; // when receiving packets make sure beacon cannot emit

		// When telemetry is enabled we ack packets and send info about FC back
		if (openlrs_dev->bind_data.flags & TELEMETRY_MASK) {
			if ((tx_buf[0] ^ openlrs_dev->rx_buf[0]) & 0x40) {
				// resend last message
			} else {
				tx_buf[0] &= 0xc0;
				tx_buf[0] ^= 0x40; // swap sequence as we have new data

				// Check for data on serial link
				uint8_t bytes = 0;
				// Append data from the com interface if applicable.
				/*if (openlrs_dev->tx_out_cb) {
					// Try to get some data to send
					bool need_yield = false;
					bytes = (openlrs_dev->tx_out_cb) (openlrs_dev->tx_out_context, &tx_buf[1], 8, NULL, &need_yield);
				}*/

				if (bytes > 0) {
					tx_buf[0] |= (0x37 + bytes);
				} else {
					// tx_buf[0] lowest 6 bits left at 0
					tx_buf[1] = openlrs_dev->LastRSSI;
					/*if (FlightBatteryStateHandle()) {
						FlightBatteryStateData bat;
						FlightBatteryStateGet(&bat);
						// FrSky protocol normally uses 3.3V at 255 but
						// divider from display can be set internally
						tx_buf[2] = (uint8_t) bat.Voltage / 25.0f * 255;
						tx_buf[3] = (uint8_t) bat.Current / 60.0f * 255;
					} else {*/
						tx_buf[2] = 0; // these bytes carry analog info. package
						tx_buf[3] = 0; // battery here
					//}
					tx_buf[4] = (openlrs_dev->lastAFCCvalue >> 8);
					tx_buf[5] = openlrs_dev->lastAFCCvalue & 0xff;
					tx_buf[6] = countSetBits(openlrs_dev->LinkQuality & 0x7fff);
				}
			}

			// This will block until sent
			tx_packet(openlrs_dev, tx_buf, 9);
		}

		// Once a packet has been processed, flip back into receiving mode
		openlrs_dev->rf_mode = Receive;
		rx_reset();

		openlrs_dev->willhop = 1;
	}

	if (openlrs_dev->link_acquired) {
		// For missing packets to be properly trigger a well timed channel hop, this method should be called fairly close (but not sooner)
		// than 1ms after the packet was expected to trigger this path
		if ((openlrs_dev->numberOfLostPackets < openlrs_dev->hopcount) && (getUsSince(openlrs_dev->lastPacketTimeUs) > (getInterval(&openlrs_dev->bind_data) + packet_timeout_us))) {
			//DEBUG_PRINTF(2,"OLRS WARN: Lost packet: %d\r\n", openlrs_dev->numberOfLostPackets);
			// we lost packet, hop to next channel
			openlrs_dev->LinkQuality <<= 1;
			openlrs_dev->willhop = 1;
			if (openlrs_dev->numberOfLostPackets == 0) {
				openlrs_dev->linkLossTimeMs = timeMs;
				openlrs_dev->nextBeaconTimeMs = 0;
			}
			openlrs_dev->numberOfLostPackets++;
			openlrs_dev->lastPacketTimeUs += getInterval(&openlrs_dev->bind_data);
			openlrs_dev->willhop = 1;
		} else if ((openlrs_dev->numberOfLostPackets >= openlrs_dev->hopcount) && (getUsSince(openlrs_dev->lastPacketTimeUs) > (getInterval(&openlrs_dev->bind_data) * openlrs_dev->hopcount))) {
			//DEBUG_PRINTF(2,"ORLS WARN: Trying to resync\r\n");
			// hop slowly to allow resync with TX
			openlrs_dev->LinkQuality = 0;
			openlrs_dev->willhop = 1;
			openlrs_dev->lastPacketTimeUs = timeUs;
		}

		if (openlrs_dev->numberOfLostPackets) {

		    TURN_B2_LED_OFF();

			if (openlrs_dev->failsafeDelay &&
				(openlrs_dev->FailsafeActive == OPENLRSSTATUS_FAILSAFEACTIVE_INACTIVE) &&
				((timeMs - openlrs_dev->linkLossTimeMs) > ((uint32_t) openlrs_dev->failsafeDelay)))
			{
				//DEBUG_PRINTF(2,"Failsafe activated: %d %d\r\n", timeMs, openlrs_dev->linkLossTimeMs);
				openlrs_dev->FailsafeActive = OPENLRSSTATUS_FAILSAFEACTIVE_ACTIVE;
				//failsafeApply();
				FailSafeHandling();
				openlrs_dev->nextBeaconTimeMs = (timeMs + 1000UL * openlrs_dev->beacon_period) | 1; //beacon activating...
			}

			if ((openlrs_dev->beacon_frequency) && (openlrs_dev->nextBeaconTimeMs) &&
					((timeMs - openlrs_dev->nextBeaconTimeMs) < 0x80000000)) {

				// Indicate that the beacon is now active so we can trigger extra ones below
				openlrs_dev->beacon_armed = true;

				//DEBUG_PRINTF(2,"Beacon time: %d\r\n", openlrs_dev->nextBeaconTimeMs);
				// Only beacon when disarmed
				if (!Armed) {
					beacon_send(openlrs_dev, false); // play cool tune
					init_rfm(openlrs_dev, 0);   // go back to normal RX
					rx_reset();
					openlrs_dev->nextBeaconTimeMs = (timeMs +  1000UL * openlrs_dev->beacon_period) | 1; // avoid 0 in time
				}
			}
		}

	} else {
		// Waiting for first packet, hop slowly
		if (getUsSince(openlrs_dev->lastPacketTimeUs) > (getInterval(&openlrs_dev->bind_data) * openlrs_dev->hopcount)) {
			//DEBUG_PRINTF(3,"Trying to get first packet\r\n");
			openlrs_dev->lastPacketTimeUs = timeUs;
			openlrs_dev->willhop = 1;
		}
	}

	if (openlrs_dev->willhop == 1) {
		openlrs_dev->rf_channel++;

		if ((openlrs_dev->rf_channel == MAXHOPS) || (openlrs_dev->bind_data.hopchannel[openlrs_dev->rf_channel] == 0)) {
			openlrs_dev->rf_channel = 0;
		}

		if ((openlrs_dev->beacon_frequency) && (openlrs_dev->nextBeaconTimeMs) && openlrs_dev->beacon_armed) {
			// Listen for RSSI on beacon channel briefly for 'trigger'
			uint8_t brssi = beaconGetRSSI(openlrs_dev);
			if (brssi > ((openlrs_dev->beacon_rssi_avg>>2) + 20)) {
				openlrs_dev->nextBeaconTimeMs = timeMs + 1000L;
			}
			openlrs_dev->beacon_rssi_avg = (openlrs_dev->beacon_rssi_avg * 3 + brssi * 4) >> 2;

			rfmSetCarrierFrequency(openlrs_dev->bind_data.rf_frequency);
		}

		rfmSetChannel(openlrs_dev, openlrs_dev->rf_channel);
		rx_reset();
		openlrs_dev->willhop = 0;
	}

	// Update UAVO
	//OpenLRSStatusSet(&openlrs_status);
}

uint8_t OpenLRS_RSSI_Get(struct openlrs_dev *openlrs_dev)
{
	if(openlrs_dev->FailsafeActive == OPENLRSSTATUS_FAILSAFEACTIVE_ACTIVE)
		return 0;
	else {
		//OpenLRSData openlrs_data;
		//OpenLRSGet(&openlrs_data);

		uint16_t LQ = openlrs_dev->LinkQuality & 0x7fff;
		// count number of 1s in LinkQuality
		LQ  = LQ - ((LQ >> 1) & 0x5555);
		LQ  = (LQ & 0x3333) + ((LQ >> 2) & 0x3333);
		LQ  = LQ + (LQ >> 4);
		LQ &= 0x0F0F;
		LQ = (LQ * 0x0101) >> 8;

		switch(openlrs_dev->RSSI_Type) {
		case OPENLRS_RSSI_TYPE_COMBINED:
			if ((uint8_t)LQ == 15) {
				return (uint8_t)((openlrs_dev->LastRSSI >> 1)+128);
			} else {
				return LQ * 9;
			}
		case OPENLRS_RSSI_TYPE_RSSI:
			return openlrs_dev->LastRSSI;
		case OPENLRS_RSSI_TYPE_LINKQUALITY:
			return (uint8_t)(LQ << 4);
		default:
			return 0;
		}
	}
}

/*****************************************************************************
* Task and device setup
*****************************************************************************/

bool OpenLRSInit(struct openlrs_dev *openlrs_dev){

    // Before initializing everything, make sure device found
    rx_reset();
    uint8_t device_type = rfm22_read_claim(RFM22_DEVICE_TYPE) & RFM22_DT_MASK;
    if (device_type != 0x08)
        return false;

    if (binding.version == BINDING_VERSION) {
        openlrs_dev->bind_data.version = binding.version;
        openlrs_dev->bind_data.serial_baudrate = binding.serial_baudrate;
        openlrs_dev->bind_data.rf_frequency = binding.rf_frequency;
        openlrs_dev->bind_data.rf_magic = binding.rf_magic;
        openlrs_dev->bind_data.rf_power = binding.rf_power;
        openlrs_dev->bind_data.rf_channel_spacing = binding.rf_channel_spacing;
        openlrs_dev->bind_data.modem_params = binding.modem_params;
        openlrs_dev->bind_data.flags = binding.flags;
        uint32_t i;
        for (i = 0; i < OPENLRS_HOPCHANNEL_NUMELEM; i++)
            openlrs_dev->bind_data.hopchannel[i] = binding.hopchannel[i];
    }

    // Copy beacon settings over
    openlrs_dev->beacon_frequency = DEFAULT_BEACON_FREQUENCY;
    openlrs_dev->beacon_delay = DEFAULT_BEACON_DEADTIME;
    openlrs_dev->beacon_period = DEFAULT_BEACON_INTERVAL;

    openlrs_dev->failsafeDelay = DEFAULT_FAILSAFE_DELAY;

    // Bind the configuration to the device instance
    openlrs_dev->gpio_direction = GPIO0_TX_GPIO1_RX;

    if (openlrs_dev->bind_data.version == BINDING_VERSION)
        openlrs_setup(openlrs_dev, false);
    else
        openlrs_setup(openlrs_dev, true);

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

    static struct openlrs_dev openlrs_dev;
    g_openlrs_dev = &openlrs_dev;

    ReceiverInit();

    if (!OpenLRSInit(g_openlrs_dev))
          while (TRUE)
            {
              TOGGLE_O_LED();                  // RFM22b not answering
              chThdSleepMilliseconds(500);   // Toggle LED forever
            }

    bool rssi_sampled = false;

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

        uint32_t time_since_packet_us = getUsSince(g_openlrs_dev->lastPacketTimeUs);

        if (!rssi_sampled) {
            // If we had not sampled RSSI yet, schedule a bit early to try and catch while "packet is in the air"
            uint32_t time_till_measure_rssi_us  = (getInterval(&g_openlrs_dev->bind_data) - packet_advance_time_us) - time_since_packet_us;
            delay_ms = (time_till_measure_rssi_us + 999) / 1000;
            //DEBUG_PRINTF(3, "T1: %d\r\n", delay_ms);
        } else {
            // If we have sampled RSSI we want to schedule to hop when a packet has been missed
            uint32_t time_till_timeout_us  = (getInterval(&g_openlrs_dev->bind_data) + packet_timeout_us) - time_since_packet_us;
            delay_ms = (time_till_timeout_us + 999) / 1000;
            //DEBUG_PRINTF(3, "T2: %d %d\r\n", time_till_timeout_us,delay_ms, time_since_packet_us);
        }

        // Maximum delay based on packet time
        const uint32_t max_delay = (getInterval(&g_openlrs_dev->bind_data) + packet_timeout_us) / 1000;
        if (delay_ms > max_delay) delay_ms = max_delay;

        if (chBSemWaitTimeout(&RFMDataReady,MS2ST(delay_ms)) == MSG_TIMEOUT) {
            if (!rssi_sampled) {
                // We timed out to sample RSSI
                if (g_openlrs_dev->numberOfLostPackets < 2) {

                    g_openlrs_dev->lastRSSITimeUs = g_openlrs_dev->lastPacketTimeUs;
                    g_openlrs_dev->LastRSSI = rfmGetRSSI(); // Read the RSSI value

                    //DEBUG_PRINTF(3, "Sampled RSSI: %d %d\r\n", g_openlrs_dev->LastRSSI, delay);
                }
            } else {
                // We timed out because packet was missed
                //DEBUG_PRINTF(3, "ISR Timeout. Missed packet: %d %d %d\r\n", delay, getInterval(&g_openlrs_dev->bind_data), time_since_packet_us);
                openlrs_rx_loop(g_openlrs_dev);
            }

            rssi_sampled = true;
        } else {
            //DEBUG_PRINTF(3, "ISR %d %d %d\r\n", delay, getInterval(&g_openlrs_dev->bind_data), time_since_packet_us);

            // Process incoming data
            openlrs_rx_loop(g_openlrs_dev);

            // When a packet has been received (processed below) indicate we need to sample a new RSSI
            rssi_sampled = false;
        }


        //DEBUG_PRINTF(3, "Processing time %d\r\n", getUsSince(g_openlrs_dev->lastPacketTimeUs));
    }
}

void RFMISR(EXTDriver *extp, expchannel_t channel)
{
  (void) extp;
  (void) channel;

  chSysLockFromISR();
  if (g_openlrs_dev->rf_mode == Transmit) {
      g_openlrs_dev->rf_mode = Transmitted;
  }
  else if (g_openlrs_dev->rf_mode == Receive) {
      g_openlrs_dev->rf_mode = Received;
  }
  chBSemSignalI(&RFMDataReady);
  chSysUnlockFromISR();
}

/*****************************************************************************
* SPI Read/Write Functions
*****************************************************************************/

static void rfm22_claimBus(void)
{
    spiAcquireBus(&SPID3);
    spiStart(&SPID3, &HSSpiConfig);
}

static void rfm22_releaseBus(void)
{
    spiReleaseBus(&SPID3);
}

static void rfm22_write_claim(uint8_t addr, uint8_t data)
{
    uint8_t TxBuffer[2];

    TxBuffer[0] = addr | 0x80;
    TxBuffer[1] = data;
    spiAcquireBus(&SPID3);
    spiStart(&SPID3, &HSSpiConfig);
    spiSelect(&SPID3);
    spiSend(&SPID3, 2, TxBuffer);
    spiUnselect(&SPID3);
    spiReleaseBus(&SPID3);
}

static uint8_t rfm22_read_claim(uint8_t addr)
{
    uint8_t RxBuffer[2];
    uint8_t TxBuffer[2];

    TxBuffer[0] = addr & 0x7F;
    TxBuffer[1] = 0xFF;
    spiAcquireBus(&SPID3);
    spiStart(&SPID3, &HSSpiConfig);
    spiSelect(&SPID3);
    spiExchange(&SPID3, 2, TxBuffer, RxBuffer);
    spiUnselect(&SPID3);
    spiReleaseBus(&SPID3);
    return RxBuffer[1];
}

static void rfm22_write(uint8_t addr,uint8_t data)
{
    uint8_t TxBuffer[2];

    TxBuffer[0] = addr | 0x80;
    TxBuffer[1] = data;

    spiSelect(&SPID3);
    spiSend(&SPID3, 2, TxBuffer);
    spiUnselect(&SPID3);
}

static uint8_t rfm22_read(uint8_t addr)
{
    uint8_t RxBuffer[2];
    uint8_t TxBuffer[2];

    TxBuffer[0] = addr & 0x7F;
    TxBuffer[1] = 0xFF;
    spiSelect(&SPID3);
    spiExchange(&SPID3, 2, TxBuffer, RxBuffer);
    spiUnselect(&SPID3);
    return RxBuffer[1];
}
