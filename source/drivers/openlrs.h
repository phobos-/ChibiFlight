#define OPENLRSNG_VERSION 0x0382

#define MAXHOPS      24
#define OPENLRS_PPM_NUM_CHANNELS 16

// Factory setting values, modify via the CLI

//####### RADIOLINK RF POWER (beacon is always 100/13/1.3mW) #######
// 7 == 100mW (or 1000mW with M3)
// 6 == 50mW (use this when using booster amp), (800mW with M3)
// 5 == 25mW
// 4 == 13mW
// 3 == 6mW
// 2 == 3mW
// 1 == 1.6mW
// 0 == 1.3mW
#define DEFAULT_RF_POWER 7

#define DEFAULT_CHANNEL_SPACING 5 // 50kHz
#define DEFAULT_HOPLIST 22,10,19,34,49,41
#define DEFAULT_RF_MAGIC 0xDEADFEED

//  0 -- 4800bps, best range
//  1 -- 9600bps, medium range
//  2 -- 19200bps, medium range
#define DEFAULT_DATARATE 2

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
#define DEFAULT_FLAGS       (CHANNELS_8 | TELEMETRY_PASSTHRU)

// helper macro for European PMR channels
#define EU_PMR_CH(x) (445993750L + 12500L * (x)) // valid for ch1-ch8

// helper macro for US FRS channels 1-7
#define US_FRS_CH(x) (462537500L + 25000L * (x)) // valid for ch1-ch7

#define DEFAULT_BEACON_FREQUENCY 435000000 // disable beacon
#define DEFAULT_BEACON_DEADTIME 30 // time to wait until go into beacon mode (30s)
#define DEFAULT_BEACON_INTERVAL 10 // interval between beacon transmits (10s)

#define BINDING_POWER     0x06 // not lowest since may result fail with RFM23BP

#define TELEMETRY_PACKETSIZE 9

#define BIND_MAGIC (0xDEC1BE15 + (OPENLRSNG_VERSION & 0xfff0))
#define BINDING_VERSION ((OPENLRSNG_VERSION & 0x0ff0)>>4)

#define DEFAULT_FAILSAFE_DELAY 500

#define RFMTYPE 433
// HW frequency limits
#if (RFMTYPE == 868)
#  define MIN_RFM_FREQUENCY 848000000
#  define MAX_RFM_FREQUENCY 888000000
#  define DEFAULT_CARRIER_FREQUENCY 868000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 868000000 // Hz
#elif (RFMTYPE == 915)
#  define MIN_RFM_FREQUENCY 895000000
#  define MAX_RFM_FREQUENCY 935000000
#  define DEFAULT_CARRIER_FREQUENCY 915000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 915000000 // Hz
#else
#  define MIN_RFM_FREQUENCY 413000000
#  define MAX_RFM_FREQUENCY 463000000
#  define DEFAULT_CARRIER_FREQUENCY 435000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 435000000 // Hz
#endif

#define RFM22_DEVICE_TYPE                         0x00  // R
#define RFM22_DT_MASK                             0x1F

#define OPENLRS_HOPCHANNEL_NUMELEM 6

enum RF_MODE {
  Available, Transmit, Receive, Transmitted, Received,
};

enum gpio_direction { GPIO0_TX_GPIO1_RX, GPIO0_RX_GPIO1_TX };

typedef enum { OPENLRS_RSSI_TYPE_COMBINED=0, OPENLRS_RSSI_TYPE_RSSI=1, OPENLRS_RSSI_TYPE_LINKQUALITY=2 }  __attribute__((packed)) OpenLRSRSSI_TypeOptions;
typedef enum { OPENLRSSTATUS_FAILSAFEACTIVE_INACTIVE=0, OPENLRSSTATUS_FAILSAFEACTIVE_ACTIVE=1 }  __attribute__((packed)) OpenLRSStatusFailsafeActiveOptions;

THD_FUNCTION(OpenLRSThread, arg);
