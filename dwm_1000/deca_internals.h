#ifndef _DECA_INTERNAL_H
#define _DECA_INTERNAL_H

#define BUFFER_LEN 50

#define DWT_SUCCESS (0)
#define DWT_ERROR   (-1)

#define DWT_TIME_UNITS          (1.0/499.2e6/128.0) //!< = 15.65e-12 s

#define DWT_DEVICE_ID   (0xDECA0130) 		//!< DW1000 MP device ID

//! constants for selecting the bit rate for data TX (and RX)
//! These are defined for write (with just a shift) the TX_FCTRL register
#define DWT_BR_110K		0	//!< UWB bit rate 110 kbits/s
#define DWT_BR_850K		1	//!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8		2	//!< UWB bit rate 6.8 Mbits/s

//! constants for specifying the (Nominal) mean Pulse Repetition Frequency
//! These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs
#define DWT_PRF_16M		1	//!< UWB PRF 16 MHz
#define DWT_PRF_64M		2	//!< UWB PRF 64 MHz

//! constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols
#define DWT_PAC8		0	//!< PAC  8 (recommended for RX of preamble length  128 and below
#define DWT_PAC16		1	//!< PAC 16 (recommended for RX of preamble length  256
#define DWT_PAC32		2	//!< PAC 32 (recommended for RX of preamble length  512
#define DWT_PAC64		3	//!< PAC 64 (recommended for RX of preamble length 1024 and up

//! constants for specifying TX Preamble length in symbols
// preamble length (PE + TXPSR bits)
#define DWT_PLEN_4096	0x03	//! Standard preamble length 4096 symbols
#define DWT_PLEN_2048	0x0A	//! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536	0x06	//! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024	0x02	//! Standard preamble length 1024 symbols
#define DWT_PLEN_512	0x0D	//! Non-standard preamble length 512 symbols
#define DWT_PLEN_256	0x09	//! Non-standard preamble length 256 symbols
#define DWT_PLEN_128	0x05	//! Non-standard preamble length 128 symbols
#define DWT_PLEN_64		0x01	//! Standard preamble length 64 symbols

/* preamble codes. */
#define PREAMBLE_CODE_16MHZ_1       1
#define PREAMBLE_CODE_16MHZ_2       2
#define PREAMBLE_CODE_16MHZ_3       3
#define PREAMBLE_CODE_16MHZ_4       4
#define PREAMBLE_CODE_16MHZ_5       5
#define PREAMBLE_CODE_16MHZ_6       6
#define PREAMBLE_CODE_16MHZ_7       7
#define PREAMBLE_CODE_16MHZ_8       8
#define PREAMBLE_CODE_64MHZ_9       9
#define PREAMBLE_CODE_64MHZ_10      10
#define PREAMBLE_CODE_64MHZ_11      11
#define PREAMBLE_CODE_64MHZ_12      12
#define PREAMBLE_CODE_64MHZ_17      17
#define PREAMBLE_CODE_64MHZ_18      18
#define PREAMBLE_CODE_64MHZ_19      19
#define PREAMBLE_CODE_64MHZ_20      20

/* channel of operation. */
#define CHANNEL_1   1
#define CHANNEL_2   2
#define CHANNEL_3   3
#define CHANNEL_4   4
#define CHANNEL_5   5
#define CHANNEL_7   7

/* frame length settings. */
#define FRAME_LENGTH_NORMAL 0x00
#define FRAME_LENGTH_EXTENDED 0x03


//! callback events
#define DWT_SIG_RX_NOERR			0
#define DWT_SIG_TX_DONE             1		// Frame has been sent
#define DWT_SIG_RX_OKAY             2       // Frame Received with Good CRC
#define DWT_SIG_RX_ERROR            3       // Frame Received but CRC is wrong
#define DWT_SIG_RX_TIMEOUT          4       // Timeout on receive has elapsed
#define DWT_SIG_TX_AA_DONE			6		// ACK frame has been sent (as a result of auto-ACK)

#define DWT_SIG_RX_PHR_ERROR        8       // Error found in PHY Header
#define DWT_SIG_RX_SYNCLOSS         9       // Un-recoverable error in Reed Solomon Decoder
#define DWT_SIG_RX_SFDTIMEOUT       10      // Saw preamble but got no SFD within configured time
#define DWT_SIG_RX_PTOTIMEOUT		11		// Got preamble detection timeout (no preamble detected)
#define DWT_SIG_TX_PENDING			12		// Delayed TX is pending
#define DWT_SIG_TX_ERROR			13      // TX failed
#define DWT_SIG_RX_PENDING 			14 		// RX has been re-enabled

#define DWT_SFDTOC_DEF				0x1041	// default SFD timeout value

#define DWT_PHRMODE_STD				0x0		// standard PHR mode
#define DWT_PHRMODE_EXT				0x3		// DW proprietary extended frames PHR mode

// Defined constants for "mode" bitmask parameter passed into dwt_starttx() function.
#define DWT_START_TX_IMMEDIATE      0
#define DWT_START_TX_DELAYED        1
#define DWT_RESPONSE_EXPECTED		2
#define DWT_RESPONSE_DELAYED_EXPECTED 3


//frame filtering configuration options
#define DWT_FF_NOTYPE_EN			0x000			// no frame types allowed (FF disabled)
#define DWT_FF_COORD_EN				0x002			// behave as coordinator (can receive frames with no dest address (PAN ID has to match))
#define DWT_FF_BEACON_EN			0x004			// beacon frames allowed
#define DWT_FF_DATA_EN				0x008			// data frames allowed
#define DWT_FF_ACK_EN				0x010			// ack frames allowed
#define DWT_FF_MAC_EN				0x020			// mac control frames allowed
#define DWT_FF_RSVD_EN				0x040			// reserved frame types allowed

//DW1000 interrupt events
#define DWT_INT_TFRS			0x00000080			// frame sent
#define DWT_INT_LDED            0x00000400			// micro-code has finished execution
#define DWT_INT_RFCG			0x00004000			// frame received with good CRC
#define DWT_INT_RPHE			0x00001000			// receiver PHY header error
#define DWT_INT_RFCE			0x00008000			// receiver CRC error
#define DWT_INT_RFSL			0x00010000			// receiver sync loss error
#define DWT_INT_RFTO			0x00020000			// frame wait timeout
#define DWT_INT_RXOVRR			0x00100000			// receiver overrun
#define DWT_INT_RXPTO			0x00200000			// preamble detect timeout
#define DWT_INT_SFDT			0x04000000			// SFD timeout
#define DWT_INT_ARFE			0x20000000			// frame rejected (due to frame filtering configuration)


//DW1000 SLEEP and WAKEUP configuration parameters
#define DWT_LOADLDO		 0x1000						 // ONW_LLDO - on wakeup load the LDO tune value
#define DWT_LOADUCODE    0x0800                      // ONW_LLDE - on wakeup load the LDE ucode
#define DWT_PRESRV_SLEEP 0x0100                      // PRES_SLEEP - on wakeup preserve sleep bit
#define DWT_LOADOPSET    0x0080						 // ONW_L64P - on wakeup load operating parameter set for 64 PSR
#define DWT_CONFIG       0x0040						 // ONW_LDC - on wakeup restore (load) the saved configurations (from AON array into HIF)
#define DWT_TANDV        0x0001                      // ONW_RADC - on wakeup run ADC to sample temperature and voltage sensor values

#define DWT_XTAL_EN		 0x10						// keep XTAL running during sleep
#define DWT_WAKE_SLPCNT  0x8						// wake up after sleep count
#define DWT_WAKE_CS      0x4						// wake up on chip select
#define DWT_WAKE_WK      0x2						// wake up on WAKEUP PIN
#define DWT_SLP_EN       0x1						// enable sleep/deep sleep functionality

//DW1000 INIT configuration parameters
#define DWT_LOADLDOTUNE   0x8
#define DWT_LOADTXCONFIG  0x4
#define DWT_LOADANTDLY    0x2
#define DWT_LOADXTALTRIM  0x1
#define DWT_LOADNONE	  0x0

//DW1000 OTP operating parameter set selection
#define DWT_OPSET_64LEN   0x0
#define DWT_OPSET_TIGHT   0x1
#define DWT_OPSET_DEFLT   0x2

#define DWT_PRF_16M_RFDLY   (513.9067f)
#define DWT_PRF_64M_RFDLY   (514.462f)


//in us
#define REPLY_TIME 7000
// timer/counter overflow (40 bits)
#define TIME_OVERFLOW 1099511627776

// Time resolution in micro-seconds of time based registers/values.
// Each bit in a timestamp counts for a period of approx. 15.65ps
#define TIME_RES 0.000015650040064103f
#define TIME_RES_INV 63897.6f

// Speed of radio waves [m/s] * timestamp resolution [~15.65ps] of DW1000
//299792458 m/s * 15.65 *10^-12  -> velocità luce con la risoluzione 15.65ps
#define SPEED_OF_LIGH 0.0046917639786159f
#define DISTANCE_OF_RADIO 0.0046917639786159f
#define DISTANCE_OF_RADIO_INV 213.139451293f

#define SPEED_OF_LIGHT      (299702547.0)     // in m/s in air

#define MASK_40BIT			(0x00FFFFFFFFFF)  // MP counter is 40 bits

#endif
