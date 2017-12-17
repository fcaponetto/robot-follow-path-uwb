#ifndef _DWM_1000_H
#define _DWM_1000_H

#include <Arduino.h>
#include <SPI.h>
#include "deca_regs.h"
#include "deca_types.h"
#include "deca_internals.h"
#include "conversion.h"

/*debug mode
	DEBUG 0 - no print
	DEBUG 1 - essential print
	DEBUG 2 - verbose mode
*/
#ifndef DEBUG
#define DEBUG 0
#endif

//TODO levare
#define DWSFD_BIT 17
#define TNSSFD_BIT 20
#define RNSSFD_BIT 21

typedef struct
{
    /* pins */
    unsigned int _ss;
    unsigned int _irq;
    unsigned int _rst;

    uint32      deviceID;
    uint32      partID;
    uint32      lotID;
    byte       chan;               // added chan here - used in the reading of acc
    byte       longFrames;        // flag in non-standard long frame mode
    byte		otprev;			// OTP revision number (read during initialisation)
    uint32      txFCTRL;           // keep TX_FCTRL register config
    uint16      rfrxDly;            // rf delay (delay though the RF blocks before the signal comes out of the antenna i.e. "antenna delay")
    uint16      rftxDly;            // rf delay (delay though the RF blocks before the signal comes out of the antenna i.e. "antenna delay")
    uint32      antennaDly;         // antenna delay read from OTP 64 PRF value is in high 16 bits and 16M PRF in low 16 bits
    byte       xtrim;              // xtrim value read from OTP
    byte       dblbuffon;          // double rx buffer mode flag
    uint32      sysCFGreg;         // local copy of system config register
    uint32      txPowCfg[12];       // stores the Tx power configuration read from OTP (6 channels consecutively with PRF16 then 64, e.g. Ch 1 PRF16 is index 0 and 64 index 1)


    uint32      states[3];         //MP workaround debug states register
    byte       statescount;
    byte		wait4resp;			//wait 4 response was set with last TX start command
    int         prfIndex;

    uint32		ldoTune;			//low 32 bits of LDO tune value

    // callback
    void(*_dwm_txcallback)(void) = 0;
    void(*_dwm_rxcallback)(void) = 0;

    void(*_dwm_senddonecallback)(void) = 0;
    void(*_dwm_receivedonecallback)(void) = 0;
    void(*_dwm_receivefailedcallback)(void) = 0;
    void(*_dwm_receiveTimeoutcallback)(void) = 0;

} dwm_local_data_t;

//#pragma pack(1)
typedef struct
{
    uint8 channel ;         //!< channel number {1, 2, 3, 4, 5, 7 }
    uint8 pulseFrequency;   //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
    uint8 preambleLength;	//!< DWT_PLEN_64..DWT_PLEN_4096
    uint8 pacSize ;			//!< Acquisition Chunk Size (Relates to RX preamble length)
    uint8 preambleCode ;    //!< TX preamble code && RX preamble code
    uint8 nsSFD ;			//!< Boolean should we use non-standard SFD for better performance
    uint8 dataRate ;		//!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
    uint8 phrMode ;			//!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint8 smartPowerEn ;    //!< Smart Power enable / disable
    uint16 sfdTO ;			//!< SFD timeout value (in symbols)

}dwt_config_t ;
//__attribute__ ((packed))  dwt_config_t ;
//#pragma pack()


class DWM1000 {
public:
    static int dwm_initialize(int irq, int ss, int rst);
    static int dwm_configure(dwt_config_t *);
    static void dwm_tune(dwt_config_t *);
    static void setDataRate(byte rate);
    static void setPulseFrequency(byte freq);
    static void setPreambleLength(byte prealen);
    static void setChannel(byte channel);
    static void setPreambleCode(byte preacode);
    static void setBit(byte data[], unsigned int n, unsigned int bit, boolean val);

    static void dwm_setReceiveInterrupt();
    static void dwm_setSendInterrupt();
    static void dwm_setTransmitCallback(void(*txcallback)(void));
    static void dwm_setReceiveCallback(void(*rxcallback)(void));
    static void dwm_setSendDone(void(*senddonecallback)(void));
    static void dwm_setReceiveDone(void(*receivedonecallback)(void));
    static void dwm_setReceiveFailed(void(*receivefailedcallback)(void));
    static void dwm_setReceiveTimeout(void(*receiveTimeoutcallback)(void));

    static void	dwm_receive(bool val);
    static void	dwm_readDataReceived();
    static void dwm_returnDataReceived(byte *, uint32 *);
    static void	dwm_transmit(byte data[], unsigned int lenght, byte mode);

    static void dwm_setaddress16(uint16 shortAddress);
    static void dwm_setpanid(uint16 panID);
    static void dwm_setEUI(byte *d);
    static void dwm_getEUI(byte *eui64);

    static void dwm_readRXTimestamp(byte *);
    static void dwm_readTXTimestamp(byte *);

    static void dwm_setDelayedTRXTime(byte []);
    static void dwm_setrxtimeout(uint16 time);
    static void dwm_readSysTime(byte * timestamp);

    static void dwm_setTXAntennaDelay(uint16 txDelay);
    static void dwm_setRXAntennaDelay(uint16 rxDelay);
    static void dwm_enableframefilter(uint16 enable);

    static void	dwm_forceTRXOff(void);
    static void	dwm_rxReset(void);
    static void dwm_softreset(void);
    static int dwm_rxenable(int delayed);

    static byte _txfctrl[TX_FCTRL_LEN];
    static byte _chanctrl[CHAN_CTRL_LEN];

    static byte _pacSize;
    static byte _pulseFrequency;
    static byte _dataRate;
    static byte _preambleLength;
    static byte _preambleCode;
    static byte _channel;
    static boolean _smartPower;

private:
    static uint32 dwm_otpRead(uint32 address);
    static int dwm_loaducodefromrom(void);
    static void	dwm_setInterrupt(uint32 bitmask, byte enable);

    static void dwm_setAutoRXreEnable(int enable);

    static void dwm_setDataRate(byte rate);

    static void	idle();
    static void	dwm_reset();

    static void	clearReceiveStatus();

    static uint32 dwm_readDevID();

    static uint32 dwm_read32Bit(int regFileID, int regOffset);
    static uint16 dwm_read16bitoffsetreg(int regFileID, int regOffset);
    static int dwm_readFromDevice(uint16  recordNumber, uint16  index, uint32  length, byte *buffer);

    static int dwm_write32Bit(int regFileID, int regOffset, uint32 regval);
    static int dwm_write16bitoffsetreg(int regFileID, int regOffset, uint16 regval);
    static int dwm_writeToDevice(uint16 recordNumber, uint16 index, uint32 length, const byte *buffer);

    static void	dwm_enableClocks(int clocks);

    static void	dwm_isr(void);
    static void	dwm_syncrxbufptrs(void);


    static void dwm_startTX(byte mode);

    static void dwm_enterSleepAfterTX(int enable);

    static void dwm_disablesequencing(void);
    static void dwm_aonarrayupload(void);
    static void dwm_setsmarttxpower(int enable);

};
#endif

