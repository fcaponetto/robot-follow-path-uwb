#include "dwm_1000.h"
#include "digitalWriteFast.h"

// Defines for enable_clocks function
#define FORCE_SYS_XTI  0
#define ENABLE_ALL_SEQ 1
#define FORCE_SYS_PLL  2
#define READ_ACC_ON    7
#define READ_ACC_OFF   8
#define FORCE_OTP_ON   11
#define FORCE_OTP_OFF  12
#define FORCE_TX_PLL   13

//OTP addresses definitions
#define LDOTUNE_ADDRESS (0x04)
#define PARTID_ADDRESS (0x06)
#define LOTID_ADDRESS  (0x07)
#define VBAT_ADDRESS   (0x08)
#define VTEMP_ADDRESS  (0x09)
#define TXCFG_ADDRESS  (0x10)
#define ANTDLY_ADDRESS (0x1C)
#define XTRIM_ADDRESS  (0x1E)

const SPISettings _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);
const SPISettings _slowSPI = SPISettings(2000000L, MSBFIRST, SPI_MODE0);
const SPISettings* _settingSPI = &_fastSPI;

static dwm_local_data_t dwm1000local;
static dwt_config_t *dwm_configure;


byte DWM1000::_txfctrl[TX_FCTRL_LEN];
byte DWM1000::_chanctrl[CHAN_CTRL_LEN];


//TODO mettere DWT_PAC8 senza valori static
byte DWM1000::_pacSize = DWT_PAC8;
byte DWM1000::_pulseFrequency = DWT_PRF_16M;
//TODO cambiare con DWT_BR_110K
byte DWM1000::_dataRate = DWT_BR_6M8;
byte DWM1000::_preambleLength = DWT_PLEN_128;
byte DWM1000::_preambleCode = PREAMBLE_CODE_16MHZ_4;
byte DWM1000::_channel = CHANNEL_7;
boolean DWM1000::_smartPower = false;


//these are default antenna delays for EVB1000, these can be used if there is no calibration data in the DW1000,
//or instead of the calibration data
const uint16 rfDelays[2] = {
        (uint16) ((DWT_PRF_16M_RFDLY/ 2.0) * 1e-9 / DWT_TIME_UNITS),//PRF 16
        (uint16) ((DWT_PRF_64M_RFDLY/ 2.0) * 1e-9 / DWT_TIME_UNITS)
};

int DWM1000::dwm_initialize(int irq, int ss, int rst) {
    dwm1000local._ss = ss;
    dwm1000local._irq = irq;
    dwm1000local._rst = rst;

    pinMode(dwm1000local._ss, OUTPUT);
    pinMode(dwm1000local._irq, INPUT);
    digitalWrite(dwm1000local._irq, LOW);

    // start SPI
    SPI.begin();
    SPI.usingInterrupt(digitalPinToInterrupt(dwm1000local._irq));
    // pin and basic member setup

    // attach interrupt
    attachInterrupt(digitalPinToInterrupt(dwm1000local._irq),DWM1000::dwm_isr, RISING);

    digitalWrite(dwm1000local._ss, HIGH);

    if (dwm1000local._rst > 0) {
        pinMode(dwm1000local._rst, OUTPUT);
        digitalWrite(dwm1000local._rst, HIGH);
    }

    dwm_reset();
    dwm_softreset();

    dwm1000local.dblbuffon = 0; //double mode off by default
    dwm1000local.statescount = 0;
    dwm1000local.prfIndex = 0; //16M
    dwm1000local.ldoTune = 0;
    dwm1000local.wait4resp = 0;
    byte plllockdetect = EC_CTRL_PLLLCK;
    uint16 otp_addr = 0;

    dwm1000local._dwm_txcallback = NULL;
    dwm1000local._dwm_rxcallback = NULL;

    dwm1000local.deviceID = dwm_readDevID();

    /* read and validate device ID return -1 if not recognized */
    if (DWT_DEVICE_ID != dwm1000local.deviceID)			// MP IC ONLY (i.e. DW1000) FOR THIS CODE
        return DWT_ERROR;

    dwm_enableClocks(FORCE_SYS_XTI); 					//NOTE: set system clock to XTI - this is necessary to make sure the values read by _dwt_otpread are reliable
    //dwm_writeToDevice(EXT_SYNC_ID, EC_CTRL_OFFSET, 1, &plllockdetect); //

    //read OTP revision number
    otp_addr = dwm_otpRead(XTRIM_ADDRESS) & 0xffff;		// Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
    dwm1000local.otprev = (otp_addr >> 8) & 0xff;		// OTP revision is next byte


    dwm1000local.rfrxDly = rfDelays[0];
    dwm1000local.rftxDly = rfDelays[0];

    dwm_setTXAntennaDelay(dwm1000local.rfrxDly);
    dwm_setTXAntennaDelay(dwm1000local.rftxDly);

    delay(1);

    dwm_loaducodefromrom();

    delay(1);

    dwm_enableClocks(ENABLE_ALL_SEQ);						//enable clocks for sequencing

    // disable double buffering, enable polarity hight
    uint32 _syscfg = 0;
    _syscfg = 0x00001200;
    dwm_write32Bit(SYS_CFG_ID, 0x00, _syscfg);

    return DWT_SUCCESS;
    /* disable all interrupts */
    //byte _sysmask[4];
    //memset(_sysmask, 0, 4);
    //_dwm_writeToDevice(SYS_MASK_ID, 0x00, 4, _sysmask);
} // end dwm_initialize()

void DWM1000::dwm_reset() {
    if (dwm1000local._rst < 0) {
        dwm_softreset();
    }
    else {
        digitalWrite(dwm1000local._rst, LOW);
        delay(10);
        digitalWrite(dwm1000local._rst, HIGH);
        delay(10);
        // force into idle mode (although it should be already after reset)
        idle();
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn dwt_softreset()
*
*  @brief this function resets the DW1000
*
* input parameters:
*
* output parameters
*
* no return value
*/
void DWM1000::dwm_softreset(void)
{
    byte temp[1] = { 0 };

    dwm_disablesequencing();
    dwm_enableClocks(FORCE_SYS_XTI); //set system clock to XTI

    //clear any AON auto download bits (as reset will trigger AON download)
    dwm_write16bitoffsetreg(AON_ID, AON_WCFG_OFFSET, 0x0);
    //clear the wakeup configuration
    dwm_writeToDevice(AON_ID, AON_CFG0_OFFSET, 1, temp);
    //upload the new configuration
    dwm_aonarrayupload();

    //reset HIF, TX, RX and PMSC
    dwm_readFromDevice(PMSC_ID, 0x3, 1, temp);

    temp[0] &= 0x0F;
    dwm_writeToDevice(PMSC_ID, 0x3, 1, &temp[0]);

    //DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
    delay(1);
    //Can also poll the PLL lock flag, but then the SPI needs to be < 3MHz !!

    temp[0] |= 0xF0;
    dwm_writeToDevice(PMSC_ID, 0x3, 1, &temp[0]);

    dwm1000local.wait4resp = 0;
}

// upload always on array configuration and go to sleep
void DWM1000::dwm_aonarrayupload(void)
{
    byte buf[1];

    // Upload array
    buf[0] = 0x00;
    dwm_writeToDevice(AON_ID,AON_CTRL_OFFSET,1,buf);
    buf[0] = 0x02;
    dwm_writeToDevice(AON_ID,AON_CTRL_OFFSET,1,buf);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_disablesequencing()
 *
 * @brief  This function disables the TX blocks sequencing, it disables PMSC control of RF blocks, system clock is also set to XTAL
 *
 * input parameters none
 *
 * output parameters none
 *
 * no return value
 */
void DWM1000::dwm_disablesequencing(void) //disable sequencing and go to state "INIT"
{
    dwm_enableClocks(FORCE_SYS_XTI); //set system clock to XTI

    dwm_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE); //disable PMSC ctrl of RF and RX clk blocks
}

void DWM1000::idle() {
    uint32 _sysctrl = 0;
    _sysctrl =dwm_read32Bit(SYS_CTRL_ID, 0x00);
    _sysctrl |= 0x00000040;
    dwm_write32Bit(SYS_CTRL_ID, 0x00, _sysctrl);
}

void DWM1000::dwm_enableClocks(int clocks)
{
    byte reg[2];

    dwm_readFromDevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, reg);
    switch (clocks)
    {
        case ENABLE_ALL_SEQ:
        {
            _settingSPI = &_fastSPI;
            reg[0] = 0x00;
            reg[1] = reg[1] & 0xfe;
        }
            break;
        case FORCE_SYS_XTI:
        {
            //system and rx
            _settingSPI = &_slowSPI;
            reg[0] = 0x01 | (reg[0] & 0xfc);
        }
            break;
        case FORCE_SYS_PLL:
        {
            //Serial.println("PLL");
            //system
            _settingSPI = &_fastSPI;
            reg[0] = 0x02 | (reg[0] & 0xfc);
        }
            break;
        default:
            break;
    }


    // Need to write lower byte separately before setting the higher byte(s)
    dwm_writeToDevice(PMSC_ID, PMSC_CTRL0_OFFSET, 1, &reg[0]);
    dwm_writeToDevice(PMSC_ID, 0x1, 1, &reg[1]);

} // end _dwt_enableclocks()

uint32 DWM1000::dwm_readDevID(void)
{
    return dwm_read32Bit(DEV_ID_ID, 0);
}

uint32 DWM1000::dwm_read32Bit(int regFileID, int regOffset)
{
    uint32 regval = DWT_ERROR;
    int j;
    byte buffer[4];

    int result = dwm_readFromDevice(regFileID, regOffset, 4, buffer); // read 4 bytes (32-bits) register into buffer

    if (result == DWT_SUCCESS)
    {
        for (j = 3; j >= 0; j--)
        {
            regval = (regval << 8) + buffer[j];        // sum
        }
    }
    return regval;

} // end dwm_read32bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
* @fn dwt_read16bitoffsetreg()
*
* @brief  this function is used to read 16-bit value from the DW1000 device registers
*
* input parameters:
* @param regFileID - ID of register file or buffer being accessed
* @param regOffset - the index into register file or buffer being accessed
*
* output parameters
*
* returns 16 bit register value (success), or DWT_DECA_ERROR for error
*/
uint16 DWM1000::dwm_read16bitoffsetreg(int regFileID, int regOffset)
{
    uint16  regval = DWT_ERROR;
    byte   buffer[2];

    int result = dwm_readFromDevice(regFileID, regOffset, 2, buffer); // read 2 bytes (16-bits) register into buffer

    if (result == DWT_SUCCESS)
    {
        regval = (buffer[1] << 8) + buffer[0];        // sum
    }
    return regval;

} // end dwt_read16bitoffsetreg()

int DWM1000::dwm_write32Bit(int regFileID, int regOffset, uint32 regval)
{
    int	j;
    int	reg;
    byte buffer[4];

    for (j = 0; j < 4; j++)
    {
        buffer[j] = regval & 0xff;
        regval >>= 8;
    }

    reg = dwm_writeToDevice(regFileID, regOffset, 4, buffer);

    return reg;

} // end dwt_write32bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
* @fn dwt_write16bitoffsetreg()
*
* @brief  this function is used to write 16-bit value to the DW1000 device registers
*
* input parameters:
* @param regFileID - ID of register file or buffer being accessed
* @param regOffset - the index into register file or buffer being accessed
* @param regval    - the value to write
*
* output parameters
*
* returns DWT_DECA_SUCCESS for success, or DWT_DECA_ERROR for error
*/
int DWM1000::dwm_write16bitoffsetreg(int regFileID, int regOffset, uint16 regval)
{
    int reg;
    byte   buffer[2];

    buffer[0] = regval & 0xFF;
    buffer[1] = regval >> 8;

    reg = dwm_writeToDevice(regFileID, regOffset, 2, buffer);

    return reg;

} // end dwt_write16bitoffsetreg()

int DWM1000::dwm_writeToDevice(uint16 recordNumber, uint16 index, uint32 length,const byte *buffer)
{
    byte header[3];										// buffer to compose header in
    int	cnt = 0;										// counter for length of header

    /* Write message header selecting WRITE operation and addresses as appropriate (this is one to three bytes long) */
    if (index == 0)										// for index of 0, no sub-index is required
    {
        header[cnt++] = 0x80 | recordNumber;            // bit-7 is WRITE operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
    }
    else
    {
        header[cnt++] = 0xC0 | recordNumber;            // bit-7 is WRITE operation, bit-6 one=sub-address follows, bits 5-0 is reg file id

        if (index <= 127)                               // for non-zero index < 127, just a single sub-index byte is required
        {
            header[cnt++] = (byte)index;                // bit-7 zero means no extension, bits 6-0 is index.
        }
        else
        {
            header[cnt++] = 0x80 | (byte)(index);       // bit-7 one means extended index, bits 6-0 is low seven bits of index.
            header[cnt++] = (byte)(index >> 7);         // 8-bit value = high eight bits of index.
        }
    }

    /* write it to the SPI */
    SPI.beginTransaction(*_settingSPI);
    digitalWriteFast(dwm1000local._ss, LOW);
    for (int i = 0; i < cnt; i++) {
        SPI.transfer(header[i]);
    }
    for (int i = 0; i < length; i++) {
        SPI.transfer(buffer[i]);
    }
    delayMicroseconds(5);
    digitalWriteFast(dwm1000local._ss, HIGH);
    SPI.endTransaction();


} // end dwm_writetodevice()

int DWM1000::dwm_readFromDevice(uint16  recordNumber, uint16  index, uint32  length, byte *buffer)
{
    byte	header[3];                                     // buffer to compose header in
    int		cnt = 0;                                      // counter for length of header

    /* Write message header selecting READ operation and addresses as appropriate (this is one to three bytes long) */
    if (index == 0)                                     // for index of 0, no sub-index is required
    {
        header[cnt++] = (byte)recordNumber;				// bit-7 zero is READ operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
    }
    else
    {
        header[cnt++] = (byte)(0x40 | recordNumber);    // bit-7 zero is READ operation, bit-6 one=sub-address follows, bits 5-0 is reg file id

        if (index <= 127)                               // for non-zero index < 127, just a single sub-index byte is required
        {
            header[cnt++] = (byte)index;                // bit-7 zero means no extension, bits 6-0 is index.
        }
        else
        {
            header[cnt++] = 0x80 | (byte)(index);       // bit-7 one means extended index, bits 6-0 is low seven bits of index.
            header[cnt++] = (byte)(index >> 7);         // 8-bit value = high eight bits of index.
        }
    }

    /* do the read from the SPI */
    SPI.beginTransaction(*_settingSPI);
    digitalWriteFast(dwm1000local._ss, LOW);
    for (int i = 0; i < cnt; i++) {
        SPI.transfer(header[i]);
    }
    for (int i = 0; i < length; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    delayMicroseconds(5);
    digitalWriteFast(dwm1000local._ss, HIGH);
    SPI.endTransaction();

    return DWT_SUCCESS;
} // end dwm_readfromdevice()

void DWM1000::dwm_receive(bool val) {
    idle();
    clearReceiveStatus();

    dwm_setAutoRXreEnable(val);

    //start receive
    uint32 _sysctrl = 0;
    _sysctrl = 0x00000100;
    dwm_write32Bit(SYS_CTRL_ID, 0x00, _sysctrl);

    delay(5);
}

void DWM1000::clearReceiveStatus() {
    byte _sysstatus[5];
    memset(_sysstatus, 0, 5);
    _sysstatus[0] = 0x00;
    _sysstatus[1] = 0xF4;
    _sysstatus[2] = 0x05;
    dwm_writeToDevice(SYS_STATUS_ID, 0x00, 5, _sysstatus);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn dwt_setautorxreenable()
*
*  @brief This call enables the auto rx re-enable feature
*
* input parameters
* @param enable - 1 to enable, 0 to disable the feature
*
* output parameters
*
* no return value
*/
void DWM1000::dwm_setAutoRXreEnable(int enable)
{
    byte byte = 0;

    if (enable)
    {
        //enable auto re-enable of the receiver
        dwm1000local.sysCFGreg |= SYS_CFG_RXAUTR;
    }
    else
    {
        //disable auto re-enable of the receiver
        dwm1000local.sysCFGreg &= ~SYS_CFG_RXAUTR;
    }

    byte = dwm1000local.sysCFGreg >> 24;

    dwm_writeToDevice(SYS_CFG_ID, 3, 1, &byte);
}

void DWM1000::dwm_readDataReceived() {
    // 10 bits of RX frame control register
    byte rxFrameInfo[4];
    dwm_readFromDevice(RX_FINFO_ID, 0x00, 4, rxFrameInfo);

    rxFrameInfo[0] &= 0x7F;
    unsigned int length = rxFrameInfo[0];

    if (length > 2) {
        length -= 2;
    }
    //Serial.print("Len received: "); Serial.println(length);

    if (length <= 0) {
        return;
    }
    byte buffer[length];
    dwm_readFromDevice(RX_BUFFER_ID, 0x00, length, buffer);

    //Serial.print("Received: ");
    for (int i = 0; i < length; i++) {
        //Serial.print(buffer[i], HEX); Serial.print(" ");
    }
    //Serial.println("");

    delay(5);
}

void DWM1000::dwm_returnDataReceived(byte * buffer, uint32 *length) {
    // 10 bits of RX frame control register
    byte rxFrameInfo[4];
    dwm_readFromDevice(RX_FINFO_ID, 0x00, 4, rxFrameInfo);

    rxFrameInfo[0] &= 0x7F;
    *length = rxFrameInfo[0];

    if (*length > 2) {
        *length -= 2;
    }

    dwm_readFromDevice(RX_BUFFER_ID, 0x00, *length, buffer);
}

void DWM1000::dwm_transmit(byte buffer[], unsigned int length, byte mode) {
    idle();

    // clear transmit status
    byte _sysstatus[5];
    memset(_sysstatus, 0, 5);
    _sysstatus[0] = 0xF0;
    dwm_writeToDevice(SYS_STATUS_ID, 0x00, 5, _sysstatus);

    dwm_writeToDevice(TX_BUFFER_ID, 0x00, length, buffer);

    byte _txfctrl[5];
    length += 2;
    _txfctrl[0] = (byte)(length & 0xFF);
    dwm_writeToDevice(TX_FCTRL_ID, 0x00, 1, _txfctrl);

    dwm_startTX(mode);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readsystime()
 *
 *  @brief This is used to read the system time
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read system time
 *
 * output parameters
 * @param timestamp - the timestamp buffer will contain the value after the function call
 *
 * returns
 */
void DWM1000::dwm_readSysTime(byte * timestamp)
{
    dwm_readFromDevice(SYS_TIME_ID, 0, SYS_TIME_LEN, timestamp) ; //get the adjusted time of arrival

}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setdelayedtrxtime()
 *
 * @brief  This API function configures the delayed transmit time or the delayed rx on time
 *
 * input parameters
 * @param starttime - the tx/rx start time (the 32 bits should be the high 32 bits of the system time at which to send the message,
 * or at which to turn on the receiver)
 *
 * output parameters none
 *
 * no return value
 */
void DWM1000::dwm_setDelayedTRXTime(byte starttime[])
{
    dwm_writeToDevice(DX_TIME_ID, 0, 5, starttime) ;

    // memset(time, 0x00, 5);
    // dwm_readFromDevice(DX_TIME_ID, 0x00, 5, time);
    // Serial.println("Time Delayed:\t");
    // for(i = 0; i < 5; i++){
    // Serial.print(time[i], HEX); Serial.print("\t");
    // }
    // Serial.println("");

} // end dwt_setdelayedtrxtime()

/*! ------------------------------------------------------------------------------------------------------------------
* @fn dwt_starttx()
*
*  @brief This call initiates the transmission, input parameter indicates which TX mode is used see below
*
* input parameters:
* @param mode - if 0 immediate TX (no response expected)
*               if 1 delayed TX (no response expected)
*               if 2 immediate TX (response expected - so the receiver will be automatically turned on after TX is done)
*               if 3 delayed TX (response expected - so the receiver will be automatically turned on after TX is done)
*
* output parameters
*
* returns DWT_DECA_SUCCESS for success, or DWT_DECA_ERROR for error (e.g. a delayed transmission will fail if the delayed time has passed)
*/
void DWM1000::dwm_startTX(byte mode)
{
    byte temp = 0x00;
    uint16 checkTxOK = 0;

    if (mode & DWT_RESPONSE_EXPECTED)
    {
        temp = (byte)SYS_CTRL_WAIT4RESP; //set wait4response bit
        //dwm_writeToDevice(SYS_CTRL_ID, 0, 1, &temp);
        dwm1000local.wait4resp = 1;
    }

    if (mode & DWT_START_TX_DELAYED)
    {
        uint32 status ;

        //both SYS_CTRL_TXSTRT and SYS_CTRL_TXDLYS to correctly enable TX
        temp |= (byte)(SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT) ;
        dwm_writeToDevice(SYS_CTRL_ID,0,1,&temp) ;
        checkTxOK = dwm_read16bitoffsetreg(SYS_STATUS_ID,3) ;
        status = dwm_read32Bit(SYS_STATUS_ID, 0) ;          // read status register
        if ((checkTxOK & SYS_STATUS_TXERR) == 0)	 // Transmit Delayed Send set over Half a Period away or Power Up error (there is enough time to send but not to power up individual blocks).
        {
            //Serial.println("tx delayed");
            //retval = DWT_SUCCESS ;                                          // All okay
        }
        else
        {
            //I am taking DSHP set to Indicate that the TXDLYS was set too late for the specified DX_TIME.
            //Remedial Action - (a) cancel delayed send
            temp = (byte)SYS_CTRL_TRXOFF;                                  // this assumes the bit is in the lowest byte
            dwm_writeToDevice(SYS_CTRL_ID,0,1,&temp) ;
            //note event Delayed TX Time too Late
            //could fall through to start a normal send (below) just sending late.....
            //... instead return and assume return value of 1 will be used to detect and recover from the issue.

            //clear the "auto TX to sleep" bit
            dwm_enterSleepAfterTX(0);
            dwm1000local.wait4resp = 0;
            //TODO return error
            //Serial.println("\tERROR TX DELAYED");// Failed !
        }
    }


    temp |= (byte)SYS_CTRL_TXSTRT;
    dwm_writeToDevice(SYS_CTRL_ID, 0, 1, &temp);

} // end dwt_starttx()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn  dwt_setrxtimeout()
 *
 * @brief This call enables RX timeout (SY_STAT_RFTO event)
 *
 * input parameters
 * @param time - how long the receiver remains on from the RX enable command
 *               The time parameter used here is in 1.0256 us (512/499.2MHz) units
 *               If set to 0 the timeout is disabled.
 *
 * output parameters
 *
 * no return value
 */
void DWM1000::dwm_setrxtimeout(uint16 time)
{
    byte temp ;

    dwm_readFromDevice(SYS_CFG_ID,3,1,&temp) ;           // read register

    if(time > 0)
    {
        dwm_write16bitoffsetreg(RX_FWTO_ID, 0x0, time) ;

        temp |= (byte)(SYS_CFG_RXWTOE>>24);
        // OR in 32bit value (1 bit set), I know this is in high byte.
        dwm1000local.sysCFGreg |= SYS_CFG_RXWTOE;

        dwm_writeToDevice(SYS_CFG_ID,3,1,&temp) ;
    }
    else
    {
        temp &= ~((byte)(SYS_CFG_RXWTOE>>24));
        // AND in inverted 32bit value (1 bit clear), I know this is in high byte.
        dwm1000local.sysCFGreg &= ~(SYS_CFG_RXWTOE);

        dwm_writeToDevice(SYS_CFG_ID,3,1,&temp) ;

        //dwt_write16bitoffsetreg(RX_FWTO_ID,0,0) ;          // clearing the time is not needed
    }

} // end dwt_setrxtimeout()

void DWM1000::dwm_setSendInterrupt() {
    //Send done
    //Send failed
    dwm_setInterrupt(DWT_INT_TFRS, true);
}

void DWM1000::dwm_setReceiveInterrupt() {
    //Receive done
    //Receive failed
    //dwm_setInterrupt(0x0000D000, true);
    dwm_setInterrupt(DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_SFDT, true);
}

/*! ------------------------------------------------------------------------------------------------------------------
  * @fn void dwt_setinterrupt( uint32 bitmask, uint8 enable)
  *
  * @brief This function enables the specified events to trigger an interrupt.
  * The following events can be enabled:
  * DWT_INT_TFRS         0x00000080          // frame sent
  * DWT_INT_RFCG         0x00004000          // frame received with good CRC
  * DWT_INT_RPHE         0x00001000          // receiver PHY header error
  * DWT_INT_RFCE         0x00008000          // receiver CRC error
  * DWT_INT_RFSL         0x00010000          // receiver sync loss error
  * DWT_INT_RFTO         0x00020000          // frame wait timeout
  * DWT_INT_RXPTO        0x00200000          // preamble detect timeout
  * DWT_INT_SFDT         0x04000000          // SFD timeout
  * DWT_INT_ARFE         0x20000000          // frame rejected (due to frame filtering configuration)
  *
  *
  * input parameters:
  * @param bitmask - sets the events which will generate interrupt
  * @param enable - if set the interrupts are enabled else they are cleared
  *
  * output parameters
  *
  * no return value
  */
void DWM1000::dwm_setInterrupt(uint32 bitmask, byte enable)
{
    uint32 mask;

    // need to beware of interrupts occurring in the middle of following read modify write cycle
    cli();

    mask = dwm_read32Bit(SYS_MASK_ID, 0);           // read register

    if (enable)
    {
        mask |= bitmask;
    }
    else
    {
        mask &= ~bitmask;  // clear the bit
    }
    dwm_write32Bit(SYS_MASK_ID, 0, mask);            // new value

    sei();
}

void DWM1000::dwm_setTransmitCallback(void(*txcallback)(void))
{
    dwm1000local._dwm_txcallback = txcallback;
}

void DWM1000::dwm_setSendDone(void(*senddonecallback)(void)) {
    dwm1000local._dwm_senddonecallback = senddonecallback;
}

void DWM1000::dwm_setReceiveDone(void(*receivedonecallback)(void)) {
    dwm1000local._dwm_receivedonecallback = receivedonecallback;
}

void DWM1000::dwm_setReceiveFailed(void(*receivefailedcallback)(void)) {
    dwm1000local._dwm_receivefailedcallback = receivefailedcallback;
}

void DWM1000::dwm_setReceiveCallback(void(*rxcallback)(void))
{
    dwm1000local._dwm_rxcallback = rxcallback;
}

void DWM1000::dwm_setReceiveTimeout(void(*receiveTimeoutcallback)(void)){
    dwm1000local._dwm_receiveTimeoutcallback = receiveTimeoutcallback;
}

void DWM1000::dwm_isr(void) {
    uint32  status = 0;
    uint32  clear = 0; // will clear any events seen

    status = dwm_read32Bit(SYS_STATUS_ID, 0);            // read status register low 32bits

    //fix for bug 622 - LDE done flag gets latched on a bad frame
    if ((status & SYS_STATUS_LDEDONE) && (dwm1000local.dblbuffon == 0))
    {
        if ((status & (SYS_STATUS_LDEDONE | SYS_STATUS_RXPHD | SYS_STATUS_RXSFDD)) != (SYS_STATUS_LDEDONE | SYS_STATUS_RXPHD | SYS_STATUS_RXSFDD))
        {
            //got LDE done but other flags SFD and PHR are clear - this is a bad frame - reset the transceiver
            dwm_forceTRXOff(); //this will clear all events

            dwm_rxReset();
            //leave any TX events for processing (e.g. if we TX a frame, and then enable rx,
            //we can get into here before the TX frame done has been processed, when we are polling (i.e. slow to process the TX)
            status &= CLEAR_ALLTX_EVENTS;
            //re-enable the receiver - if auto rx re-enable set
            if (dwm1000local.sysCFGreg & SYS_CFG_RXAUTR)
            {
                dwm_write16bitoffsetreg(SYS_CTRL_ID, 0, (uint16)SYS_CTRL_RXENAB);
            }
            else
            {
                if (dwm1000local._dwm_receivefailedcallback != NULL)
                    dwm1000local._dwm_receivefailedcallback;
            }
        }
    }

    //
    // 1st check for RX frame received or RX timeout and if so call the rx callback function
    //
    if (status & SYS_STATUS_RXFCG) // Receiver FCS Good
    {
        if (status & SYS_STATUS_LDEDONE)  // LDE done/finished
        {
            // bug 634 - overrun overwrites the frame info data... so both frames should be discarded
            // read frame info and other registers and check for overflow again
            // if overflow set then discard both frames...

            if (status & SYS_STATUS_RXOVRR) //NOTE when overrun both HS and RS pointers point to the same buffer
            {
                //when the overrun happens the frame info data of the buffer A (which contains the older frame e.g. seq. num = x)
                //will be corrupted with the latest frame (seq. num = x + 2) data, both the host and IC are pointing to buffer A
                //we are going to discard this frame - turn off transceiver and reset receiver
                dwm_forceTRXOff();

                dwm_rxReset();

                if (dwm1000local.sysCFGreg & SYS_CFG_RXAUTR) //re-enable of RX is ON, then re-enable here (ignore error)
                {
                    dwm_write16bitoffsetreg(SYS_CTRL_ID, 0, (uint16)SYS_CTRL_RXENAB);
                }
                else //the RX will be re-enabled by the application, report an error
                {
                    if (dwm1000local._dwm_receivedonecallback != NULL)
                    {
                        (*(dwm1000local._dwm_receivedonecallback))();
                    }
                }

                return;
            }
            else //no overrun condition - proceed to process the frame
            if (dwm1000local.dblbuffon == 0) //if no double buffering
            {
                //clear all receive status bits (as we are finished with this receive event)
                clear |= status & CLEAR_ALLRXGOOD_EVENTS;
                dwm_write32Bit(SYS_STATUS_ID, 0, clear);         // write status register to clear event bits we have seen

                //NOTE: clear the event which caused interrupt means once the rx is enabled or tx is started
                //new events can trigger and give rise to new interrupts
                //call the RX call-back function to process the RX event
                if (dwm1000local._dwm_receivedonecallback != NULL)
                {
                    (*(dwm1000local._dwm_receivedonecallback))();
                }
                return;
            }

            //}//end of no overrun

        } //if LDE_DONE is set (this means we have both SYS_STATUS_RXFCG and SYS_STATUS_LDEDONE)
        else //no LDE_DONE ?
        {
            //printf("NO LDE done or LDE error\n");
            if (!(dwm1000local.sysCFGreg & SYS_CFG_RXAUTR))
            {
                dwm_forceTRXOff();
            }
            dwm_rxReset();	//reset the RX
            dwm1000local.wait4resp = 0;
            //call the RX call-back function to process the RX event
            if (dwm1000local._dwm_receivedonecallback != NULL)
            {
                (*(dwm1000local._dwm_receivefailedcallback))();
            }
        }
        return;
    } // end if CRC is good
    else
    {
        //
        // Check for TX frame sent event and signal to upper layer.
        //
        if (status & SYS_STATUS_TXFRS)  // Transmit Frame Sent
        {
            //Serial.println("Transmit done");

            clear |= CLEAR_ALLTX_EVENTS; //clear TX event bits
            dwm_write32Bit(SYS_STATUS_ID, 0, clear);         // write status register to clear event bits we have seen

            //NOTE: clear the event which caused interrupt means once the rx is enabled or tx is started
            //new events can trigger and give rise to new interrupts
            //call the TX call-back function to process the TX event

            //if (dwm1000local.wait4resp) //wait4response was set with the last TX start command
            //{
            //	//if using wait4response and the ACK has been sent as the response requested it
            //	//the receiver will be re-enabled, so issue a TRXOFF command to disable and prevent any
            //	//unexpected interrupts
            //	dwm_forceTRXOff();
            //}


            if (dwm1000local._dwm_senddonecallback != NULL){
                (*(dwm1000local._dwm_senddonecallback))();
            }

        }
        else if (status & SYS_STATUS_RXRFTO)                 // Receiver Frame Wait timeout:
        {
            clear |= status & SYS_STATUS_RXRFTO;
            dwm_write32Bit(SYS_STATUS_ID, 0, clear);         // write status register to clear event bits we have seen

            if (dwm1000local._dwm_receiveTimeoutcallback != NULL)
            {
                (*(dwm1000local._dwm_receiveTimeoutcallback))();
            }
            dwm1000local.wait4resp = 0;
        }
        else if (status & CLEAR_ALLRXERROR_EVENTS)//catches all other error events
        {
            //Serial.print(status , BIN);Serial.println("");
            clear |= status & CLEAR_ALLRXERROR_EVENTS;
            dwm_write32Bit(SYS_STATUS_ID, 0, clear);         // write status register to clear event bits we have seen

            dwm1000local.wait4resp = 0;
            //NOTE: clear the event which caused interrupt means once the rx is enabled or tx is started
            //new events can trigger and give rise to new interrupts


            //fix for bug 622 - LDE done flag gets latched on a bad frame / reset receiver
            if (!(dwm1000local.sysCFGreg & SYS_CFG_RXAUTR))
            {
                dwm_forceTRXOff(); //this will clear all events
            }
            dwm_rxReset();	//reset the RX

            //end of fix for bug 622 - LDE done flag gets latched on a bad frame

            if (dwm1000local._dwm_receivefailedcallback != NULL)
            {
                (*(dwm1000local._dwm_receivefailedcallback))();
            }
            status &= CLEAR_ALLTX_EVENTS;
        }
    }
}

void DWM1000::dwm_forceTRXOff(void)
{
    byte temp;
    uint32 mask;

    temp = (byte)SYS_CTRL_TRXOFF;                       // this assumes the bit is in the lowest byte

    mask = dwm_read32Bit(SYS_MASK_ID, 0);  //read set interrupt mask

    // need to beware of interrupts occurring in the middle of following read modify write cycle
    // we can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
    // event has just happened before the radio was disabled)
    // thus we need to disable interrupt during this operation
    cli();

    dwm_write32Bit(SYS_MASK_ID, 0, 0); //clear interrupt mask - so we don't get any unwanted events

    dwm_writeToDevice(SYS_CTRL_ID, 0, 1, &temp); //disable the radio

    // Forcing Transceiver off - so we do not want to see any new events that may have happened
    dwm_write32Bit(SYS_STATUS_ID, 0, (CLEAR_ALLTX_EVENTS | CLEAR_ALLRXERROR_EVENTS | CLEAR_ALLRXGOOD_EVENTS));

    dwm_syncrxbufptrs();

    dwm_write32Bit(SYS_MASK_ID, 0, mask); //set interrupt mask to what it was

    //enable/restore interrupts again...
    sei();
    dwm1000local.wait4resp = 0;
} // end deviceforcetrxoff()

void DWM1000::dwm_syncrxbufptrs(void)
{
    byte  buff;
    //need to make sure that the host/IC buffer pointers are aligned before starting RX
    (SYS_STATUS_ID, 3, 1, &buff);

    if ((buff & (SYS_STATUS_ICRBP >> 24)) !=              /* IC side Receive Buffer Pointer */
        ((buff & (SYS_STATUS_HSRBP >> 24)) << 1))   /* Host Side Receive Buffer Pointer */
    {
        byte hsrb = 0x01;
        dwm_writeToDevice(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1, &hsrb);       // we need to swap rx buffer status reg (write one to toggle internally)
    }
}

void DWM1000::dwm_rxReset(void)
{
    byte resetrx = 0xe0;
    //set rx reset
    dwm_writeToDevice(PMSC_ID, 0x3, 1, &resetrx);

    resetrx = 0xf0; //clear RX reset
    dwm_writeToDevice(PMSC_ID, 0x3, 1, &resetrx);
}

//
// _dwt_otpread - function to read the OTP memory. Ensure that MR,MRa,MRb are reset to 0.
//
uint32 DWM1000::dwm_otpRead(uint32 address)
{
    byte buf[4];
    uint32 ret_data;

    buf[1] = (address >> 8) & 0xff;
    buf[0] = address & 0xff;

    // Write the address
    dwm_writeToDevice(OTP_IF_ID, OTP_ADDR, 2, buf);

    // Assert OTP Read (self clearing)
    buf[0] = 0x03; // 0x03 for manual drive of OTP_READ
    dwm_writeToDevice(OTP_IF_ID, OTP_CTRL, 1, buf);
    buf[0] = 0x00; // Bit0 is not autoclearing, so clear it (Bit 1 is but we clear it anyway).
    dwm_writeToDevice(OTP_IF_ID, OTP_CTRL, 1, buf);

    // Read read data, available 40ns after rising edge of OTP_READ
    //ret_data = dwm_read32bitoffsetreg(OTP_IF_ID, OTP_RDAT);

    // Return the 32bit of read data
    return (ret_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn _dwt_loaducodefromrom()
*
* @brief  load ucode from OTP MEMORY or ROM
*
* input parameters
*
* output parameters
*
* no return value
*/
int DWM1000::dwm_loaducodefromrom(void)
{
    byte wr_buf[2];

    //set up clocks
    wr_buf[1] = 0x03;
    wr_buf[0] = 0x01;
    dwm_writeToDevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, wr_buf);
    //kick off the LDE load
    dwm_write16bitoffsetreg(OTP_IF_ID, OTP_CTRL, OTP_CTRL_LDELOAD); // set load LDE kick bit

    delay(1); // Allow time for code to upload (should take up to 120 us)

    wr_buf[1] = 0x02;
    wr_buf[0] = 0x00;
    dwm_writeToDevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, wr_buf);

    //default clocks (ENABLE_ALL_SEQ)
    //dwm_enableClocks(ENABLE_ALL_SEQ); //enable clocks for sequencing


    return DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn dwt_setpanid()
*
*  @brief This is used to set the PAN ID
*
*
* input parameters
* @param panID - this is the PAN ID
*
* output parameters
*
* no return value
*/
void DWM1000::dwm_setpanid(uint16 panID)
{
    // PAN ID is high 16 bits of register
    dwm_write16bitoffsetreg(PANADR_ID, 2, panID);     // set the value
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn dwt_setaddress16()
*
*  @brief This is used to set 16-bit (short) address
*
*
* input parameters
* @param shortAddress - this sets the 16 bit short address
*
* output parameters
*
* no return value
*/
void DWM1000::dwm_setaddress16(uint16 shortAddress)
{
    // short address into low 16 bits
    dwm_write16bitoffsetreg(PANADR_ID, 0, shortAddress);     // set the value
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn dwt_seteui()
*
*  @brief This is used to set the EUI 64-bit (long) address
*
* input parameters
* @param eui64 - this is the pointer to a buffer that contains the 64bit address
*
* output parameters
*
* no return value
*/
void DWM1000::dwm_setEUI(byte *eui64)
{
    dwm_writeToDevice(EUI_64_ID, 0x0, 8, eui64);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_geteui()
 *
 *  @brief This is used to get the EUI 64-bit from the DW1000
 *
 * input parameters
 * @param eui64 - this is the pointer to a buffer that will contain the read 64-bit EUI value
 *
 * output parameters
 *
 * no return value
 */
void DWM1000::dwm_getEUI(byte *eui64)
{
    dwm_readFromDevice(EUI_64_ID, 0x0, 8, eui64);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn dwt_readtxtimestamp()
*
*  @brief This is used to read the TX timestamp (adjusted with the programmed antenna delay)
*
* input parameters
* @param timestamp - a pointer to a 5-byte buffer which will store the read TX timestamp time
*
* output parameters - the timestamp buffer will contain the value after the function call
*
* no return value
*/
void DWM1000::dwm_readTXTimestamp(byte timestamp[])
{
    dwm_readFromDevice(TX_TIME_ID, 0, TX_TIME_TX_STAMP_LEN, timestamp); // read bytes directly into buffer

}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn dwt_readrxtimestamp()
*
*  @brief This is used to read the RX timestamp (adjusted time of arrival)
*
* input parameters
* @param timestamp - a pointer to a 5-byte buffer which will store the read RX timestamp time
*
* output parameters - the timestamp buffer will contain the value after the function call
*
* no return value
*/
void DWM1000::dwm_readRXTimestamp(byte timestamp[])
{
    dwm_readFromDevice(RX_TIME_ID, 0, RX_TIME_RX_STAMP_LEN, timestamp); //get the adjusted time of arrival
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_entersleepaftertx(int enable)
 *
 *  @brief sets the auto TX to sleep bit. This means that after a frame
 *  transmission the device will enter deep sleep mode. The dwt_setdeepsleep() function
 *  needs to be called before this to configure the on-wake settings
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events)
 *
 * input parameters
 * @param enable - 1 to configure the device to enter deep sleep after TX, 0 - disables the configuration
 *
 * output parameters
 *
 * no return value
 */
void DWM1000::dwm_enterSleepAfterTX(int enable)
{
    uint32 reg = dwm_read32Bit(PMSC_ID, PMSC_CTRL1_OFFSET);
    //set the auto TX -> sleep bit
    if(enable)
    {
        reg |= PMSC_CTRL1_ATXSLP;
    }
    else
    {
        reg &= ~(PMSC_CTRL1_ATXSLP);
    }
    dwm_write32Bit(PMSC_ID, PMSC_CTRL1_OFFSET, reg);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxantennadelay()
 *
 * @brief This API function writes the antenna delay (in time units) to RX registers
 *
 * input parameters:
 * @param rxDelay - this is the total (RX) antenna delay value, which
 *                          will be programmed into the RX register
 *
 * output parameters
 *
 * no return value
 */
void DWM1000::dwm_setRXAntennaDelay(uint16 rxDelay)
{
    // -------------------------------------------------------------------------------------------------------------------
    // set the antenna delay
    dwm1000local.rfrxDly = rxDelay;
    dwm_write16bitoffsetreg(LDE_IF_ID,LDE_RXANTD_OFFSET,dwm1000local.rfrxDly) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_settxantennadelay()
 *
 * @brief This API function writes the antenna delay (in time units) to TX registers
 *
 * input parameters:
 * @param txDelay - this is the total (TX) antenna delay value, which
 *                          will be programmed into the TX delay register
 *
 * output parameters
 *
 * no return value
 *
 */
void DWM1000::dwm_setTXAntennaDelay(uint16 txDelay)
{
    // -------------------------------------------------------------------------------------------------------------------
    // set the tx antenna delay for auto tx timestamp adjustment
    dwm1000local.rftxDly  = txDelay;
    dwm_write16bitoffsetreg(TX_ANTD_ID, 0x0, dwm1000local.rftxDly) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_enableframefilter()
 *
 *  @brief This is used to enable the frame filtering - (the default option is to
 *  accept any data and ack frames with correct destination address
 *
 * input parameters
 * @param - bitmask - enables/disables the frame filtering options according to
 *      DWT_FF_NOTYPE_EN        0x000   no frame types allowed
 *      DWT_FF_COORD_EN         0x002   behave as coordinator (can receive frames with no destination address (PAN ID has to match))
 *      DWT_FF_BEACON_EN        0x004   beacon frames allowed
 *      DWT_FF_DATA_EN          0x008   data frames allowed
 *      DWT_FF_ACK_EN           0x010   ack frames allowed
 *      DWT_FF_MAC_EN           0x020   mac control frames allowed
 *      DWT_FF_RSVD_EN          0x040   reserved frame types allowed
 *
 * output parameters
 *
 * no return value
 */
void DWM1000::dwm_enableframefilter(uint16 enable)
{
    uint32 sysconfig = SYS_CFG_MASK & dwm_read32Bit(SYS_CFG_ID, 0) ;    // read sysconfig register

    if(enable)
    {
        //enable frame filtering and configure frame types
        sysconfig &= ~(SYS_CFG_FF_ALL_EN);   //clear all
        sysconfig |= (enable & SYS_CFG_FF_ALL_EN) | SYS_CFG_FFE;
    }
    else
    {
        sysconfig &= ~(SYS_CFG_FFE);
    }

    dwm1000local.sysCFGreg = sysconfig ;
    dwm_write32Bit(SYS_CFG_ID, 0, sysconfig) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setsmarttxpower()
 *
 * @brief This call enables or disables the smart TX power feature.
 *
 * input parameters
 * @param enable - this enables or disables the TX smart power (1 = enable, 0 = disable)
 *
 * output parameters
 *
 * no return value
 */
void DWM1000::dwm_setsmarttxpower(int enable)
{
    //config system register
    dwm1000local.sysCFGreg = dwm_read32Bit(SYS_CFG_ID, 0) ;            // read sysconfig register

    //disable smart power configuration
    if(enable)
    {
        dwm1000local.sysCFGreg &= ~(SYS_CFG_DIS_STXP) ;
    }
    else
    {
        dwm1000local.sysCFGreg |= SYS_CFG_DIS_STXP ;
    }

    dwm_write32Bit(SYS_CFG_ID, 0, dwm1000local.sysCFGreg) ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_rxenable(int delayed)
 *
 * @brief This call turns on the receiver, can be immediate or delayed.
 *  The receiver will stay turned on, listening to any messages until
 *  it either receives a good frame, an error (CRC, PHY header, Reed Solomon) or  it times out (SFD, Preamble or Frame).
 *
 * input parameters
 * @param delayed - TRUE the receiver is turned on after some delay (as programmed with dwt_setdelayedtime())
 *
 * @return DWT_DECA_SUCCESS for success, or DWT_DECA_ERROR for error (e.g. a delayed receive enable will be too far in the future if delayed time has passed (if delayed time is > 8s from now))
 */
int DWM1000::dwm_rxenable(int delayed)
{
    uint16 temp ;
    byte temp1 = 0;
    dwm_syncrxbufptrs();

    temp = (uint16)SYS_CTRL_RXENAB ;

    if (delayed)
    {
        temp |= (uint16)SYS_CTRL_RXDLYE ;
    }

    dwm_write16bitoffsetreg(SYS_CTRL_ID,0,temp) ;

    if (delayed) //check for errors
    {
        //uint32 status1 = dwt_read32bitreg(SYS_STATUS_ID) ;          // read status register

        dwm_readFromDevice(SYS_STATUS_ID,3,1,&temp1) ;

        if (temp1 & (SYS_STATUS_HPDWARN >> 24)) //if delay has not passed do delayed else immediate RX on
        {
            dwm_forceTRXOff(); //turn the delayed receive off, and do immediate receive, return warning indication
            temp = (uint16)SYS_CTRL_RXENAB; //clear the delay bit
            dwm_write16bitoffsetreg(SYS_CTRL_ID,0,temp) ;
            return DWT_ERROR;
        }
    }

    return DWT_SUCCESS;
} // end dwt_rxenable()


int DWM1000::dwm_configure(dwt_config_t * config){
//    dwm_setsmarttxpower(config->smartPowerEn);
//    setDataRate(config->dataRate);
//    setPulseFrequency(config->pulseFrequency);
//    setPreambleLength(config->preambleLength);
//    setChannel(config->channel);
//    setPreambleCode(config->preambleCode);

    dwm_setsmarttxpower(0);
    setDataRate(_dataRate);
    setPulseFrequency(_pulseFrequency);
    setPreambleLength(_preambleLength);
    setChannel(_channel);
    setPreambleCode(_preambleCode);

    dwm_writeToDevice(CHAN_CTRL_ID, 0x00, CHAN_CTRL_LEN, _chanctrl);
    dwm_writeToDevice(TX_FCTRL_ID, 0x00, TX_FCTRL_LEN, _txfctrl) ;

    //dwm_tune(config);
}

void DWM1000::setDataRate(byte rate) {
    rate &= 0x03;
    _txfctrl[1] &= 0x83;
    _txfctrl[1] |= (byte)((rate << 5) & 0xFF);

    // for 110 kbps we need to special setup
    if(_dataRate == DWT_BR_110K) {
        dwm1000local.sysCFGreg |= SYS_CFG_RXM110K;
    } else {
        dwm1000local.sysCFGreg &= (~SYS_CFG_RXM110K) ;
    }
    dwm_write32Bit(SYS_CFG_ID, 0, dwm1000local.sysCFGreg) ;

    // SFD mode and type (non-configurable, as in Table )
    if(rate == DWT_BR_6M8) {
        setBit(_chanctrl, CHAN_CTRL_LEN, DWSFD_BIT, false);
        setBit(_chanctrl, CHAN_CTRL_LEN, TNSSFD_BIT, false);
        setBit(_chanctrl, CHAN_CTRL_LEN, RNSSFD_BIT, false);
    } else {
        setBit(_chanctrl, CHAN_CTRL_LEN, DWSFD_BIT, true);
        setBit(_chanctrl, CHAN_CTRL_LEN, TNSSFD_BIT, true);
        setBit(_chanctrl, CHAN_CTRL_LEN, RNSSFD_BIT, true);
    }

    byte sfdLength;
    if(_dataRate == DWT_BR_6M8) {
        sfdLength = 0x08;
    } else if(_dataRate == DWT_BR_850K) {
        sfdLength = 0x10;
    } else {
        sfdLength = 0x40;
    }
    dwm_writeToDevice(USR_SFD_ID, 0x00, 1, &sfdLength);

    _dataRate = rate;
}

void DWM1000::setPulseFrequency(byte freq) {
    freq &= 0x03;
    _txfctrl[2] &= 0xFC;
    _txfctrl[2] |= (byte)(freq & 0xFF);
    _chanctrl[2] &= 0xF3;
    _chanctrl[2] |= (byte)((freq << 2) & 0xFF);
    _pulseFrequency = freq;
}

void DWM1000::setPreambleLength(byte prealen) {
    prealen &= 0x0F;
    _txfctrl[2] &= 0xC3;
    _txfctrl[2] |= (byte)((prealen << 2) & 0xFF);
    if(prealen == DWT_PLEN_64 || prealen == DWT_PLEN_128) {
        _pacSize = DWT_PAC8;
    } else if(prealen == DWT_PLEN_256 || prealen == DWT_PLEN_512) {
        _pacSize = DWT_PAC16;
    } else if(prealen == DWT_PLEN_1024) {
        _pacSize = DWT_PAC32;
    } else {
        _pacSize = DWT_PAC64;
    }
    _preambleLength = prealen;
}

void DWM1000::setChannel(byte channel) {
    channel &= 0xF;
    _chanctrl[0] = ((channel | (channel << 4)) & 0xFF);
    _channel = channel;
}

void DWM1000::setPreambleCode(byte preacode) {
    preacode &= 0x1F;
    _chanctrl[2] &= 0x3F;
    _chanctrl[2] |= ((preacode << 6) & 0xFF);
    _chanctrl[3] = 0x00;
    _chanctrl[3] = ((((preacode >> 2) & 0x07) | (preacode << 3)) & 0xFF);
    _preambleCode = preacode;
}

void DWM1000::setBit(byte data[], unsigned int n, unsigned int bit, boolean val) {
    int idx;
    int shift;

    idx = bit / 8;

    byte* targetByte = &data[idx];
    shift = bit % 8;
    if(val) {
        bitSet(*targetByte, shift);
    } else {
        bitClear(*targetByte, shift);
    }
}

void DWM1000::dwm_tune(dwt_config_t * config) {
    // these registers are going to be tuned/configured
    byte agctune1[AGC_TUNE1_LEN];
    byte agctune2[AGC_TUNE2_LEN];
    byte agctune3[AGC_TUNE3_LEN];

    byte drxtune0b[DRX_TUNE0b_LEN];
    byte drxtune1a[DRX_TUNE1a_LEN];
    byte drxtune1b[DRX_TUNE1b_LEN];

    byte drxtune2[DRX_TUNE2_LEN];
    byte drxtune4H[DRX_DRX_TUNE4H_LEN];
    byte ldecfg1[LDE_CFG1_LEN];
    byte ldecfg2[LDE_CFG2_LEN];
    byte lderepc[LDE_REPC_LEN];
    byte txpower[TX_POWER_LEN];
    byte rfrxctrlh[RF_RXCTRLH_LEN];
    byte rftxctrl[RF_TXCTRL_LEN];
    byte tcpgdelay[TC_PGDELAY_LEN];
    byte fspllcfg[FS_PLLTUNE_LEN];
    byte fsplltune[4];
    byte fsxtalt[FS_XTALT_LEN];

    byte _preambleLength;


    // AGC_TUNE1
//    switch (config->pulseFrequency){
//        case DWT_PRF_16M:
//            Conversion::convertValueToBytes(agctune1, 0x8870,AGC_TUNE1_LEN);
//            break;
//        case DWT_PRF_64M:
//            Conversion::convertValueToBytes(agctune1, 0x889B, AGC_TUNE1_LEN);
//            break;
//    }
    if(_pulseFrequency == DWT_PRF_16M) {
        Conversion::convertValueToBytes(agctune1, 0x8870,AGC_TUNE1_LEN);
    } else if(_pulseFrequency == DWT_PRF_64M) {
        Conversion::convertValueToBytes(agctune1, 0x889B, AGC_TUNE1_LEN);
    }


    // AGC_TUNE2
    Conversion::convertValueToBytes(agctune2, 0x2502A907L, AGC_TUNE2_LEN);
    // AGC_TUNE3
    Conversion::convertValueToBytes(agctune3, 0x0035, AGC_TUNE3_LEN);
    // DRX_TUNE0b (already optimized according to Table 20 of user manual)

//    switch (config->dataRate){
//        case DWT_BR_110K:
//            Conversion::convertValueToBytes(drxtune0b, 0x0016, DRX_TUNE0b_LEN);
//            break;
//        case DWT_BR_850K:
//            Conversion::convertValueToBytes(drxtune0b, 0x0006, DRX_TUNE0b_LEN);
//            break;
//        case DWT_BR_6M8:
//            Conversion::convertValueToBytes(drxtune0b, 0x0001, DRX_TUNE0b_LEN);
//            break;
//    }
    if(_dataRate == DWT_BR_110K) {
        Conversion::convertValueToBytes(drxtune0b, 0x0016, DRX_TUNE0b_LEN);
    } else if(_dataRate == DWT_BR_850K) {
        Conversion::convertValueToBytes(drxtune0b, 0x0006, DRX_TUNE0b_LEN);
    } else if(_dataRate == DWT_BR_6M8) {
        Conversion::convertValueToBytes(drxtune0b, 0x0001, DRX_TUNE0b_LEN);
    }

//    switch (config->pulseFrequency){
//        case DWT_PRF_16M:
//            Conversion::convertValueToBytes(drxtune1a, 0x0087, DRX_TUNE1a_LEN);
//            break;
//        case DWT_PRF_64M:
//            Conversion::convertValueToBytes(drxtune1a, 0x008D, DRX_TUNE1a_LEN);
//            break;
//    }


//    // DRX_TUNE1a
    if(_pulseFrequency == DWT_PRF_16M) {
        Conversion::convertValueToBytes(drxtune1a, 0x0087, DRX_TUNE1a_LEN);
    } else if(_pulseFrequency == DWT_PRF_64M) {
        Conversion::convertValueToBytes(drxtune1a, 0x008D, DRX_TUNE1a_LEN);
    }


//    switch (config->preambleLength){
//        case DWT_PLEN_1536:
//            if(config->dataRate == DWT_BR_110K) {
//                Conversion::convertValueToBytes(drxtune1b, 0x0064, DRX_TUNE1b_LEN);
//            }
//            break;
//        case DWT_PLEN_2048:
//            if(config->dataRate == DWT_BR_110K) {
//                Conversion::convertValueToBytes(drxtune1b, 0x0064, DRX_TUNE1b_LEN);
//            }
//            break;
//        case DWT_PLEN_4096:
//            if(config->dataRate == DWT_BR_110K) {
//                Conversion::convertValueToBytes(drxtune1b, 0x0064, DRX_TUNE1b_LEN);
//            }
//            break;
//        case !DWT_PLEN_64:
//            if(config->dataRate == (DWT_BR_850K || DWT_BR_6M8)) {
//                Conversion::convertValueToBytes(drxtune1b, 0x0020, DRX_TUNE1b_LEN);
//            }
//            break;
//        default:
//            if(config->dataRate == DWT_BR_6M8) {
//                Conversion::convertValueToBytes(drxtune1b, 0x0010, DRX_TUNE1b_LEN);
//            }
//            break;
//    }

    // DRX_TUNE1b
    if(_preambleLength ==  DWT_PLEN_1536 || _preambleLength ==  DWT_PLEN_2048||
       _preambleLength ==  DWT_PLEN_4096) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(drxtune1b, 0x0064, DRX_TUNE1b_LEN);
        }
    } else if(_preambleLength != DWT_PLEN_64) {
        if(_dataRate == DWT_BR_850K || _dataRate == DWT_BR_6M8) {
            Conversion::convertValueToBytes(drxtune1b, 0x0020, DRX_TUNE1b_LEN);
        }
    } else {
        if(_dataRate == DWT_BR_6M8) {
            Conversion::convertValueToBytes(drxtune1b, 0x0010, DRX_TUNE1b_LEN);
        }
    }

        /*switch (config->pacSize){
            case DWT_PAC8:
                switch (config->pulseFrequency){
                    case DWT_PRF_16M:
                        Conversion::convertValueToBytes(drxtune2, 0x311A002DL, DRX_TUNE2_LEN);
                        break;
                    case DWT_PRF_64M:
                        Conversion::convertValueToBytes(drxtune2, 0x313B006BL, DRX_TUNE2_LEN);
                        break;
                }
                break;
            case DWT_PAC16:
                switch (config->pulseFrequency){
                    case DWT_PRF_16M:
                        Conversion::convertValueToBytes(drxtune2, 0x331A0052L, DRX_TUNE2_LEN);
                        break;
                    case DWT_PRF_64M:
                        Conversion::convertValueToBytes(drxtune2, 0x333B00BEL, DRX_TUNE2_LEN);
                        break;
                }
                break;
            case DWT_PAC32:
                switch (config->pulseFrequency){
                    case DWT_PRF_16M:
                        Conversion::convertValueToBytes(drxtune2, 0x351A009AL, DRX_TUNE2_LEN);
                        break;
                    case DWT_PRF_64M:
                        Conversion::convertValueToBytes(drxtune2, 0x353B015EL, DRX_TUNE2_LEN);
                        break;
                }
                break;
            case DWT_PAC64:
                switch (config->pulseFrequency){
                    case DWT_PRF_16M:
                        Conversion::convertValueToBytes(drxtune2, 0x371A011DL, DRX_TUNE2_LEN);
                        break;
                    case DWT_PRF_64M:
                        Conversion::convertValueToBytes(drxtune2, 0x373B0296L, DRX_TUNE2_LEN);
                        break;
                }
                break;

        }*/

    // DRX_TUNE2
    if(_pacSize == DWT_PAC8) {
        if(_pulseFrequency == DWT_PRF_16M) {
            Conversion::convertValueToBytes(drxtune2, 0x311A002DL, DRX_TUNE2_LEN);
        } else if(_pulseFrequency == DWT_PRF_64M) {
            Conversion::convertValueToBytes(drxtune2, 0x313B006BL, DRX_TUNE2_LEN);
        }
    } else if(_pacSize == DWT_PAC16) {
        if(_pulseFrequency == DWT_PRF_16M) {
            Conversion::convertValueToBytes(drxtune2, 0x331A0052L, DRX_TUNE2_LEN);
        } else if(_pulseFrequency == DWT_PRF_64M) {
            Conversion::convertValueToBytes(drxtune2, 0x333B00BEL, DRX_TUNE2_LEN);
        }
    } else if(_pacSize == DWT_PAC32) {
        if(_pulseFrequency == DWT_PRF_16M) {
            Conversion::convertValueToBytes(drxtune2, 0x351A009AL, DRX_TUNE2_LEN);
        } else if(_pulseFrequency == DWT_PRF_64M) {
            Conversion::convertValueToBytes(drxtune2, 0x353B015EL, DRX_TUNE2_LEN);
        }
    } else if(_pacSize == DWT_PAC64) {
        if(_pulseFrequency == DWT_PRF_16M) {
            Conversion::convertValueToBytes(drxtune2, 0x371A011DL, DRX_TUNE2_LEN);
        } else if(_pulseFrequency == DWT_PRF_64M) {
            Conversion::convertValueToBytes(drxtune2, 0x373B0296L, DRX_TUNE2_LEN);
        }
    }

//    switch (config->preambleLength){
//        case DWT_PLEN_64:
//            Conversion::convertValueToBytes(drxtune4H, 0x0010, DRX_DRX_TUNE4H_LEN);
//            break;
//        default:
//            Conversion::convertValueToBytes(drxtune4H, 0x0028, DRX_DRX_TUNE4H_LEN);
//            break;
//    }

    // DRX_TUNE4H
    if(_preambleLength == DWT_PLEN_64) {
        Conversion::convertValueToBytes(drxtune4H, 0x0010, DRX_DRX_TUNE4H_LEN);
    } else {
        Conversion::convertValueToBytes(drxtune4H, 0x0028, DRX_DRX_TUNE4H_LEN);
    }

//    switch (config->channel){
//        case CHANNEL_1 || CHANNEL_2 || CHANNEL_3 || CHANNEL_5:
//            Conversion::convertValueToBytes(rfrxctrlh, 0xD8, RF_RXCTRLH_LEN);
//            break;
//        default:
//            Conversion::convertValueToBytes(rfrxctrlh, 0xD8, RF_RXCTRLH_LEN);
//            break;
//    }

    // RF_RXCTRLH
    if(_channel != CHANNEL_4 && _channel != CHANNEL_7) {
        Conversion::convertValueToBytes(rfrxctrlh, 0xD8, RF_RXCTRLH_LEN);
    } else {
        Conversion::convertValueToBytes(rfrxctrlh, 0xBC, RF_RXCTRLH_LEN);
    }

//    // RX_TXCTRL && TC_PGDELAY FS_PLLCFG and FS_PLLTUNE and RF_RXCTRLH
//    switch (config->channel){
//        case CHANNEL_1:
//            Conversion::convertValueToBytes(rftxctrl, 0x00005C40L, RF_TXCTRL_LEN);
//            Conversion::convertValueToBytes(tcpgdelay, 0xC9, TC_PGDELAY_LEN);
//            Conversion::convertValueToBytes(fspllcfg, 0x09000407L, FS_PLLCFG_LEN);
//            Conversion::convertValueToBytes(fsplltune, 0x1E, FS_PLLTUNE_LEN);
//            Conversion::convertValueToBytes(rfrxctrlh, 0xD8, RF_RXCTRLH_LEN);
//            switch (config->pulseFrequency){
//                case DWT_PRF_16M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x15355575L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x75757575L, TX_POWER_LEN);
//                    break;
//                case DWT_PRF_64M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x07274767L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x67676767L, TX_POWER_LEN);
//                    break;
//            }
//            break;
//        case CHANNEL_2:
//            Conversion::convertValueToBytes(rftxctrl, 0x00045CA0L, RF_TXCTRL_LEN);
//            Conversion::convertValueToBytes(tcpgdelay, 0xC2, TC_PGDELAY_LEN);
//            Conversion::convertValueToBytes(fspllcfg, 0x08400508L, FS_PLLCFG_LEN);
//            Conversion::convertValueToBytes(fsplltune, 0x26, FS_PLLTUNE_LEN);
//            Conversion::convertValueToBytes(rfrxctrlh, 0xD8, RF_RXCTRLH_LEN);
//            switch (config->pulseFrequency){
//                case DWT_PRF_16M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x15355575L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x75757575L, TX_POWER_LEN);
//                    break;
//                case DWT_PRF_64M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x07274767L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x67676767L, TX_POWER_LEN);
//                    break;
//            }
//            break;
//        case CHANNEL_3:
//            Conversion::convertValueToBytes(rftxctrl, 0x00086CC0L, RF_TXCTRL_LEN);
//            Conversion::convertValueToBytes(tcpgdelay, 0xC5, TC_PGDELAY_LEN);
//            Conversion::convertValueToBytes(fspllcfg, 0x08401009L, FS_PLLCFG_LEN);
//            Conversion::convertValueToBytes(fsplltune, 0x5E, FS_PLLTUNE_LEN);
//            Conversion::convertValueToBytes(rfrxctrlh, 0xD8, RF_RXCTRLH_LEN);
//            switch (config->pulseFrequency){
//                case DWT_PRF_16M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x0F2F4F6FL, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x6F6F6F6FL, TX_POWER_LEN);
//                    break;
//                case DWT_PRF_64M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x2B4B6B8BL, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x8B8B8B8BL, TX_POWER_LEN);
//                    break;
//            }
//            break;
//            break;
//        case CHANNEL_4:
//            Conversion::convertValueToBytes(rftxctrl, 0x00045C80L, RF_TXCTRL_LEN);
//            Conversion::convertValueToBytes(tcpgdelay, 0xC5, TC_PGDELAY_LEN);
//            Conversion::convertValueToBytes(fspllcfg, 0x08400508L, FS_PLLCFG_LEN);
//            Conversion::convertValueToBytes(fsplltune, 0x26, FS_PLLTUNE_LEN);
//            Conversion::convertValueToBytes(rfrxctrlh, 0xD8, RF_RXCTRLH_LEN);
//            switch (config->pulseFrequency){
//                case DWT_PRF_16M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x1F1F3F5FL, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x5F5F5F5FL, TX_POWER_LEN);
//                    break;
//                case DWT_PRF_64M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x3A5A7A9AL, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x9A9A9A9AL, TX_POWER_LEN);
//                    break;
//            }
//            break;
//        case CHANNEL_5:
//            Conversion::convertValueToBytes(rftxctrl, 0x001E3FE0L, RF_TXCTRL_LEN);
//            Conversion::convertValueToBytes(tcpgdelay, 0xC0, TC_PGDELAY_LEN);
//            Conversion::convertValueToBytes(fspllcfg, 0x0800041DL, FS_PLLCFG_LEN);
//            Conversion::convertValueToBytes(fsplltune, 0xA6, FS_PLLTUNE_LEN);
//            Conversion::convertValueToBytes(rfrxctrlh, 0xD8, RF_RXCTRLH_LEN);
//            switch (config->pulseFrequency){
//                case DWT_PRF_16M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x0E082848L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x48484848L, TX_POWER_LEN);
//                    break;
//                case DWT_PRF_64M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x25456585L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x85858585L, TX_POWER_LEN);
//                    break;
//            }
//            break;
//        case CHANNEL_7:
//            Conversion::convertValueToBytes(rftxctrl, 0x001E7DE0L, RF_TXCTRL_LEN);
//            Conversion::convertValueToBytes(tcpgdelay, 0x93, TC_PGDELAY_LEN);
//            Conversion::convertValueToBytes(fspllcfg, 0x0800041DL, FS_PLLCFG_LEN);
//            Conversion::convertValueToBytes(fsplltune, 0xA6, FS_PLLTUNE_LEN);
//            Conversion::convertValueToBytes(rfrxctrlh, 0xD8, RF_RXCTRLH_LEN);
//            switch (config->pulseFrequency){
//                case DWT_PRF_16M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x32527292L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x92929292L, TX_POWER_LEN);
//                    break;
//                case DWT_PRF_64M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x5171B1D1L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0xD1D1D1D1L, TX_POWER_LEN);
//                    break;
//            }
//            break;
//    }

    // RX_TXCTRL
    if(_channel == CHANNEL_1) {
        Conversion::convertValueToBytes(rftxctrl, 0x00005C40L, RF_TXCTRL_LEN);
    } else if(_channel == CHANNEL_2) {
        Conversion::convertValueToBytes(rftxctrl, 0x00045CA0L, RF_TXCTRL_LEN);
    } else if(_channel == CHANNEL_3) {
        Conversion::convertValueToBytes(rftxctrl, 0x00086CC0L, RF_TXCTRL_LEN);
    } else if(_channel == CHANNEL_4) {
        Conversion::convertValueToBytes(rftxctrl, 0x00045C80L, RF_TXCTRL_LEN);
    } else if(_channel == CHANNEL_5) {
        Conversion::convertValueToBytes(rftxctrl, 0x001E3FE0L, RF_TXCTRL_LEN);
    } else if(_channel == CHANNEL_7) {
        Conversion::convertValueToBytes(rftxctrl, 0x001E7DE0L, RF_TXCTRL_LEN);
    }

    // TC_PGDELAY
    if(_channel == CHANNEL_1) {
        Conversion::convertValueToBytes(tcpgdelay, 0xC9, TC_PGDELAY_LEN);
    } else if(_channel == CHANNEL_2) {
        Conversion::convertValueToBytes(tcpgdelay, 0xC2, TC_PGDELAY_LEN);
    } else if(_channel == CHANNEL_3) {
        Conversion::convertValueToBytes(tcpgdelay, 0xC5, TC_PGDELAY_LEN);
    } else if(_channel == CHANNEL_4) {
        Conversion::convertValueToBytes(tcpgdelay, 0x95, TC_PGDELAY_LEN);
    } else if(_channel == CHANNEL_5) {
        Conversion::convertValueToBytes(tcpgdelay, 0xC0, TC_PGDELAY_LEN);
    } else if(_channel == CHANNEL_7) {
        Conversion::convertValueToBytes(tcpgdelay, 0x93, TC_PGDELAY_LEN);
    }

    // FS_PLLCFG and FS_PLLTUNE
    if(_channel == CHANNEL_1) {
        Conversion::convertValueToBytes(fspllcfg, 0x09000407L, FS_PLLCFG_LEN);
        Conversion::convertValueToBytes(fsplltune, 0x1E, FS_PLLTUNE_LEN);
    } else if(_channel == CHANNEL_2 || _channel == CHANNEL_4) {
        Conversion::convertValueToBytes(fspllcfg, 0x08400508L, FS_PLLCFG_LEN);
        Conversion::convertValueToBytes(fsplltune, 0x26, FS_PLLTUNE_LEN);
    } else if(_channel == CHANNEL_3) {
        Conversion::convertValueToBytes(fspllcfg, 0x08401009L, FS_PLLCFG_LEN);
        Conversion::convertValueToBytes(fsplltune, 0x5E, FS_PLLTUNE_LEN);
    } else if(_channel == CHANNEL_5 || _channel == CHANNEL_7) {
        Conversion::convertValueToBytes(fspllcfg, 0x0800041DL, FS_PLLCFG_LEN);
        Conversion::convertValueToBytes(fsplltune, 0xA6, FS_PLLTUNE_LEN);
    }

    // LDE_CFG1
    Conversion::convertValueToBytes(ldecfg1, 0xD, LDE_CFG1_LEN);

//    switch (config->pulseFrequency){
//        case DWT_PRF_16M:
//            Conversion::convertValueToBytes(ldecfg2, 0x1607, LDE_CFG2_LEN);
//            break;
//        case DWT_PRF_64M:
//            Conversion::convertValueToBytes(ldecfg2, 0x0607, LDE_CFG2_LEN);
//            break;
//    }

    // LDE_CFG2
    if(_pulseFrequency == DWT_PRF_16M) {
        Conversion::convertValueToBytes(ldecfg2, 0x1607, LDE_CFG2_LEN);
    } else if(_pulseFrequency == DWT_PRF_64M) {
        Conversion::convertValueToBytes(ldecfg2, 0x0607, LDE_CFG2_LEN);
    }


//    switch (config->preambleCode){
//        case PREAMBLE_CODE_16MHZ_1:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x5998, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_16MHZ_2:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x5998, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_16MHZ_3 :
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x51EA, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_16MHZ_4:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x428E >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x428E, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_16MHZ_5:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x451E >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x451E, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_16MHZ_6:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x2E14 >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x2E14, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_16MHZ_7:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x8000 >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x8000, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_16MHZ_8 :
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x51EA, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_64MHZ_9:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x28F4 >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x28F4, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_64MHZ_10:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x3332, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_64MHZ_11:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x3AE0 >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x3AE0, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_64MHZ_12:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x3D70 >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x3D70, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_64MHZ_17:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x3332, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_64MHZ_18:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x35C2, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case  PREAMBLE_CODE_64MHZ_19:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x35C2, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//        case PREAMBLE_CODE_64MHZ_20:
//            switch (config->dataRate){
//                case DWT_BR_110K:
//                    Conversion::convertValueToBytes(lderepc, ((0x47AE >> 3) & 0xFFFF), LDE_REPC_LEN);
//                    break;
//                default:
//                    Conversion::convertValueToBytes(lderepc, 0x47AE, LDE_REPC_LEN);
//                    break;
//            }
//            break;
//
//    }

    // LDE_REPC
    if(_preambleCode == PREAMBLE_CODE_16MHZ_1 || _preambleCode == PREAMBLE_CODE_16MHZ_2) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x5998, LDE_REPC_LEN);
        }
    } else if(_preambleCode == PREAMBLE_CODE_16MHZ_3 || _preambleCode == PREAMBLE_CODE_16MHZ_8) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x51EA, LDE_REPC_LEN);
        }
    } else if(_preambleCode == PREAMBLE_CODE_16MHZ_4) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x428E >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x428E, LDE_REPC_LEN);
        }
    } else if(_preambleCode == PREAMBLE_CODE_16MHZ_5) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x451E >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x451E, LDE_REPC_LEN);
        }
    } else if(_preambleCode == PREAMBLE_CODE_16MHZ_6) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x2E14 >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x2E14, LDE_REPC_LEN);
        }
    } else if(_preambleCode == PREAMBLE_CODE_16MHZ_7) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x8000 >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x8000, LDE_REPC_LEN);
        }
    } else if(_preambleCode == PREAMBLE_CODE_64MHZ_9) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x28F4 >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x28F4, LDE_REPC_LEN);
        }
    } else if(_preambleCode == PREAMBLE_CODE_64MHZ_10 || _preambleCode == PREAMBLE_CODE_64MHZ_17) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x3332, LDE_REPC_LEN);
        }
    } else if(_preambleCode == PREAMBLE_CODE_64MHZ_11) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x3AE0 >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x3AE0, LDE_REPC_LEN);
        }
    } else if(_preambleCode == PREAMBLE_CODE_64MHZ_12) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x3D70 >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x3D70, LDE_REPC_LEN);
        }
    } else if(_preambleCode == PREAMBLE_CODE_64MHZ_18 || _preambleCode == PREAMBLE_CODE_64MHZ_19) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x35C2, LDE_REPC_LEN);
        }
    } else if(_preambleCode == PREAMBLE_CODE_64MHZ_20) {
        if(_dataRate == DWT_BR_110K) {
            Conversion::convertValueToBytes(lderepc, ((0x47AE >> 3) & 0xFFFF), LDE_REPC_LEN);
        } else {
            Conversion::convertValueToBytes(lderepc, 0x47AE, LDE_REPC_LEN);
        }
    }


//    switch (config->channel){
//        case CHANNEL_1:
//            switch (config->pulseFrequency){
//                case DWT_PRF_16M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x15355575L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x75757575L, TX_POWER_LEN);
//                    break;
//                case DWT_PRF_64M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x07274767L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x67676767L, TX_POWER_LEN);
//                    break;
//            }
//            break;
//        case CHANNEL_3:
//            switch (config->pulseFrequency){
//                case DWT_PRF_16M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x0F2F4F6FL, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x6F6F6F6FL, TX_POWER_LEN);
//                    break;
//                case DWT_PRF_64M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x2B4B6B8BL, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x8B8B8B8BL, TX_POWER_LEN);
//                    break;
//            }
//            break;
//        case CHANNEL_4:
//            switch (config->pulseFrequency){
//                case DWT_PRF_16M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x1F1F3F5FL, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x5F5F5F5FL, TX_POWER_LEN);
//                    break;
//                case DWT_PRF_64M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x3A5A7A9AL, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x9A9A9A9AL, TX_POWER_LEN);
//                    break;
//            }
//            break;
//        case CHANNEL_5:
//            switch (config->pulseFrequency){
//                case DWT_PRF_16M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x0E082848L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x48484848L, TX_POWER_LEN);
//                    break;
//                case DWT_PRF_64M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x25456585L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x85858585L, TX_POWER_LEN);
//                    break;
//            }
//            break;
//        case CHANNEL_7:
//            switch (config->pulseFrequency){
//                case DWT_PRF_16M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x32527292L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0x92929292L, TX_POWER_LEN);
//                    break;
//                case DWT_PRF_64M:
//                    if(config->smartPowerEn)
//                        Conversion::convertValueToBytes(txpower, 0x5171B1D1L, TX_POWER_LEN);
//                    else
//                        Conversion::convertValueToBytes(txpower, 0xD1D1D1D1L, TX_POWER_LEN);
//                    break;
//            }
//            break;
//    }


    // TX_POWER (enabled smart transmit power control)
    if(_channel == CHANNEL_1 || _channel == CHANNEL_2) {
        if(_pulseFrequency == DWT_PRF_16M) {
            if(_smartPower) {
                Conversion::convertValueToBytes(txpower, 0x15355575L, TX_POWER_LEN);
            } else {
                Conversion::convertValueToBytes(txpower, 0x75757575L, TX_POWER_LEN);
            }
        } else if(_pulseFrequency == DWT_PRF_64M) {
            if(_smartPower) {
                Conversion::convertValueToBytes(txpower, 0x07274767L, TX_POWER_LEN);
            } else {
                Conversion::convertValueToBytes(txpower, 0x67676767L, TX_POWER_LEN);
            }
        }
    } else if(_channel == CHANNEL_3) {
        if(_pulseFrequency == DWT_PRF_16M) {
            if(_smartPower) {
                Conversion::convertValueToBytes(txpower, 0x0F2F4F6FL, TX_POWER_LEN);
            } else {
                Conversion::convertValueToBytes(txpower, 0x6F6F6F6FL, TX_POWER_LEN);
            }
        } else if(_pulseFrequency == DWT_PRF_64M) {
            if(_smartPower) {
                Conversion::convertValueToBytes(txpower, 0x2B4B6B8BL, TX_POWER_LEN);
            } else {
                Conversion::convertValueToBytes(txpower, 0x8B8B8B8BL, TX_POWER_LEN);
            }
        }
    } else if(_channel == CHANNEL_4) {
        if(_pulseFrequency == DWT_PRF_16M) {
            if(_smartPower) {
                Conversion::convertValueToBytes(txpower, 0x1F1F3F5FL, TX_POWER_LEN);
            } else {
                Conversion::convertValueToBytes(txpower, 0x5F5F5F5FL, TX_POWER_LEN);
            }
        } else if(_pulseFrequency == DWT_PRF_64M) {
            if(_smartPower) {
                Conversion::convertValueToBytes(txpower, 0x3A5A7A9AL, TX_POWER_LEN);
            } else {
                Conversion::convertValueToBytes(txpower, 0x9A9A9A9AL, TX_POWER_LEN);
            }
        }
    } else if(_channel == CHANNEL_5) {
        if(_pulseFrequency == DWT_PRF_16M) {
            if(_smartPower) {
                Conversion::convertValueToBytes(txpower, 0x0E082848L, TX_POWER_LEN);
            } else {
                Conversion::convertValueToBytes(txpower, 0x48484848L, TX_POWER_LEN);
            }
        } else if(_pulseFrequency ==DWT_PRF_64M) {
            if(_smartPower) {
                Conversion::convertValueToBytes(txpower, 0x25456585L, TX_POWER_LEN);
            } else {
                Conversion::convertValueToBytes(txpower, 0x85858585L, TX_POWER_LEN);
            }
        }
    } else if(_channel == CHANNEL_7) {
        if(_pulseFrequency == DWT_PRF_16M) {
            if(_smartPower) {
                Conversion::convertValueToBytes(txpower, 0x32527292L, TX_POWER_LEN);
            } else {
                Conversion::convertValueToBytes(txpower, 0x92929292L, TX_POWER_LEN);
            }
        } else if(_pulseFrequency == DWT_PRF_64M) {
            if(_smartPower) {
                Conversion::convertValueToBytes(txpower, 0x5171B1D1L, TX_POWER_LEN);
            } else {
                Conversion::convertValueToBytes(txpower, 0xD1D1D1D1L, TX_POWER_LEN);
            }
        }
    }

    // write configuration back to chip
    dwm_writeToDevice(AGC_CTRL_ID, AGC_TUNE1_OFFSET, AGC_TUNE1_LEN, agctune1);
    dwm_writeToDevice(AGC_CTRL_ID, AGC_TUNE2_OFFSET, AGC_TUNE2_LEN, agctune2);
    dwm_writeToDevice(AGC_CTRL_ID, AGC_TUNE3_OFFSET, AGC_TUNE3_LEN, agctune3);

    dwm_writeToDevice(DRX_CONF_ID, DRX_TUNE0b_OFFSET, DRX_TUNE0b_LEN, drxtune0b);
    dwm_writeToDevice(DRX_CONF_ID, DRX_TUNE1a_OFFSET, DRX_TUNE1a_LEN, drxtune1a);
    dwm_writeToDevice(DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_LEN, drxtune1b);
    dwm_writeToDevice(DRX_CONF_ID, DRX_TUNE2_OFFSET, DRX_TUNE2_LEN, drxtune2);
    dwm_writeToDevice(DRX_CONF_ID, DRX_DRX_TUNE4HOFFSET, DRX_DRX_TUNE4H_LEN, drxtune4H);

    dwm_writeToDevice(LDE_IF_ID , LDE_CFG1_OFFSET, LDE_CFG1_LEN, ldecfg1);
    dwm_writeToDevice(LDE_IF_ID , LDE_CFG2_OFFSET, LDE_CFG2_LEN, ldecfg2);
    dwm_writeToDevice(LDE_IF_ID , LDE_REPC_OFFSET, LDE_REPC_LEN, lderepc);

    dwm_writeToDevice(TX_POWER_ID, 0x00 , TX_POWER_LEN, txpower);

    dwm_writeToDevice(RF_CONF_ID, RF_RXCTRLH_OFFSET, RF_RXCTRLH_LEN, rfrxctrlh);
    dwm_writeToDevice(RF_CONF_ID, RF_TXCTRL_OFFSET, RF_TXCTRL_LEN, rftxctrl);

    dwm_writeToDevice(TX_CAL_ID, TC_PGDELAY_OFFSET, TC_PGDELAY_LEN, tcpgdelay);

    dwm_writeToDevice(FS_CTRL_ID, FS_PLLTUNE_OFFSET, FS_PLLTUNE_LEN, fsplltune);
    dwm_writeToDevice(FS_CTRL_ID, FS_PLLCFG_OFFSET, 4, fspllcfg);
    dwm_writeToDevice(FS_CTRL_ID, FS_XTALT_OFFSET, FS_XTALT_LEN, fsxtalt);
}

