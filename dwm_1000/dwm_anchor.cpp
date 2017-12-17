#include "dwm_anchor.h"


//byte DWMAnchor::_longAddressTag[8];  //Big-Endian
//byte DWMAnchor::_shortAddressTag[2];
//RangingInit DWMAnchor::rangingInit;
//Response DWMAnchor::response;
//Report DWMAnchor::report;
//DWMTimeAnchor DWMAnchor::timeAnchor;
//uint32 DWMAnchor::timeElapsed = 0;
//uint32 DWMAnchor::_timer = 0;
//bool DWMAnchor::_alive = false;

DWMAnchor::DWMAnchor() { }

DWMAnchor::~DWMAnchor() { }

void DWMAnchor::startAnchor(int irq, int ss, int rst, byte * short_address) {

	init(irq, ss, rst);
	
	setLongAddress();
	setShortAddress(short_address);
	setPanID();
	//setShortAddressTag();

	DWM1000::dwm_setSendDone(&handleSent);
	DWM1000::dwm_setReceiveDone(&handleReceived);
	DWM1000::dwm_setReceiveFailed(&handleReceiveFailed);

	DWM1000::dwm_setEUI(_longAddress);
	uint16 pan = _panID[0] + (_panID[1] << 8);
	DWM1000::dwm_setpanid(pan);
	uint16 addr = _shortAddress[0] + (_shortAddress[1] << 8);
	DWM1000::dwm_setaddress16(addr);
	DWM1000::dwm_receive(false);
	DWM1000::dwm_setReceiveInterrupt();
	DWM1000::dwm_setSendInterrupt();
	DWM1000::dwm_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering

}

/* Long address of Anchor */
void DWMAnchor::setLongAddress(){
	_longAddress[0] = 0xAA;
	_longAddress[1] = 0xAA;
	_longAddress[2] = 0xAA;
	_longAddress[3] = 0xAA;
	_longAddress[4] = 0xAA;
	_longAddress[5] = 0xAA;
	_longAddress[6] = 0xAA;
	_longAddress[7] = 0xAA;
}

void DWMAnchor::get_long_address_TAG(byte frame[]) {
    _longAddressTag[0] = frame[2];
    _longAddressTag[1] = frame[3];
    _longAddressTag[2] = frame[4];
    _longAddressTag[3] = frame[5];
    _longAddressTag[4] = frame[6];
    _longAddressTag[5] = frame[7];
    _longAddressTag[6] = frame[8];
    _longAddressTag[7] = frame[9];
}

void DWMAnchor::get_short_address_TAG(byte frame[]){
	_shortAddressTag[0] = frame[7];
	_shortAddressTag[1] = frame[8];
}

void DWMAnchor::messageArrived() {
    DWM1000::dwm_returnDataReceived(buffer, &length);
    //DWM1000::dwm_readDataReceived();
    _messageArrived = detectMessageType(buffer);

    switch(_messageArrived){
        case BLINK:
#if PRINT
            Serial.println("Receive blink");
#endif
            DWM1000::dwm_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ACK frames;
            get_long_address_TAG(buffer);

            rangingInit.generateRangingInit(buffer, _longAddressTag, _shortAddress, _shortAddressTag);

            _previousMessageIn = NUL;
            _messageSend = RANGING_INIT;
            _nextMessageIn = POLL;
            break;
        case POLL:
#if PRINT
            Serial.println("Receive poll");
#endif
            timeAnchor.setTimestampPollRX();

            get_short_address_TAG(buffer);
            response.generateResponse(buffer, _shortAddressTag, _shortAddress);
            _previousMessageIn = BLINK;
            _messageSend = RESPONSE;
            _nextMessageIn = FINAL;

            break;
        case FINAL:
#if PRINT
            Serial.println("Receive final");
#endif

            timeAnchor.setTimestampFinalRX();

            timeAnchor.setRound1(buffer);
            timeAnchor.setReply2(buffer);
            timeAnchor.calculateTOF();

            report.generateReport(buffer, _shortAddressTag, _shortAddress, &timeAnchor);

            _previousMessageIn = POLL;
            _messageSend = REPORT;
            _nextMessageIn = POLL;

            break;
    }
}

void DWMAnchor::messageSend() {
    switch (_messageSend){
        case RANGING_INIT:
#if PRINT
            Serial.println("*****   RANGING INIT SENT");
#endif
            break;
        case RESPONSE:
            timeAnchor.setTimestampResponseTX();
#if PRINT
            Serial.println("*****   RESPONSE SENT");
#endif
            break;
        case REPORT:
#if PRINT
            Serial.println("*****   REPORT SENT");
#endif
            break;
    }
}

void DWMAnchor::loop() {

	timeElapsed = millis();
	if (timeElapsed - _timer > TIME_REFRESH) {
        _timer = timeElapsed;
        if (_alive) _alive = false;
        else DWM1000::dwm_receive(false);
    }

    if(_interruptReceiveFailed){
        _interruptReceiveFailed = false;
        DWM1000::dwm_receive(false);
    }

    //TODO gestire timeout
    if(_interruptTimeout){
        DWM1000::dwm_setrxtimeout(0);
        DWM1000::dwm_receive(false);
    }

    if(_interruptReceive){
        _interruptReceive = false;
        _alive = true;
        messageArrived();
    }

    if(_interruptSend){
        _interruptSend = false;
        messageSend();
    }
}

