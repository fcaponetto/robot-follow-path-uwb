#include "dwm_tag.h"


/* Interface's Callback */
void(Position::*complete_measure_meter)(double);
void(Position::*complete_measure_tof)(byte *);

DWMTag::DWMTag() : DWMDevice(){ }

/* A class that instanziate DWMTag must implement the interface Position */
DWMTag::DWMTag(Position *p) : DWMDevice() {
    position = p;
}

DWMTag::~DWMTag() { }

void DWMTag::startTag(int irq, int ss, int rst, byte * short_address) {
    init(irq, ss, rst);
	
	setLongAddress();
	setShortAddress(short_address);
	setPanID();
	
	DWM1000::dwm_setSendDone(&handleSent);
	DWM1000::dwm_setReceiveDone(&handleReceived);
	DWM1000::dwm_setReceiveFailed(&handleReceiveFailed);
	DWM1000::dwm_setReceiveTimeout(&handleTimeoutReceive);

	DWM1000::dwm_setEUI(_longAddress);
	
	uint16 pan = _panID[0] + (_panID[1] << 8);
	DWM1000::dwm_setpanid(pan);

	DWM1000::dwm_setReceiveInterrupt();
	DWM1000::dwm_setSendInterrupt();
	
	DWM1000::dwm_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ACK frames;
	uint16 addr = _shortAddress[0] + (_shortAddress[1] << 8);
	DWM1000::dwm_setaddress16(addr);
}

void DWMTag::setLongAddress(){
	_longAddress[0] = 0xBB;
	_longAddress[1] = 0xBB;
	_longAddress[2] = 0xBB;
	_longAddress[3] = 0xBB;
	_longAddress[4] = 0xBB;
	_longAddress[5] = 0xBB;
	_longAddress[6] = 0xBB;
	_longAddress[7] = 0xBB;
}

void DWMTag::getShortAddressTag(byte frame[], byte shortAddreddTag[]) {
    memcpy(shortAddreddTag, frame + 16, 2);
}

void DWMTag::getShortAddressAnchor(byte frame[], byte shortAddreddAnchor[]) {
    memcpy(shortAddreddAnchor, frame + 13, 2);
}

void DWMTag::sendBlink() {
#if PRINT
    Serial.println("BLINK GENERATED");
#endif
    blink.generateBlinkFrame(buffer, _longAddress);
    _previousMessageIn = NUL;
    _messageSend = BLINK;
    _nextMessageIn = RANGING_INIT;
}

void DWMTag::sendPoll() {
#if PRINT
    Serial.println("POLL GENERATED");
#endif
    led1 = !led1;
    digitalWrite(4, led1);
    poll.generatePoll(buffer, _shortAddress, _shortAddressAnchor);
    _previousMessageIn = RANGING_INIT;
    _messageSend = POLL;
    _nextMessageIn = RESPONSE;
}

/* Set Callback */
void DWMTag::set_complete_measure_meter(void(Position::*ptrf)(double)) {
    complete_measure_meter = ptrf;
}

void DWMTag::set_complete_measure_tof(void(Position::*ptrf)(byte *)) {
    complete_measure_tof = ptrf;
}

void DWMTag::messageArrived() {
    DWM1000::dwm_returnDataReceived(buffer, &length);
    //DWM1000::dwm_readDataReceived();
    _messageArrived = detectMessageType(buffer);

    switch(_messageArrived){
        case RANGING_INIT:
#if PRINT
            Serial.println("RANGING_INIT RECEIVED");
#endif
            //TODO se si comincia col Blink, assegnare propriamente lo short address
            getShortAddressTag(buffer, _shortAddress);
            getShortAddressAnchor(buffer, _shortAddressAnchor);

            _previousMessageIn = NUL;
            _messageSend = POLL;
            _nextMessageIn = RESPONSE;

            poll.generatePoll(buffer, _shortAddress, _shortAddressAnchor);
            break;

        case RESPONSE:
#if PRINT
            Serial.println("RESPONSE RECEIVED");
#endif
            timeTag.setTimestampResponseRX();

            final.generateFinal(buffer, _shortAddress, _shortAddressAnchor, &timeTag);

            _previousMessageIn = RANGING_INIT;
            _messageSend = FINAL;
            _nextMessageIn = REPORT;

#if DEBUG
            timeTag.printInformation();
#endif
            break;

        case REPORT:
#if PRINT
            Serial.println("REPORT RECEIVED");
#endif
            timeTag.setTof(buffer);
            //timeTag.calculateTof();

            //position->handle_complete_measure(timeTag.calculateTof());
            if(complete_measure_meter != 0)
                (position->*complete_measure_meter)(timeTag.calculateTof());

            if(complete_measure_tof != 0) {
                timeTag.calculateTof();
                (position->*complete_measure_tof)(timeTag.getTOF());
            }
                //completeM(timeTag.calculateTof());
            led1 = !led1;
            digitalWrite(4, led1);

            _previousMessageIn = RESPONSE;
            _messageSend = NUL;
            _nextMessageIn = RESPONSE;

            break;
    }
}

void DWMTag::messageSend() {
    switch (_messageSend) {
        case BLINK:
#if PRINT
            Serial.println("BLINK SENT");
#endif
            break;

        case POLL:
#if PRINT
            Serial.println("POLL SENT");
#endif
            timeTag.setTimestampPollTX();
            break;

        case FINAL:
#if PRINT
            Serial.println("Final SENT");
#endif
            break;
    }

}

void DWMTag::loop() {

    if(_interruptReceiveFailed){
        _interruptReceiveFailed = false;
        DWM1000::dwm_receive(false);
    }

    //TODO gestire timeout
    if(_interruptTimeout){
        _interruptTimeout = false;
        //Serial.println("Received timeout");
        DWM1000::dwm_setrxtimeout(0);  // Clear timeout
        DWM1000::dwm_receive(false);
    }

    if(_interruptReceive){
        _interruptReceive = false;
        messageArrived();
    }

    if(_interruptSend){
        _interruptSend = false;
        messageSend();
    }
}

