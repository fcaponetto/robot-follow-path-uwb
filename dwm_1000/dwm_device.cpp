#include "dwm_device.h"

volatile bool DWMDevice::_interruptSend = false;
volatile bool DWMDevice::_interruptReceive = false;
volatile bool DWMDevice::_interruptReceiveFailed = false;
volatile bool DWMDevice::_interruptTimeout = false;

DWMDevice::DWMDevice() : led1(false){
    pinMode(4, OUTPUT);
    digitalWrite(4, led1);
}

DWMDevice::~DWMDevice(){

}

void DWMDevice::init(int irq, int ss, int rst){

	if(DWM1000::dwm_initialize(irq, ss, rst) != DWT_SUCCESS){
        Serial.println("Error to inizialize device");
        return;
    }
    
    Serial.println("DECA - model: 01, version: 30");
    
    dwt_config_t instConfig;
    uint8 dr_mode = 8;

//    instConfig.chan = CHANNEL_7;
//    instConfig.txCode = PREAMBLE_CODE_16MHZ_4;
//    instConfig.rxCode = PREAMBLE_CODE_16MHZ_4;
//    instConfig.prf = DWT_PRF_16M ;
//    instConfig.rxPAC = DWT_PAC8;
//    instConfig.nsSFD = 0 ;
//    instConfig.sfdTO = 0 ;
//    instConfig.dataRate = DWT_BR_6M8 ;
//    instConfig.txPreambLength = DWT_PLEN_128;
//    instConfig.smartPowerEn = false;

    instConfig.channel = chConfig[dr_mode].channel ;
    instConfig.preambleCode = chConfig[dr_mode].preambleCode;
    instConfig.pulseFrequency = chConfig[dr_mode].pulseRepFreq ;
    instConfig.pacSize = chConfig[dr_mode].pacSize ;
    instConfig.nsSFD = chConfig[dr_mode].nsSFD ;
    instConfig.sfdTO = chConfig[dr_mode].sfdTO ;
    instConfig.dataRate = chConfig[dr_mode].dataRate ;
    instConfig.preambleLength = chConfig[dr_mode].preambleLength ;
    instConfig.smartPowerEn = false;

    
    //int use_otpdata = DWT_LOADANTDLY | DWT_LOADXTALTRIM;
    //DWM1000::dwm_configure(&instConfig, use_otpdata);
    DWM1000::dwm_configure(&instConfig);
    DWM1000::dwm_tune(&instConfig);
}

void DWMDevice::setPanID(){
	_panID[0] = 0xCA;
	_panID[1] = 0xDE;
}

void DWMDevice::setShortAddress(byte * short_address){
    _shortAddress[0] = short_address[0];
    _shortAddress[1] = short_address[1];
}

short DWMDevice::detectMessageType(byte data[]) {
	if (data[0] == BLINK_FC){
	return BLINK;}
	else if(data[0] == RANGING_FC_0 && data[1] == RANGING_FC_1 && data[15] == RANGING_INIT_CONTROL){
	return RANGING_INIT;}
	else if(data[9] == FINAL_CONTROL){
	return FINAL;}
	else if(data[9] == POLL_CONTROL){
	return POLL;}
	else if(data[9] == RESPONSE_CONTROL){
	return RESPONSE;}
	else if(data[9] == REPORT_CONTROL){
	return REPORT;}
}

void DWMDevice::handleSent(){
    _interruptSend = true;
}

void DWMDevice::handleReceived(){
    _interruptReceive = true;
}

void DWMDevice::handleTimeoutReceive() {
    _interruptTimeout = true;
}

void DWMDevice::handleReceiveFailed(){
    _interruptReceiveFailed = true;
}
