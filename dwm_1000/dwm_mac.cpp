#include "dwm_mac.h"

void DWMMac::incrementSequenceNumber() {
	_sequenceNumber++;
	_sequenceNumber %= 255;
}


/*******************************************************************************************************************************************/
Blink::Blink() {
	_sequenceNumber = 0;
	// dovrebbe essere 10
	_lenFrame = 12;
}

void Blink::generateBlinkFrame(byte frame[], byte tagID[]) {
	//Frame Control
	*frame = BLINK_FC;
	//sequence number
	*(frame + 1) = _sequenceNumber;
	
	frame[2] = tagID[0];
	frame[3] = tagID[1];
	frame[4] = tagID[2];
	frame[5] = tagID[3];
	frame[6] = tagID[4];
	frame[7] = tagID[5];
	frame[8] = tagID[6];
	frame[9] = tagID[7];

	DWM1000::dwm_transmit(frame, _lenFrame, DWT_RESPONSE_EXPECTED);
	
	incrementSequenceNumber();
	//DWM1000::dwm_setrxtimeout(64000); // 64 ms
}

/*******************************************************************************************************************************************/
RangingInit::RangingInit(){
	_lenFrame = 20;
}

void RangingInit::generateRangingInit(byte frame[], byte _destAddressTag[], byte _shortAdressAnchor[], byte _shortAddressTag[]) {
	
	frame[0] = RANGING_FC_0;
	frame[1] = RANGING_FC_1;
	frame[2] = _sequenceNumber;
	frame[3] = PAN_ID_0;
	frame[4] = PAN_ID_1;

	// 0xEEEEEEEEEEEEEEEE			/* Long-Destination address TAG */
	frame[5] = _destAddressTag[0];
	frame[6] = _destAddressTag[1];
	frame[7] = _destAddressTag[2];
	frame[8] = _destAddressTag[3];
	frame[9] = _destAddressTag[4];
	frame[10] = _destAddressTag[5];
	frame[11] = _destAddressTag[6];
	frame[12] = _destAddressTag[7];
	
	// 0x1111						/*Short-Source address ANCHOR */
	frame[13] = _shortAdressAnchor[0];
	frame[14] = _shortAdressAnchor[1];

									/* This octet value of 0x20 identifies the message as a range report */
	frame[15] = RANGING_INIT_CONTROL;

	// short address tag = 0x2222  	/* This 16-bit field specifies the 16-bit address to be used by Tag for the  ranging phase instead of its 64-bit address. */
	frame[16] = _shortAddressTag[0];
	frame[17] = _shortAddressTag[1];
	
	//delay time
	frame[18] = 0x01;
	frame[19] = 0x01;

	DWM1000::dwm_transmit(frame, _lenFrame, DWT_RESPONSE_EXPECTED);

	incrementSequenceNumber();
	//DWM1000::dwm_setrxtimeout(64000); // 64 ms
}

/*******************************************************************************************************************************************/
Poll::Poll() {
	_lenFrame = 10;
}

void Poll::generatePoll(byte frame[], byte _shortAddressTag[], byte _shortAdressAnchor[]) {
	frame[0] = RANGING_PHASE_FC_0;
	frame[1] = RANGING_PAHSE_FC_1;
	frame[2] = _sequenceNumber;
	frame[3] = PAN_ID_0;
	frame[4] = PAN_ID_1;
									/*Short-Destination address ANCHOR */
	frame[5] = _shortAdressAnchor[0];
	frame[6] = _shortAdressAnchor[1];
									/*Short-Source address TAG */
	frame[7] = _shortAddressTag[0];
	frame[8] = _shortAddressTag[1];

	*(frame + 9) = POLL_CONTROL;

	DWM1000::dwm_transmit(frame, _lenFrame, DWT_RESPONSE_EXPECTED);

	incrementSequenceNumber();
	//DWM1000::dwm_setrxtimeout(64000); // 64 ms
}

/*******************************************************************************************************************************************/
Response::Response() {
	_lenFrame = 10;
}

void Response::generateResponse(byte frame[], byte _shortAddressTag[], byte _shortAdressAnchor[]) {
	frame[0] = RANGING_PHASE_FC_0;
	frame[1] = RANGING_PAHSE_FC_1;
	frame[2] = _sequenceNumber;
	frame[3] = PAN_ID_0;
	frame[4] = PAN_ID_1;
									/*Short-Destaination address TAG */
	frame[5] = _shortAddressTag[0];
	frame[6] = _shortAddressTag[1];
									/*Short-Source address ANCHOR */
	frame[7] = _shortAdressAnchor[0];
	frame[8] = _shortAdressAnchor[1];
	
	frame[9] = RESPONSE_CONTROL;

	DWM1000::dwm_transmit(frame, _lenFrame, DWT_RESPONSE_EXPECTED);
	incrementSequenceNumber();

	//DWM1000::dwm_setrxtimeout(64000); // 64 ms
}

/*******************************************************************************************************************************************/
Final::Final() {
	_lenFrame = 20;
}

void Final::generateFinal(byte frame[], byte _shortAddressTag[], byte _shortAdressAnchor[], DWMTimeTag *timeTag) {
	frame[0] = RANGING_PHASE_FC_0;
	frame[1] = RANGING_PAHSE_FC_1;
	frame[2] = _sequenceNumber;
	frame[3] = PAN_ID_0;
	frame[5] = PAN_ID_1;
									/*Short-Destination address ANCHOR */
	frame[5] = _shortAdressAnchor[0];
	frame[6] = _shortAdressAnchor[1];
									/*Short-Source address TAG */
	frame[7] = _shortAddressTag[0];
	frame[8] = _shortAddressTag[1];

	frame[9] = FINAL_CONTROL;


	timeTag->setTimestampFinalTX();
	memcpy(frame + 10, timeTag->getRound1(), 5);
	memcpy(frame + 15, timeTag->getReply2(), 5);

	//after DWT_START_TX_DELAYED
	DWM1000::dwm_transmit(frame, _lenFrame, DWT_RESPONSE_DELAYED_EXPECTED);
	
	incrementSequenceNumber();
	//DWM1000::dwm_setrxtimeout(64000); // 64 ms
}



/*******************************************************************************************************************************************/

Report::Report() {
	_lenFrame = 14;
}

void Report::generateReport(byte frame[], byte _shortAddressTag[], byte _shortAdressAnchor[], DWMTimeAnchor *timeAnchor) {
	frame[0] = RANGING_PHASE_FC_0;
	frame[1] = RANGING_PAHSE_FC_1;
	frame[2] = _sequenceNumber;
	frame[3] = PAN_ID_0;
	frame[4] = PAN_ID_1;
	/*Short-Destaination address TAG */
	frame[5] = _shortAddressTag[0];
	frame[6] = _shortAddressTag[1];
	/*Short-Source address ANCHOR */
	frame[7] = _shortAdressAnchor[0];
	frame[8] = _shortAdressAnchor[1];

	frame[9] = REPORT_CONTROL;

	memcpy(frame + 10, timeAnchor->getTOF(), 5);

	//DWM1000::dwm_setrxtimeout(1000000); // 1 secondo
	DWM1000::dwm_transmit(frame, _lenFrame, DWT_RESPONSE_EXPECTED);

	incrementSequenceNumber();
	//DWM1000::dwm_setrxtimeout(64000); // 64 ms
}