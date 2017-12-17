#include "dwm_time.h"
#include "dwm_device.h"

void DWMTimeTag::setTimestampPollTX() {
	//DWM1000::dwm_readTXTimestamp(_timestampPollTX);
	//Conversion::Array64BigEndianToInt64LittleEndian(_timestampPollTX, 5, &_pollTX);
}

byte * DWMTimeTag::getTimestampPollTX() {
	return _timestampPollTX;
}

void DWMTimeTag::setTimestampResponseRX() {
	DWM1000::dwm_readRXTimestamp(_timestampResponseRX);
}

byte * DWMTimeTag::getTimestampResponseRX() {
	return _timestampResponseRX;
}

// FinalTX = SysTime + DelayCode
void DWMTimeTag::setTimestampFinalTX() {	
	setTimeDelayCode();

	/* Read the actual time */
	//DWM1000::dwm_readSysTime(_sysTimestamp);
	//Conversion::Array64BigEndianToInt64LittleEndian(_sysTimestamp, 5, &_sysTime);

	//uint64 correction;

	//correction = _sysTime - _responseRX;
	Conversion::Array64BigEndianToInt64LittleEndian(_timestampResponseRX, 5, &_responseRX);
	
	/* Add the delay of code */
	//Conversion::Int64LittleEndianToArray64BigEndian(_totalDelay, 5, _sysTime + _delaySend);
	Conversion::Int64LittleEndianToArray64BigEndian(_timestampFinalTX, 5, _responseRX + _delaySend);
	
	/* The last 9 bit have to be equal to zero */
	_totalDelay[0] = 0;
	_totalDelay[1] &= 254;
	
	DWM1000::dwm_setDelayedTRXTime(_timestampFinalTX); //_totalDelay is Big-Endian

	//Conversion::Int64LittleEndianToArray64BigEndian(_timestampFinalTX, 5, _sysTime + _delaySend  + ((uint16) ((DWT_PRF_16M_RFDLY/ 2.0) * 1e-9/DWT_TIME_UNITS)));
	//Conversion::Int64LittleEndianToArray64BigEndian(_timestampFinalTX, 5, _responseRX + _delaySend  + 16384);
}

byte * DWMTimeTag::getTimestampFinalTX(){
	return _timestampFinalTX;
}


//Round1 Big-Endian
byte* DWMTimeTag::getRound1(){
	DWM1000::dwm_readTXTimestamp(_timestampPollTX);
	Conversion::Array64BigEndianToInt64LittleEndian(_timestampPollTX, 5, &_pollTX);
	//Conversion::Array64BigEndianToInt64LittleEndian(_timestampResponseRX, 5, &_responseRX);


	Conversion::Int64LittleEndianToArray64BigEndian(_round1, 5, _responseRX - _pollTX);


	return _round1;
}

//reply2 Big-Endian
byte* DWMTimeTag::getReply2() {
	Conversion::Array64BigEndianToInt64LittleEndian(_timestampFinalTX, 5, &_finalTX); // same Little-Endian

	//Conversion::Int64LittleEndianToArray64BigEndian(_reply2, 5,  _finalTX  - _responseRX);
	Conversion::Int64LittleEndianToArray64BigEndian(_reply2, 5,  _delaySend  + 16384);

	return _reply2;
}

void DWMTimeTag::setTimeDelayCode(){
	float timeUs = 30000; //us  //TODO perchè?
	_delaySend = (uint64) timeUs * TIME_RES_INV;
}

void DWMTimeTag::setTof(byte frame[]){
	memcpy(_TOF, frame + 10, 5);
}

double DWMTimeTag::calculateTof(){

	_timeOfFlight = 0;
	Conversion::Array64BigEndianToInt64LittleEndian(_TOF, 5, &_timeOfFlight);

    _timeOfFlight -= 16384;  // Antenna delay
	distance = (double) _timeOfFlight * DISTANCE_OF_RADIO;

#if PRINT
        if (distance > 0) {
            Serial.println(distance, 3); //Serial.println(" m");
        }
#endif
    return distance;
}

byte* DWMTimeTag::getTOF(){
    return _TOF;
}



void DWMTimeTag::printInformation(){
	
		/*******************************************************************************************************/
		Serial.println("****************************");
		Serial.print("ResponseRX:\t");
		Conversion::Print5BytesBigEndian(_timestampResponseRX, true);
		//Serial.print("\t");Serial.print((uint32)_responseRX, HEX);

		Serial.print("PollTX:\t\t");
		Conversion::Print5BytesBigEndian(_timestampPollTX, true);
		//Serial.print("\t");Serial.print((uint32)_pollTX, HEX);
		
		Serial.print("Round1: \t");
		Conversion::Print5BytesBigEndian(_round1, true);

		/*******************************************************************************************************/
		Serial.println("****************************"); 
		Serial.print("FinalTX:\t");
		Conversion::Print5BytesBigEndian(_timestampFinalTX, true);
		//Serial.print("\t");Serial.print((uint32)_finalTX, HEX);

		Serial.print("ResponseRX:\t");
		Conversion::Print5BytesBigEndian(_timestampResponseRX, true);
		//Serial.print("\t");Serial.print((uint32)_responseRX, HEX);
		
		Serial.print("Reply2: \t");
		Conversion::Print5BytesBigEndian(_reply2, true);
		Serial.println("****************************");

		/*******************************************************************************************************/
		Serial.print("SysTime:\t");
		Conversion::Print5BytesBigEndian(_sysTimestamp, false); Conversion::PrintSeconds(_sysTimestamp);
		
		Serial.print("TotDelay:\t");
		Conversion::Print5BytesBigEndian(_totalDelay, false); Conversion::PrintSeconds(_totalDelay);
		Serial.println("****************************"); 
}





/***************************************************************************************************************************************************************/






void DWMTimeAnchor::setTimestampPollRX(){
	DWM1000::dwm_readRXTimestamp(_timestampPollRX);
}

byte* DWMTimeAnchor::getTimestampPollRX(){
	return _timestampPollRX;
}

void DWMTimeAnchor::setTimestampResponseTX(){
	DWM1000::dwm_readTXTimestamp(_timestampResponseTX);
}

byte* DWMTimeAnchor::getTimestampResponseTX(){
	return _timestampResponseTX;
}

void DWMTimeAnchor::setTimestampFinalRX(){
	DWM1000::dwm_readRXTimestamp(_timestampFinalRX);
}

byte* DWMTimeAnchor::getTimestampFinalRX(){
	return _timestampFinalRX;
}

void DWMTimeAnchor::setRound1(byte *r){
	memcpy(_round1, r + 10, 5);
}

void DWMTimeAnchor::setReply2(byte *r){
	memcpy(_reply2, r + 15, 5);
}

//round2 Big-Endian
void DWMTimeAnchor::getRound2(){
	Conversion::Array64BigEndianToInt64LittleEndian(_timestampFinalRX, 5, &_finalRX);
	Conversion::Array64BigEndianToInt64LittleEndian(_timestampResponseTX, 5, &_responseTX);

	Conversion::Int64LittleEndianToArray64BigEndian(_round2, 5, _finalRX  - _responseTX);
}

//Note call before getRound2
//reply1 Big-Endian
void DWMTimeAnchor::getReply1(){
	Conversion::Array64BigEndianToInt64LittleEndian(_timestampPollRX, 5, &_pollRX);

	Conversion::Int64LittleEndianToArray64BigEndian(_reply1, 5, _responseTX - _pollRX);
}

void DWMTimeAnchor::printInformation(){
}

double DWMTimeAnchor::calculateTOF(){
	getRound2();  	//Big-Endian
	getReply1();	//Big-Endian
		
	Conversion::Array64BigEndianToInt64LittleEndian(_reply1, 5, &_reply1_uint);
	Conversion::Array64BigEndianToInt64LittleEndian(_reply2, 5, &_reply2_uint);
	Conversion::Array64BigEndianToInt64LittleEndian(_round1, 5, &_round1_uint);
	Conversion::Array64BigEndianToInt64LittleEndian(_round2, 5, &_round2_uint);
	
 	Ra = (uint32) (_round1_uint & MASK_40BIT);
	Db = (uint32) (_reply1_uint & MASK_40BIT);

	// response final round trip delay time is calculated as
	// (tagFinalRxTime - anchorRespTxTime) - (tagFinalTxTime - anchorRespRxTime)
	Rb = (uint32)(_round2_uint & MASK_40BIT);
	Da = (uint32)(_reply2_uint & MASK_40BIT);
	
 	RaRbxDaDb = (((uint64)Ra))*(((uint64)Rb)) - (((uint64)Da))*(((uint64)Db));
	
	RbyDb = ((uint64)Rb + (uint64)Db);

	RayDa = ((uint64)Ra + (uint64)Da);

	//time-of-flight
    _tof = _timeOfFlight =(double) ( RaRbxDaDb/(((double)RbyDb) + ((double)RayDa)) );
    _timeOfFlight -= 16384.0;
	
	//_timeOfFlight = convertdevicetimetosec((uint32)tof1);
	//distance = distance - dwm_getrangebias(5, (float) distance, DWT_PRF_16M);
	distance = _timeOfFlight * DISTANCE_OF_RADIO;
	//distance -= offset;

	//_tof = (uint64) ( RaRbxDaDb/(RbyDb + RayDa) );
	Conversion::Int64LittleEndianToArray64BigEndian(_TOF, 8, _tof);

	//_timeOfFlight = (float)(((((_round1 * _round2) - (_reply1 * _reply2))) / (_round1 - _round2 + _reply1 - _reply2)) * SPEED_OF_LIGHT);
	
	//_timeOfFlight = (double) (((_round1 - _reply2 + _round2 - _reply1) / 4) * SPEED_OF_LIGHT);
	
	
#if DEBUG
		Serial.println("****************************");
		Serial.print("Round1: \t");
		Conversion::Print5BytesBigEndian(_round1, false); Serial.print((uint32)_round1_uint); Serial.println("");
		//Serial.print("\t");Serial.print((uint32)_round1, HEX);
		//Serial.println("");

		Serial.print("Reply2: \t");
		Conversion::Print5BytesBigEndian(_reply2, false); Serial.print((uint32)_reply2_uint); Serial.println("");
		//Serial.print("\t");Serial.print((uint32)_reply2, HEX);
		//Serial.println("");
		Serial.println("****************************");

		/**********************************************************************************************/

		Serial.print("FinalRX:\t");
		Conversion::Print5BytesBigEndian(_timestampFinalRX, true);
		//Serial.print("\t");Serial.print((uint32)_finalRX, HEX);

		Serial.print("ResponTX:\t");
		Conversion::Print5BytesBigEndian(_timestampResponseTX, true);
		//Serial.print("\t");Serial.print((uint32)_responseTX, HEX);

		Serial.print("Round2: \t");
		Conversion::Print5BytesBigEndian(_round2, false);	Serial.print((uint32)_round2_uint); Serial.println("");
		//Serial.print("\t");Serial.print((uint32)_round2, HEX);
		//Serial.println("");
		Serial.println("****************************");

		/***********************************************************************************************/

		Serial.print("ResponTX:\t");
		Conversion::Print5BytesBigEndian(_timestampResponseTX, true);
		//Serial.print("\t");Serial.print((uint32)_responseTX, HEX);

		Serial.print("PollRX: \t");
		Conversion::Print5BytesBigEndian(_timestampPollRX, true);
		//Serial.print("\t");Serial.print((uint32)_pollRX, HEX);

		Serial.print("Reply1: \t");
		Conversion::Print5BytesBigEndian(_reply1, false);	Serial.print((uint32)_reply1_uint); Serial.println("");
		//Serial.print("\t");Serial.print((uint32)_reply1, HEX);
		//Serial.println("");
		Serial.println("****************************");

		byte temp[8];
		Conversion::Int64LittleEndianToArray64BigEndian(temp, 8, RaRbxDaDb);

		uint64 tof = (int64) ( RaRbxDaDb/(RbyDb + RayDa) );
		Conversion::Int64LittleEndianToArray64BigEndian(temp, 8, tof);
		Serial.print("TOF:\t"); 
		for(int i = 7; i>= 0; i--){
			Serial.print((uint32)temp[i], HEX); Serial.print("\t");
		}
		Serial.println("");
		Serial.print("TOF:\t"); Serial.print(_timeOfFlight, 10); Serial.println("");
#endif

//	if(distance > 0) {
//		Serial.print("\t Distance:\t");
//		Serial.print(distance, 3);
//		Serial.println(" m");
//	}
	return distance;
}

double DWMTimeAnchor::convertdevicetimetosec(uint32 dt)
{
    double f = 0;

    f =  dt * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}

byte* DWMTimeAnchor::getTOF(){
	return _TOF;
}