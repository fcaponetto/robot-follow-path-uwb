#ifndef DWM_TIME_H
#define DWM_TIME_H

#include "dwm_1000.h"
#include <math.h>

class DWMTime {
public:
};

class DWMTimeTag : public DWMTime{
public:
	void 	setTimestampPollTX();
	byte* getTimestampPollTX();
	void 	setTimestampResponseRX();
	byte* getTimestampResponseRX();
	void 	setTimestampFinalTX();
	byte* getTimestampFinalTX();
	byte* getRound1();
	byte* getReply2();
	
	void printInformation();
	double calculateTof();
	void setTof(byte []);
    byte* 	getTOF();
	
private:
	byte _timestampPollTX[5];		//big endian
	byte _timestampResponseRX[5];	//big endian
	byte _timestampFinalTX[5];	//little endian
	byte _round1[5];
	byte _reply2[5];
	
	byte _sysTimestamp[5]; //big endian
	byte _totalDelay[5];   //big endian

	byte _TOF[5];	// Big-Endian
	
	/* variable to compute operation */
	uint64 _pollTX;
	uint64 _responseRX;
	uint64 _finalTX;
	uint64 _delaySend;  //little endian
	uint64 _sysTime;	//little endian
	uint64 _timeOfFlight;
	double distance;

	
	void setTimeDelayCode();
};


/************************************************************************************/

class DWMTimeAnchor : public DWMTime{
public:
	void 	setTimestampPollRX();
	byte* 	getTimestampPollRX();
	void 	setTimestampResponseTX();
	byte* 	getTimestampResponseTX();
	void 	setTimestampFinalRX();
	byte*	getTimestampFinalRX();
	
	void 	getRound2();
	void 	getReply1();
	void 	setRound1(byte *);
	void 	setReply2(byte *);
	
	void 	printInformation();
	double 	calculateTOF();
    byte* 	getTOF();
private:
    const int offset = 0x4d;
	byte _timestampPollRX[5];
	byte _timestampResponseTX[5];
	byte _timestampFinalRX[5];
	
	byte  _reply1[5];
	byte  _reply2[5];
	byte  _round1[5];
	byte  _round2[5];
	byte  _TOF[5];
	
	/* All Little-Endian  for calculation tof */
	uint64 _pollRX;
	uint64 _responseTX;
	uint64 _finalRX;
	uint64 _reply1_uint;
	uint64 _reply2_uint;
	uint64 _round1_uint;
	uint64 _round2_uint;
	uint64 _tof;

	uint32 Rb, Da, Ra, Db ;
	uint64 RaRbxDaDb;
	uint64 RbyDb;
	uint64 RayDa;

	double _timeOfFlight;
	double distance;
	
	double convertdevicetimetosec(uint32 dt);
	
};

#endif
