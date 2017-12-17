#ifndef DWM_MAC_H
#define DWM_MAC_H

#include "dwm_1000.h"
#include "dwm_twr_types.h"
#include "dwm_time.h"
#include <math.h> 
#include "conversion.h"


class DWMMac {
public:

protected:
	uint16 _sequenceNumber;
	uint16 _lenFrame;
	void incrementSequenceNumber();
};

class Blink : public DWMMac {
public:
	Blink();
	void generateBlinkFrame(byte [], byte []);
private:
	
};

class RangingInit :public DWMMac {
public:
	RangingInit();
	void generateRangingInit(byte [], byte [], byte [], byte []);
private:
};

class Poll :public DWMMac {
public:
	Poll();
	void generatePoll(byte[], byte[], byte[]);
private:
};

class Response :public DWMMac {
public:
	Response();
	void generateResponse(byte[], byte[], byte[]);
private:
};

class Final :public DWMMac {
public:
	Final();
	void generateFinal(byte[], byte[], byte[], DWMTimeTag *time);
private:
	void setDelaySendFinalMessage(DWMTimeTag *);
	void setTime();

};

class Report : public  DWMMac{
public:
	Report();
	void generateReport(byte[], byte[], byte[],DWMTimeAnchor *);
private:
};

#endif