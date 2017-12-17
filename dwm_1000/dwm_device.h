#ifndef DWM_RANGING_H
#define DWM_RANGING_H

#define PRINT 0
#define TIME_TICK_BLINK 500  //ms
#define TIME_TICK_POLL 175  //ms
#define TIME_INCREMENT_ANGLE 4  //tempo_giro = 1400ms /360
#define TIME_REFRESH 500

#define MEASURES 8
#define ANGLE 45.0


#include "dwm_1000.h"
#include "dwm_mac.h"
#include "dwm_twr_types.h"
#include "dwm_time.h"
extern "C" {
#include "configuration.h"
};


class DWMDevice {
protected:
	byte _longAddress[8];
	byte _shortAddress[2];
	byte _panID[2];
	byte buffer[BUFFER_LEN];

	volatile byte _previousMessageIn;
    volatile byte _messageSend;
	volatile byte _nextMessageIn;
    volatile byte _messageArrived;

    static volatile bool _interruptSend;
    static volatile bool _interruptReceive;
    static volatile bool _interruptReceiveFailed;
    static volatile bool _interruptTimeout;

	uint32 length;
	bool led1;

	short detectMessageType(byte []);
	void setPanID();
    void setShortAddress(byte *);

    static void handleTimeoutReceive();
    static void handleSent();
    static void handleReceived();
    static void handleReceiveFailed();
public:
	DWMDevice();
	~DWMDevice();
    void init(int , int , int );
};
#endif