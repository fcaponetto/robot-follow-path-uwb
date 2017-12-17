#ifndef DWM_ANCHOR_H
#define DWM_ANCHOR_H

#include "dwm_device.h"


class DWMAnchor : public DWMDevice {
private:
    bool _alive;
	uint32 _timer;
	uint32 timeElapsed;

    byte _longAddressTag[8];  //Big-Endian
    byte _shortAddressTag[2];
    RangingInit rangingInit;
    Response response;
    Report report;
    DWMTimeAnchor timeAnchor;

	void setLongAddress();
	void printInformation();

    /* Information from TAG */
	void get_long_address_TAG(byte []);
    void get_short_address_TAG(byte []);

	void setRound1(byte []);
	void setReply2(byte []);
	void calculateTOF();

    void messageArrived();
    void messageSend();
public:
    DWMAnchor();
    ~DWMAnchor();
	void startAnchor(int , int , int, byte *);
	void loop();

	double convertdevicetimetosec(uint32 dt);
};
#endif