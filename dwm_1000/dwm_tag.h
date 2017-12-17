#ifndef DWM_TAG_H
#define DWM_TAH_H

#include "dwm_device.h"
#include "position.h"

class DWMTag : public DWMDevice {
private:
    uint8 _shortAddressAnchor[2];

    uint32 _timerBlick;
    uint32 _timeElapsedBlick;
    uint32 _timeElapsedPoll;
    uint32 _timerPoll;
    uint32 oldGiro;
    uint32 newGiro;

    Blink blink;
    Poll poll;
    Final final;
    DWMTimeTag timeTag;

    Position * position;

    /* methods */
    void setLongAddress();

    void getShortAddressTag(byte[], byte[]);
    void getShortAddressAnchor(byte[], byte[]);

    void messageArrived();
    void messageSend();

public:
    DWMTag();
    DWMTag(Position *p);
    ~DWMTag();
    void startTag(int , int , int , byte *);
    void loop();
    void sendPoll();
    void sendBlink();

    void set_complete_measure_meter(void(Position::*ptrf)(double));
    void set_complete_measure_tof(void(Position::*ptrf)(byte *));
};
#endif