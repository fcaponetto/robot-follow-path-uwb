//
// Created by nando on 15/03/16.
//

#ifndef DWM_TWOTAG_H
#define DWM_TWOTAG_H

#include "dwm_tag.h"
#include "position.h"
#include "Wire.h"

class TwoTag : Position {
private:
    DWMTag *tag;
    byte * _time_of_flight;
    double _meter;
    int irq;
    int ss;
    int rst;
    byte * short_address;
    uint8 addess_i2c;
public:
    static TwoTag * classToUse;
    TwoTag(int, int, int, byte *, uint8);
    ~TwoTag();

    void setup();
    void handle_distance(const double);  //derivated
    void handle_time_of_flight(byte *);
    void compute_measure();
    void send_measure();
    void resetTag();
    static void handle_on_receive_i2c(int);
    static void handle_on_request_i2c();

    void loop();
};


#endif //DWM_TWOTAG_H
