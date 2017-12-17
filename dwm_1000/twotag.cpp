//
// Created by nando on 15/03/16.
//

#include "twotag.h"

TwoTag* TwoTag::classToUse = 0;

TwoTag::TwoTag(int irq, int ss, int rst, byte * short_address, uint8 addess_i2c)
        : irq(irq) ,ss(ss), rst(rst), short_address(short_address), addess_i2c(addess_i2c) {
    tag = new DWMTag(this);
    classToUse = this;
}

TwoTag::~TwoTag() {}

void TwoTag::setup() {
    tag->startTag(irq, ss, rst, short_address);
    //tag->set_complete_measure_meter(&Position::handle_distance);
    tag->set_complete_measure_tof(&Position::handle_time_of_flight);

    pinMode(7, OUTPUT);
    digitalWrite(7, LOW);

    Wire.begin(addess_i2c);
    Wire.onReceive(handle_on_receive_i2c);      //register event
    Wire.onRequest(handle_on_request_i2c);      //register event
}

void TwoTag::handle_on_receive_i2c(int numBytes) {
#if PRINT
    Serial.println("Received i2c");
#endif

    while(Wire.available() > 0) {
        char c = Wire.read();
        if (c == 'r') {
#if PRINT
            Serial.println("RESET");
#endif
            classToUse->resetTag();
        }
        else if(c == 's')
            classToUse->compute_measure();
    }
}

void TwoTag::handle_on_request_i2c() {
    classToUse->send_measure();
    while(Wire.available()>0) {
        Wire.read();
    }
}

void TwoTag::resetTag() {
    if(tag)
        delete tag;
    tag = new DWMTag(this);
    setup();
}

void TwoTag::compute_measure() {
    tag->sendPoll();
}

/* Sent Measure to I2C to Nucleo Board */
void TwoTag::send_measure() {
    Wire.write(_time_of_flight, 5); // respond with message of 5 bytes
    digitalWrite(7, LOW);  //reset interrupt to nucleo
}

void TwoTag::handle_distance(const double m) {
    //Serial.print("Complete measure meter: "); Serial.println(m, 4);
    while(Wire.available()>0) {
        Wire.read();
    }
    digitalWrite(7, HIGH); //interrupt to nucleo
}

void TwoTag::handle_time_of_flight(byte * tof) {

    //Serial.println("Complete measure tof :");
    while(Wire.available()>0) {
        Wire.read();
    }
    _time_of_flight = tof;

    digitalWrite(7, HIGH);  //interrupt to Nucleo Board
}

void TwoTag::loop(){
    tag->loop();
}