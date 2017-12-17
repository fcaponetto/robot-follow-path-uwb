//
// Created by nando on 15/03/16.
//

#ifndef DWM_ROTATION_H
#define DWM_ROTATION_H

#include "dwm_tag.h"
#include <Wire.h>
#include "fit_values.h"

typedef struct{
    double distance;
    uint16 fixedAngle;
    uint16 realAngle;
} distance_t;


class Rotation : Position {
private:
    distance_t distances[MEASURES];

    uint8 indexDistances;
    uint8 giro;
    uint8 circle;

    uint32 _timerBlick;
    uint32 _timeElapsedBlick;
    uint32 _timeElapsedPoll;
    uint32 _timerPoll;
    uint32 oldGiro;
    uint32 newGiro;

    float _realAngle;
    float _fixedAngle;
    float min_angle;
    float _timeMeasure;
    float _timeCircle;
    double min_distance;

    static volatile bool completeRot;
    volatile bool _paired = true;

    DWMTag *tag;

    static void handleCompleteRotation();
    void handle_distance(double);
    void handle_time_of_flight(byte *);
    void completeRotation();
    void trasmitOnI2C();
    void calculateFitValues();

    void flushArray();
    void blickLED();

public:
    Rotation();
    ~Rotation();
    void loop();

    void setup(int, int, int, byte *);
};


#endif //DWM_ROTATION_H
