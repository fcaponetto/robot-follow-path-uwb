//
// Created by nando on 17/03/16.
//

#ifndef DWM_POSITION_H
#define DWM_POSITION_H

#include "position.h"

class Position {
public:
    virtual void handle_distance(const double) = 0;
    virtual void handle_time_of_flight(byte *) = 0;
};


#endif //DWM_POSITION_H
