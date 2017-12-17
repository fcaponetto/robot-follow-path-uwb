//
// Created by nando on 15/03/16.
//
#include "rotation.h"
#define PRINT_ROTATION 0

volatile bool Rotation::completeRot = false;
bool led2 = true;

Rotation::Rotation() {
    tag = new DWMTag(this);
}

Rotation::~Rotation() { }

void Rotation::setup(int irq, int ss, int rst, byte * short_address) {
    tag->startTag(irq, ss, rst, short_address);
    tag->set_complete_measure_meter(&Position::handle_distance);

    pinMode(3, INPUT);
    pinMode(4, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(3), handleCompleteRotation, FALLING);

    Wire.begin(); // join i2c bus (address optional for master)
}

void Rotation::handleCompleteRotation() {
    completeRot = true;
    //tone(8, 523, 40);
}

void Rotation::handle_time_of_flight(byte *) {

}

void Rotation::handle_distance(const double measure) {
    //Serial.println(measure);
    if (_paired) {
        /* Store the values calculated of angle and distanza */
        distances[indexDistances].distance = measure;
        distances[indexDistances].fixedAngle = _fixedAngle;
        distances[indexDistances].realAngle = _realAngle;

        if (indexDistances < MEASURES - 1) indexDistances++;
//        else
//            calculateFitValues();
#if PRINT
                        Serial.print("Index: ");
                        Serial.println(indexDistances);
                        Serial.print("Angle: ");
#endif
    }

    if (!_paired) _paired = true;
}

void Rotation::completeRotation() {
    if (circle < 5) circle++;

    if (_paired) tag->sendPoll();

    if (circle == 4) oldGiro = millis();

    if (circle == 5) {
        //tone(8, 523, 40);
        digitalWrite(7, led2);
        led2 = !led2;
        newGiro = millis();
#if PRINT_ROTATION
        Serial.print("\t\tTime rotation: "); Serial.println(newGiro - oldGiro);
#endif
        _timeCircle = (newGiro - oldGiro);
        _timeMeasure = ceil((newGiro - oldGiro) / 360);
        oldGiro = newGiro;
        calculateFitValues();
    }

    _realAngle = 0;
    _fixedAngle = ANGLE;
    indexDistances = 0;
    flushArray();
}

/* Correlation: https://en.wikipedia.org/wiki/Correlation_and_dependence */
void Rotation::calculateFitValues() {

    float mean_precalculated_value = 3.000;
    float mean_new_value = 0.0;
    byte i, j, t;

    for (i = 0; i < MEASURES; i++) {
        mean_new_value += distances[i].distance;
    }
    mean_new_value /= MEASURES;

    float correaltion = 0.0;
    float covarianza;
    float deviazioneStandard;
    float deviazioneStandard2;
    float temp_angle = ANGLE_FIT;
    min_distance = 1000.00;

    for (i = 0, t = 0; i < 36; i++, t += 8) {
        covarianza = 0.0;
        deviazioneStandard = 0.0;
        deviazioneStandard2 = 0.0;
        for (j = 0; j < MEASURES; j++) {

            /* Calculate min-distance one time */
            if (i == 0) if (distances[i].distance < min_distance)
                min_distance = distances[i].distance;

            covarianza += (distances[j].distance - mean_new_value) *
                          (pgm_read_float_near(valuesFit + j + t) - mean_precalculated_value);
            deviazioneStandard += pow((distances[j].distance - mean_new_value), 2);
            deviazioneStandard2 += pow((pgm_read_float_near(valuesFit + j + t) - mean_precalculated_value), 2);
        }


        deviazioneStandard *= deviazioneStandard2;
        deviazioneStandard = sqrt(deviazioneStandard);

        /* if new correlation is greater than older, it's uptaded */
        if ((covarianza / deviazioneStandard) > correaltion) {
            correaltion = covarianza / deviazioneStandard;
            //Serial.print("Correlazione: "); Serial.println(correaltion, 3);
            min_angle = temp_angle;  /* Store the angle where the correlation in less than the previosus */
        }

        temp_angle += ANGLE_FIT;
    }

    trasmitOnI2C();
}

void Rotation::trasmitOnI2C() {

    /*Serial.print("Angle: "); */ Serial.println(min_angle);
    /*Serial.print("Distance: "); */ Serial.println(min_distance);
//    Conversion::Int64LittleEndianToArray64BigEndian(tof, 4, min_distance / DISTANCE_OF_RADIO);
//    Conversion::Int64LittleEndianToArray64BigEndian(angle, 4, min_angle);
//    Wire.beginTransmission(8); 					// transmit to device #8
//    Wire.write(tof, 4);
//    Wire.write(angle, 4);
//    Wire.endTransmission();    				 // stop transmitting

    //tone(8, min_distance * 200, 50);
    flushArray();

}

void Rotation::flushArray() {
    for (int i = 0; i < MEASURES; i++) {
        distances[i].distance = 0;
        distances[i].fixedAngle = 0;
        distances[i].realAngle = 0;
    }
}

void Rotation::blickLED() {
    digitalWrite(4, HIGH);   // turn the LED on (HIGH is the voltage level)     // wait for a second
    digitalWrite(4, LOW);    // turn the LED off by making the voltage LOW
}


void Rotation::loop() {
    if (circle == 5) {
        if (!_paired) /* if it's not in ranging phase, send blink */
        {
            _timeElapsedBlick = millis();
            if (_timeElapsedBlick - _timerBlick >= TIME_TICK_BLINK) {
                _timerBlick = _timeElapsedBlick;
                tag->sendBlink();
                return;
            }
        }

        if (_paired) {
            _timeElapsedPoll = millis();
            if (_timeElapsedPoll - _timerPoll >= _timeMeasure) {
                //Serial.println(_timeElapsedPoll - _timerPoll);
                _realAngle += (360 * (_timeElapsedPoll - _timerPoll)) / _timeCircle;
                _timerPoll = _timeElapsedPoll;

                if (_realAngle >= _fixedAngle && (_fixedAngle < 360)) {
                    _fixedAngle += ANGLE;
                    tag->sendPoll();
                }
            }
        }
    }

    if (completeRot) {
        completeRot = false;
        completeRotation();
    }

    tag->loop();
}
