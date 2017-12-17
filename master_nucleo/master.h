#ifndef _MASTER_H_
#define _MASTER_H
#include "mbed.h"
#include "types.h"
#include "rotatewheel.h"

#define DISTANCE_OF_RADIO 0.0046917639786159f
#define BASE 0.81f

class Master{
protected:
  volatile bool complete_left;
  volatile bool complete_right;
  volatile bool reset_tag;
  volatile bool ir_led;
  bool make_measure_left;
  bool make_measure_right;
  float distance_left;            //from TAG1 (left)
  float distance_right;            //from TAG2 (right)
  const int addr1;         //TAG_LEFT
  const int addr2;         //TAG_RIGHT
  char cmd[1];

  Timer timer_reset_tag;
  I2C i2c ;                     //SDA=D4, SCL=D5
  Serial pc;                    //Tx, Rx
  InterruptIn int_tag1;         //D3
  InterruptIn int_tag2;         //D2
  InterruptIn int_ir_led;       //ir_led - D8
  DigitalOut dir_pin;           //D6
  DigitalOut step_pin;          //D7
  
  RotateWheel rot;
  
  void complete_measure_left();
  void complete_measure_right();
  void Array64BigEndianToInt64LittleEndian(char input[], uint32 length, uint64 *output);
  
  void read_measure(float *, const int);
  virtual void PrintInformation() = 0;
  virtual uint8 StateMachine() = 0;
  virtual void loop() = 0;
  void handle_reset_tags();
  void handle_ir_led();
  void SetupInitialPosition();
  void initSlave();
  
public:
  Master();
  ~Master();
};

#endif