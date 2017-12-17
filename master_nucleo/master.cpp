#include "master.h"

Master::Master() : pc(USBTX, USBRX), 
                   int_tag1(PB_0), 
                   int_tag2(PA_12), 
                   int_ir_led(PF_1), 
                   dir_pin(PB_1), 
                   step_pin(PF_0), 
                   i2c(PB_7, PB_6),
                   addr1(0x02),
                   addr2(0x04),
                   rot(&dir_pin, &step_pin){
    complete_left = false;
    complete_right = false;
    reset_tag = false;
    ir_led = false;
    make_measure_left = true;
    make_measure_right = false;
    distance_left = 0;            //from TAG1 (left)
    distance_right = 0;            //from TAG2 (right)
    
    initSlave();
}

Master::~Master(){ }

void Master::complete_measure_left()
{
    complete_left = true;
}

void Master::complete_measure_right()
{
    complete_right = true;
}

void Master::handle_reset_tags(){
    reset_tag = true;
}

void Master::handle_ir_led(){
    ir_led = true;
}

void Master::initSlave(){
    int_tag1.rise(this, &Master::complete_measure_left);  // attach the address of the flip function to the rising edge
    int_tag2.rise(this, &Master::complete_measure_right);
    int_ir_led.fall(this, &Master::handle_ir_led);
    
    //TODO verificare perchè non funziona il reset
//    *cmd = 'r';
//    i2c.write(addr1, cmd, 1);
//    i2c.write(addr2, cmd, 1);
    *cmd = 's';
}

void Master::Array64BigEndianToInt64LittleEndian(char input[], uint32 length, uint64 *output)
{
    for (int j = length - 1; j >= 0; j--) {
        *output = (*output << 8) + input[j];
    }
}

void Master::read_measure(float *distance, const int addr){
    unsigned long long _timeOfFlight;
    char _TOF[5];
    memset(_TOF, 0x00, 5);
    i2c.read(addr, _TOF, 5); // request 5 bytes from slave device #1

    Array64BigEndianToInt64LittleEndian(_TOF, 5, &_timeOfFlight);
    _timeOfFlight -= 16384;  // Antenna delay

    *distance = (double) (((unsigned int)_timeOfFlight) * DISTANCE_OF_RADIO);
    //pc.printf("\%f\n",distance);
}


void Master::SetupInitialPosition(){
  while(!ir_led){
      rot.RotateDeg(5*10, 200);
  }
  ir_led = false;
}