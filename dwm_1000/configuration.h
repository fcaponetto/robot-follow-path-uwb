//
// Created by Nando on 13/04/2016.
//

#ifndef DWM1000_CONFIGURATION_H_H
#define DWM1000_CONFIGURATION_H_H

#include "deca_types.h"
#include "deca_internals.h"

typedef struct
{
    uint8 channel;              // valid range is 1 to 11
    uint8 preambleCode ;        // 00 = use NS code, 1 to 24 selects code
    uint8 pulseRepFreq ;        // NOMINAL_4M, NOMINAL_16M, or NOMINAL_64M
    uint8 dataRate ;            // DATA_RATE_1 (110K), DATA_RATE_2 (850K), DATA_RATE_3 (6M81)
    uint8 preambleLength ;      // values expected are 64, (128), (256), (512), 1024, (2048), and 4096
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO;               //!< SFD timeout value (in symbols) e.g. preamble length (128) + SFD(8) - PAC + some margin ~ 135us... DWT_SFDTOC_DEF; //default value
} instance_config_t ;


extern  instance_config_t chConfig[9];

#endif //DWM1000_CONFIGURATION_H_H
