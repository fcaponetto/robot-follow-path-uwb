//
// Created by Nando on 13/04/2016.
//
#include "configuration.h"

//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
instance_config_t chConfig[9] ={
        //mode 0 - S1: 7 off, 6 off, 5 off
        {
                2,              // channel
                DWT_PRF_16M,    // prf
                DWT_BR_110K,    // datarate
                3,             // preambleCode
                DWT_PLEN_1024,  // preambleLength
                DWT_PAC32,      // pacSize
                1,       // non-standard SFD
                (1025 + 64 - 32) //SFD timeout
        },
        //mode 1
        {
                2,              // channel
                DWT_PRF_16M,    // prf
                DWT_BR_6M8,    // datarate
                3,             // preambleCode
                DWT_PLEN_128,   // preambleLength
                DWT_PAC8,       // pacSize
                0,       // non-standard SFD
                (129 + 8 - 8) //SFD timeout
        },
        //mode 2
        {
                2,              // channel
                DWT_PRF_64M,    // prf
                DWT_BR_110K,    // datarate
                9,             // preambleCode
                DWT_PLEN_1024,  // preambleLength
                DWT_PAC32,      // pacSize
                1,       // non-standard SFD
                (1025 + 64 - 32) //SFD timeout
        },
        //mode 3
        {
                2,              // channel
                DWT_PRF_64M,    // prf
                DWT_BR_6M8,    // datarate
                9,             // preambleCode
                DWT_PLEN_128,   // preambleLength
                DWT_PAC8,       // pacSize
                0,       // non-standard SFD
                (129 + 8 - 8) //SFD timeout
        },
        //mode 4
        {
                5,              // channel
                DWT_PRF_16M,    // prf
                DWT_BR_110K,    // datarate
                3,             // preambleCode
                DWT_PLEN_1024,  // preambleLength
                DWT_PAC32,      // pacSize
                1,       // non-standard SFD
                (1025 + 64 - 32) //SFD timeout
        },
        //mode 5
        {
                5,              // channel - try this configuration with channel 7
                DWT_PRF_16M,    // prf
                DWT_BR_6M8,    // datarate
                3,             // preambleCode
                DWT_PLEN_128,   // preambleLength
                DWT_PAC8,       // pacSize
                0,       // non-standard SFD
                (129 + 8 - 8) //SFD timeout
        },
        //mode 6
        {
                5,              // channel
                DWT_PRF_64M,    // prf
                DWT_BR_110K,    // datarate
                9,             // preambleCode
                DWT_PLEN_1024,  // preambleLength
                DWT_PAC32,      // pacSize
                1,       // non-standard SFD
                (1025 + 64 - 32) //SFD timeout
        },
        //mode 7
        {
                5,              // channel
                DWT_PRF_64M,    // prf
                DWT_BR_6M8,    // datarate
                9,             // preambleCode
                DWT_PLEN_128,   // preambleLength
                DWT_PAC8,       // pacSize
                0,       // non-standard SFD
                (129 + 8 - 8) //SFD timeout
        },
        //mode 8
        {
                7,              // channel - try this configuration with channel 7
                DWT_PRF_16M,                    // prf
                DWT_BR_6M8,                     // datarate
                PREAMBLE_CODE_16MHZ_4,          // preambleCode
                DWT_PLEN_128,                   // preambleLength
                DWT_PAC8,                       // pacSize
                0,                              // non-standard SFD
                0                               //SFD timeout
        }
};

