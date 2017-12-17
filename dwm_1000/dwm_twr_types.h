#ifndef DWM_TWR_TYPES_H
#define DWM_TWR_TYPES_H

// messages used in the ranging protocol
#define NUL -1
#define BLINK 0
#define RANGING_INIT 1
#define POLL 2
#define RESPONSE 3
#define FINAL 4
#define REPORT 5

#define BLINK_FC 0xC5
#define RANGING_FC_0 0x41
#define RANGING_FC_1 0x8C
#define RANGING_PHASE_FC_0 0x41
#define RANGING_PAHSE_FC_1 0x88
#define PAN_ID_0 0xCA
#define PAN_ID_1 0xDE

#define RANGING_INIT_CONTROL 0x20
#define POLL_CONTROL 0x61
#define RESPONSE_CONTROL 0x50
#define FINAL_CONTROL 0x69
#define REPORT_CONTROL 0x6A

#define DEFAULT_REPLY_DELAY 7000

#endif