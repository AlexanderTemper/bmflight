#pragma once

#include "msp/msp.h"


bool mspProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst);
void mspFcProcessReply(mspPacket_t *cmd);


mspResult_e mspProcessInCommand(uint8_t cmdMSP, sbuf_t *src);
mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply);

