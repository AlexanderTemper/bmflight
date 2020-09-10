#pragma once
#include "global.h"
#include "msp/msp.h"



void mspFcProcessReply(mspPacket_t *cmd);
mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply);

