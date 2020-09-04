#pragma once

#include "global.h"

#include "fc/fc.h"

void init_EEPROM(void (*read_FuncPtr)(config_t* config),void (*write_FuncPtr)(config_t* config));
void read_EEPROM(config_t* config);
void write_EEPROM(config_t* config);
