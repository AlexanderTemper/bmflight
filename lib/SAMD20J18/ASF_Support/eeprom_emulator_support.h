#ifndef EEPROM_EMULATOR_SUPPORT_H_
#define EEPROM_EMULATOR_SUPPORT_H_

#include "fc/fc.h"

void eeprom_emulator_initialize(void);
void samd20j18_read_EEPROM(config_t* config);
void samd20j18_write_EEPROM(config_t* config);
#endif /* EEPROM_EMULATOR_SUPPORT_H_ */
