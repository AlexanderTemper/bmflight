#include "eeprom/eeprom_emulation.h"
static void (*read_FuncPointer)(config_t* config);
static void (*write_FuncPointer)(config_t* config);

void init_EEPROM(void (*read_FuncPtr)(config_t* config), void (*write_FuncPtr)(config_t* config)) {
    read_FuncPointer = read_FuncPtr;
    write_FuncPointer = write_FuncPtr;
}
void read_EEPROM(config_t* config) {
    read_FuncPointer(config);
}
void write_EEPROM(config_t* config) {
    write_FuncPointer(config);
}
