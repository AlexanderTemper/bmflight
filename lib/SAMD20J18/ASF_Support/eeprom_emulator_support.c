#include "eeprom_emulator_support.h"

#include "eeprom.h"

// TODO Solve fuse Problem NVMCTRL_EEPROM_SIZE for now we don't save the config in flash => write the params to default_config
#include <string.h>
static config_t tempConfig; //for fake

//static void eeprom_emulator_configure(void) {
//    /* Setup EEPROM emulator service */
//    enum status_code error_code = eeprom_emulator_init();
//    if (error_code == STATUS_ERR_NO_MEMORY) {
//        while (true) {
//            /* No EEPROM section has been set in the device's fuses */
//        }
//    } else if (error_code != STATUS_OK) {
//        /* Erase the emulated EEPROM memory (assume it is unformatted or
//         * irrecoverably corrupt) */
//        eeprom_emulator_erase_memory();
//        eeprom_emulator_init();
//    }
//}

void eeprom_emulator_initialize(void) {
    //eeprom_emulator_configure();
}

void samd20j18_read_EEPROM(config_t* config) {
    memcpy(config, &tempConfig, sizeof(config_t));
    //eeprom_emulator_read_buffer(0, (uint8_t* const ) config, sizeof(config_t));
}

void samd20j18_write_EEPROM(config_t* config) {
    memcpy(&tempConfig, config, sizeof(config_t));
//    eeprom_emulator_write_buffer(0, (const uint8_t*) config, sizeof(config_t));
//    eeprom_emulator_commit_page_buffer();
//    samd20j18_read_EEPROM(config);
    //if (b == 1) blinkLED(15,20,1); TBD
}
