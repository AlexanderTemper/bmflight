#include <pinmux.h>
#include <port.h>

#include "pin_support.h"

static uint8_t pinmux[LEDS_COUNT];

void samd20j18_set_pin(status_leds_e pinId, bool level) {
    port_pin_set_output_level(pinmux[pinId], level);
}

void samd20j18_pins_initialize(void) {
    struct port_config config_prt_pin;
    port_get_config_defaults(&config_prt_pin);
    config_prt_pin.direction = PORT_PIN_DIR_OUTPUT;
    port_pin_set_config(PIN_PA28, &config_prt_pin);
    port_pin_set_config(PIN_PB02, &config_prt_pin);
    port_pin_set_config(PIN_PA24, &config_prt_pin);

    //port_pin_set_output_level(PIN_PA24, true);

    pinmux[ARM_LED] = PIN_PA24; //LED_GRUEN
    pinmux[CALIBRATION_LED] = PIN_PA28; //LED_GELB
    pinmux[ERROR_LED] = PIN_PB02;//PIN_PA28; //LED_ROT
}
