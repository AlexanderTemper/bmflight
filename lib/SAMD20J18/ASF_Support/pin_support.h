#ifndef PIN_SUPPORT_H_
#define PIN_SUPPORT_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "io/pin.h"

void samd20j18_set_pin(status_leds_e pinId, bool level);
void samd20j18_pins_initialize(void);

#endif /* PIN_SUPPORT_H_ */
