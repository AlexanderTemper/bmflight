#ifndef TC_SUPPORT_H_
#define TC_SUPPORT_H_

#include <stdint.h>

void tc_initialize(void);
void wait_for_msec(uint32_t msec);
uint32_t millis_samd20j18(void);
uint32_t micros_samd20j18(void);

#endif /* TC_SUPPORT_H_ */

