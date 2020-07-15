#ifndef usart_support_h_
#define usart_support_h_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include "usart.h"
#include "usart_interrupt.h"

#include "drivers/serial.h"

/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/

/*! value of 115200 that is used to set USART baud rate */
#define USART_BAUDRATE_115200	UINT32_C(115200)
/*! value of 9600 that is used to set USART baud rate */
#define USART_BAUDRATE_9600	UINT32_C(9600)
/*! loaded onto USART baud rate register initially */
#define USART_BAUDRATE			USART_BAUDRATE_115200

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

/*! SERCOM USART driver software instance structure, used to retain
* software state information of the associated hardware module instance */
extern struct usart_module usart_instance;
/*! USART receive callback flag (set after each USART reception) */
extern volatile bool usart_callback_receive_flag;

/*! USART receive callback flag (set after each USART transmission) */
extern volatile bool usart_callback_transmit_flag;

/*! USART Rx byte */
extern uint16_t usart_rx_byte;

void usart_initialize(serialPort_t *instance);

void mspSerialUartWriteCallback(serialPort_t *instance);

#endif /* usart_support_h_ */
