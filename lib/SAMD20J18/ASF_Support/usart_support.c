#include "usart_support.h"

/*! SERCOM USART driver software instance structure, used to retain
 * software state information of the associated hardware module instance */
struct usart_module msp_usart_instance;

/*! USART receive callback flag (set after each USART reception) */
volatile bool usart_callback_receive_flag;

/*! USART receive callback flag (set after each USART transmission) */
volatile bool usart_callback_transmit_flag;

/*! USART Rx byte */
uint16_t usart_rx_byte;

volatile uint16_t rx_byte;

static serialPort_t *mspPort;

/************************************************************************/
/* Function Definitions                                                 */
/************************************************************************/
static void msp_usart_configure(void) {
    /* USART's configuration structure */
    struct usart_config config_usart;

    /* get USART configuration defaults */
    usart_get_config_defaults(&config_usart);

    /* set USART Baudrate*/
    config_usart.baudrate = UINT32_C(115200);
    /* Set USART GCLK */
    config_usart.generator_source = GCLK_GENERATOR_2;
    /* Se USART MUX setting */
    config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
    /* Configure pad 0 for unused */
    config_usart.pinmux_pad0 = PINMUX_UNUSED;
    /* Configure pad 1 for unused */
    config_usart.pinmux_pad1 = PINMUX_UNUSED;
    /* Configure pad 2 for tx */
    config_usart.pinmux_pad2 = PINMUX_PA20D_SERCOM3_PAD2;
    /* Configure pad 3 for rx */
    config_usart.pinmux_pad3 = PINMUX_PA21D_SERCOM3_PAD3;

    /* Initialize SERCOM3 as a USART module*/
    while (usart_init(&msp_usart_instance, SERCOM3, &config_usart) != STATUS_OK)
        ;

    /* Enable the USART module */
    usart_enable(&msp_usart_instance);
}

static void msp_usart_callback_receive(struct usart_module * const usart_module_ptr) {
//    uartDevice_t *uartdev = &mspDevice;
//    uartPort_t *s = &uartdev->port;
//
//    instance->rxBuffer[instance->rxBufferHead] = usart_rx_byte;
//    if (instance->rxBufferHead + 1 >= instance->rxBufferSize) {
//       instance->rxBufferHead = 0;
//    } else {
//       instance->rxBufferHead++;
//    }
    /* Initiate a new job to listen to USART port for a new byte */
    usart_read_job(&msp_usart_instance, &usart_rx_byte);
}

static void msp_usart_callback_transmit(struct usart_module * const usart_module_ptr) {
    mspSerialUartWriteCallback(mspPort);
}

static void msp_usart_configure_callbacks(void) {
    /* Configure USART receive callback */
    usart_register_callback(&msp_usart_instance, msp_usart_callback_receive, USART_CALLBACK_BUFFER_RECEIVED);
    usart_enable_callback(&msp_usart_instance, USART_CALLBACK_BUFFER_RECEIVED);

    /* Configure USART transmit callback */
    usart_register_callback(&msp_usart_instance, msp_usart_callback_transmit, USART_CALLBACK_BUFFER_TRANSMITTED);
    usart_callback_transmit_flag = true;
    usart_enable_callback(&msp_usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
}

void usart_initialize(serialPort_t *instance) {
    mspPort = instance;
    /* Initialize the variables */
    usart_rx_byte = 0;

    /* Configure the USART Module */
    msp_usart_configure();

    /* Configure USART callbacks */
    msp_usart_configure_callbacks();

    /* Enable the interrupt to receive the first byte */
    usart_read_job(&msp_usart_instance, &usart_rx_byte);
}

void mspSerialUartWriteCallback(serialPort_t *instance) {
    uint32_t fromWhere = instance->txBufferTail;
    // already running
    if (msp_usart_instance.remaining_tx_buffer_length > 0) {
        return;
    }
    // nothing to transmit
    if (instance->txBufferHead == instance->txBufferTail) {
        return;
    }
    mspPort = instance; //update ref
    // start transmitting
    if (instance->txBufferHead > instance->txBufferTail) {
        usart_write_buffer_job(&msp_usart_instance, (uint8_t *) &instance->txBuffer[fromWhere], instance->txBufferHead - instance->txBufferTail);
        instance->txBufferTail = instance->txBufferHead;
    } else {
        usart_write_buffer_job(&msp_usart_instance, (uint8_t *) &instance->txBuffer[fromWhere], instance->txBufferSize - instance->txBufferTail);
        instance->txBufferTail = 0;
    }
}
