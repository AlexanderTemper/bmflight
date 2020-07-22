#include "usart.h"
#include "usart_interrupt.h"
#include "usart_support.h"

static struct usart_module msp_usart_instance;

/*! USART Rx byte */
static uint16_t usart_rx_byte;

static bool blockWriteToHW = false;
static serialPort_t *usedPort;
/**
 * setup serial interface on for msp port
 */
static void msp_usart_configure(void) {
    /* USART's configuration structure */
    struct usart_config config_usart;

    /* get USART configuration defaults */
    usart_get_config_defaults(&config_usart);

    /* set USART Baudrate*/
    config_usart.baudrate = UINT32_C(115200);
    /* Set USART GCLK */
    config_usart.generator_source = GCLK_GENERATOR_3;
    /* Se USART MUX setting */
    config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
    /* Configure pad 0 for unused */
    config_usart.pinmux_pad0 = 0xFFFFFFFF;
    /* Configure pad 1 for unused */
    config_usart.pinmux_pad1 = 0xFFFFFFFF;
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
    usedPort->rxBuffer[usedPort->rxBufferHead] = usart_rx_byte;
    if (usedPort->rxBufferHead + 1 >= usedPort->rxBufferSize) {
        usedPort->rxBufferHead = 0;
    } else {
        usedPort->rxBufferHead++;
    }
    usart_read_job(&msp_usart_instance, &usart_rx_byte);
}

static void msp_usart_callback_transmit(struct usart_module * const usart_module_ptr) {

    //transmission already running
    if (msp_usart_instance.remaining_tx_buffer_length > 0) {
        return;
    }
    //nothing to do
    if (usedPort->txBufferHead == usedPort->txBufferTail) {
        return;
    }
    // start new write job
    if (usedPort->txBufferHead > usedPort->txBufferTail) {
        if (STATUS_OK
                == usart_write_buffer_job(&msp_usart_instance, &usedPort->txBuffer[usedPort->txBufferTail], usedPort->txBufferHead - usedPort->txBufferTail)) {
            usedPort->txBufferTail = usedPort->txBufferHead;
        }

    } else {
        if (STATUS_OK
                == usart_write_buffer_job(&msp_usart_instance, &usedPort->txBuffer[usedPort->txBufferTail], usedPort->txBufferSize - usedPort->txBufferTail)) {
            usedPort->txBufferTail = 0;
        }
    }
}

static void msp_usart_configure_callbacks(void) {
    /* Configure USART receive callback */
    usart_register_callback(&msp_usart_instance, msp_usart_callback_receive, USART_CALLBACK_BUFFER_RECEIVED);
    usart_enable_callback(&msp_usart_instance, USART_CALLBACK_BUFFER_RECEIVED);

    /* Configure USART transmit callback */
    usart_register_callback(&msp_usart_instance, msp_usart_callback_transmit, USART_CALLBACK_BUFFER_TRANSMITTED);
    usart_enable_callback(&msp_usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
}

void samd20j18_serialWrite(serialPort_t *instance, uint8_t ch) {
    system_interrupt_enter_critical_section();
    instance->txBuffer[instance->txBufferHead] = ch;

    if (instance->txBufferHead + 1 >= instance->txBufferSize) {
        instance->txBufferHead = 0;
    } else {
        instance->txBufferHead++;
    }
    system_interrupt_leave_critical_section();
    if (!blockWriteToHW) {
        msp_usart_callback_transmit(0);
    }
}

uint8_t samd20j18_serialRead(serialPort_t *instance) {
    uint8_t ch;
    system_interrupt_enter_critical_section();
    ch = instance->rxBuffer[instance->rxBufferTail];
    if (instance->rxBufferTail + 1 >= instance->rxBufferSize) {
        instance->rxBufferTail = 0;
    } else {
        instance->rxBufferTail++;
    }
    system_interrupt_leave_critical_section();
    return ch;
}

uint32_t samd20j18_serialTotalRxWaiting(const serialPort_t *instance) {
    uint32_t bytes = 0;
    system_interrupt_enter_critical_section();
    if (instance->rxBufferHead >= instance->rxBufferTail) {
        bytes =  instance->rxBufferHead - instance->rxBufferTail;
    } else {
        bytes =  instance->rxBufferSize + instance->rxBufferHead - instance->rxBufferTail;
    }
    system_interrupt_leave_critical_section();
    return bytes;
}

uint32_t samd20j18_serialTotalTxFree(const serialPort_t *instance) {
    uint32_t bytesUsed, bytesFree;
    system_interrupt_enter_critical_section();
    if (instance->txBufferHead >= instance->txBufferTail) {
        bytesUsed = instance->txBufferHead - instance->txBufferTail;
    } else {
        bytesUsed = instance->txBufferSize + instance->txBufferHead - instance->txBufferTail;
    }
    // -1 so the buffer can never get full
    bytesFree = (instance->txBufferSize - 1) - bytesUsed;
    system_interrupt_leave_critical_section();

    return bytesFree;
}

void samd20j18_beginWrite(serialPort_t *instance) {
    blockWriteToHW = true;
}

void samd20j18_endWrite(serialPort_t *instance) {
    blockWriteToHW = false;
    msp_usart_callback_transmit(0);
}

bool samd20j18_isSerialTransmitBufferEmpty(const serialPort_t *instance) {
    system_interrupt_enter_critical_section();
    bool empty = instance->txBufferHead == instance->txBufferTail;
    system_interrupt_leave_critical_section();
    return empty;
}

/**
 * setup serial interface
 * @param wp
 * @param rp
 */
void samd20j18_serial_initialize(serialPort_t *instance) {
    usart_rx_byte = 0;
    usedPort = instance;
    /* Configure the USART Module */
    msp_usart_configure();

    /* Configure USART callbacks */
    msp_usart_configure_callbacks();

    /* Enable the interrupt to receive the first byte */
    usart_read_job(&msp_usart_instance, &usart_rx_byte);
}

