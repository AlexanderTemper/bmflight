#include "usart.h"
#include "usart_interrupt.h"
#include "drivers/SAMD20J18/serial.h"

/*! value of 115200 that is used to set USART baud rate */
#define USART_BAUDRATE_115200   UINT32_C(115200)
/*! value of 9600 that is used to set USART baud rate */
#define USART_BAUDRATE_9600     UINT32_C(9600)
/*! loaded onto USART baud rate register initially */
#define USART_BAUDRATE          USART_BAUDRATE_115200

static struct usart_module msp_usart_instance;

/*! USART Rx byte */
static uint16_t usart_rx_byte;
static writeCallbackFuncPtr writeCallback;
static readCallbackFuncPtr readCallback;
//static volatile uint16_t rx_byte;

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
    readCallback(usart_rx_byte);
    usart_read_job(&msp_usart_instance, &usart_rx_byte);
}

static void msp_usart_callback_transmit(struct usart_module * const usart_module_ptr) {
    writeCallback();
}

static void msp_usart_configure_callbacks(void) {
    /* Configure USART receive callback */
    usart_register_callback(&msp_usart_instance, msp_usart_callback_receive, USART_CALLBACK_BUFFER_RECEIVED);
    usart_enable_callback(&msp_usart_instance, USART_CALLBACK_BUFFER_RECEIVED);

    /* Configure USART transmit callback */
    usart_register_callback(&msp_usart_instance, msp_usart_callback_transmit, USART_CALLBACK_BUFFER_TRANSMITTED);
    usart_enable_callback(&msp_usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
}

/**
 * start new transmission or return with false if already running
 * @param tx_data
 * @param length
 * @return
 */
bool samd20j18_serial_write(uint8_t *tx_data, uint16_t length) {
    //transmission already running
    if (msp_usart_instance.remaining_tx_buffer_length > 0) {
        return false;
    }
    return STATUS_OK == usart_write_buffer_job(&msp_usart_instance, tx_data, length);
}

/**
 * setup serial interface
 * @param wp
 * @param rp
 */
void samd20j18_serial_initialize(writeCallbackFuncPtr wp,readCallbackFuncPtr rp) {
    usart_rx_byte = 0;
    writeCallback = wp;
    readCallback = rp;
    /* Configure the USART Module */
    msp_usart_configure();

    /* Configure USART callbacks */
    msp_usart_configure_callbacks();

    /* Enable the interrupt to receive the first byte */
    usart_read_job(&msp_usart_instance, &usart_rx_byte);
}

