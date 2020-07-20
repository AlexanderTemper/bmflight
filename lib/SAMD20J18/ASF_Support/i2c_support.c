/**********************************************************************/

#include "i2c_support.h"


struct i2c_master_module i2c_master_instance;

/************************************************************************/
/* Function Definitions                                                 */
/************************************************************************/

enum status_code i2c_initialize(void) {
    uint8_t status = STATUS_OK;
    /*
    struct i2c_master_config config_i2c_master;
    i2c_master_get_config_defaults(&config_i2c_master);

    config_i2c_master.baud_rate = I2C_MASTER_BAUD_RATE_400KHZ;
    config_i2c_master.buffer_timeout = 1000;


    config_i2c_master.pinmux_pad0 = PINMUX_PA22C_SERCOM3_PAD0;
    config_i2c_master.pinmux_pad1 = PINMUX_PA23C_SERCOM3_PAD1;
    status = i2c_master_init(&i2c_master_instance, SERCOM3, &config_i2c_master);
    if (status == STATUS_OK) {
        i2c_master_enable(&i2c_master_instance);
    }
*/
    return status;
}
