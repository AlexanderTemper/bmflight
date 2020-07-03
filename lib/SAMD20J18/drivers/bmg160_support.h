#ifndef BMG160_SUPPORT_H_
#define BMG160_SUPPORT_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include "bmg160.h"
#include "spi_support.h"
#include "tc_support.h"

/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/

/*! BMG160 SPI slave select pin */
#define BMG160_SS_PIN PIN_PA27

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

/*! Instantiates a bmg160 software instance structure, which holds
* relevant information about BMG160 and links communication to the SPI bus. */
extern struct bmg160_t bmg160;
/*! It instantiates an SPI slave software instance structure, used to configure
* the correct SPI transfer mode settings for an attached slave (here BMM160 is the slave).
* For example it holds the SS pin number of the corresponding slave. */
extern struct spi_slave_inst bmg160_spi_slave;

/************************************************************************/
/* Function Declarations                                                */
/************************************************************************/

/*!
* @brief		Initializes BMA280 accelerometer sensor and its required connections
*
* @param [in]	NULL
*
* @param [out]	NULL
*
* @return		NULL
*
*/
void bmg_init(void);

/*!
* @brief		Sends data to BMG160 via SPI
*
* @param[in]	dev_addr	Device I2C slave address (not used)
*
* @param[in]	reg_addr	Address of destination register
*
* @param[in]	reg_data	Pointer to data buffer to be sent
*
* @param[in]	length		Length of the data to be sent
*
* @retval		0			BMG160_SUCCESS
* @retval		-1			BMG160_ERROR
*
*/
int8_t bmg_spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t length);

/*!
* @brief		Receives data from BMM050 on SPI
*
* @param[in]	dev_addr	Device I2C slave address (not used)
*
* @param[in]	reg_addr	Address of destination register
*
* @param[out]	reg_data	Pointer to data buffer to be received
*
* @param[in]	length		Length of the data to be received
*
* @retval		0			BMG160_SUCCESS
* @retval		-1			BMG160_ERROR
*
*/
int8_t bmg_spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *rx_data, uint8_t length);

/*!
* @brief		Initializes BMG160 gyroscope sensor and its required connections
*
* @param[in]	msec	Delay length in terms of milliseconds
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void bmg_delay_msec(uint32_t msec);


#endif /* BMG160_SUPPORT_H_ */
