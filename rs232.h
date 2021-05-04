/*
 * rs232.h
 *
 *  Created on: May 4, 2021
 *      Author: huybk213
 */

#ifndef COMPONENTS_RS232_INCLUDE_RS232_H_
#define COMPONENTS_RS232_INCLUDE_RS232_H_

#include <stdint.h>


/**
 * @brief		Start uart rx task
 * @param[in]	rx_num  GPIO RX number
 */
void rs232_uart_rx_start(uint32_t rx_num);

/**
 * @brief		Get number of rx bytes availble on ringbuffer
 * @retval		Number of bytes availble in rx buffer
 */
uint32_t rs232_rx_availble(void);

/**
 * @brief		Read rx data
 * @param[in]	buffer Pointer to buffer will hold data
 * @param[in]	length Size of buffer
 * @retval		Number of bytes read
 */
uint32_t rs232_read(uint8_t *buffer, uint32_t length);

#endif /* COMPONENTS_RS232_INCLUDE_RS232_H_ */
