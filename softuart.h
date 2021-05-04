/** @file    softuart.h
  * @author  minhneo, huytv
  * @version 0.1
  * @brief   Cung cap cac ham de quan ly cac chuc nang uart mem
  *          su dung time va ngat ngoai
  * @note    software uart baud: 115200, 8-N-1
  */

#ifndef __SOFTUART_H_
#define __SOFTUART_H_

/*********************************************************************************
 * INCLUDE
 */
#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#include "esp_attr.h"
#define SOFTWARE_UART_SECTION_IRAM    IRAM_ATTR

#define SOFTUART_TX_PIN_INVALID     (-1)
#define SOFTUART_RX_PIN_INVALID     (-1)


// #define SOFTUART_RX_MAX_SIZE        (128)

/*********************************************************************************
 * DEFINE
 */

#define SOFTUART_CREATE_DEFAULT()   {   \
                                        .rx_queue_buffer            = (void*)0,   \
                                        .rx_queue_size              = 0x00,       \
                                        .ringbuffer_received_wr_idx          = 0x00,       \
                                        .ringbuffer_received_read_idx      = 0x00,       \
                                        .tx_done_flag               = 0x00,       \
                                        .rx_done_flag               = 0x00,       \
                                        .tx_shift_byte             = 0x00,       \
                                        .tx_shift_position          = 0x00,       \
                                        .rx_shift_byte             = 0x00,       \
                                        .rx_shift_position          = 0x00,       \
										.stopbit_error_count        = 0x00,	      \
										.timer_rx_run                  = 0x00,			\
										.timer_tx_run                  = 0x00,			\
                                    }

/*********************************************************************************
 * TYPEDEF
 */

/**
 * @brief software uart mode enumeration 
 */
typedef enum
{
    SOFUART_MODE_TX,
    SOFUART_MODE_RX
} softuart_mode_t;

typedef void (*softuart_timer_set_cb_t)(uint32_t timer_id, bool enable);
typedef void (*softuart_gpio_irq_control_cb_t)(uint32_t rx_pin, bool enable);
typedef uint32_t (*softuart_get_tick_cb_t)(void);
typedef uint32_t (*softuart_read_input_gpio_cb_t)(uint32_t rx_pin);
typedef void (*softuart_write_output_gpio_cb_t)(uint32_t tx_pin, uint32_t level);
typedef void (*softuart_on_frame_error_cb_t)(void *uart);
typedef void (*softuart_on_overrun_cb_t)(void *uart);
typedef void (*software_uart_fifo_full_cb_t)(void *uart);
typedef void (*software_uart_delay_us_cb_t)(uint32_t us);

/**
 * @brief software uart struct
 */
typedef struct
{
    int32_t tx_pin;
    int32_t rx_pin;
    
    softuart_read_input_gpio_cb_t read_rx_pin;
    softuart_write_output_gpio_cb_t write_tx_pin;
    softuart_gpio_irq_control_cb_t ctrol_rx_irq;
    softuart_on_frame_error_cb_t on_frame_error;
    softuart_on_overrun_cb_t on_overrun_cb;
    software_uart_fifo_full_cb_t fifo_full;
    software_uart_delay_us_cb_t delay;

    volatile uint32_t tx_timer_id;
    volatile uint32_t rx_timer_id;

    softuart_timer_set_cb_t timer_rx_control;
    softuart_timer_set_cb_t timer_tx_control;
    softuart_get_tick_cb_t get_tick;

    volatile softuart_mode_t mode;
    
    volatile uint8_t tx_byte;
    volatile uint8_t *rx_queue_buffer;          // Rx ring buffer
    volatile uint32_t  rx_queue_size;            // Size of uart rx ringbuffer
    
    volatile uint32_t ringbuffer_received_wr_idx;        // Total byte received
    volatile uint32_t ringbuffer_received_read_idx;    // Current rx read index

    volatile bool tx_done_flag;
    volatile bool rx_done_flag;

    volatile uint8_t tx_shift_byte;            // Temporary TX byte for shifted in interrupt service
    volatile uint8_t tx_shift_position;         // Temprary RX bit shift position

    volatile uint8_t rx_shift_byte;            // Temporary RX byte for shifted in interrupt service
    volatile uint8_t rx_shift_position;         // Temporary RX bit shift position

    volatile uint32_t stopbit_error_count;

    uint32_t timer_rx_run;
    uint32_t timer_tx_run;
} softuart_t;


/*********************************************************************************
 * EXTERN FUNCTION
 */

/**
 * @brief  Init software uart
 * @param  softuart_t *uart  - pointer software uart
 *         FTM_Type *base    - FTM peripheral base address
 *         IRQn_Type interrupt - 
 * @retval NONE
 */
void softuart_initialize(softuart_t *uart);

/**
 * @brief  Go to transmit mode
 * @param  softuart_t *uart  - pointer software uart
 * @retval NONE
 */
void softuart_go_transmit(softuart_t *uart);

/**
 * @brief  Go to receive mode
 * @param  softuart_t *uart  - pointer software uart
 * @retval NONE
 */
void softuart_go_receive(softuart_t *uart);

/**
 * @brief  Clear receive buffer
 * @param  softuart_t *uart  - pointer software uart
 * @retval NONE
 */
void softuart_rx_clear(softuart_t *uart);

/**
 * @brief  Transmit number byte in data
 * @param  softuart_t *uart  - pointer software uart
 *         uint8_t* pdata    - pointer data transmit
 *         uint16_t length   - length of data transmit
 * @retval TRUE 			 -  Data was sent
 * 		   FALSE			 - Error
 */
bool softuart_transmit(softuart_t *uart, uint8_t *pdata, uint16_t length);

/**
 * @brief  Check number byte avaiable in receive buffer
 * @param  softuart_t *uart  - pointer software uart
 * @retval NONE
 */
uint8_t softuart_rx_availble(softuart_t *uart);

/**
 * @brief  Peek one byte in receive buffer
 * @param  softuart_t *uart  - pointer software uart
 * @retval NONE
 */
uint8_t softuart_rx_peak(softuart_t *uart);

/**
 * @brief  		Read one byte in receive buffer
 * @param[in]	softuart_t *uart  - pointer software uart
 * @param[out]	data 			  - Uart new data
 * @retval 		TRUE 			  - Data read
 * 				FALSE			  - Buffer empty
 */
bool softuart_read_byte(softuart_t *uart, uint8_t *data);

/**
 * @brief  		Read bytes array in receive buffer
 * @param[in]	softuart_t *uart  - pointer software uart
 * @param[in]	size		      - Number of bytes want to read
 * @param[out]	data 			  - Uart new data
 * @retval 		Number of bytes received
 */
uint32_t softuart_read_bytes(softuart_t *uart, uint8_t *data, uint32_t size);

/**
 * @brief   Start receive one character
 * @param   softuart_t *uart  - pointer software uart
 * @retval  TRUE : UART enter rx mode success
 * 			FALSE : UART enter rx mode failed
 */
SOFTWARE_UART_SECTION_IRAM inline bool softuart_start_received(softuart_t *uart);

/**
 * @brief   Get RX state
 * @param   softuart_t *uart  - pointer software uart
 * @retval  TRUE : UART is enter rx mode
 * 			FALSE : UART is not enter rx mode
 */
SOFTWARE_UART_SECTION_IRAM bool softuart_is_rx_idle(softuart_t *uart);

/**
 * @brief   Receive one byte in time interrupt callback
 * @param   softuart_t *uart  - pointer software uart
 * @retval  NONE
 */
SOFTWARE_UART_SECTION_IRAM void softuart_timer_rx_process_callback(softuart_t *uart);

/**
 * @brief   Receive one byte in time interrupt callback
 * @param   softuart_t *uart  - pointer software uart
 * @retval  NONE
 */
SOFTWARE_UART_SECTION_IRAM void softuart_timer_tx_process_callback(softuart_t *uart);

/**
 * @brief   Get uart TX-RX status
 * @retval  TRUE : TX-RX busy
 * 			FALSE : TX_RX is not busy
 */
bool software_uart_in_is_busy(softuart_t *uart);

#endif /* __SOFTUART_H_ */
