/** @file    softuart.c
  * @author  minhneo
  * @version 0.1
  * @brief   Cung cap cac ham de quan ly cac chuc nang uart mem
  *          su dung time va ngat ngoai
  * @note    software uart baud: 115200, 8-N-1
  */

/*********************************************************************************
 * INCLUDE
 */
#include "softuart.h"
/*********************************************************************************
 * DEFINE
 */

//#define SUART_RXPIN 15
//#define SUART_TXPIN 16
//
//#define RXPIN PE15
//#define TXPIN PE16



// #define FTM_1_CLOCK_SOURCE CLOCK_GetFreq(kCLOCK_CoreSysClk)
/*********************************************************************************
 * EXTERN FUNCTION
 */

/*********************************************************************************
 * STATIC VARIABLE
 */

/*********************************************************************************
 * STATIC FUNCTION
 */
static bool softuart_write_bytes(softuart_t *uart, uint8_t data);
/*********************************************************************************
 * EXTERN FUNCTION
 */

/**
 * @brief  Init software uart
 * @param  softuart_t *uart  - pointer software uart
 *         FTM_Type *base    - FTM peripheral base address
 *         
 * @retval NONE
 */
void softuart_initialize(softuart_t *uart)
{
    /* init variable */
    uart->tx_byte = 0x00;
    uart->ringbuffer_received_wr_idx = 0x00;
    uart->ringbuffer_received_read_idx = 0x00;
    uart->mode = SOFUART_MODE_RX;
    uart->rx_done_flag = false;
    uart->tx_done_flag = false;
    uart->rx_shift_byte = 0;
    uart->rx_shift_position = 0;
    uart->tx_shift_byte = 0;
    uart->tx_shift_position = 0;
}

/**
 * @brief  Go to transmit mode
 * @param  softuart_t *uart  - pointer software uart
 * @retval NONE
 */
void softuart_go_transmit(softuart_t *uart)
{
//    uart->mode = SOFUART_MODE_TX;
}

/**
 * @brief  Go to receive mode
 * @param  softuart_t *uart  - pointer software uart
 * @retval NONE
 */
void softuart_go_receive(softuart_t *uart)
{
//    uart->mode = SOFUART_MODE_RX;
}

/**
 * @brief  Clear receive buffer
 * @param  softuart_t *uart  - pointer software uart
 * @retval NONE
 */
void softuart_rx_clear(softuart_t *uart)
{
    uart->ringbuffer_received_wr_idx = 0x00;
    uart->ringbuffer_received_read_idx = 0x00;
    memset((void *)uart->rx_queue_buffer, 0, uart->rx_queue_size);
}

/**
 * @brief  Clear receive buffer
 * @param  softuart_t *uart  - pointer software uart
 *         uint8_t* pdata    - pointer data transmit
 *         uint16_t length   - length of data transmit
 * @retval TRUE 			 - Data was sent
 * 		   FALSE			 - Error
 */
bool softuart_transmit(softuart_t *uart, uint8_t *pdata, uint16_t length)
{
	if (uart->tx_pin == SOFTUART_TX_PIN_INVALID
		|| uart->write_tx_pin == NULL)
	{
		return false;
	}
    /* Go to transmit mode */
    softuart_go_transmit(uart);

    while (length)
    {
        /* write one byte */
        if (!softuart_write_bytes(uart, *pdata))
        {
        	return false;
        }
        pdata++;
        length--;
    }

    /* Go to transmit mode */
    softuart_go_receive(uart);
    return true;
}

/**
 * @brief  Check number byte avaiable in receive buffer
 * @param  softuart_t *uart  - pointer software uart
 * @retval NONE
 */
uint8_t softuart_rx_availble(softuart_t *uart)
{
	if (uart->ringbuffer_received_wr_idx == uart->ringbuffer_received_read_idx)
	{
		return 0;
	}
	else if (uart->ringbuffer_received_wr_idx > uart->ringbuffer_received_read_idx)
	{
		return uart->ringbuffer_received_wr_idx - uart->ringbuffer_received_read_idx;
	}

    return (uart->rx_queue_size + uart->ringbuffer_received_wr_idx - uart->ringbuffer_received_read_idx);
}

/**
 * @brief  Peek one byte in receive buffer
 * @param  softuart_t *uart  - pointer software uart
 * @retval NONE
 */
uint8_t softuart_rx_peak(softuart_t *uart)
{
    return uart->rx_queue_buffer[uart->ringbuffer_received_read_idx];
}

bool softuart_read_byte(softuart_t *uart, uint8_t *data)
{
	if (softuart_rx_availble(uart))
	{
		*data = uart->rx_queue_buffer[uart->ringbuffer_received_read_idx++];
		if (uart->ringbuffer_received_read_idx == uart->rx_queue_size)
		{
			uart->ringbuffer_received_read_idx = 0;
		}
		return true;
	}
    return false;
}

uint32_t softuart_read_bytes(softuart_t *uart, uint8_t *data, uint32_t size)
{
	uint32_t i = 0;
	for (i = 0; i < size; i++)
	{
		if (softuart_rx_availble(uart))
		{
			*data++ = uart->rx_queue_buffer[uart->ringbuffer_received_read_idx++];
			if (uart->ringbuffer_received_read_idx == uart->rx_queue_size)
			{
				uart->ringbuffer_received_read_idx = 0;
			}
		}
		else
		{
			break;
		}
	}
	return i;
}

inline SOFTWARE_UART_SECTION_IRAM uint32_t softuart_rx_write_buffer_availble(softuart_t *uart)
{
	uint32_t size;

	if (uart->ringbuffer_received_read_idx == uart->ringbuffer_received_wr_idx)
	{
		size = uart->rx_queue_size;
	}
	else if (uart->ringbuffer_received_read_idx > uart->ringbuffer_received_wr_idx)
	{
		size = uart->ringbuffer_received_read_idx - uart->ringbuffer_received_wr_idx;
	}
	else
	{
		size = uart->rx_queue_size - (uart->ringbuffer_received_wr_idx - uart->ringbuffer_received_read_idx);
	}
	return size - 1;
}


static bool softuart_write_bytes(softuart_t *uart, uint8_t data)
{
    /* send data to tx bufer */
    uart->tx_byte = data;
    uart->tx_done_flag = false;
    
    if (uart->timer_tx_control)
	{
    	uart->timer_tx_control(uart->tx_timer_id, true);
	}
    uart->timer_tx_run = true;

    /* Wait tx_done_flag */
    uint32_t timeout = uart->get_tick();

    while (uart->tx_done_flag == false)
    {
        if (uart->get_tick() - timeout > 2)
        {
        	if (uart->timer_tx_control)
			{
        		uart->timer_tx_control(uart->tx_timer_id, false);
			}
        	uart->timer_tx_run = false;
        	return false;
        }
    }

    /* Stop timer */
    if (uart->timer_tx_control)
	{
		uart->timer_tx_control(uart->tx_timer_id, false);
	}
    uart->timer_tx_run = false;
    return true;
}


SOFTWARE_UART_SECTION_IRAM bool softuart_start_received(softuart_t *uart)
{
    if ((uart->rx_shift_position == 0x00) && (uart->read_rx_pin(uart->rx_pin) == 0))
    {
        if (uart->timer_rx_control)
		{
        	uart->timer_rx_control(uart->rx_timer_id, true);
		}

        uart->timer_rx_run = true;
        if (uart->ctrol_rx_irq)
		{
        	uart->ctrol_rx_irq(uart->rx_pin, false);
		}
        uart->rx_shift_position++;
        uart->rx_shift_byte = 0x00;
        return true;
    }
    else
    {
        if (uart->on_frame_error)
		{
			uart->on_frame_error(uart);
		}
    }
    return false;
}


SOFTWARE_UART_SECTION_IRAM bool softuart_is_rx_idle(softuart_t *uart)
{
	return (uart->rx_shift_position);
}

SOFTWARE_UART_SECTION_IRAM void softuart_timer_rx_process_callback(softuart_t *uart)
{
	/* 8 data bit */
	uint8_t pin_val = uart->read_rx_pin(uart->rx_pin);
	if (uart->rx_shift_position && uart->rx_shift_position < 9)
	{
		if (pin_val)
		{
			uart->rx_shift_byte |= ((0x01 << (uart->rx_shift_position - 1)));
		}
		uart->rx_shift_position++;
		return;
	}

	/* stop bit */
	if ((pin_val && (uart->rx_shift_position == 9)))
	{
		uart->rx_shift_position = 0;
		if (uart->timer_rx_control)
		{
			uart->timer_rx_control(uart->rx_timer_id, false);
		}
		uart->timer_rx_run = false;
		if (uart->ctrol_rx_irq)
		{
			uart->ctrol_rx_irq(uart->rx_pin, true);
		}
		uart->stopbit_error_count = 0;


		/* get data in head */
		if (softuart_rx_write_buffer_availble(uart))
		{
			uart->rx_queue_buffer[uart->ringbuffer_received_wr_idx++] = uart->rx_shift_byte;
			if (uart->ringbuffer_received_wr_idx == uart->rx_queue_size)
			{
				uart->ringbuffer_received_wr_idx = 0;
			}
		}
		else
		{
			if (uart->fifo_full)
			{
				uart->fifo_full(uart);
			}
		}
		uart->rx_shift_byte = 0;
		return;
	}
	else
//        else if (uart->rx_shift_position)
	{
		if (uart->stopbit_error_count++ >= 18)
		{
			uart->stopbit_error_count = 0;

			if (uart->timer_rx_control)
			{
				uart->timer_rx_control(uart->rx_timer_id, false);
			}
			uart->timer_rx_run = false;
			if (uart->ctrol_rx_irq)
			{
				uart->ctrol_rx_irq(uart->rx_pin, true);
			}
			if (uart->on_frame_error)
			{
				uart->on_frame_error(uart);
			}
			uart->rx_shift_position = 0;
			uart->rx_shift_byte = 0x00;
		}
	}
}

SOFTWARE_UART_SECTION_IRAM void softuart_timer_tx_process_callback(softuart_t *uart)
{
	/* start bit */
	if (uart->tx_shift_position == 0)
	{
		uart->tx_shift_position++;
		uart->tx_shift_byte = uart->tx_byte;
		if (uart->write_tx_pin)
		{
			uart->write_tx_pin(uart->tx_pin, 0);
		}
		return;
	}

	/* 8 data bit */
	if (uart->tx_shift_position < 9)
	{
		if ((uart->tx_shift_byte >> (uart->tx_shift_position - 1)) & 0x01)
		{
			if (uart->write_tx_pin) uart->write_tx_pin(uart->tx_pin, 1);
		}
		else
		{
			if (uart->write_tx_pin) uart->write_tx_pin(uart->tx_pin, 0);
		}
		uart->tx_shift_position++;
		return;
	}

	/* stop bit */
	if (uart->tx_shift_position == 9)
	{
		uart->tx_shift_position = 0;
		if (uart->write_tx_pin)
		{
			uart->write_tx_pin(uart->tx_pin, 1);
		}

		uart->tx_done_flag = true;
		if (uart->timer_tx_control)
		{
			uart->timer_tx_control(uart->tx_timer_id, false);
		}

		uart->timer_tx_run = false;
		return;
	}
}

bool software_uart_in_is_busy(softuart_t *uart)
{
	if (uart->tx_shift_position || uart->rx_shift_position)
	{
		return true;
	}
	return false;
}

