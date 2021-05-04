/*
 * hopqui.c
 *
 *  Created on: May 4, 2021
 *      Author: huybk213
 */


#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "lwrb/lwrb.h"
#include "freertos/timers.h"
#include "softuart.h"

#define RMT_RX_MAX_BIT_QUEUE_SIZE 256

static const char *TAG = "rs232";
static rmt_channel_t m_rmt_rx_channel = RMT_CHANNEL_0;
static uint8_t m_rmt_rx_bit_queue[RMT_RX_MAX_BIT_QUEUE_SIZE];
static lwrb_t m_ringbuffer_rmt_rx_bit;
TimerHandle_t m_time_handler_clr_uart;
static uint32_t m_gpio_rx_num = 0;

static softuart_t m_softuart_rs232 = SOFTUART_CREATE_DEFAULT();

static inline uint32_t get_nb_of_bit(uint32_t time)
{
	return (time+2)/9;		// baudrate 115200 =>> 1 bit is 8.5us
}

static void parse_rmt_rx(rmt_item32_t *items, uint32_t size)
{
	for (uint32_t i = 0; i < size; i++)
	{
        uint32_t nb_of_zero_bit = get_nb_of_bit((items+i)->duration0);
        uint32_t nb_of_set_bit = get_nb_of_bit((items+i)->duration1);
        if ((items+i)->duration1 == 0)
        {
        	// this is stop bit
        	nb_of_set_bit = 1;
        }
//        printf("------\r\n%u-%u-%u bits, %u-%u-%u bits, total %u bits\r\n",  (items+i)->level0, (items+i)->duration0, nb_of_zero_bit,
//        													(items+i)->level1, (items+i)->duration1,
//															nb_of_set_bit,
//															nb_of_zero_bit + nb_of_set_bit);

        if (lwrb_get_free(&m_ringbuffer_rmt_rx_bit) < (nb_of_zero_bit + nb_of_set_bit))
        {
        	ESP_LOGE(TAG, "RMT queue full");
        	return;
        }
        if (nb_of_zero_bit)
        {
        	uint8_t tmp = 0;
        	if (nb_of_zero_bit > 10)
        	{
        		ESP_LOGE(TAG, "Invalid input");
        	}
        	for (uint32_t i = 0; i < nb_of_zero_bit; i++)
        	{
        		if (lwrb_write(&m_ringbuffer_rmt_rx_bit, &tmp, 1) == 0)
        		{
        			ESP_LOGE(TAG, "Queue bit full\r\n");
        		}
        	}
        }

        if (nb_of_set_bit)
        {
        	uint8_t tmp = 1;
        	if (nb_of_set_bit > 10)
        	{
        		ESP_LOGE(TAG, "Invalid input");
        	}
        	for (uint32_t i = 0; i < nb_of_set_bit; i++)
        	{
        		if (lwrb_write(&m_ringbuffer_rmt_rx_bit, &tmp, 1) == 0)
        		{
        			ESP_LOGE(TAG, "Queue bit full\r\n");
        		}
        	}
        }
		if (!xTimerIsTimerActive(m_time_handler_clr_uart))
		{
			xTimerStart(m_time_handler_clr_uart, 0);
		}
		xTimerReset(m_time_handler_clr_uart, 0);
		uint32_t retry = 10;
        while (lwrb_get_full(&m_ringbuffer_rmt_rx_bit) && retry)
        {
        	if (m_softuart_rs232.rx_shift_position == 0x00)
        	{
        		retry--;
        		softuart_start_received(&m_softuart_rs232);
        	}
        	if (m_softuart_rs232.rx_shift_position)
        	{
        		softuart_timer_rx_process_callback(&m_softuart_rs232);
        	}
        }
	}
}

static void on_clear_uart(void *arg)
{
	BaseType_t rw_ctx;
	uint8_t tmp;
	while (lwrb_read(&m_ringbuffer_rmt_rx_bit, &tmp, 1) > 0)
	{

	}
	m_softuart_rs232.rx_shift_position = 0;
	xTimerStopFromISR(m_time_handler_clr_uart, &rw_ctx);
	if (rw_ctx)
	{
		portYIELD_FROM_ISR(rw_ctx);
	}
}

/**
 * @brief RMT Receive Task
 *
 */
static uint8_t m_rx_buffer[256];
static uint32_t fake_softuart_gpio_read(uint32_t pin)
{
	uint32_t tmp = 0;
	lwrb_read(&m_ringbuffer_rmt_rx_bit, &tmp, 1);
	return tmp;
}
static void rmt_uart_task(void *arg)
{
    size_t length = 0;
    RingbufHandle_t rb = NULL;
    rmt_item32_t *items = NULL;

    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(m_gpio_rx_num, m_rmt_rx_channel);
    rmt_rx_config.rx_config.idle_threshold = 200;
    rmt_rx_config.rx_config.filter_ticks_thresh = 2;
    rmt_rx_config.rx_config.filter_en = true;
    rmt_config(&rmt_rx_config);
    rmt_set_mem_block_num(m_rmt_rx_channel, 8);
    assert(ESP_OK == rmt_driver_install(m_rmt_rx_channel, 4096, 0));
    rmt_set_mem_block_num(m_rmt_rx_channel, 8);

    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(m_rmt_rx_channel, &rb);
    assert(rb != NULL);

    m_softuart_rs232.timer_rx_control = NULL;
    m_softuart_rs232.timer_tx_control = NULL;
    m_softuart_rs232.ctrol_rx_irq = NULL;
    m_softuart_rs232.on_frame_error = NULL;
    m_softuart_rs232.rx_timer_id = 0;
    m_softuart_rs232.tx_timer_id = 1;
    m_softuart_rs232.get_tick = xTaskGetTickCount;
    m_softuart_rs232.tx_pin = -1;
    m_softuart_rs232.rx_pin = -1;
    m_softuart_rs232.read_rx_pin = fake_softuart_gpio_read;
    m_softuart_rs232.write_tx_pin = NULL;
    m_softuart_rs232.ctrol_rx_irq = NULL;
    m_softuart_rs232.rx_queue_buffer = m_rx_buffer;
    m_softuart_rs232.rx_queue_size = sizeof(m_rx_buffer);
    softuart_initialize(&m_softuart_rs232);

    lwrb_init(&m_ringbuffer_rmt_rx_bit, m_rmt_rx_bit_queue, sizeof(m_rmt_rx_bit_queue));
    rmt_rx_start(m_rmt_rx_channel, true);
    m_time_handler_clr_uart = xTimerCreate("clr_uart", 30, pdFALSE, NULL, on_clear_uart);

    while (1)
    {
        items = (rmt_item32_t *) xRingbufferReceive(rb, &length, portMAX_DELAY);
        if (items)
        {
            length /= 4; // one RMT = 4 Bytes
            parse_rmt_rx(items, length);
            vRingbufferReturnItem(rb, (void *) items);
        }
//
//		while (1)
//		{
//			uint32_t size;
//			uint8_t tmp[16];
//			size = softuart_read_bytes(&m_softuart_rs232, tmp, sizeof(tmp));
//			if (size)
//			{
//				for (uint32_t i = 0; i < size; i++)
//				{
//					printf("%c", tmp[i]);
//					fflush(stdout);
//				}
//			}
//			else
//			{
//				break;
//			}
//		}
	}
    rmt_driver_uninstall(m_rmt_rx_channel);
    vTaskDelete(NULL);
}

void rs232_uart_rx_start(uint32_t gpio_num)
{
	static bool task_created = false;
	if (task_created == false)
	{
		task_created = true;
		m_gpio_rx_num = gpio_num;
		xTaskCreate(rmt_uart_task, "rmt_rx_task", 2048, NULL, 10, NULL);
	}
}

uint32_t rs232_rx_availble(void)
{
	return softuart_rx_availble(&m_softuart_rs232);
}

uint32_t rs232_read(uint8_t *buffer, uint32_t length)
{
	uint32_t size;
	size = softuart_read_bytes(&m_softuart_rs232, buffer, length);
	return size;
}


