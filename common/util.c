/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/usb/dwc/otg_fs.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/spi.h>
#include <stdio.h>
#include <errno.h>

#include "general.h"
#include "usb.h"
#include "serialno.h"

uint32_t DelayCounter;

extern usbd_device *usbd_dev;

/* USB incoming buffer */
char buf_usb_in[4096];
int usb_data_count;



void sys_tick_handler(void)
{
	DelayCounter++;
}

void delay_ms(uint32_t ms)
{
	DelayCounter = 0;
	while (DelayCounter < ms * 1000) __asm__("nop")
		;
}

void delay_us(uint32_t us)
{
	DelayCounter = 0;
	while (DelayCounter < us) __asm__("nop")
		;
}

int _write(int file, uint8_t *const ptr, const size_t len)
{
/*	if (file == 1) {
		size_t sent = 0;
		for (size_t offset = 0; offset < len; offset += 64)
		{
			const size_t count = MIN(len - offset, 64);
			nvic_disable_irq(USB_IRQ);
			usbd_ep_write_packet(usbd_dev, 0x82, ptr + offset, count);
			nvic_enable_irq(USB_IRQ);
			sent += count;
			delay_us(125);// why is this needed?
		}
		return sent;
	}

	errno = EIO;
	return -1;*/
	int i;
	if (file == 1) {
		for (i = 0; i < len; i++)
			usart_send_blocking(USART2, ptr[i]);
		return i;
	}

	errno = EIO;
	return -1;
}

bool usart_data_waiting(uint32_t port)
{
	return ((USART_SR(port) & USART_SR_RXNE) != 0);
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

	if (len) {
		for(int i = 0; i < len; i++){
			buf_usb_in[i] = buf[i];
			usb_data_count++;
		}
	}
}

bool usb_data_waiting(void)
{
	return ( usb_data_count != 0);
}

void usb_wait_recv_ready(void)
{
	while (!usb_data_waiting());
}

uint16_t usb_recv(void)
{
	usb_data_count--;
	return buf_usb_in[0];
}

uint16_t usb_recv_blocking()
{
	usb_wait_recv_ready();
	return usb_recv();
}

void usb_wait_send_ready()
{
}

void usb_send(uint16_t data)
{
	uint8_t packet_buf[64];
	uint8_t packet_size = 0;
	packet_buf[0]=data;
	packet_size = 1;
	while (usbd_ep_write_packet(usbd_dev, 0x82, packet_buf, packet_size) == 0);
}

void usb_send_blocking(uint16_t data)
{
	usb_wait_send_ready();
	usb_send(data);
}

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		return USBD_REQ_HANDLED;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding)) {
			return USBD_REQ_NOTSUPP;
		}

		return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}


void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
			cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}
