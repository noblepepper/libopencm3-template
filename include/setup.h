

#ifndef INCLUDE_SETUP_H
#define INCLUDE_SETUP_H

#include <stdint.h>
#include <stdbool.h>

void clock_setup(void);
void systick_setup(void);
void usart_setup(void);
void gpio_setup(void);
void usb_wait_recv_ready(void);
uint16_t usb_recv(void);
uint16_t usb_recv_blocking(void);
/*void usb_wait_send_ready(void);
void usb_send(uint16_t data);
void usb_send_blocking(uint16_t data);
void usb_setup(void);*/
void i2c_setup(void);
void spi1_setup(void);
void spi3_setup(void);
bool usb_data_waiting(void);

#endif /* INCLUDE_SETUP_H */
