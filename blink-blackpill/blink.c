/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (c) 2015 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "util.h"
#include "setup.h"
#include "blackpill-f4.h"


int main(void)
{
	int i;
	clock_setup();
	gpio_setup();
	systick_setup();
	usart_setup();
	while (1) {
		gpio_set(LED_PORT, LED_PIN);	/* LED on */
		printf("off\r\n");
		delay_ms(100);
		gpio_clear(LED_PORT, LED_PIN);	/* LED off */
		printf("on\r\n");
		delay_ms(100);
	}

	return 0;
}
