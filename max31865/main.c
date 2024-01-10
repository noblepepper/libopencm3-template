/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
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

#include "setup.h"
#include "util.h"
#include "max31865.h"
#include <math.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>


int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;
	
	float resistance[NUM_SENSORS];
 	float temperature;
//	float temperature2[NUM_SENSORS];
//	float temperature3[NUM_SENSORS];
	float Rref = 439;

	clock_setup();
	systick_setup();
	//usb_setup();
	usart_setup();
	gpio_setup();
	spi3_setup();
	delay_ms(5000);

	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		chip_select(i, false);
	}
	printf("\r\n\r\nHello\r\n");
	for (uint8_t i = 0; i < NUM_SENSORS; i++)
	{
		printf("Sensor %i \r\n", i);
		print_max31865_registers(i);
		set_max31865_to_power_up(i);
		print_max31865_registers(i);
		init_max31865_triggered_60hz(i);
		print_max31865_registers(i);
	}
	delay_ms(1000);
	
	while (true) {
		int i, j;
		gpio_toggle(GPIOC, GPIO13);
		for (i = 0; i < NUM_SENSORS; i++)
		{
			resistance[i] = 0;
		}
		for (j = 0; j < 64; j++)
		{
			/* start all sensors */
			for (i = 0; i < NUM_SENSORS; i++)
			{
				one_shot(i);
			}
			/* read each one as it completes */
			for (i = 0; i < NUM_SENSORS; i++)
			{
				while (!ready(i));
				resistance[i] += 
					read_rtd_resistance(i, Rref)/64;
			}
		}
		for (i = 0; i < NUM_SENSORS; i++)
		{
			temperature = get_temperature_method3(resistance[i]);
			printf("Sensor %i %3.2f ",i,  temperature);
		}
		printf("\r\n");
//		temperature2 = get_temperature_method2(resistance);
//		temperature3 = get_temperature_method3(resistance);
//		printf("temperature2 F  %3.2f ", temperature2);
//		printf("temperature3 F  %3.2f \r\n", temperature3);
	}

	return 0;
}
