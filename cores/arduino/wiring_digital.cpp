/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"

#ifdef __cplusplus
 extern "C" {
#endif

uint32_t pinmap[14] = {D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13};
static int32_t digitalPinGet(uint32_t ulPin){
  if (ulPin >= 14) {
	  return -1;
  }
  return pinmap[ulPin];
}

void pinMode( uint32_t ulPin, uint32_t ulMode )
{
  __RW uint32_t *port_mode;
  __RW uint32_t *pupd_en;
  tGPIO_REG *gpio;
  uint32_t pin;
  uint32_t pnum;
  // Handle the case the pin isn't usable as PIO
  pin = digitalPinGet(ulPin);
  if(pin < 0) return;

  pnum = pin & 0xf;
  switch(pin >> 4) {
	  case 0:
		  port_mode = &(SYSCON->pa_port_mode);
		  pupd_en = &(SYSCON->pa_pupd_en);
		  gpio = GPIO1;
	  break;
	  case 1:
		  port_mode = &(SYSCON->pb_port_mode);
		  pupd_en = &(SYSCON->pb_pupd_en);
		  gpio = GPIO2;
	  break;
	  default:
		  port_mode = &(SYSCON->pc_port_mode);
		  pupd_en = &(SYSCON->pc_pupd_en);
		  gpio = GPIO3;
	  break;
  }

  switch ( ulMode )
  {
    case INPUT:
      // Set pin as GPIO
	  *port_mode = *port_mode & ~ (1 << pnum);
	  *pupd_en = *pupd_en & ~((1 << pnum) | (1 << (pnum + 16)));
	  gpio->out_mode &= ~( 1 << pnum);
    break ;

    case INPUT_PULLUP:
      // Set pin as GPIO
	  *port_mode = *port_mode & ~ (1 << pnum);
      // Set pin to input mode with pull-up resistor enabled
	  *pupd_en = *pupd_en | (1 << pnum);
	  *pupd_en = *pupd_en & ~(1 << (pnum+16));
	  gpio->out_mode &= ~( 1 << pnum);
    break ;

    case INPUT_PULLDOWN:
      // Set pin as GPIO
	  *port_mode = *port_mode & ~ (1 << pnum);
      // Set pin to input mode with pull-down resistor enabled
	  *pupd_en = *pupd_en & ~(1 << pnum);
	  *pupd_en = *pupd_en | (1 << (pnum+16));
	  gpio->out_mode &= ~( 1 << pnum);
    break ;

    case OUTPUT:
      // Set pin as GPIO
	  *port_mode = *port_mode & ~ (1 << pnum);
      // Set pin to output mode
	  gpio->out_mode |= ( 1 << pnum);
    break ;

    default:
      // do nothing
    break ;
  }
}

void digitalWrite( uint32_t ulPin, uint32_t ulVal )
{
  tGPIO_REG *gpio;
  uint32_t pin;
  uint32_t pnum;
  // Handle the case the pin isn't usable as PIO

  pin = digitalPinGet(ulPin);
  if(pin < 0) return;

  pnum = pin & 0xf;
  switch(pin >> 4) {
	  case 0:
		  gpio = GPIO1;
	  break;
	  case 1:
		  gpio = GPIO2;
	  break;
	  default:
		  gpio = GPIO3;
	  break;
  }

  switch ( ulVal )
  {
    case LOW:
		gpio->set_reset = (1 << (pnum + 16));
    break ;

    default:
		gpio->set_reset = (1 << pnum);
    break ;
  }

  return ;
}

int digitalRead( uint32_t ulPin )
{
  uint32_t pnum = ulPin & 0xf;
  tGPIO_REG *gpio;
  // Handle the case the pin isn't usable as PIO
  if (ulPin >= PIN_MAX) {
	  return LOW;
  }

  switch(ulPin >> 4) {
	  case 0:
		  gpio = GPIO1;
	  break;
	  case 1:
		  gpio = GPIO2;
	  break;
	  default:
		  gpio = GPIO3;
	  break;
  }

  if (gpio->data & (1 << pnum))
  {
    return HIGH ;
  }

  return LOW ;
}

#ifdef __cplusplus
}
#endif

/*
 * vim:ts=4 sw=4
 */
