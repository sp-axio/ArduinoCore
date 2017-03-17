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
#include "wiring_private.h"

static void pinMode_mf( uint32_t ulPin, EPioType ulPeripheral )
{
  uint32_t pnum = ulPin & 0xf;
  uint32_t pinMask = 1 << pnum;
  uint32_t mfMask;
  __RW uint32_t *port_mode = NULL;
  __RW uint32_t *mfsel = NULL;

  switch(ulPin >> 4) {
	  case 0:
		  port_mode = &(SYSCON->pa_port_mode);
		  if (pnum < 8)
			  mfsel = &(SYSCON->pa_mfsel_lo);
		  else
			  mfsel = &(SYSCON->pa_mfsel_hi);
	  break;
	  case 1:
		  port_mode = &(SYSCON->pb_port_mode);
		  if (pnum < 8)
			  mfsel = &(SYSCON->pb_mfsel_lo);
		  else
			  mfsel = &(SYSCON->pb_mfsel_hi);
	  break;
	  default:
		  port_mode = &(SYSCON->pc_port_mode);
		  if (pnum < 8)
			  mfsel = &(SYSCON->pc_mfsel_lo);
		  else
			  mfsel = &(SYSCON->pc_mfsel_hi);
	  break;
  }

  // set MF
  *port_mode |= pinMask;
  if (pnum >= 8) {
	  pnum -= 8;
  }
  mfMask = (0xf << ((pnum & 0x7)*4));
  *mfsel &= ~mfMask;
  *mfsel |= (ulPeripheral << ((pnum & 0x7)*4));
}

int pinPeripheral( uint32_t ulPin, EPioType ulPeripheral )
{
  // Handle the case the pin isn't usable as PIO
  if (ulPin >= PIN_MAX) {
	  return -1;
  }

  switch ( ulPeripheral )
  {
    case PIO_DIGITAL:
    case PIO_INPUT:
    case PIO_INPUT_PULLUP:
    case PIO_OUTPUT:
      // Disable peripheral muxing, done in pinMode

      // Configure pin mode, if requested
      if ( ulPeripheral == PIO_INPUT ) {
        pinMode( ulPin, INPUT ) ;
      } else {
        if ( ulPeripheral == PIO_INPUT_PULLUP ) {
          pinMode( ulPin, INPUT_PULLUP ) ;
        } else {
          if ( ulPeripheral == PIO_OUTPUT ) {
            pinMode( ulPin, OUTPUT ) ;
          } else {
            // PIO_DIGITAL, do we have to do something as all cases are covered?
          }
        }
      }
    break ;

    case PIO_SPI:
    case PIO_UART:
    case PIO_I2C:
    case PIO_TIMER:
    case PIO_SDIO:
    	pinMode_mf(ulPin, ulPeripheral);
    break ;

    case PIO_NOT_A_PIN:
      return -1l ;
    break ;
  }

  return 0l ;
}

/* vim: ts=4 sw=4
 */
