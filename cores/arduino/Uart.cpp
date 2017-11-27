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

#include "ewbm_uart.h"
#include "Uart.h"
#include "Arduino.h"
#include "wiring_private.h"
#define UARTBUF_SIZE 256

Uart::Uart(uint32_t _bank, uint8_t _pinRX, uint8_t _pinTX)
{
  memset(&uart, 0, sizeof(uart));
  uart.bank = (eUART_BANK)_bank;

  uc_pinRX = _pinRX;
  uc_pinTX = _pinTX;
}

void Uart::begin(unsigned long baudrate)
{
  begin(baudrate, (uint8_t)SERIAL_8N1);
}

void uart_rx_handler(void *p)
{
  Uart *u = (Uart *)p;
  if (u)
    u->IrqHandler();
}

void Uart::begin(unsigned long baudrate, uint16_t config)
{
  pinPeripheral(uc_pinRX, PIO_UART);
  pinPeripheral(uc_pinTX, PIO_UART);

  uart.baudRate = (eUART_BAUDRATE)baudrate;
  if (config & HARDSER_DATA_5) {
    uart.wordLength = UART_WORDLENGTH_5BIT;
  } else if (config & HARDSER_DATA_6) {
    uart.wordLength = UART_WORDLENGTH_6BIT;
  } else if (config & HARDSER_DATA_7) {
    uart.wordLength = UART_WORDLENGTH_7BIT;
  } else {
    uart.wordLength = UART_WORDLENGTH_8BIT;
  }

  UART_Init(&uart);

  UART_SetParity(&uart, (eUART_PARITY)(config & HARDSER_PARITY_MASK));

  UART_SetStopbit(&uart, (eUART_STOPBIT)((config & HARDSER_STOP_BIT_MASK) >> 4));

  // parity, stopbit
//  UART_EnableFifo(&uart, UART_FIFO_LEVEL_7_OVER_8, UART_FIFO_LEVEL_1_OVER_8);
  UART_DisableFifo(&uart);
  UART_EnableUart(&uart, UART_MODE_TX_RX);
  UART_EnableInterrupt(&uart, UART_MODE_RX, NULL, NULL, uart_rx_handler, (void *)this);
}

void Uart::end()
{
  rxBuffer.clear();
}

void Uart::flush()
{
}

void Uart::IrqHandler()
{
  //if (UART_DataAvailable(&uart))  {
    uart.reg->imsc &= ~UART_IMSC_RXIM;
    rxBuffer.store_char(UART_ReceiveChar(&uart));
    uart.reg->imsc |= UART_IMSC_RXIM;
  //}

  /*
  if (sercom->isUARTError()) {
    sercom->acknowledgeUARTError();
    // TODO: if (sercom->isBufferOverflowErrorUART()) ....
    // TODO: if (sercom->isFrameErrorUART()) ....
    // TODO: if (sercom->isParityErrorUART()) ....
    sercom->clearStatusUART();
  }
  */
}

int Uart::available()
{
  return rxBuffer.available();
}

int Uart::availableForWrite()
{
    return UART_FifoFull(&uart) ? 0: 1;
}

int Uart::peek()
{
  return rxBuffer.peek();
}

int Uart::read()
{
  int v;
  uart.reg->imsc &= ~UART_IMSC_RXIM;
  v = rxBuffer.read_char();
  uart.reg->imsc |= UART_IMSC_RXIM;
  return v;
}

size_t Uart::write(const uint8_t data)
{
  UART_TransmitChar(&uart, data);
  return 1;
}

size_t Uart::write(uint8_t *buf, size_t count)
{
    uint8_t testbuf[UARTBUF_SIZE];
    uint32_t o = 0;

    if (o + count < UARTBUF_SIZE) {
	memcpy(&testbuf[o], buf, count);
	o += count;
    }
    UART_Transmit(&uart, buf, count, 0);
    return count;
}

/* vim: sw=2
 */
