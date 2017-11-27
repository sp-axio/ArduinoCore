/*
 * Copyright 2017 Security Platform Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "Arduino.h"
#include "variant.h"

#ifdef __cplusplus
  extern "C" {
#endif

void delay( uint32_t dwMs ) 
{
	__m_delay(dwMs);
}

#ifdef __cplusplus
  }
#endif

Uart Serial(UART_BANK_2, D0, D1);
Uart Serial1(UART_BANK_1, D6, D7);
Uart Serial3(UART_BANK_3, D4, D5);
SPIClass SPI(SPI_BANK_1, D11, D12, D13, D10);

void serialEvent() __attribute__((weak));
void serialEvent() { }
void serialEvent1() __attribute__((weak));
void serialEvent1() { }
void serialEvent3() __attribute__((weak));
void serialEvent3() { }
void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
  if (Serial1.available()) serialEvent1();
  if (Serial3.available()) serialEvent3();
}
/* vim:ts=4 sw=4
 */
