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
#pragma once
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef enum _EPioType
{
  PIO_NOT_A_PIN=-1,     /* Not under control of a peripheral. */
  PIO_GPIO=0,         
  PIO_SPI,           
  PIO_UART,           
  PIO_I2C,       
  PIO_TIMER,            
  PIO_SDIO,        
  PIO_INPUT,
  PIO_INPUT_PULLUP,
  PIO_OUTPUT,
  PIO_DIGITAL = PIO_GPIO,
  PIO_MAX,
} EPioType ;


#ifdef __cplusplus
} // extern "C"
#endif
