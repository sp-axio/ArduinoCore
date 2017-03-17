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
#ifndef __VARIANTS_H__
#define __VARIANTS_H__

#include "ewbm_padmux.h"
#include "ewbm_delay.h"

#include "Uart.h"
#include "SPI.h"

#define VARIANT_MCK 100000000

#define A0 PA0
#define A1 PA1
#define A2 PA3
#define A3 PA10
#define A4 PA5
#define A5 PA4

#define D0  PB5
#define D1  PB4
#define D2  PA12
#define D3  PA12
#define D4  PB7
#define D5  PB6
#define D6  PA9
#define D7  PA8
#define D8  PA15
#define D9  PA14
#define D10 PB1
#define D11 PB3
#define D12 PB2
#define D13 PB0

#define SDA PA5
#define SCL PA4

//#define LED_BUILTIN 1

void delay( uint32_t dwMs ) ;

#ifdef __cplusplus

extern Uart Serial;
extern Uart Serial1;

#endif

#endif /* __VARIANTS_H__ */
