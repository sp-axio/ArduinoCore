/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "SPI.h"
#include "wiring_private.h"


SPIClass::SPIClass(uint32_t _bank, uint8_t _pinMOSI, uint8_t _pinMISO, uint8_t _pinSCK, uint8_t _pinSSN)
{
  memset(&spi, 0, sizeof(spi));
  spi.bank = (eSPI_BANK)_bank;

  spi_pinMOSI = _pinMOSI;
  spi_pinMISO = _pinMISO;
  spi_pinSCK  = _pinSCK;
  spi_pinSSN  = _pinSSN;
}

void SPIClass::begin(eSPI_DATA_SIZE_SELECT df, uint8_t pres) {
	pinPeripheral(spi_pinMOSI, PIO_SPI);
	pinPeripheral(spi_pinMISO, PIO_SPI);
	pinPeripheral(spi_pinSCK,  PIO_SPI);
	pinPeripheral(spi_pinSSN,  PIO_SPI);

	init(df, pres);
}

void SPIClass::init(eSPI_DATA_SIZE_SELECT df, uint8_t pres) {
	spi.dataFormat = df;
	spi.prescale   = pres; /* SPI clock = 48MHz / (8 * (1 + prescale)) */

	SPI_Init(&spi);
	SPI_DataSize(&spi);
	SPI_PreScale(&spi);
}


void SPIClass::beginTransaction(SPISettings settings)
{
	spi.clockPolarity = 0;
	spi.clockPhase    = 0;

	SPI_ClockPolarity(&spi);
	SPI_ClockPhase(&spi);
}


void SPIClass::endTransaction(void)
{
}

void SPIClass::end() {
	SPI_DeInit(&spi);
}


byte SPIClass::transfer(uint8_t _data) {
	SPI_Write(&spi, _data);
	return SPI_Read(&spi);
}


