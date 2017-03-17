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

#pragma once

#include "Arduino.h"
#include "wiring_constants.h"
#include "ewbm_spi.h"

#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01

#define AXIO_BUILDER 

class SPISettings {
public:
	SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
		if (__builtin_constant_p(clock)) {
			init_AlwaysInline(clock, bitOrder, dataMode);
		} else {
			init_MightInline(clock, bitOrder, dataMode);
		}
	}
	SPISettings() { init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0); }
private:
	void init_MightInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
		init_AlwaysInline(clock, bitOrder, dataMode);
	}
	void init_AlwaysInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {
		spi_mode = dataMode;
	}
//	uint32_t config;
	uint8_t spi_mode;
	BitOrder border;
	friend class SPIClass;
};



class SPIClass {
  public:
    SPIClass(uint32_t bank, uint8_t _mosi, uint8_t _miso, uint8_t _sck, uint8_t _ssn);
	// Transfer functions
	byte transfer(uint8_t _data); 

	void beginTransaction(SPISettings settings); 
	void endTransaction(void);

	void begin(void) { begin(SPI_DSS_08_BIT_DATA, 0);}
	void begin(eSPI_DATA_SIZE_SELECT df, uint8_t prescale);
	void end(void);

  private:
	void init(eSPI_DATA_SIZE_SELECT df, uint8_t prescale);
	tSPI_HANDLE spi; 

    uint8_t spi_pinMOSI;
    uint8_t spi_pinMISO;
    uint8_t spi_pinSCK;
    uint8_t spi_pinSSN;
};

extern SPIClass SPI;
