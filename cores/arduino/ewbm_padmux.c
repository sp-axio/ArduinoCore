/**
  ******************************************************************************
  * @file    ewbm_padmux.c
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   Source file of padmux setting
  ******************************************************************************
  * @attention
  *
  * Copyright(c) 2015 ~ 2020 eWBM Korea , Ltd.
  * All rights reserved.
  * This software contains confidential information of eWBM Korea Co.,Ltd.
  * and unauthorized distribution of this software, or any portion of it, are
  * prohibited.
  *
  ******************************************************************************
  */

#include "ewbm_padmux.h"
#include "padmux_config.h"
#include "ewbm_clock.h"

#include <stdint.h>


#ifndef SYS_PADMUX
    #define SYS_PADMUX      NON
#endif

#ifndef SPI_PADMUX
    #define SPI_PADMUX      NON
#endif

#ifndef UART_PADMUX
    #define UART_PADMUX     NON
#endif

#ifndef I2C_PADMUX
    #define I2C_PADMUX      NON
#endif

#ifndef TIMER_PADMUX
    #define TIMER_PADMUX    NON
#endif

#ifndef SDIO_PADMUX
    #define SDIO_PADMUX     NON
#endif


static void padmux_factory_init(void)
{
    SYSCON->pa_port_mode = 0x0000F0C9;
    SYSCON->pa_mfsel_lo  = 0x00000000;
    SYSCON->pa_mfsel_hi  = 0x00000000;

    SYSCON->pb_port_mode = 0x00000000;
    SYSCON->pb_mfsel_lo  = 0x00000000;
    SYSCON->pb_mfsel_hi  = 0x00000000;

    SYSCON->pc_port_mode = 0x00000000;
    SYSCON->pc_mfsel_lo  = 0x00000000;
    SYSCON->pc_mfsel_hi  = 0x00000000;
}

static void padmux_set(uint8_t* padmux, uint8_t mode)
{
    uint32_t portId;
    uint32_t pinNum;;

    if ( padmux[0] == NON )
        return;

    for ( portId=0; portId<PADMUX_PORT_MAX; portId++ )
    {
        volatile uint32_t* sysconPort = (volatile uint32_t*)(&SYSCON->pa_port_mode + portId);
        volatile uint32_t* sysconMf   = (volatile uint32_t*)(&SYSCON->pa_mfsel_lo + (portId<<1));
        uint32_t portMode = 0;
        uint32_t mfselLo  = 0;
        uint32_t mfselHi  = 0;

        uint8_t pinLoStart = PA0  + (portId*16);
        uint8_t pinLoEnd   = PA7  + (portId*16);
        uint8_t pinHiStart = PA8  + (portId*16);
        uint8_t pinHiEnd   = PA15 + (portId*16);

        for ( pinNum=0; pinNum<PIN_MAX; pinNum++ )
        {
            if ( padmux[pinNum] == NON )
                break;

            if (( pinLoStart <= padmux[pinNum] ) && ( padmux[pinNum] <= pinLoEnd ))
            {
                uint32_t shiftValue = padmux[pinNum] - (portId*16);
                mfselLo  |= (mode << (shiftValue*4));
                portMode |= (1 << shiftValue);
            }

            if (( pinHiStart <= padmux[pinNum] ) && ( padmux[pinNum] <= pinHiEnd ))
            {
                uint32_t shiftValue = padmux[pinNum] - 8 - (portId*16);
                mfselHi  |= (mode << (shiftValue*4));
                portMode |= (1 << (shiftValue+8));
            }
        }

        sysconPort[0] |= portMode;
        sysconMf[0]   |= mfselLo;
        sysconMf[1]   |= mfselHi;
    }
}

void padmux_init(void)
{
    uint8_t sysPadmux[PIN_MAX+1]   = {SYS_PADMUX,NON,};
    uint8_t spiPadmux[PIN_MAX+1]   = {SPI_PADMUX,NON,};
    uint8_t uartPadmux[PIN_MAX+1]  = {UART_PADMUX,NON,};
    uint8_t i2cPadmux[PIN_MAX+1]   = {I2C_PADMUX,NON,};
    uint8_t timerPadmux[PIN_MAX+1] = {TIMER_PADMUX,NON,};
    uint8_t sdioPadmux[PIN_MAX+1]  = {SDIO_PADMUX,NON,};

    padmux_factory_init();

    padmux_set(sysPadmux,   PADMUX_MODE_SYS);
    padmux_set(spiPadmux,   PADMUX_MODE_SPI);
    padmux_set(uartPadmux,  PADMUX_MODE_UART);
    padmux_set(i2cPadmux,   PADMUX_MODE_I2C);
    padmux_set(timerPadmux, PADMUX_MODE_TIMER);
    padmux_set(sdioPadmux,  PADMUX_MODE_SDIO);

    // Peri clock all reset (except syscon)
    CLKRST->peri_rst = 0x3EF7707D;
    while(!(CLKRST->peri_rst & 0x3EF7707D));
}


