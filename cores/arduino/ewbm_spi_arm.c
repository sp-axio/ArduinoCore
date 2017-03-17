/**
  ******************************************************************************
  * @file    ewbm_spi_arm.c
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   SPI module driver
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

/* Header File */
#include "ewbm_spi_arm.h"
#include "ewbm_delay.h"

/** @addtogroup EWBM_Peripheral
  * @{
  */

/** @addtogroup EWBM_SPI_ARM
  * @{
  */

/** @addtogroup SPI_External_Func
  * @{
  */

/** @addtogroup SPI_External_Func_G1
  *  @brief    Initialization and Configuration functions
  * @{
  */

/**
  * @brief  Initializes the Master of SPI according to the specified parameters
  * @param  reg: pointer to a tSPI_REG structure
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_Init(tSPI_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    /* SPI Master Mode */
    reg->cr1 &= ~(SPI_CR1_MS);

    /* SPI Frame Format */
    reg->cr0 &= ~(SPI_CR0_FRF_MASK<<4);
    reg->cr0 |= SPI_CR0_FRF(0);

    /* SPI Data Size Select */
    reg->cr0 &= ~SPI_CR0_DSS_MASK;
    reg->cr0 |= SPI_CR0_DSS(SPI_DSS_16_BIT_DATA);

	reg->cr0 |= SPI_CR0_SPO;
	reg->cr0 |= SPI_CR0_SPH;

    /* SPI Prescale */
    reg->cr0 &= ~(SPI_CR0_SCR_MASK<<8);
    reg->cr0 |= SPI_CR0_SCR(0);

    /* SPI Clock prescale divisor */
    //reg->cpsr |= 0x02;
    reg->cpsr = 0x8;

    /* SPI Enable & Slave-mode output disable */
    reg->cr1 |= (SPI_CR1_SSE | SPI_CR1_SOD);

    return EWBMOK;
}

/**
  * @brief  Initializes the Slave of SPI according to the specified parameters
  * @param  reg: pointer to a tSPI_REG structure.
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_SlaveInit(tSPI_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    /* SPI Slave Mode */
    reg->cr1 |= (SPI_CR1_MS);

    /* SPI Frame Format */
    reg->cr0 &= ~(SPI_CR0_FRF_MASK<<4);
    reg->cr0 |= SPI_CR0_FRF(0);

    /* SPI Data Size Select */
    reg->cr0 &= ~SPI_CR0_DSS_MASK;
    reg->cr0 |= SPI_CR0_DSS(SPI_DSS_16_BIT_DATA);

    /* SPI Prescale */
    reg->cr0 &= ~(SPI_CR0_SCR_MASK<<8);
    reg->cr0 |= SPI_CR0_SCR(0);

    /* SPI Clock prescale divisor */
    reg->cpsr |= 0x2;

    /* Slave-mode output enable */
    reg->cr1 &= ~(SPI_CR1_SOD);

    /* SPI Enable */
    reg->cr1 |= (SPI_CR1_SSE);

    return EWBMOK;
}

/**
  * @brief  DeInitializes the SPI
  * @param  reg: pointer to a tSPI_REG structure.
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_DeInit(tSPI_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    /* SPI Disable */
    reg->cr1 &= ~(SPI_CR1_SSE);

    return EWBMOK;
}

/**
  * @}
  */


/** @addtogroup SPI_Func_G2
  *  @brief    Operation functions
  * @{
  */

/**
  * @brief  Set to Prescale configuration
  * @param  reg: pointer to a tSPI_REG structure.
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_PreScale(tSPI_REG* reg, uint8_t prescale)
{
    if ( reg == NULL )
        return EWBMINVARG;

    reg->cr0 &= ~(SPI_CR0_SCR_MASK<<8);
    reg->cr0 |= SPI_CR0_SCR(prescale);

    return EWBMOK;
}

/**
  * @brief  Set to Clock polarity configuration
  * @param  reg: pointer to a tSPI_REG structure.
  * @param  clockPolarity: select spi clock polarity
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_ClockPolarity(tSPI_REG* reg, uint8_t clockPolarity)
{
    if ( reg == NULL )
        return EWBMINVARG;

    if ( clockPolarity == 1 )
    {
        reg->cr0 |= SPI_CR0_SPO;
    }
    else
    {
        reg->cr0 &= ~SPI_CR0_SPO;
    }

    return EWBMOK;
}

/**
  * @brief  Set to Clock Phase configuration
  * @param  reg: pointer to a tSPI_REG structure.
  * @param  clockPhase: select spi clock phase
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_ClockPhase(tSPI_REG* reg, uint8_t clockPhase)
{
    if ( reg == NULL )
        return EWBMINVARG;

    if ( clockPhase == 1 )
    {
        reg->cr0 |= SPI_CR0_SPH;
    }
    else
    {
        reg->cr0 &= ~SPI_CR0_SPH;
    }

    return EWBMOK;
}

/**
  * @brief  Set to data size
  * @param  reg: pointer to a tSPI_REG structure.
  * @param  frameFormat: select spi data size
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_DataSize(tSPI_REG* reg, eSPI_DATA_SIZE_SELECT dataSize)
{
    if ( reg == NULL )
        return EWBMINVARG;

    reg->cr0 &= ~SPI_CR0_DSS_MASK;
    reg->cr0 |= SPI_CR0_DSS(dataSize);

    return EWBMOK;
}

/**
  * @brief  Set to Frame Format
  * @param  reg: pointer to a tSPI_REG structure.
  * @param  frameFormat: select spi frame format
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_FrameFormat(tSPI_REG* reg, eSPI_FRAME_FORMAT frameFormat)
{
    if ( reg == NULL )
        return EWBMINVARG;

    reg->cr0 &= ~(SPI_CR0_FRF_MASK<<4);
    reg->cr0 |= SPI_CR0_FRF(frameFormat);

    return EWBMOK;
}

/**
  * @brief  Set to mode select
  * @param  reg: pointer to a tSPI_REG structure.
  * @param  mode: select spi mode
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_ModeSelect(tSPI_REG* reg, eSPI_MODE_SELECT mode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case SPI_MASTER_MODE:
            reg->cr1 &= ~SPI_CR1_MS;
            break;

        case SPI_SLAVE_MODE:
            reg->cr1 |= SPI_CR1_MS;
            break;

        default:
            break;
    }

    return EWBMOK;
}

/**
  * @brief  Set to enable of SPI
  * @param  reg: pointer to a tSPI_REG structure.
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_Enable(tSPI_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    reg->cr1 |= SPI_CR1_SSE;

    return EWBMOK;
}

/**
  * @brief  Set to disable of SPI
  * @param  reg: pointer to a tSPI_REG structure.
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_Disable(tSPI_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    reg->cr1 &= ~SPI_CR1_SSE;

    return EWBMOK;
}

/**
  * @brief  Set to interrupt enable
  * @param  reg: pointer to a tSPI_REG structure.
  * @param  mode: interrupt type that want to disable
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_EnableInterrupt(tSPI_REG* reg, eSPI_INTR_MODE mode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case SPI_INTR_MODE_TX:
            reg->imsc |= SPI_IMSC_TXIM;
            break;

        case SPI_INTR_MODE_RX:
            reg->imsc |= SPI_IMSC_RXIM;
            break;

        case SPI_INTR_MODE_TXRX:
            reg->imsc |= (SPI_IMSC_TXIM|SPI_IMSC_RXIM);
            break;

        default:
            return EWBMINVARG;
    }

    return EWBMOK;
}

/**
  * @brief  Set to interrupt disable
  * @param  reg: pointer to a tSPI_REG structure.
  * @param  mode: interrupt type that want to disable
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_DisableInterrupt(tSPI_REG* reg, eSPI_INTR_MODE mode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case SPI_INTR_MODE_TX:
            reg->imsc &= ~SPI_IMSC_TXIM;
            break;

        case SPI_INTR_MODE_RX:
            reg->imsc &= ~SPI_IMSC_RXIM;
            break;

        case SPI_INTR_MODE_RT:
            reg->imsc &= ~SPI_IMSC_RTIM;
            break;

        case SPI_INTR_MODE_ROR:
            reg->imsc &= ~SPI_IMSC_RORIM;
            break;

        case SPI_INTR_MODE_TXRX:
            reg->imsc &= ~(SPI_IMSC_TXIM|SPI_IMSC_RXIM);
            break;

        default:
            return EWBMINVARG;
    }

    return EWBMOK;
}

/**
  * @brief  SPI Check the interrupt Mask.
  * @param  reg: pointer to a tSPI_REG structure.
  * @retval return to interrupt flag according to interrupt type.
  */
eSPI_INTR_MODE SPI_ARM_GetIntrMask(tSPI_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    /* Get current interrupt mask */
    if ( reg->mis & SPI_MIS_TXMIS )
    {
        return SPI_INTR_MODE_TX;
    }
    else
    if ( reg->mis & SPI_MIS_RXMIS )
    {
        return SPI_INTR_MODE_RX;
    }

    return SPI_INTR_MODE_MAX;
}

/**
  * @brief  Set to interrupt clear
  * @param  reg: pointer to a tSPI_REG structure.
  * @param mode : interrupt type that want to clear
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_IntrClear(tSPI_REG* reg, eSPI_INTR_MODE mode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    if ( mode == SPI_INTR_MODE_RT )
    {
        /* Clear RT interrupt */
        reg->icr |= SPI_ICR_RTIC;
    }
    else
    if ( mode == SPI_INTR_MODE_ROR )
    {
        /* Clear ROR interrupt */
        reg->icr |= SPI_ICR_RORIC;
    }

    return EWBMOK;
}

/**
  * @brief  set DMA enable for SPI
  * @param  reg: pointer to a tSPI_REG structure. module
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_DmaEnable(tSPI_REG* reg, eSPI_DMA_MODE dmaMode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    switch ( dmaMode )
    {
        case SPI_DMA_MODE_TX:
            reg->dmacr |= SPI_DMACR_TXDMAE;
            break;

        case SPI_DMA_MODE_RX:
            reg->dmacr |= SPI_DMACR_RXDMAE;
            break;

        case SPI_DMA_MODE_TXRX:
            reg->dmacr |= (SPI_DMACR_TXDMAE|SPI_DMACR_RXDMAE);
            break;

        default:
            break;
    }

    return EWBMOK;
}

/**
  * @brief  set DMA disable for SPI
  * @param  reg: pointer to a tSPI_REG structure. module
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_DmaDisable(tSPI_REG* reg, eSPI_DMA_MODE dmaMode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    switch ( dmaMode )
    {
        case SPI_DMA_MODE_TX:
            reg->dmacr &= ~SPI_DMACR_TXDMAE;
            break;

        case SPI_DMA_MODE_RX:
            reg->dmacr &= ~SPI_DMACR_RXDMAE;
            break;

        case SPI_DMA_MODE_TXRX:
            reg->dmacr &= ~(SPI_DMACR_TXDMAE|SPI_DMACR_RXDMAE);
            break;

        default:
            break;
    }

    return EWBMOK;
}

/**
  * @brief  Transmit data
  * @param  reg: pointer to a tSPI_REG structure. module
  * @param  data: tx data
  * @retval  if success, return EWBMOK value
  */

int SPI_ARM_Write(tSPI_REG* reg, uint16_t data)
{
    if ( reg == NULL )
        return EWBMINVARG;

    while ( SPI_ARM_Status(reg, SPI_PSR_TNF) == 0 )
        ;

    reg->dr = data;

//    while ( SPI_ARM_Status(reg, SPI_PSR_RNE) == RESET )
//        ;

    return EWBMOK;
}

/**
  * @brief  Receive data
  * @param  reg: pointer to a tSPI_REG structure. module
  * @retval  return received data
  */

uint16_t SPI_ARM_Read(tSPI_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    while ( SPI_ARM_Status(reg, SPI_PSR_RNE) == 0 )
        ;

    return reg->dr;
}

/**
  * @brief  Check the Status
  * @param  reg: pointer to a tSPI_REG structure
  * @param  data: counter mode
  * @retval return 1 or 0 value
  */
uint8_t SPI_ARM_Status(tSPI_REG* reg, uint8_t data)
{
    if ( (reg->sr & data) != (uint16_t)0 )
        return 1;

    return 0;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

