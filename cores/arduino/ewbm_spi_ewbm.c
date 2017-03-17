/**
  ******************************************************************************
  * @file    ewbm_spi_ewbm.c
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
#include "ewbm_spi_ewbm.h"
#include "ewbm_delay.h"

/** @addtogroup EWBM_Peripheral
  * @{
  */

/** @addtogroup EWBM_SPI_EWBM
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
  * @param  reg: pointer to a tSPI3_REG structure
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_Init(tSPI3_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    /* SPI Master Mode */
    reg->ctrl &= ~(SPI3_CR_MST_SLV);

    /* SPI Frame Format */
    reg->ctrl &= ~(SPI3_CR_FRM_FORMAT_MASK<<2);
    reg->ctrl |= SPI3_CR_FRM_FORMAT(0);

    /* SPI Data Size Select */
    reg->ctrl &= ~(SPI3_CR_DATA_SIZE_MASK<<8);
    reg->ctrl |= SPI3_CR_DATA_SIZE(SPI_DSS_16_BIT_DATA);

    /* SPI Prescale */
    reg->ctrl &= ~(SPI3_CR_BAUDRATE_MASK<<12);
    reg->ctrl |= SPI3_CR_BAUDRATE(0);

    /* SPI Clock prescale divisor */
    reg->ext_ctrl &= ~(SPI3_EXT_CLK_PRESC_MASK<<8);
    reg->ext_ctrl |= SPI3_EXT_CLK_PRESC(0x5);

    /* SPI data output enable*/
    reg->ctrl &= ~ SPI3_CR_SDO_EN;
    /* SPI Enable & Slave-mode output disable */
    reg->ctrl |= SPI3_CR_EN;
    reg->ext_ctrl |= SPI3_EXT_SSEL_DIS;
    return EWBMOK;
}

/**
  * @brief  Initializes the Slave of SPI according to the specified parameters
  * @param  reg: pointer to a tSPI3_REG structure.
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_SlaveInit(tSPI3_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    /* SPI Slave Mode */
    reg->ctrl |= (SPI3_CR_MST_SLV);

    /* SPI Frame Format */
    reg->ctrl &= ~(SPI3_CR_FRM_FORMAT_MASK<<2);
    reg->ctrl |= SPI3_CR_FRM_FORMAT(0);

    /* SPI Data Size Select */
    reg->ctrl &= ~(SPI3_CR_DATA_SIZE_MASK<<8);
    reg->ctrl |= SPI3_CR_DATA_SIZE(SPI_DSS_16_BIT_DATA);

    /* SPI Prescale */
    reg->ctrl &= ~(SPI3_CR_BAUDRATE_MASK<<12);
    reg->ctrl |= SPI3_CR_BAUDRATE(0);

    /* SPI Clock prescale divisor */
    reg->ext_ctrl &= ~(SPI3_EXT_CLK_PRESC_MASK<<8);
    reg->ext_ctrl |= SPI3_EXT_CLK_PRESC(2);

    /* Slave-mode output enable */
    reg->ext_ctrl &= ~(SPI3_EXT_SSEL_DIS);

   /* SPI Enable */
    reg->ctrl |= SPI3_CR_EN;
    return EWBMOK;
}

/**
  * @brief  DeInitializes the SPI
  * @param  reg: pointer to a tSPI3_REG structure.
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_DeInit(tSPI3_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    /* SPI Disable */
    reg->ctrl &= ~(SPI3_CR_EN);
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
  * @param  reg: pointer to a tSPI3_REG structure.
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_PreScale(tSPI3_REG* reg, uint8_t prescale)
{
    if ( reg == NULL )
        return EWBMINVARG;

    reg->ctrl &= ~(SPI3_CR_BAUDRATE_MASK<<12);
    reg->ctrl |= SPI3_CR_BAUDRATE(prescale);

    return EWBMOK;
}

/**
  * @brief  Set to Clock polarity configuration
  * @param  reg: pointer to a tSPI3_REG structure.
  * @param  clockPolarity: select spi clock polarity
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_ClockPolarity(tSPI3_REG* reg, uint8_t clockPolarity)
{
    if ( reg == NULL )
        return EWBMINVARG;

    if ( clockPolarity == 1 )
        reg->ctrl |= SPI3_CR_CLK_POL;
    else
        reg->ctrl &= ~SPI3_CR_CLK_POL;

    return EWBMOK;
}

/**
  * @brief  Set to Clock Phase configuration
  * @param  reg: pointer to a tSPI3_REG structure.
  * @param  clockPhase: select spi clock phase
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_ClockPhase(tSPI3_REG* reg, uint8_t clockPhase)
{
    if ( reg == NULL )
        return EWBMINVARG;

    if ( clockPhase == 1 )
        reg->ctrl |= SPI3_CR_PHASE;
    else
        reg->ctrl &= ~SPI3_CR_PHASE;

    return EWBMOK;
}

/**
  * @brief  Set to data size
  * @param  reg: pointer to a tSPI3_REG structure.
  * @param  frameFormat: select spi data size
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_DataSize(tSPI3_REG* reg, eSPI_DATA_SIZE_SELECT dataSize)
{
    if ( reg == NULL )
        return EWBMINVARG;

    reg->ctrl &= ~(SPI3_CR_DATA_SIZE_MASK<<8);
    reg->ctrl |= SPI3_CR_DATA_SIZE(dataSize);

    return EWBMOK;
}

/**
  * @brief  Set to Frame Format
  * @param  reg: pointer to a tSPI3_REG structure.
  * @param  frameFormat: select spi frame format
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_FrameFormat(tSPI3_REG* reg, eSPI_FRAME_FORMAT frameFormat)
{
    if ( reg == NULL )
        return EWBMINVARG;

    reg->ctrl &= ~(SPI3_CR_FRM_FORMAT_MASK<<2);
    reg->ctrl |= SPI3_CR_FRM_FORMAT(frameFormat);

    return EWBMOK;
}

/**
  * @brief  Set to mode select
  * @param  reg: pointer to a tSPI3_REG structure.
  * @param  mode: select spi mode
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_ModeSelect(tSPI3_REG* reg, eSPI_MODE_SELECT mode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case SPI_MASTER_MODE:
            reg->ctrl &= ~(SPI3_CR_MST_SLV);
            break;

        case SPI_SLAVE_MODE:
            reg->ctrl |= (SPI3_CR_MST_SLV);
            break;

        default:
            break;
    }

    return EWBMOK;
}

/**
  * @brief  Set to enable of SPI
  * @param  reg: pointer to a tSPI3_REG structure.
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_Enable(tSPI3_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    reg->ctrl |= SPI3_CR_EN;

    return EWBMOK;
}

/**
  * @brief  Set to disable of SPI
  * @param  reg: pointer to a tSPI3_REG structure.
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_Disable(tSPI3_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    reg->ctrl &= ~SPI3_CR_EN;
    return EWBMOK;
}

/**
  * @brief  Set to interrupt enable
  * @param  reg: pointer to a tSPI3_REG structure.
  * @param  mode: interrupt type that want to disable
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_EnableInterrupt(tSPI3_REG* reg, eSPI_INTR_MODE mode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case SPI_INTR_MODE_TX:
            reg->int_en |= SPI3_INT_TX_FIFO_HE_EN;
            break;

        case SPI_INTR_MODE_RX:
            reg->int_en |= SPI3_INT_RX_FIFO_HF_EN;
            break;

        case SPI_INTR_MODE_TXRX:
            reg->int_en |= SPI3_INT_TX_FIFO_HE_EN|SPI3_INT_RX_FIFO_HF_EN;
            break;

        default:
            return EWBMINVARG;
    }

    return EWBMOK;
}

/**
  * @brief  Set to interrupt disable
  * @param  reg: pointer to a tSPI3_REG structure.
  * @param  mode: interrupt type that want to disable
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_DisableInterrupt(tSPI3_REG* reg, eSPI_INTR_MODE mode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    switch ( mode )
    {
        case SPI_INTR_MODE_TX:
            //-->reg->imsc &= ~SPI_IMSC_TXIM;
            reg->int_en &= ~SPI3_INT_TX_FIFO_HE_EN;
            break;

        case SPI_INTR_MODE_RX:
            //reg->imsc &= ~SPI_IMSC_RXIM;
            reg->int_en &= ~SPI3_INT_RX_FIFO_HF_EN;
            break;

        case SPI_INTR_MODE_RT:
            reg->int_en &= ~SPI3_INT_TX_TOUT;
            break;

        case SPI_INTR_MODE_ROR:
            reg->int_en &= ~SPI3_INT_RX_OVR;
            break;

        case SPI_INTR_MODE_TXRX:
            reg->int_en &= ~(SPI3_INT_TX_FIFO_HE_EN|SPI3_INT_RX_FIFO_HF_EN);
            break;

        default:
            return EWBMINVARG;
    }

    return EWBMOK;
}

/**
  * @brief  SPI Check the interrupt Mask.
  * @param  reg: pointer to a tSPI3_REG structure.
  * @retval return to interrupt flag according to interrupt type.
  */
eSPI_INTR_MODE SPI_EWBM_GetIntrMask(tSPI3_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    /* Get current interrupt mask */
    if ( reg->int_sts_clr & SPI3_STS_TX_FIFO_RDY )
    {
        return SPI_INTR_MODE_TX;
    }
    else
    if ( reg->int_sts_clr & SPI3_STS_RX_FIFO_RDY )
    {
        return SPI_INTR_MODE_RX;
    }

    return SPI_INTR_MODE_MAX;
}

/**
  * @brief  Set to interrupt clear
  * @param  reg: pointer to a tSPI3_REG structure.
  * @param mode : interrupt type that want to clear
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_IntrClear(tSPI3_REG* reg, eSPI_INTR_MODE mode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    if ( ( mode == SPI_INTR_MODE_RT ) || ( mode == SPI_INTR_MODE_RX) )
    {
        /* Clear RT interrupt */
        reg->int_sts_clr |= SPI3_INT_RX_CLR;
    }
    else
    if ( mode == SPI_INTR_MODE_ROR )
    {
        /* Clear ROR interrupt */
        reg->int_sts_clr |= SPI3_INT_TX_CLR;
    }

    return EWBMOK;
}

/**
  * @brief  set DMA enable for SPI
  * @param  reg: pointer to a tSPI3_REG structure. module
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_DmaEnable(tSPI3_REG* reg, eSPI_DMA_MODE dmaMode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    switch ( dmaMode )
    {
        case SPI_DMA_MODE_TX:
            reg->ext_ctrl |= SPI3_EXT_TX_DMA_EN;
            break;

        case SPI_DMA_MODE_RX:
            reg->ext_ctrl |= SPI3_EXT_RX_DMA_EN;
            break;

        case SPI_DMA_MODE_TXRX:
            reg->ext_ctrl |= (SPI3_EXT_TX_DMA_EN|SPI3_EXT_RX_DMA_EN);
            break;

        default:
            break;
    }

    return EWBMOK;
}

/**
  * @brief  set DMA disable for SPI
  * @param  reg: pointer to a tSPI3_REG structure. module
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_DmaDisable(tSPI3_REG* reg, eSPI_DMA_MODE dmaMode)
{
    if ( reg == NULL )
        return EWBMINVARG;

    switch ( dmaMode )
    {
        case SPI_DMA_MODE_TX:
            reg->ext_ctrl &= ~SPI3_EXT_TX_DMA_EN;
            break;

        case SPI_DMA_MODE_RX:
            reg->ext_ctrl &= ~SPI3_EXT_RX_DMA_EN;
            break;

        case SPI_DMA_MODE_TXRX:
            reg->ext_ctrl &= ~(SPI3_EXT_TX_DMA_EN|SPI3_EXT_RX_DMA_EN);
            break;

        default:
            break;
    }

    return EWBMOK;
}

/**
  * @brief  Transmit data
  * @param  reg: pointer to a tSPI3_REG structure. module
  * @param  data: tx data
  * @retval  if success, return EWBMOK value
  */

int SPI_EWBM_Write(tSPI3_REG* reg, uint16_t data)
{
    if ( reg == NULL )
        return EWBMINVARG;

    while ( SPI_EWBM_Status(reg, SPI3_STS_TX_FIFO_EMPTY) == 0 )
        ;

    reg->data = data;

//    while ( SPI_EWBM_Status(reg, SPI3_STS_RX_FIFO_FULL) == RESET )
//        ;

    return EWBMOK;
}

/**
  * @brief  Receive data
  * @param  reg: pointer to a tSPI3_REG structure. module
  * @retval  return received data
  */

uint16_t SPI_EWBM_Read(tSPI3_REG* reg)
{
    if ( reg == NULL )
        return EWBMINVARG;

    while ( SPI_EWBM_Status(reg, SPI3_STS_RX_FIFO_RDY) == 0 )
        ;

    return reg->data;
}

/**
  * @brief  Check the Status
  * @param  reg: pointer to a tSPI3_REG structure
  * @param  data: counter mode
  * @retval return SET or RESET value
  */
uint8_t SPI_EWBM_Status(tSPI3_REG* reg, uint8_t data)
{
    if ( (reg->state & data) != (uint16_t)0 )
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

