/**
  ******************************************************************************
  * @file    ewbm_reg.h
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   Header file of eWBM MS500 reg
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

#ifndef __EWBM_REG_H__
#define __EWBM_REG_H__

/**
 * @name On Chip Memory
 * @{
 */

/**
* @beief     SRAM Address map define
*/
#define ON_CHIP_SRAM1                   (0x10000000)
#define ON_CHIP_SRAM2                   (0x10010000)
#define ON_CHIP_SRAM3                   (0x10020000)
#define ON_CHIP_BOOT_MEM                (0x1FFFE000)
#define BSPI_FREG_BASE                  (0x31000000)

/**@}*/

/**
 * @name APB1 Peripheral Address
 * @{
 */
#define APB1_PERI_BASE                      (0x40000000)
#define APB_SYSTEM_CONTROL                  (APB1_PERI_BASE + 0x0000)
#define APB_OTP_MEM_CONTROL                 (APB1_PERI_BASE + 0x0800)
#define APB_CLOCK_RESET                     (APB1_PERI_BASE + 0x1000)
#define APB_WATCHDOG_TMR                    (APB1_PERI_BASE + 0x2000)
#define APB_GP_TIMER1                       (APB1_PERI_BASE + 0x3000)
#define APB_GP_TIMER2                       (APB1_PERI_BASE + 0x3800)
#define APB_SPI1                            (APB1_PERI_BASE + 0x4000)
#define APB_SPI2                            (APB1_PERI_BASE + 0x5000)
#define APB_UART1                           (APB1_PERI_BASE + 0x6000)
#define APB_UART2                           (APB1_PERI_BASE + 0x7000)
#define APB_I2C1                            (APB1_PERI_BASE + 0x8000)
#define APB_I2C2                            (APB1_PERI_BASE + 0x9000)
#define APB_I2C3                            (APB1_PERI_BASE + 0xA000)
#define APB_I2C4                            (APB1_PERI_BASE + 0xB000)
#define APB_SPI3                            (APB1_PERI_BASE + 0xC000)
#define APB_UART3                           (APB1_PERI_BASE + 0XD000)
#define APB_PMU                             (APB1_PERI_BASE + 0xE000)
#define APB_RTC                             (APB1_PERI_BASE + 0xF000)
#define APB_BATTERY_SRAM                    (APB1_PERI_BASE + 0xF800)
/**@}*/


/**
 * @name AHB Peripheral Address Map
 * @{
 */
#define AHB_PERI_BASE                       (0x40020000)
#define AHB_GPIO1                           (AHB_PERI_BASE + 0x0000)
#define AHB_GPIO2                           (AHB_PERI_BASE + 0x1000)
#define AHB_GPIO3                           (AHB_PERI_BASE + 0x2000)
/**@}*/



/**
 * @name AHB Control Address Map
 * @{
 */
#define AHB_CNTR_BASE                   (0x50000000)
#define AHB_DMA                         (AHB_CNTR_BASE + 0x00000)
#define AHB_SDIO_MASTER                 (AHB_CNTR_BASE + 0x03000)
#define AHB_SDIO_SLAVE                  (AHB_CNTR_BASE + 0x04000)
#define AHB_SDIO_CIS_SLAVE              (AHB_CNTR_BASE + 0x05000)
#define AHB_TRNG                        (AHB_CNTR_BASE + 0x06000)
#define AHB_SPACC                       (AHB_CNTR_BASE + 0x10000)
/**@}*/

/**
 * @name ICACHE Control Address Map
 * @{
 */
#define ICACHE_BASE                     (0xFFFFF000)
/**@}*/

#define __RO      volatile const  // read only
#define __WO      volatile        // write only
#define __RW      volatile        // read-write
#define __ROAC    volatile        // read only, automatic clear
#define __WOAC    volatile        // write only, automatic clear
#define __RWAC    volatile        // read-write, automatic clear
#define __RW1C    volatile        // Read-only status, Write-1-to-clear status
#define __RWOAC   volatile          // read-write, automatic clear



/* Data Structure */
typedef struct _tSYSCON_REG {
    __RO    uint32_t    status;
    __RW    uint32_t    shadow_map;
    __RW    uint32_t    sys_ctrl;
    __RW    uint32_t    pwrl_ctrl;
    __RW    uint32_t    nmi_sts;
    __RW    uint32_t    nmi_en;
    __RO    uint32_t    sys_int_sts;
    __RW    uint32_t    sys_int_en;
    __RW    uint32_t    pa_port_mode;
    __RW    uint32_t    pb_port_mode; // 0x24
    __RW    uint32_t    pc_port_mode;
    __RW    uint32_t    pd_port_mode;
    __RW    uint32_t    pe_port_mode;
            uint32_t    reserved0[3];
    __RW    uint32_t    pa_drv_str;
    __RW    uint32_t    pb_drv_str;
    __RW    uint32_t    pc_drv_str;
    __RW    uint32_t    pd_drv_str;
    __RW    uint32_t    pe_drv_str;
            uint32_t    reserved1[3];
    __RW    uint32_t    pa_pupd_en;
    __RW    uint32_t    pb_pupd_en;
    __RW    uint32_t    pc_pupd_en;
    __RW    uint32_t    pd_pupd_en;
    __RW    uint32_t    pe_pupd_en;
            uint32_t    reserved2[3];
    __RW    uint32_t    pa_mfsel_lo;
    __RW    uint32_t    pa_mfsel_hi;
    __RW    uint32_t    pb_mfsel_lo;
    __RW    uint32_t    pb_mfsel_hi;
    __RW    uint32_t    pc_mfsel_lo;
    __RW    uint32_t    pc_mfsel_hi;
    __RW    uint32_t    pd_mfsel_lo;
    __RW    uint32_t    pd_mfsel_hi;
    __RW    uint32_t    pe_mfsel_lo;
    __RW    uint32_t    pe_mfsel_hi;
            uint32_t    reserved3[6];
    __RW    uint32_t    port_lock;
    __RW    uint32_t    event_router;
            uint32_t    reserved4;
    __RW    uint32_t    analog_trim;
    __RW    uint32_t    pdv_sel18;
    __RW    uint32_t    sb_cfg;
    __RW    uint32_t    miscellounious_if;
    __RW    uint32_t    pab_smt;
    __RW    uint32_t    pcd_smt;
            uint32_t    reserved5[3];
    __RO    uint32_t    chip_id[4];
} tSYSCON_REG;

#define SYSCON          ((tSYSCON_REG*)APB_SYSTEM_CONTROL)

#define PDV_SEL18_MAIN        (1<<0)
#define PDV_SEL18_SDIO_S      (1<<1)
#define PDV_SEL18_SDIO_M      (1<<2)
#define PDV_SEL18_RTC         (1<<4)


/***********************************************************************************************
 *                                      CLKRST START                                           *
************************************************************************************************/
typedef struct _tCLKRST_REG
{
     __I  uint32_t clk_sts;             /* 0x00 : Clock Status */
     __IO uint32_t clk_int_en;          /* 0x04 : Clock Interrupt Enable */
     __IO uint32_t clk_int_sts_clr;     /* 0x08 : Clock Interrupt Staus & Clear */
     __I  uint32_t reserved0;           /* 0x0C */
     __IO uint32_t clk_ctrl;
     __IO uint32_t reserved1;
     __IO uint32_t pll_ctrl;
     __IO uint32_t reserved2;
     __IO uint32_t diva_ctrl;
     __IO uint32_t divb_ctrl;
     __IO uint32_t divc_ctrl;
     __IO uint32_t divd_ctrl;
     __IO uint32_t sys_clk_ctrl;
     __IO uint32_t spif_clk_ctrl;
     __IO uint32_t st_clk_ctrl;
     __IO uint32_t tmr_clk_ctrl;
     __IO uint32_t clk_ctrl_lock;
     __IO uint32_t peri_clk_sel;
     __IO uint32_t peri_clk_en;
     __IO uint32_t peri_clk_lpen;
     __I  uint32_t reserved3[3];
     __IO uint32_t aon_clkrst_ctrl;
     __IO uint32_t rst_state;
     __IO uint32_t grp_rst_ctrl;
     __IO uint32_t peri_rst;
     __I  uint32_t reserved4[4];
     __I  uint32_t peri_id;
} tCLKRST_REG;

#define CLKRST      ((tCLKRST_REG*)APB_CLOCK_RESET)

/* Clock Status */
#define XTAL_RDY            (1<<1)
#define PLL_LOCK            (1<<4)
#define PLL_UNLOCK          (1<<5)
#define DIVA_RDY            (1<<8)
#define DIVB_RDY            (1<<9)
#define DIVC_RDY            (1<<10)
#define DIVD_RDY            (1<<11)
#define AHB_CLK_RDY         (1<<16)
#define CPU_CLK_RDY         (1<<17)
#define APB_CLK_RDY         (1<<18)
#define ST_CLK_RDY          (1<<20)
#define TMR_CLK_RDY         (1<<22)

/* Clock Source */
#define CLK_SRC_LS_IRC      (0x00)
#define CLK_SRC_HS_IRC      (0x01)
#define CLK_SRC_XTAL        (0x02)
#define CLK_SRC_PLL         (0x03)
#define CLK_SRC_DIVA        (0x04)
#define CLK_SRC_DIVB        (0x05)
#define CLK_SRC_DIVC        (0x06)
#define CLK_SRC_DIVD        (0x07)

#define XTAL_EN             (1<<0)
#define XTAL_BYPASS         (1<<1)
#define HS_IRC_EN           (1<<2)

/* System Clock */
#define SYS_CLK_SEL(x)      ((x)<<0)
#define CPU_PRES(x)         ((x)<<4)
#define APB_PRES(x)         ((x)<<8)
#define AHB_CLK_LPEN        (1<<17)
#define APB_CLK_LPEN        (1<<18)
#define SRAM_CLK_LPEN       (1<<24)
#define DS_CLK_MODE(x)      ((x)<<30)

/* SPIF Clock */
#define SPIF_CLK_SEL(x)     (x<<0)
#define SPIF_CLK_EN         (1<<4)
#define SPIF_CLK_LPEN       (1<<5)

/* SysTick Clock */
#define ST_CLK_PRES(x)      (x<<4)
#define ST_CLK_EN           (1<<6)

/* Timer Clock */
#define TMR_CLK_EN(x)       (x<<8)
#define TMR_CLK_LPEN(x)     (x<<16)

/* PLL Clock */
#define PLL_CLK_SEL(x)      (x<<0)
#define PLL_EN              (1<<5)
#define PLL_BYPS            (1<<6)
#define PLL_RST             (1<<7)
#define PLL_FB_DIV(x)       (x<<8)
#define PLL_PRE_DIV(x)      (x<<16)
#define PLL_POST_DIV(x)     (x<<20)
#define PLL_BWA(x)          (x<<24)

/* DIVx Clock */
#define DIVx_CLK_SEL(x)     (x<<0)
#define DIVx_VAL(x)         (x<<8)
#define DIVD_EN             (1<<14)

/* Peri Clock Select */
#define UART_CLK_SEL(x)     (x<<8)
#define SPI_CLK_SEL(x)      (x<<12)
#define SDMMC_CLK_SEL(x)    (x<<16)
#define EXTO1_CLK_SEL(x)    (x<<24)
#define EXTO2_CLK_SEL(x)    (x<<28)

/* Peri Clock Enable */
#define DMAC_CLK_EN         (1<<0)
#define WDT_CLK_EN          (1<<3)
#define GPIOx_CLK_EN(x)     (x<<4)
#define UARTx_CLK_EN(x)     (x<<12)
#define SPIx_CLK_EN(x)      (x<<16)
#define I2Cx_CLK_EN(x)      (x<<20)
#define AIRI_CLK_EN         (1<<25)
#define SDSLV_CLK_EN        (1<<26)
#define SDMMC_CLK_EN        (1<<27)
#define CRYP_CLK_EN         (1<<28)
#define PKA_CLK_EN          (1<<29)
#define EXTO_CLK1_EN        (1<<30)
#define EXTO_CLK2_EN        (1<<31)


/***********************************************************************************************
 *                                      UART START                                             *
************************************************************************************************/

// UART Register structure
typedef struct _tUART_REG {
    __RW uint32_t dr;            // UART Data register
    __RW uint32_t rsr_ecr;       // UART Receive status register(read)/Error clear register(write)
         uint32_t reserved0[4];  // Reserved
    __RO uint32_t fr;            // UART Flag register
         uint32_t reserved1;     // Reserved
    __RW uint32_t ilpr;          // UART IrDA low-power counter register
    __RW uint32_t ibrd;          // UART Interger baud rate divisor
    __RW uint32_t fbrd;          // UART Fractional baud rate divisor
    __RW uint32_t lcr_h;         // UART Line Contrl register, High byte
    __RW uint32_t cr;            // UART Control register
    __RW uint32_t ifls;          // UART Interrupt FIFO level select register
    __RW uint32_t imsc;          // UART Interrupt mask set/clear register
    __RO uint32_t ris;           // UART Raw interrupt status register
    __RO uint32_t mis;           // UART Masked interrupt status register
    __WO uint32_t icr;           // UART Interrupt clear register
    __RW uint32_t dmacr;         // UART DMA control register
} tUART_REG;

// UART PeriphID/CellID register
#define UART_PERIPH_ID0(c)    (*((__RO uint32_t*)((uint8_t*)c+0xFE0)))
#define UART_PERIPH_ID1(c)    (*((__RO uint32_t*)((uint8_t*)c+0xFE4)))
#define UART_PERIPH_ID2(c)    (*((__RO uint32_t*)((uint8_t*)c+0xFE8)))
#define UART_PERIPH_ID3(c)    (*((__RO uint32_t*)((uint8_t*)c+0xFEC)))
#define UART_PCELL_ID0(c)     (*((__RO uint32_t*)((uint8_t*)c+0xFF0)))
#define UART_PCELL_ID1(c)     (*((__RO uint32_t*)((uint8_t*)c+0xFF4)))
#define UART_PCELL_ID2(c)     (*((__RO uint32_t*)((uint8_t*)c+0xFF8)))
#define UART_PCELL_ID3(c)     (*((__RO uint32_t*)((uint8_t*)c+0xFFC)))

// UART Address define
#define UART1                 ((tUART_REG*)APB_UART1)
#define UART2                 ((tUART_REG*)APB_UART2)
#define UART3                 ((tUART_REG*)APB_UART3)
#define UART4                 ((tUART_REG*)APB_UART4)

// UART Data register bit define
#define UART_DR_DATA_MASK              (0xFF)  // Receive(read)/Transimit(write) data [7:0]
#define UART_DR_FE                     (1<<8)  // Framing error
#define UART_DR_PE                     (1<<9)  // Parity error
#define UART_DR_BE                     (1<<10)  // Break error
#define UART_DR_OE                     (1<<11)  // Overrun error

// UART Receive status(read)/Error clear(write) register bit define
#define UART_RSR_ECR_FE                (1<<0)  // Framing error
#define UART_RSR_ECR_PE                (1<<1)  // Parity error
#define UART_RSR_ECR_BE                (1<<2)  // Break error
#define UART_RSR_ECR_OE                (1<<3)  // Overrun error

// UART Flag register bit define
#define UART_FR_CTS                    (1<<0)  // Clear to send
#define UART_FR_DSR                    (1<<1)  // Data set ready
#define UART_FR_DCD                    (1<<2)  // Data carrier detect
#define UART_FR_BUSY                   (1<<3)  // UARTFR busy
#define UART_FR_RXFE                   (1<<4)  // Receive FIFO empty
#define UART_FR_TXFF                   (1<<5)  // Transmit FIFO full
#define UART_FR_RXFF                   (1<<6)  // Receive FIFO full
#define UART_FR_TXFE                   (1<<7)  // Transmit FIFO empty
#define UART_FR_RI                     (1<<8)  // Ring indicator

// UART IrDA low-power counter register bit define
#define UART_ILPR_ILPDVSR_MASK         (0xFF)  // 8-bit low power divisor value

// UART Integer baud rate register bit define
#define UART_IBRD_BAUD_DIVINT_MASK     (0xFFFF)  // The integer baud rate divisor

// UART Fractional baud rate register bit define
#define UART_FBRD_BAUD_DIVFRAC_MASK    (0x3F)  // The fractional baud rate divisor

// UART Line control register bit define
#define UART_LCR_H_BRK                 (1<<0)  // Send break
#define UART_LCR_H_PEN                 (1<<1)  // Parity enable
#define UART_LCR_H_EPS                 (1<<2)  // Even parity select
#define UART_LCR_H_STP2                (1<<3)  // Two stop bits select
#define UART_LCR_H_FEN                 (1<<4)  // Enable FIFOs
#define UART_LCR_H_WLEN_MASK           (0x60)  // Word Length [6:5]
#define UART_LCR_H_SPS                 (1<<7)  // Stick parity select

// UART Control register bit define
#define UART_CR_UARTEN                 (1<<0)  // UART enable
#define UART_CR_SIREN                  (1<<1)  // SIR enable
#define UART_CR_SIRLP                  (1<<2)  // IrDA SIR low power mode
#define UART_CR_LBE                    (1<<7)  // Loopback enable
#define UART_CR_TXE                    (1<<8)  // Transmit enable
#define UART_CR_RXE                    (1<<9)  // Receive enable
#define UART_CR_DTR                    (1<<10)  // Datat transmit ready
#define UART_CR_RTS                    (1<<11)  // Request to send
#define UART_CR_OUT1                   (1<<12)  // UART Out1 modem status output
#define UART_CR_OUT2                   (1<<13)  // UART Out2 modem status output
#define UART_CR_RTSEN                  (1<<14)  // RTS hardware flow control enable
#define UART_CR_CTSEN                  (1<<15)  // CTS hardware flow control enable

// UART Interrupt FIFO level select register bit define
#define UART_IFLS_TXIFLSEL_MASK        (0x07)  // Trnasmit interrupt FIFO level select
#define UART_IFLS_RXIFLSEL_MASK        (0x38)  // Receive interrupt FIFO level select

// UART Interrupt mask set/clear register bit define
#define UART_IMSC_RIMIM                (1<<0)  // nUARTRI modem interrupt mask
#define UART_IMSC_CTSMIM               (1<<1)  // nUARTCTS modem interrupt mask
#define UART_IMSC_DCDMIM               (1<<2)  // nUARTDCD modem interrupt mask
#define UART_IMSC_DSRMIM               (1<<3)  // nUARTDSR modem interrupt mask
#define UART_IMSC_RXIM                 (1<<4)  // Receive interrupt mask
#define UART_IMSC_TXIM                 (1<<5)  // Transmit interrupt mask
#define UART_IMSC_RTIM                 (1<<6)  // Receive timeout interrupt mask
#define UART_IMSC_FEIM                 (1<<7)  // Framing error interrupt mask
#define UART_IMSC_PEIM                 (1<<8)  // Parity error interrupt mask
#define UART_IMSC_BEIM                 (1<<9)  // Break error intterupt mask
#define UART_IMSC_OEIM                 (1<<10)  // Overrun error interrupt mask

// UART Raw interrupt status register bit define
#define UART_RIS_RIRMIS                (1<<0)  // nUARTRI modem interrupt status
#define UART_RIS_CTSRMIS               (1<<1)  // nUARTCTS modem interrupt status
#define UART_RIS_DCDRMIS               (1<<2)  // nUARTDCD modem interrupt status
#define UART_RIS_DSRRMIS               (1<<3)  // nUARTDSR modem interrupt status
#define UART_RIS_RXRIS                 (1<<4)  // Receive interrupt status
#define UART_RIS_TXRIS                 (1<<5)  // Transmit interrupt status
#define UART_RIS_RTRIS                 (1<<6)  // Receive timeout interrupt status
#define UART_RIS_FERIS                 (1<<7)  // Framing error interrupt status
#define UART_RIS_PERIS                 (1<<8)  // Parity error interrupt status
#define UART_RIS_BERIS                 (1<<9)  // Break error intterupt status
#define UART_RIS_OERIS                 (1<<10)  // Overrun error interrupt status

// UART Interrupt mask set/clear register bit define
#define UART_MIS_RIMMIS                (1<<0)  // nUARTRI modem masked interrupt status
#define UART_MIS_CTSMMIS               (1<<1)  // nUARTCTS modem masked interrupt status
#define UART_MIS_DCDMMIS               (1<<2)  // nUARTDCD modem masked interrupt status
#define UART_MIS_DSRMMIS               (1<<3)  // nUARTDSR modem masked interrupt status
#define UART_MIS_RXMIS                 (1<<4)  // Receive masked interrupt status
#define UART_MIS_TXMIS                 (1<<5)  // Transmit masked interrupt status
#define UART_MIS_RTMIS                 (1<<6)  // Receive timeout masked interrupt status
#define UART_MIS_FEMIS                 (1<<7)  // Framing error masked interrupt status
#define UART_MIS_PEMIS                 (1<<8)  // Parity error masked interrupt status
#define UART_MIS_BEMIS                 (1<<9)  // Break error masked interrupt status
#define UART_MIS_OEMIS                 (1<<10)  // Overrun error masked interrupt status

// UART Interrupt clear register bit define
#define UART_ICR_RIMIC                 (1<<0)  // nUARTRI modem interrupt clear
#define UART_ICR_CTSMIC                (1<<1)  // nUARTCTS modem interrupt clear
#define UART_ICR_DCDMIC                (1<<2)  // nUARTDCD modem interrupt clear
#define UART_ICR_DSRMIC                (1<<3)  // nUARTDSR modem interrupt clear
#define UART_ICR_RXIC                  (1<<4)  // Receive interrupt clear
#define UART_ICR_TXIC                  (1<<5)  // Transmit interrupt clear
#define UART_ICR_RTIC                  (1<<6)  // Receive timeout interrupt clear
#define UART_ICR_FEIC                  (1<<7)  // Framing error interrupt clear
#define UART_ICR_PEIC                  (1<<8)  // Parity error interrupt clear
#define UART_ICR_BEIC                  (1<<9)  // Break error intterupt clear
#define UART_ICR_OEIC                  (1<<10)  // Overrun error interrupt clear

#define UART_ALL_MASK                  (0x07FF)  // All interrupt mask

// UART DMA control register bit define
#define UART_DMACR_RXDMAE              (1<<0)  // Receive DMA enable
#define UART_DMACR_TXDMAE              (1<<1)  // Transmit DMA enable
#define UART_DMACR_DMAONERR            (1<<2)  // DMA on error

/**************************************************************************************
 * UART Peripheral identification registers
 * The UARTPeriphID0-3 registers are four 8-bit registers, that span address locations
 * 0xFE0 - 0xFEC. The registers can conceptually be treated as a 32-bit register.
 * The read only registers provide the following options of the peripheral
 *
 * PartNumber    [11:0]  This is used to identify the peripheral.
 *                       The three digits product code 0x011 is used.
 * Designer ID   [19:12] This is the identification of the designer.
 *                       ARM Limited is 0x41 (ASCII A).
 * Revision      [23:20] This is the revision number of the peripheral.
 *                       The revision number starts from 0.
 * Configuration [31:24] This is the configuration option of the peripheral.
 *                       The configuration value is 0.
 *************************************************************************************/
#define UART_PERIPH_ID0_PARTNUM_MASK   (0xFF)  // PartNumber[7:0], these bits read back as 0x11
#define UART_PERIPH_ID1_PARTNUM_MASK   (0x0F)  // PartNumber[11:8], these bits read back as 0x0
#define UART_PERIPH_ID1_DESIGNER_MASK  (0xF0)  // Designer[15:12], these bits read back as 0x01
#define UART_PERIPH_ID2_DESIGNER_MASK  (0x0F)  // Designer[19:16], these bits read back as 0x03
#define UART_PERIPH_ID2_REVISION_MASK  (0xF0)  // Revision[23:20], these bits read back as 0x04
#define UART_PERIPH_ID3_CONFIG_MASK    (0xFF)  // Configuration[31:24], these bits read back as 0x00

/*****************************************************************************
 * UART PrimeCell identification registers
 * The UARTPCellID0-3registers are four 8-bit wide registers,
 * that span address locations 0xFF0-0xFFC.
 * The registers can conceptually be treated as a 32-bit register.
 * The register is used as a standard cross-peripheral identification system.
 * The UARTPCellID register is set to 0xB105F00D.
 *****************************************************************************/
#define UART_PCELL_ID0_MASK            (0xFF)  // These bits read back as 0x0D
#define UART_PCELL_ID1_MASK            (0xFF)  // These bits read back as 0xF0
#define UART_PCELL_ID2_MASK            (0xFF)  // These bits read back as 0x05
#define UART_PCELL_ID3_MASK            (0xFF)  // These bits read back as 0xB1


/***********************************************************************************************
 *                                      DMA START                                              *
************************************************************************************************/

// DMAC Register structure
typedef struct _tDMAC_REG {
    __RO uint32_t intstatus;          // DMAC Interrupt status register
    __RO uint32_t inttcstatus;        // DMAC Interrupt terminal count status register
    __WO uint32_t inttcclear;         // DMAC Interrupt terminal count clear register
    __RO uint32_t interrorstatus;     // DMAC Interrupt error status register
    __WO uint32_t interrclr;          // DMAC Interrupt error clear register
    __RO uint32_t rawinttcstatus;     // DMAC Raw interrupt terminal count status register
    __RO uint32_t rawinterrorstatus;  // DMAC Raw error interrupt status register
    __RO uint32_t enbldchns;          // DMAC Enabled channel register
    __RW uint32_t softbreq;           // DMAC Software burst request register
    __RW uint32_t softsreq;           // DMAC Software single request register
    __RW uint32_t softlbreq;          // DMAC Software last burst request register
    __RW uint32_t softlsreq;          // DMAC Software last single request regitser
    __RW uint32_t configuration;      // DMAC Configuration register
    __RW uint32_t sync;               // DMAC Synchronization register
} tDMAC_REG;

// DMAC Channel Control register
#define DMAC_C0_SRCADDR(c)          (*((__RW uint32_t*)((uint8_t*)c+0x100)))  // DMAC Channel0 source address register
#define DMAC_C0_DSTADDR(c)          (*((__RW uint32_t*)((uint8_t*)c+0x104)))  // DMAC Channel0 destination address register
#define DMAC_C0_LLI(c)              (*((__RW uint32_t*)((uint8_t*)c+0x108)))  // DMAC Channel0 linked list item register
#define DMAC_C0_CONTROL(c)          (*((__RW uint32_t*)((uint8_t*)c+0x10C)))  // DMAC Channel0 control register
#define DMAC_C0_CONFIGURATION(c)    (*((__RW uint32_t*)((uint8_t*)c+0x110)))  // DMAC Channel0 configuration reigster
#define DMAC_C1_SRCADDR(c)          (*((__RW uint32_t*)((uint8_t*)c+0x120)))  // DMAC Channel1 source address register
#define DMAC_C1_DSTADDR(c)          (*((__RW uint32_t*)((uint8_t*)c+0x124)))  // DMAC Channel1 destination address register
#define DMAC_C1_LLI(c)              (*((__RW uint32_t*)((uint8_t*)c+0x128)))  // DMAC Channel1 linked list item register
#define DMAC_C1_CONTROL(c)          (*((__RW uint32_t*)((uint8_t*)c+0x12C)))  // DMAC Channel1 control register
#define DMAC_C1_CONFIGURATION(c)    (*((__RW uint32_t*)((uint8_t*)c+0x130)))  // DMAC Channel1 configuration reigster

// DMAC Test register
#define DMAC_ITCR(c)                (*((__RW uint32_t*)((uint8_t*)c+0x500)))
#define DMAC_ITOP1(c)               (*((__RW uint32_t*)((uint8_t*)c+0x504)))
#define DMAC_ITOP2(c)               (*((__RW uint32_t*)((uint8_t*)c+0x508)))
#define DMAC_ITOP3(c)               (*((__RW uint32_t*)((uint8_t*)c+0x50C)))

// DMAC PeriphID/CellID register
#define DMAC_PERIPH_ID0(c)          (*((__RO uint32_t*)((uint8_t*)c+0xFE0)))
#define DMAC_PERIPH_ID1(c)          (*((__RO uint32_t*)((uint8_t*)c+0xFE4)))
#define DMAC_PERIPH_ID2(c)          (*((__RO uint32_t*)((uint8_t*)c+0xFE8)))
#define DMAC_PERIPH_ID3(c)          (*((__RO uint32_t*)((uint8_t*)c+0xFEC)))
#define DMAC_PCELL_ID0(c)           (*((__RO uint32_t*)((uint8_t*)c+0xFF0)))
#define DMAC_PCELL_ID1(c)           (*((__RO uint32_t*)((uint8_t*)c+0xFF4)))
#define DMAC_PCELL_ID2(c)           (*((__RO uint32_t*)((uint8_t*)c+0xFF8)))
#define DMAC_PCELL_ID3(c)           (*((__RO uint32_t*)((uint8_t*)c+0xFFC)))

// DMA Address define
#define DMA                        ((tDMAC_REG*)AHB_DMA)

// DMAC Interrupt status register bit define
#define DMAC_INT_STATUS_MASK                     (0x03)  // Status of the DMA interrupt after masking

// DMAC Interrupt terminal count status register bit define
#define DMAC_INT_TC_STATUS_MASK                  (0x03)  // Interrupt terminal count request status

// DMAC Interrupt terminal count clear register bit define
#define DMAC_INT_TC_CLEAR_MASK                   (0x03)  // Terminal count request clear

// DMAC Interrupt error status register bit define
#define DMAC_INT_ERROR_STATUS_MASK               (0x03)  // Interrupt error status

// DMAC Interrupt error clear register bit define
#define DMAC_INT_ERR_CLR_MASK                    (0x03)  // Interrupt error clear

// DMAC Raw interrupt terminal count status register bit define
#define DMAC_RAW_INT_TC_STATUS_MASK              (0x03)  // Status of the terminal count interrupt prior to masking

// DMAC Raw error interrupt status register bit define
#define DMAC_RAW_INT_ERROR_STATUS_MASK           (0x03)  // Status of the error interrupt prior to masking

// DMAC Enabled channel register bit define
#define DMAC_ENABLED_CHANNELS_MASK               (0x03)  // Channel enable status

// DMAC Software burst request register bit define
#define DMAC_SOFT_B_REQ_MASK                     (0xFFFF)  // Software burst request

// DMAC Software single request register bit define
#define DMAC_SOFT_S_REQ_MASK                     (0xFFFF)  // Software single request

// DMAC Software last burst request register bit define
#define DMAC_SOFT_LB_REQ_MASK                    (0xFFFF)  // Software last burst request

// DMAC Software last single request regitser bit define
#define DMAC_SOFT_LS_REQ_MASK                    (0xFFFF)  // Software last single request

// DMAC Configuration register bit define
#define DMAC_CONFIGURATION_ENABLE                (1<<0)  // SMDMAC enable
#define DMAC_CONFIGURATION_ENDIAN                (1<<1)  // AHB Master endianness configuration

// DMAC Synchronization register bit define
#define DMAC_SYNC_MASK                           (0xFFFF)  // DMA synchronization logic for DMA request signals enabled or disabled

// DMAC Channel0/1 source address register bit define
#define DMAC_CHX_SRC_ADDR_MASK                   (0xFFFFFFFF)  // DMA source address

// DMAC Channel0/1 destination address register bit define
#define DMAC_CHX_DEST_ADDR_MASK                  (0xFFFFFFFF)  // DMA destination address

// DMAC Channel0/1 linked list item register bit define
#define DMAC_CHX_LLI_MASK                        (0xFFFFFFFC)  //  Linked list item

// DMAC Channel0/1 control register bit define
#define DMAC_CHX_CTRL_TRANSFER_SIZE_MASK         (0x00000FFF)  // Transfer size
#define DMAC_CHX_CTRL_SB_SIZE_MASK               (0x00007000)  // Source burst size
#define DMAC_CHX_CTRL_DB_SIZE_MASK               (0x00038000)  // Destination burst size
#define DMAC_CHX_CTRL_SRC_WIDTH_MASK             (0x001C0000)  // Source transfer width
#define DMAC_CHX_CTRL_DEST_WIDTH_MASK            (0x00E00000)  // Destination transfer width
#define DMAC_CHX_CTRL_SRC_INCREMENT              (1<<26)  // Source increment
#define DMAC_CHX_CTRL_DEST_INCREMENT             (1<<27)  // Destination increment
#define DMAC_CHX_CTRL_PROTECTION_MASK            (0x70000000)  // Protection
//#define DMAC_CHX_CTRL_TCI_ENABLE                 (1<<31)  // Terminal count interrupt enable
#define DMAC_CHX_CTRL_TCI_ENABLE                 0x80000000


// DMAC Channel0/1 configuration reigster bit define
#define DMAC_CHX_CONFIG_ENABLE                   (1<<0)  // Channel enable
#define DMAC_CHX_CONFIG_SRC_PERI_MASK            (0x1E)  // Source peripheral
#define DMAC_CHX_CONFIG_DEST_PERI_MASK           (0x03C0)  // Destination peripheral
#define DMAC_CHX_CONFIG_FLOW_CTRL_MASK           (0x3800)  // Flow control and transfer type
#define DMAC_CHX_CONFIG_INT_ERROR                (1<<14)  // Interrupt error mask
#define DMAC_CHX_CONFIG_INT_TERMINAL_CNT         (1<<15)  // Terminal count interrupt mask
#define DMAC_CHX_CONFIG_LOCK                     (1<<16)  // Lock
#define DMAC_CHX_CONFIG_ACTIVE                   (1<<17)  // Active
#define DMAC_CHX_CONFIG_HALT                     (1<<18)  // Halt

/**************************************************************************************
 * The DMACPeriphID0-3 Registers, with address offsets of 0xFE0, 0xFE4, 0xFE8, and
 * 0xFEC respectively, are four 8-bit registers that span address locations 0xFE0-0xFEC
 * You can treat the registers conceptually as a 32-bit register. The read-only registers provide
 * the following options of the peripheral
 *
 * PartNumber    [11:0]  This identifies the peripheral.
 *                       The three digits product code is 0x081.
 * Designer ID   [19:12] This is the identification of the designer.
 *                       ARM Limited is 0x41 (ASCII A).
 * Revision      [23:20] This is the revision number of the peripheral.
 *                       The revision number starts from 0.
 * Configuration [31:24] This is the configuration option of the peripheral.
 *************************************************************************************/
#define DMAC_PERIPH_ID0_PARTNUM_MASK             (0xFF)  // PartNumber[7:0], these bits read back as 0x81
#define DMAC_PERIPH_ID1_PARTNUM_MASK             (0x0F)  // PartNumber[11:8], these bits read back as 0x0
#define DMAC_PERIPH_ID1_DESIGNER_MASK            (0xF0)  // Designer[15:12], these bits read back as 0x01
#define DMAC_PERIPH_ID2_DESIGNER_MASK            (0x0F)  // Designer[19:16], these bits read back as 0x04
#define DMAC_PERIPH_ID2_REVISION_MASK            (0xF0)  // Revision[23:20], these bits read back as 0x01
#define DMAC_PERIPH_ID3_CONFIG_CHANNEL_MASK      (0x07)  // Configuration[26:24], Indicates the number of channels, these bits read back as 0x00, 2 channels
#define DMAC_PERIPH_ID3_CONFIG_MASTER_NUM        (1<<3)  // Configuration[27], Indicates the number of AHB masters, these bits read back as 0x00, one AHB master interface
#define DMAC_PERIPH_ID3_CONFIG_BUS_WIDTH_MASK    (0x70)  // Configuration[30:28], Indicates the AHB master bus width, these bits read back as 0x00, 32bit wide
#define DMAC_PERIPH_ID3_CONFIG_DMA_REQ_NUM       (1<<7)  // Configuration[31], Indicates the number of DMA source requestors, these bits read back as 0x00, 16 DMA requestors

/***************************************************************************************
 * The DMACPCellID0-3 Registers, with address offsets of 0xFF0, 0xFF4, 0xFF8, and 0xFFC
 * respectively, are four 8-bit wide registers, that span address locations 0xFF0-0xFFC.
 * You can treat the registers conceptually as a 32-bit register.
 * The register is a standard cross-peripheral identification system.
 * The DMACPCellID Register is set to 0xB105F00D.
 **************************************************************************************/
#define DMAC_PCELL_ID0_MASK                      (0xFF)  // These bits read back as 0x0D
#define DMAC_PCELL_ID1_MASK                      (0xFF)  // These bits read back as 0xF0
#define DMAC_PCELL_ID2_MASK                      (0xFF)  // These bits read back as 0x05
#define DMAC_PCELL_ID3_MASK                      (0xFF)  // These bits read back as 0xB1



/***********************************************************************************************
 *                                      I2C START                                              *
************************************************************************************************/

// I2C reg data structure
typedef struct _tI2C_REG {
    __RW   uint32_t pres_lo;     // I2C Clock Prescaler Register Low-byte [MASTER mode only]
    __RW   uint32_t pres_hi;     // I2C Clock Prescaler Register High-byte [MASTER mode only]
    __RW   uint32_t ctrl;        // I2C Control Register
    __RW   uint32_t txrx;        // I2C Transmit Register (WO) / I2C Receive Register (RO)
    __RW   uint32_t cmdstate;    // I2C Command Register [MASTER mode only](WO) / I2C State Register  [MASTER mode only](RO)
    __RW   uint32_t dev_id;      // I2C Device ID Register [SLAVE mode only]
    __RWAC uint32_t tx_dma;      // I2C Transmit DMA Control Register
    __RWAC uint32_t rx_dma;      // I2C Receive DMA Control Register
    __ROAC uint32_t sts;         // I2C Status Register
    __RW   uint32_t int_en;      // I2C Interrupt Enable Register
    __RW   uint32_t int_stsclr;  // I2C Interrupt Status Register (RO) / I2C Interrupt Clear Register (WOAC)
    __RO   uint32_t peri_id0;    // I2C Peripheral ID0 Register
    __RO   uint32_t peri_id1;    // I2C Peripheral ID1 Register
    __RO   uint32_t peri_id2;    // I2C Peripheral ID2 Register
    __RO   uint32_t peri_id3;    // I2C Peripheral ID3 Register
} tI2C_REG;

#define I2C1    ((tI2C_REG*)APB_I2C1)
#define I2C2    ((tI2C_REG*)APB_I2C2)
#define I2C3    ((tI2C_REG*)APB_I2C3)
#define I2C4    ((tI2C_REG*)APB_I2C4)

// I2C CONTROL BIT DEFINE
#define I2C_CTRL_RST            (1<<0)  // soft reset bit 1:activated, 0:deactivated
#define I2C_CTRL_SLVMODE        (1<<1)  // core select bit 1:slave, 0:master
#define I2C_CTRL_TRANDIR        (1<<2)  // DMA Transfer Direction Type select bit 1:P2P, 0:m2p/p2m
#define I2C_CTRL_EN             (1<<7)  // core select bit 1:enable, 0:disable

// I2C COMMAND BIT DEFINE
#define I2C_CMD_SENDACK         (0<<3)  // when a receiver, sent ACK
#define I2C_CMD_SENDNACK        (1<<3)  // when a receiver, sent NACK
#define I2C_CMD_WRITE           (1<<4)  // write to slave
#define I2C_CMD_READ            (1<<5)  // read from slave
#define I2C_CMD_STOP            (1<<6)  // generate stop condition
#define I2C_CMD_START           (1<<7)  // generate (repeated) start condition

// I2C STATE BIT DEFINE
#define I2C_STATE_TIP           (1<<1)  // Transfer in progress 1:transferring data, 0:transfer operation is idle
#define I2C_STATE_BUSBUSY       (1<<6)  // I2C bus busy 1:after START signal detected, 0:after STOP signal detected
#define I2C_STATE_RXACK         (1<<7)  // Received acknowledge from slave 1:No acknowledge received, 0:Acknowledge received

// I2C TX DMA BIT DEFINE
#define I2C_TX_DMA_SIZE_MASK    (0x7f)  // [6:0]  Transmit DMA Size "0x01"~"0x7F" when master, "0x00" when slave
#define I2C_TX_DMA_EN           (1<<7)  // I2C core Tx DMA enable bit 1:Tx DMA is enabled, 0:Tx DMA is disabled

// I2C RX DMA BIT DEFINE
#define I2C_RX_DMA_SIZE_MASK    (0x7f)  // [6:0]  Receive DMA Size "0x01"~"0x7F" when master, "0x00" when slave
#define I2C_RX_DMA_EN           (1<<7)  // I2C core Rx DMA enable bit 1:Rx DMA is enabled, 0:Rx DMA is disabled

// I2C STATUS BIT DEFINE
#define I2C_STS_TRAN_DONE       (1<<0)  // Transfer complete (Non-DMA single byte or DMA transfer)
#define I2C_STS_ARB_LOST        (1<<1)  // Arbitration lost [MASTER mode only], This bit is set when the core lost arbitration

// I2C INTERRUPT BIT DEFINE
#define I2C_TRAN_INT_EN         (1<<0)  // Transfer Interrupt Enable (Non-DMA single byte or DMA transfer)
#define I2C_ARB_INT_EN          (1<<1)  // Arbitration lost Interrupt Enable

// I2C INTERRUPT STATUS BIT DEFINE
#define I2C_TRAN_INT_STS        (1<<0)  // Transfer Interrupt Status (Non-DMA single byte or DMA transfer)
#define I2C_ARB_INT_STS         (1<<1)  // Arbitration lost Interrupt Status

// I2C INTERRUPT CLEAR BIT DEFINE
#define I2C_TRAN_INT_CLR        (1<<0)  // Transfer Interrupt Clear (Non-DMA single byte or DMA transfer)
#define I2C_ARB_INT_CLR         (1<<1)  // Arbitration lost Interrupt Clear


/***********************************************************************************************
 *                                      SPI START                                              *
************************************************************************************************/

// SPI register structure
typedef struct _tSPI_REG {
    __RW uint32_t cr0;            // SPI Control register 0
    __RW uint32_t cr1;            // SPI Control register 1
    __RW uint32_t dr;             // SPI Data register
    __RO uint32_t sr;             // SPI Status register
    __RW uint32_t cpsr;           // SPI Clock prescale register
    __RW uint32_t imsc;           // SPI Interrupt mask set or clear register
    __RO uint32_t ris;            // SPI Raw interrupt status register
    __RO uint32_t mis;            // SPI Masked interrupt status register
    __WO uint32_t icr;            // SPI Interrupt clear register
    __RW uint32_t dmacr;          // SPI DMA control register
         uint32_t reserved0[22];  // Reserved
         uint32_t reserved1[4];   // Reserved for test
         uint32_t reserved2[976]; // Reserved
         uint32_t reserved3[4];   // reserved for future expansion
    __RO uint32_t periphID[4];    // SPI Peripheral identification register
    __RO uint32_t pcellID[4];     // SPI PrimeCell identification register
} tSPI_REG;

#define SPI1                         ((tSPI_REG*)APB_SPI1)
#define SPI2                         ((tSPI_REG*)APB_SPI2)

// SPI Control register 0
#define SPI_CR0_DSS_MASK             (0x0F)
#define SPI_CR0_DSS(x)               ((x&~SPI_CR0_DSS_MASK)|x)<< 0  // Data Size Select
#define SPI_CR0_FRF_MASK             (0x03)
#define SPI_CR0_FRF(x)               ((x&~SPI_CR0_FRF_MASK)|x)<<4  // Frame Format
#define SPI_CR0_SPO                  (1<<6)  // SPICLKOUT polarity
#define SPI_CR0_SPH                  (1<<7)  // SPICLKOUT phase
#define SPI_CR0_SCR_MASK             (0xFF)
#define SPI_CR0_SCR(x)               ((x&~SPI_CR0_SCR_MASK)|x)<<8  // Serial clock rate

// SPI Control register 1
#define SPI_CR1_LBM                  (1<<0)  // Loop back mode
#define SPI_CR1_SSE                  (1<<1)  // serial port enable
#define SPI_CR1_MS                   (1<<2)  // Master or slave mode
#define SPI_CR1_SOD                  (1<<3)  // Slave mode output disable

// SPI Data register
#define SPI_DR_MASK                  (0xFFFF)

// SPI Status register
#define SPI_PSR_TFE                  (1<<0)  // Transmit FIFO empty, RO
#define SPI_PSR_TNF                  (1<<1)  // Transmit FIFO not full, RO
#define SPI_PSR_RNE                  (1<<2)  // Receive FIFO not empty, RO
#define SPI_PSR_RFF                  (1<<3)  // Receive FIFO full, RO
#define SPI_PSR_BSY                  (1<<4)  // PrimeCell SPI busy flag, RO

// SPI Clock prescale register
#define SPI_CPSR_MASK                (0xFF)  // Clock prescale divisor

// SPI Interrupt mask set/ clear register
#define SPI_IMSC_RORIM               (1<<0)  // Receive overrun interrupt mask
#define SPI_IMSC_RTIM                (1<<1)   // Receive timeout interrupt mask
#define SPI_IMSC_RXIM                (1<<2)  // Receive FIFO interrupt mask
#define SPI_IMSC_TXIM                (1<<3)  // Transmit FIFO interrupt mask

// SPI Raw interrupt status register
#define SPI_RIS_RORRIS               (1<<0)  // Gives the raw intr state, prior to masking of the SPIRORINTR intr
#define SPI_RIS_RTRIS                (1<<1)  // Gives the raw intr state, prior to masking of the SPIRTINTR intr
#define SPI_RIS_RXRIS                (1<<2)  // Gives the raw intr state, prior to masking of the SPIRXINTR intr
#define SPI_RIS_TXRIS                (1<<3)  // Gives the raw intr state, prior to masking of the SPITXINTR intr

// SPI Masked interrupt status register
#define SPI_MIS_RORMIS               (1<<0)  // Gives the receive over run masked intr status, after masking of the SPIRORINTR intr
#define SPI_MIS_RTMIS                (1<<1)  // Gives the receive timeout masked intr state, after masking of the SPIRTINTR intr
#define SPI_MIS_RXMIS                (1<<2)  // Gives the FIFO masked intr state, after masking of the SPIRXINTR intr
#define SPI_MIS_TXMIS                (1<<3)  // Gives the transmit FIFO masked intr state, after masking of the SPITXINTR intr

// SPI Interrupt clear rergister
#define SPI_ICR_RORIC                (1<<0)  // Clears the SPIRORINTR interrupt
#define SPI_ICR_RTIC                 (1<<1)  // Clears the SPIRTINTR interrupt

// SPI DMA control register
#define SPI_DMACR_RXDMAE             (1<<0)  // Receive DMA Enable
#define SPI_DMACR_TXDMAE             (1<<1)  // Transmit DMA Enable

// SPI Peripheral identification register
#define SPI_PERI_ID0_PARTNUM0_MASK   (0xFF)  // These bits read back as 0x22
#define SPI_PERI_ID1_PARTNUM1_MASK   (0x0F)  // These bits read back as 0x0
#define SPI_PERI_ID1_DESIGNER0_MASK  (0xF0)  // These bits read back as 0x1
#define SPI_PERI_ID2_DESIGNER1_MASK  (0x0F)  // These bits read back as 0x4
#define SPI_PERI_ID2_REVISION_MASK   (0xF0)  // These bits return the peri revision
#define SPI_PERI_ID3_CONFIG_MASK     (0xFF)  // These bits read back as 0x00

// SPI PrimeCell identification register
#define SPI_PRIMECELL_ID0_MASK       (0x0D)   // These bits read back as 0x0D
#define SPI_PRIMECELL_ID1_MASK       (0xF0)  // These bits read back as 0xF0
#define SPI_PRIMECELL_ID2_MASK       (0x05)  // These bits read back as 0x05
#define SPI_PRIMECELL_ID3_MASK       (0xB1)  // These bits read back as 0xB1


/***********************************************************************************************
 *                                      SPI3 START                                             *
************************************************************************************************/
// SPI3 register structure
typedef struct _tSPI3_REG {
    __RW    uint32_t ctrl;
    __RW    uint32_t ext_ctrl;
    __RWOAC uint32_t state;
    __RW    uint32_t data;
            uint32_t reserved0[4];
    __ROAC  uint32_t raw_int_sts;
    __RW    uint32_t int_en;
    __RW1C  uint32_t int_sts_clr;
            uint32_t reserved1[3];
    __RO    uint32_t peri_id0;
    __RO    uint32_t peri_id1;
} tSPI3_REG;

#define SPI3                         ((tSPI3_REG*)APB_SPI3)

//SPI3 Control register
#define SPI3_CR_EN                      (1<<0) // Enable or Disable
#define SPI3_CR_MST_SLV                 (1<<1) // Slave- Master configuration
#define SPI3_CR_FRM_FORMAT_MASK         (0x3)
#define SPI3_CR_FRM_FORMAT(x)           ((x&~SPI3_CR_FRM_FORMAT_MASK)|x)<<2 // frame format
#define SPI3_CR_CLK_POL                 (1<<4) // Clock Polarity
#define SPI3_CR_PHASE                   (1<<5) // Clock Phase
#define SPI3_CR_DATA_LINE               (1<<6) // Data Line Mode
#define SPI3_CR_SDO_EN                  (1<<7) // Data output enable
#define SPI3_CR_DATA_SIZE_MASK          (0xF)
#define SPI3_CR_DATA_SIZE(x)            ((x&~SPI3_CR_DATA_SIZE_MASK)|x)<<8 // data size
#define SPI3_CR_BAUDRATE_MASK           (0xF)
#define SPI3_CR_BAUDRATE(x)             ((x&~SPI3_CR_BAUDRATE_MASK)|x)<<12 // frame format

//SPI3 Extended Control register
#define SPI3_EXT_SSEL_DIS               (1<<0)  // slave select output disable in master configuration
#define SPI3_EXT_SSEL_CTRL              (1<<1)  // slave select control mode
#define SPI3_EXT_BIT_ASSER              (1<<2)  // slave select assertion for SW control
#define SPI3_EXT_BIT_ORDER              (1<<4)  // bit order of serialization
#define SPI3_EXT_LOOPBK_MODE            (1<<5)  // loop-back mode
#define SPI3_EXT_TX_DMA_EN              (1<<6)  // transmit DMA enable
#define SPI3_EXT_RX_DMA_EN              (1<<7)  // receive DMA enable
#define SPI3_EXT_CLK_PRESC_MASK         (0x7F)
#define SPI3_EXT_CLK_PRESC(x)           ((x&~SPI3_EXT_CLK_PRESC_MASK)|x)<<8 // prescale

//SPI3 Status register
#define SPI3_STS_TX_FIFO_RDY            (1<<0) // Transmit FIFO Write Ready
#define SPI3_STS_RX_FIFO_RDY            (1<<1) // Receive FIFO Read Ready
#define SPI3_STS_TX_FIFO_EMPTY          (1<<2) // Transmit FIFO Empty
#define SPI3_STS_RX_FIFO_FULL           (1<<3) // Receive FIFO Full
//SPI3 control command register
#define SPI3_CMD_CLR_SDO_EN             (1<<0)  // clear SDO_EN bit in the SPI_CTRL register
#define SPI3_CMD_SET_SDO_EN             (1<<1)  // set SDO_EN bit in the SPI_CTRL register
#define SPI3_CMD_CLR_SSEL_DIS           (1<<2)  // Clear SSEL_DIS bit in the SPI_EXT_CTRL register
#define SPI3_CMD_SET_SSEL_DIS           (1<<3)  // set SSEL_DIS bit in the SPI_EXT_CTRL register
#define SPI3_CMD_CLR_SSEL_ASSERT        (1<<4) // clear SSEL_ASSERT bit in the SPI_EXT_CTRL register
#define SPI3_CMD_SET_SSEL_ASSERT        (1<<5) // set SSEL_ASSERT bit in the SPI_EXT_CTRL register
//SPI3 interrupt enable register
#define SPI3_INT_TX_FIFO_HE_EN          (1<<0) // TX FIFO at least Half-Empty
#define SPI3_INT_RX_FIFO_HF_EN          (1<<1) // RX FIFO at least Half_FULL
#define SPI3_INT_RX_OVR                 (1<<4) // RX FIFO Overrun Error
#define SPI3_INT_TX_TOUT                (1<<5) // Rx Read Timeout Error

//SPI3 interrupt clear
#define SPI3_INT_TX_CLR                 (1<<0)
#define SPI3_INT_RX_CLR                 (1<<1)



/***********************************************************************************************
 *                                      BSPI START                                             *
************************************************************************************************/

// SPI register structure
typedef struct _tBSPI_REG {
    __RO uint32_t r_addr;
    __RW uint32_t r_data;
    __RW uint32_t ctrl;
    __RW uint32_t m_wbstn;
    __RO uint32_t m_jdec;
    __RW uint32_t m_status;
    __RW uint32_t m_config;
    __WO uint32_t m_1cmd;
    __WO uint32_t m_serase;
    __WO uint32_t m_berase;
    __RW uint32_t ist_set0;
         uint32_t reserved[2];
    __RO uint32_t peri_id;
} tBSPI_REG;

#define BSPI                         ((tBSPI_REG*)BSPI_FREG_BASE)

#define BSPI_CTRL_TMD_SINGLE             (0<<0)
#define BSPI_CTRL_TMD_DUAL               (2<<0)
#define BSPI_CTRL_TMD_QUAD               (3<<0)
#define BSPI_CTRL_M_TMD_FAST_EN          (1<<2)
#define BSPI_CTRL_R_HOLD_EN              (1<<4)
#define BSPI_CTRL_M_DUMMY(x)             ((x)<<8)
#define BSPI_CTRL_M_DUMMY_EN             (1<<10)
#define BSPI_CTRL_M_CTINU_EN             (1<<11)
#define BSPI_CTRL_P_HOLD                 (1<<12)
#define BSPI_CTRL_P_WP                   (1<<13)

#define BSPI_M_1CMD_M_CERASE             (1<<0)
#define BSPI_M_1CMD_M_WRDISE             (1<<2)
#define BSPI_M_1CMD_M_WREN               (1<<3)
#define BSPI_M_1CMD_M_PDDIS              (1<<4)
#define BSPI_M_1CMD_M_PDEN               (1<<5)


/***********************************************************************************************
 *                                      SDIO MASTER START                                      *
************************************************************************************************/
typedef struct _tSDIO_M_REG {
    __RW    uint32_t sdma_addr;
    __RW    uint16_t blk_size;
    __RW    uint16_t blk_cnt;
    __RW    uint32_t arg;
    __RW    uint16_t tran_mode;
    __RW    uint16_t cmd;
    __ROAC  uint32_t resp[4];
    __RW    uint32_t data;
    __ROAC  uint32_t pres_state;
    __RW    uint8_t  host_ctrl;
    __RW    uint8_t  pwr_ctrl;
    __RW    uint8_t  blkgap_ctrl;
            uint8_t  reserved0;
    __RW    uint16_t clk_ctrl;
    __RW    uint8_t  tout_ctrl;
    __RW    uint8_t  sw_rst;
    __RW1C  uint16_t int_sts;
    __RW    uint16_t err_sts;
    __RW    uint16_t int_en;
    __RW    uint16_t err_en;
    __RW    uint16_t sig_en;
    __RW    uint16_t err_sig_en;
    __ROAC  uint16_t err_acmd;
    __RW    uint16_t host_ctrl2;
            uint32_t reserved1[32];
    __RW    uint16_t host_ctrl3;
            uint16_t reserved2;
    __RW    uint32_t clk_ctrl2;
    __RO    uint32_t peri_id;
            uint32_t reserved3[12];
    __RO    uint16_t host_ver;
} tSDIO_M_REG;

#define SDIO_M                          ((tSDIO_M_REG*)AHB_SDIO_MASTER)

#define SDIO_M_TRAN_DMA_EN              (1<<0)
#define SDIO_M_TRAN_BLKCNT_EN           (1<<1)
#define SDIO_M_TRAN_AUTO_CMD12          (1<<2)
#define SDIO_M_TRAN_AUTO_CMD23          (1<<3)
#define SDIO_M_TRAN_DIR                 (1<<4)
#define SDIO_M_TRAN_MULT_BLK            (1<<5)

#define SDIO_M_PRES_CMD_INHB            (1<<0)
#define SDIO_M_PRES_DATA_INHB           (1<<1)
#define SDIO_M_PRES_DATA_LINE_ACT       (1<<2)
#define SDIO_M_PRES_DOING_WRITE         (1<<8)
#define SDIO_M_PRES_DOING_READ          (1<<9)
#define SDIO_M_PRES_SPACE_AVAIL         (1<<10)
#define SDIO_M_PRES_DATA_AVAIL          (1<<11)
#define SDIO_M_PRES_CARD_PRES           (1<<16)
#define SDIO_M_PRES_CARD_STATE_STBL     (1<<17)
#define SDIO_M_PRES_CARD_DET_LVL        (1<<18)
#define SDIO_M_PRES_DATA_LVL            (1<<20)
#define SDIO_M_PRES_CMD_LVL             (1<<24)

#define SDIO_M_CTRL_4BIT                (1<<1)
#define SDIO_M_CTRL_HISPD               (1<<2)
#define SDIO_M_CTRL_8BIT                (1<<5)

#define SDIO_M_CLK_INT_EN               (1<<0)
#define SDIO_M_CLK_INT_STBL             (1<<1)
#define SDIO_M_CLK_DIV_HI(x)            ((x)<<6)
#define SDIO_M_CLK_DIV(x)               ((x)<<8)

#define SDIO_M_RST_ALL                  (1<<0)
#define SDIO_M_RST_CMD                  (1<<1)
#define SDIO_M_RST_DATA                 (1<<2)

#define SDIO_M_INT_RESP                 (1<<0)
#define SDIO_M_INT_DATA_END             (1<<1)
#define SDIO_M_INT_BLK_GAP              (1<<2)
#define SDIO_M_INT_DMA_END              (1<<3)
#define SDIO_M_INT_CARD_INS             (1<<6)
#define SDIO_M_INT_CARD_REM             (1<<7)
#define SDIO_M_INT_CARD_INT             (1<<8)
#define SDIO_M_INT_BOOT_ACK             (1<<11)
#define SDIO_M_INT_ERR                  (1<<15)
#define SDIO_M_INT_ALL                  (0xFFFF)

#define SDIO_M_ERR_CMD_TOUT             (1<<0)
#define SDIO_M_ERR_CMD_CRC              (1<<1)
#define SDIO_M_ERR_CMD_END              (1<<2)
#define SDIO_M_ERR_IDX                  (1<<3)
#define SDIO_M_ERR_DATA_TOUT            (1<<4)
#define SDIO_M_ERR_DATA_CRC             (1<<5)
#define SDIO_M_ERR_DATA_END             (1<<6)
#define SDIO_M_ERR_ACMD                 (1<<8)
#define SDIO_M_ERR_BUS_RESP             (1<<12)
#define SDIO_M_ERR_BUS_TOUT             (1<<13)
#define SDIO_M_ERR_FIFO                 (1<<14)
#define SDIO_M_ERR_ALL                  (0xFFFF)

#define SDIO_M_CTRL_UHS                 (1<<1)
#define SDIO_M_CTRL_DDR                 (1<<2)
#define SDIO_M_CTRL_VDD180              (1<<3)
#define SDIO_M_CTRL_DRV_TYPE(x)         ((x)<<4)

#define SDIO_M_CMD_LINE_MODE            (1<<0)
#define SDIO_M_CMD_LINE_CTL             (1<<1)
#define SDIO_M_BOOT_EN                  (1<<2)
#define SDIO_M_BOOT_ACK                 (1<<3)
#define SDIO_M_DMA_REQ_EN               (1<<8)
#define SDIO_M_SDMA_ALIGN               (1<<9)
#define SDIO_M_CLK_STOP_DIS             (1<<13)
#define SDIO_M_RST_LINE                 (1<<15)

#define SDIO_M_FBCLK_DLY_VAL(x)         ((x)<<0)
#define SDIO_M_DIN_DLY_VAL(x)           ((x)<<8)
#define SDIO_M_DOUT_CLK_SEL             (1<<16)

/***********************************************************************************************
 *                                      SDIO SLAVE START                                       *
************************************************************************************************/
typedef struct _tSDIO_S_REG {
    __RW    uint32_t clk_wakeup;
    __RW    uint32_t cccr;
    __RW    uint32_t adma_sys_addr;
    __RO    uint32_t card_rdy;
    __RW    uint32_t fun_rdy;
    __RW    uint32_t ahb_fn0_int_en;
    __RW    uint32_t ahb_fn0_int;
    __RW    uint32_t soft_rst_ahb;
    __RW    uint32_t glb_int_ena;
    __RW    uint32_t glb_int_sts;
    __RW    uint32_t csa_pointer;
    __RW    uint32_t io_acc_mode;
    __RW    uint32_t last_fetch_addr;
    __RW    uint32_t uhs_support;
    __RW    uint32_t clk_delay_timer;
    __RO    uint32_t power_control;
    __RO    uint32_t power_state;
            uint32_t reserved0[47];
    __RW    uint32_t data_port;
    __RW    uint32_t ocr;
    __RW    uint8_t  int_idnet;
    __RW    uint8_t  int_en;
            uint16_t reserved1[1];
    __RW    uint32_t ahb_transcnt;
    __RW    uint32_t ahb_sdio_transcnt;
    __RW    uint32_t ahb_fn1_int;
    __RW    uint32_t ahb_fn1_int_en;
    __RW    uint32_t fbr;
    __RW    uint32_t ior;
    __RO    uint32_t sd_host_gp;
    __WO    uint32_t arm_gp;
    __RW    uint32_t rddata_rdy;
    __RW    uint32_t blksize;
    __RW    uint32_t argument;
    __RW    uint32_t wrblkcnt;
    __RW    uint32_t rdblkcnt;
} tSDIO_S_REG;

#define SDIO_S                         ((tSDIO_S_REG*)AHB_SDIO_SLAVE)
#define SDIO_S_CIS                     ((tSDIO_S_CIS*)AHB_SDIO_CIS_SLAVE)
#define SDIO_S_CIS1                    ((tSDIO_S_CIS1*)AHB_TRNG)


// AHB Clock wake upt register (0x00)
#define SDIO_S_AHB_CLK_WAKEUP_AUTO_EN                   (1<<0)
#define SDIO_S_AHB_CLK_WAKEUP_MANUAL_EN                 (1<<1)

// AHB CCCR Register (0x04)
#define SDIO_S_CCCR_REVISION_MASK                       (0x0F)
#define SDIO_S_CCCR_REVISION(x)                         (((x&~SDIO_S_CCCR_REVISION_MASK)|x)<<0)
#define SDIO_S_CCCR_SDIO_REVISION_MASK                  (0x0F)
#define SDIO_S_CCCR_SDIO_REVISION(x)                    (((x&~SDIO_S_CCCR_SDIO_REVISION_MASK)|x)<<4)
#define SDIO_S_CCCR_SD_REVISION_MASK                    (0x0F)
#define SDIO_S_CCCR_SD_REVISION(x)                      (((x&~SDIO_S_CCCR_SD_REVISION_MASK)|x)<<8)
#define SDIO_S_CCCR_SCSI                                (1<<12)
#define SDIO_S_CCCR_SDC                                 (1<<13)
#define SDIO_S_CCCR_SMB                                 (1<<14)
#define SDIO_S_CCCR_SRW                                 (1<<15)
#define SDIO_S_CCCR_SBS                                 (1<<16)
#define SDIO_S_CCCR_S4MI                                (1<<17)
#define SDIO_S_CCCR_LSC                                 (1<<18)
#define SDIO_S_CCCR_4BLS                                (1<<19)
#define SDIO_S_CCCR_SMPC                                (1<<20)
#define SDIO_S_CCCR_SHS                                 (1<<21)
#define SDIO_S_CCCR_CMD_INDEX_MASK                      (0x3F)
#define SDIO_S_CCCR_CMD_INDEX(x)                        (((x&~SDIO_S_CCCR_CMD_INDEX_MASK)|x)<<22)

// CARDRDY Register
#define SDIO_S_CARDRDY_RDY                              (1<<0)

//FUNRDY Register
#define SDIO_S_FUNRDY_RDY                               (1<<0)

// AHB_FN0_INT_EN Register (0x14)
#define SDIO_S_AHB_FN0_INT_EN_AHBSOFT_RST_EN            (1<<0)
#define SDIO_S_AHB_FN0_INT_EN_VOLT_SWITCH_CMD_EN        (1<<1)
#define SDIO_S_AHB_FN0_INT_EN_CMD19_RD_STRT_EN          (1<<2)
#define SDIO_S_AHB_FN0_INT_EN_CMD19_RD_TRN_OVR_EN       (1<<3)
#define SDIO_S_AHB_FN0_INT_EN_FN0_WR_STRT_EN            (1<<4)
#define SDIO_S_AHB_FN0_INT_EN_FN0_WR_TRN_OVR_EN         (1<<5)
#define SDIO_S_AHB_FN0_INT_EN_FN0_RD_STRT_EN            (1<<6)
#define SDIO_S_AHB_FN0_INT_EN_FN0_RD_OVR_EN             (1<<7)
#define SDIO_S_AHB_FN0_INT_EN_FN0_RD_ERR_EN             (1<<8)
#define SDIO_S_AHB_FN0_INT_EN_FN0_ADMA_END_EN           (1<<9)
#define SDIO_S_AHB_FN0_INT_EN_FN0_ADMA_INT_EN           (1<<10)
#define SDIO_S_AHB_FN0_INT_EN_FN0_ADMA_ERR_EN           (1<<11)

//SDIO_S_AHB_FN0_INT Register (0x18)
#define SDIO_S_AHB_FN0_INT_AHBSOFT_RST                  (1<<0)
#define SDIO_S_AHB_FN0_INT_VOLT_SWITCH_CMD              (1<<1)
#define SDIO_S_AHB_FN0_INT_CMD19_RD_STRT                (1<<2)
#define SDIO_S_AHB_FN0_INT_CMD19_RD_TRN_OVR             (1<<3)
#define SDIO_S_AHB_FN0_INT_FN0_WR_STRT                  (1<<4)
#define SDIO_S_AHB_FN0_INT_FN0_WR_TRN_OVR               (1<<5)
#define SDIO_S_AHB_FN0_INT_FN0_RD_STRT                  (1<<6)
#define SDIO_S_AHB_FN0_INT_FN0_RD_OVR                   (1<<7)
#define SDIO_S_AHB_FN0_INT_FN0_RD_ERR                   (1<<8)
#define SDIO_S_AHB_FN0_INT_FN0_ADMA_END                 (1<<9)
#define SDIO_S_AHB_FN0_INT_FN0_ADMA_INT                 (1<<10)
#define SDIO_S_AHB_FN0_INT_FN0_ADMA_ERR                 (1<<11)
#define SDIO_S_AHB_FN0_INT_EN_ALL                       (0xFFF)

//SDIO_S_SOFT_RST_AHB Register (0x1C)
#define SDIO_S_SOFT_RST_AHB_AHBSOFT_VALID               (1<<0)

//SDIO_S_GLB_INT_ENA(0x20)
#define SDIO_S_GLB_INT_ENA_FN0_INT_TO_ARM_EN            (1<<0)
#define SDIO_S_GLB_INT_ENA_FN1_INT_TO_ARM_EN            (1<<1)
#define SDIO_S_GLB_INT_ENA_ALL                          (0x3)

//SDIO_S_GLB_INT_STS(0x24)
#define SDIO_S_GLB_INT_STS_FN0_INT_TO_ARM               (1<<0)
#define SDIO_S_GLB_INT_STS_FN1_INT_TO_ARM               (1<<1)

//SDIO_S_CSA_POINTER(0x28)
#define SDIO_S_CSA_POINTER_CSA_POINTER_MASK             (0xFFFF)
#define SDIO_S_CSA_POINTER_CSA_POINTER(x)               (((x&~SDIO_S_CSA_POINTER_CSA_POINTER_MASK)|x)<<0)

//SDIO_S_IO_ACC_MODE register (0x2C)
#define SDIO_S_IO_ACC_MODE_SSDR50                       (1<<0)
#define SDIO_S_IO_ACC_MODE_SSDR104                      (1<<1)
#define SDIO_S_IO_ACC_MODE_SDDR50                       (1<<2)
#define SDIO_S_IO_ACC_MODE_SDTA                         (1<<3)
#define SDIO_S_IO_ACC_MODE_SDTC                         (1<<4)
#define SDIO_S_IO_ACC_MODE_SDTD                         (1<<5)
#define SDIO_S_IO_ACC_MODE_SAI                          (1<<6)

//SDIO_S_UHS_SUPPORT (0x34)
#define SDIO_S_UHS_SUPPORT_MODE                         (1<<0)
#define SDIO_S_UHS_SUPPORT_DDR_DLY_SELECT               (1<<1)
#define SDIO_S_UHS_SUPPORT_CARD_VOLT_ACCEPTED           (1<<2)
#define SDIO_S_UHS_SUPPORT_SD_CLK_LINE_SWITCHED         (1<<3)
#define SDIO_S_UHS_SUPPORT_SD_CMD_LINE_SWITCHED         (1<<4)

//SDIO_S_POWER_CONTROL(0x3C)
#define SDIO_S_POWER_CONTROL_EMPC                       (1<<0)
#define SDIO_S_POWER_CONTROL_EPS_FUN1                   (1<<1)

//SDIO_S_POWER_STATE (0x40)
#define SDIO_S_POWER_STATE_PWR_STATE_FN1_MASK           (0x3)
#define SDIO_S_POWER_STATE_PWR_STATE_FN1(x)             (((x&~SDIO_S_POWER_STATE_PWR_STATE_FN1_MASK)|x)<<0)

/*Function1 register description*/
// SDIO_S_OCR register (0x104)
#define SDIO_S_OCR_OCR_MASK                             (0xFFFF)
#define SDIO_S_OCR_OCR(x)                               (((x&~SDIO_S_OCR_OCR_MASK)|x)<<0)

//SDIO_S_INT_IDNET Register (0x108)
#define SDIO_S_INT_IDNET_READ_DATA_READY_INT            (1<<0)
#define SDIO_S_INT_IDNET_READ_ERROR                     (1<<1)
#define SDIO_S_INT_IDNET_MESSAGE_FROM_ARM               (1<<2)
#define SDIO_S_INT_IDNET_ACK_TO_SD_HOST                 (1<<3)

//SDIO_S_INT_EN Register (0x109)
#define SDIO_S_INT_EN_READ_DATA_READY_INT               (1<<0)
#define SDIO_S_INT_EN_READ_ERROR                        (1<<1)
#define SDIO_S_INT_EN_MESSAGE_FROM_ARM                  (1<<2)
#define SDIO_S_INT_EN_ACK_TO_SD_HOST                    (1<<3)

//SDIO_S_AHB_TRANSCNT Register (0x10C)
#define SDIO_S_AHB_TRANSCNT_AHB_XFER_CNT_MASK           (0xFFFF)
#define SDIO_S_AHB_TRANSCNT_AHB_XFER_CNT(x)             (((x&~SDIO_S_AHB_TRANSCNT_AHB_XFER_CNT_MASK)|x)<<0)

//SDIO_S_AHB_SDIOTRANSCNT (0x110)
#define SDIO_S_AHB_SDIOTRANSCNT_AHB_XFER_CNT_MASK       (0xFFFF)
#define SDIO_S_AHB_SDIOTRANSCNT_AHB_XFER_CNT(x)         (((x&~SDIO_S_AHB_SDIOTRANSCNT_AHB_XFER_CNT_MASK)|x)<<0)

//SDIO_S_AHB_FN1_INT(0x114)
#define SDIO_S_AHB_FN1_INT_FN1_WR_OVER                  (1<<0)
#define SDIO_S_AHB_FN1_INT_FN1_RD_OVER                  (1<<1)
#define SDIO_S_AHB_FN1_INT_FN1_RD_ERROR                 (1<<2)
#define SDIO_S_AHB_FN1_INT_FUN1_RST                     (1<<3)
#define SDIO_S_AHB_FN1_INT_SD_HOST_FN1_MSG_RDY          (1<<4)
#define SDIO_S_AHB_FN1_INT_FN1_ACK_TO_ARM               (1<<5)
#define SDIO_S_AHB_FN1_INT_FN1_SDIO_RD_START            (1<<6)
#define SDIO_S_AHB_FN1_INT_FN1_SDIO_WR_START            (1<<7)
#define SDIO_S_AHB_FN1_INT_ADMA_END_INT                 (1<<8)
#define SDIO_S_AHB_FN1_INT_FN1_SUSPEND                  (1<<9)
#define SDIO_S_AHB_FN1_INT_RESUME                       (1<<10)
#define SDIO_S_AHB_FN1_INT_ADMA_INT                     (1<<11)
#define SDIO_S_AHB_FN1_INT_ADMA_ERR                     (1<<12)
#define SDIO_S_AHB_FN1_INT_FUN1_EN                      (1<<13)

//SDIO_S_AHB_FN1_INT_EN (0x118)
#define SDIO_S_AHB_FN1_INT_EN_FN1_WR_OVER_EN            (1<<0)
#define SDIO_S_AHB_FN1_INT_EN_FN1_RD_OVER_EN            (1<<1)
#define SDIO_S_AHB_FN1_INT_EN_FN1_RD_ERROR_EN           (1<<2)
#define SDIO_S_AHB_FN1_INT_EN_FUN1_RST_EN               (1<<3)
#define SDIO_S_AHB_FN1_INT_EN_SD_HOST_FN1_MSG_RDY_EN    (1<<4)
#define SDIO_S_AHB_FN1_INT_EN_FN1_ACK_TO_ARM_EN         (1<<5)
#define SDIO_S_AHB_FN1_INT_EN_FN1_SDIO_RD_START_EN      (1<<6)
#define SDIO_S_AHB_FN1_INT_EN_FN1_SDIO_WR_START_EN      (1<<7)
#define SDIO_S_AHB_FN1_INT_EN_ADMA_END_INT_EN           (1<<8)
#define SDIO_S_AHB_FN1_INT_EN_FN1_SUSPEND_EN            (1<<9)
#define SDIO_S_AHB_FN1_INT_EN_RESUME_EN                 (1<<10)
#define SDIO_S_AHB_FN1_INT_EN_ADMA_INT_EN               (1<<11)
#define SDIO_S_AHB_FN1_INT_EN_ADMA_ERR_EN               (1<<12)
#define SDIO_S_AHB_FN1_INT_EN_FUN_EN_INT_EN             (1<<13)
#define SDIO_S_AHB_FN1_INT_EN_ALL                       (0x3FFF)

//SDIO_S_FBR (0x11C)
#define SDIO_S_FBRIO_DEVICE_CODE_1_MASK                 (0xF)
#define SDIO_S_FBRIO_DEVICE_CODE_1(x)                   (((x&~SDIO_S_FBRIO_DEVICE_CODE_1_MASK)|x)<<0)
#define SDIO_S_FBRIO_FUN_CSA_SUP                        (1<<4)
#define SDIO_S_FBRIO_EXTENDED_IO_DEVICE_CODE_1_MASK     (0xFF)
#define SDIO_S_FBRIO_EXTENDED_IO_DEVICE_CODE_1(x)       (((x&~SDIO_S_FBRIO_EXTENDED_IO_DEVICE_CODE_1_MASK)|x)<<0)
#define SDIO_S_FBRIO_SPS                                1<<13

//SDIO_S_IOR (0x120)
#define SDIO_S_IOR_REG                                  (1<<0)

//SDIO_S_RDDATARDY (0x12C)
#define SDIO_S_FUNC_1_READY_DATA_READY                  (1<<0)

//SDIO_S_BLKSIZE(0x130)
#define SDIO_S_BLKSIZE_BLK_SIZE_MASK                    (0xFFF)
#define SDIO_S_BLKSIZE_BLK_SIZE(x)                      (((x&~SDIO_S_BLKSIZE_BLK_SIZE_MASK)|x)<<0)
#define SDIO_S_BLKSIZE_SIN_MUL_BLK                      (1<<12)


//SDIO_S_WRBLKCNT (0x138)
#define SDIO_S_WRBLKCNT_WR_BLK_CNT_MASK                 (0xFFFF)
#define  SDIO_S_WRBLKCNT_WR_BLK_CNT(x)                  (((x&~SDIO_S_WRBLKCNT_WR_BLK_CNT_MASK)|x)<<0)

// SDIO_S_RDBLKCNT (0x13C)
#define SDIO_S_RDBLKCNT_RD_BLK_CNT_MASK                 (0xFFFF)
#define SDIO_S_RDBLKCNT_RD_BLK_CNT(x)                   (((x&~SDIO_S_RDBLKCNT_RD_BLK_CNT_MASK)|x)<<0)

/***********************************************************************************************
 *                                      GPIO START                                             *
************************************************************************************************/

// GPIO Data Structure
typedef struct _tGPIO_REG {
    __RO   uint32_t port_mode;     // Port Mode Register
    __RW   uint32_t out_mode;      // GPIO Port Output Mode Register
    __RO   uint32_t data;          // GPIO Port Data Register
    __RW   uint32_t data_out;      // GPIO Port Data Output Register
    __WOAC uint32_t set_reset;     // GPIO Port Set/Reset Register
    __RW   uint32_t reserved0;     // Reserved
    __RW   uint32_t int_type;      // GPIO Port Interrupt Type Register
    __RW   uint32_t int_pol;       // GPIO Port Interrupt Polarity Register
    __ROAC uint32_t raw_sts;       // GPIO Port Interrupt Raw Status Register
    __RW   uint32_t int_en;        // GPIO Port Interrupt Enable Register
    __RW1C uint32_t int_stsclr;    // GPIO Port Interrupt Status Register (Read) / GPIO Port Interrupt Clear Register (Write)
    __RW   uint32_t reserved1[4];  // Reserved
    __RO   uint32_t peri_id;       // GPIO Peripheral ID Register
} tGPIO_REG;

#define GPIO1    ((tGPIO_REG*)AHB_GPIO1)
#define GPIO2    ((tGPIO_REG*)AHB_GPIO2)
#define GPIO3    ((tGPIO_REG*)AHB_GPIO3)
#define GPIO4    ((tGPIO_REG*)AHB_GPIO4)
#define GPIO5    ((tGPIO_REG*)AHB_GPIO5)


/***********************************************************************************************
 *                                      RTC START                                              *
************************************************************************************************/

// RTC Register structure
typedef struct _tRTC_REG {
    __RW   uint32_t clk_div;      // (NOT USED) RTC Clock Devider Value Register
    __RW   uint32_t ctrl;         // RTC Control Register
    __RW   uint32_t intr_en;      // RTC Interrupt Mask Register
    __RW1C uint32_t intr_stsclr;  // RTC Interrupt Status Register, RTC Interrupt Clear Register
    __RW   uint32_t alarm_mask;   // RTC Alarm Mask Register
    __RW   uint32_t wakeup_cnt;   // RTC Wakeup Register
    __RW   uint32_t calib;        // RTC Calibration Register
    __RW   uint32_t time;         // RTC Time Register
    __RW   uint32_t date;         // RTC Date Register
    __RO   uint32_t sub_sec;      // RTC Sub Second Register
    __RW   uint32_t alarm_time;   // RTC Alarm Time Register
    __RW   uint32_t alarm_date;   // RTC Alarm Date Register
    __RO   uint32_t ts_time;      // RTC Timestamp Time Register
    __RO   uint32_t ts_date;      // RTC Timestamp Date Register
    __RO   uint32_t ts_sub_sec;   // RTC Timestamp Sub-second Register
    __RO   uint32_t peri_id;      // RTC Peripheral ID Register
} tRTC_REG;

#define RTC    ((tRTC_REG*)APB_RTC)

// RTC Control bit define
#define RTC_CTRL_INIT                 (1<<0)    // RTC Initialization
#define RTC_CTRL_ALARM_EN             (1<<1)    // Alarm Enable
#define RTC_CTRL_ONE_SHOT_ALARM       (1<<2)    // One-shot Alarm
#define RTC_CTRL_WAKEUP_EN            (1<<3)    // Wakeup Timer Enable
#define RTC_CTRL_TSTAMP_EN            (1<<4)    // Timestamp Enable
//#define RTC_CTRL_CLK_SEL            (1<<5)    // (NOT USED) Clock source selction 0: XTAL osc(default), 1: APB clock
#define RTC_CTRL_SUB_SEC_DIS          (1<<6)    // Sub Second Counter Disable
//                                    (1<<7)    // Reserved
//#define RTC_CTRL_XTAL_EN            (1<<8)    // (NOT USED) RTC X'tal clock enable
//#define RTC_CTRL_XTAL_BYPASS        (1<<9)    // (NOT USED) RTC X'tal Bypass Mode enable 0: Crystal Connected (default), 1: Use external clock source instead of crystal
//#define RTC_CTRL_RESET              (1<<10)   // (NOT USED) Software Reset for RTC
#define RTC_CTRL_BKSRAM_MODE          (0x3)     // [13:12] RTC-domain Backup SRAM Mode 00: Normal (default), 01: Auto-Retentioin, 10: Retention Mode, 11: Reserved

// RTC Interrupt status bit define
#define RTC_INTR_STS_ALARM            (1<<0)    // Alarm Interrupt
#define RTC_INTR_STS_WAKEUP           (1<<1)    // Wakeup Auto-reload Counter Reaches 0
#define RTC_INTR_STS_TSTAMP_TRIG      (1<<2)    // Timestamp Event Occure

// RTC Alarm mask bit define
#define RTC_ALARM_SECOND_MASK         (1<<0)    // Second value is not compared for the alarm
#define RTC_ALARM_MINUTE_MASK         (1<<1)    // Minutes value is not compared for the alarm
#define RTC_ALARM_HOUR_MASK           (1<<2)    // Hour value is not compared for the alarm
#define RTC_ALARM_DAY_MASK            (1<<3)    // Day value is not compared for the alarm
#define RTC_ALARM_MONTH_MASK          (1<<4)    // Month value is not compared for the alarm
#define RTC_ALARM_YEAR_MASK           (1<<5)    // Year value is not compared for the alarm
#define RTC_ALARM_DAY_OF_WEEK_MASK    (1<<6)    // DAY of Week value is not compared for the alarm

// RTC Calibration bit define
#define RTC_CALIB_CNT                 (0xffff)  // [15:0] Calibration counter value
#define RTC_CALIB_DIR                 (1<<16)   // Calibration direction 0: Forward calibration, 1: Backward calibration

// RTC Time bit define
#define RTC_TIME_SEC                  (0xf)     // [3:0] Seconds in BCD format
#define RTC_TIME_TEN_SEC              (0x7)     // [6:4] Ten seconds in BCD format
#define RTC_TIME_MIN                  (0xf)     // [11:8] Minutes in BCD format
#define RTC_TIME_TEN_MIN              (0x7)     // [14:12] Ten minutes in BCD format
#define RTC_TIME_HOURS                (0xf)     // [19:16] Hours in BCD format
#define RTC_TIME_TEN_HOURS            (0x3)     // [21:20] Ten hours in BCD format

// RTC Date bit define
#define RTC_DATE_DAYS                 (0xf)     // [3:0] Days in BCD format
#define RTC_DATE_TEN_DAYS             (0x3)     // [5:4] Ten days in BCD format
#define RTC_DATE_MONTHS               (0xf)     // [11:8] Months in BCD format
#define RTC_DATE_TEN_MONTHS           (1<<12)   // Ten months in BCD format
#define RTC_DATE_DAY_OF_WEEK          (0x7)     // [15:13] Day of week 000: forbidden, 001: Sunday, ..., 111: Saturday
#define RTC_DATE_YEARS                (0xf)     // [19:16] Years in BCD format
#define RTC_DATE_TEN_YEARS            (0xf)     // [23:20] Ten years in BCD format
#define RTC_DATE_CENT                 (0xf)     // [27:24] Centuries in BCD format
#define RTC_DATE_TEN_CENT             (0xf)     // [31:28] Ten centuries in BCD format

// RTC Alarm time bit define
#define RTC_ALARM_TIME_SEC            (0xf)     // [3:0] Seconds in BCD format
#define RTC_ALARM_TIME_TEN_SEC        (0x7)     // [6:4] Ten seconds in BCD format
#define RTC_ALARM_TIME_MIN            (0xf)     // [11:8] Minutes in BCD format
#define RTC_ALARM_TIME_TEN_MIN        (0x7)     // [14:12] Ten minutes in BCD format
#define RTC_ALARM_TIME_HOURS          (0xf)     // [19:16] Hours in BCD format
#define RTC_ALARM_TIME_TEN_HOURS      (0x3)     // [21:20] Ten hours in BCD format

// RTC Alarm data bit define
#define RTC_TS_TIME_SEC               (0xf)     // [3:0] Seconds in BCD format
#define RTC_TS_TIME_TEN_SEC           (0x7)     // [6:4] Ten seconds in BCD format
#define RTC_TS_TIME_MIN               (0xf)     // [11:8] Minutes in BCD format
#define RTC_TS_TIME_TEN_MIN           (0x7)     // [14:12] Ten minutes in BCD format
#define RTC_TS_TIME_HOURS             (0xf)     // [19:16] Hours in BCD format
#define RTC_TS_TIME_TEN_HOURS         (0x3)     // [21:20] Ten hours in BCD format

// RTC Timestamp data bit define
#define RTC_TS_DATE_DAYS              (0xf)     // [3:0] Days in BCD format
#define RTC_TS_DATE_TEN_DAYS          (0x3)     // [5:4] Ten days in BCD format
#define RTC_TS_DATE_MONTHS            (0xf)     // [11:8] Months in BCD format
#define RTC_TS_DATE_TEN_MONTHS        (1<<12)   // Ten months in BCD format
#define RTC_TS_DATE_DAY_OF_WEEK       (0x7)     // [15:13] Day of week 000: forbidden, 001: Sunday, ..., 111: Saturday
#define RTC_TS_DATE_YEARS             (0xf)     // [19:16] Years in BCD format
#define RTC_TS_DATE_TEN_YEARS         (0xf)     // [23:20] Ten years in BCD format
#define RTC_TS_DATE_CENT              (0xf)     // [27:24] Centuries in BCD format
#define RTC_TS_DATE_TEN_CENT          (0xf)     // [31:28] Ten centuries in BCD format

// RTC Peripheral ID bit define
#define RTC_PERI_ID_PART_NUM          (0xfff)   // [11:0] RTC Part Number
#define RTC_PERI_ID_CONFIG_NUM        (0xf)     // [15:12] RTC Configuration Number
#define RTC_PERI_ID_MAJ_REV           (0xf)     // [19:16] RTC Major Revision
#define RTC_PERI_ID_MIN_REV           (0xf)     // [23:20] RTC Minor Revision
#define RTC_PERI_ID_RSVD_NUM          (0xff)    // [31:24] RTC Reserved Number


/***********************************************************************************************
 *                                      GPT START                                              *
************************************************************************************************/

// GPT reg data structure
typedef struct _tGPT_REG {
    __RW   uint32_t tmr1_ctrl;     // Timer 1 Control Register
    __RW   uint32_t tmr1_match;    // Timer 1 Match Value Register
    __RW   uint32_t tmr1_mask;     // Timer 1 Mask Value Register
    __RO   uint32_t tmr1_cnt;      // Timer 1 Counter Status Register
    __RO   uint32_t tmr1_tstamp;   // Timer 1 Time Stamp Register
           uint32_t reserved0[3];   // Reserved
    __RW   uint32_t tmr2_ctrl;     // Timer 2 Control Register
    __RW   uint32_t tmr2_match;    // Timer 2 Match Value Register
    __RW   uint32_t tmr2_mask;     // Timer 2 Mask Value RegisterF
    __RO   uint32_t tmr2_cnt;      // Timer 2 Counter Status Register
    __RO   uint32_t tmr2_tstamp;   // Timer 2 Time Stamp Register
           uint32_t reserved1[3];   // Reserved
    __RW   uint32_t tmr_ctrl;      // Timer Control Register
    __RW   uint32_t tmr_en;        // Timer Enable Register
    __RW   uint32_t tmr_int_en;    // Timer Interrupt Enable Register
    __RW1C uint32_t tmr_int_sts;   // Timer Interrupt Status Register
           uint32_t reserved2[10];  // Reserved
    __RO   uint32_t peri_id0;      // Timer Peripheral ID0 Register
    __RO   uint32_t peri_id1;      // Timer Peripheral ID1 Register
} tGPT_REG;

#define TIMER1    ((tGPT_REG*)APB_GP_TIMER1)
#define TIMER2    ((tGPT_REG*)APB_GP_TIMER2)

//TIMER CHANNEL CONTROL BIT DEFINE
#define TIMER_CTRL_MODE          (3<<0)  // Timer Mode
#define TIMER_CTRL_TRIG_MODE     (3<<2)  // Timer Trigger Mode
#define TIMER_CTRL_TRIGER_POL    (1<<4)  // Timer Polarity 0:Falling-edge, 1:Rising-edge
#define TIMER_CTRL_TRIGER_SEL    (1<<5)  // Trigger Select 0:External(default), 1:Internal
#define TIMER_CTRL_MATCH_OUT     (1<<6)  // Match Out Value
#define TIMER_CTRL_OUT_MODE      (3<<8)  // Timer Match Out Mode

//TIMER BASE BIT DEFINE
#define TIMER_BASE_CNT_RST      (1<<0)    // Timer Counter Reset
#define TIMER_BASE_MST_SEL      (1<<1)    // Master Timer Select
#define TIMER_BASE_LINK_MODE    (3<<2)    // Timer Link Mode
#define TIMER_BASE_PRESCALER    (0xf<<4)  // Timer Prescaler Value

//TIMER ENABLE BIT DEFINE
#define TIMER1_EN    (1<<0)  // Timer 1 Enable
#define TIMER2_EN    (1<<1)  // Timer 2 Enable

//TIMER INTERRUPT BIT DEFINE
#define TIMER1_MAT_INTR_EN    (1<<0)  // Timer1 Match Interrupt
#define TIMER2_MAT_INTR_EN    (1<<1)  // Timer2 Match Interrupt
#define TIMER1_CAP_INTR_EN    (1<<4)  // Timer1 Capture Interrupt
#define TIMER2_CAP_INTR_EN    (1<<5)  // Timer2 Capture Interrupt

/***********************************************************************************************
 *                                      WDT START                                              *
************************************************************************************************/

// WDT reg data structure
typedef struct _tWDT_REG {
    __RW   uint32_t ctrl;      // Watchdog Timer Control Register
    __RO   uint32_t sts;       // Watchdog Timer Status Register
    __RW   uint32_t load;      // Watchdog Timer Load Value Register
    __RW   uint32_t ewi;       // Watchdog Early Wakeup Interrupt Value Register
    __RW   uint32_t win;       // Watchdog Window Value Register
    __RW   uint32_t lock;      // Watchdog Timer Lock Register
    __RO   uint32_t peri_id0;  // WDT Peripheral ID0 Register
    __RO   uint32_t peri_id1;  // WDT Peripheral ID1 Register
} tWDT_REG;

#define WDT    ((tWDT_REG*)APB_WATCHDOG_TMR)

// WDT CONTROL REG BIT DEFINE
#define WDT_CTRL_WDT_EN           (1<<0)    // Watchdog Timer Enable
#define WDT_CTRL_EWI_EN           (1<<1)    // Early Wakeup Interrupt Enable
#define WDT_CTRL_WWDT_EN          (1<<2)    // Windowed WDT Enable
#define WDT_CTRL_TEST_EN          (1<<3)    // Test Mode Enable
#define WDT_CTRL_TMR_BASE(x)      (x<<4)    // Watchdog Timer Base
#define WDT_CTRL_TMR_BASE_MAX     (0xc<<4)  // Watchdog Timer Base

// WDT STATUS REG BIT DEFINE
#define WDT_STS_EWI_STS           (1<<0)    // Early Wakeup Interrupt Status
#define WDT_STS_TEST_INT          (1<<1)    // Test Interrupt Status
#define WDT_STS_WDT_CNT_FLAG      (0xfffc)  // Current Bits[15:2] of Watchdog Timer Count

// WDT STATUS REG BIT DEFINE
#define WDT_LOAD_FLAG             (0xffff)  // Watchdog Timer Load Value
#define WDT_EWI_FLAG              (0xffff)  // Watchdog Early Wakeup Interrupt Value
#define WDT_WIN_FLAG              (0xffff)  // Watchdog Window Value


// WDT LOCK REG BIT DEFINE
#define WDT_LOCK_CODE_FLAG        (0x7fff)  // WDT Register Lock Code
#define WDT_LOCK_SEQ              (1<<15)   // lock sequence key

/***********************************************************************************************
 *                                      NVM START                                              *
************************************************************************************************/
// NVM reg data structure
typedef struct _tNVM_REG {
    __RW	uint32_t nvm_mode;      // NVM Mode Control Register
    __RW	uint32_t nvm_ctrl;      // NVM Control Signal Register
    __RW	uint32_t nvm_pgmcnt;    // NVM Programming Period Value
    __RW	uint32_t nvm_protkey;   // NVM read cypering control register to read nvm_data
    __RW    uint32_t nvm_treg;      // NVM read cypering control register to read nvm_data
} tNVM_REG;

#define NVM    ((tNVM_REG*)APB_OTP_MEM_CONTROL)


// NVM MODE Control Register
#define NVM_MODE_PD                     (1<<0)  // NVM Power down enable (active high)
#define NVM_MODE_PS                     (1<<1)  // NVM Pass 2.5V program voltage to internal for program enable (active high)

// NVM Control Signal Register
#define NVM_CTRL_CSB                    (1<<0)  // NVM chip seleted enable(active low)
#define NVM_CTRL_PGENB                  (1<<1)  // NVM program enable (active low)
#define NVM_CTRL_LOAD                   (1<<2)  // NVM sense amplifier and load data into latch enable(active high)
#define NVM_CTRL_STROBE                 (1<<3)  // NVM Array read or program access enable (active high)
#define NVM_CTRL_ADDR_MASK              (0x7F)
#define NVM_CTRL_ADDR(x)                ((((x)&~NVM_CTRL_ADDR_MASK)|(x))<<4)        //NVM byte address
#define NVM_CTRL_BIT_POS_MASK           (0x7)
#define NVM_CTRL_BIT_POS(x)             ((((x)&~NVM_CTRL_BIT_POS_MASK)|(x))<<11)    //NVM bit position of NVM_ADDR in Program mode
#define NVM_CTRL_DATA                   (16)
#define NVM_CTRL_STT_BUSY               (1<<27) //NVM busy status to get data from e-FUSE in bootin
#define NVM_CTRL_STT_BUSY_JTAG          (1<<28) //NVM busy status as accessing by JTAG
#define NVM_CTRL_STT_TRIM_VALID         (1<<29) //NVM loading status to get analog IP trimming data from e-Fuse
#define NVM_CTRL_STT_ID_VALID           (1<<30) //NVM loading status to get KEY from e-Fuse
#define NVM_CTRL_STT_BOOT_READY         (1<<31) //NVM loading status to get data(KEY,DATA..) from e-Fuse in booting

// NVM programming Period value
#define NVM_PGM_STR_CNT_MASK            (0x3FF)
#define NVM_PGM_STR_CNT(x)              ((((x)&~NVM_PGM_STR_CNT_MASK)|(x))<<0) // programming strobe counter


/***********************************************************************************************
 *                                      PMU START                                              *
************************************************************************************************/

// CACHE reg data structure
typedef struct _tPMU_REG {
    __RO   uint32_t lpm_stt;
    __RW   uint32_t lpm_conf;
    __RW   uint32_t lpm_wakeup_cnt;
} tPMU_REG;

#define PMU    ((tPMU_REG*)APB_PMU)

#define LPM_STT_POWER_STS(x)            (((x)>>12) & 0xF)

#define LPM_CONF(x)                     ((x)<<12)
#define LPM_CONF_VOLT_DOMAIN_EN         (1<<15)
#define LPM_CONF_PERI_HANDSHAKE_EN      (1<<16)



/***********************************************************************************************
 *                                      I-CACHE START                                              *
************************************************************************************************/

// CACHE reg data structure
typedef struct _tICACHE_REG {
           uint32_t reserved0;
    __RW   uint32_t cache_ctrl;
    __RW   uint32_t cache_config;
           uint32_t reserved1[4080];
    __RO   uint32_t peri_id;
} tICACHE_REG;

#define ICACHE    ((tICACHE_REG*)ICACHE_BASE)

#define ICACHE_EN            (1<<0)
#define ICACHE_PREFETCH_EN   (1<<0)


/***********************************************************************************************
 *                                      TRNG START                                             *
************************************************************************************************/
typedef struct _tTRNG_REG
{
    __RW    uint32_t ctrl;              // 0x00
    __RW    uint32_t stat;              // 0x04
    __RW    uint32_t mode;              // 0x08
    __RW    uint32_t smode;             // 0x0C
    __RW    uint32_t int_enable;        // 0x10
    __RW    uint32_t int_stat;          // 0x14
    __RW    uint32_t feature;           // 0x18
            uint32_t reserved0;         // 0x1C
    __RW    uint32_t rand_base[8];      // 0x20
    __RW    uint32_t seed_base[8];      // 0x40
    __RW    uint32_t auto_rqst;         // 0x60
    __RW    uint32_t auto_age;          // 0x64
    __RW    uint32_t ia_rdata;          // 0x68
    __RW    uint32_t ia_wdata;          // 0x6C
    __RW    uint32_t ia_addr;           // 0x70
    __RW    uint32_t ia_cmd;            // 0x74
} tTRNG_REG;

#define TRNG    ((tTRNG_REG*)AHB_TRNG)




/***********************************************************************************************
 *                                      SPACC START                                            *
************************************************************************************************/

typedef struct _tSPACC_REG
{
    __RW    uint32_t int_en;         // 0x00000L
    __RW    uint32_t int_stat;       // 0x00004L
    __RW    uint32_t int_ctrl;       // 0x00008L
    __RW    uint32_t fifo_stat;      // 0x0000CL
    __RW    uint32_t sdma_brst_sz;   // 0x00010L
    __RW    uint32_t hsm_cmd_req;    // 0x00014L
    __RW    uint32_t hsm_cmd_gnt;    // 0x00018L
            uint32_t reserved1[1];
    __RW    uint32_t src_ptr;        // 0x00020L
    __RW    uint32_t dst_ptr;        // 0x00024L
    __RW    uint32_t offset;         // 0x00028L
    __RW    uint32_t pre_add_len;    // 0x0002CL
    __RW    uint32_t post_add_len;   // 0x00030L
    __RW    uint32_t proc_len;       // 0x00034L
    __RW    uint32_t icv_len;        // 0x00038L
    __RW    uint32_t icv_offset;     // 0x0003CL
    __RW    uint32_t iv_offset;      // 0x00040L
    __RW    uint32_t sw_ctrl;        // 0x00044L
    __RW    uint32_t aux_info;       // 0x00048L
    __RW    uint32_t ctrl;           // 0x0004CL
    __RW    uint32_t stat_pop;       // 0x00050L
    __RW    uint32_t status;         // 0x00054L
            uint32_t reserved2[10];
    __RW    uint32_t stat_wd_ctrl;   // 0x00080L
            uint32_t reserved3[31];
    __RW    uint32_t key_sz;         // 0x00100L
            uint32_t reserved4[15];
    __RW    uint32_t virtual_rqst;   // 0x00140L
    __RW    uint32_t virtual_alloc;  // 0x00144L
    __RW    uint32_t virtual_prio;   // 0x00148L
            uint32_t reserved5[1];
    __RW    uint32_t virtual_rc4_key_rqst; // 0x00150L
    __RW    uint32_t virtual_rc4_key_gnt;  // 0x00154L
            uint32_t reserved6[10];
    __RW    uint32_t id;             // 0x00180L
    __RW    uint32_t config;         // 0x00184L
    __RW    uint32_t hsm_version;    // 0x00188L
            uint32_t reserved7[1];
    __RW    uint32_t config2;        // 0x00190L
            uint32_t reserved8[11];
    __RW    uint32_t secure_ctrl;    // 0x001C0L
    __RW    uint32_t secure_release; // 0x001C4L
            uint32_t reserved9[14];
    __RW    uint32_t sk_load;        // 0x00200L
    __RW    uint32_t sk_stat;        // 0x00204L
            uint32_t reserved10[14];
    __RW    uint32_t sk_key;         // 0x00240L
            uint32_t reserved11[47];
    __RW    uint32_t hsm_ctx_cmd;    // 0x00300L
    __RW    uint32_t hsm_ctx_stat;   // 0x00304L
            uint32_t reserved12[3902];
            uint8_t  ctx_ciph_key[4][0x40]; // 0x04000L     // for aes : 4 contexts * 64 bytes
            uint8_t  reserved13[0x4000-0x100];
            uint8_t  ctx_hash_key[4][0x40];  // 0x08000L     // 4 contexts * 64 bytes
} tSPACC_REG;

#define SPACC    ((tSPACC_REG*)AHB_SPACC)


/***********************************************************************************************
 *                                      ARIA START                                              *
************************************************************************************************/

#define ARIA_CNTR_BASE          (0x70000000)
#define ARIA_BASE_ADDR          (ARIA_CNTR_BASE + 0x00000)

typedef struct _t_ARIAREG {
    union _e_ARIA_CR {
        struct _t_ARIA_CR {
            __IOM uint32_t  ENABLE:1;
                  uint32_t  KEY_SIZE:2;
            __IOM uint32_t  MODE:2;
            __IOM uint32_t  RESERVED0:2;
            __IOM uint32_t  CCF_IE:1;
            __IOM uint32_t  ERR_IE:1;
            __IOM uint32_t  DMA_IN_EN:1;
            __IOM uint32_t  DMA_OUT_EN:1;
                  uint32_t  RESERVED1:21;
        }FIELD;
        __IOM  uint32_t FIELD_VAL;
    }ARIA_CR;

    union _e_ARIA_SR {
        struct _t_ARIA_SR {
            __IM uint32_t  INT_STS:1;
            __IM uint32_t  PD_ERR:1;
            __IM uint32_t  WR_ERR:2;
                 uint32_t  RESERVED0:28;
        }FIELD;
        __IM uint32_t  FIELD_VAL;
    }ARIA_SR;

    __OM  uint32_t  ARIA_DINR;
    __IM  uint32_t  ARIA_DOUTR;
    __IOM uint32_t  ARIA_KEYRR0;
    __IOM uint32_t  ARIA_KEYRR1;
    __IOM uint32_t  ARIA_KEYRR2;
    __IOM uint32_t  ARIA_KEYRR3;
    __IOM uint32_t  ARIA_KEYLR0;
    __IOM uint32_t  ARIA_KEYLR1;
    __IOM uint32_t  ARIA_KEYLR2;
    __IOM uint32_t  ARIA_KEYLR3;
    __IOM uint32_t  PERI_ID;
}t_ARIAREG;

// ARIA CR Register bit define
#define ARIA_KEY_SIZE_128B          0x00    /*!< 128 Bit ARIA KEY SIZE */
#define ARIA_KEY_SIZE_192B          0x01    /*!< 192 Bit ARIA KEY SIZE */
#define ARIA_KEY_SIZE_256B          0x02    /*!< 256 Bit ARIA KEY SIZE */
#define ARIA_KEY_SIZE_NOTUSED       0x03

#define ARIA_MODE_ENCRYPTION        0x00    /*!< ARIA Encryption Mode */
#define ARIA_MODE_NOT_USED0         0x01
#define ARIA_MODE_DECRYPTION        0x02    /*!< ARIA Decrption Mode */
#define ARIA_MODE_NOT_USED1         0x03

#define ARIA_REG_FIELD_ENABLE       0x01
#define ARIA_REG_FIELD_DISABLE      0x00

#define ARIA_PERI_PARTNUM           0xD30


#endif // __EWBM_REG_H__
