/**
  ******************************************************************************
  * @file    ewbm_clock.c
  * @author  eWBM
  * @version v0.1
  * @date    16-06-2016
  * @brief   Source file of clock setting
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

#include "ewbm_clock.h"

static void pll_control(void)
{
    __attribute__((unused))uint32_t val;
    uint32_t i;

    CLKRST->clk_ctrl = HS_IRC_EN | XTAL_EN;
    while((CLKRST->clk_sts & XTAL_RDY) != XTAL_RDY);

    CLKRST->sys_clk_ctrl = SYS_CLK_SEL(CLK_SRC_HS_IRC) | CPU_PRES(0) | APB_PRES(0);
    CLKRST->spif_clk_ctrl = SPIF_CLK_SEL(CLK_SRC_HS_IRC) | SPIF_CLK_EN;

    CLKRST->pll_ctrl = CLKRST->pll_ctrl | PLL_EN | PLL_BYPS;
    CLKRST->pll_ctrl = CLKRST->pll_ctrl & ~PLL_EN;
    CLKRST->pll_ctrl = CLKRST->pll_ctrl | PLL_EN | PLL_BYPS;

    CLKRST->pll_ctrl = PLL_CLK_SEL(CLK_SRC_XTAL) | PLL_FB_DIV(7) | PLL_PRE_DIV(0) | PLL_POST_DIV(0) | PLL_BYPS | PLL_RST | PLL_BWA(8);
    CLKRST->pll_ctrl = CLKRST->pll_ctrl | PLL_EN;

    /* wait for 5000 cycles, setup time from PLL setting to PD(power down) */
    for ( i=0; i<100; i++ )
        val = CLKRST->clk_sts;

    CLKRST->pll_ctrl = CLKRST->pll_ctrl & ~PLL_RST;
    CLKRST->pll_ctrl = CLKRST->pll_ctrl & ~PLL_BYPS;

    /* wait for 5000 cycles, setup time from PLL setting to PD(power down) */
    for ( i=0; i<100; i++ )
        val = CLKRST->clk_sts;

    /* PLL Control : PLL enable */
    CLKRST->pll_ctrl = CLKRST->pll_ctrl & ~PLL_RST;

    /* wait for 5000 cycles, setup time*/
    for ( i=0; i<500; i++ )
        val = CLKRST->clk_sts;

    /* PLL Control : PLL RST release */
    CLKRST->pll_ctrl = CLKRST->pll_ctrl & ~PLL_RST;
    while((CLKRST->clk_sts & PLL_LOCK) != PLL_LOCK);

    CLKRST->diva_ctrl = DIVx_CLK_SEL(CLK_SRC_PLL)  | DIVx_VAL(1);           // 96MHz
    while((CLKRST->clk_sts & DIVA_RDY) != DIVA_RDY);

    CLKRST->divb_ctrl = DIVx_CLK_SEL(CLK_SRC_DIVA) | DIVx_VAL(1);           // 48MHz
    while((CLKRST->clk_sts & DIVB_RDY) != DIVB_RDY);

    CLKRST->divc_ctrl = DIVx_CLK_SEL(CLK_SRC_DIVA) | DIVx_VAL(1);           // 48MHz
    while((CLKRST->clk_sts & DIVC_RDY) != DIVC_RDY);

    CLKRST->divd_ctrl = DIVx_CLK_SEL(CLK_SRC_DIVA) | DIVx_VAL(1) | DIVD_EN; // 24MHz
    while((CLKRST->clk_sts & DIVD_RDY) != DIVD_RDY);

    /* wait for status valid, setup time*/
    for ( i=0; i<100; i++ )
        val = CLKRST->clk_sts;
}

static void clock_select(void)
{
    CLKRST->sys_clk_ctrl  = SYS_CLK_SEL(CLK_SRC_DIVA) | CPU_PRES(0) | APB_PRES(1) |
                            AHB_CLK_LPEN | APB_CLK_LPEN | SRAM_CLK_LPEN | DS_CLK_MODE(1);
    while((CLKRST->clk_sts & (AHB_CLK_RDY|CPU_CLK_RDY|APB_CLK_RDY)) != (AHB_CLK_RDY|CPU_CLK_RDY|APB_CLK_RDY));

    CLKRST->spif_clk_ctrl = SPIF_CLK_SEL(CLK_SRC_PLL) | SPIF_CLK_EN;

    CLKRST->st_clk_ctrl   = ST_CLK_EN;
    while((CLKRST->clk_sts & ST_CLK_RDY) != ST_CLK_RDY);

    CLKRST->tmr_clk_ctrl  = TMR_CLK_EN(1) | TMR_CLK_LPEN(1) | TMR_CLK_EN(2) | TMR_CLK_LPEN(2);
    while((CLKRST->clk_sts & TMR_CLK_RDY) != TMR_CLK_RDY);

    CLKRST->peri_clk_sel  = UART_CLK_SEL(CLK_SRC_DIVB) | SPI_CLK_SEL(CLK_SRC_DIVB) | SDMMC_CLK_SEL(CLK_SRC_DIVA);

    CLKRST->peri_clk_en   = 0xFFFFFFFF;
    CLKRST->peri_clk_lpen = 0xFFFFFFFF;
}

void clock_init(void)
{
    SYSCON->pd_drv_str = 0xFFFFFFFF;   // QSPI Drive Strength 4mA
    SYSCON->pcd_smt    = 0x0000FFFF;   // QSPI Schmitt Trigger Off
    
#ifdef __QSPIBOOT__
    ICACHE->cache_config |= ICACHE_EN;                // I-Cache Prefetch Enable
    ICACHE->cache_ctrl   |= ICACHE_PREFETCH_EN;       // I-Cache Enable
#endif

    pll_control();
    clock_select();
}

