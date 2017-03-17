/**
  ******************************************************************************
  * @file    trng.c
  * @author  eWBM
  * @version v0.8
  * @date    20-05-2016
  * @brief   True Number Generator
  ******************************************************************************
  * @attention
  *
  * Copyright(c) 2015 ~ 2020 eWBM Korea , Ltd.
  * All rights reserved.
  * This software contains confidential information of eWBM Korea Co.,Ltd.
  * and unauthorized distribution of this software, or any portion of it, are
  * prohibited.
  *
  * Brief history
  * -------------
  *  2016-05-20 : Initialize and Gen/Get random number
  *  2016-05-22 : TRNG Initialize and Generate random number
  *  2016-05-24 : TRNG Re-seed and check the Re-seed occur
  *
  ******************************************************************************
  */

#include <string.h>
#include <stdint.h>
#include "trng.h"

/* IRQ bitfields (used for both IE and ISTAT registers) */
#define TRNG_IRQ_GLBL_EN    31
#define TRNG_IRQ_RQST_ALARM  3
#define TRNG_IRQ_AGE_ALARM   2
#define TRNG_IRQ_SEED_DONE   1
#define TRNG_IRQ_RAND_RDY    0

#define TRNG_IRQ_GLBL_EN_MASK    (1ul<<TRNG_IRQ_GLBL_EN)
#define TRNG_IRQ_RQST_ALARM_MASK (1ul<<TRNG_IRQ_RQST_ALARM)
#define TRNG_IRQ_AGE_ALARM_MASK  (1ul<<TRNG_IRQ_AGE_ALARM)
#define TRNG_IRQ_SEED_DONE_MASK  (1ul<<TRNG_IRQ_SEED_DONE)
#define TRNG_IRQ_RAND_RDY_MASK   (1ul<<TRNG_IRQ_RAND_RDY)

/* MODE register bitfields */
#define TRNG_MODE_R256               3

/* SMODE register bitfields */
#define TRNG_SMODE_MAX_REJECTS      16
#define TRNG_SMODE_MAX_REJECTS_BITS  8
#define TRNG_SMODE_SECURE            8
#define TRNG_SMODE_NONCE             2

/* STAT register bitfields */
#define TRNG_STAT_RESEEDING          31
#define TRNG_STAT_GENERATING         30
#define TRNG_STAT_SRVC_RQST          27
#define TRNG_STAT_RESEED_REASON      16
#define TRNG_STAT_RESEED_REASON_BITS  3
#define TRNG_STAT_SEEDED              9
#define TRNG_STAT_SECURE              8
#define TRNG_STAT_R256                3
#define TRNG_STAT_NONCE_MODE          2

#define TRNG_RANDOM_MAX_BYTE        32
#define TRNG_RANDOM_REMIND_BYTE     (TRNG_RANDOM_MAX_BYTE-1)



/* STAT.RESEED_REASON values */
typedef enum {
   TRNG_RESEED_HOST     = 0,
   TRNG_RESEED_NONCE    = 3,
   TRNG_RESEED_PIN      = 4,
   TRNG_RESEED_UNSEEDED = 7,
} eTRNG_RESEED;

/* CTRL register commands */
typedef enum {
   TRNG_CMD_NOP,
   TRNG_CMD_GEN_RAND,
   TRNG_CMD_RAND_RESEED,
   TRNG_CMD_NONCE_RESEED,
} eTRNG_CMD;


static int trng_is_reseeding(void)
{
    tTRNG_REG* baseAddr = TRNG;
    uint32_t   stat = baseAddr->stat;


    if ( (stat >> TRNG_STAT_RESEEDING) & 1 )
        return TRNG_OK;

    if ( (stat >> TRNG_STAT_SRVC_RQST) & 1 )
    {
        uint32_t istat = baseAddr->int_stat;
        if ( (istat >> TRNG_IRQ_SEED_DONE) & 1 )
            return TRNG_OK;
    }

    return TRNG_FAIL;
}

static int trng_rmw(volatile uint32_t* reg, uint32_t val, uint32_t mask)
{
    uint32_t tmp;

    tmp = *reg;
    tmp &= ~mask;
    tmp |= (val & mask);
    *reg = tmp;

    return tmp;
}

static void trng_random_number_generate(void)
{
    tTRNG_REG* baseAddr = TRNG;
    
    baseAddr->int_stat = TRNG_IRQ_RAND_RDY_MASK;
    baseAddr->mode     = 1UL << TRNG_MODE_R256;
    baseAddr->ctrl     = TRNG_CMD_GEN_RAND;

    while( ((baseAddr->int_stat & TRNG_IRQ_RAND_RDY_MASK) == 0) || ((baseAddr->stat & TRNG_STAT_GENERATING) == TRNG_STAT_GENERATING));
}

static int TRNG_initialized = 0;
void ESAL_TRNG_Init(void)
{
    tTRNG_REG* baseAddr = TRNG;
    uint32_t   runVal = 0;

    if (TRNG_initialized)
	return;
    TRNG_initialized = 1;

    baseAddr->smode = 0x000A0100;
    baseAddr->ctrl  = 0x2UL;

    do {
        runVal = baseAddr->stat;
        runVal &= 0x0200;
    } while ( runVal != 0x200 );
}

int ESAL_TRNG_GetRandomData(void* outbuf, uint32_t readSz, const void* reseed)
{
    tTRNG_REG* baseAddr = TRNG;
    int ret = TRNG_OK;
    int remaining = readSz;
    int copyLen;

    memset(outbuf, 0x0, readSz);

    if ( reseed != NULL ) {
        ret = ESAL_TRNG_ReSeed((const void*)reseed);
        if ( ret != TRNG_OK )
            return ret;
    } else {
        trng_random_number_generate();
    }

    do {
        copyLen = remaining;
        if (copyLen > TRNG_RANDOM_MAX_BYTE) {
            copyLen = TRNG_RANDOM_MAX_BYTE;
        }
        memcpy((void*)outbuf+readSz-remaining, (void*)&baseAddr->rand_base, copyLen);
        remaining -= copyLen;
        trng_random_number_generate();
    } while(remaining > 0);

    return TRNG_OK;
}

int ESAL_TRNG_ReSeed(const void* nonce)
{
    tTRNG_REG* baseAddr = TRNG;
    uint32_t   stat;
    uint32_t   smode;
    uint32_t   nonceBuf[8] = {0};
    int        i;

    if ( nonce )
    {
        smode = trng_rmw(&baseAddr->smode, -1, 1UL<<TRNG_SMODE_NONCE);
        memcpy((void*)nonceBuf,(void*)nonce,sizeof nonceBuf);

        do {
            stat = (baseAddr->stat >> TRNG_STAT_NONCE_MODE) & 0x1UL;
        } while( stat == 0 );

        for ( i=0; i<8; i++ )
            baseAddr->seed_base[i] = nonceBuf[i];

        baseAddr->ctrl  = TRNG_CMD_NONCE_RESEED;
        baseAddr->smode = (smode & ~(1UL<<TRNG_SMODE_NONCE));
    }
    else
    {
        trng_rmw(&baseAddr->smode, 0x0UL, 1UL<<TRNG_SMODE_NONCE);
        baseAddr->ctrl = TRNG_CMD_RAND_RESEED;
    }

    return trng_is_reseeding();
}

int ESAL_TRNG_GetSeed(void* out)
{
    tTRNG_REG* baseAddr = TRNG;
    uint32_t   seedBuf[8] = {0,};
    int i;

    if ( !((baseAddr->stat >> TRNG_STAT_SEEDED) & 0x1 ) )
        return TRNG_FAIL;

    for ( i=0; i<8; i++ )
        seedBuf[i] = baseAddr->seed_base[i];

    memcpy((void*)out, (void*)seedBuf, 8);

    return TRNG_OK;
}

