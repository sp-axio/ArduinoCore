/**
  ******************************************************************************
  * @file    clp300_ecc.c
  * @author  eWBM
  * @version  0.7
  * @date    2016-06-13
  * @brief   Public Key Accelerator Firmware for ECC
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
  *  2016-06-09 : Draft by Leon Cho
  *  2016-06-10 : Point Addition add
  *  2016-06-13 : Implement ECC Point arithmetic operation
  *
  ******************************************************************************
  */
#include "clp300.h"

#ifndef ECC_MINIMAL
static const tPKA_CURVE_DATA SEC192R1 = {
    24,
    {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0x18, 0x8D, 0xA8, 0x0E, 0xB0, 0x30, 0x90, 0xF6, 0x7C, 0xBF, 0x20, 0xEB, 0x43, 0xA1, 0x88, 0x00, 0xF4, 0xFF, 0x0A, 0xFD, 0x82, 0xFF, 0x10, 0x12},
    {0x07, 0x19, 0x2B, 0x95, 0xFF, 0xC8, 0xDA, 0x78, 0x63, 0x10, 0x11, 0xED, 0x6B, 0x24, 0xCD, 0xD5, 0x73, 0xF9, 0x77, 0xA1, 0x1E, 0x79, 0x48, 0x11},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC},
    {0x64, 0x21, 0x05, 0x19, 0xE5, 0x9C, 0x80, 0xE7, 0x0F, 0xA7, 0xE9, 0xAB, 0x72, 0x24, 0x30, 0x49, 0xFE, 0xB8, 0xDE, 0xEC, 0xC1, 0x46, 0xB9, 0xB1},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x99, 0xDE, 0xF8, 0x36, 0x14, 0x6B, 0xC9, 0xB1, 0xB4, 0xD2, 0x28, 0x31},
    "SECP_192R1",
    {0x28, 0xBE, 0x56, 0x77, 0xEA, 0x05, 0x81, 0xA2, 0x46, 0x96, 0xEA, 0x5B, 0xBB, 0x3A, 0x6B, 0xEE, 0xCE, 0x66, 0xBA, 0xCC, 0xDE, 0xB3, 0x59, 0x61,},
    {0x8F, 0x63, 0xC8, 0x41, 0xCC, 0x4C, 0xC3, 0xF7, 0x9E, 0x9E, 0x1A, 0x9C, 0xB7, 0x9D, 0x94, 0x8E, 0x88, 0x26, 0x72, 0x07, 0x0D, 0xDB, 0xCF, 0x2F,},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x99, 0xDE, 0xF8, 0x36, 0x14, 0x6B, 0xC9, 0xB1, 0xB4, 0xD2, 0x28, 0x31,},
};

static const tPKA_CURVE_DATA SEC224R1 = {
    28,
    {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xB7, 0x0E, 0x0C, 0xBD, 0x6B, 0xB4, 0xBF, 0x7F, 0x32, 0x13, 0x90, 0xB9, 0x4A, 0x03, 0xC1, 0xD3, 0x56, 0xC2, 0x11, 0x22, 0x34, 0x32, 0x80, 0xD6, 0x11, 0x5C, 0x1D, 0x21},
    {0xBD, 0x37, 0x63, 0x88, 0xB5, 0xF7, 0x23, 0xFB, 0x4C, 0x22, 0xDF, 0xE6, 0xCD, 0x43, 0x75, 0xA0, 0x5A, 0x07, 0x47, 0x64, 0x44, 0xD5, 0x81, 0x99, 0x85, 0x00, 0x7E, 0x34},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE},
    {0xB4, 0x05, 0x0A, 0x85, 0x0C, 0x04, 0xB3, 0xAB, 0xF5, 0x41, 0x32, 0x56, 0x50, 0x44, 0xB0, 0xB7, 0xD7, 0xBF, 0xD8, 0xBA, 0x27, 0x0B, 0x39, 0x43, 0x23, 0x55, 0xFF, 0xB4},
    {0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x16, 0xA2, 0xE0, 0xB8, 0xF0, 0x3E, 0x13, 0xDD, 0x29, 0x45, 0x5C, 0x5C, 0x2A, 0x3D},
    "SECP_224R1",
    {0xD4, 0xBA, 0xA4, 0xCF, 0x18, 0x22, 0xBC, 0x47, 0xB1, 0xE9, 0x79, 0x61, 0x6A, 0xD0, 0x9D, 0x91, 0x97, 0xA5, 0x45, 0x52, 0x6B, 0xDA, 0xAE, 0x6C, 0x3A, 0xD0, 0x12, 0x89,},
    {0xF8, 0x61, 0xAC, 0x50, 0xD8, 0xEA, 0x6C, 0xEC, 0x38, 0x25, 0x44, 0x85, 0x29, 0xDE, 0xB3, 0x36, 0x6D, 0x62, 0x58, 0x7F, 0xD6, 0xE2, 0x42, 0x70, 0x6A, 0x1F, 0xC2, 0xEB,},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x16, 0xA2, 0xE0, 0xB8, 0xF0, 0x3E, 0x13, 0xDD, 0x29, 0x45, 0x5C, 0x5C, 0x2A, 0x3D,},
};
#endif

static const tPKA_CURVE_DATA SEC256R1 = {
    32,
    {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0x6B, 0x17, 0xD1, 0xF2, 0xE1, 0x2C, 0x42, 0x47, 0xF8, 0xBC, 0xE6, 0xE5, 0x63, 0xA4, 0x40, 0xF2, 0x77, 0x03, 0x7D, 0x81, 0x2D, 0xEB, 0x33, 0xA0, 0xF4, 0xA1, 0x39, 0x45, 0xD8, 0x98, 0xC2, 0x96},
    {0x4F, 0xE3, 0x42, 0xE2, 0xFE, 0x1A, 0x7F, 0x9B, 0x8E, 0xE7, 0xEB, 0x4A, 0x7C, 0x0F, 0x9E, 0x16, 0x2B, 0xCE, 0x33, 0x57, 0x6B, 0x31, 0x5E, 0xCE, 0xCB, 0xB6, 0x40, 0x68, 0x37, 0xBF, 0x51, 0xF5},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0x00, 0x00, 0x00, 0x04, 0xFF, 0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFB, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03},
    {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC},
    {0x5A, 0xC6, 0x35, 0xD8, 0xAA, 0x3A, 0x93, 0xE7, 0xB3, 0xEB, 0xBD, 0x55, 0x76, 0x98, 0x86, 0xBC, 0x65, 0x1D, 0x06, 0xB0, 0xCC, 0x53, 0xB0, 0xF6, 0x3B, 0xCE, 0x3C, 0x3E, 0x27, 0xD2, 0x60, 0x4B},
    {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBC, 0xE6, 0xFA, 0xAD, 0xA7, 0x17, 0x9E, 0x84, 0xF3, 0xB9, 0xCA, 0xC2, 0xFC, 0x63, 0x25, 0x51},
    "SECP_256R1",
    {0x66, 0xE1, 0x2D, 0x94, 0xF3, 0xD9, 0x56, 0x20, 0x28, 0x45, 0xB2, 0x39, 0x2B, 0x6B, 0xEC, 0x59, 0x46, 0x99, 0x79, 0x9C, 0x49, 0xBD, 0x6F, 0xA6, 0x83, 0x24, 0x4C, 0x95, 0xBE, 0x79, 0xEE, 0xA2},
    {0x60, 0xD0, 0x66, 0x33, 0xA9, 0xD6, 0x28, 0x1C, 0x50, 0xFE, 0x77, 0xEC, 0xC5, 0x88, 0xC6, 0xF6, 0x48, 0xC9, 0x44, 0x08, 0x7D, 0x74, 0xD2, 0xE4, 0xCC, 0xD1, 0xC8, 0xAA, 0xEE, 0x00, 0xBC, 0x4F},
    {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBC, 0xE6, 0xFA, 0xAD, 0xA7, 0x17, 0x9E, 0x84, 0xF3, 0xB9, 0xCA, 0xC2, 0xFC, 0x63, 0x25, 0x51},
};

#ifndef ECC_MINIMAL
static const tPKA_CURVE_DATA SEC384R1 = {
    48,
    {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF},
    {0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xFF, 0xFA, 0xFF, 0xFF, 0xFF, 0xFB, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01},
    {0xAA, 0x87, 0xCA, 0x22, 0xBE, 0x8B, 0x05, 0x37, 0x8E, 0xB1, 0xC7, 0x1E, 0xF3, 0x20, 0xAD, 0x74, 0x6E, 0x1D, 0x3B, 0x62, 0x8B, 0xA7, 0x9B, 0x98, 0x59, 0xF7, 0x41, 0xE0, 0x82, 0x54, 0x2A, 0x38, 0x55, 0x02, 0xF2, 0x5D, 0xBF, 0x55, 0x29, 0x6C, 0x3A, 0x54, 0x5E, 0x38, 0x72, 0x76, 0x0A, 0xB7},
    {0x36, 0x17, 0xDE, 0x4A, 0x96, 0x26, 0x2C, 0x6F, 0x5D, 0x9E, 0x98, 0xBF, 0x92, 0x92, 0xDC, 0x29, 0xF8, 0xF4, 0x1D, 0xBD, 0x28, 0x9A, 0x14, 0x7C, 0xE9, 0xDA, 0x31, 0x13, 0xB5, 0xF0, 0xB8, 0xC0, 0x0A, 0x60, 0xB1, 0xCE, 0x1D, 0x7E, 0x81, 0x9D, 0x7A, 0x43, 0x1D, 0x7C, 0x90, 0xEA, 0x0E, 0x5F},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x01},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFC},
    {0xB3, 0x31, 0x2F, 0xA7, 0xE2, 0x3E, 0xE7, 0xE4, 0x98, 0x8E, 0x05, 0x6B, 0xE3, 0xF8, 0x2D, 0x19, 0x18, 0x1D, 0x9C, 0x6E, 0xFE, 0x81, 0x41, 0x12, 0x03, 0x14, 0x08, 0x8F, 0x50, 0x13, 0x87, 0x5A, 0xC6, 0x56, 0x39, 0x8D, 0x8A, 0x2E, 0xD1, 0x9D, 0x2A, 0x85, 0xC8, 0xED, 0xD3, 0xEC, 0x2A, 0xEF},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC7, 0x63, 0x4D, 0x81, 0xF4, 0x37, 0x2D, 0xDF, 0x58, 0x1A, 0x0D, 0xB2, 0x48, 0xB0, 0xA7, 0x7A, 0xEC, 0xEC, 0x19, 0x6A, 0xCC, 0xC5, 0x29, 0x73},
    "SECP_384R1",
    {0x0C, 0x84, 0xEE, 0x01, 0x2B, 0x39, 0xBF, 0x21, 0x3F, 0xB0, 0x5B, 0x7A, 0x28, 0x26, 0x68, 0x95, 0xD4, 0x0D, 0x49, 0x17, 0x4A, 0xAB, 0x1C, 0xC5, 0xBC, 0x3E, 0x48, 0x3A, 0xFC, 0xB8, 0x29, 0x47, 0xFF, 0x3D, 0x81, 0xE5, 0xDF, 0x1A, 0xA4, 0x19, 0x2D, 0x31, 0x9B, 0x24, 0x19, 0xB4, 0x09, 0xA9, },
    {0x35, 0x5C, 0xA8, 0x7D, 0xE3, 0x9D, 0xBB, 0x1F, 0xA1, 0x50, 0x20, 0x6C, 0xE4, 0xF1, 0x94, 0xAC, 0x78, 0xD4, 0xBA, 0x58, 0x66, 0xD6, 0x17, 0x87, 0xEE, 0x6C, 0x8E, 0x3D, 0xF4, 0x56, 0x24, 0xCE, 0x54, 0xA8, 0x85, 0x99, 0x5D, 0x20, 0xBB, 0x2B, 0x6E, 0xD4, 0x60, 0x89, 0xE8, 0x8F, 0xDC, 0x45, },
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC7, 0x63, 0x4D, 0x81, 0xF4, 0x37, 0x2D, 0xDF, 0x58, 0x1A, 0x0D, 0xB2, 0x48, 0xB0, 0xA7, 0x7A, 0xEC, 0xEC, 0x19, 0x6A, 0xCC, 0xC5, 0x29, 0x73, },
};
#endif




extern int get_pka_flags(void);

static int pka_ecc_load_curve_data(tPKA_CURVE_DATA* pc)
{
    int ret = CRYPTO_OK;

    tPKA_CMD_LIST cmd[] = {
        {
            .reg_name = (uint8_t*)"D0",
            .reg_val = (uint8_t*)pc->m
        },
        {
            .reg_name = (uint8_t*)"D1",
            .reg_val = (uint8_t*)pc->mp
        },
        {
            .reg_name = (uint8_t*)"D3",
            .reg_val = (uint8_t*)pc->r
        }
    };

    ret = ESAL_PKA_WriteOperandMultiple(cmd, sizeof(cmd)/sizeof(cmd[0]), pc->size);

    return ret;
}

static int pka_ecc_load_curve_pver(tPKA_CURVE_DATA* pc)
{
    int ret = CRYPTO_OK;

    tPKA_CMD_LIST cmd[] = {
        {
            .reg_name = (uint8_t*)"A6",
            .reg_val = (uint8_t*)pc->a
        },
        {
            .reg_name = (uint8_t*)"A7",
            .reg_val = (uint8_t*)pc->b
        }
    };

    ret = ESAL_PKA_WriteOperandMultiple(cmd, sizeof(cmd)/sizeof(cmd[0]), pc->size);

    return ret;
}

static int pka_ecc_load_curv_pmult(tPKA_CURVE_DATA* pc, uint8_t loadbase)
{
    int ret = CRYPTO_OK;

    if ( loadbase != 0 )
    {
        ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"A2", pc->x, pc->size);
        if ( ret != CRYPTO_OK )
            return ret;

        ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"B2", pc->y, pc->size);
        if ( ret != CRYPTO_OK )
            return ret;
    }

    ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"A6", pc->a, pc->size);

    return ret;
}

static tPKA_CURVE_DATA* pc = NULL;
void ESAL_PKA_ECC_SetSECPCurveParam(ePKA_CURVE_PARAM curv)
{
    switch ( curv )
    {
#ifndef ECC_MINIMAL
        case SECP_192R1:
            pc = (tPKA_CURVE_DATA*)&SEC192R1;
            break;

        case SECP_224R1:
            pc = (tPKA_CURVE_DATA*)&SEC224R1;
            break;
#endif

        case SECP_256R1:
            pc = (tPKA_CURVE_DATA*)&SEC256R1;
            break;

#ifndef ECC_MINIMAL
        case SECP_384R1:
            pc = (tPKA_CURVE_DATA*)&SEC384R1;
            break;
#endif

        default:
            pc = NULL;
    }
}

tPKA_CURVE_DATA* ESAL_PKA_ECC_GetSECPCurveParam(void)
{
    return pc;
}

int ESAL_PKA_ECC_PointVerification(tPKA_CURVE_DATA* pc, uint8_t* rx, uint8_t* ry)
{
    int ret = CRYPTO_OK;

    tPKA_CMD_LIST cmd[] = {
        {
            .reg_name = (uint8_t*)"A2",
            .reg_val = rx
        },
        {
            .reg_name = (uint8_t*)"B2",
            .reg_val = ry
        }
    };

    CRYPTO_ASSERT(!pc);

    if ( !rx && !ry )
    {
        cmd[0].reg_val = (uint8_t*)pc->x;
        cmd[1].reg_val = (uint8_t*)pc->y;
    }

    ret = pka_ecc_load_curve_data(pc);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = pka_ecc_load_curve_pver(pc);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_WriteOperandMultiple(cmd, sizeof(cmd)/sizeof(cmd[0]), pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_PVER, 0, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    if ( (get_pka_flags() & (1ul << PKA_FLAG_ZERO)) <= 0 )
        return CRYPTO_FAIL;

    return CRYPTO_OK;
}

int ESAL_PKA_ECC_PointAddition(tPKA_CURVE_DATA* pc, uint8_t* px, uint8_t* py, uint8_t* qx, uint8_t* qy)
{
    int ret = CRYPTO_OK;

    tPKA_CMD_LIST cmd[] = {
        {
            .reg_name = (uint8_t*)"A6",
            .reg_val = (uint8_t*)pc->a
        },
        {
            .reg_name = (uint8_t*)"A2",
            .reg_val = (uint8_t*)px,
        },
        {
            .reg_name = (uint8_t*)"B2",
            .reg_val = (uint8_t*)py
        },
        {
            .reg_name = (uint8_t*)"A3",
            .reg_val = (uint8_t*)qx,
        },
        {
            .reg_name = (uint8_t*)"B3",
            .reg_val = (uint8_t*)qy
        }
    };

    CRYPTO_ASSERT(!pc);

    ret = pka_ecc_load_curve_data(pc);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_WriteOperandMultiple(cmd, sizeof(cmd)/sizeof(cmd[0]), pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_PADD, 0, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"A2", (void*)qx, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"B2", (void*)qy, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    return CRYPTO_OK;
}

int ESAL_PKA_ECC_PointMultiflication(tPKA_CURVE_DATA* pc, int keysz, int loadbase, uint8_t* key, uint8_t* blind, uint8_t* px, uint8_t* py)
{
    int ret = CRYPTO_OK;

    tPKA_CMD_LIST cmd[] = {
        {
            .reg_name = (uint8_t*)"A2",
            .reg_val = px
        },
        {
            .reg_name = (uint8_t*)"B2",
            .reg_val = py
        }
    };

    if ( blind )
    {
        ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"A7", (void*)blind, pc->size);
        if ( ret != CRYPTO_OK )
            return ret;
    }
    else
    {
        ret = ESAL_PKA_ZeroWriteOperandRegister((uint8_t*)"A7", pc->size);
        if ( ret != CRYPTO_OK )
            return ret;
    }
    
    ret = pka_ecc_load_curve_data(pc);
    if ( ret != CRYPTO_OK )
        return ret;
    
    ret = pka_ecc_load_curv_pmult(pc, loadbase);
    if ( ret != CRYPTO_OK )
        return ret;
    
    if ( key )
    {
        ret = ESAL_PKA_WriteOperandRegister((uint8_t*)"D7", (void*)key, keysz);
        if ( ret != CRYPTO_OK )
            return ret;
    }
    
    if ( !loadbase )
    {
        ret = ESAL_PKA_WriteOperandMultiple(cmd, sizeof(cmd)/sizeof(cmd[0]), pc->size);
        if ( ret != CRYPTO_OK )
            return ret;
    }
    
    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_PMULT, 0, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"A2", (void*)px, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"B2", (void*)py, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    return CRYPTO_OK;
}

int ESAL_PKA_ECC_PointDouble(tPKA_CURVE_DATA* pc, uint8_t* px, uint8_t* py)
{
    int ret = CRYPTO_OK;

    tPKA_CMD_LIST cmd[] = {
        {
            .reg_name = (uint8_t*)"A6",
            .reg_val = pc->a
        },
        {
            .reg_name = (uint8_t*)"A3",
            .reg_val = px
        },
        {
            .reg_name = (uint8_t*)"B3",
            .reg_val = py
        }
    };

    ret = pka_ecc_load_curve_data(pc);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_WriteOperandMultiple(cmd, sizeof(cmd)/sizeof(cmd[0]), pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_PDBL, 0, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"A2", (void*)px, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"B2", (void*)py, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    return CRYPTO_OK;
}

int ESAL_PKA_ECC_Shamir(tPKA_CURVE_DATA* pc, uint8_t* qx, uint8_t* qy, uint8_t* k1, uint8_t* k2)
{
    int ret = CRYPTO_OK;

    tPKA_CMD_LIST cmd[] = {
        {
            .reg_name = (uint8_t*)"A7",
            .reg_val = k1
        },
        {
            .reg_name = (uint8_t*)"D7",
            .reg_val = k2
        },
        {
            .reg_name = (uint8_t*)"A3",
            .reg_val = qx
        },
        {
            .reg_name = (uint8_t*)"B3",
            .reg_val = qy
        }
    };

    ret = pka_ecc_load_curve_data(pc);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = pka_ecc_load_curv_pmult(pc, 1);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_WriteOperandMultiple(cmd, sizeof(cmd)/sizeof(cmd[0]), pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_FirmwareStart(PKA_ENTRY_SHAMIR,0,pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"A3", (void*)qx, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    ret = ESAL_PKA_ReadOperandRegister((uint8_t*)"B3", (void*)qy, pc->size);
    if ( ret != CRYPTO_OK )
        return ret;

    return CRYPTO_OK;

}

int ESAL_PKA_ECC_Init(void)
{
    uint32_t endian = 0;

    ESAL_PKA_RegisterIoctl(PKA_CONF, &endian, PKA_READ_REG);

    return CRYPTO_OK;
}
