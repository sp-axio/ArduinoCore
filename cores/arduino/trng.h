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
#ifndef __TRNG_H__
#define __TRNG_H__

#include "ewbm_device.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

typedef enum
{
    TRNG_OK             = 0,
    TRNG_FAIL           = -1,
} eTRNG_ERROR;

void ESAL_TRNG_Init(void);
int  ESAL_TRNG_GetRandomData(void* out, uint32_t readSz, const void* reseed);
int  ESAL_TRNG_ReSeed(const void* nonce);
int  ESAL_TRNG_GetSeed(void* out);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
