/*
 *  Minimal configuration for TLS 1.1 (RFC 4346)
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */
/*
 * Minimal configuration for TLS 1.1 (RFC 4346), implementing only the
 * required ciphersuite: MBEDTLS_TLS_RSA_WITH_3DES_EDE_CBC_SHA
 *
 * See README.txt for usage instructions.
 */

#ifndef MBEDTLS_CONFIG_H
#define MBEDTLS_CONFIG_H

/* System support */
//#define MBEDTLS_HAVE_ASM
//#define MBEDTLS_HAVE_TIME

/* mbed TLS feature support */
#define MBEDTLS_CIPHER_MODE_CBC
#define MBEDTLS_CIPHER_MODE_CTR
#define MBEDTLS_CIPHER_MODE_CFB

#define MBEDTLS_PKCS1_V15
#define MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED
#define MBEDTLS_SSL_PROTO_TLS1_1

#define MBEDTLS_SSL_DTLS_HELLO_VERIFY
#define MBEDTLS_SSL_PROTO_DTLS
#define MBEDTLS_SSL_COOKIE_C
//#define MBEDTLS_TIMING_C

/* mbed TLS modules */
#define MBEDTLS_AES_C
#define MBEDTLS_AES_ALT
#define MBEDTLS_GCM_C
#define MBEDTLS_CCM_C
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ECP_ALT
//#define MBEDTLS_ECP_DP_SECP192R1_ENABLED
//#define MBEDTLS_ECP_DP_SECP224R1_ENABLED
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_DP_SECP384R1_ENABLED
#define MBEDTLS_SHA1_C
//#define MBEDTLS_SHA1_ALT
#define MBEDTLS_SHA256_C
//#define MBEDTLS_SHA256_ALT
#define MBEDTLS_ASN1_PARSE_C
#define MBEDTLS_ASN1_WRITE_C
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_BIGNUM_C_ALT
#define MBEDTLS_CIPHER_C
#define MBEDTLS_CTR_DRBG_C
//#define MBEDTLS_DES_C
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_ENTROPY_HARDWARE_ALT
#define MBEDTLS_MD_C
#define MBEDTLS_MD5_C
#define MBEDTLS_OID_C
#define MBEDTLS_PK_C
#define MBEDTLS_PK_PARSE_C
//#define MBEDTLS_RSA_C
#define MBEDTLS_SSL_CLI_C
#define MBEDTLS_SSL_TLS_C
#define MBEDTLS_X509_CRT_PARSE_C
#define MBEDTLS_X509_USE_C

/* For test certificates */
#define MBEDTLS_BASE64_C
#define MBEDTLS_CERTS_C
#define MBEDTLS_PEM_PARSE_C
#define MBEDTLS_PEM_WRITE_C
#define MBEDTLS_X509_CREATE_C
#define MBEDTLS_X509_CSR_WRITE_C
#define MBEDTLS_PK_WRITE_C
#define MBEDTLS_X509_CSR_PARSE_C

#define MBEDTLS_PLATFORM_C
#define MBEDTLS_PLATFORM_PRINTF_ALT

//#define MBEDTLS_SELF_TEST
#define MBEDTLS_DEBUG_C

/* For testing with compat.sh */
//#define MBEDTLS_FS_IO
//#define MBEDTLS_NET_C
//#define MBEDTLS_SSL_SRV_C

// MBEDTLS_SSL_MAX_CONTENT_LEN 4096		// default 16384

#include "mbedtls/check_config.h"

#endif /* MBEDTLS_CONFIG_H */
