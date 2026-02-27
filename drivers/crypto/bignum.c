/**
 * SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2021-2026 Mindgrove Technologies. All rights reserved.
 * 
 * @license Licensed under the Apache License, Version 2.0 (see LICENSE).
 * @licenseblock
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * @endlicenseblock
 * 
 * Project                   : Secure IoT SoC
 * @file   bignum_driver.c
 * @brief  This is the Big Number Integer Library Driver source file for multi-precision arithmetic.
 * @details Provides implementations to perform large integer operations
 *          beyond native data type limits.
 * @version 1.0
 * @authors Harini P (harinip@mindgrovetech.in)
 * @date 10-01-2026 
 * 
 * @section History
 * -----------------------------------------------------------------------------
 * Date       | Version | Modified by           | Description                   
 * -----------|---------|-----------------------|-------------------------------
 * 10-01-2026 | 1.0     | Harini P              | Initial release.              
 * -----------------------------------------------------------------------------
 */

#include "bignum.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>

static inline uint16_t BigNum_Count_Bits(const bn_int *a) {
    uint16_t bits = 0U;
    bn_digit top_limb;
    if (a->used == 0U) {
        return 0U;
    }
    bits = (a->used - 1U) * 64U;
    top_limb = a->dp[a->used - 1U];
    while (top_limb > 0U) {
        top_limb >>= 1;
        bits++;
    }
    return bits;
}

static inline bool BigNum_Get_Bit(const bn_int *a, uint16_t position) {
    uint16_t limb_idx = position / 64U;
    uint16_t bit_idx  = position % 64U;
    if (limb_idx >= a->used) {
        return 0;
    }
    return (a->dp[limb_idx] >> bit_idx) & 1U;
}

uint16_t Print_BigNum_Int_to_Hex(const bn_int *a) {
    uint8_t byte;
    //CHECK_NULL(a);

    if (a->used == 0U) {
        (void)printk("0\n");
        return 0;
    }
    for (int i = a->used - 1U; i >= 0; i--) {
            for (uint16_t j = 7U; j >= 0U; j--) {
                byte = (a->dp[i] >> (j * 8U)) & 0xFFU;
                (void)printk("%02x", byte);
            }
    }
    (void)printk("\n");
    return 0;
}

uint16_t BigNum_Set_Digit(bn_int *a, bn_digit d) {
    //CHECK_NULL(a);
    BN_ZERO(a);
    a->dp[0] = d;
    a->used = (d == 0U) ? 0U : 1U;
    a->sign = ZERO_POSITIVE;
    return 0;
}

/* Left shift a big integer by n bits */
uint16_t BigNum_Left_Shift(bn_int *a, uint16_t n) {
    uint16_t limb_shift;
    uint16_t bit_shift;
    //CHECK_NULL(a);

    limb_shift = n / 64U;
    bit_shift = n % 64U;

    // 1. Shift entire limbs
    if (limb_shift > 0U) {
        for (int i = a->used - 1U; i >= 0; i--) {
            if (((uint16_t)i + limb_shift) < BIGNUM_SIZE) {
                a->dp[(uint16_t)i + limb_shift] = a->dp[i];
            }
        }
        for (uint16_t i = 0U; i < limb_shift; i++) {
            a->dp[i] = 0U;
        }
        a->used += limb_shift;
    }

    // 2. Shift remaining bits
    if (bit_shift > 0U) {
        bn_digit carry = 0;
        for (uint16_t i = 0U; i < a->used; i++) {
            bn_digit next_carry = a->dp[i] >> (64U - bit_shift);
            a->dp[i] = (a->dp[i] << bit_shift) | carry;
            carry = next_carry;
        }
        // Handle carry out of the top limb
        if ((carry > 0U) && (a->used < BIGNUM_SIZE)) {
            a->dp[a->used++] = carry;
        }
    }
    BN_CLAMP(a);
    return 0;
}

/* General Modulus: r = a % b 
r = 0
for each bit i in a (MSB → LSB):
    r = r * 2 + i
    if r ≥ b:
        r = r - b
*/
uint16_t BigNum_Mod(const bn_int *a, const bn_int *b, bn_int *c) {
    uint16_t bit_len;
    bn_int result;
    uint16_t ret = 0;
    //CHECK_NULL(a);
    //CHECK_NULL(b);
    //CHECK_NULL(c);

    // Optimization: If a < b, then a % b = a
    if (BigNum_Compare(a, b) == LESS_THAN) {
        BN_COPY(a, c);
        return 0;
    }
    BN_ZERO(&result);

    // Bit-wise Long Division (MSB to LSB)
    bit_len = BigNum_Count_Bits(a);

    for (int16_t i = bit_len - 1U; i >= 0; i--) {
        // r = r << 1
        ret = BigNum_Left_Shift(&result, 1);

        if (ret == 0) {
            // r[0] |= (a_bit_at_i)
            if (BigNum_Get_Bit(a, i)) {
                result.dp[0] |= 1;
                if (result.used == 0U) {
                    result.used = 1U;
                }
            }
        }
        // if r >= b, r = r - b
        if ((ret == 0) && BigNum_Compare(&result, b) != LESS_THAN) {
            ret = BigNum_Subract(&result, b, &result);
        }
    }
    if (ret == 0) {
        BN_COPY(&result, c);
    }
    return 0;
}

uint16_t BigNum_Calculate_R2_Mod_N(const bn_int *n, bn_int *result) {
    bn_int r;
    uint16_t ret = 0;
    //CHECK_NULL(n);
    //CHECK_NULL(result);

    BN_ZERO(&r);

    /* r = 1 */
    r.dp[0] = 1;
    r.used  = 1U;
    r.sign  = ZERO_POSITIVE;

    /* Compute r = 2^4096 mod n */
    for (uint16_t i = 0U; i < 4096U; i++) {
        /* r <<= 1 */
        ret = BigNum_Left_Shift(&r, 1);

        /* if r >= n, r -= n */
        if ((ret == 0) && (BigNum_Compare(&r, n) != LESS_THAN)) {
            ret = BigNum_Subract(&r, n, &r);
        }
    }
    if (ret == 0) {
        BN_COPY(&r, result);
    }
    return ret;
}

int BigNum_Compare(const bn_int *a, const bn_int *b) {
    //CHECK_NULL(a);
    //CHECK_NULL(b);

    if (a->used > b->used) {
        return GREATER_THAN;
    }
    if (a->used < b->used) {
        return LESS_THAN;
    }
    for (int i = a->used - 1U; i >= 0; i--) {
        if (a->dp[i] > b->dp[i]) {
            return GREATER_THAN;
        }
        if (a->dp[i] < b->dp[i]) {
            return LESS_THAN;
        }
    }
    return EQUAL_TO;
}

int BigNum_Compare_Digit(const bn_int *a, bn_digit digit) {
    //CHECK_NULL(a);

    if (a->used > 1U) {
        return GREATER_THAN;
    }
    if (a->used == 0U) {
        return (digit == 0U) ? EQUAL_TO : LESS_THAN;
    }
    if (a->dp[0] > digit) {
        return GREATER_THAN;
    }
    if (a->dp[0] < digit) {
        return LESS_THAN;
    }

    return EQUAL_TO;
}

uint16_t BigNum_Subract(const bn_int *a, const bn_int *b, bn_int *c) {
    bn_digit borrow = 0U;
    uint16_t max_len;

    //CHECK_NULL(a);
    //CHECK_NULL(b);
    //CHECK_NULL(c);
    
    /* Basic subtraction a - b assuming a >= b */
    max_len = a->used;

    for (uint16_t i = 0U; i < max_len; i++) {
        bn_digit x = a->dp[i];
        bn_digit y = (i < b->used) ? b->dp[i] : 0U;

        unsigned __int128 diff = (unsigned __int128)x - (unsigned __int128)y - \
                                 (unsigned __int128)borrow;
        c->dp[i] = (bn_digit)diff;

        /* Extract borrow (MSB of 128-bit result) */
        borrow = (bn_digit)((diff >> 127U) & 1U);
    }

    c->used = max_len;
    BN_CLAMP(c);

    return 0;
}

uint16_t BigNum_Unsigned_Bin_Size(const bn_int *a, size_t *size) {
    size_t bit_count;
    //CHECK_NULL(a);
    //CHECK_NULL(size);

    if (a->used == 0U) {
        *size = 1U;
        return 0;
    }

    bit_count = BigNum_Count_Bits(a);
    *size = (bit_count / 8U + (((bit_count & 7U) != 0U) ? 1U : 0U));

    return 0;
}

uint16_t BigNum_Read_Unsigned_Bin(bn_int *a, const uint8_t *b, uint16_t len) {
    uint16_t limb_idx = 0U;
    uint16_t byte_in_limb = 0U;

    //CHECK_NULL(a);
    //CHECK_NULL(b);

    BN_ZERO(a);

    /* Iterate backwards so b[0] is MSB, b[len-1] is LSB */
    for (int16_t i = len - 1U; i >= 0; i--) {
        a->dp[limb_idx] |= (bn_digit)(((bn_digit)b[i]) << \
                           (uint16_t)(byte_in_limb * (uint16_t)8U));
        byte_in_limb++;
        if (byte_in_limb == 8U) {
            byte_in_limb = 0U;
            limb_idx++;
        }
    }
    a->used = limb_idx + 1U;
    BN_CLAMP(a);
    a->sign = ZERO_POSITIVE;
    return 0;
}

uint16_t BigNum_Write_Unsigned_Bin(const bn_int *a, uint8_t *b, uint16_t len) {
    uint16_t byte_count;
    uint16_t  out_idx;
    uint16_t limb;
    bn_digit digit;
    //CHECK_NULL(a);
    //CHECK_NULL(b);

    /* Determine required byte count */
    byte_count = (a->used == 0U) ? 1U : ((BigNum_Count_Bits(a) + 7U) / 8U);

    /* Buffer size validation */
    if (byte_count > len) {
        //log_emit(ERROR, "Output buffer too small\r\n");
        //return EINVAL;
    }

    /* Handle zero case */
    if (a->used == 0U) {
        b[0] = 0U;
        return 0;
    }

    /* Zero output buffer */
    for (uint16_t i = 0U; i < byte_count; i++) {
        b[i] = 0U;
    }
    out_idx = byte_count - 1U;

    /* Write limbs to output buffer */
    for (limb = 0U; limb < a->used; limb++) {
        digit = a->dp[limb];
        for (uint32_t j = 0U; (j < 8U) && (out_idx >= 0U); j++) {
            b[out_idx] = (uint8_t)(digit & 0xFFU);
            digit >>= 8U;
            out_idx--;
        }
    }
    return 0;
}
