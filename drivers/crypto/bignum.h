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
 * @file   bignum.h
 * @brief  This is the Big Number Integer Library header file for multi-precision
 *         arithmetic.
 * @details Provides definitions and prototypes to perform large integer
 *          operations beyond native data type limits.
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

#ifndef BSP_INCLUDE_BIGNUM_H_
#define BSP_INCLUDE_BIGNUM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
// #include "io.h"

/**
 * @defgroup BIGNUM_SIZE_MACROS Big Number Size Macros
 * @brief Macros defining digit size and storage capacity.
 * @{
 */

/** @brief Maximum number of digits (limbs) supported. */
#define BIGNUM_SIZE   65U

/** @brief Number of bits per digit (limb). */
#define DIGIT_BIT     64U
/** @} */

/**
 * @defgroup BIGNUM_SIGN_MACROS Big Number Sign Macros
 * @brief Macros representing the sign of a big number.
 * @{
 */

/** @brief Represents zero or positive sign. */
#define ZERO_POSITIVE   0U

/** @brief Represents negative sign. */
#define NEGATIVE        1U
/** @} */

/**
 * @defgroup BIGNUM_COMPARISON_MACROS Big Number Comparison Macros
 * @brief Macros defining comparison and boolean results.
 * @{
 */

/** @brief Boolean YES value. */
#define YES             1

/** @brief Boolean NO value. */
#define NO              0

/** @brief Indicates equality result. */
#define EQUAL_TO        0

/** @brief Indicates less-than result. */
#define LESS_THAN      -1

/** @brief Indicates greater-than result. */
#define GREATER_THAN    1
/** @} */

/**
 * @defgroup BIGNUM_UTILITY_MACROS Big Number Utility Macros
 * @brief Macros for initialization and basic operations.
 * @{
 */

/** @brief Initializes a big number structure to zero. */
#define BN_INIT(a)      (void *)memset((a), 0, sizeof(bn_int))

/** @brief Sets a big number to zero. */
#define BN_ZERO(a)      BN_INIT(a)

/** @brief Checks if the big number is zero. */
#define BN_IS_ZERO(a)   (((a)->used == 0) ? YES : NO)

/** @brief Checks if the big number is even. */
#define BN_IS_EVEN(a)   (((a)->used == 0 || (((a)->dp[0] & 1U) == 0U)) \
                        ? YES : NO)

/** @brief Checks if the big number is odd. */
#define BN_IS_ODD(a)    (((a)->used != 0 && (((a)->dp[0] & 1U) != 0U)) \
                        ? YES : NO)

/** @brief Removes leading zero digits from the big number. */
#define BN_CLAMP(a)     do {                                             \
                            while (((a)->used > 0U) &&                    \
                                ((a)->dp[(a)->used - 1U] == 0U)) {         \
                                --((a)->used);                           \
                            }                                            \
                        } while (0)

/** @brief Copies one big number into another. */
#define BN_COPY(a, b)   do {                                                 \
                            if ((a) != (b)) {                                \
                                (void)memcpy((b), (a), sizeof(bn_int));            \
                            }                                                \
                        } while (0)

/** @brief Computes absolute value of a big number. */
#define BN_ABS(a, b)    { BN_COPY(a, b); (b)->sign = ZERO_POSITIVE; }
/** @} */

/**
 * @brief Defines the digit type for big numbers.
 *
 * @details Each big integer is stored as an array of 64-bit unsigned
 *          digits (limbs) using this type.
 */
typedef uint64_t bn_digit;

/**
 * @brief Big integer structure.
 *
 * @details Represents a multi-precision integer using an array of
 *          fixed-size digits. It stores the digit array, the number
 *          of active digits, and the sign of the value.
 */
typedef struct {
    /** Array of digits (least significant limb at index 0) */
    bn_digit dp[BIGNUM_SIZE];

    /** Number of active digits currently used (0 indicates the value is zero)*/
    uint16_t used;

    /** Sign of the integer (POSITIVE (include zero) or NEGATIVE) */
    uint8_t sign;
} bn_int;

/* Function prototypes */

/**
 * @brief Prints a big integer in hexadecimal format.
 * 
 * @details This function prints the given big integer in hexadecimal
 *          representation, starting from the most significant limb.
 *          If the value is zero, it prints 0.
 * 
 * @param a Pointer to the bn_int structure to be printed.
 * 
 * @return Returns SUCCESS on successful print; otherwise returns error
 *         if the input pointer is NULL.
 */
uint16_t Print_BigNum_Int_to_Hex(const bn_int *a);

/**
 * @brief Sets a big integer to a single digit value.
 * 
 * @details This function initializes the big integer to the specified
 *          digit value.
 * 
 * @param a Pointer to the bn_int to be initialized.
 * @param d Digit value to assign.
 * 
 * @return Returns SUCCESS on success; otherwise returns error if
 *         the pointer is NULL.
 */
uint16_t BigNum_Set_Digit(bn_int *a, bn_digit d);

/**
 * @brief Performs left shift operation on a big integer.
 * 
 * @details This function shifts the big integer left by n bits.
 *          It handles both limb-level and bit-level shifts and
 *          updates the used length accordingly.
 * 
 * @param a Pointer to the bn_int structure to be shifted.
 * @param n Number of bits to shift.
 * 
 * @return Returns SUCCESS on successful shift.
 */
uint16_t BigNum_Left_Shift(bn_int *a, uint16_t n);

/**
 * @brief Computes modulus of two big integers.
 * 
 * @details This function computes c = a % b using bit-wise long
 *          division from MSB to LSB.
 * 
 * @param a Pointer to dividend.
 * @param b Pointer to divisor.
 * @param c Pointer to result (remainder).
 * 
 * @return Returns SUCCESS on success; otherwise returns error
 *         if any pointer is NULL.
 */
uint16_t BigNum_Mod(const bn_int *a, const bn_int *b, bn_int *c);

/**
 * @brief Computes R^2 mod n.
 * 
 * @details This function computes 2^4096 mod n using repeated 
 *          left shifts and modular reduction at each step.
 * 
 * @param n Pointer to modulus.
 * @param result Pointer to store computed result.
 * 
 * @return Returns SUCCESS on success; otherwise returns error
 *         if any pointer is NULL.
 */
uint16_t BigNum_Calculate_R2_Mod_N(const bn_int *n, bn_int *result);

/**
 * @brief Compares two big integers.
 * 
 * @details This function compares two big integers and determines
 *          whether one is greater than, less than, or equal to the other.
 * 
 * @param a Pointer to first operand.
 * @param b Pointer to second operand.
 * 
 * @return Returns 1 if , BN_LT, or BN_EQ. Returns error if pointer is NULL.
 */
int BigNum_Compare(const bn_int *value1, const bn_int *value2);

/**
 * @brief Compares a big integer with a single digit.
 * 
 * @details This function compares the big integer with a given
 *          digit value.
 * 
 * @param a Pointer to big integer.
 * @param digit Digit value for comparison.
 * 
 * @return Returns BN_GT, BN_LT, or BN_EQ. Returns error if pointer is NULL.
 */
int BigNum_Compare_Digit(const bn_int *a, bn_digit digit);

/**
 * @brief Subtracts two big integers.
 * 
 * @details This function computes c = a - b assuming a >= b.
 *          Borrow handling is performed using extended precision.
 * 
 * @param a Pointer to minuend.
 * @param b Pointer to subtrahend.
 * @param c Pointer to result.
 * 
 * @return Returns SUCCESS on success; otherwise returns error code
 *         if any pointer is NULL.
 */
uint16_t BigNum_Subract(const bn_int *a, const bn_int *b, bn_int *c);

/**
 * @brief Returns the byte size of a big integer.
 *
 * @details Calculates the byte size of the big integer in unsigned
 *          binary format. Returns 1 if the big integer is zero.
 *
 * @param a    Pointer to the source big integer.
 * @param size Pointer to a size_t variable that will hold the computed
 *             byte size on success.
 *
 * @return Returns SUCCESS on success; otherwise returns error code
 *         if any pointer is NULL.
 */
uint16_t BigNum_Unsigned_Bin_Size(const bn_int *a, size_t *size);

/**
 * @brief Reads an unsigned binary buffer into a big integer.
 * 
 * @details This function converts a big-endian byte array into
 *          an internal bn_int representation.
 * 
 * @param a Pointer to big integer to be populated.
 * @param b Pointer to input byte buffer.
 * @param len Length of input buffer.
 * 
 * @return Returns SUCCESS on success; otherwise returns error code
 *         if any pointer is NULL.
 */
uint16_t BigNum_Read_Unsigned_Bin(bn_int *a, const uint8_t *b, uint16_t len);

/**
 * @brief Writes a big integer to an unsigned binary buffer.
 * 
 * @details This function converts the big integer into a big-endian
 *          byte array representation.
 * 
 * @param a Pointer to big integer.
 * @param b Pointer to output buffer.
 * @param len Size of output buffer.
 * 
 * @return Returns SUCCESS on success; otherwise returns error code
 *         if pointer is NULL or buffer is too small.
 */
uint16_t BigNum_Write_Unsigned_Bin(const bn_int *a, uint8_t *b, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif  // BSP_INCLUDE_BIGNUM_H_