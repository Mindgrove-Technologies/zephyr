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
 * @file rsa_padding.h
 * @brief Contains APIs for RSA padding schemes used in cryptographic operations.
 * @details This header file provides function prototypes  required for
 *          implementing RSA padding schemes, including PKCS#1 v1.5, OAEP for
 *          encryption, and PSS for signing.
 * @version 1.0
 * @authors Dayana Devi K (dayana@mindgrovetech.in)
 * @date 02-02-2026
 *
 * @section History
 * -----------------------------------------------------------------------------
 * Date       | Version | Modified by           | Description
 * -----------|---------|-----------------------|-------------------------------
 * 02-02-2026 | 1.0     | Dayana Devi K         | Initial release.
 * -----------------------------------------------------------------------------
 */

#ifndef BSP_INCLUDE_RSA_PADDING_H_
#define BSP_INCLUDE_RSA_PADDING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

// #include "sha256.h"
// #include "crypto_defines.h"
// #include "errors.h"

/**
 * @brief Encrypts a message using RSAES PKCS#1 v1.5 padding.
 *
 * @details This function applies PKCS#1 v1.5 padding to the input message
 *          and produces an RSA-encrypted block. The padding format is:
 *          0x00 || 0x02 || PS || 0x00 || message, where PS is a randomly
 *          generated non-zero padding string. The function ensures the
 *          input message fits within the RSA block size.
 *
 * @param output Pointer to the buffer to store the padded output (must be RSA_BLOCK_SIZE).
 * @param output_length Length of the output buffer in bytes.
 * @param input Pointer to the input message.
 * @param input_length Length of the input message in bytes.
 *
 * @return `SUCCESS` if encryption succeeds,
 *         `EFAULT` if any pointer is NULL,
 *         `EINVAL` if input length exceeds allowable size or output length is invalid.
 */
uint16_t RSAES_PKCS1_v1_5_Encrypt(uint8_t *output, size_t output_length, uint8_t *input,
				  size_t input_length);

/**
 * @brief Decrypts a message using RSAES PKCS#1 v1.5 padding.
 *
 * @details This function removes PKCS#1 v1.5 padding from an RSA-encrypted
 *          block and retrieves the original message. It verifies that the
 *          padding format (0x00 || 0x02 || PS || 0x00) is correct, and
 *          that the padding string PS is at least 8 bytes long.
 *
 * @param output Pointer to the buffer to store the decrypted message.
 * @param output_length Pointer to variable holding the length of the decrypted message.
 * @param input Pointer to the padded/encrypted input.
 * @param input_length Length of the input buffer in bytes (must be RSA_BLOCK_SIZE).
 *
 * @return `SUCCESS` if decryption succeeds,
 *         `EFAULT` if any pointer is NULL,
 *         `EINVAL` if input length or padding format is invalid.
 */
uint16_t RSAES_PKCS1_v1_5_Decrypt(uint8_t *output, size_t *output_length, uint8_t *input,
				  size_t input_length);

/**
 * @brief Generates a PKCS#1 v1.5 signature for a message.
 *
 * @details This function hashes the input message using SHA-256, constructs
 *          the DigestInfo structure, and applies PKCS#1 v1.5 padding for
 *          signing. The resulting signature block is ready for RSA signing.
 *
 * @param output Pointer to the buffer to store the signature (must be RSA_BLOCK_SIZE).
 * @param output_length Length of the output buffer in bytes.
 * @param input Pointer to the input message to sign.
 * @param input_length Length of the input message in bytes.
 *
 * @return `SUCCESS` if signing succeeds,
 *         `EFAULT` if any pointer is NULL,
 *         `EINVAL` if output length is invalid or hashing fails.
 */
uint16_t RSASSA_PKCS1_v1_5_Sign(uint8_t *output, size_t output_length, uint8_t *input,
				size_t input_length);

/**
 * @brief Verifies a PKCS#1 v1.5 signature against a message.
 *
 * @details This function checks the signature format, extracts the DigestInfo,
 *          hashes the input message, and compares the computed hash with
 *          the one in the signature. Any mismatch or invalid formatting
 *          results in verification failure.
 *
 * @param signature Pointer to the signature to verify.
 * @param signature_length Length of the signature (must be RSA_BLOCK_SIZE).
 * @param input Pointer to the original message.
 * @param input_length Length of the original message in bytes.
 *
 * @return `SUCCESS` if verification succeeds,
 *         `EFAULT` if any pointer is NULL,
 *         `EINVAL` if signature format or hash verification fails.
 */
uint16_t RSASSA_PKCS1_v1_5_Verify(uint8_t *signature, size_t signature_length, uint8_t *input,
				  size_t input_length);

/**
 * @brief Encrypts a message using RSAES-OAEP padding.
 *
 * @details This function implements OAEP padding using SHA-256. It computes
 *          a label hash, generates a data block (DB) and seed, applies mask
 *          generation functions (MGF1), and outputs the padded message:
 *          0x00 || maskedSeed || maskedDB.
 *
 * @param output Pointer to the output buffer (must be RSA_BLOCK_SIZE).
 * @param output_length Length of the output buffer in bytes.
 * @param input Pointer to the message to encrypt.
 * @param input_length Length of the input message in bytes.
 * @param label Optional label associated with the message.
 * @param label_length Length of the label in bytes.
 *
 * @return `SUCCESS` if encryption succeeds,
 *         `EFAULT` if any pointer is NULL,
 *         `EINVAL` if input length exceeds maximum allowed or output length is invalid.
 */
uint16_t RSAES_OAEP_Encrypt(uint8_t *output, size_t output_length, uint8_t *input,
			    size_t input_length, uint8_t *label, size_t label_length);

/**
 * @brief Decrypts a message using RSAES-OAEP padding.
 *
 * @details This function reverses OAEP padding applied to an RSA-encrypted block.
 *          It recovers the original message by computing masks using MGF1,
 *          verifying the label hash, and checking the padding format.
 *
 * @param input Pointer to the RSA-encrypted input (must be RSA_BLOCK_SIZE).
 * @param input_length Length of the input buffer in bytes.
 * @param output Pointer to the buffer to store the decrypted message.
 * @param output_length Pointer to variable holding the length of the decrypted message.
 * @param label Optional label associated with the message.
 * @param label_length Length of the label in bytes.
 *
 * @return `SUCCESS` if decryption succeeds,
 *         `EFAULT` if any pointer is NULL,
 *         `EINVAL` if padding format or label hash is invalid.
 */
uint16_t RSAES_OAEP_Decrypt(uint8_t *input, size_t input_length, uint8_t *output,
			    size_t *output_length, uint8_t *label, size_t label_length);

/**
 * @brief Generates an RSASSA-PSS signature for a message.
 *
 * @details This function implements PSS padding using SHA-256. It hashes the
 *          input message, optionally uses a provided salt or generates a random
 *          salt, constructs the padded data block, applies MGF1, and outputs
 *          the signature block with the trailer byte 0xBC.
 *
 * @param output Pointer to the buffer to store the signature (must be RSA_BLOCK_SIZE).
 * @param output_length Length of the output buffer in bytes.
 * @param input Pointer to the message to sign.
 * @param input_length Length of the input message in bytes.
 * @param salt Optional pointer to salt bytes.
 * @param salt_length Length of the salt in bytes.
 * @param use_provided_salt Flag indicating whether to use the provided salt (1)
 *                          or generate randomly (0).
 *
 * @return `SUCCESS` if signing succeeds,
 *         `EFAULT` if any pointer is NULL or salt is invalid,
 *         `EINVAL` if output length or salt length is invalid.
 */
uint16_t RSASSA_PSS_Sign(uint8_t *output, size_t output_length, uint8_t *input, size_t input_length,
			 uint8_t *salt, size_t salt_length, uint8_t use_provided_salt);

/**
 * @brief Verifies an RSASSA-PSS signature against a message.
 *
 * @details This function reverses PSS padding, extracts the salt, hashes the
 *          message with the extracted salt, and compares the resulting hash
 *          to the one in the signature. The trailer byte 0xBC is also verified.
 *
 * @param signature Pointer to the signature to verify (must be RSA_BLOCK_SIZE).
 * @param signature_length Length of the signature in bytes.
 * @param input Pointer to the original message.
 * @param input_length Length of the original message in bytes.
 * @param expected_salt_length Length of the salt expected in the signature.
 *
 * @return `SUCCESS` if verification succeeds,
 *         `EFAULT` if any pointer is NULL,
 *         `EINVAL` if signature format, hash, or salt length is invalid.
 */
uint16_t RSASSA_PSS_Verify(uint8_t *signature, size_t signature_length, uint8_t *input,
			   size_t input_length, size_t expected_salt_length);

#ifdef __cplusplus
}
#endif

#endif // BSP_INCLUDE_RSA_PADDING_H_
