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
 * @file rsa_padding_driver.c
 * @brief Contains RSA padding driver implementations.
 * @details This file provides implementations of RSA padding schemes 
 *          and helper functions for encryption, decryption, signing, 
 *          and signature verification. It provides PKCS#1 v1.5 padding 
 *          for encryption and signing, OAEP padding for encryption, 
 *          and PSS padding for probabilistic signatures.
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

#include "rsa_padding.h"
#include "crypto_mindgrove_rsa.h"
#include "crypto_mindgrove_sha.h"
#include "string.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <errno.h>
// #include <zephyr/random/random.h>

#define RSA_BLOCK_SIZE 256U

//static void printk_hexdump(const char *label, const uint8_t *data, size_t len) {
//    //printk("--- %s (%zu bytes) ---\n", label, len);
//    for (size_t i = 0; i < len; i++) {
//        //printk("%02x ", data[i]);
//        if ((i + 1) % 16 == 0) //printk("\n");
//    }
//    //printk("\n------------------\n");
//}

/* SHA256 digest info */
static uint8_t sha256_digestinfo[] = {
    0x30, 0x31, 0x30, 0x0d, 0x06, 0x09,
    0x60, 0x86, 0x48, 0x01, 0x65, 0x03,
    0x04, 0x02, 0x01, 0x05, 0x00,
    0x04, 0x20
};

/* Helper function to generate mask of data block, and seed */
static uint16_t Mask_Generation(uint8_t *mask, size_t maskLen,
                                const uint8_t *seed, size_t seedLen) {
    uint8_t result = 0;
    size_t generated_bytes = 0;
    size_t num_of_bytes = 0;
    uint32_t counter = 0;
    uint8_t counter_data[4];
    uint8_t hash_output[SHA256_HASH_LEN];
    uint8_t buffer[seedLen + 4U];

    if ((mask == NULL) || (seed == NULL)) {
        return EFAULT;
    }

    while (generated_bytes < maskLen) {
        /* I2OSP(counter, 4) */
        counter_data[0] = (counter >> 24) & 0xFFU;
        counter_data[1] = (counter >> 16) & 0xFFU;
        counter_data[2] = (counter >> 8)  & 0xFFU;
        counter_data[3] = counter & 0xFFU;

        /* Hashing (seed || Counter_data) */
        (void) memcpy(buffer, seed, seedLen);
        (void) memcpy(buffer + seedLen, counter_data, 4);

        result = SHA256_Single_Run(hash_output, buffer, (seedLen + 4U) * 8U);
        if (result != SUCCESS) {
            return result;
        }

        num_of_bytes = ((maskLen - generated_bytes) < SHA256_HASH_LEN) ?
                       ((maskLen - generated_bytes)) : (SHA256_HASH_LEN);

        (void) memcpy(mask + generated_bytes, hash_output, num_of_bytes);

        generated_bytes += num_of_bytes;
        counter++;
    }

    return SUCCESS;
}

uint16_t RSAES_PKCS1_v1_5_Encrypt(uint8_t *output, size_t output_length,
                                  uint8_t *input, size_t input_length) {
    uint16_t index = 0;
    size_t padding_string_len = 0;
    uint8_t random_num = 0;

    if ((output == NULL) || (input == NULL)) {
        return EFAULT;
    }

    if (input_length > (RSA_BLOCK_SIZE - 11U)) {
        //log_emit(ERROR, "Input too long to pad. \n\r");
        return EINVAL;
    }

    if (output_length != RSA_BLOCK_SIZE) {
        //log_emit(ERROR, "Invalid output buffer size. \n\r");
        return EINVAL;
    }

    padding_string_len = (RSA_BLOCK_SIZE - input_length) - 3U;

    /* Header and encryption byte */
    output[index++] = 0x00;
    output[index++] = 0x02;

    /* Padding string and separator */
    for (size_t i = 0; i < padding_string_len; i++) {
        random_num = 0;
        do {
            random_num = (uint8_t)rand() & 0xFF;
        } while (random_num == 0U);
        output[index++] = random_num;
    }

    output[index++] = 0x00;

    /* Append input */
    (void) memcpy(output + index, input, input_length);
   
    //printk("PKCS1 v1.5 Padding Success\n");
    //printk_hexdump("Padded Block (EM)", output, RSA_BLOCK_SIZE);

    return SUCCESS;
}

uint16_t RSAES_PKCS1_v1_5_Decrypt(uint8_t *output, size_t *output_length,
                                  uint8_t *input, size_t input_length) {
    uint16_t index = 0;
    uint16_t padding_string_start = 0;
    size_t padding_string_len = 0;

    if ((output == NULL) || (output_length == NULL) || (input == NULL)) {
        return EFAULT;
    }

    *output_length = 0;

    if (input_length != RSA_BLOCK_SIZE) {
        //log_emit(ERROR, "Invalid input length. \n\r");
        return EINVAL;
    }

    /* Check header bytes */
    if (input[index++] != 0x00U) {
        //log_emit(DEBUG, "Invalid PKCS#1 v1.5 header bytes. \n\r");
        return EINVAL;
    }

    if (input[index++] != 0x02U) {
        //log_emit(DEBUG, "Invalid PKCS#1 v1.5 header bytes. \n\r");
        return EINVAL;
    }

    padding_string_start = index;

    /* Check PS and separator */
    while ((index < input_length) && (input[index] != 0x00U)) {
        index++;
    }

    if (index >= input_length) {
        //log_emit(DEBUG, "Separator byte not found.\n\r");
        return EINVAL;
    }

    padding_string_len = (size_t) index - (size_t) padding_string_start;
    if (padding_string_len < 8U) {
        //log_emit(DEBUG, "Padding string too short (< 8 bytes).\n\r");
        return EINVAL;
    }

    index++;

    *output_length = input_length - index;
    (void) memcpy(output, input + index, *output_length);

    return SUCCESS;
}

uint16_t RSASSA_PKCS1_v1_5_Sign(uint8_t *output, size_t output_length,
                                uint8_t *input, size_t input_length) {
    uint16_t index = 0;
    uint16_t result = 0;
    size_t padding_string_len = 0;
    size_t digest_info_len = sizeof(sha256_digestinfo) + SHA256_HASH_LEN;
    uint8_t hash_output[SHA256_HASH_LEN];

    if ((output == NULL) || (input == NULL)) {
        return EFAULT;
    }

    if (output_length != RSA_BLOCK_SIZE) {
        //log_emit(ERROR, "Invalid output buffer size. \n\r");
        return EINVAL;
    }

    /* Hash the input message */
    result = SHA256_Single_Run(hash_output, input, input_length * 8U);
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while hashing the message! \n\r");
        return result;
    }

    padding_string_len = (RSA_BLOCK_SIZE - digest_info_len) - 3U;

    /* Header and Signature byte */
    output[index++] = 0x00;
    output[index++] = 0x01;

    /* Padding string and separator */
    (void) memset(output + index, 0xFF, padding_string_len);
    index += (uint16_t) padding_string_len;

    output[index++] = 0x00;

    /* Append sha256 digest info and hash of the input message */
    (void) memcpy(output + index, sha256_digestinfo, sizeof(sha256_digestinfo));
    index += sizeof(sha256_digestinfo);

    (void) memcpy(output + index, hash_output, SHA256_HASH_LEN);

    return SUCCESS;
}

uint16_t RSASSA_PKCS1_v1_5_Verify(uint8_t *signature,
                                  size_t signature_length,
                                  uint8_t *input, size_t input_length) {
    uint16_t index = 0;
    uint16_t result = 0;
    uint16_t padding_string_start = 0;
    size_t padding_string_len = 0;
    uint8_t computed_hash[SHA256_HASH_LEN];

    if ((signature == NULL) || (input == NULL)) {
        return EFAULT;
    }

    if (signature_length != RSA_BLOCK_SIZE) {
        //log_emit(ERROR, "Invalid length. \n\r");
        return EINVAL;
    }
    //printk("Inside rsa padding : Recovered EM: ");
    for(int i=0; i<32; i++) //printk("%02x ", signature[i]);
    //printk("\n");
    /* Hash the input message */
    result = SHA256_Single_Run(computed_hash, input, input_length * 8U);
    //printk("After Single Run, Computed Hash: ");
    for(int i=0; i<32; i++) //printk("%02x ", computed_hash[i]);
    //printk("\n");
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while hashing the message! \n\r");
        return result;
    }

    /* Check header bytes */
    if (signature[index++] != 0x00U) {
        //log_emit(DEBUG, "Invalid PKCS#1 v1.5 header bytes. \n\r");
        return EINVAL;
    }

    if (signature[index++] != 0x01U) {
        //log_emit(DEBUG, "Invalid PKCS#1 v1.5 header bytes. \n\r");
        return EINVAL;
    }

    padding_string_start = index;

    /* Check PS and separator */
    while ((index < signature_length) && (signature[index] == 0xFFU)) {
        index++;
    }

    if (index >= signature_length) {
        //log_emit(DEBUG, "Separator byte not found.\n\r");
        return EINVAL;
    }

    padding_string_len = (size_t) index - (size_t) padding_string_start;
    if (padding_string_len < 8U) {
        //log_emit(DEBUG, "Padding string too short (< 8 bytes).\n\r");
        return EINVAL;
    }

    if (signature[index++] != 0x00U) {
        //log_emit(DEBUG, "Invalid separator byte. \n\r");
        return EINVAL;
    }

    /* Check DigestInfo */
    if (memcmp(signature + index, sha256_digestinfo,
               sizeof(sha256_digestinfo)) != 0) {
        //log_emit(DEBUG, "Invalid SHA256 digest info. \n\r");
        return EINVAL;
    }

    index += sizeof(sha256_digestinfo);

    /* Hash comparison */
    result = 0;
    for (uint8_t i = 0; i < SHA256_HASH_LEN; i++) {
        result |= signature[index + i] ^ computed_hash[i];
    }

    if (result != 0U) {
        //log_emit(DEBUG, "Hash mismatch. \n\r");
        return EINVAL;
    }

    index += SHA256_HASH_LEN;

    if (index != signature_length) {
        //log_emit(DEBUG, "Extra bytes after signature. \n\r");
        return EINVAL;
    }

    return SUCCESS;
}

uint16_t RSAES_OAEP_Encrypt(uint8_t *output, size_t output_length,
                            uint8_t *input, size_t input_length,
                            uint8_t *label, size_t label_length) {
    uint8_t empty_input = 0;
    uint16_t index = 0;
    uint16_t result = 0;
    size_t padding_string_len;
    size_t data_block_length = RSA_BLOCK_SIZE - SHA256_HASH_LEN - 1U;
    uint8_t label_hash[SHA256_HASH_LEN];
    uint8_t seed[SHA256_HASH_LEN];
    uint8_t data_block[256];
    uint8_t dbMask[256];
    uint8_t maskedDB[256];
    uint8_t seedMask[SHA256_HASH_LEN];
    uint8_t maskedSeed[SHA256_HASH_LEN];

    if ((output == NULL) || (input == NULL)) {
        return EFAULT;
    }

    if (input_length > (RSA_BLOCK_SIZE - (2U * SHA256_HASH_LEN) - 2U)) {
        //log_emit(ERROR, "Input too long to pad. \n\r");
        return EINVAL;
    }

    if (output_length != RSA_BLOCK_SIZE) {
        //log_emit(ERROR, "Invalid output buffer size. \n\r");
        return EINVAL;
    }

    /* Hash the label */
    if ((label != NULL) && (label_length > 0U)) {
        result = SHA256_Single_Run(label_hash, label, label_length * 8U);
    } else {
        result = SHA256_Single_Run(label_hash, &empty_input, 0);
    }

    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while hashing label! \n\r");
        return result;
    }

    /* Generating Data Block = label hash || PS || 0x01 || message */
    (void) memcpy(data_block + index, label_hash, SHA256_HASH_LEN);
    index += SHA256_HASH_LEN;

    padding_string_len = data_block_length - input_length -
                         SHA256_HASH_LEN - 1U;
    (void) memset(data_block + index, 0x00, padding_string_len);
    index += (uint16_t) padding_string_len;

    data_block[index++] = 0x01;

    (void) memcpy(data_block + index, input, input_length);

    /* Generating seed */
    for (uint8_t i = 0; i < SHA256_HASH_LEN; i++) {
        seed[i] = (uint8_t)rand()& 0xFFU;
    }

    /* Generating maskedDB = DB XOR dbMask
       dbMask = MGF(seed, data_block_length) */
    result = Mask_Generation(dbMask, data_block_length, seed, SHA256_HASH_LEN);
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while generating Data block mask!\n\r");
        return result;
    }

    for (size_t i = 0; i < data_block_length; i++) {
        maskedDB[i] = data_block[i] ^ dbMask[i];
    }

    /* Generating maskedSeed = seed XOR seedMask
       seedMask = MGF(maskedDB, hashlen) */
    result = Mask_Generation(seedMask, SHA256_HASH_LEN, maskedDB,
                         data_block_length);
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while generating seed mask!\n\r");
        return result;
    }

    for (uint8_t i = 0; i < SHA256_HASH_LEN; i++) {
        maskedSeed[i] = seed[i] ^ seedMask[i];
    }

    /* Encrypted_Data = 0x00 || maskedSeed || maskedDB */
    index = 0;
    output[index++] = 0x00;
    (void) memcpy(output + index, maskedSeed, SHA256_HASH_LEN);
    index += SHA256_HASH_LEN;
    (void) memcpy(output + index, maskedDB, data_block_length);

    //printk("OAEP Padding Success\n");
    //printk_hexdump("Original Seed", seed, SHA256_HASH_LEN);
    //printk_hexdump("Final OAEP Block", output, RSA_BLOCK_SIZE);

    return SUCCESS;
}

uint16_t RSAES_OAEP_Decrypt(uint8_t *input, size_t input_length,
                            uint8_t *output, size_t *output_length,
                            uint8_t *label, size_t label_length) {
    uint8_t empty_input = 0;
    uint16_t index = 0;
    uint16_t result = 0;
    size_t data_block_length = RSA_BLOCK_SIZE - SHA256_HASH_LEN - 1U;
    uint8_t label_hash[SHA256_HASH_LEN];
    uint8_t seed[SHA256_HASH_LEN];
    uint8_t data_block[256];
    uint8_t dbMask[256];
    uint8_t seedMask[SHA256_HASH_LEN];
    uint8_t *maskedSeed;
    uint8_t *maskedDB;

    if ((input == NULL) || (output == NULL) || (output_length == NULL)) {
        return EFAULT;
    }

    *output_length = 0;

    if (input_length != RSA_BLOCK_SIZE) {
        //log_emit(ERROR, "Invalid input length. \n\r");
        return EINVAL;
    }

    /* Hash the label */
    if ((label != NULL) && (label_length > 0U)) {
        result = SHA256_Single_Run(label_hash, label, label_length * 8U);
    } else {
        result = SHA256_Single_Run(label_hash, &empty_input, 0U);
    }

    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while hashing label!\n\r");
        return result;
    }

    /* Check header and parsing maskedSeed, maskedDB */
    if (input[index++] != 0x00U) {
        //log_emit(DEBUG, "Invalid OAEP header byte. \n\r");
        return EINVAL;
    }

    maskedSeed = input + index;
    index += SHA256_HASH_LEN;
    maskedDB = input + index;

    /* Generating seedMask = MGF(maskedDB, hashLen) */
    result = Mask_Generation(seedMask, SHA256_HASH_LEN, maskedDB,
                         data_block_length);
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while generating seed mask! \n\r");
        return result;
    }

    /* seed = maskedSeed XOR seedMask */
    for (uint8_t i = 0; i < SHA256_HASH_LEN; i++) {
        seed[i] = maskedSeed[i] ^ seedMask[i];
    }

    /* dbMask = MGF(seed, dbLen) */
    result = Mask_Generation(dbMask, data_block_length, seed, SHA256_HASH_LEN);
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while generating Data block mask!\n\r");
        return result;
    }

    /* Generating Data Block = maskedDB XOR dbMask */
    for (size_t i = 0; i < data_block_length; i++) {
        data_block[i] = maskedDB[i] ^ dbMask[i];
    }

    /* Check label hash */
    if (memcmp(data_block, label_hash, SHA256_HASH_LEN) != 0) {
        //log_emit(DEBUG, "Label hash mismatch. \n\r");
        return EINVAL;
    }

    /* Check separator and PS */
    index = SHA256_HASH_LEN;
    while ((index < data_block_length) && (data_block[index] == 0x00U)) {
        index++;
    }

    if (index >= data_block_length) {
        //log_emit(DEBUG, "Separator byte not found. \n\r");
        return EINVAL;
    }

    if (data_block[index++] != 0x01U) {
        //log_emit(DEBUG, "Invalid Separator byte. \n\r");
        return EINVAL;
    }

    *output_length = data_block_length - index;
    (void) memcpy(output, data_block + index, *output_length);

    return SUCCESS;
}

uint16_t RSASSA_PSS_Sign(uint8_t *output, size_t output_length,
                         uint8_t *input, size_t input_length,
                         uint8_t *salt, size_t salt_length,
                         uint8_t use_provided_salt) {
    uint16_t result = 0;
    size_t padding_string_len = 0;
    size_t data_block_length = RSA_BLOCK_SIZE - SHA256_HASH_LEN - 1U;
    uint8_t message_hash[SHA256_HASH_LEN];
    uint8_t local_salt[256] = {0};
    uint8_t M_prime_hash[SHA256_HASH_LEN];
    uint8_t M_prime[8U + SHA256_HASH_LEN + 256U];
    uint8_t data_block[256];
    uint8_t dbMask[256];
    uint8_t *salt_to_use = NULL;

    if ((output == NULL) || (input == NULL)) {
        return EFAULT;
    }

    if (output_length != RSA_BLOCK_SIZE) {
        //log_emit(ERROR, "Invalid output buffer size. \n\r");
        return EINVAL;
    }

    if (salt_length > (data_block_length - 1U)) {
        //log_emit(ERROR, "Salt length too large. \n\r");
        return EINVAL;
    }

    /* Hash the input message */
    result = SHA256_Single_Run(message_hash, input, input_length * 8U);
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while hashing the message! \n\r");
        return result;
    }

    if (use_provided_salt == 1U) {
        if ((salt == NULL) && (salt_length > 0U)) {
            return EFAULT;
        }
        salt_to_use = salt;
    } else {
        /* Generate random salt */
        for (size_t i = 0; i < salt_length; i++) {
            local_salt[i] = (uint8_t)rand() & 0xFFU;
        }
        salt_to_use = local_salt;
    }

    /* Generating M_prime = 00 00 00 00 00 00 00 00 || message hash || salt */
    (void) memset(M_prime, 0x00, 8);
    (void) memcpy(M_prime + 8, message_hash, SHA256_HASH_LEN);
    if (salt_length > 0U) {
        (void) memcpy(M_prime + 8U + SHA256_HASH_LEN, salt_to_use, salt_length);
    }

    /* Hashing M_prime */
    result = SHA256_Single_Run(M_prime_hash, M_prime,
                               (8U + SHA256_HASH_LEN + salt_length) * 8U);
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while hashing M prime. \n\r");
        return result;
    }

    /* Generating Data Block = PS || 0x01 || salt */
    padding_string_len = data_block_length - salt_length - 1U;
    (void) memset(data_block, 0x00, padding_string_len);
    data_block[padding_string_len] = 0x01U;
    if (salt_length > 0U) {
       (void) memcpy(data_block + padding_string_len + 1U, salt_to_use,
                    salt_length);
    }

    /* Generating dbMask = MGF(M_prime_hash, data_block_length) 
       *maskedDB = data_block XOR dbMask */
    result = Mask_Generation(dbMask, data_block_length, M_prime_hash,
                         SHA256_HASH_LEN);
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error in MGF1!\n\r");
        return result;
    }

    for (size_t i = 0; i < data_block_length; i++) {
        output[i] = data_block[i] ^ dbMask[i];
    }

    /* Set leftmost bit to zero */
    output[0] &= 0x7FU;

    (void) memcpy(output + data_block_length, M_prime_hash, SHA256_HASH_LEN);
    output[RSA_BLOCK_SIZE - 1U] = 0xBCU;

    //printk("PSS Encoding Success\n");
    //printk_hexdump("PSS Salt", salt_to_use, salt_length);
    //printk_hexdump("PSS Encoded Message", output, RSA_BLOCK_SIZE);

    return SUCCESS;
}

uint16_t RSASSA_PSS_Verify(uint8_t *signature, size_t signature_length,
                           uint8_t *input, size_t input_length,
                           size_t expected_salt_length) {
    uint16_t result = 0;
    size_t data_block_length = RSA_BLOCK_SIZE - SHA256_HASH_LEN - 1U;
    size_t padding_string_len = 0;
    uint8_t message_hash[SHA256_HASH_LEN];
    uint8_t M_prime_hash[SHA256_HASH_LEN];
    uint8_t data_block[256];
    uint8_t dbMask[256];
    uint8_t M_prime[8U + SHA256_HASH_LEN + 256U];
    uint8_t *signature_hash;

    if ((signature == NULL) || (input == NULL)) {
        return EFAULT;
    }

    if (signature_length != RSA_BLOCK_SIZE) {
        //log_emit(ERROR, "Invalid signature length. \n\r");
        return EINVAL;
    }

    if (expected_salt_length > (data_block_length - 1U)) {
        //log_emit(ERROR, "Salt length too large. \n\r");
        return EINVAL;
    }

    if (signature[RSA_BLOCK_SIZE - 1U] != 0xBCU) {
        //log_emit(DEBUG, "Invalid trailer byte. \n\r");
        return EINVAL;
    }

    /* Hash the input message */
    result = SHA256_Single_Run(message_hash, input, input_length * 8U);
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while hashing the message! \n\r");
        return result;
    }

    /* Parsing signature hash and generate dbMask */
    signature_hash = signature + data_block_length;

    result = Mask_Generation(dbMask, data_block_length, signature_hash,
                         SHA256_HASH_LEN);
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while generating Data block mask! \n\r");
        return result;
    }

    /* Generating Data Block = maskedDB XOR dbMask */
    for (size_t i = 0; i < data_block_length; i++) {
        data_block[i] = signature[i] ^ dbMask[i];
    }

    /* Set leftmost bit to zero */
    data_block[0] &= 0x7FU;

    /* Check seperator and PS */
    padding_string_len = data_block_length - expected_salt_length - 1U;

    for (size_t i = 0; i < padding_string_len; i++) {
        if (data_block[i] != 0x00U) {
            //log_emit(DEBUG, "Invalid padding. \n\r");
            return EINVAL;
        }
    }

    if (data_block[padding_string_len] != 0x01U) {
        //log_emit(DEBUG, "Invalid separator. \n\r");
        return EINVAL;
    }

    /* Generating M_prime = 00 00 00 00 00 00 00 00 || message hash || salt */
    (void) memset(M_prime, 0x00, 8U);
    (void) memcpy(M_prime + 8U, message_hash, SHA256_HASH_LEN);
    if (expected_salt_length > 0U) {
        (void) memcpy(M_prime + 8U + SHA256_HASH_LEN,
               data_block + padding_string_len + 1U,
               expected_salt_length);
    }

    /* Hashing M_prime */
    result = SHA256_Single_Run(M_prime_hash, M_prime,
                            (8U + SHA256_HASH_LEN + expected_salt_length) * 8U);
    if (result != SUCCESS) {
        //log_emit(ERROR, "Error while hashing M prime. \n\r");
        return result;
    }

    /* Hash comparison */
    result = 0U;
    for (uint8_t i = 0; i < SHA256_HASH_LEN; i++) {
        result |= M_prime_hash[i] ^ signature_hash[i];
    }

    if (result != 0U) {
        //log_emit(DEBUG, "Hash mismatch. \n\r");
        return EINVAL;
    }

    return SUCCESS;
}