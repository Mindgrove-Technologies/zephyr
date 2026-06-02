# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

# Source to mbedtls which the RSA driver uses for R^2 mod N
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/crypto/mbedtls
)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rsa_2048_tests)

target_sources(app PRIVATE src/main.c)

if(CONFIG_RSA_TEST_OAEP_DECRYPT_01)
    target_sources(app PRIVATE src/test_rsa_oaep_decrypt_01.c)
endif()
if(CONFIG_RSA_TEST_OAEP_DECRYPT_02)
    target_sources(app PRIVATE src/test_rsa_oaep_decrypt_02.c)
endif()
if(CONFIG_RSA_TEST_OAEP_ENCRYPT)
    target_sources(app PRIVATE src/test_rsa_oaep_encrypt.c)
endif()
if(CONFIG_RSA_TEST_PKCS_ENC_DEC)
    target_sources(app PRIVATE src/test_rsa_pkcs_enc_dec.c)
endif()
if(CONFIG_RSA_TEST_PKCS_SIGN_GEN)
    target_sources(app PRIVATE src/test_rsa_pkcs_sign_gen.c)
endif()
if(CONFIG_RSA_TEST_PKCS_SIGN_VERIF)
    target_sources(app PRIVATE src/test_rsa_pkcs_sign_verif.c)
endif()
if(CONFIG_RSA_TEST_PSS_SIGN_GEN)
    target_sources(app PRIVATE src/test_rsa_pss_sign_gen.c)
endif()
if(CONFIG_RSA_TEST_PSS_SIGN_VERIF)
    target_sources(app PRIVATE src/test_rsa_pss_sign_verif.c)
endif()