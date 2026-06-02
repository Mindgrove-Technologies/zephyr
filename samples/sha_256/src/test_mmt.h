#ifndef TEST_MMT_H
#define TEST_MMT_H

/* One function per MMT file. Only declared here;
 * the linker will only see the ones that were compiled in. */

int test_sha256_mmt_01(void);
int test_sha256_mmt_02(void);
int test_sha256_mmt_03(void);
int test_sha256_mmt_04(void);
int test_sha256_mmt_05(void);
int test_sha256_mmt_06(void);

#endif /* TEST_MMT_H */