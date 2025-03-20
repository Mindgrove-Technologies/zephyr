/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
uint8_t data=23;
uint8_t data1=34;
const uint32_t data2=45;
void printnums(int i){
	printf("I: %d\n", i++);
	__asm volatile("ebreak");
}

int main(void)
{
	uint8_t local=90;
	__asm volatile("ebreak");
	printf("Hello world!\n");
	__asm volatile("ebreak");
	int i = 0;
	while(i < 10){
		printnums(i);
		i=i+1;
	}
	return 0;
}