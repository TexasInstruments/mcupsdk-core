#ifndef __TEST_DATA_H__
#define __TEST_DATA_H__

#include <stdint.h>

#define TEST_DATA_LENGTH (32U * 1024U / 4)

extern uint32_t byte_array_dec_1[TEST_DATA_LENGTH] __attribute__((section(".data.flashdata"))) ;

#endif 