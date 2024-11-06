#ifndef __TEST_DATA_H__
#define __TEST_DATA_H__

#include <stdint.h>

#define TEST_DATA_LENGTH (1024U * 1024U / 4)

extern uint32_t byte_array_dec_1[TEST_DATA_LENGTH] __attribute__((section(".data.flashdata_1"))) ;

extern uint32_t byte_array_dec_2[TEST_DATA_LENGTH] __attribute__((section(".data.flashdata_2"))) ;

extern uint32_t byte_array_dec_3[TEST_DATA_LENGTH] __attribute__((section(".data.flashdata_3"))) ;

#endif 