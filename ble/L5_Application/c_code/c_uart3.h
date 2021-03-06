/**
 * @file
 * This is a C header file that other C code can use to access UART2
 * driver that is written in C++
 *
 * Note that we have a C header file, but its implementation in C++
 * such that we can invoke C++ Uart driver
 *
 * The C Unit-test framework can use this header file to "Mock" the
 * UART2 API and carry out the tests.
 */
#ifndef UART3_C_H__
#define UART3_C_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

bool c_uart3_init(unsigned int baudRate, int rxQSize, int txQSize);

bool c_uart3_getChar(char *byte, uint32_t timeout_ms);

void c_setNumber(char num);

void getstring(char* pBuff, int maxLen, unsigned int timeout);

#ifdef __cplusplus
}
#endif
#endif /* UART3_C_H__ */
