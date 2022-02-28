/* Edge Impulse inferencing library
 * Copyright (c) 2020 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <stdarg.h>
// #include "FreeRTOS.h"
// #include "task.h"
#include "printf.h"
#include "mt3620.h"
#include "os_hal_uart.h"
#include "string.h"
#include "os_hal_gpt.h"

static const uintptr_t GPT_BASE = 0x21030000;

static const UART_PORT uart_port_num = OS_HAL_UART_ISU0;

__attribute__((weak)) EI_IMPULSE_ERROR ei_run_impulse_check_canceled() {
    return EI_IMPULSE_OK;
}

void lp_write_reg32(uintptr_t baseAddr, size_t offset, uint32_t value)
{
	*(volatile uint32_t*)(baseAddr + offset) = value;
}

uint32_t lp_read_reg32(uintptr_t baseAddr, size_t offset)
{
	return *(volatile uint32_t*)(baseAddr + offset);
}

void lp_gpt3_wait_microseconds(uint32_t microseconds)
{
	// GPT3_INIT = initial counter value
	lp_write_reg32(GPT_BASE, 0x54, 0x0);

	// GPT3_CTRL
	uint32_t ctrlOn = 0x0;
	ctrlOn |= (0x19) << 16; // OSC_CNT_1US (default value)
	ctrlOn |= 0x1;          // GPT3_EN = 1 -> GPT3 enabled
	lp_write_reg32(GPT_BASE, 0x50, ctrlOn);

	// GPT3_CNT
	while (lp_read_reg32(GPT_BASE, 0x58) < microseconds)
	{
		// empty.
	}

	// GPT_CTRL -> disable timer
	lp_write_reg32(GPT_BASE, 0x50, 0x0);
}

/**
 * Cancelable sleep, can be triggered with signal from other thread
 */
__attribute__((weak)) EI_IMPULSE_ERROR ei_sleep(int32_t time_ms) {
    lp_gpt3_wait_microseconds(time_ms * 1000);
    return EI_IMPULSE_OK;
}

uint64_t ei_read_timer_ms() {
    return mtk_os_hal_gpt_get_cur_count((gpt_num)2);
}

uint64_t ei_read_timer_us() {
    return mtk_os_hal_gpt_get_cur_count((gpt_num)2) * 1000;
}

__attribute__((weak)) void ei_printf(const char *format, ...) {
    char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    for (int i = 0; i < r; i++) {
        mtk_os_hal_uart_put_char(uart_port_num, print_buf[i]);
    }
}

__attribute__((weak)) void ei_printf_float(float f) {
    printf("%f", f);
}

__attribute__((weak)) void *ei_malloc(size_t size) {
    return malloc(size);
}

__attribute__((weak)) void *ei_calloc(size_t nitems, size_t size) {
    void *ptr = malloc(nitems * size);
    memset(ptr, 0, nitems * size);
    return ptr;
}

__attribute__((weak)) void ei_free(void *ptr) {
    free(ptr);
}

#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C"
#endif
__attribute__((weak)) void DebugLog(const char* s) {
    ei_printf("%s", s);
}
