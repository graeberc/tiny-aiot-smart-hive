/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
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

#include "../ei_classifier_porting.h"
#if defined(ARDUINO)
#include <Arduino.h>

#ifdef ESP32
#include <esp_heap_caps.h>
#endif

#define EI_WEAK_FN __attribute__((weak))

EI_WEAK_FN EI_IMPULSE_ERROR ei_run_impulse_check_canceled() {
    return EI_IMPULSE_OK;
}

EI_WEAK_FN EI_IMPULSE_ERROR ei_sleep(int32_t time_ms) {
    delay(time_ms);
    return EI_IMPULSE_OK;
}

uint64_t ei_read_timer_ms() {
    return millis();
}

uint64_t ei_read_timer_us() {
    return micros();
}

EI_WEAK_FN void ei_serial_set_baudrate(int baudrate) {
    Serial.begin(baudrate);
}

EI_WEAK_FN void ei_printf(const char *format, ...) {
    char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}

EI_WEAK_FN void ei_printf_float(float f) {
    ei_printf("%f", f);
}

EI_WEAK_FN void *ei_malloc(size_t size) {
#ifdef ESP32
    return heap_caps_aligned_alloc(16, size, MALLOC_CAP_SPIRAM);
#else
    return malloc(size);
#endif
}

EI_WEAK_FN void *ei_calloc(size_t nitems, size_t size) {
#ifdef ESP32
    void *ptr = heap_caps_aligned_alloc(16, nitems * size, MALLOC_CAP_SPIRAM);
    if (ptr) memset(ptr, 0, nitems * size);
    return ptr;
#else
    return calloc(nitems, size);
#endif
}

EI_WEAK_FN void ei_free(void *ptr) {
    free(ptr);
}

#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C"
#endif
EI_WEAK_FN void DebugLog(const char* s) {
    ei_printf("%s", s);
}

#endif 