#ifndef _INCLUDE_UTIL_H
#define _INCLUDE_UTIL_H 1

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void msleep(uint32_t delay);

void ledOn(void);
void ledOff(void);
void toggleLed(void);

void debugLed(uint8_t n);

extern volatile uint32_t system_millis;

#ifdef __cplusplus
}
#endif

#endif // !_INCLUDE_UTIL_H
