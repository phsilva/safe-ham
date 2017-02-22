#ifndef _INCLUDE_DW1000_H_
#define _INCLUDE_DW1000_H_ 1

#include <libopencm3/stm32/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOSI GPIO7
#define MISO GPIO6
#define CLK GPIO5
#define CS GPIO4

void setup_dw1000(void);

#ifdef __cplusplus
}
#endif

#endif // !_INCLUDE_DW1000_H_
