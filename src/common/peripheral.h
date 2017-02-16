#ifndef _INCLUDE_PERIPHERAL_H
#define _INCLUDE_PERIPHERAL_H 1

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void peripheral_init(void);

void set_spi_slow(void);
void set_spi_fast(void);

#ifdef __cplusplus
}
#endif

#endif // !_INCLUDE_PERIPHERAL_H
