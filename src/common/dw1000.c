#include "dw1000.h"

#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "deca_device_api.h"

#include "util.h"

decaIrqStatus_t decamutexon(void)
{
    return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
    (void)s;
}

void deca_sleep(unsigned int time_ms)
{
    msleep(time_ms);
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    // taskENTER_CRITICAL();

    gpio_clear(GPIOA, CS);

    for (uint16 i = 0; i < headerLength; i++)
        (void)spi_xfer(SPI1, headerBuffer[i]);

    for (uint32 i = 0; i < readlength; i++)
        readBuffer[i] = spi_xfer(SPI1, 0x00);

    gpio_set(GPIOA, CS);

    // taskEXIT_CRITICAL();

    return 0;
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
    // taskENTER_CRITICAL();

    gpio_clear(GPIOA, CS);

    for (uint16 i = 0; i < headerLength; i++)
        (void)spi_xfer(SPI1, headerBuffer[i]);

    for (uint32 i = 0; i < bodylength; i++)
        (void)spi_xfer(SPI1, bodyBuffer[i]);

    gpio_set(GPIOA, CS);

    // taskEXIT_CRITICAL();

    return 0;
}
