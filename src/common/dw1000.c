#include "dw1000.h"

#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#include "deca_device_api.h"
#include "deca_regs.h"

#include "util.h"
#include "peripheral.h"

decaIrqStatus_t decamutexon(void)
{
    uint8_t irqEnabled = nvic_get_irq_enabled(NVIC_EXTI1_IRQ);

    if (irqEnabled)
        nvic_disable_irq(NVIC_EXTI1_IRQ);
    return irqEnabled;
}

void decamutexoff(decaIrqStatus_t irqEnabled)
{
    if (irqEnabled)
        nvic_enable_irq(NVIC_EXTI1_IRQ);
}

void deca_sleep(unsigned int time_ms)
{
    msleep(time_ms);
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    decaIrqStatus_t irqStatus = decamutexon();

    gpio_clear(GPIOA, CS);

    for (uint16 i = 0; i < headerLength; i++)
        (void)spi_xfer(SPI1, headerBuffer[i]);

    for (uint32 i = 0; i < readlength; i++)
        readBuffer[i] = spi_xfer(SPI1, 0x00);

    gpio_set(GPIOA, CS);

    decamutexoff(irqStatus);

    return 0;
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
    decaIrqStatus_t irqStatus = decamutexon();

    gpio_clear(GPIOA, CS);

    for (uint16 i = 0; i < headerLength; i++)
        (void)spi_xfer(SPI1, headerBuffer[i]);

    for (uint32 i = 0; i < bodylength; i++)
        (void)spi_xfer(SPI1, bodyBuffer[i]);

    gpio_set(GPIOA, CS);

    decamutexoff(irqStatus);

    return 0;
}

void setup_dw1000(void)
{
    dwt_config_t config = {
        2,                /* Channel number. */
        DWT_PRF_64M,      /* Pulse repetition frequency. */
        DWT_PLEN_128,     /* Preamble length. Used in TX only. */
        DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
        9,                /* TX preamble code. Used in TX only. */
        9,                /* RX preamble code. Used in RX only. */
        0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_6M8,       /* Data rate. */
        DWT_PHRMODE_STD,  /* PHY header mode. */
        (128 + 1 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    };

    // reset
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
    gpio_clear(GPIOA, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);

    msleep(5);

    set_spi_slow();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        while (1)
        {
        }
    }

    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_CPLOCK))
    {
    }

    set_spi_fast();

    dwt_configure(&config);

    dwt_enableframefilter(0);

    // IRQ polarity: active high
    dwt_write32bitreg(SYS_CFG_ID, SYS_CFG_HIRQ_POL);

    // interruption setup
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO1);
    nvic_enable_irq(NVIC_EXTI1_IRQ);
    exti_select_source(EXTI1, GPIOA);
    exti_set_trigger(EXTI1, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI1);

    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
}

void exti1_isr(void)
{
    exti_reset_request(EXTI1);
    dwt_isr();
}
