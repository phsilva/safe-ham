#include "peripheral.h"

#include "button_boot.h"
#include "dw1000.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h> // for sys_tick_handler
#include <libopencm3/cm3/systick.h>

static void setup_boot_button(void)
{
    button_boot();
}

static void setup_clock(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

static void setup_systick(void)
{
    /* clock rate / 1000 to get 1mS interrupt rate */
    systick_set_reload(168000);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    /* this done last */
    systick_interrupt_enable();
}

static void setup_gpio(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    gpio_set(GPIOA, GPIO8);
}

static void setup_spi(void)
{
    /*
    MOSI: PA7
    MISO: PA6
    CLK : PA5
    CS  : PA4
  */

    /* chip select */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CS);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, CLK | MISO | MOSI);

    rcc_periph_clock_enable(RCC_SPI1);

    /* set to high which is not-selected */
    gpio_set(GPIOA, CS);

    gpio_set_af(GPIOA, GPIO_AF5, CLK | MISO | MOSI);

    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256,
                    /* high or low for the peripheral device */
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    /* CPHA: Clock phase: read on rising edge of clock */
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    /* DFF: Date frame format (8 or 16 bit) */
                    SPI_CR1_DFF_8BIT,
                    /* Most or Least Sig Bit First */
                    SPI_CR1_MSBFIRST);

    spi_enable(SPI1);
}

void set_spi_slow()
{
    spi_disable(SPI1);

    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256,
                    /* high or low for the peripheral device */
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    /* CPHA: Clock phase: read on rising edge of clock */
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    /* DFF: Date frame format (8 or 16 bit) */
                    SPI_CR1_DFF_8BIT,
                    /* Most or Least Sig Bit First */
                    SPI_CR1_MSBFIRST);

    spi_enable(SPI1);
}

void set_spi_fast()
{
    spi_disable(SPI1);

    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_8,
                    /* high or low for the peripheral device */
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    /* CPHA: Clock phase: read on rising edge of clock */
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    /* DFF: Date frame format (8 or 16 bit) */
                    SPI_CR1_DFF_8BIT,
                    /* Most or Least Sig Bit First */
                    SPI_CR1_MSBFIRST);

    spi_enable(SPI1);
}

void peripheral_init(void)
{
    setup_boot_button();
    setup_clock();
    setup_systick();
    setup_gpio();
    setup_spi();
}