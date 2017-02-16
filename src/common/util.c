#include "util.h"

#include <libopencm3/stm32/gpio.h>

volatile uint32_t system_millis;

void sys_tick_handler(void)
{
    system_millis++;
}

void msleep(uint32_t delay)
{
    uint32_t wake = system_millis + delay;
    while (wake > system_millis)
        ;
}

void ledOn(void)
{
    gpio_clear(GPIOA, GPIO8);
}

void ledOff(void)
{
    gpio_set(GPIOA, GPIO8);
}

void toggleLed()
{
    gpio_toggle(GPIOA, GPIO8);
}

void debugLed(uint8_t n)
{
    for (uint8_t i = 0; i < n; i++)
    {
        ledOn();
        msleep(300);
        ledOff();
        msleep(300);
    }

    msleep(1000);
}