#include <stdio.h>

#include "deca_device_api.h"
#include "deca_regs.h"

#include "../common/peripheral.h"
#include "../common/usb.h"
#include "../common/util.h"

#include <libopencm3/stm32/gpio.h>

#define FRAME_LEN_MAX 127
#define RESP_RX_TIMEOUT_UUS 2700
#define PRE_TIMEOUT 16

static void setup_dw1000(void)
{
    /* Default communication configuration. We use here EVK1000's default mode (mode 3). */
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

    /* Configure DW1000. See NOTE 3 below. */
    dwt_configure(&config);
}

int main(void)
{
    peripheral_init();

    usbd_device *usbd_dev = NULL;
    setup_usb(&usbd_dev);
    globalUSB = usbd_dev;

    debugLed(3);

    while (!usb_connected)
    {
        usbd_poll(usbd_dev);
    }

    while (!dw_run)
    {
        usbd_poll(usbd_dev);
    }

    setup_dw1000();
    printf("OK, DW1000 ready: 0x%04lx.\r\n", dwt_read32bitreg(SYS_CFG_ID));
    usbd_poll(usbd_dev);

    static uint8 rx_buffer[FRAME_LEN_MAX];

    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    static uint32 status_reg = 0;

    /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    static uint16 frame_len = 0;

    dwt_enableframefilter(0);
    dwt_setrxtimeout(60000);
    // dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    while (1)
    {
        int i;
        for (i = 0; i < FRAME_LEN_MAX; i++)
        {
            rx_buffer[i] = 0;
        }

        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO | SYS_STATUS_ALL_RX_ERR)))
        {
            // printf("%ld: RX STATUS 0x%04lx 0x%04lx\r\n", system_millis, status_reg, dwt_read32bitreg(SYS_CFG_ID));
            // usbd_poll(usbd_dev);
        };

        if (status_reg & SYS_STATUS_CLKPLL_LL)
        {
            /* Clear event. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_CLKPLL_LL);
            printf("%ld: ERROR SYS_STATUS_CLKPLL_LL\r\n", system_millis);
        }

        if (status_reg & SYS_STATUS_RFPLL_LL)
        {
            /* Clear event. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RFPLL_LL);
            printf("%ld: ERROR SYS_STATUS_RFPLL_LL\r\n", system_millis);
        }

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* A frame has been received, copy it to our local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
                char msg[13];
                memcpy(msg, rx_buffer, frame_len);
                msg[12] = '\0';
                printf("%ld: RX OK: %d 0x%04lx 0x%04lx %s\r\n", system_millis, frame_len, status_reg, dwt_read32bitreg(SYS_CFG_ID), msg);
                toggleLed();
            }

            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD);
        }
        else if (status_reg & SYS_STATUS_ALL_RX_ERR)
        {
            printf("%ld: RX ERROR 0x%04lx\r\n", system_millis, status_reg);
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
        else if (status_reg & SYS_STATUS_RXRFTO)
        {
            printf("%ld: RX TIMEOUT 0x%04lx\r\n", system_millis, status_reg);
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);
        }
        else if (status_reg & SYS_STATUS_RXPTO)
        {
            printf("%ld: RX PREAMBLE TIMEOUT 0x%04lx\r\n", system_millis, status_reg);
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPTO);
        }

        usbd_poll(usbd_dev);
    }

    return 0;
}
