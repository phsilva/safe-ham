#include <stdio.h>

#include "deca_device_api.h"
#include "deca_regs.h"

#include "../common/peripheral.h"

#define ANCHOR_TX #to get unique usb code for TX and RX
#include "../common/usb.h"
#include "../common/util.h"

#include <libopencm3/stm32/gpio.h>

#define BLINK_FRAME_SN_IDX 1
#define TX_DELAY_MS 1000

static void setup_dw1000(void)
{
    /* Default communication configuration. We use here EVK1000's default mode (mode 3). */
    dwt_config_t config = {
        5,                /* Channel number. */
        DWT_PRF_16M,      /* Pulse repetition frequency. */
        DWT_PLEN_128,     /* Preamble length. Used in TX only. */
        DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
        4,                /* TX preamble code. Used in TX only. */
        4,                /* RX preamble code. Used in RX only. */
        0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_6M8,       /* Data rate. */
        DWT_PHRMODE_STD,  /* PHY header mode. */
        (128 + 1 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    };

    /* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10/11: frame check-sum, automatically set by DW1000.  */
    /* Index to access to sequence number of the blink frame in the tx_msg array. */

    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
    gpio_clear(GPIOA, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);

    msleep(5);

    set_spi_slow();

    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
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
    printf("OK, DW1000 ready.\r\n");
    usbd_poll(usbd_dev);

    uint32 status_reg = 0;
    uint8 tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};

    while (1)
    {
        /* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_msg), 0, 0);     /* Zero offset in TX buffer, no ranging. */

        /* Start transmission. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it.*/
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_TXFRS | SYS_STATUS_TXERR)))
        {
        }

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

        if (status_reg & SYS_STATUS_TXFRS)
        {
            uint32_t rf_status = dwt_read32bitoffsetreg(RF_CONF_ID, RF_STATUS_OFFSET);

            printf("%ld: TX OK: 0x%04lx 0x%04lx\r\n", system_millis, status_reg, rf_status);

            /* Clear TX frame sent event. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

            /* Increment the blink frame sequence number (modulo 256). */
            tx_msg[BLINK_FRAME_SN_IDX]++;
        }
        else if (status_reg & SYS_STATUS_TXERR)
        {
            if (status_reg & SYS_STATUS_TXBERR)
            {
                /* Clear event. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXBERR);
                printf("%ld: ERROR SYS_STATUS_TXBERR\r\n", system_millis);
            }
        }

        toggleLed();

        usbd_poll(usbd_dev);

        /* Execute a delay between transmissions. */
        msleep(100);
    }

    return 0;
}
