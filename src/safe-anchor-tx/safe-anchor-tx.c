#include <stdio.h>

#define ANCHOR_TX #to get unique usb code for TX and RX

#include "deca_device_api.h"
#include "deca_regs.h"

#include "../common/peripheral.h"
#include "../common/usb.h"
#include "../common/util.h"
#include "../common/dw1000.h"

static volatile uint8_t tx_confirmed = 0;

static void tx_ok_cb(const dwt_cb_data_t *cb_data)
{
    toggleLed();

    printf("%ld: TX OK: 0x%04lx\r\n", system_millis, cb_data->status);
    usbd_poll(globalUSB);

    tx_confirmed = 1;
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

    dwt_setcallbacks(&tx_ok_cb, NULL, NULL, NULL);

    printf("OK, DW1000 ready: 0x%04lx.\r\n", dwt_read32bitreg(SYS_CFG_ID));
    usbd_poll(usbd_dev);

    uint8 tx_msg[] = {0xC5, 0, 'M', 'E', 'G', 'A', 'F', 'A', 'K', 'E', 0, 0};

    while (1)
    {
        /* Write frame data to DW1000 and prepare transmission. */
        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_msg), 0, 0);     /* Zero offset in TX buffer, no ranging. */

        /* Start transmission. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* wait until IRQ confirm TX ok */
        while (!tx_confirmed)
        {
        }

        /* Execute a delay between transmissions. */
        msleep(100);

        tx_confirmed = 0;
    }

    return 0;
}
