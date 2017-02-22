#include <stdio.h>

#include "deca_device_api.h"
#include "deca_regs.h"

#include "../common/peripheral.h"
#include "../common/usb.h"
#include "../common/util.h"
#include "../common/dw1000.h"

#define FRAME_LEN_MAX 127
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

static volatile uint8_t rx_confirmed = 0;
static volatile uint8 rx_buffer[FRAME_LEN_MAX];

static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    for (int i = 0; i < FRAME_LEN_MAX; i++)
    {
        rx_buffer[i] = 0;
    }

    if (cb_data->datalength <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
        printf("%ld: RX OK: 0x%04lx (%d) %s\r\n", system_millis, cb_data->status, cb_data->datalength, rx_buffer);
        usbd_poll(globalUSB);
        toggleLed();
    }

    rx_confirmed = 1;
}

static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    printf("%ld: RX TIMEOUT 0x%04lx\r\n", system_millis, cb_data->status);
    usbd_poll(globalUSB);
    rx_confirmed = 1;
}

static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    printf("%ld: RX ERROR 0x%04lx\r\n", system_millis, cb_data->status);
    usbd_poll(globalUSB);
    rx_confirmed = 1;
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

    dwt_setcallbacks(NULL, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
    dwt_setrxtimeout(60000);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    printf("OK, DW1000 ready: 0x%04lx.\r\n", dwt_read32bitreg(SYS_CFG_ID));
    usbd_poll(usbd_dev);

    while (1)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        while (!rx_confirmed)
        {
        }

        rx_confirmed = 0;
    }

    return 0;
}
