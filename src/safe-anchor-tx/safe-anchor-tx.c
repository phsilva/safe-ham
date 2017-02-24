#include <stdio.h>

#define ANCHOR_TX #to get unique usb code for TX and RX

#include "deca_device_api.h"
#include "deca_regs.h"

#include "../common/peripheral.h"
#include "../common/usb.h"
#include "../common/util.h"
#include "../common/dw1000.h"
#include "../common/anchor_common.h"

#define ANCHOR_STATE_IDLE 0
#define ANCHOR_STATE_WAITING_RESP_MESSAGE 1
#define ANCHOR_STATE_RESP_MESSAGE_RECEIVED 2
#define ANCHOR_STATE_RESP_MESSAGE_FAILED 3
#define ANCHOR_STATE_FINAL_MESSAGE_SENT 4

static volatile uint8_t anchor_state = ANCHOR_STATE_IDLE;

static void send_poll_message(void){
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);     /* Zero offset in TX buffer, no ranging. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    frame_seq_nb++;
}

static void send_final_message(void){
    uint32 final_tx_time;
    int ret;
    /* Retrieve poll transmission and response reception timestamp. */
    poll_tx_ts = get_tx_timestamp_u64();
    resp_rx_ts = get_rx_timestamp_u64();
    /* Compute final message transmission time. See NOTE 10 below. */
    final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(final_tx_time);
    /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
    final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
    /* Write all timestamps in the final message. See NOTE 11 below. */
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
    /* Write and send final message. See NOTE 8 below. */
    tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
    ret = dwt_starttx(DWT_START_TX_DELAYED);
    frame_seq_nb++;
}

static void tx_ok_cb(const dwt_cb_data_t *cb_data)
{
    if (anchor_state==ANCHOR_STATE_FINAL_MESSAGE_SENT){
        anchor_state=ANCHOR_STATE_IDLE;
    }
}

static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    if (anchor_state == ANCHOR_STATE_WAITING_RESP_MESSAGE){

        fill_receive_buffer(cb_data);

        if (is_resp_message()){
            anchor_state = ANCHOR_STATE_RESP_MESSAGE_RECEIVED;
        }else{
            anchor_state = ANCHOR_STATE_RESP_MESSAGE_FAILED;
        }
    }else{
        //????discard????
    }
}
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    if (anchor_state == ANCHOR_STATE_WAITING_RESP_MESSAGE){
        printf("%ld: RX TIMEOUT 0x%04lx\r\n", system_millis, cb_data->status);
        usbd_poll(globalUSB);
        anchor_state = ANCHOR_STATE_RESP_MESSAGE_FAILED;
    }
}

static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    if (anchor_state == ANCHOR_STATE_WAITING_RESP_MESSAGE){
        printf("%ld: RX ERROR 0x%04lx\r\n", system_millis, cb_data->status);
        usbd_poll(globalUSB);
        anchor_state = ANCHOR_STATE_RESP_MESSAGE_FAILED;
    }
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

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);//???????
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);//???????
    //dwt_setpreambledetecttimeout(PRE_TIMEOUT); ?????????
    
    dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
    //dwt_setrxtimeout(60000);//????

    printf("OK, DW1000 ready: 0x%04lx.\r\n", dwt_read32bitreg(SYS_CFG_ID));
    usbd_poll(usbd_dev);

    while (1)
    {

        if (anchor_state==ANCHOR_STATE_IDLE){
            msleep(100);
            send_poll_message();
            anchor_state=ANCHOR_STATE_WAITING_RESP_MESSAGE;
        }else if (anchor_state==ANCHOR_STATE_RESP_MESSAGE_RECEIVED){
            send_final_message();
            anchor_state=ANCHOR_STATE_FINAL_MESSAGE_SENT;
        }else if (anchor_state==ANCHOR_STATE_RESP_MESSAGE_FAILED){
            //TODO???
        }
        
      
    }

    return 0;
}
