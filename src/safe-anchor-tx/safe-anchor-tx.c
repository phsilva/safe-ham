#include <stdio.h>

#define ANCHOR_TX #to get unique usb code for TX and RX

#include "deca_device_api.h"
#include "deca_regs.h"

#include "../common/peripheral.h"
#include "../common/usb.h"
#include "../common/util.h"
#include "../common/dw1000.h"

/////////////////////////////COMMON BLOCK BEGIN/////////////////////////////
static uint8 frame_seq_nb = 0;
typedef signed long long int64;
typedef unsigned long long uint64;
#define RX_BUF_LEN 20
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define SPEED_OF_LIGHT 299702547 /* Speed of light in air, in metres per second. */
static double tof;
static double distance;
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_buffer[RX_BUF_LEN];
static void fill_receive_buffer(const dwt_cb_data_t *cb_data){
    for (int i = 0; i < RX_BUF_LEN; i++)
    {
        rx_buffer[i] = 0;
    }
    if (cb_data->datalength <= RX_BUF_LEN)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }   
}
static bool is_resp_message(){
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    return (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0);
}
static bool is_poll_message(){
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    return (memcmp(rx_buffer, tx_poll_msg, ALL_MSG_COMMON_LEN) == 0);
}
static bool is_final_message(){
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    return (memcmp(rx_buffer, tx_final_msg, ALL_MSG_COMMON_LEN) == 0);
}

static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
static uint64 poll_tx_ts;
static uint64 poll_rx_ts;
static uint64 resp_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;
static uint64 final_tx_ts;
/////////////////////////////COMMON BLOCK END/////////////////////////////

//VOODOO NUMBERS
#define UUS_TO_DWT_TIME 65536 
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
#define RESP_RX_TIMEOUT_UUS 2700
#define PRE_TIMEOUT 8
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
#define FINAL_RX_TIMEOUT_UUS 3300
#define PRE_TIMEOUT 8

static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

#define ANCHOR_STATE_IDLE 0
#define ANCHOR_STATE_WAITING_RESP_MESSAGE 1
#define ANCHOR_STATE_RESP_MESSAGE_RECEIVED 2
#define ANCHOR_STATE_RESP_MESSAGE_FAILED 3
#define ANCHOR_STATE_FINAL_MESSAGE_SENT 4

static volatile uint8_t anchor_state = ANCHOR_STATE_IDLE;

static void send_poll_message(){
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);     /* Zero offset in TX buffer, no ranging. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    frame_seq_nb++;
}

static void send_final_message(){
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
