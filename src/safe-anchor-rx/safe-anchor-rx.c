#include <stdio.h>

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


#define ANCHOR_STATE_IDLE 0
#define ANCHOR_STATE_WAITING_POLL_MESSAGE 1
#define ANCHOR_STATE_POLL_MESSAGE_RECEIVED 2
#define ANCHOR_STATE_POLL_MESSAGE_FAILED 3
#define ANCHOR_STATE_RESP_MESSAGE_SENT 4
#define ANCHOR_STATE_WAITING_FINAL_MESSAGE 5
#define ANCHOR_STATE_FINAL_MESSAGE_RECEIVED 6
#define ANCHOR_STATE_FINAL_MESSAGE_FAILED 7

static volatile uint8_t anchor_state = ANCHOR_STATE_IDLE;


static int send_resp_message(){
    uint32 resp_tx_time;
    int ret;

    /* Retrieve poll reception timestamp. */
    poll_rx_ts = get_rx_timestamp_u64();

    /* Set send time for response. See NOTE 9 below. */
    resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

    /* Write and send the response message. See NOTE 10 below.*/
    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    //TODO check ret == DWT_ERROR ... do something here... abort?
    return ret;
}



static void tx_ok_cb(const dwt_cb_data_t *cb_data)
{

}

static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    if (anchor_state==ANCHOR_STATE_WAITING_POLL_MESSAGE){

        fill_receive_buffer(cb_data);//ponteiro certo aqui?

        if (is_poll_message()){
            anchor_state = ANCHOR_STATE_POLL_MESSAGE_RECEIVED;
        }else{
            anchor_state = ANCHOR_STATE_POLL_MESSAGE_FAILED;
        }
    }else if (anchor_state==ANCHOR_STATE_WAITING_FINAL_MESSAGE){

        fill_receive_buffer(cb_data);//ponteiro certo aqui?

        if (is_final_message()){
            anchor_state = ANCHOR_STATE_FINAL_MESSAGE_RECEIVED;
        }else{
            anchor_state = ANCHOR_STATE_FINAL_MESSAGE_FAILED;
        }
    }
}

static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    if (anchor_state == ANCHOR_STATE_WAITING_POLL_MESSAGE){
        printf("%ld: RX TIMEOUT 0x%04lx\r\n", system_millis, cb_data->status);
        usbd_poll(globalUSB);
        anchor_state = ANCHOR_STATE_POLL_MESSAGE_FAILED;
    }else if (anchor_state == ANCHOR_STATE_WAITING_FINAL_MESSAGE){
        printf("%ld: RX TIMEOUT 0x%04lx\r\n", system_millis, cb_data->status);
        usbd_poll(globalUSB);
        anchor_state = ANCHOR_STATE_FINAL_MESSAGE_FAILED;
    }
}

static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    if (anchor_state == ANCHOR_STATE_WAITING_POLL_MESSAGE){
        printf("%ld: RX ERROR 0x%04lx\r\n", system_millis, cb_data->status);
        usbd_poll(globalUSB);
        anchor_state = ANCHOR_STATE_POLL_MESSAGE_FAILED;
    }else if (anchor_state == ANCHOR_STATE_WAITING_FINAL_MESSAGE){
        printf("%ld: RX ERROR 0x%04lx\r\n", system_millis, cb_data->status);
        usbd_poll(globalUSB);
        anchor_state = ANCHOR_STATE_FINAL_MESSAGE_FAILED;
    }
}

static void reset_ranging(){
    //TODO how to properly reset the ranging???
    //this can come from error situations...

    /* Clear reception timeout to start next ranging process. */
    dwt_setrxtimeout(0);
    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void send_result_to_serial(){
    uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
    uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
    double Ra, Rb, Da, Db;
    int64 tof_dtu;
    /* Retrieve response transmission and final reception timestamps. */
    resp_tx_ts = get_tx_timestamp_u64();
    final_rx_ts = get_rx_timestamp_u64();
    /* Get timestamps embedded in the final message. */
    final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
    /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
    poll_rx_ts_32 = (uint32)poll_rx_ts;
    resp_tx_ts_32 = (uint32)resp_tx_ts;
    final_rx_ts_32 = (uint32)final_rx_ts;
    Ra = (double)(resp_rx_ts - poll_tx_ts);
    Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
    Da = (double)(final_tx_ts - resp_rx_ts);
    Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
    tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
    tof = tof_dtu * DWT_TIME_UNITS;
    distance = tof * SPEED_OF_LIGHT;
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

    /* Set preamble timeout for expected frames. See NOTE 6 below. */
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
    //dwt_setrxtimeout(60000);

    printf("OK, DW1000 ready: 0x%04lx.\r\n", dwt_read32bitreg(SYS_CFG_ID));
    usbd_poll(usbd_dev);

    while (1)
    {
        if (anchor_state==ANCHOR_STATE_IDLE){
            reset_ranging();
            anchor_state=ANCHOR_STATE_WAITING_POLL_MESSAGE;
        }else if (anchor_state==ANCHOR_STATE_POLL_MESSAGE_RECEIVED){
            int ret = send_resp_message();
            if (ret == DWT_ERROR){
                //log?
                anchor_state=ANCHOR_STATE_IDLE;
            }else{
                frame_seq_nb++;
                anchor_state=ANCHOR_STATE_WAITING_FINAL_MESSAGE;
            }
        }else if (anchor_state==ANCHOR_STATE_FINAL_MESSAGE_RECEIVED){
            send_result_to_serial();
            anchor_state=ANCHOR_STATE_IDLE;
        }

    }

    return 0;
}
