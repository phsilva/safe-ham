#ifndef _INCLUDE_ANCHOR_COMMON_H_
#define _INCLUDE_ANCHOR_COMMON_H_ 1

#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#include "deca_device_api.h"
#include "deca_regs.h"

#include "util.h"
#include "peripheral.h"

/////////////////////////////COMMON BLOCK BEGIN/////////////////////////////

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

extern uint8 frame_seq_nb;

extern double tof;
extern double distance;
extern uint8 tx_poll_msg[12];
extern uint8 rx_resp_msg[15];
extern uint8 tx_resp_msg[15];
extern uint8 tx_final_msg[24];
extern uint8 rx_buffer[];
extern uint64 poll_tx_ts;
extern uint64 poll_rx_ts;
extern uint64 resp_rx_ts;
extern uint64 resp_tx_ts;
extern uint64 final_rx_ts;
extern uint64 final_tx_ts;

void fill_receive_buffer(const dwt_cb_data_t *cb_data);
bool is_resp_message(void);
bool is_poll_message(void);
bool is_final_message(void);
void final_msg_set_ts(uint8 *ts_field, uint64 ts);
void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);





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

#endif