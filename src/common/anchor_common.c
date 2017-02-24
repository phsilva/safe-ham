#include "anchor_common.h"

#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#include "deca_device_api.h"
#include "deca_regs.h"

#include "util.h"
#include "peripheral.h"


uint8 frame_seq_nb = 0;

double tof;
double distance;
uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0}; //if modified, change size in .h too!
uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0}; //if modified, change size in .h too!
uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0}; //if modified, change size in .h too!
uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //if modified, change size in .h too!
uint8 rx_buffer[RX_BUF_LEN];

uint64 poll_tx_ts;
uint64 poll_rx_ts;
uint64 resp_rx_ts;
uint64 resp_tx_ts;
uint64 final_rx_ts;
uint64 final_tx_ts;

void fill_receive_buffer(const dwt_cb_data_t *cb_data){
    for (int i = 0; i < RX_BUF_LEN; i++)
    {
        rx_buffer[i] = 0;
    }
    if (cb_data->datalength <= RX_BUF_LEN)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }   
}
bool is_resp_message(void){
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    return (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0);
}
bool is_poll_message(void){
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    return (memcmp(rx_buffer, tx_poll_msg, ALL_MSG_COMMON_LEN) == 0);
}
bool is_final_message(void){
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    return (memcmp(rx_buffer, tx_final_msg, ALL_MSG_COMMON_LEN) == 0);
}

void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}
void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}
uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}