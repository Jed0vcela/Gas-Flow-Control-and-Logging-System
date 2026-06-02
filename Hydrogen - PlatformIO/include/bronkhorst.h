#pragma once
/**
 * bronkhorst.h  –  Bronkhorst F-201CV ProPar RS232 driver
 *
 * Echo suppression: byte-matching against the last transmitted command.
 * Each incoming byte is compared to the expected echo; matching bytes
 * are silently dropped. The first byte that doesn't match (or after the
 * full echo is consumed) is treated as the start of the MFC's response.
 * This is robust regardless of timing.
 *
 * Confirmed working commands:
 *   Measure:  :06 80 04 01 20 01 20 \r\n
 *   Setpoint: :06 80 04 01 21 01 21 \r\n
 *   Write SP: :06 80 01 01 21 HH LL \r\n
 */

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MFC_BAUD  38400
#define MFC_NODE  0x80

typedef enum {
    MFC_OK = 0,
    MFC_ERR_FORMAT,
    MFC_ERR_DEVICE,
    MFC_ERR_NOT_READY,
} mfc_result_t;

typedef struct {
    uart_inst_t *uart;
    uint         tx_pin, rx_pin;

    int32_t  measure_raw;   /* 0-32000, -1 = no data */
    int32_t  setpoint_raw;  /* 0-32000, -1 = no data */

    uint32_t tx_count, rx_count, err_count;

    /* RX frame buffer */
    uint8_t  rx_buf[64];
    uint8_t  rx_head;

    /* Echo suppression: byte-match against last sent command */
    uint8_t  echo_buf[24];  /* copy of last sent command       */
    uint8_t  echo_len;      /* total bytes in echo_buf         */
    uint8_t  echo_pos;      /* how many echo bytes consumed    */

    bool     initialized;
} mfc_state_t;

void         mfc_init            (mfc_state_t *s, uart_inst_t *uart,
                                   uint tx_pin, uint rx_pin);
mfc_result_t mfc_poll            (mfc_state_t *s);
void         mfc_request_measure (mfc_state_t *s);
void         mfc_request_setpoint(mfc_state_t *s);
mfc_result_t mfc_write_setpoint  (mfc_state_t *s, uint16_t value);
int32_t      mfc_raw_to_pct100   (int32_t raw);

#ifdef __cplusplus
}
#endif