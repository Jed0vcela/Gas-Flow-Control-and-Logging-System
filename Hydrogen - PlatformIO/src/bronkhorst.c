/**
 * bronkhorst.c  –  Bronkhorst F-201CV ProPar RS232
 *
 * Echo suppression strategy:
 *   When we transmit a command, we store an exact copy in echo_buf.
 *   mfc_poll processes incoming bytes one at a time:
 *     - While echo_pos < echo_len: if the byte matches echo_buf[echo_pos],
 *       increment echo_pos and discard the byte (it's our own echo).
 *       If it doesn't match, the echo is already done (MFC replied faster
 *       than expected) — mark echo complete and process this byte normally.
 *     - Once echo is fully consumed: accumulate into rx_buf until '\n',
 *       then parse the frame.
 *
 *   This approach is timing-independent and never flushes the UART.
 */

#include "bronkhorst.h"
#include <stdio.h>
#include <string.h>
#include "hardware/timer.h"

static uint8_t hex_nibble(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0;
}
static uint8_t phb(const char *s) {
    return (uint8_t)((hex_nibble(s[0]) << 4) | hex_nibble(s[1]));
}

/* Transmit command and store it as the expected echo */
static void send_cmd(mfc_state_t *s, const char *cmd) {
    /* Drain any pending RX bytes. Wait up to 30ms for the previous response
     * to fully arrive, then flush it. Uses a polling loop with small sleeps
     * so core0 is not blocked and can service the multicore shared variables. */
    uint32_t deadline = to_ms_since_boot(get_absolute_time()) + 30;
    while (to_ms_since_boot(get_absolute_time()) < deadline) {
        while (uart_is_readable(s->uart)) uart_getc(s->uart);
        sleep_us(200);
    }
    while (uart_is_readable(s->uart)) uart_getc(s->uart);

    uint8_t len = (uint8_t)strlen(cmd);
    if (len > sizeof(s->echo_buf)) len = (uint8_t)sizeof(s->echo_buf);
    memcpy(s->echo_buf, cmd, len);
    s->echo_len  = len;
    s->echo_pos  = 0;
    s->rx_head   = 0;

    /* Enable TX pin only for the duration of transmission */
    gpio_set_function(s->tx_pin, GPIO_FUNC_UART);
    const char *p = cmd;
    while (*p) uart_putc_raw(s->uart, (uint8_t)*p++);
    uart_tx_wait_blocking(s->uart);
    /* Return TX pin to high-impedance input */
    gpio_set_function(s->tx_pin, GPIO_FUNC_SIO);
    gpio_set_dir(s->tx_pin, GPIO_IN);

    s->tx_count++;
}

static void send_read(mfc_state_t *s, uint8_t par_idx) {
    char cmd[18];
    uint8_t pp = (uint8_t)(0x20 | (par_idx & 0x0F));
    snprintf(cmd, sizeof(cmd), ":068004%02X%02X%02X%02X\r\n",
             0x01u, (unsigned)pp, 0x01u, (unsigned)pp);
    send_cmd(s, cmd);
}

void mfc_init(mfc_state_t *s, uart_inst_t *uart, uint tx_pin, uint rx_pin) {
    memset(s, 0, sizeof(*s));
    s->measure_raw = s->setpoint_raw = -1;
    s->uart = uart; s->tx_pin = tx_pin; s->rx_pin = rx_pin;
    uart_init(uart, MFC_BAUD);
    /* TX starts as high-impedance input — only driven during transmission */
    gpio_set_function(tx_pin, GPIO_FUNC_SIO);
    gpio_set_dir(tx_pin, GPIO_IN);
    /* RX always active */
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_hw_flow(uart, false, false);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart, true);
    s->initialized = true;
}

void mfc_request_measure (mfc_state_t *s) { if (s->initialized) send_read(s, 0); }
void mfc_request_setpoint(mfc_state_t *s) { if (s->initialized) send_read(s, 1); }

mfc_result_t mfc_write_setpoint(mfc_state_t *s, uint16_t value) {
    if (!s->initialized) return MFC_ERR_NOT_READY;
    if (value > 32000) value = 32000;
    // printf("DBG write SP raw=%u\r\n", (unsigned)value);
    char cmd[22];
    snprintf(cmd, sizeof(cmd), ":0680010121%02X%02X\r\n",
             (unsigned)((value >> 8) & 0xFF), (unsigned)(value & 0xFF));
    send_cmd(s, cmd);
    return MFC_OK;
}

mfc_result_t mfc_poll(mfc_state_t *s) {
    if (!s->initialized) return MFC_ERR_NOT_READY;

    while (uart_is_readable(s->uart)) {
        uint8_t c = (uint8_t)uart_getc(s->uart);

        /* Echo suppression disabled: send_cmd() flushes the RX FIFO before
         * transmitting, so no echo bytes will be in the FIFO by the time
         * the response arrives. The command and response share a common
         * prefix (":06800") which caused the old suppressor to consume
         * the response SOF ':' as an echo byte, losing the frame. */

        /* ---- Frame accumulation ----------------------------------------- */
        if (c == ':') {
            /* New frame start – always reset buffer.
             * Handles noise and the case where a ':' appears in the echo
             * mismatch path above. */
            s->rx_head = 0;
        }

        if (s->rx_head == 0 && c != ':') continue;  /* wait for SOF */
        if (c == '\r') continue;                      /* skip CR     */

        if (s->rx_head < (uint8_t)(sizeof(s->rx_buf) - 1)) {
            s->rx_buf[s->rx_head++] = c;
        } else {
            s->rx_head = 0; s->err_count++; continue;
        }

        if (c != '\n') continue;

        /* ---- Complete frame on '\n' -------------------------------------- */
        s->rx_buf[s->rx_head] = '\0';
        s->rx_head = 0;
        const char *ln  = (const char *)s->rx_buf;
        size_t      flen = strlen(ln);

        if (flen < 12 || ln[0] != ':') { s->err_count++; continue; }

        const char *p  = ln + 1;
        uint8_t len_b  = phb(p); p += 2;
        /* node */        phb(p); p += 2;
        uint8_t cmd_b  = phb(p); p += 2;

        /* Status/ack frame */
        if (cmd_b == 0x00) {
            uint8_t err = phb(p);
            s->rx_count++;
            if (err) { s->err_count++; return MFC_ERR_DEVICE; }
            return MFC_OK;
        }

        /* Data frame */
        if (len_b >= 6 && flen >= 15) {
            /* proc */       phb(p); p += 2;
            uint8_t par_b  = phb(p); p += 2;
            uint8_t hh     = phb(p); p += 2;
            uint8_t ll     = phb(p);
            int32_t val    = (int32_t)(((uint16_t)hh << 8) | ll);
            uint8_t idx    = par_b & 0x1F;
            if      (idx == 0) s->measure_raw  = val;
            else if (idx == 1) s->setpoint_raw = val;
            s->rx_count++;
            return MFC_OK;
        }

        s->err_count++;
        return MFC_ERR_FORMAT;
    }
    return MFC_ERR_NOT_READY;
}

int32_t mfc_raw_to_pct100(int32_t raw) {
    if (raw < 0) return -1;
    return (raw * 10000L) / 32000L;
}