/**
 * main.cpp
 *
 * UART0 GP0/GP1  - MFC #1 (Bronkhorst, 38400 baud)
 * UART1 GP4/GP5  - MFC #2 (Bronkhorst, 38400 baud)
 * ADC0  GP26     - NTC1 (10k NTC B=3950, 10k pullup)
 * ADC1  GP27     - NTC2
 * ADC2  GP28     - Pressure (linear, 0-10 bar)
 * USB CDC        - CSV telemetry
 *
 * MFC polling: alternates MFC1/MFC2 every request (100ms each).
 *
 * sw_1 GP15 -> select MFC2 setpoint  (active low)
 * sw_2 GP14 -> select MFC1 setpoint  (active low)
 * sw_3 GP13 -> select CSV interval   (active low)
 * ENC        -> adjust selected parameter
 * ENC_BTN    -> confirm / send selected parameter
 *
 * CALIBRATION MENU: hold sw_1+sw_2+sw_3 simultaneously for 1s to enter.
 * Inside menu: ENC scrolls/adjusts, ENC_BTN moves to next field.
 * sw_1 held 1s = save+exit, sw_2 held 1s = exit without saving.
 *
 * Pressure calibration uses integer math:
 *   press_x100 = (adc_mv * gain_x10000 / 10000) + offset_x100
 * Defaults: gain_x10000=25 (0.0025 V/bar), offset_x100=-125 (-1.25 bar)
 *
 * Config saved to last flash sector (offset 0x1FF000, 4KB).
 */

#include "adc_measurement.h"
#include "bronkhorst.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "ili9341.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/time.h"

/* ---- MFC pins ------------------------------------------------------------ */
#define MFC1_TX  0
#define MFC1_RX  1
#define MFC2_TX  4
#define MFC2_RX  5

/* ---- GPIO ---------------------------------------------------------------- */
#define PWM_PIN     3
#define LED_ONBOARD 25
#define LED_KA      7
#define sw_1        15
#define sw_2        14
#define sw_3        13
#define ENC_A       10
#define ENC_B       12
#define ENC_BTN     11

/* ---- ADC ----------------------------------------------------------------- */
#define ADC_VREF_F   3.3f
#define ADC_COUNTS_F 4096.0f

/* ---- Timing -------------------------------------------------------------- */
#define MFC_POLL_US     100000ULL
#define LCD_MIN_US       20000ULL
#define KA_HALF_US      500000ULL

/* ---- CSV interval -------------------------------------------------------- */
#define CSV_MS_MIN      50
#define CSV_MS_MAX   10000
#define CSV_MS_STEP     50
#define CSV_MS_DEFAULT 500

/* ---- Setpoint ------------------------------------------------------------ */
#define SP_MIN   0
#define SP_MAX   10000
#define SP_STEP  100

/* ---- Encoder selection modes --------------------------------------------- */
#define SEL_MFC1  1
#define SEL_MFC2  2
#define SEL_CSV   3

/* ---- LCD ----------------------------------------------------------------- */
#define LCD_COLS  40
#define LCD_ROWS  30
#define COL_RIGHT 20

/* ==========================================================================
 * Flash config
 * Last sector of 2MB flash. Erase granularity = 4096 bytes.
 * struct is 16 bytes, padded to 256 (one page) for program alignment.
 * ========================================================================== */
#define CFG_FLASH_OFFSET  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define CFG_MAGIC         0xCAFEBABEu

/* Default pressure calibration:
 *   gain_x10000 = 25   means gain  = 0.0025  (bar per mV)
 *   offset_x100 = -125 means offset = -1.25  bar
 * Formula: press_x100 = (adc_mv * gain_x10000 / 10000) + offset_x100
 *
 * NTC calibration stored as integers:
 *   ntc_r0_ohm   : R0 in ohms        (default 10000)
 *   ntc_rpull_ohm: pullup in ohms    (default 10000)
 *   ntc_beta     : Beta value        (default 3950)
 *   ntc_t0_x10   : T0 in K * 10     (default 2981 = 298.1 K)
 */
#define CFG_GAIN_DEFAULT       14000
#define CFG_OFFSET_DEFAULT     (-800)
#define CFG_GAIN_MIN           1
#define CFG_GAIN_MAX           100000
#define CFG_GAIN_STEP          1
#define CFG_OFFSET_MIN         (-500000)
#define CFG_OFFSET_MAX         500000
#define CFG_OFFSET_STEP        1

#define CFG_NTC_R0_DEFAULT     10000
#define CFG_NTC_RPULL_DEFAULT  10000
#define CFG_NTC_BETA_DEFAULT   3950
#define CFG_NTC_T0X10_DEFAULT  2981
#define CFG_NTC_R_MIN          100
#define CFG_NTC_R_MAX          1000000
#define CFG_NTC_R_STEP         1      /* slow=1 ohm, fast=10 ohm */
#define CFG_NTC_BETA_MIN       1000
#define CFG_NTC_BETA_MAX       9999
#define CFG_NTC_BETA_STEP      1
#define CFG_NTC_T0X10_MIN      2000
#define CFG_NTC_T0X10_MAX      3500
#define CFG_NTC_T0X10_STEP     1

/* Pressure warning/alarm in bar*100. Step = 10 (0.1 bar) coarse, 100 (1 bar) fast */
#define CFG_PRESS_WARN_DEFAULT  1400   /*  8.00 bar */
#define CFG_PRESS_ALARM_DEFAULT 1600   /*  9.00 bar */
#define CFG_PRESS_MIN           0
#define CFG_PRESS_MAX           10000 /* 100.00 bar */
#define CFG_PRESS_STEP          10    /* 0.1 bar */

typedef struct {
    uint32_t magic;
    int32_t  press_gain_x10000;
    int32_t  press_offset_x100;
    int32_t  ntc_r0_ohm;
    int32_t  ntc_rpull_ohm;
    int32_t  ntc_beta;
    int32_t  ntc_t0_x10;
    int32_t  press_warn_x100;   /* warning threshold bar*100 */
    int32_t  press_alarm_x100;  /* alarm threshold bar*100   */
    uint32_t show_ntc2_mv;      /* 0=off, 1=on               */
    uint32_t checksum;
} cfg_t;

static cfg_t g_cfg;

static uint32_t cfg_checksum(const cfg_t *c) {
    return c->magic
         + (uint32_t)c->press_gain_x10000
         + (uint32_t)c->press_offset_x100
         + (uint32_t)c->ntc_r0_ohm
         + (uint32_t)c->ntc_rpull_ohm
         + (uint32_t)c->ntc_beta
         + (uint32_t)c->ntc_t0_x10
         + (uint32_t)c->press_warn_x100
         + (uint32_t)c->press_alarm_x100
         + c->show_ntc2_mv;
}

static void cfg_defaults(cfg_t *c) {
    c->magic              = CFG_MAGIC;
    c->press_gain_x10000  = CFG_GAIN_DEFAULT;
    c->press_offset_x100  = CFG_OFFSET_DEFAULT;
    c->ntc_r0_ohm         = CFG_NTC_R0_DEFAULT;
    c->ntc_rpull_ohm      = CFG_NTC_RPULL_DEFAULT;
    c->ntc_beta           = CFG_NTC_BETA_DEFAULT;
    c->ntc_t0_x10         = CFG_NTC_T0X10_DEFAULT;
    c->press_warn_x100    = CFG_PRESS_WARN_DEFAULT;
    c->press_alarm_x100   = CFG_PRESS_ALARM_DEFAULT;
    c->show_ntc2_mv       = 0;
    c->checksum           = cfg_checksum(c);
}

static void cfg_load(void) {
    const cfg_t *flash = (const cfg_t *)(XIP_BASE + CFG_FLASH_OFFSET);
    if (flash->magic == CFG_MAGIC &&
        flash->checksum == cfg_checksum(flash)) {
        g_cfg = *flash;
    } else {
        cfg_defaults(&g_cfg);
    }
}

static void cfg_save(void) {
    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    g_cfg.checksum = cfg_checksum(&g_cfg);
    memcpy(page, &g_cfg, sizeof(g_cfg));
    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(CFG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CFG_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(irq);
}

/* ==========================================================================
 * Pressure sensor (integer only)
 * press_x100 = (adc_mv * gain_x10000 / 10000) + offset_x100
 * ========================================================================== */
static int32_t pressure_read_x100(void) {
    adc_select_input(2);
    /* adc_mv = raw * 3300 / 4096  — use 32-bit, no overflow (4095*3300=13.5M) */
    int32_t adc_mv = (int32_t)((uint32_t)adc_read() * 3300u / 4096u);
    return (adc_mv * g_cfg.press_gain_x10000 / 10000) + g_cfg.press_offset_x100;
}

/* ==========================================================================
 * LCD shadow buffer
 * ========================================================================== */
static char shadow[LCD_ROWS][LCD_COLS + 1];

static void lrow_ic(screen_control_t *s, int row,
                    const char *txt, color_t paper, color_t ink) {
    char buf[LCD_COLS + 1];
    int n = (int)strlen(txt);
    if (n > LCD_COLS) n = LCD_COLS;
    memcpy(buf, txt, (size_t)n);
    memset(buf + n, ' ', (size_t)(LCD_COLS - n));
    buf[LCD_COLS] = '\0';
    if (memcmp(buf, shadow[row], LCD_COLS) == 0) return;
    memcpy(shadow[row], buf, LCD_COLS + 1);
    TftSetCursor(s, 0, row);
    TftPutString(s, buf, row, TEXT_HEIGHT, paper, ink);
}

static void lfield_ic(screen_control_t *s, int col, int row,
                      const char *txt, int width, color_t paper, color_t ink) {
    if (col + width > LCD_COLS) width = LCD_COLS - col;
    char f[LCD_COLS + 1];
    int n = (int)strlen(txt);
    if (n > width) n = width;
    memcpy(f, txt, (size_t)n);
    memset(f + n, ' ', (size_t)(width - n));
    f[width] = '\0';
    if (memcmp(shadow[row] + col, f, (size_t)width) == 0) return;
    memcpy(shadow[row] + col, f, (size_t)width);
    TftSetCursor(s, col, row);
    TftPutString(s, f, row, TEXT_HEIGHT, paper, ink);
}

static void lcd_clear_shadow(void) {
    memset(shadow, 0, sizeof(shadow));
}

/* Force-redraw one row (ignores shadow match) */
static void lrow_force(screen_control_t *s, int row,
                       const char *txt, color_t paper, color_t ink) {
    memset(shadow[row], 0, LCD_COLS + 1);
    lrow_ic(s, row, txt, paper, ink);
}

/* ==========================================================================
 * Encoder IRQ
 * ========================================================================== */
static volatile int32_t enc_delta = 0;
static void encoder_irq(uint gpio, uint32_t events) {
    (void)gpio; (void)events;
    bool a = gpio_get(ENC_A), b = gpio_get(ENC_B);
    if (a == b) enc_delta--; else enc_delta++;
}
static int32_t enc_read_clear(void) {
    uint32_t irq = save_and_disable_interrupts();
    int32_t v = enc_delta; enc_delta = 0;
    restore_interrupts(irq); return v;
}

/* ---- Beep ---------------------------------------------------------------- */
static alarm_id_t g_beep_id; static uint g_beep_slice;
static int64_t beep_off(alarm_id_t id, void *u) {
    (void)id; (void)u; pwm_set_enabled(g_beep_slice, false);
    gpio_set_function(PWM_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(PWM_PIN, GPIO_OUT); gpio_put(PWM_PIN, 0); return 0;
}
static void beep(uint32_t ms) {
    if (g_beep_id > 0) { cancel_alarm(g_beep_id); g_beep_id = 0; }
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    g_beep_slice = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_clkdiv(g_beep_slice, 125.0f); pwm_set_wrap(g_beep_slice, 500);
    pwm_set_chan_level(g_beep_slice, pwm_gpio_to_channel(PWM_PIN), 250);
    pwm_set_enabled(g_beep_slice, true);
    g_beep_id = add_alarm_in_ms(ms, beep_off, NULL, false);
}

/* ---- GPIO ---------------------------------------------------------------- */
static void init_gpio(void) {
    const uint btns[] = {sw_1, sw_2, sw_3, ENC_BTN};
    for (size_t i = 0; i < 4; i++) {
        gpio_init(btns[i]); gpio_set_dir(btns[i], GPIO_IN); gpio_pull_up(btns[i]);
    }
    gpio_init(ENC_A); gpio_set_dir(ENC_A, GPIO_IN); gpio_pull_up(ENC_A);
    gpio_init(ENC_B); gpio_set_dir(ENC_B, GPIO_IN); gpio_pull_up(ENC_B);
    gpio_set_irq_enabled_with_callback(ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, encoder_irq);
    gpio_init(LED_ONBOARD); gpio_set_dir(LED_ONBOARD, GPIO_OUT);
    gpio_init(LED_KA);      gpio_set_dir(LED_KA,      GPIO_OUT);
    gpio_put(LED_KA, 0);
}

/* ---- NTC ----------------------------------------------------------------- */
static int32_t ntc_read_x10(uint8_t ch) {
    adc_select_input(ch);
    uint16_t raw = adc_read();
    if (raw == 0 || raw >= 4095) return -9999;
    float v    = (raw / ADC_COUNTS_F) * ADC_VREF_F;
    float rpull = (float)g_cfg.ntc_rpull_ohm;
    float r0    = (float)g_cfg.ntc_r0_ohm;
    float beta  = (float)g_cfg.ntc_beta;
    float t0    = (float)g_cfg.ntc_t0_x10 / 10.0f;
    float rntc  = rpull * v / (ADC_VREF_F - v);
    float invt  = 1.0f / t0 + logf(rntc / r0) / beta;
    return (int32_t)((1.0f / invt - 273.15f) * 10.0f);
}

/* ---- Format helpers ------------------------------------------------------ */
static void fmt_x10(char *b, size_t n, int32_t v, const char *u) {
    if (v <= -9990) { snprintf(b, n, "---%s", u); return; }
    int32_t av = v < 0 ? -v : v;
    snprintf(b, n, "%s%ld.%ld%s",
             v < 0 ? "-" : "",
             (long)(av / 10), (long)(av % 10), u);
}
static void fmt_x100(char *b, size_t n, int32_t v, const char *u) {
    if (v == -1 || v <= -9990) { snprintf(b, n, "---%s", u); return; }
    int32_t av = v < 0 ? -v : v;
    snprintf(b, n, "%s%ld.%02ld%s",
             v < 0 ? "-" : "",
             (long)(av / 100), (long)(av % 100), u);
}
/* Like fmt_x100 but treats no value as <= -9990 only (0 is valid) */
static void fmt_x100_sensor(char *b, size_t n, int32_t v, const char *u) {
    if (v <= -9990) { snprintf(b, n, "---%s", u); return; }
    int32_t av = v < 0 ? -v : v;
    snprintf(b, n, "%s%ld.%02ld%s", v < 0 ? "-" : "",
             (long)(av/100), (long)(av%100), u);
}

/* ---- LCD MFC column (6 rows) --------------------------------------------- */
static void lcd_mfc(screen_control_t *s, const mfc_state_t *m,
                    int col, color_t flow_ink, int base) {
    char tmp[20], line[22];
    const int W = 19;

    fmt_x100(tmp, sizeof(tmp), mfc_raw_to_pct100(m->measure_raw), "%");
    snprintf(line, sizeof(line), "M:%-9s", tmp);
    lfield_ic(s, col, base+0, line, W, kBlack, flow_ink);

    fmt_x100(tmp, sizeof(tmp), mfc_raw_to_pct100(m->setpoint_raw), "%");
    snprintf(line, sizeof(line), "SP:%-8s", tmp);
    lfield_ic(s, col, base+1, line, W, kBlack, kCyan);

    lfield_ic(s, col, base+2, "", W, kBlack, kWhite);

    snprintf(line, sizeof(line), "TX:%-9lu", (unsigned long)m->tx_count);
    lfield_ic(s, col, base+3, line, W, kBlack, kWhite);

    snprintf(line, sizeof(line), "RX:%-9lu", (unsigned long)m->rx_count);
    lfield_ic(s, col, base+4, line, W, kBlack, kWhite);

    snprintf(line, sizeof(line), "Err:%-6lu", (unsigned long)m->err_count);
    lfield_ic(s, col, base+5, line, W, kBlack, m->err_count > 0 ? kRed : kWhite);
}

static uint16_t pct100_to_raw(int32_t p) {
    if (p <= 0) return 0; if (p >= 10000) return 32000;
    return (uint16_t)((p * 32000L) / 10000L);
}

/* ==========================================================================
 * Calibration menu
 *
 * Enter : hold sw_1+sw_2+sw_3 for 1 second.
 * ENC   : adjust (with acceleration)  ENC BTN : next field
 * sw_1 held 1s : SAVE   sw_2 held 1s : exit no save
 *
 * Fields: 0=gain 1=offset 2=warn 3=alarm 4=R0 5=Rpull 6=Beta 7=T0 8=NTC2mv
 * ========================================================================== */
#define CAL_FIELDS    9
#define CAL_HOLD_US   1000000ULL
#define CAL_DFLT_COL  26   /* column for right-aligned defaults */

static void draw_hold_bar(screen_control_t *s, int row, uint64_t held_us,
                          uint64_t total_us, const char *label, color_t ink) {
    const int BW = LCD_COLS - 12;
    int fill = (int)((held_us * (uint64_t)BW) / total_us);
    if (fill > BW) fill = BW;
    char line[LCD_COLS + 2];
    int pos = snprintf(line, sizeof(line), "%-7s [", label);
    for (int i = 0; i < BW; i++) line[pos++] = (i < fill) ? '=' : ' ';
    line[pos++] = ']'; line[pos] = '\0';
    memset(shadow[row], 0, LCD_COLS + 1);
    lrow_ic(s, row, line, kBlack, ink);
}

/* Render one cal field row with value left-side and default right-aligned
 * at a fixed column so all defaults line up regardless of value length. */
static void cal_row(screen_control_t *s, int row, int f, int field,
                    const char *label, const char *val, const char *dflt) {
    char line[LCD_COLS + 2];
    int llen = snprintf(line, sizeof(line), "[%s] %s%s",
                        (field == f) ? "*" : " ", label, val);
    while (llen < CAL_DFLT_COL && llen < LCD_COLS) line[llen++] = ' ';
    int dlen = (int)strlen(dflt);
    int start = LCD_COLS - dlen;
    if (start < llen) start = llen;
    if (start + dlen <= LCD_COLS) { memcpy(line + start, dflt, (size_t)dlen); llen = start + dlen; }
    while (llen < LCD_COLS) line[llen++] = ' ';
    line[LCD_COLS] = '\0';
    memset(shadow[row], 0, LCD_COLS + 1);
    lrow_ic(s, row, line, (field == f) ? kRed : kBlack, (field == f) ? kCyan : kWhite);
}

static void cal_menu(screen_control_t *screen) {
    cfg_t    cal      = g_cfg;
    int      field    = 0;
    bool     enc_prev = (bool)gpio_get(ENC_BTN);
    uint64_t sw1_down_us = 0, sw2_down_us = 0;
    bool     sw1_was_down = false, sw2_was_down = false;

    lcd_clear_shadow();
    TftClearScreenBuffer(screen, kBlack, kWhite);
    TftFullScreenWrite(screen);
    beep(100);

    while (true) {
        uint64_t now = time_us_64();
        bool sw1 = !gpio_get(sw_1);
        bool sw2 = !gpio_get(sw_2);
        bool sw3 = !gpio_get(sw_3);
        bool enc_btn = (bool)gpio_get(ENC_BTN);

        if (sw1 && !sw1_was_down) sw1_down_us = now;
        if (sw2 && !sw2_was_down) sw2_down_us = now;
        sw1_was_down = sw1; sw2_was_down = sw2;

        uint64_t sw1_held = sw1 ? (now - sw1_down_us) : 0;
        uint64_t sw2_held = sw2 ? (now - sw2_down_us) : 0;

        if (sw1_held >= CAL_HOLD_US) {
            g_cfg = cal; cfg_save();
            memset(shadow[29], 0, LCD_COLS + 1);
            lrow_ic(screen, 29, "*** SAVED ***", kBlack, kGreen);
            TftFullScreenSelectiveWrite(screen, 1200);
            beep(300); sleep_ms(600); break;
        }
        if (sw2_held >= CAL_HOLD_US) {
            memset(shadow[29], 0, LCD_COLS + 1);
            lrow_ic(screen, 29, "*** NOT SAVED ***", kBlack, kYellow);
            TftFullScreenSelectiveWrite(screen, 1200);
            beep(50); sleep_ms(100); beep(50); sleep_ms(500); break;
        }

        if (!enc_btn && enc_prev) { field = (field + 1) % CAL_FIELDS; beep(20); }
        enc_prev = enc_btn;

        int32_t ticks = enc_read_clear();
        if (ticks) {
            int32_t at = ticks < 0 ? -ticks : ticks;
            bool fast = (at > 3);
            switch (field) {
            case 0: { int32_t s = fast ? 10 : CFG_GAIN_STEP;
                cal.press_gain_x10000 += ticks*s;
                if (cal.press_gain_x10000 < CFG_GAIN_MIN) cal.press_gain_x10000=CFG_GAIN_MIN;
                if (cal.press_gain_x10000 > CFG_GAIN_MAX) cal.press_gain_x10000=CFG_GAIN_MAX; break; }
            case 1: { int32_t s = fast ? 10 : CFG_OFFSET_STEP;
                cal.press_offset_x100 += ticks*s;
                if (cal.press_offset_x100 < CFG_OFFSET_MIN) cal.press_offset_x100=CFG_OFFSET_MIN;
                if (cal.press_offset_x100 > CFG_OFFSET_MAX) cal.press_offset_x100=CFG_OFFSET_MAX; break; }
            case 2: { int32_t s = fast ? 100 : CFG_PRESS_STEP;
                cal.press_warn_x100 += ticks*s;
                if (cal.press_warn_x100 < CFG_PRESS_MIN) cal.press_warn_x100=CFG_PRESS_MIN;
                if (cal.press_warn_x100 > CFG_PRESS_MAX) cal.press_warn_x100=CFG_PRESS_MAX; break; }
            case 3: { int32_t s = fast ? 100 : CFG_PRESS_STEP;
                cal.press_alarm_x100 += ticks*s;
                if (cal.press_alarm_x100 < CFG_PRESS_MIN) cal.press_alarm_x100=CFG_PRESS_MIN;
                if (cal.press_alarm_x100 > CFG_PRESS_MAX) cal.press_alarm_x100=CFG_PRESS_MAX; break; }
            case 4: { int32_t s = fast ? CFG_NTC_R_STEP*10 : CFG_NTC_R_STEP;
                cal.ntc_r0_ohm += ticks*s;
                if (cal.ntc_r0_ohm < CFG_NTC_R_MIN) cal.ntc_r0_ohm=CFG_NTC_R_MIN;
                if (cal.ntc_r0_ohm > CFG_NTC_R_MAX) cal.ntc_r0_ohm=CFG_NTC_R_MAX; break; }
            case 5: { int32_t s = fast ? CFG_NTC_R_STEP*10 : CFG_NTC_R_STEP;
                cal.ntc_rpull_ohm += ticks*s;
                if (cal.ntc_rpull_ohm < CFG_NTC_R_MIN) cal.ntc_rpull_ohm=CFG_NTC_R_MIN;
                if (cal.ntc_rpull_ohm > CFG_NTC_R_MAX) cal.ntc_rpull_ohm=CFG_NTC_R_MAX; break; }
            case 6: { int32_t s = fast ? CFG_NTC_BETA_STEP*10 : CFG_NTC_BETA_STEP;
                cal.ntc_beta += ticks*s;
                if (cal.ntc_beta < CFG_NTC_BETA_MIN) cal.ntc_beta=CFG_NTC_BETA_MIN;
                if (cal.ntc_beta > CFG_NTC_BETA_MAX) cal.ntc_beta=CFG_NTC_BETA_MAX; break; }
            case 7: { int32_t s = fast ? CFG_NTC_T0X10_STEP*10 : CFG_NTC_T0X10_STEP;
                cal.ntc_t0_x10 += ticks*s;
                if (cal.ntc_t0_x10 < CFG_NTC_T0X10_MIN) cal.ntc_t0_x10=CFG_NTC_T0X10_MIN;
                if (cal.ntc_t0_x10 > CFG_NTC_T0X10_MAX) cal.ntc_t0_x10=CFG_NTC_T0X10_MAX; break; }
            case 8:
                cal.show_ntc2_mv = cal.show_ntc2_mv ? 0u : 1u; break;
            }
        }

        /* ---- Draw -------------------------------------------------------- */
        char line[LCD_COLS + 2], val[24], dflt[16];
        memset(val, 0, sizeof(val));
        memset(dflt, 0, sizeof(dflt));

        lrow_ic(screen, 0, "=== CALIBRATION ===", kBlack, kRed);
        lrow_ic(screen, 1, "ENC:adjust  ENC BTN:next field", kBlack, kWhite);

        if (sw1 && sw1_held < CAL_HOLD_US)
            draw_hold_bar(screen, 2, sw1_held, CAL_HOLD_US, "SAVE", kGreen);
        else lrow_ic(screen, 2, "sw1 hold 1s: SAVE+exit", kBlack, kGreen);

        if (sw2 && sw2_held < CAL_HOLD_US)
            draw_hold_bar(screen, 3, sw2_held, CAL_HOLD_US, "EXIT", kYellow);
        else lrow_ic(screen, 3, "sw2 hold 1s: exit no save", kBlack, kYellow);

        snprintf(line, sizeof(line), "sw1:[%s] sw2:[%s] sw3:[%s]  f:%d/8",
                 sw1?"X":" ", sw2?"X":" ", sw3?"X":" ", field);
        lrow_ic(screen, 4, line, kBlack, kWhite);

        lrow_ic(screen, 5, "-- PRESSURE ---------------dflt", kBlack, kCyan);

        snprintf(val, sizeof(val), "%ld", (long)cal.press_gain_x10000);
        snprintf(dflt, sizeof(dflt), "[%d]", CFG_GAIN_DEFAULT);
        cal_row(screen, 6, 0, field, "gain:", val, dflt);

        snprintf(val, sizeof(val), "%ld", (long)cal.press_offset_x100);
        snprintf(dflt, sizeof(dflt), "[%d]", CFG_OFFSET_DEFAULT);
        cal_row(screen, 7, 1, field, "offset:", val, dflt);

        /* live pressure preview — kept strictly to LCD_COLS chars */
        adc_select_input(2);
        int32_t adc_mv  = (int32_t)((uint32_t)adc_read() * 3300u / 4096u);
        int32_t preview = (adc_mv * cal.press_gain_x10000 / 10000) + cal.press_offset_x100;
        { int32_t av = preview<0?-preview:preview;
          snprintf(line, LCD_COLS + 1, "P:%s%ld.%02ld bar ADC:%ldmV",
                   preview<0?"-":"",(long)(av/100),(long)(av%100),(long)adc_mv);
          line[LCD_COLS] = '\0'; }
        lrow_ic(screen, 8, line, kBlack, kMagenta);

        snprintf(val, sizeof(val), "%ld.%02ld bar",
                 (long)(cal.press_warn_x100/100),(long)(cal.press_warn_x100%100));
        snprintf(dflt, sizeof(dflt), "[%d.%02d]",
                 CFG_PRESS_WARN_DEFAULT/100, CFG_PRESS_WARN_DEFAULT%100);
        cal_row(screen, 9, 2, field, "warn:", val, dflt);

        snprintf(val, sizeof(val), "%ld.%02ld bar",
                 (long)(cal.press_alarm_x100/100),(long)(cal.press_alarm_x100%100));
        snprintf(dflt, sizeof(dflt), "[%d.%02d]",
                 CFG_PRESS_ALARM_DEFAULT/100, CFG_PRESS_ALARM_DEFAULT%100);
        cal_row(screen, 10, 3, field, "alarm:", val, dflt);

        lrow_ic(screen, 11, "-- NTC --------------------dflt", kBlack, kCyan);

        snprintf(val, sizeof(val), "%ld ohm", (long)cal.ntc_r0_ohm);
        snprintf(dflt, sizeof(dflt), "[%d]", CFG_NTC_R0_DEFAULT);
        cal_row(screen, 12, 4, field, "R0:", val, dflt);

        snprintf(val, sizeof(val), "%ld ohm", (long)cal.ntc_rpull_ohm);
        snprintf(dflt, sizeof(dflt), "[%d]", CFG_NTC_RPULL_DEFAULT);
        cal_row(screen, 13, 5, field, "Rpull:", val, dflt);

        snprintf(val, sizeof(val), "%ld", (long)cal.ntc_beta);
        snprintf(dflt, sizeof(dflt), "[%d]", CFG_NTC_BETA_DEFAULT);
        cal_row(screen, 14, 6, field, "Beta:", val, dflt);

        { int32_t tc = cal.ntc_t0_x10 - 2731;
          int32_t at = tc<0?-tc:tc;
          snprintf(val, sizeof(val), "%ld.%ldK %s%ld.%ldC",
                   (long)(cal.ntc_t0_x10/10),(long)(cal.ntc_t0_x10%10),
                   tc<0?"-":"",(long)(at/10),(long)(at%10)); }
        snprintf(dflt, sizeof(dflt), "[%d]", CFG_NTC_T0X10_DEFAULT);
        cal_row(screen, 15, 7, field, "T0:", val, dflt);

        /* NTC live previews */
        { adc_select_input(0);
          uint16_t rraw = adc_read(); int32_t np = -9999;
          if (rraw>0 && rraw<4095) {
              float v = (rraw/ADC_COUNTS_F)*ADC_VREF_F;
              float rn = (float)cal.ntc_rpull_ohm*v/(ADC_VREF_F-v);
              float t0 = (float)cal.ntc_t0_x10/10.0f;
              float it = 1.0f/t0 + logf(rn/(float)cal.ntc_r0_ohm)/(float)cal.ntc_beta;
              np = (int32_t)((1.0f/it - 273.15f)*10.0f);
          }
          char tmp[12]; fmt_x10(tmp, sizeof(tmp), np, "C");
          snprintf(line, LCD_COLS + 1, "NTC1:%s", tmp);
          line[LCD_COLS] = '\0';
          lrow_ic(screen, 16, line, kBlack, kMagenta); }

        { adc_select_input(1);
          int32_t ntc2mv = (int32_t)((uint32_t)adc_read() * 3300u / 4096u);
          snprintf(line, LCD_COLS + 1, "NTC2 ADC: %ld mV", (long)ntc2mv);
          line[LCD_COLS] = '\0';
          lrow_ic(screen, 17, line, kBlack, kMagenta); }

        lrow_ic(screen, 18, "-- OTHER ------------------dflt", kBlack, kCyan);

        snprintf(val, sizeof(val), "%s", cal.show_ntc2_mv ? "ON " : "OFF");
        snprintf(dflt, sizeof(dflt), "[OFF]");
        cal_row(screen, 19, 8, field, "NTC2 mV:", val, dflt);

        lrow_ic(screen, 21, "----------------------------", kBlack, kWhite);
        lrow_ic(screen, 22, "2x Bronkhorst MFC controller", kBlack, kWhite);
        lrow_ic(screen, 23, "Communication via RS-232", kBlack, kWhite);
        lrow_ic(screen, 24, "Designed by Jedovcela 2026", kBlack, kWhite);

        TftFullScreenSelectiveWrite(screen, 1200);
        sleep_ms(20);
    }

    /* Restore main screen */
    lcd_clear_shadow();
    TftClearScreenBuffer(screen, kBlack, kWhite);
    TftFullScreenWrite(screen);
}

/* ==========================================================================
 * Core 1: USB serial command scanner
 *
 * scanf blocks until a line arrives — the correct USB CDC RX approach,
 * same as printf+stdio_flush is the correct TX approach.
 * Commands passed to core 0 via multicore FIFO:
 *   upper 16 bits = MFC index (1 or 2)
 *   lower 16 bits = setpoint value (0-10000)
 * ========================================================================== */
static void core1_usb_scanner(void) {
    while (!stdio_usb_connected()) sleep_ms(100);
    sleep_ms(200);

    char    line[32];
    uint8_t len = 0;

    while (true) {
        int ch = getchar_timeout_us(50000);

        bool timeout = (ch == PICO_ERROR_TIMEOUT);
        bool eol     = !timeout && ((char)ch == '\r' || (char)ch == '\n');

        if ((eol || (timeout && len > 0)) && len > 0) {
            line[len] = '\0';
            int idx = 0, val = -1;
            if (sscanf(line, "SP%d %d", &idx, &val) == 2 &&
                (idx == 1 || idx == 2) && val >= 0 && val <= 10000) {
                multicore_fifo_push_blocking(((uint32_t)idx << 16) | (uint32_t)val);
                printf("OK SP%d=%d\r\n", idx, val);
                stdio_flush();
            } else {
                printf("ERR use: SP1 <0-10000> or SP2 <0-10000>\r\n");
                stdio_flush();
            }
            len = 0;
        } else if (!timeout && !eol) {
            char c = (char)ch;
            if (len < (uint8_t)(sizeof(line) - 1))
                line[len++] = c;
            else
                len = 0;
        }
    }
}

/* ==========================================================================
 * main
 * ========================================================================== */
int main(void) {

    stdio_init_all();
    /* Remove uart0 from stdio input so MFC1 response bytes don't leak
     * into getchar_timeout_us on core1. USB CDC stdio remains active. */
    stdio_uart_deinit();
    init_gpio();
    adc_measurement_init();
    cfg_load();

    mfc_state_t mfc1 = {0}, mfc2 = {0};
    mfc_init(&mfc1, uart0, MFC1_TX, MFC1_RX);
    mfc_init(&mfc2, (uart_inst_t *)(UART1_BASE), MFC2_TX, MFC2_RX);

    ILI9341_Init();
    spi_set_baudrate(ILI9341_SPI_PORT, 20000000);
    ILI9341_SetLED(200);

    screen_control_t screen = {0};
    lcd_clear_shadow();
    TftClearScreenBuffer(&screen, kBlack, kWhite);
    sleep_ms(100);
    TftFullScreenWrite(&screen);   /* first write — clears shadow buffer */
    sleep_ms(100);
    TftFullScreenWrite(&screen);   /* second write — catches any residual pixels */

    sleep_ms(300);
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());

    /* Launch core1 USB scanner after all hardware is ready.
     * Launching before stdio/USB init causes core1 to get no input. */
    multicore_launch_core1(core1_usb_scanner);

    /* ---- state ----------------------------------------------------------- */
    uint64_t now_us = time_us_64();

    int      active      = 1;
    uint64_t next_req_us = now_us;
    bool     m1_req_meas = true;
    bool     m2_req_meas = true;

    uint64_t lcd_last_us = 0;
    uint64_t csv_next_us = now_us;
    uint64_t ka_last_us  = now_us;

    int32_t cpu_filt = (int32_t)get_cpu_temp();
    bool    ka_state = false;

    int     enc_sel      = SEL_MFC1;
    int32_t sp1          = 0;
    int32_t sp2          = 0;
    int32_t csv_ms       = CSV_MS_DEFAULT;
    int32_t csv_ms_active = CSV_MS_DEFAULT;
    bool    sp1_dirty    = false;
    bool    sp2_dirty    = false;
    bool    csv_dirty    = false;

    bool enc_prev = true;
    bool sw1_prev = true, sw2_prev = true, sw3_prev = true;
    bool csv_hdr  = false;

    /* Cal menu entry: track how long all 3 buttons are held */
    uint64_t all3_since = 0;
    bool     all3_armed = false;

    int32_t ntc1 = -9999, ntc2 = -9999, press = 0;
    int32_t ntc2_mv = 0;
    uint64_t alarm_beep_us = 0;
    uint64_t fps_ref = now_us; uint32_t fps_cnt = 0; uint16_t fps = 0;

    printf("\r\n=== MFC Monitor ===\r\n");

    /* ---- main loop ------------------------------------------------------- */
    while (true) {
        now_us = time_us_64();

        /* keep-alive */
        if (now_us - ka_last_us >= KA_HALF_US) {
            ka_last_us = now_us; ka_state = !ka_state;
            gpio_put(LED_KA, ka_state);
        }
        gpio_put(LED_ONBOARD, !gpio_get(LED_ONBOARD));

        /* ---- Buttons ---------------------------------------------------- */
        bool sw1 = (bool)gpio_get(sw_1);
        bool sw2 = (bool)gpio_get(sw_2);
        bool sw3 = (bool)gpio_get(sw_3);

        /* Cal menu: hold all 3 for 1 second */
        if (!sw1 && !sw2 && !sw3) {
            if (!all3_armed) { all3_since = now_us; all3_armed = true; }
            if (all3_armed && (now_us - all3_since) >= 1000000ULL) {
                all3_armed = false;
                cal_menu(&screen);
                /* After menu, reset button state to avoid spurious presses */
                sw1_prev = sw2_prev = sw3_prev = true;
                enc_prev = true;
                enc_read_clear();
            }
        } else {
            all3_armed = false;
        }

        /* Normal button actions (only when not all-3 held) */
        if (sw2 || sw3) { if (!sw1 && sw1_prev) { enc_sel = SEL_MFC1; beep(20); } }
        if (sw1 || sw3) { if (!sw2 && sw2_prev) { enc_sel = SEL_MFC2; beep(20); } }
        if (sw1 || sw2) { if (!sw3 && sw3_prev) { enc_sel = SEL_CSV;  beep(20); } }
        sw1_prev = sw1; sw2_prev = sw2; sw3_prev = sw3;

        /* ---- Encoder: adjust selected parameter -------------------------- */
        int32_t ticks = enc_read_clear();
        if (ticks) {
            if (enc_sel == SEL_MFC1) {
                sp1 += ticks * SP_STEP;
                if (sp1 < SP_MIN) sp1 = SP_MIN;
                if (sp1 > SP_MAX) sp1 = SP_MAX;
                sp1_dirty = true;
            } else if (enc_sel == SEL_MFC2) {
                sp2 += ticks * SP_STEP;
                if (sp2 < SP_MIN) sp2 = SP_MIN;
                if (sp2 > SP_MAX) sp2 = SP_MAX;
                sp2_dirty = true;
            } else {
                csv_ms += ticks * CSV_MS_STEP;
                if (csv_ms < CSV_MS_MIN) csv_ms = CSV_MS_MIN;
                if (csv_ms > CSV_MS_MAX) csv_ms = CSV_MS_MAX;
                csv_dirty = true;
            }
        }

        /* ---- ENC button -------------------------------------------------- */
        bool enc_btn = (bool)gpio_get(ENC_BTN);
        if (!enc_btn && enc_prev) {
            if (enc_sel == SEL_MFC1) {
                mfc_write_setpoint(&mfc1, pct100_to_raw(sp1));
                sp1_dirty = false;
            } else if (enc_sel == SEL_MFC2) {
                mfc_write_setpoint(&mfc2, pct100_to_raw(sp2));
                sp2_dirty = false;
            } else {
                csv_ms_active = csv_ms;
                csv_dirty     = false;
            }
            beep(30);
        }
        enc_prev = enc_btn;

        /* ---- USB serial command input ------------------------------------ */
        /* ---- USB serial commands from core 1 ----------------------------- */
        /* core1_usb_scanner pushes: upper16=MFC idx, lower16=setpoint value */
        if (multicore_fifo_rvalid()) {
            uint32_t msg = multicore_fifo_pop_blocking();
            int      idx = (int)(msg >> 16);
            int32_t  val = (int32_t)(msg & 0xFFFF);
            uint16_t raw = pct100_to_raw(val);
            if (idx == 1) { mfc_write_setpoint(&mfc1, raw); sp1 = val; sp1_dirty = false; }
            else           { mfc_write_setpoint(&mfc2, raw); sp2 = val; sp2_dirty = false; }
        }

        /* ---- MFC polling ------------------------------------------------- */
        mfc_poll(&mfc1);
        mfc_poll(&mfc2);

        if ((int64_t)(now_us - next_req_us) >= 0) {
            if (active == 1) {
                if (m1_req_meas) mfc_request_measure(&mfc1);
                else             mfc_request_setpoint(&mfc1);
                m1_req_meas = !m1_req_meas;
            } else {
                if (m2_req_meas) mfc_request_measure(&mfc2);
                else             mfc_request_setpoint(&mfc2);
                m2_req_meas = !m2_req_meas;
            }
            active      = (active == 1) ? 2 : 1;
            next_req_us = now_us + MFC_POLL_US;
        }

        /* ---- CPU temp ---------------------------------------------------- */
        cpu_filt = (cpu_filt * 9 + (int32_t)get_cpu_temp()) / 10;

        /* ---- ADC sensors ------------------------------------------------- */
        ntc1  = ntc_read_x10(0);
        ntc2  = ntc_read_x10(1);
        press = pressure_read_x100();
        /* NTC2 raw mV for optional display */
        if (g_cfg.show_ntc2_mv) {
            adc_select_input(1);
            ntc2_mv = (int32_t)((uint32_t)adc_read() * 3300u / 4096u);
        }

        /* ---- Pressure alarm beep ----------------------------------------- */
        if (press >= g_cfg.press_alarm_x100) {
            if ((int64_t)(now_us - alarm_beep_us) >= 0) {
                beep(80);
                alarm_beep_us = now_us + 500000ULL;  /* beep every 500ms */
            }
        }

        /* ---- LCD --------------------------------------------------------- */
        if (now_us - lcd_last_us >= LCD_MIN_US) {
            char line[48], tmp1[16];
            uint32_t uptime = (to_ms_since_boot(get_absolute_time()) - start_ms) / 1000;
            bool usb = stdio_usb_connected();

            lrow_ic(&screen, 0, "=== MFC Monitor ===", kBlack, kCyan);
            lrow_ic(&screen, 1, "", kBlack, kWhite);

            snprintf(line, sizeof(line), "USB:%-3s  t=%lu s",
                     usb ? "OK" : "---", (unsigned long)uptime);
            lrow_ic(&screen, 2, line, kBlack, usb ? kGreen : kRed);

            lrow_ic(&screen, 3, "", kBlack, kWhite);
            lrow_ic(&screen, 4, "-MFC1-------------- -MFC2--------------",
                    kBlack, kCyan);
            lrow_ic(&screen, 5, "", kBlack, kWhite);

            lcd_mfc(&screen, &mfc1, 0,        kGreen,  6);
            lcd_mfc(&screen, &mfc2, COL_RIGHT, kYellow, 6);

            lrow_ic(&screen, 12, "", kBlack, kWhite);

            fmt_x10(tmp1, sizeof(tmp1), ntc1, "C");
            snprintf(line, sizeof(line), "NTC1: %-10s", tmp1);
            lrow_ic(&screen, 13, line, kBlack, kYellow);

            /* Optional NTC2 raw mV */
            if (g_cfg.show_ntc2_mv) {
                snprintf(line, sizeof(line), "NTC2: %ld mV", (long)ntc2_mv);
                lrow_ic(&screen, 14, line, kBlack, kYellow);
            } else {
                lrow_ic(&screen, 14, "", kBlack, kWhite);
            }

            /* Pressure with warning/alarm colour */
            fmt_x100_sensor(tmp1, sizeof(tmp1), press, " bar");
            bool p_alarm = press >= g_cfg.press_alarm_x100;
            bool p_warn  = press >= g_cfg.press_warn_x100;
            if (p_alarm) {
                snprintf(line, sizeof(line), "Press:%-8s *** ALARM ***", tmp1);
                lrow_ic(&screen, 15, line, kBlack, kRed);
            } else if (p_warn) {
                snprintf(line, sizeof(line), "Press:%-8s ** WARN **", tmp1);
                lrow_ic(&screen, 15, line, kBlack, kYellow);
            } else {
                snprintf(line, sizeof(line), "Press: %-12s", tmp1);
                lrow_ic(&screen, 15, line, kBlack, kMagenta);
            }

            lrow_ic(&screen, 16, "", kBlack, kWhite);

            const char *sel_name =
                (enc_sel == SEL_MFC1) ? "MFC1 SP" :
                (enc_sel == SEL_MFC2) ? "MFC2 SP" : "CSV ms ";
            bool sel_dirty =
                (enc_sel == SEL_MFC1) ? sp1_dirty :
                (enc_sel == SEL_MFC2) ? sp2_dirty : csv_dirty;

            if (enc_sel == SEL_CSV) {
                snprintf(line, sizeof(line), "ENC>%s: %ld ms  (act:%ld)",
                         sel_name, (long)csv_ms, (long)csv_ms_active);
            } else {
                int32_t sel_val = (enc_sel == SEL_MFC1) ? sp1 : sp2;
                fmt_x100(tmp1, sizeof(tmp1), sel_val, "%FS");
                snprintf(line, sizeof(line), "ENC>%s: %-10s", sel_name, tmp1);
            }
            lrow_ic(&screen, 17, line, kBlack, kYellow);

            lrow_ic(&screen, 18,
                    sel_dirty ? "[press ENC to confirm]  "
                              : "                        ",
                    kBlack, kMagenta);

            lrow_ic(&screen, 19, "Hold sw1+sw2+sw3: calibration", kBlack, kWhite);

            snprintf(line, sizeof(line), "CSV: %ld ms (act:%ld ms)",
                     (long)csv_ms, (long)csv_ms_active);
            lrow_ic(&screen, 27, line, kBlack, kWhite);

            fmt_x10(tmp1, sizeof(tmp1), cpu_filt, "C");
            snprintf(line, sizeof(line), "CPU: %-10s", tmp1);
            lrow_ic(&screen, 28, line, kBlack, kWhite);

            snprintf(line, sizeof(line), "FPS: %-5u", fps);
            lrow_ic(&screen, 29, line, kBlack, kWhite);

            TftFullScreenSelectiveWrite(&screen, 1200);

            fps_cnt++;
            if (now_us - fps_ref >= 1000000ULL) {
                fps     = (uint16_t)(fps_cnt * 1000000ULL / (now_us - fps_ref));
                fps_cnt = 0; fps_ref = now_us;
            }
            lcd_last_us = now_us;
        }

        /* ---- CSV --------------------------------------------------------- */
        if ((int64_t)(now_us - csv_next_us) >= 0) {
            csv_next_us = now_us + (uint64_t)csv_ms_active * 1000ULL;
            if (stdio_usb_connected()) {
                if (!csv_hdr) {
                    printf("# Serial commands: SP1 <0-10000> | SP2 <0-10000>"
                           "  (value = %%FS x100, e.g. SP1 5000 = 50.00%%)\r\n");
                    printf("t[ms],"
                           "flow1_x100[%%],flow1_set_pointx100[%%],"
                           "flow2_x100[%%],flow2_set_pointx100[%%],"
                           "ntc1_x10[degC],"
                           "press_x100[bar]\r\n");
                    csv_hdr = true;
                }
                printf("%lu,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                       (unsigned long)(now_us / 1000),
                       (long)mfc_raw_to_pct100(mfc1.measure_raw),
                       (long)mfc_raw_to_pct100(mfc1.setpoint_raw),
                       (long)mfc_raw_to_pct100(mfc2.measure_raw),
                       (long)mfc_raw_to_pct100(mfc2.setpoint_raw),
                       (long)ntc1, (long)press);
            } else {
                csv_hdr = false;
            }
        }
    }
    return 0;
}