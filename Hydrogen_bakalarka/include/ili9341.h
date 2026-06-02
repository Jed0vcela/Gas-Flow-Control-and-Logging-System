///////////////////////////////////////////////////////////////////////////////
//
//  Modified for hardwired GPIO pins and LED PWM control
//  Based on original by Roman Piksaykin [piksaykin@gmail.com], R2BDY
//
///////////////////////////////////////////////////////////////////////////////
#ifndef _ILI9341_H
#define _ILI9341_H

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"

// Simple assert macro (define assert_ if not already defined)
#ifndef assert_
#ifdef NDEBUG
#define assert_(x) ((void)0)
#else
#define assert_(x) do { if (!(x)) { printf("ASSERT FAILED: %s:%d\n", __FILE__, __LINE__); while(1); } } while(0)
#endif
#endif

#include "ili9341hw.h"
#include "font_8x8.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// HARDWIRED GPIO CONFIGURATION
// ============================================================================
// Customize these for your hardware connections
#define ILI9341_SPI_PORT    spi0        // SPI port (spi0 or spi1)
#define ILI9341_SPI_FREQ    24000000    // 24 MHz SPI clock

#define ILI9341_GPIO_SCK    18           // SPI Clock
#define ILI9341_GPIO_MOSI   19           // SPI MOSI (Master Out Slave In)
#define ILI9341_GPIO_MISO   16           // SPI MISO (Master In Slave Out)
#define ILI9341_GPIO_DC     20           // Data/Command select
#define ILI9341_GPIO_RESET  21           // Reset (active low)
#define ILI9341_GPIO_CS     17           // Chip Select (active low)
#define ILI9341_GPIO_LED    22           // Backlight LED (PWM controlled)

// Default LED brightness (0-255, where 255 is full brightness)
#define ILI9341_LED_DEFAULT_BRIGHTNESS  200

#define GET_DATA_BIT(p, n)  ((*((uint32_t *)(p) + ((n) >> 5)) \
        >> (31 - ((n) & 31))) & 1)
#define SET_DATA_BIT(p, n)  (*((uint32_t *)(p) + ((n) >> 5)) \
        |= (0x80000000 >> ((n) & 31)))
#define CLR_DATA_BIT(p, n)  (*((uint32_t *)(p) + ((n) >> 5)) \
        &=~(0x80000000 >> ((n) & 31)))

typedef enum 
{
    kBlack,
    kBlue,
    kRed,
    kMagenta,
    kGreen,
    kCyan,
    kYellow,
    kWhite
} color_t;

typedef struct
{
    int16_t mCursorX;                       // Cursor-
    int16_t mCursorY;                       // position.
    uint8_t mCursorType;                    // Not yet implemented. [*]

    color_t mCanvasPaper;                   // Default- 
    color_t mCanvasInk;                     // canvas colors.

    uint32_t mpPixBuffer[PIX_W32COUNT];     // Black-white 1bpp canvas.

    uint8_t mpColorBuffer[TEXT_CHARCOUNT];  // 8x8 block attributes:
                                // Flash|Changed|Pap2|Pap1|Pap0|Ink2|Ink1|Ink0.
                                // `Flash' blinking attribute (cursors) [*].
                                // `Changed` need to send to device flag.
                                // `Paper` color, `Ink` color [0..7].
} screen_control_t;

/* ============================================================================
   HARDWARE I/O LOW LEVEL OPERATIONS
   ============================================================================ */

/**
 * @brief Initialize ILI9341 display hardware
 * Initializes SPI, GPIO pins, and display controller
 * Also initializes LED backlight with default brightness
 */
void ILI9341_Init(void);

/**
 * @brief Set chip select state (for sharing SPI bus)
 * @param state 0 = enable (CS low), 1 = disable (CS high)
 * 
 * Use this when sharing SPI bus with other peripherals:
 * - Call ILI9341_CS_Enable() before display operations
 * - Call ILI9341_CS_Disable() after display operations
 */
void ILI9341_CS_Set(int state);
void ILI9341_CS_Enable(void);   // Convenience wrapper (CS low)
void ILI9341_CS_Disable(void);  // Convenience wrapper (CS high)

/**
 * @brief Set LED backlight brightness using PWM
 * @param brightness 0-255 (0 = off, 255 = full brightness)
 * 
 * Example:
 *   ILI9341_SetLED(128);  // 50% brightness
 *   ILI9341_SetLED(255);  // Full brightness
 *   ILI9341_SetLED(0);    // Backlight off
 */
void ILI9341_SetLED(uint8_t brightness);

/**
 * @brief Get current LED brightness
 * @return Current brightness (0-255)
 */
uint8_t ILI9341_GetLED(void);

// Internal low-level commands (you don't usually need these)
void ILI9341_SetCommand(uint8_t cmd);
void ILI9341_CommandParam(uint8_t data);
void ILI9341_SetOutWriting(const int start_col, const int end_col,
                            const int start_page, const int end_page);
void ILI9341_WriteData(void *buffer, int bytes);

/* ============================================================================
   SCREEN BUFFER OPERATIONS - TEXT
   ============================================================================ */

void TftClearScreenBuffer(screen_control_t *pscr, color_t paper, color_t ink);
void TftSetCursor(screen_control_t *pscr, int x, int y);

void TftPutChar(screen_control_t *pscr, int x, int y, int paper, int ink, 
                char chr);

void TftPutColorAttr(screen_control_t *pscr, int x, int y, int paper, int ink);

void TftPutString(screen_control_t *pscr, const char* str, int top_y, 
                int bot_y, int paper, int ink);

void TftPrintf(screen_control_t *pscr, int top_y, int bot_y, int paper,
                int ink, const char* str, ...);

void TftScrollVerticalZone(screen_control_t *pscr, int top_y, int bot_y);

/* ============================================================================
   SCREEN BUFFER OPERATIONS - GRAPHICS
   ============================================================================ */

void TftPutPixel(screen_control_t *pscr, int x, int y, color_t paper, color_t ink);
void TftPutLine(screen_control_t *pscr, int x0, int y0, int x1, int y1);
void TftPutTextLabel(screen_control_t *pscr, const char *pstr, int x_pix, 
                    int y_pix, bool over);
void TftClearRect8(screen_control_t *pscr, int x, int y);

/* ============================================================================
   HARDWARE I/O OPERATIONS
   ============================================================================ */

void TftFullScreenWrite(screen_control_t *pscr);
int TftFullScreenSelectiveWrite(screen_control_t *pscr, int nblock_max);
void TftSymbolWrite(screen_control_t *pscr, int sym_x, int sym_y);

/* ============================================================================
   COLOR PALETTE
   ============================================================================ */

static const uint16_t spPalette[8] = 
{
    0x0000, // Black.
    0x1F00, // Blue.
    0x00F8, // Red.
    0x1FF8, // Magenta.
    0xE007, // Green 0,0xFF,0.
    0xFF07, // Cyan 0xFF,0xFF,0.
    0xE0FF, // Yellow 0xFF,0xFF,0.
    0xffff  // White.
};

#ifdef __cplusplus
}
#endif

#endif
