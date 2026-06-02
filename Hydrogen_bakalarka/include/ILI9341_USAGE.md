# ILI9341 Display Library - Modified for Hardwired Pins

## Overview

This is a modified version of the ILI9341 display library optimized for:
- **Hardwired GPIO pins** (no need to pass pin configuration)
- **LED PWM backlight control**
- **Shared SPI bus support** (public CS control)

## Hardware Connections

```
RP2040 Pico → ILI9341 Display
━━━━━━━━━━━━━━━━━━━━━━━━━━━━
GP02 (SCK)   → SCK
GP03 (MOSI)  → MOSI (SDI)
GP04 (MISO)  → MISO (SDO)
GP06         → DC (Data/Command)
GP07         → RESET
GP08         → CS (Chip Select)
GP05         → LED (Backlight, PWM)
3.3V         → VCC
GND          → GND
```

## Features

### ✅ Hardwired Pins
Pins are defined at compile time in `ili9341.h`:
```c
#define ILI9341_GPIO_SCK    2
#define ILI9341_GPIO_MOSI   3
#define ILI9341_GPIO_MISO   4
#define ILI9341_GPIO_DC     6
#define ILI9341_GPIO_RESET  7
#define ILI9341_GPIO_CS     8
#define ILI9341_GPIO_LED    5
```

### ✅ LED PWM Control
Backlight brightness controlled via PWM (0-255):
```c
ILI9341_SetLED(255);  // Full brightness
ILI9341_SetLED(128);  // 50% brightness
ILI9341_SetLED(0);    // Backlight off
```

### ✅ Shared SPI Bus
Public CS control for sharing SPI bus with other devices:
```c
// Talk to display
ILI9341_CS_Enable();
// ... display operations ...
ILI9341_CS_Disable();

// Talk to other SPI device (e.g., SD card)
other_device_cs_enable();
// ... other device operations ...
other_device_cs_disable();
```

## Basic Usage

### Simple Example

```c
#include "pico/stdlib.h"
#include "ili9341.h"

int main() {
    stdio_init_all();
    
    // Initialize display (hardwired pins)
    ILI9341_Init();
    
    // Set LED brightness
    ILI9341_SetLED(200);  // 78% brightness
    
    // Create screen buffer
    screen_control_t screen = {0};
    TftClearScreenBuffer(&screen, kBlack, kWhite);
    
    // Write some text
    TftPutString(&screen, "Hello World!", 0, 0, kBlack, kWhite);
    
    // Update display
    TftFullScreenWrite(&screen);
    
    while(1) {
        tight_loop_contents();
    }
}
```

### Advanced Example with LED Control

```c
#include "pico/stdlib.h"
#include "ili9341.h"
#include "adc_measurement.h"  // Your ADC library

int main() {
    stdio_init_all();
    
    // Initialize display
    ILI9341_Init();
    
    // Initialize ADC
    adc_measurement_init();
    
    // Create screen buffer
    screen_control_t screen = {0};
    TftClearScreenBuffer(&screen, kBlack, kWhite);
    
    while(1) {
        // Read ADC values
        adc_readings_t readings;
        adc_read_all_channels(&readings);
        
        // Display voltage
        char buf[32];
        snprintf(buf, sizeof(buf), "VIN: %ld.%03ldV", 
                 readings.calibrated[CHANNEL_VIN] / 1000,
                 readings.calibrated[CHANNEL_VIN] % 1000);
        
        TftPutString(&screen, buf, 0, 0, kBlack, kCyan);
        
        // Display current
        snprintf(buf, sizeof(buf), "IOUT: %ld mA", 
                 readings.calibrated[CHANNEL_IOUT]);
        
        TftPutString(&screen, buf, 1, 1, kBlack, kYellow);
        
        // Update display
        TftFullScreenSelectiveWrite(&screen, 100);
        
        // Adjust LED brightness based on voltage
        uint8_t brightness = (readings.calibrated[CHANNEL_VIN] * 255) / 33000;
        ILI9341_SetLED(brightness);
        
        sleep_ms(100);
    }
}
```

### Shared SPI Bus Example

```c
#include "pico/stdlib.h"
#include "ili9341.h"
#include "sd_card.h"  // Example: SD card on same SPI bus

int main() {
    stdio_init_all();
    
    // Initialize display
    ILI9341_Init();
    
    // Initialize SD card (uses same SPI bus)
    sd_card_init();
    
    screen_control_t screen = {0};
    TftClearScreenBuffer(&screen, kBlack, kWhite);
    
    // Talk to display
    ILI9341_CS_Enable();
    TftPutString(&screen, "Reading SD card...", 0, 0, kBlack, kWhite);
    TftFullScreenWrite(&screen);
    ILI9341_CS_Disable();
    
    // Talk to SD card
    sd_card_cs_enable();
    char data[512];
    sd_card_read_block(0, data);
    sd_card_cs_disable();
    
    // Display result
    ILI9341_CS_Enable();
    TftPutString(&screen, "SD card data read!", 1, 1, kBlack, kGreen);
    TftFullScreenSelectiveWrite(&screen, 100);
    ILI9341_CS_Disable();
    
    while(1) {
        tight_loop_contents();
    }
}
```

## API Reference

### Initialization
```c
void ILI9341_Init(void);
```
Initialize display with hardwired pins. Call once at startup.

### LED Control
```c
void ILI9341_SetLED(uint8_t brightness);  // 0-255
uint8_t ILI9341_GetLED(void);             // Returns current brightness
```

### Chip Select Control (for shared SPI)
```c
void ILI9341_CS_Set(int state);      // 0=enable, 1=disable
void ILI9341_CS_Enable(void);        // CS low (active)
void ILI9341_CS_Disable(void);       // CS high (inactive)
```

### Screen Buffer Functions
```c
// Text operations
void TftClearScreenBuffer(screen_control_t *pscr, color_t paper, color_t ink);
void TftPutString(screen_control_t *pscr, const char* str, int top_y, int bot_y, int paper, int ink);
void TftPrintf(screen_control_t *pscr, int top_y, int bot_y, int paper, int ink, const char* str, ...);

// Graphics operations
void TftPutPixel(screen_control_t *pscr, int x, int y, color_t paper, color_t ink);
void TftPutLine(screen_control_t *pscr, int x0, int y0, int x1, int y1);

// Display update
void TftFullScreenWrite(screen_control_t *pscr);  // Full update
int TftFullScreenSelectiveWrite(screen_control_t *pscr, int nblock_max);  // Partial update
```

### Colors
```c
typedef enum {
    kBlack,
    kBlue,
    kRed,
    kMagenta,
    kGreen,
    kCyan,
    kYellow,
    kWhite
} color_t;
```

## Customizing Pin Assignments

If your hardware uses different pins, edit `ili9341.h`:

```c
// Change these to match your hardware
#define ILI9341_GPIO_SCK    2   // Your SCK pin
#define ILI9341_GPIO_MOSI   3   // Your MOSI pin
#define ILI9341_GPIO_MISO   4   // Your MISO pin
#define ILI9341_GPIO_DC     6   // Your DC pin
#define ILI9341_GPIO_RESET  7   // Your RESET pin
#define ILI9341_GPIO_CS     8   // Your CS pin
#define ILI9341_GPIO_LED    5   // Your LED pin
```

Then recompile.

## Memory Efficient Design

This library uses a clever 2-plane memory structure:
- **Graphics plane**: 1-bit per pixel (9600 bytes)
- **Color plane**: 8x8 blocks with attributes (1200 bytes)
- **Total**: 10,800 bytes vs 153,600 bytes for full RGB565

Perfect for menu systems, text displays, and retro graphics!

## CMakeLists.txt

```cmake
add_executable(my_project
    main.c
    ili9341.c
)

target_link_libraries(my_project
    pico_stdlib
    hardware_spi
    hardware_pwm
)
```

## Notes

- **SPI Speed**: Default 24MHz (can be changed in `ili9341.h`)
- **PWM Frequency**: ~488kHz (well above flicker perception)
- **Thread Safety**: Not thread-safe, use mutex if accessing from multiple cores
- **Shared SPI**: Always disable CS when not actively using display

## Advantages of Hardwired Pins

✅ **Simpler API** - No config struct to pass around
✅ **Smaller code** - No runtime pin configuration overhead
✅ **Faster** - Direct constants, no indirection
✅ **Less RAM** - No config structure storage
✅ **Type safety** - Pins checked at compile time

## Migration from Original Library

Old code:
```c
ili9341_config_t config;
ILI9341_Init(&config, spi0, 24000000, 4, 8, 2, 3, 7, 6);
```

New code:
```c
ILI9341_Init();  // That's it!
```

All the rest remains the same!
