/**
 * adc_measurement.h
 * 
 * ADC voltage measurement with multiplexer support for RP2040
 * Uses 2 multiplexer control pins to select between 8 analog channels
 * 
 * Integer-only calibration system:
 * - All values in mV, mA, or mC (milli-units)
 * - No floating point math
 * - Per-channel gain and offset calibration
 */

#ifndef ADC_MEASUREMENT_H
#define ADC_MEASUREMENT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


// ADC hardware inputs (RP2040)
#define ADC_INPUT_0          0   // GPIO26
#define ADC_INPUT_1          1   // GPIO27
#define ADC_INPUT_2          2   // GPIO28

// ADC configuration
#define ADC_VREF_MV          3300     // Reference voltage in mV
#define ADC_RESOLUTION       4096     // 12-bit ADC (0-4095)
#define ADC_SETTLING_NOPS    2        // NOP delay for multiplexer settling

// ============================================================================
// DEFAULT CALIBRATION VALUES - CUSTOMIZE FOR YOUR HARDWARE
// ============================================================================
// These are the initial values loaded at startup
// You can modify these for your specific hardware, then each product can
// be fine-tuned individually after production if needed
//
// Formula: calibrated = ((raw_mv * gain) / divisor) - offset

#define DEFAULT_TEMP2_GAIN        100
#define DEFAULT_TEMP2_DIVISOR     1
#define DEFAULT_TEMP2_OFFSET      0


// Generic defaults (used only if channel-specific defaults not defined)
#define DEFAULT_GAIN         1000     // Gain multiplier (1000 = 1.000x)
#define DEFAULT_DIVISOR      1000     // Divisor for gain (allows decimal precision)
#define DEFAULT_OFFSET       0        // DC offset to subtract (in mV/mA/mC)



/**
 * Initialize ADC subsystem and multiplexer control pins
 * Loads default calibration values
 */
void adc_measurement_init(void);



/**
 * Read a single channel (raw ADC value)
 * 
 * @param channel Channel to read (use adc_channel_t enum)
 * @return Raw ADC value (0-4095)
 */
uint16_t adc_read_channel_raw(uint8_t channel);



/**
 * Convert raw ADC value to millivolts (integer only)
 * Formula: mv = (raw * VREF_MV) / ADC_RESOLUTION
 * 
 * @param raw_value Raw ADC reading (0-4095)
 * @return Voltage in millivolts
 */
int32_t adc_raw_to_mv(uint16_t raw_value);

uint32_t get_cpu_temp();

#ifdef __cplusplus
}
#endif

#endif // ADC_MEASUREMENT_H