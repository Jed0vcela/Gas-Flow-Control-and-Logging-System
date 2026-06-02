/**
 * adc_measurement.c
 * 
 * ADC voltage measurement implementation with integer-only calibration
 * All calculations use mV, mA, or mC (milli-units) - NO FLOATING POINT
 */

#include "adc_measurement.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>
#include <math.h>


#define R0       10000
#define RPULL    10000
#define BETA     3950




//NOP-based delay for multiplexer settling time
static inline void delay_nops(int n) {
    for (volatile int i = 0; i < n; ++i) {
        __asm volatile ("nop");
    }
}


/**
 * Initialize ADC subsystem and multiplexer control pins
 */
void adc_measurement_init(void) {
    // Initialize ADC hardware
    adc_init();
    
    // Initialize GPIO pins for ADC inputs
    adc_gpio_init(26);  // ADC0
    adc_gpio_init(27);  // ADC1
    adc_gpio_init(28);  // ADC2
    adc_set_temp_sensor_enabled(1);
    /*
    // Initialize multiplexer control pins as outputs
    gpio_init(ADC_MUX1_PIN);
    gpio_set_dir(ADC_MUX1_PIN, GPIO_OUT);
    gpio_put(ADC_MUX1_PIN, 0);
    
    gpio_init(ADC_MUX2_PIN);
    gpio_set_dir(ADC_MUX2_PIN, GPIO_OUT);
    gpio_put(ADC_MUX2_PIN, 0);
    */
}

/**
 * Convert raw ADC value to millivolts (integer only)
 * Formula: mv = (raw * VREF_MV) / ADC_RESOLUTION
 * Example: raw=2048 -> (2048 * 3300) / 4096 = 1650 mV
 */
int32_t adc_raw_to_mv(uint16_t raw_value) {
    // Use 32-bit to prevent overflow during multiplication
    int32_t mv = ((int32_t)raw_value * ADC_VREF_MV) / ADC_RESOLUTION;
    return mv;
}


/**
 * Read a single channel (raw value)
 */
uint16_t adc_read_channel_raw(uint8_t channel) {
    if (channel > 2) {
        return 0;
    }
    
    switch(channel) {
        case 0:
            adc_select_input(ADC_INPUT_0);
            break;
        case 1:
            adc_select_input(ADC_INPUT_1);
            break;
        case 2:
            adc_select_input(ADC_INPUT_2);
            break;

        default:
            return 0;
    }
    
    return adc_read();
}

uint32_t get_cpu_temp() {
    adc_select_input(4); // 4
    uint16_t value = adc_read();

    uint32_t voltage_mv = (value * 3300) >> 12;

        // Výpočet teploty v desetinách stupně Celsia (T * 10)
        // Vzorec: 270 - (voltage_mv - 706) / 0.1721
        // Pro celá čísla: 270 - ((voltage_mv - 706) * 10000) / 1721
    int32_t temp_x10 = 270 - ((int32_t)(voltage_mv - 706) * 10000) / 1721;

    return temp_x10;
}
