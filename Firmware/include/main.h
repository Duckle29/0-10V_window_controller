#include <Arduino.h>
#include <Wire.h>
#include <avr/sleep.h>

#include "sht4x.h"

//#define NO_SERIAL

const uint8_t pin_feedback = PIN_PA3;
const uint8_t pin_dac = PIN_PA6;
const uint8_t pin_open = PIN_PA4;
const uint8_t pin_close = PIN_PA5;

const uint8_t unused_pins[] = {
    //PIN_PB2, PIN_PB3, // UART
    PIN_PA1, PIN_PA2, PIN_PA7
};

const uint16_t dial_range[] = {8, 740};
const uint16_t closed_range[] = {2000, 5000}; // 20-50C
const uint16_t open_range[] = {1000, 4000}; // 10-40C
const uint16_t max_temp = 12500; // 0-125C. uint will overflow to >600C
const uint8_t pos_out_range[] = {20, 235}; // 0.784 - 9.22V. Actuator closed-open range is 1-9V

// Actuator should be able to move a full 400mm stroke in ~90s
const uint16_t pos_timeout = 120; // [s]
const uint16_t pos_deadband_mm = 10; // [mm]
const uint16_t stroke = 400; // [mm]

const uint8_t pos_deadband_ticks = pos_deadband_mm * 255 / stroke; // [DAC ticks]
 
uint16_t sp_closed, sp_open;
uint8_t old_pos, new_pos = 0;
uint32_t deadline = 0 + 2*pos_timeout;

bool force_close, force_open = false;

uint8_t get_pos_out(uint16_t temp, uint16_t sp_closed, uint16_t sp_open);
void prepare_sleep();
void go_sleep();
void RTC_init();

SHT4x sht40;
