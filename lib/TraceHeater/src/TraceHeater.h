#ifndef TRACE_HEATER_H_
#define TRACE_HEATER_H_

#include "application.h"
#include <cstdint>
#include <math.h>

// Trace Heater
#define TRACE_HEATER_PIN D7
#define TRACE_HEATER_ON                                                                            \
    LOW // LOW activates the PMOS, while HIGH disables the PMOS controlling the trace heater current
#define TRACE_HEATER_DEFAULT_BOARD_TAU                                                             \
    1061 // Originally calculated to be 1592, but wasn't quite right
#define TRACE_HEATER_SAFETY_MAX_TEMP 39.0
#define TRACE_HEATER_TIMER_PERIOD 5000 // Cal tick() every TRACE_HEATER_TIMER_PERIOD milliseconds
#define TRACE_HEATER_COOL_PERIOD 30000 // Must be divisible by TRACE_HEATER_TIMER_PERIOD
#define TRACE_HEATER_COOL_CYCLE_COUNT (TRACE_HEATER_COOL_PERIOD / TRACE_HEATER_TIMER_PERIOD)
#define TRACE_HEATER_DELTA 1.0
#define TRACE_HEATER_DELTA_MULTIPLIER 12.0
#define TRACE_HEATER_ALLOWED_COMPARE_ERROR 0.01
#define TRACE_HEATER_ALLOWED_AIM_ERROR 1.0

typedef enum trace_heater_st_e {
    TRACE_INIT, // Turn off the trace heater, measure initial temperature
    TRACE_AIM,  // Calculate target temperature, turn on/off heater, wait until target temperature
                // reached
    TRACE_COOL, // Turn off the trace heater, wait TRACE_HEATER_COOL_PERIOD milliseconds
    TRACE_COMPARE_CALC // See whether the ambient temperature was too high or too low
} trace_heater_st_t;

class TraceHeater {
  public:
    TraceHeater(uint32_t board_time_const, float (*read_temp_funct)(void), uint16_t heat_pin,
                uint8_t on_value);
    TraceHeater(uint32_t board_time_const, float (*read_temp_funct)(void));
    void tick();
    bool hasNewTemperatureData();
    float getTemperatureData();
    void reset();
    void begin();

  private:
    float getExpectedTemperature(
        float elapsedTimeSec, float ambientTemp,
        float initTemp); // Calculate what the board temperature should be after the given elapsed
                         // time and the estimated time constant, ambient temperature, and the
                         // measured initial temperature
    float (*read_temp_funct)(void);
    // void (*turn_on_heater_funct)();
    // void (*turn_off_heater_funct)();
    uint16_t heat_pin = TRACE_HEATER_PIN;
    uint8_t on_value = TRACE_HEATER_ON;
    uint8_t off_value = !TRACE_HEATER_ON;
    trace_heater_st_t trace_heater_st = TRACE_INIT;
    bool has_new_data = false;
    float temperature_data = 0.0;
    float temp_amb = 0.0;
    float temp_target = 10.0;
    uint16_t elapsed_cool_cycles = 0;
    uint32_t tau = TRACE_HEATER_DEFAULT_BOARD_TAU;
};

#endif