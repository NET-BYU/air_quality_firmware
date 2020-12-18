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
    1697 // Originally calculated to be 1592, but wasn't quite right
// #define TRACE_HEATER_SAFETY_MAX_TEMP 39.0

typedef enum trace_heater_st_e {
    heater_init,
    heater_target_up,
    heater_wait_until_heated,
    heater_target_down,
    heater_wait_until_cooled,
    heater_rebase
} trace_heater_st_t;

class TraceHeater {
  public:
    TraceHeater();
    TraceHeater(uint32_t board_time_const, float (*read_int_temp_funct)(void),
                float (*read_ext_temp_funct)(void), uint32_t (*read_unix_time_funct)(void));
    void trace_heater_loop();
    void reset();
    void begin();
    float getEstimatedTemperature();
    trace_heater_st_t state;
    float T_E;
    uint32_t t_h;
    uint32_t t_h0;
    uint32_t t_h1;
    float T_H;
    float T_H_prime;
    uint32_t t_c;
    uint32_t t_c0;
    uint32_t t_c1;
    float T_C;
    float T_C_prime;
    float internal_temperature;

    void setTimeConst(uint32_t time_const) { this->tau = time_const; };
    void setIntTempFunct(float (*read_temp_funct)(void)) {
        this->read_int_temp_funct = read_temp_funct;
    };
    void setExtTempFunct(float (*read_temp_funct)(void)) {
        this->read_ext_temp_funct = read_temp_funct;
    };
    void setUnixTimeFunct(uint32_t (*read_unix_time_funct)(void)) {
        this->read_unix_time_funct = read_unix_time_funct;
    }

  private:
    float calculateAmbientTemperature(float tau, float t, float initTemp, float afterTemp);
    void turn_on_heater() { digitalWrite(TRACE_HEATER_PIN, TRACE_HEATER_ON); }
    void turn_off_heater() { digitalWrite(TRACE_HEATER_PIN, !TRACE_HEATER_ON); }
    float (*read_ext_temp_funct)(void);
    float (*read_int_temp_funct)(void);
    uint32_t (*read_unix_time_funct)(void);
    uint32_t tau = TRACE_HEATER_DEFAULT_BOARD_TAU;
    Logger heaterLog;
};

#endif