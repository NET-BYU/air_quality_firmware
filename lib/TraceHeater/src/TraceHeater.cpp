#include "TraceHeater.h"

TraceHeater::TraceHeater() : heaterLog("TraceHeater") {
    this->tau = TRACE_HEATER_DEFAULT_BOARD_TAU;
    this->read_int_temp_funct = NULL;
    this->read_ext_temp_funct = NULL;
    this->read_unix_time_funct = NULL;
}

TraceHeater::TraceHeater(uint32_t board_time_const, float (*read_int_temp_funct)(void),
                         float (*read_ext_temp_funct)(void), uint32_t (*read_unix_time_funct)(void))
    : heaterLog("TraceHeater") {
    this->tau = board_time_const;
    this->read_int_temp_funct = read_int_temp_funct;
    this->read_ext_temp_funct = read_ext_temp_funct;
    this->read_unix_time_funct = read_unix_time_funct;
    if (this->tau == 0) {
        this->tau = TRACE_HEATER_DEFAULT_BOARD_TAU;
    }
}

void TraceHeater::begin() {
    // Set the Trace heeater to be initially off
    pinMode(TRACE_HEATER_PIN, OUTPUT);
    turn_off_heater();
}

float TraceHeater::calculateAmbientTemperature(float tau, float t, float initTemp,
                                               float afterTemp) {
    if (tau == 0) {
        tau = TRACE_HEATER_DEFAULT_BOARD_TAU;
    }
    float denom = (1 - exp(-t / tau));
    if (denom == 0) {
        heaterLog.error("TraceHeater: t must be greater than 0!");
        return INFINITY;
    }
    return ((afterTemp - initTemp) / denom) + initTemp;
}

void TraceHeater::trace_heater_loop() {
    switch (this->state) {
    case heater_init:
        heaterLog.trace("heater_init state");
        turn_off_heater();
        this->T_E = read_int_temp_funct();
        this->state = heater_target_up;
        break;
    case heater_target_up:
        heaterLog.trace("heater_target_up state");
        this->T_H = this->T_E + 10; // TODO Consider whether we should make this 12?
        // Add T_H_MAX cap
        turn_on_heater();
        this->t_h0 =
            read_unix_time_funct(); // TODO think about RTC reliability. Should we use a counter?
        this->state = heater_wait_until_heated;
        break;

    case heater_wait_until_heated: // TODO: Control better how often it's called
        heaterLog.trace(
            "heater_wait_until_heated state. Attempting to reach %f, but current temp is %f",
            this->T_H, read_int_temp_funct());
        if (read_int_temp_funct() >= this->T_H) {
            this->state = heater_target_down;
        }
        break;

    case heater_target_down:
        heaterLog.trace("heater_target_down state");
        turn_off_heater();
        this->t_h1 = read_unix_time_funct();
        this->t_h = this->t_h1 - this->t_h0;
        this->T_H_prime = read_int_temp_funct();
        this->T_C = this->T_H_prime - 2.0;
        this->t_c0 = read_unix_time_funct();
        this->state = heater_wait_until_cooled;
        break;

    case heater_wait_until_cooled:
        heaterLog.trace(
            "heater_wait_until_cooled state. Attempting to reach %f, but current temp is %f",
            this->T_C, read_int_temp_funct());
        if (read_int_temp_funct() >= this->T_H) {
            if (read_int_temp_funct() <= this->T_C) {
                this->state = heater_rebase;
            }
            break;

        case heater_rebase:
            heaterLog.trace("heater_rebase state");
            this->t_c1 = read_unix_time_funct();
            this->t_c = this->t_c1 - this->t_c0;
            this->T_C_prime = read_int_temp_funct();
            this->T_E =
                calculateAmbientTemperature(this->tau, this->t_c, this->T_H_prime, this->T_C_prime);
            break;
        }
    }
}

float TraceHeater::getEstimatedTemperature() { return this->T_E; }

void TraceHeater::reset() {
    state = heater_init;
    T_E = 0.0;
    T_H = 10.0;
}