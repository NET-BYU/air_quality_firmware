#include "TraceHeater.h"

TraceHeater::TraceHeater() : heaterLog("TraceHeater") {
    this->tau = TRACE_HEATER_DEFAULT_BOARD_TAU;
    this->read_int_temp_funct = NULL;
    this->read_ext_temp_funct = NULL;
    this->heat_pin = TRACE_HEATER_PIN;
    this->on_value = TRACE_HEATER_ON;
    this->off_value = !TRACE_HEATER_ON;
    this->read_unix_time_funct = NULL;
}

TraceHeater::TraceHeater(uint32_t board_time_const, float (*read_int_temp_funct)(void),
                         float (*read_ext_temp_funct)(void), uint32_t (*read_unix_time_funct)(void),
                         uint16_t heat_pin, uint8_t on_value)
    : heaterLog("TraceHeater") {
    this->tau = board_time_const;
    this->read_int_temp_funct = read_int_temp_funct;
    this->read_ext_temp_funct = read_ext_temp_funct;
    this->read_unix_time_funct = read_unix_time_funct;
    if (this->tau == 0) {
        this->tau = TRACE_HEATER_DEFAULT_BOARD_TAU;
    }
    this->heat_pin = heat_pin;
    this->on_value = on_value;
    this->off_value = !on_value;
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
    this->heat_pin = TRACE_HEATER_PIN;
    this->on_value = TRACE_HEATER_ON;
    this->off_value = !TRACE_HEATER_ON;
}

void TraceHeater::begin() {
    // Set the Trace heeater to be initially off
    pinMode(heat_pin, OUTPUT);
    digitalWrite(heat_pin, off_value);
}

/*void TraceHeater::tick() {
    float temp_m = read_int_temp_funct();
    float temp_e = 0.0;
    float temp_ext = 0.0;
    if (read_int_temp_funct == NULL || read_ext_temp_funct == NULL) {
        return;
    }
    switch (trace_heater_st) {
    case TRACE_INIT:
        heaterLog.info("Heater State: TRACE_INIT");
        digitalWrite(heat_pin, off_value);
        temp_amb = read_ext_temp_funct();
        target_adjust = 0.0;
        if (isinf(temp_amb)) {
            heaterLog.info("SHT31 not available! Turning off and going to INIT");
            return;
        }
        heaterLog.info("initial temperature is %f", temp_amb);
        trace_heater_st = TRACE_AIM;
    case TRACE_AIM:
        heaterLog.info("Heater State: TRACE_AIM");
        temp_target =
            temp_amb + 11.0; // Aiming for 11 degrees higher often gives us about 10 degrees higher
        if (temp_target - target_adjust > TRACE_HEATER_SAFETY_MAX_TEMP) {
            target_adjust =
                (temp_target -
                 TRACE_HEATER_SAFETY_MAX_TEMP); // temp_target = TRACE_HEATER_SAFETY_MAX_TEMP;
        }
        if (isinf(temp_m)) {
            digitalWrite(heat_pin, off_value);
            trace_heater_st = TRACE_INIT;
            heaterLog.info("SHT31 not available! Turning off and going to INIT");
            break;
        }
        heaterLog.info("Heater attempting to reach target %f, current temp is %f",
                       temp_target - target_adjust, temp_m);
        if (temp_target - target_adjust > (temp_m + TRACE_HEATER_ALLOWED_AIM_ERROR)) {
            digitalWrite(heat_pin, on_value);
            heaterLog.info("Heating up to reach target temperature!");
            trace_heater_st = TRACE_AIM;
            if ((prev_temp_m < temp_m + TRACE_HEATER_ALLOWED_COMPARE_ERROR) &&
                (prev_temp_m > temp_m - TRACE_HEATER_ALLOWED_COMPARE_ERROR)) {
                if (equal_count >= 5) {
                    target_adjust += TRACE_HEATER_DELTA;
                }
                equal_count++;
            } else {
                equal_count = 0;
            }
            break;
        } else if (temp_target - target_adjust < (temp_m - TRACE_HEATER_ALLOWED_AIM_ERROR)) {
            equal_count = 0;
            digitalWrite(heat_pin, off_value);
            heaterLog.info("Cooling down to reach target temperature!");
            trace_heater_st = TRACE_AIM;
            break;
        } else {
            equal_count = 0;
            heaterLog.info("Target temperature reached");
            trace_heater_st = TRACE_COOL;
            elapsed_cool_cycles = 0;
            temp_target = temp_m;
        }
    case TRACE_COOL:
        heaterLog.info("Heater State: TRACE_COOL");
        digitalWrite(heat_pin, off_value);
        if (elapsed_cool_cycles < TRACE_HEATER_COOL_CYCLE_COUNT) {
            elapsed_cool_cycles++;
            trace_heater_st = TRACE_COOL;
            break;
        } else {
            trace_heater_st = TRACE_COMPARE_CALC;
        }
    case TRACE_COMPARE_CALC:
        heaterLog.info("Heater State: TRACE_COMPARE_CALC");
        temp_e = getExpectedTemperature(TRACE_HEATER_COOL_PERIOD / 1000.0, temp_amb,
                                        temp_target - target_adjust);
        if (isinf(temp_m)) {
            digitalWrite(heat_pin, off_value);
            trace_heater_st = TRACE_INIT;
            heaterLog.info("SHT31 not available! Turning off and going to INIT");
            return;
        }
        heaterLog.info("Heater: temp_e = %f, temp_m = %f", temp_e, temp_m);
        if (temp_m < (temp_e - TRACE_HEATER_ALLOWED_COMPARE_ERROR)) {
            heaterLog.info("Heater: Too hot! Cooling down.");
            temp_amb -= TRACE_HEATER_DELTA;
            if (target_adjust >=
                TRACE_HEATER_DELTA) { // If we were too hot before but are cooling down
                target_adjust -= TRACE_HEATER_DELTA; // Slowly get rid of the adjustment, because we
                                                     // won't need it as much anymore
            } else {
                target_adjust = 0.0;
            }
        } else if (temp_m > (temp_e + TRACE_HEATER_ALLOWED_COMPARE_ERROR)) {
            heaterLog.info("Heater: Too cold! Heating up.");
            temp_amb += TRACE_HEATER_DELTA;
            temp_ext = read_ext_temp_funct();
            if (temp_ext != INFINITY && temp_amb > temp_ext) {
                temp_amb = temp_ext;
            }
        } else {
            heaterLog.info("Heater: Just right! Temperature is 10 degrees above ambient.");
            temp_amb = temp_m - 10.0 + target_adjust;
        }
        trace_heater_st = TRACE_AIM;
    }
    prev_temp_m = temp_m;
}*/

#define TWO_MINUTES 24
#define TWO_MINUTES_TIME 120
#define THIRTY_SECONDS 6
#define THIRTY_SECONDS_TIME 30
void TraceHeater::tick() {
    float T;
    uint32_t end_time;
    if (read_int_temp_funct == NULL || read_ext_temp_funct == NULL) {
        return;
    }
    switch (trace_heater_st) {
    case TRACE_INIT:
        heaterLog.info("state=TRACE_INIT");
        temp_amb = read_int_temp_funct();
        temp_target = temp_amb + 11.0;
        heaterLog.info("Ambient=%f, Target=%f", temp_amb, temp_target);
        trace_heater_st = TRACE_AIM;
    case TRACE_AIM:
        heaterLog.info("state=TRACE_AIM");
        if (temp_target > read_int_temp_funct()) {
            digitalWrite(heat_pin, on_value);
            temp_increasing = true;
            heaterLog.info("Heating up to reach target %f", temp_target);
        } else {
            digitalWrite(heat_pin, off_value);
            temp_increasing = false;
            heaterLog.info("Cooling down to reach target %f", temp_target);
        }
        trace_heater_st = TRACE_AIM_WAIT;
        wait_count = 0;
        if (read_unix_time_funct != NULL) {
            wait_start_time = read_unix_time_funct();
        } else {
            wait_start_time = 0;
        }
        break;
    case TRACE_AIM_WAIT:
        heaterLog.info("state=TRACE_AIM_WAIT");
        if ((wait_count >= TWO_MINUTES) ||
            (read_unix_time_funct != NULL &&
             (read_unix_time_funct() - wait_start_time >= TWO_MINUTES_TIME)) ||
            (temp_increasing && read_int_temp_funct() >= temp_target) ||
            (!temp_increasing && read_int_temp_funct() <= temp_target)) {
            digitalWrite(heat_pin, off_value);
            T_0 = read_int_temp_funct();
            heaterLog.info("Finished waiting after %d ticks, T_0=%f", wait_count, T_0);
            if (read_unix_time_funct != NULL) {
                wait_start_time = read_unix_time_funct();
            } else {
                wait_start_time = 0;
            }
            wait_count = 0;
            trace_heater_st = TRACE_COOL;
            break;
        } else {
            wait_count++;
            break;
        }
    case TRACE_COOL:
        heaterLog.info("state=TRACE_COOL");
        if (wait_count >= THIRTY_SECONDS ||
            (read_unix_time_funct != NULL &&
             (read_unix_time_funct() - wait_start_time >= THIRTY_SECONDS_TIME))) {
            T = read_int_temp_funct();
            if (read_unix_time_funct != NULL && wait_start_time != 0 &&
                (end_time = read_unix_time_funct()) != 0) {
                temp_amb = getAmbientTemperature((float)(end_time - wait_start_time), T_0, T) + 1.0;
            } else {
                temp_amb = getAmbientTemperature((float)(wait_count * 5), T_0, T) + 1.0;
                heaterLog.info("Getting ambient temperature after %d seconds, with T_0=%f, T=%f. "
                               "Result temp_amb=%f",
                               wait_count * 5, T_0, T, temp_amb);
            }
            temp_target = temp_amb + 11.0;
            trace_heater_st = TRACE_AIM;
            break;
        } else {
            wait_count++;
        }
    }
}

float TraceHeater::getEstimatedTemperature() { return temp_amb; }

// Calculate what the board temperature should be after the given elapsed time (seconds) and the
// estimated time constant (seconds), ambient temperature (Celsius), and the measured initial
// temperature (Celsius)
float TraceHeater::getExpectedTemperature(float t, float ambientTemp, float initTemp) {
    if (tau == 0) {
        tau = TRACE_HEATER_DEFAULT_BOARD_TAU;
    }
    return (ambientTemp - initTemp) * (1 - exp(-t / tau)) + initTemp;
}

// Calculate the ambient temperature
float TraceHeater::getAmbientTemperature(float t, float initTemp, float afterTemp) {
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

void TraceHeater::reset() {
    trace_heater_st = TRACE_INIT;
    temp_amb = 0.0;
    temp_target = 10.0;
    elapsed_cool_cycles = 0;
}