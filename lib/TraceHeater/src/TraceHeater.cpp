#include "TraceHeater.h"

TraceHeater::TraceHeater(float (*read_temp_funct)(void), void (*turn_on_heater_funct)(),
                         void (*turn_off_heater_funct)(), Logger *heaterLog) {
    this->read_temp_funct = read_temp_funct;
    this->turn_on_heater_funct = turn_on_heater_funct;
    this->turn_off_heater_funct = turn_off_heater_funct;
    this->heaterLog = heaterLog;
}

void TraceHeater::tick() {
    float temp_m = 0.0;
    float temp_e = 0.0;
    switch (trace_heater_st) {
    case TRACE_INIT:
        heaterLog->info("Heater State: TRACE_INIT");
        turn_off_heater_funct();
        temp_amb = read_temp_funct();
        heaterLog->info("initial temperature is %f", temp_amb);
        trace_heater_st = TRACE_AIM;
    case TRACE_AIM:
        heaterLog->info("Heater State: TRACE_AIM");
        temp_target =
            temp_amb + 11.0; // Aiming for 11 degrees higher often gives us about 10 degrees higher
        if (temp_target > TRACE_HEATER_SAFETY_MAX_TEMP)
            temp_target = TRACE_HEATER_SAFETY_MAX_TEMP;
        temp_m = read_temp_funct();
        heaterLog->info("Heater attempting to reach target %f, current temp is %f", temp_target,
                        temp_m);
        if (temp_target > (temp_m + TRACE_HEATER_ALLOWED_AIM_ERROR)) {
            turn_on_heater_funct();
            heaterLog->info("Heating up to reach target temperature!");
            trace_heater_st = TRACE_AIM;
            break;
        } else if (temp_target < (temp_m - TRACE_HEATER_ALLOWED_AIM_ERROR)) {
            turn_off_heater_funct();
            heaterLog->info("Cooling down to reach target temperature!");
            trace_heater_st = TRACE_AIM;
            break;
        } else {
            heaterLog->info("Target temperature reached");
            trace_heater_st = TRACE_COOL;
            elapsed_cool_cycles = 0;
            temp_target = temp_m;
        }
    case TRACE_COOL:
        heaterLog->info("Heater State: TRACE_COOL");
        turn_off_heater_funct();
        if (elapsed_cool_cycles < TRACE_HEATER_COOL_CYCLE_COUNT) {
            elapsed_cool_cycles++;
            trace_heater_st = TRACE_COOL;
            break;
        } else {
            trace_heater_st = TRACE_COMPARE_CALC;
        }
    case TRACE_COMPARE_CALC:
        heaterLog->info("Heater State: TRACE_COMPARE_CALC");
        temp_e = (temp_amb - temp_target) * (1 - exp(-(TRACE_HEATER_COOL_PERIOD / 1000.0) /
                                                     ((float)TRACE_HEATER_DEFAULT_BOARD_TAU))) +
                 temp_target;
        temp_m = read_temp_funct();
        heaterLog->info("Heater: temp_e = %f, temp_m = %f", temp_e, temp_m);
        if (temp_m < (temp_e - TRACE_HEATER_ALLOWED_COMPARE_ERROR)) {
            heaterLog->info("Heater: Too hot! Cooling down.");
            temp_amb -= TRACE_HEATER_DELTA;
        } else if (temp_m > (temp_e + TRACE_HEATER_ALLOWED_COMPARE_ERROR)) {
            heaterLog->info("Heater: Too cold! Heating up.");
            temp_amb += TRACE_HEATER_DELTA;
        } else {
            heaterLog->info("Heater: Just right! Temperature is 10 degrees above ambient.");
            temp_amb = temp_m - 10.0;
            has_new_data = true;
            temperature_data = temp_amb;
        }
        trace_heater_st = TRACE_AIM;
    }
}

bool TraceHeater::hasNewTemperatureData() { return has_new_data; }

float TraceHeater::getTemperatureData() {
    has_new_data = false;
    return temperature_data;
}

void TraceHeater::reset() {
    trace_heater_st = TRACE_INIT;
    has_new_data = false;
    temperature_data = 0.0;
    temp_amb = 0.0;
    temp_target = 10.0;
    elapsed_cool_cycles = 0;
}