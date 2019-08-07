/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.0-dev */

#ifndef PB_SENSOR_PACKET_PB_H_INCLUDED
#define PB_SENSOR_PACKET_PB_H_INCLUDED
#include "pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _SensorPacket {
    int32_t timestamp;
    int32_t sequence;
    bool has_temperature;
    int32_t temperature;
    bool has_humidity;
    int32_t humidity;
    bool has_rtc_temperature;
    int32_t rtc_temperature;
    bool has_pm1;
    int32_t pm1;
    bool has_pm2_5;
    int32_t pm2_5;
    bool has_pm4;
    int32_t pm4;
    bool has_pm10;
    int32_t pm10;
    bool has_card_present;
    bool card_present;
    bool has_queue_size;
    int32_t queue_size;
    bool has_app_version;
    int32_t app_version;
    bool has_co2;
    int32_t co2;
    bool has_voltage;
    int32_t voltage;
    bool has_current;
    int32_t current;
    bool has_total_energy;
    int32_t total_energy;
    bool has_reset_reason;
    int32_t reset_reason;
    bool has_free_memory;
    int32_t free_memory;
    bool has_power;
    int32_t power;
/* @@protoc_insertion_point(struct:SensorPacket) */
} SensorPacket;


/* Initializer values for message structs */
#define SensorPacket_init_default                {0, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define SensorPacket_init_zero                   {0, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define SensorPacket_timestamp_tag               1
#define SensorPacket_sequence_tag                2
#define SensorPacket_temperature_tag             3
#define SensorPacket_humidity_tag                4
#define SensorPacket_rtc_temperature_tag         5
#define SensorPacket_pm1_tag                     6
#define SensorPacket_pm2_5_tag                   7
#define SensorPacket_pm4_tag                     8
#define SensorPacket_pm10_tag                    9
#define SensorPacket_card_present_tag            10
#define SensorPacket_queue_size_tag              11
#define SensorPacket_app_version_tag             12
#define SensorPacket_co2_tag                     16
#define SensorPacket_voltage_tag                 17
#define SensorPacket_current_tag                 18
#define SensorPacket_power_tag                   22
#define SensorPacket_total_energy_tag            19
#define SensorPacket_reset_reason_tag            20
#define SensorPacket_free_memory_tag             21

/* Struct field encoding specification for nanopb */
#define SensorPacket_FIELDLIST(X, a) \
X(a, STATIC, REQUIRED, INT32, timestamp, 1) \
X(a, STATIC, REQUIRED, INT32, sequence, 2) \
X(a, STATIC, OPTIONAL, SINT32, temperature, 3) \
X(a, STATIC, OPTIONAL, INT32, humidity, 4) \
X(a, STATIC, OPTIONAL, SINT32, rtc_temperature, 5) \
X(a, STATIC, OPTIONAL, INT32, pm1, 6) \
X(a, STATIC, OPTIONAL, INT32, pm2_5, 7) \
X(a, STATIC, OPTIONAL, INT32, pm4, 8) \
X(a, STATIC, OPTIONAL, INT32, pm10, 9) \
X(a, STATIC, OPTIONAL, BOOL, card_present, 10) \
X(a, STATIC, OPTIONAL, INT32, queue_size, 11) \
X(a, STATIC, OPTIONAL, INT32, app_version, 12) \
X(a, STATIC, OPTIONAL, INT32, co2, 16) \
X(a, STATIC, OPTIONAL, INT32, voltage, 17) \
X(a, STATIC, OPTIONAL, INT32, current, 18) \
X(a, STATIC, OPTIONAL, INT32, total_energy, 19) \
X(a, STATIC, OPTIONAL, INT32, reset_reason, 20) \
X(a, STATIC, OPTIONAL, INT32, free_memory, 21) \
X(a, STATIC, OPTIONAL, INT32, power, 22)
#define SensorPacket_CALLBACK NULL
#define SensorPacket_DEFAULT NULL

extern const pb_msgdesc_t SensorPacket_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define SensorPacket_fields &SensorPacket_msg

/* Maximum encoded size of messages (where known) */
#define SensorPacket_size                        197

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
