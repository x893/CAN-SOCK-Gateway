#ifndef __MESSAGES_H__
#define __MESSAGES_H__

#include <stdint.h>

typedef struct {
    uint64_t temp;
} RegularTelemetry;

#pragma pack(push, 1)
typedef struct {
    uint16_t temp_sensor_meas[8];
    uint32_t current_sensor_meas[8];
} MainTelemetry;
#pragma pack(pop)

typedef struct {
    uint8_t data[2048];
} ExtraTelemetry;

typedef struct {
    uint8_t data[2048];
} LogTelemetry;

typedef struct {
    uint16_t fine_guidance_sreen_x;
    uint16_t fine_guidance_sreen_y;
    uint16_t fine_guidance_mems_x;
    uint16_t fine_guidance_mems_y;
} TargetPayload;

typedef struct {
    float x;
    float y;
} FineGuidanceAnglesResponse;

typedef struct {
    uint16_t id;
    uint16_t payload_length;
} OrchestratorRequestHeader;

#endif  // __MESSAGES_H__
