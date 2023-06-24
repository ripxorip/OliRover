#ifndef INTERFACE
#define INTERFACE

#include <stdint.h>

typedef enum
{
    INTERFACE_INPUT = (('I' << 0) | ('N' << 8) | ('P' << 16) | ('U' << 24)),
    INTERFACE_SENSORS = (('S' << 0) | ('E' << 8) | ('N' << 16) | ('S' << 24)),
    INTERFACE_GET_NUM_PARAMS = (('G' << 0) | ('N' << 8) | ('P' << 16) | ('A' << 24)),
    INTERFACE_REQ_PARAM_WITH_NAME = (('R' << 0) | ('P' << 8) | ('W' << 16) | ('N' << 24)),
    INTERFACE_GET_PARAM_WITH_NAME = (('G' << 0) | ('P' << 8) | ('W' << 16) | ('N' << 24)),
    INTERFACE_SET_PARAM = (('S' << 0) | ('P' << 8) | ('A' << 16) | ('R' << 24))
} interface_t;

typedef struct {
    float x;
    float y;
} interface_input_t;

typedef struct
{
    float linear_acceleration_x;
    float linear_acceleration_y;
    float linear_acceleration_z;

    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
} interface_sensors_t;

typedef struct {
    uint32_t num_parameters;
} interface_get_num_params;

typedef struct {
    uint32_t param_id;
} interface_req_param_with_name;

typedef struct {
    uint32_t param_id;
    float parameter_value;
    char parameter_name[32];
} interface_get_param_with_name;

typedef struct {
    uint32_t param_id;
    float parameter_value;
} interface_set_param;

#endif /* INTERFACE */
