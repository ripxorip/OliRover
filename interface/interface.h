#ifndef INTERFACE
#define INTERFACE

#include <stdint.h>

typedef enum
{
    INTERFACE_INPUT = (('I' << 0) | ('N' << 8) | ('P' << 16) | ('U' << 24)),
    INTERFACE_SENSORS = (('S' << 0) | ('E' << 8) | ('N' << 16) | ('S' << 24))
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

#endif /* INTERFACE */