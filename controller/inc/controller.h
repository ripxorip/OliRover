#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>

typedef struct
{
    float left;
    float right;
} controller_actuators_t;

typedef struct
{
    float linear_acceleration_x;
    float linear_acceleration_y;
    float linear_acceleration_z;

    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
} controller_sensors_t;

void controller_init();
void controller_process(controller_actuators_t *actuators, controller_sensors_t *sensors);

#endif /* CONTROLLER_H */
