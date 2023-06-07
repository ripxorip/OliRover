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
    float x;
    float y;
    float z;
} controller_sensors_t;

void controller_init();
void controller_process(controller_actuators_t *actuators, controller_sensors_t *sensors);

#endif /* CONTROLLER_H */
