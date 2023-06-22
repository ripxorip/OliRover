#ifndef CONTROLLER
#define CONTROLLER

#include <stdint.h>
#include <stddef.h>

typedef void (*comm_write_t)(uint8_t *data, uint16_t len);
typedef size_t (*comm_read_t)(uint8_t *buffer, size_t buffer_len);

typedef enum
{
    CONTROLLER_PARAMS_KP = 0,
    CONTROLLER_PARAMS_KI,
    CONTROLLER_PARAMS_KD,
    CONTROLLER_PARAMS_NUM_PARAMS
} controller_params_t;

typedef struct
{
    float x;
    float y;
} controller_input_t;

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

void controller_init(comm_read_t read_fn, comm_write_t write_fn);
void controller_process(controller_actuators_t *actuators, controller_sensors_t *sensors, controller_input_t *input);
void controller_get_num_params(uint8_t *num_params);

void controller_set_param(controller_params_t param, float value);
void controller_get_param(controller_params_t param, float *value);

void controller_get_name_from_param(controller_params_t param, char *name, uint8_t name_len);

#endif /* CONTROLLER */
