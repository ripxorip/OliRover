#include "controller.h"

static struct
{
    float params[CONTROLLER_PARAMS_NUM_PARAMS];
} internal = {0};


void controller_init() {
    /* Setup the default params FIXME (Shall be from non-volatile memory) */
    internal.params[CONTROLLER_PARAMS_KP] = 0.50;
    internal.params[CONTROLLER_PARAMS_KI] = 0.05;
    internal.params[CONTROLLER_PARAMS_KD] = 0.005;
}

void controller_process(controller_actuators_t *actuators, controller_sensors_t *sensors, controller_input_t *input) {
    /* actuators->left = internal.params[CONTROLLER_PARAMS_KP];
    actuators->right = -0.8; */
    actuators->left = input->y;
    actuators->right = input->y;
    actuators->left += input->x;
    actuators->right -= input->x;
}

void controller_get_num_params(uint8_t *num_params) {
    *num_params = CONTROLLER_PARAMS_NUM_PARAMS;
}

void controller_set_param(controller_params_t param, float value) {
    internal.params[param] = value;
}

void controller_get_param(controller_params_t param, float *value) {
    *value = internal.params[param];
}

void controller_get_name_from_param(controller_params_t param, char *name, uint8_t name_len) {
    switch (param) {
        case CONTROLLER_PARAMS_KP:
            strncpy(name, "kp", name_len);
            break;
        case CONTROLLER_PARAMS_KI:
            strncpy(name, "ki", name_len);
            break;
        case CONTROLLER_PARAMS_KD:
            strncpy(name, "kd", name_len);
            break;
        default:
            strncpy(name, "unknown", name_len);
            break;
    }
}