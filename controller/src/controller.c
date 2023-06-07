#include "controller.h"


void controller_init() {

}

void controller_process(controller_actuators_t *actuators, controller_sensors_t *sensors) {
    actuators->left = 0.2;
    actuators->right = -0.2;
}