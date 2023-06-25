#include "controller.h"
#include "interface.h"
#include "cbwo.h"

#include <string.h>
#include <stdio.h>

static struct
{
    float params[CONTROLLER_PARAMS_NUM_PARAMS];
    comm_read_t comm_read_fn;
    comm_write_t comm_write_fn;
    cbwo_t cb;
    struct
    {
        float x;
        float y;
    } input;
} internal = {0};

void controller_init(comm_read_t read_fn, comm_write_t write_fn)
{
    internal.comm_read_fn = read_fn;
    internal.comm_write_fn = write_fn;
    /* Setup the default params FIXME (Shall be from non-volatile memory) */
    internal.params[CONTROLLER_PARAMS_KP] = 0.50;
    internal.params[CONTROLLER_PARAMS_KI] = 0.05;
    internal.params[CONTROLLER_PARAMS_KD] = 0.005;
    cbwo_init(&internal.cb);
    internal.input.x = 0.0;
    internal.input.y = 0.0;
}

void write_to_rover(uint32_t id, uint8_t *data, uint16_t len)
{
    uint8_t buffer[1024] = {0};
    uint32_t offset = 0;

    /* Start of frame */
    memcpy(buffer, &id, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    /* Payload */
    memcpy(buffer + offset, data, len);
    offset += len;

    /* End of frame */
    id = ~id;
    memcpy(buffer + offset, &id, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    internal.comm_write_fn(buffer, offset);
}

int32_t verify_message(uint32_t id, size_t message_len, uint8_t *buffer, size_t buf_len)
{
    static uint8_t tmp_buffer[1024] = {0};
    static uint8_t tmp_buffer_reverse[1024] = {0};
    uint32_t id_end = 0;
    for (size_t i = 0; i < sizeof(uint32_t); i++)
    {
        uint8_t byte = 0;
        cbwo_get_value(&internal.cb, &byte, i);
        id_end |= byte << ((3 - i) * 8);
    }

    uint32_t id_start = 0;
    size_t start_offset = sizeof(uint32_t) + message_len;
    for (size_t i = start_offset; i < start_offset + sizeof(uint32_t); i++)
    {
        uint8_t byte = 0;
        cbwo_get_value(&internal.cb, &byte, i);
        id_start |= byte << ((3 - (i - start_offset)) * 8);
    }

    if ((id == id_start) && (~id == id_end))
    {
        size_t total_len = sizeof(uint32_t) + message_len + sizeof(uint32_t);

        for (size_t i = 0; i < total_len; i++)
        {
            cbwo_get_value(&internal.cb, tmp_buffer + i, i);
        }
        /* Reverse the tmp_buffer */
        for (size_t i = 0; i < total_len; i++)
        {
            tmp_buffer_reverse[i] = tmp_buffer[total_len - i - 1];
        }
        memcpy(buffer, tmp_buffer_reverse + sizeof(uint32_t), message_len);
        return message_len;
    }

    return -1;
}

void read_from_rover()
{
    static uint8_t comm_read_buffer[1024] = {0};
    static uint8_t message_buffer[1024] = {0};
    size_t len = internal.comm_read_fn(comm_read_buffer, sizeof(comm_read_buffer));
    for (size_t i = 0; i < len; i++)
    {
        cbwo_write(&internal.cb, comm_read_buffer[i]);
        if (verify_message(INTERFACE_SENSORS, sizeof(interface_sensors_t), message_buffer, len) >= 0)
        {
            interface_sensors_t *sensors = (interface_sensors_t *)(message_buffer);
        }
        if (verify_message(INTERFACE_GET_NUM_PARAMS, 0, message_buffer, sizeof(message_buffer)) >= 0)
        {
            interface_get_num_params num_params = {0};
            num_params.num_parameters = CONTROLLER_PARAMS_NUM_PARAMS;
            write_to_rover(INTERFACE_GET_NUM_PARAMS, (uint8_t *)&num_params, sizeof(interface_get_num_params));
        }
        if (verify_message(INTERFACE_REQ_PARAM_WITH_NAME, sizeof(interface_req_param_with_name), message_buffer, len) >= 0)
        {
            interface_req_param_with_name *req = (interface_req_param_with_name *)(message_buffer);
            interface_get_param_with_name resp = {0};
            controller_get_name_from_param(req->param_id, resp.parameter_name, sizeof(resp.parameter_name));
            resp.parameter_value = internal.params[req->param_id];
            resp.param_id = req->param_id;
            write_to_rover(INTERFACE_GET_PARAM_WITH_NAME, (uint8_t *)&resp, sizeof(interface_get_param_with_name));
        }
        if (verify_message(INTERFACE_SET_PARAM, sizeof(interface_set_param), message_buffer, len) >= 0)
        {
            interface_set_param *isp = (interface_set_param *)(message_buffer);
            controller_set_param(isp->param_id, isp->parameter_value);
        }
        if (verify_message(INTERFACE_INPUT, sizeof(interface_input_t), message_buffer, len) >= 0)
        {
            interface_input_t *input = (interface_input_t *)(message_buffer);
            internal.input.x = input->x;
            internal.input.y = input->y;
        }
    }
}

void controller_process(controller_actuators_t *actuators, controller_sensors_t *sensors)
{
    read_from_rover();
    /* actuators->left = internal.params[CONTROLLER_PARAMS_KP];
    actuators->right = -0.8; */

    actuators->left = internal.input.y;
    actuators->right = internal.input.y;
    actuators->left += internal.input.x;
    actuators->right -= internal.input.x;

    interface_sensors_t interface_sensors;
    interface_sensors.linear_acceleration_x = sensors->linear_acceleration_x;
    interface_sensors.linear_acceleration_y = sensors->linear_acceleration_y;
    interface_sensors.linear_acceleration_z = sensors->linear_acceleration_z;
    interface_sensors.angular_velocity_x = sensors->angular_velocity_x;
    interface_sensors.angular_velocity_y = sensors->angular_velocity_y;
    interface_sensors.angular_velocity_z = sensors->angular_velocity_z;
    write_to_rover(INTERFACE_SENSORS, (uint8_t *)&interface_sensors, sizeof(interface_sensors_t));
}

void controller_get_num_params(uint8_t *num_params)
{
    *num_params = CONTROLLER_PARAMS_NUM_PARAMS;
}

void controller_set_param(controller_params_t param, float value)
{
    internal.params[param] = value;
}

void controller_get_param(controller_params_t param, float *value)
{
    *value = internal.params[param];
}

void controller_get_name_from_param(controller_params_t param, char *name, uint8_t name_len)
{
    switch (param)
    {
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