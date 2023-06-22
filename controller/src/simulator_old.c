#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <json-c/json.h>

#include "controller.h"
#include "sim_api.h"

static struct {
    struct {
        float y;
        float x;
    } input;
} internal = {0};

int gz_sim_send_socket_fd;
int gz_sim_recv_socket_fd;
int rover_rx_socket_fd;
int rover_tx_socket_fd;

struct sockaddr_in controller_sim_recv_addr, gz_sim_recv_addr, log_recv_addr, rover_tx_addr;

void handle_joystick_data(const char *axis, float value) {
    if (strcmp(axis, "y") == 0) {
        internal.input.y = -value;
    } else if (strcmp(axis, "x") == 0) {
        internal.input.x = value;
    }
}

// FIXME shall not be done from the controller, rather the rover
void send_log_data(controller_actuators_t *actuators, controller_sensors_t *sensors)
{
    struct json_object *json_obj = json_object_new_object();

    json_object_object_add(json_obj, "actuator_left", json_object_new_double(actuators->left));
    json_object_object_add(json_obj, "actuator_right", json_object_new_double(actuators->right));

    json_object_object_add(json_obj, "sensor_linear_acceleration_x", json_object_new_double(sensors->linear_acceleration_x));
    json_object_object_add(json_obj, "sensor_linear_acceleration_y", json_object_new_double(sensors->linear_acceleration_y));
    json_object_object_add(json_obj, "sensor_linear_acceleration_z", json_object_new_double(sensors->linear_acceleration_z));

    json_object_object_add(json_obj, "sensor_angular_velocity_x", json_object_new_double(sensors->angular_velocity_x));
    json_object_object_add(json_obj, "sensor_angular_velocity_y", json_object_new_double(sensors->angular_velocity_y));
    json_object_object_add(json_obj, "sensor_angular_velocity_z", json_object_new_double(sensors->angular_velocity_z));

    struct json_object *root = json_object_new_object();
    json_object_object_add(root, "type", json_object_new_string("log"));
    json_object_object_add(root, "data", json_obj);

    const char *json_string = json_object_to_json_string(root);

    if (sendto(rover_rx_socket_fd, json_string, strlen(json_string), 0, (const struct sockaddr *)&log_recv_addr, sizeof(log_recv_addr)) < 0)
    {
        perror("sendto failed");
        exit(EXIT_FAILURE);
    }
    json_object_put(json_obj);
}

void send_parameters() {
    struct json_object *json_obj = json_object_new_object();
    for (int i = 0; i < CONTROLLER_PARAMS_NUM_PARAMS; i++) {
        char name[64];
        controller_get_name_from_param(i, name, sizeof(name));
        float value;
        controller_get_param(i, &value);
        json_object_object_add(json_obj, name, json_object_new_double(value));
    }

    struct json_object *root = json_object_new_object();
    json_object_object_add(root, "type", json_object_new_string("parameters"));
    json_object_object_add(root, "data", json_obj);

    const char *json_string = json_object_to_json_string(root);
    if (sendto(rover_rx_socket_fd, json_string, strlen(json_string), 0, (const struct sockaddr *)&log_recv_addr, sizeof(log_recv_addr)) < 0)
    {
        perror("sendto failed");
        exit(EXIT_FAILURE);
    }
    json_object_put(json_obj);
}

void read_udp_data()
{
    controller_sensors_t sensors;
    sim_api_sensor_data_t sim_sensors;

    controller_actuators_t actuators;
    sim_api_actuator_data_t sim_actuators;

    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    #define BUFFER_SIZE 4096
    static uint8_t buffer[BUFFER_SIZE];

    // Check if data is available
    int bytes_available;
    ioctl(gz_sim_send_socket_fd, FIONREAD, &bytes_available);

    if (bytes_available > 0)
    {
        ssize_t recv_len = recvfrom(gz_sim_send_socket_fd, (char *)&sim_sensors, sizeof(sim_api_sensor_data_t), 0, (struct sockaddr *)&client_addr, &addr_len);
        if (sizeof(sim_api_sensor_data_t) == recv_len)
        {
            controller_actuators_t actuators;
            sensors.linear_acceleration_x = sim_sensors.linear_acceleration_x;
            sensors.linear_acceleration_y = sim_sensors.linear_acceleration_y;
            sensors.linear_acceleration_z = sim_sensors.linear_acceleration_z;

            sensors.angular_velocity_x = sim_sensors.angular_velocity_x;
            sensors.angular_velocity_y = sim_sensors.angular_velocity_y;
            sensors.angular_velocity_z = sim_sensors.angular_velocity_z;

            controller_input_t input;
            input.x = internal.input.x;
            input.y = internal.input.y;
            controller_process(&actuators, &sensors, &input);

            sim_actuators.left = actuators.left * 20;
            sim_actuators.right = actuators.right * 20;

            sendto(gz_sim_recv_socket_fd, (const char *)&sim_actuators, sizeof(sim_api_actuator_data_t), 0, (const struct sockaddr *)&gz_sim_recv_addr, sizeof(gz_sim_recv_addr));

            send_log_data(&actuators, &sensors);
        }
    }

    // Check if data is available, FIXME this belongs to the python code
    bytes_available;
    ioctl(rover_tx_socket_fd, FIONREAD, &bytes_available);

    if (bytes_available > 0)
    {
        ssize_t recv_len = recvfrom(rover_tx_socket_fd, &buffer, BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &addr_len);
        buffer[recv_len] = '\0';
        struct json_object *root = json_tokener_parse((char *)buffer);
        struct json_object *type;
        json_object_object_get_ex(root, "type", &type);
        const char *type_string = json_object_get_string(type);
        if (strcmp(type_string, "request_settings") == 0) {
            send_parameters();
        }
        else if (strcmp(type_string, "setting") == 0) {
            struct json_object *data;
            json_object_object_get_ex(root, "data", &data);
            const char* name = json_object_get_string(json_object_object_get(data, "setting"));
            float value = json_object_get_double(json_object_object_get(data, "value"));
            uint8_t found = 0;
            for (int i = 0; i < CONTROLLER_PARAMS_NUM_PARAMS; i++) {
                char param_name[64];
                controller_get_name_from_param(i, param_name, sizeof(param_name));
                if (strcmp(param_name, name) == 0) {
                    controller_set_param(i, value);
                    found = 1;
                    break;
                }
            }
            if (!found) {
                printf("ERROR: Unknown parameter: %s\n", name);
            }
        }
        else if (strcmp(type_string, "joystick") == 0) {
            struct json_object *data;
            json_object_object_get_ex(root, "data", &data);
            const char* axis = json_object_get_string(json_object_object_get(data, "axis"));
            float value = json_object_get_double(json_object_object_get(data, "value"));
            handle_joystick_data(axis, value);
        }
        else {
            printf("Unknown command: %s\n", type_string);
        }
    }
}

int main(int argc, char *argv[])
{
    char *gz_recv_ip = getenv("GZ_SIM_RECV_IP");
    printf("GZ_SIM_RECV_IP: %s\n", gz_recv_ip);

    char *gz_recv_port = getenv("GZ_SIM_RECV_PORT");
    printf("GZ_SIM_RECV_PORT: %s\n", gz_recv_port);
    int gz_recv_port_int = atoi(gz_recv_port);

    char *controller_sim_recv_port = getenv("CONTROLLER_SIM_RECV_PORT");
    printf("CONTROLLER_SIM_RECV_PORT: %s\n", controller_sim_recv_port);
    int controller_sim_recv_port_int = atoi(controller_sim_recv_port);

    char *log_recv_ip = getenv("LOG_RECV_IP");
    printf("LOG_RECV_IP: %s\n", log_recv_ip);

    char *log_recv_port = getenv("LOG_RECV_PORT");
    printf("LOG_RECV_PORT: %s\n", log_recv_port);
    int log_recv_port_int = atoi(log_recv_port);

    char *controller_sim_commands_ip = getenv("CONTROLLER_SIM_COMMANDS_IP");
    printf("CONTROLLER_SIM_COMMANDS_IP: %s\n", controller_sim_commands_ip);

    int controller_sim_commands_port_int = atoi(getenv("CONTROLLER_SIM_COMMANDS_PORT"));
    printf("CONTROLLER_SIM_COMMANDS_PORT: %d\n", controller_sim_commands_port_int);

    /* Initialize the controller */
    controller_init();
    if ((gz_sim_send_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    if ((gz_sim_recv_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    if ((rover_rx_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    if ((rover_tx_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&controller_sim_recv_addr, 0, sizeof(controller_sim_recv_addr));
    controller_sim_recv_addr.sin_family = AF_INET;
    controller_sim_recv_addr.sin_addr.s_addr = INADDR_ANY;
    controller_sim_recv_addr.sin_port = htons(controller_sim_recv_port_int);

    memset(&rover_tx_addr, 0, sizeof(rover_tx_addr));
    rover_tx_addr.sin_family = AF_INET;
    rover_tx_addr.sin_addr.s_addr = INADDR_ANY;
    rover_tx_addr.sin_port = htons(controller_sim_commands_port_int);

    memset(&gz_sim_recv_addr, 0, sizeof(gz_sim_recv_addr));
    gz_sim_recv_addr.sin_family = AF_INET;
    gz_sim_recv_addr.sin_port = htons(gz_recv_port_int);

    memset(&log_recv_addr, 0, sizeof(log_recv_addr));
    log_recv_addr.sin_family = AF_INET;
    log_recv_addr.sin_addr.s_addr = INADDR_ANY;
    log_recv_addr.sin_port = htons(log_recv_port_int);

    if (inet_pton(AF_INET, gz_recv_ip, &(gz_sim_recv_addr.sin_addr)) <= 0)
    {
        perror("invalid address");
        exit(EXIT_FAILURE);
    }

    if (inet_pton(AF_INET, log_recv_ip, &(log_recv_addr.sin_addr)) <= 0)
    {
        perror("invalid address");
        exit(EXIT_FAILURE);
    }

    // Bind socket to the specified address and port
    if (bind(gz_sim_send_socket_fd, (const struct sockaddr *)&controller_sim_recv_addr, sizeof(controller_sim_recv_addr)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Set socket to non-blocking mode
    int flags = fcntl(gz_sim_send_socket_fd, F_GETFL, 0);
    fcntl(gz_sim_send_socket_fd, F_SETFL, flags | O_NONBLOCK);

    // Bind socket to the specified address and port
    if (bind(rover_tx_socket_fd, (const struct sockaddr *)&rover_tx_addr, sizeof(rover_tx_addr)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Set socket to non-blocking mode
    flags = fcntl(rover_tx_socket_fd, F_GETFL, 0);
    fcntl(rover_tx_socket_fd, F_SETFL, flags | O_NONBLOCK);

    int counter = 0;
    while (1)
    {
        read_udp_data();
        usleep(1000);
    }
    return 0;
}