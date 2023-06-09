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

int controller_sim_recv_socket_fd;
int gz_sim_recv_socket_fd;
int log_recv_socket_fd;

struct sockaddr_in controller_sim_recv_addr, gz_sim_recv_addr, log_recv_addr;

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

    const char *json_string = json_object_to_json_string(json_obj);

    if (sendto(log_recv_socket_fd, json_string, strlen(json_string), 0, (const struct sockaddr *)&log_recv_addr, sizeof(log_recv_addr)) < 0)
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

    // Check if data is available
    int bytes_available;
    ioctl(controller_sim_recv_socket_fd, FIONREAD, &bytes_available);

    if (bytes_available > 0)
    {
        ssize_t recv_len = recvfrom(controller_sim_recv_socket_fd, (char *)&sim_sensors, sizeof(sim_api_sensor_data_t), 0, (struct sockaddr *)&client_addr, &addr_len);
        if (sizeof(sim_api_sensor_data_t) == recv_len)
        {
            controller_actuators_t actuators;
            sensors.linear_acceleration_x = sim_sensors.linear_acceleration_x;
            sensors.linear_acceleration_y = sim_sensors.linear_acceleration_y;
            sensors.linear_acceleration_z = sim_sensors.linear_acceleration_z;

            sensors.angular_velocity_x = sim_sensors.angular_velocity_x;
            sensors.angular_velocity_y = sim_sensors.angular_velocity_y;
            sensors.angular_velocity_z = sim_sensors.angular_velocity_z;

            controller_process(&actuators, &sensors);

            sim_actuators.left = actuators.left * 20;
            sim_actuators.right = actuators.right * 20;

            sendto(gz_sim_recv_socket_fd, (const char *)&sim_actuators, sizeof(sim_api_actuator_data_t), 0, (const struct sockaddr *)&gz_sim_recv_addr, sizeof(gz_sim_recv_addr));

            send_log_data(&actuators, &sensors);
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

    /* Initialize the controller */
    controller_init();
    if ((controller_sim_recv_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    if ((gz_sim_recv_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    if ((log_recv_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&controller_sim_recv_addr, 0, sizeof(controller_sim_recv_addr));
    controller_sim_recv_addr.sin_family = AF_INET;
    controller_sim_recv_addr.sin_addr.s_addr = INADDR_ANY;
    controller_sim_recv_addr.sin_port = htons(controller_sim_recv_port_int);

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
    if (bind(controller_sim_recv_socket_fd, (const struct sockaddr *)&controller_sim_recv_addr, sizeof(controller_sim_recv_addr)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Set socket to non-blocking mode
    int flags = fcntl(controller_sim_recv_socket_fd, F_GETFL, 0);
    fcntl(controller_sim_recv_socket_fd, F_SETFL, flags | O_NONBLOCK);

    int counter = 0;
    while (1)
    {
        read_udp_data();
        usleep(1000);
    }
    return 0;
}