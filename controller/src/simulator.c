#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "controller.h"
#include "sim_api.h"

#define PORT 1338
#define BUF_SIZE 1024

#define CLIENT_IP "192.168.122.131"
#define CLIENT_PORT 1337

int server_socket_fd;
int client_socket_fd;
char buffer[BUF_SIZE];

struct sockaddr_in server_addr, dest_addr;

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
    ioctl(server_socket_fd, FIONREAD, &bytes_available);

    if (bytes_available > 0)
    {
        ssize_t recv_len = recvfrom(server_socket_fd, (char *)&sim_sensors, sizeof(sim_api_sensor_data_t), 0, (struct sockaddr *)&client_addr, &addr_len);
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

            sim_actuators.left = actuators.left*20;
            sim_actuators.right = actuators.right*20;

            sendto(client_socket_fd, (const char *)&sim_actuators, sizeof(sim_api_actuator_data_t), 0, (const struct sockaddr *)&dest_addr, sizeof(dest_addr));
        }
    }
}

int main(int argc, char *argv[])
{
    /* Initialize the controller */
    controller_init();
    if ((server_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    if ((client_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Configure server address
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    // Configure destination address
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(CLIENT_PORT);
    if (inet_pton(AF_INET, CLIENT_IP, &(dest_addr.sin_addr)) <= 0)
    {
        perror("invalid address");
        exit(EXIT_FAILURE);
    }

    // Bind socket to the specified address and port
    if (bind(server_socket_fd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Set socket to non-blocking mode
    int flags = fcntl(server_socket_fd, F_GETFL, 0);
    fcntl(server_socket_fd, F_SETFL, flags | O_NONBLOCK);

    int counter = 0;
    while (1)
    {
        read_udp_data();
        usleep(1000);
    }
    return 0;
}