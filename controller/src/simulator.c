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
#include "interface.h"

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

struct sockaddr_in controller_sim_recv_addr, gz_sim_recv_addr, rover_tx_addr, rover_rx_addr;

void controller_write_fn(uint8_t *data, uint16_t len)
{
    sendto(rover_rx_socket_fd, data, len, 0, (const struct sockaddr *)&rover_rx_addr, sizeof(rover_rx_addr));
}

size_t controller_read_fn(uint8_t *buffer, size_t buffer_len)
{
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    size_t bytes_available = 0;
    ioctl(rover_tx_socket_fd, FIONREAD, &bytes_available);

    if (bytes_available > 0)
    {
        return recvfrom(rover_tx_socket_fd, buffer, buffer_len, 0, (struct sockaddr *)&client_addr, &addr_len);
    }
    else {
        return 0;
    }
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
        }
    }

}

int main(int argc, char *argv[])
{
    char *gz_recv_ip = getenv("GZ_SIM_RX_IP");
    printf("GZ_SIM_RX_IP: %s\n", gz_recv_ip);

    char *gz_recv_port = getenv("GZ_SIM_RX_PORT");
    printf("GZ_SIM_RX_PORT: %s\n", gz_recv_port);
    int gz_recv_port_int = atoi(gz_recv_port);

    char *controller_sim_recv_port = getenv("GZ_SIM_TX_PORT");
    printf("GZ_SIM_TX_PORT: %s\n", controller_sim_recv_port);
    int controller_sim_recv_port_int = atoi(controller_sim_recv_port);

    int rover_tx_port_int = atoi(getenv("CONTROLLER_RX_PORT"));
    printf("CONTROLLER_RX_PORT: %d\n", rover_tx_port_int);

    int rover_rx_port_int = atoi(getenv("CONTROLLER_TX_PORT"));
    printf("CONTROLLER_TX_PORT: %d\n", rover_rx_port_int);

    char *rover_rx_ip = getenv("CONTROLLER_TX_IP");
    printf("ROVER_RX_IP: %s\n", rover_rx_ip);

    char *rover_tx_ip = getenv("CONTROLLER_RX_IP");
    printf("ROVER_TX_IP: %s\n", rover_tx_ip);

    /* Initialize the controller */
    controller_init(&controller_read_fn, &controller_write_fn);
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
    rover_tx_addr.sin_port = htons(rover_tx_port_int);

    memset(&rover_rx_addr, 0, sizeof(rover_rx_addr));
    rover_rx_addr.sin_family = AF_INET;
    rover_rx_addr.sin_port = htons(rover_rx_port_int);

    memset(&gz_sim_recv_addr, 0, sizeof(gz_sim_recv_addr));
    gz_sim_recv_addr.sin_family = AF_INET;
    gz_sim_recv_addr.sin_port = htons(gz_recv_port_int);

    if (inet_pton(AF_INET, gz_recv_ip, &(gz_sim_recv_addr.sin_addr)) <= 0)
    {
        perror("invalid address");
        exit(EXIT_FAILURE);
    }

    if (inet_pton(AF_INET, rover_rx_ip, &(rover_rx_addr.sin_addr)) <= 0)
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