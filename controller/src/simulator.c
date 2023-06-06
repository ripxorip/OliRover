#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "controller.h"

#define PORT 1338
#define BUF_SIZE 1024

#define CLIENT_IP "192.168.122.131"
#define CLIENT_PORT 1337

int server_socket_fd;
int client_socket_fd;
char buffer[BUF_SIZE];

void read_udp_data()
{
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    // Check if data is available
    int bytes_available;
    ioctl(server_socket_fd, FIONREAD, &bytes_available);

    if (bytes_available > 0)
    {
        // Receive message
        ssize_t recv_len = recvfrom(server_socket_fd, (char *)buffer, BUF_SIZE, 0, (struct sockaddr *)&client_addr, &addr_len);
        buffer[recv_len] = '\0';
        printf("Received: %s\n", buffer);
    }
}

int main(int argc, char *argv[])
{
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

    struct sockaddr_in server_addr, dest_addr;

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
        counter++;
        if (counter % 1000 == 0)
        {
            // Send message
            const char* message = "Hello from controller";
            sendto(client_socket_fd, (const char *)message, strlen(message), 0, (const struct sockaddr *)&dest_addr, sizeof(dest_addr));
        }
        usleep(1000);
    }
    return 0;
}