#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "controller.h"

#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_BAUD   921600

void controller_write_fn(uint8_t *data, uint16_t len)
{
    uart_write_blocking(uart0, data, len);
}

size_t controller_read_fn(uint8_t *buffer, size_t buffer_len)
{
    uint32_t bytes_read = 0;
    /* Check if there is any UART data availible */
    if (uart_is_readable(uart0) == false)
    {
        goto exit;
    }

    while (uart_is_readable(uart0) && bytes_read < buffer_len)
    {
        /* Read one byte */
        uint8_t byte;
        uart_read_blocking(uart0, &byte, 1u);
        /* Add the byte to the buffer */
        buffer[bytes_read] = byte;
        bytes_read++;
    }

exit:
    return bytes_read;
}

int main() {

	uart_init(uart0, UART_BAUD);
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	uart_set_hw_flow(uart0, false, false);


    controller_init(controller_read_fn, controller_write_fn);

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    controller_actuators_t actuators;

    controller_sensors_t sensors;
    sensors.linear_acceleration_x = 1.0f;
    sensors.linear_acceleration_y = 2.0f;
    sensors.linear_acceleration_z = 3.0f;

    sensors.angular_velocity_x = 4.0f;
    sensors.angular_velocity_y = 5.0f;
    sensors.angular_velocity_z = 6.0f;

    while (true) {
        controller_process(&actuators, &sensors);
        /* Rover/controller communication 50ms or slower */
        sleep_ms(50);
    }
}