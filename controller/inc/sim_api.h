#ifndef SIM_API
#define SIM_API

typedef struct {
    float linear_acceleration_x;
    float linear_acceleration_y;
    float linear_acceleration_z;

    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
} sim_api_sensor_data_t;

typedef struct {
    float left;
    float right;
} sim_api_actuator_data_t;

#endif /* SIM_API */
