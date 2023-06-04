cmake_minimum_required(VERSION 3.0)
project(YourProject)

# Add compiler_commands.json generation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set a variable to control whether to build with simulation
option(BUILD_WITH_SIMULATION "Build with simulation" OFF)

# Define the source files for your project
set(SOURCES
    src/main.cpp
    src/production_code.cpp
    src/mockup_code.cpp
)

# Define the include directories for your project
include_directories(
    inc
)

# Create an executable target for the production code
add_executable(YourProject ${SOURCES})

# Create an executable target for the simulation code
if (BUILD_WITH_SIMULATION)
    add_executable(YourProjectSimulation
        ${SOURCES}
        src/simulation_code.cpp
    )
else()
    message("Building without simulation.")
endif()

# Pass -DBUILD_WITH_SIMULATION=ON to cmake to build with simulation
