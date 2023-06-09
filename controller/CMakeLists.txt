cmake_minimum_required(VERSION 3.0)
project(OliRoverController)

find_package(json-c REQUIRED)

# Add compiler_commands.json generation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set a variable to control whether to build with simulation
option(BUILD_WITH_SIMULATION "Build with simulation" OFF)

# Define the source files for your project
set(SOURCES
    src/controller.c
)

# Define the include directories for your project
include_directories(
    inc
)

# Create an executable target for the simulation code
if (BUILD_WITH_SIMULATION)
    add_executable(OliRoverControllerSimulator
        src/simulator.c
        ${SOURCES}
    )
    target_link_libraries(OliRoverControllerSimulator
        json-c)
    message("Building simulation build.")
else()
    add_executable(OliRoverController
        ${SOURCES}
    )
    message("Building normal build.")
endif()

# Pass -DBUILD_WITH_SIMULATION=ON to cmake to build with simulation
