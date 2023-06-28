cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(OliRoverController)


# Add compiler_commands.json generation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set a variable to control whether to build with simulation
option(BUILD_WITH_SIMULATION "Build with simulation" OFF)

# Define the source files for your project
set(SOURCES
    src/controller.c
    src/cbwo.c
)

# Define the include directories for your project
include_directories(
    inc
    ../interface
)

# Create an executable target for the simulation code
if (BUILD_WITH_SIMULATION)
    find_package(json-c REQUIRED)
    add_executable(OliRoverControllerSimulator
        src/simulator.c
        ${SOURCES}
    )
    target_link_libraries(OliRoverControllerSimulator
        json-c)
    message("Building simulation build.")
else()

    # initialize the Raspberry Pi Pico SDK
    pico_sdk_init()

    set(Libs
        pico_stdlib
        )

    add_executable(OliRoverController 
        src/main.c
        ${SOURCES})
    target_link_libraries(OliRoverController ${Libs})
    pico_add_extra_outputs(OliRoverController)
    pico_set_linker_script(OliRoverController ${CMAKE_CURRENT_SOURCE_DIR}/app.ld)

    message("Building normal build.")
endif()

# Pass -DBUILD_WITH_SIMULATION=ON to cmake to build with simulation
