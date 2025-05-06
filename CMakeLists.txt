cmake_minimum_required(VERSION 3.31)
project(VOC_breadboard C)

set(CMAKE_C_STANDARD 11)


# Create a static library from sensor drivers
add_library(sensor_drivers
        sensirion_common.c
        sensirion_i2c.c
        sensirion_i2c_hal.c
        sgp40_i2c.c
        sht3x_i2c.c
)


add_executable(VOC_breadboard main.c)

# Link the driver library to your executable
target_link_libraries(VOC_breadboard sensor_drivers)

# Include the header paths if they're in a subfolder
target_include_directories(VOC_breadboard PRIVATE ${CMAKE_SOURCE_DIR})