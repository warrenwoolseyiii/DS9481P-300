cmake_minimum_required(VERSION 3.10)
project(c_i2c_scan C)

set(CMAKE_C_STANDARD 11)

# Add the main library's include directory
include_directories(../../include)

# Add the directory containing the library to the linker's search path
link_directories(../../build)

# Add the executable
add_executable(c_i2c_scan c_i2c_scan.c)

# Link against the ds9481p library by name
target_link_libraries(c_i2c_scan ds9481p)