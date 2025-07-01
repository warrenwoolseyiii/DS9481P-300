cmake_minimum_required(VERSION 3.10)
project(c_i2c_scan C)

set(CMAKE_C_STANDARD 11)

# Add the main library's include directory
include_directories(../../include)

# Add the executable
add_executable(c_i2c_scan c_i2c_scan.c)

# Link against the ds9481p library
# This assumes the main library has been built in the default `build` directory
target_link_libraries(c_i2c_scan ../../build/libds9481p.dylib)