cmake_minimum_required(VERSION 3.10)
project(ds9481p C)

set(CMAKE_C_STANDARD 11)

# Define the library
add_library(ds9481p SHARED src/ds9481p.c)

# Public include directories
target_include_directories(ds9481p PUBLIC 
    "${PROJECT_BINARY_DIR}"
    "${PROJECT_SOURCE_DIR}/include"
)

# Set properties for the shared library
set_target_properties(ds9481p PROPERTIES
    VERSION 1.0.0
    SOVERSION 1
    PUBLIC_HEADER "include/ds9481p.h"
)

# Installation
install(TARGETS ds9481p
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)