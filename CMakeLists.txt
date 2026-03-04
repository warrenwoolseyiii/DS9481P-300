cmake_minimum_required(VERSION 3.10)
project(ds9481p C)

set(CMAKE_C_STANDARD 11)

# Optional debug tracing — pass -DDS9481P_ENABLE_DEBUG=ON to cmake
option(DS9481P_ENABLE_DEBUG "Enable verbose stderr debug tracing" OFF)

# Define the library
add_library(ds9481p SHARED src/ds9481p.c)

if(DS9481P_ENABLE_DEBUG)
    target_compile_definitions(ds9481p PRIVATE DS9481P_DEBUG)
    message(STATUS "DS9481P debug tracing ENABLED")
endif()

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