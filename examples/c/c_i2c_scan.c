//MIT License
//
//Copyright (c) 2025 budgettsfrog@protonmail.com
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "ds9481p.h"

void scan_i2c_bus(ds9481p_device_handle handle) {
    printf("Scanning for I2C devices (addresses 1-127)...\n");
    int found_count = 0;

    for (int address = 1; address < 128; ++address) {
        if (ds9481p_i2c_start(handle) != 0) {
            fprintf(stderr, "Failed to send I2C start\n");
            continue;
        }

        // Set slave address with write bit (0)
        // A non-zero return value from write indicates an error (e.g., NACK)
        if (ds9481p_i2c_write_byte(handle, (address << 1) | 0) == 0) {
            if (found_count == 0) {
                 printf("\nFound I2C devices at the following addresses:\n");
            }
            printf(" - 0x%02X (%d)\n", address, address);
            found_count++;
        }

        if (ds9481p_i2c_stop(handle) != 0) {
            fprintf(stderr, "Failed to send I2C stop\n");
        }
    }

    if (found_count == 0) {
        printf("\nNo I2C devices found.\n");
    }
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Error: Please provide the serial port name as an argument.\n");
        fprintf(stderr, "Usage: %s /dev/tty.usbmodemXXXX\n", argv[0]);
        fprintf(stderr, "You can find the port name by running `ls /dev/tty.*` in your terminal.\n");
        return 1;
    }

    const char* port_name = argv[1];
    ds9481p_device_handle handle = ds9481p_open(port_name);

    if (!handle) {
        fprintf(stderr, "Failed to open device at %s\n", port_name);
        return 1;
    }

    printf("Successfully connected to device on %s\n", port_name);

    if (ds9481p_enter_i2c_mode(handle) != 0) {
        fprintf(stderr, "Failed to switch to I2C mode.\n");
        ds9481p_close(handle);
        return 1;
    }
    
    printf("Switched to I2C mode.\n");
    usleep(100000); // 100ms delay

    int major, minor;
    if (ds9481p_get_version(handle, &major, &minor) == 0) {
        printf("Device Firmware Version: %d.%d\n", major, minor);
    } else {
        fprintf(stderr, "Failed to get device version.\n");
    }

    scan_i2c_bus(handle);

    ds9481p_close(handle);
    printf("Device connection closed.\n");

    return 0;
}