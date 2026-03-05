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

// STHS34PF80 constants
#define STHS34PF80_ADDR       0x5A
#define WHO_AM_I_REG          0x0F
#define WHO_AM_I_EXPECTED     0xD3

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
        uint8_t status = 0xFF;
        if (ds9481p_i2c_write_byte_status(handle, (address << 1) | 0, &status) == 0 && status == 0) {
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

void read_who_am_i(ds9481p_device_handle handle) {
    printf("\nReading STHS34PF80 WHO_AM_I register (0x%02X)...\n", WHO_AM_I_REG);

    uint8_t status;

    // START
    if (ds9481p_i2c_start(handle) != 0) {
        fprintf(stderr, "  WHO_AM_I: START failed\n");
        return;
    }

    // Write slave address (write mode)
    if (ds9481p_i2c_write_byte_status(handle, (STHS34PF80_ADDR << 1) | 0, &status) != 0 || status != 0) {
        fprintf(stderr, "  WHO_AM_I: addr+W NACK (status=0x%02X)\n", status);
        ds9481p_i2c_stop(handle);
        return;
    }

    // Write register address
    if (ds9481p_i2c_write_byte_status(handle, WHO_AM_I_REG, &status) != 0 || status != 0) {
        fprintf(stderr, "  WHO_AM_I: reg addr NACK (status=0x%02X)\n", status);
        ds9481p_i2c_stop(handle);
        return;
    }

    // Repeated START
    if (ds9481p_i2c_repeated_start(handle) != 0) {
        fprintf(stderr, "  WHO_AM_I: repeated START failed\n");
        ds9481p_i2c_stop(handle);
        return;
    }

    // Write slave address (read mode)
    if (ds9481p_i2c_write_byte_status(handle, (STHS34PF80_ADDR << 1) | 1, &status) != 0 || status != 0) {
        fprintf(stderr, "  WHO_AM_I: addr+R NACK (status=0x%02X)\n", status);
        ds9481p_i2c_stop(handle);
        return;
    }

    // Read one byte with NACK (last/only byte)
    uint8_t who_am_i = 0;
    if (ds9481p_i2c_read_byte_nack(handle, &who_am_i) != 0) {
        fprintf(stderr, "  WHO_AM_I: read failed\n");
        ds9481p_i2c_stop(handle);
        return;
    }

    // STOP
    ds9481p_i2c_stop(handle);

    // Report result
    if (who_am_i == WHO_AM_I_EXPECTED) {
        printf("  WHO_AM_I = 0x%02X ✓ (matches STHS34PF80)\n", who_am_i);
    } else {
        printf("  WHO_AM_I = 0x%02X ✗ (expected 0x%02X)\n", who_am_i, WHO_AM_I_EXPECTED);
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

    for(int i = 0; i < 2; i++) {
        int major, minor;
        if (ds9481p_get_version(handle, &major, &minor) == 0) {
            printf("Device Firmware Version: %d.%d\n", major, minor);
        } else {
            fprintf(stderr, "Failed to get device version.\n");
        }
    }

    scan_i2c_bus(handle);

    // TODO: This is sensor specific test code
    // for(int i = 0; i < 1000; i++)
    //     read_who_am_i(handle);

    for(int i = 0; i < 2; i++) {
        int major, minor;
        if (ds9481p_get_version(handle, &major, &minor) == 0) {
            printf("Device Firmware Version: %d.%d\n", major, minor);
        } else {
            fprintf(stderr, "Failed to get device version.\n");
        }
    }

    ds9481p_close(handle);
    printf("Device connection closed.\n");

    return 0;
}