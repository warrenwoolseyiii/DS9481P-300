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

#include "../include/ds9481p.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

/**
 * @brief Internal device structure.
 */
struct ds9481p_device_t {
    int fd; // File descriptor for the serial port
};

DS9481P_API ds9481p_device_handle ds9481p_open(const char* port_name) {
    if (!port_name) {
        return NULL;
    }

    int fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("open_port: Unable to open port");
        return NULL;
    }

    struct termios options;
    tcgetattr(fd, &options);

    // Set baud rate
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // Set options
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10; // 1 second timeout

    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        perror("init_serial_port: Couldn't set term attributes");
        close(fd);
        return NULL;
    }

    ds9481p_device_handle handle = malloc(sizeof(struct ds9481p_device_t));
    if (!handle) {
        close(fd);
        return NULL;
    }

    handle->fd = fd;
    return handle;
}

DS9481P_API void ds9481p_close(ds9481p_device_handle handle) {
    if (handle) {
        if (handle->fd != -1) {
            close(handle->fd);
        }
        free(handle);
    }
}

DS9481P_API int ds9481p_enter_i2c_mode(ds9481p_device_handle handle) {
    if (!handle) {
        return -1;
    }

    // Reset adapter
    if (ds9481p_reset_adapter(handle) != 0) {
        perror("ds9481p_enter_i2c_mode: failed to reset adapter");
        return -1;
    }

    char cmd = 0xE5; // Enter I2C mode command
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_enter_i2c_mode: write failed");
        return -1;
    }

    // Wait for the device to be ready
    usleep(100000); // 100ms

    return 0;
}

DS9481P_API int ds9481p_i2c_start(ds9481p_device_handle handle) {
    if (!handle) return -1;
    char cmd = 0x53; // I2C start command
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_i2c_start: write failed");
        return -1;
    }
    return 0;
}

DS9481P_API int ds9481p_i2c_stop(ds9481p_device_handle handle) {
    if (!handle) return -1;
    char cmd = 0x50; // I2C stop command
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_i2c_stop: write failed");
        return -1;
    }
    return 0;
}

DS9481P_API int ds9481p_i2c_write_byte(ds9481p_device_handle handle, unsigned char byte) {
    if (!handle) return -1;
    char cmd[] = { 0x57, byte }; // I2C write byte command
    if (write(handle->fd, cmd, sizeof(cmd)) != sizeof(cmd)) {
        perror("ds9481p_i2c_write_byte: write failed");
        return -1;
    }
    unsigned char status = 0;
    if(ds9481p_read_status(handle, &status) != 0) {
        perror("ds9481p_i2c_write_byte: read status failed");
        return -1;
    }
    if(status != 0) {
        perror("ds9481p_i2c_write_byte: I2C write error");
    }
    return (status == 0) ? 0 : -1;
}

static int read_byte_with_command(ds9481p_device_handle handle, char cmd, unsigned char* byte) {
    if (!handle || !byte) return -1;
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("read_byte_with_command: write failed");
        return -1;
    }
    // Add a short delay to give the device time to respond
    usleep(10000); // 10ms
    if (read(handle->fd, byte, 1) != 1) {
        perror("read_byte_with_command: read failed");
        return -1;
    }
    return 0;
}

DS9481P_API int ds9481p_i2c_read_byte_ack(ds9481p_device_handle handle, unsigned char* byte) {
    return read_byte_with_command(handle, 0x52, byte); // I2C read byte with ACK
}

DS9481P_API int ds9481p_i2c_read_byte_nack(ds9481p_device_handle handle, unsigned char* byte) {
    return read_byte_with_command(handle, 0x4E, byte); // I2C read byte with NACK
}

DS9481P_API int ds9481p_get_version(ds9481p_device_handle handle, int* major, int* minor) {
    if (!handle || !major || !minor) {
        return -1;
    }

    unsigned char version_byte;
    if (read_byte_with_command(handle, 0x56, &version_byte) != 0) { // Get version command
        perror("ds9481p_get_version: failed to read version");
        return -1;
    }

    *major = (version_byte >> 4) & 0x0F;
    *minor = version_byte & 0x0F;

    return 0;
}

DS9481P_API int ds9481p_reset_adapter(ds9481p_device_handle handle) {
    if(!handle) {
        return -1;
    }

    char cmd = 0xC1; // Reset adapter command
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_reset_adapter: write failed");
        return -1;
    }

    // Wait for the device to be ready
    usleep(100000); // 100ms

    return 0;
}

DS9481P_API int ds9481p_read_status(ds9481p_device_handle handle, unsigned char* status) {
    return read_byte_with_command(handle, 0x48, status); // Read status command
}

DS9481P_API int ds9481p_enter_1wire_mode(ds9481p_device_handle handle) {
    if (!handle) {
        return -1;
    }

    // Return to 1-Wire mode
    char commands[] = { 0x43, 0x4F };
    if (write(handle->fd, commands, sizeof(commands)) != sizeof(commands)) {
        perror("ds9481p_enter_1wire_mode: write failed");
        return -1;
    }

    return 0;
}