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

#ifdef DS9481P_DEBUG
  #define DBG(fmt, ...) fprintf(stderr, "[DS9481P] " fmt "\n", ##__VA_ARGS__)
#else
  #define DBG(fmt, ...) ((void)0)
#endif

// Command definitions
static const uint8_t CMD_RESET_ADAPTER = 0xC1;
static const uint8_t CMD_ENTER_I2C_MODE = 0xE5;
static const uint8_t CMD_START = 0x53; // 'S'
static const uint8_t CMD_STOP = 0x50; // 'P'
static const uint8_t CMD_REPEATED_START = 0x54; // 'T'
static const uint8_t CMD_WRITE_BYTE = 0x57; // 'W'
static const uint8_t CMD_WRITE_BYTE_STATUS = 0x51; // 'Q'
static const uint8_t CMD_READ_BYTE_ACK = 0x52; // 'R'
static const uint8_t CMD_READ_BYTE_NACK = 0x4E; // 'N'
static const uint8_t CMD_READ_STATUS = 0x48; // 'H'
static const uint8_t CMD_READ_VERSION = 0x56; // 'V'
static const uint8_t CMD_SET_MODE = 0x4D; // 'M'
static const uint8_t CMD_RETURN_TO_1_WIRE_MODE[] = {0x43, 0x4F}; // 'CO'
static const uint8_t CMD_PACKETIZED_DATA = 0x5A; // 'Z'
static const unsigned int I2C_DELAY_US = 10000;

/**
 * @brief Macro to print the command and function name
 */
#define PRINT_CMD_FUN(cmd) DBG("%s: sending 0x%02X ('%c')", __func__, cmd, cmd)

/**
 * @brief Internal device structure.
 */
struct ds9481p_device_t {
    int fd; // File descriptor for the serial port
};

/**
 * @brief Internal record of the port name
 */
char _port_name[256];

/**
 * @brief Wrapper for reading from the device with a timeout.
 * @param handle The device handle.
 * @param buffer The buffer to store the read data.
 * @param length The number of bytes to read.
 * @param timeout_us The timeout in microseconds.
 * @return number of bytes read on success, -1 on failure.
 */
DS9481P_API static int _read_wrapper(ds9481p_device_handle handle, uint8_t* buffer, size_t length, unsigned int timeout_us) {
    int len = read(handle->fd, buffer, length);
    if (len != (int)length) {
        if (len == -1) len = 0;
        usleep(timeout_us);
        int len2 = read(handle->fd, buffer + len, length - len);
        if (len2 == -1) len2 = 0;
        len += len2;
    }
    return len;
}

DS9481P_API ds9481p_device_handle ds9481p_open(const char* port_name) {
    if (!port_name) {
        DBG("open: port_name is NULL");
        return NULL;
    }
    
    memset(_port_name, 0, sizeof(_port_name));
    strncpy(_port_name, port_name, sizeof(_port_name) - 1);
    DBG("open: opening %s", _port_name);

    int fd = open(_port_name, O_RDWR | O_NOCTTY | O_NDELAY);
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
    options.c_cc[VTIME] = 0;

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
    DBG("open: fd=%d, VMIN=0, VTIME=10 (1s timeout)", fd);

    // Flush the TX and RX buffers
    tcflush(fd, TCIOFLUSH);

    return handle;
}

DS9481P_API void ds9481p_close(ds9481p_device_handle handle) {
    DBG("close: handle=%p", (void*)handle);
    if (handle) {
        if (handle->fd != -1) {
            close(handle->fd);
        }
        free(handle);
    }
}

DS9481P_API int ds9481p_enter_i2c_mode(ds9481p_device_handle handle) {
    if (!handle) {
        DBG("enter_i2c_mode: handle is NULL");
        return -1;
    }
    DBG("enter_i2c_mode: resetting adapter...");

    // Reset adapter
    if (ds9481p_reset_adapter(handle) != 0) {
        perror("ds9481p_enter_i2c_mode: failed to reset adapter");
        return -1;
    }

    uint8_t cmd = CMD_ENTER_I2C_MODE;
    PRINT_CMD_FUN(cmd);
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_enter_i2c_mode: write failed");
        return -1;
    }

    // Wait for the device to be ready
    usleep(100000); // 100ms
    DBG("enter_i2c_mode: done, waited 100ms");

    return 0;
}

DS9481P_API int ds9481p_i2c_start(ds9481p_device_handle handle) {
    if (!handle) return -1;
    uint8_t cmd = CMD_START;
    PRINT_CMD_FUN(cmd);
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_i2c_start: write failed");
        return -1;
    }
    return 0;
}

DS9481P_API int ds9481p_i2c_repeated_start(ds9481p_device_handle handle) {
    if (!handle) return -1;
    uint8_t cmd = CMD_REPEATED_START;
    PRINT_CMD_FUN(cmd);
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_i2c_repeated_start: write failed");
        return -1;
    }
    return 0;
}

DS9481P_API int ds9481p_i2c_stop(ds9481p_device_handle handle) {
    if (!handle) return -1;
    uint8_t cmd = CMD_STOP;
    PRINT_CMD_FUN(cmd);
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_i2c_stop: write failed");
        return -1;
    }
    return 0;
}

DS9481P_API int ds9481p_i2c_write_byte(ds9481p_device_handle handle, unsigned char byte) {
    if (!handle) return -1;
    uint8_t cmd[] = {CMD_WRITE_BYTE, byte};
    PRINT_CMD_FUN(cmd[0]);
    DBG("i2c_write_byte: sending byte 0x%02X ('%c')", byte, byte);
    if (write(handle->fd, cmd, sizeof(cmd)) != sizeof(cmd)) {
        perror("ds9481p_i2c_write_byte: write failed");
        return -1;
    }
    return 0;
}

DS9481P_API int ds9481p_i2c_write_byte_status(ds9481p_device_handle handle, unsigned char byte, unsigned char* status) {
    if(!handle || !status) return -1;
    
    // Flush the read buffer so we don't get stale content
    tcflush(handle->fd, TCIFLUSH);

    uint8_t cmd[] = {CMD_WRITE_BYTE_STATUS, byte};
    PRINT_CMD_FUN(cmd[0]);
    DBG("i2c_write_byte_status: sending byte 0x%02X ('%c')", byte, byte);
    if (write(handle->fd, cmd, sizeof(cmd)) != sizeof(cmd)) {
        perror("ds9481p_i2c_write_byte_status: write failed");
        return -1;
    }

    if (_read_wrapper(handle, status, 1, I2C_DELAY_US) != 1) {
        perror("ds9481p_i2c_write_byte_status: read status failed");
        return -1;
    }

    DBG("i2c_write_byte_status: status=0x%02X", *status);

    return 0;
}

DS9481P_API int ds9481p_i2c_read_byte_ack(ds9481p_device_handle handle, unsigned char* byte) {
    if (!handle || !byte) return -1;

    // Flush the read buffer so we don't get stale content
    tcflush(handle->fd, TCIFLUSH);
    
    uint8_t cmd = CMD_READ_BYTE_ACK;
    PRINT_CMD_FUN(cmd);
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_i2c_read_byte_ack: write failed");
        return -1;
    }
    if (_read_wrapper(handle, byte, 1, I2C_DELAY_US) != 1) {
        perror("ds9481p_i2c_read_byte_ack: read failed");
        return -1;
    }
    DBG("i2c_read_byte_ack: got 0x%02X", *byte);
    return 0;
}

DS9481P_API int ds9481p_i2c_read_byte_nack(ds9481p_device_handle handle, unsigned char* byte) {
    if (!handle || !byte) return -1;

    // Flush the read buffer so we don't get stale content
    tcflush(handle->fd, TCIFLUSH);
    
    uint8_t cmd = CMD_READ_BYTE_NACK;
    PRINT_CMD_FUN(cmd);
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_i2c_read_byte_nack: write failed");
        return -1;
    }
    if (_read_wrapper(handle, byte, 1, I2C_DELAY_US) != 1) {
        perror("ds9481p_i2c_read_byte_nack: read failed");
        return -1;
    }
    DBG("i2c_read_byte_nack: got 0x%02X", *byte);
    return 0;
}

DS9481P_API int ds9481p_get_version(ds9481p_device_handle handle, int* major, int* minor) {
    if (!handle || !major || !minor) return -1;
    
    // Flush the read buffer so we don't get stale content
    tcflush(handle->fd, TCIFLUSH);

    uint8_t cmd = CMD_READ_VERSION;
    PRINT_CMD_FUN(cmd);
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_get_version: write failed");
        return -1;
    }
    uint8_t version_byte;
    if (_read_wrapper(handle, &version_byte, 1, I2C_DELAY_US) != 1) {
        perror("ds9481p_get_version: read failed");
        return -1;
    }

    *major = (version_byte >> 4) & 0x0F;
    *minor = version_byte & 0x0F;
    DBG("get_version: v%d.%d (raw=0x%02X)", *major, *minor, version_byte);

    return 0;
}

DS9481P_API int ds9481p_reset_adapter(ds9481p_device_handle handle) {
    if (!handle) return -1;
    uint8_t cmd = CMD_RESET_ADAPTER;
    PRINT_CMD_FUN(cmd);
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_reset_adapter: write failed");
        return -1;
    }

    // Wait for the device to be ready
    usleep(100000); // 100ms
    
    // Free the handle (according to the data sheet)
    ds9481p_close(handle);

    // Reopen the port
    char local_pname[256] = {0};
    strncpy(local_pname, _port_name, sizeof(local_pname) - 1);
    handle = ds9481p_open(local_pname);
    if (!handle) {
        fprintf(stderr, "Failed to reopen device after reset.\n");
        return -1;
    }

    return 0;
}

DS9481P_API int ds9481p_read_status(ds9481p_device_handle handle, unsigned char* status) {
    if (!handle || !status) return -1;

    // Flush the read buffer so we don't get stale content
    tcflush(handle->fd, TCIFLUSH);

    uint8_t cmd = CMD_READ_STATUS;
    PRINT_CMD_FUN(cmd);
    if (write(handle->fd, &cmd, 1) != 1) {
        perror("ds9481p_read_status: write failed");
        return -1;
    }
    if (_read_wrapper(handle, status, 1, I2C_DELAY_US) != 1) {
        perror("ds9481p_read_status: read failed");
        return -1;
    }
    DBG("read_status: status=0x%02X", *status);
    return 0;
}

DS9481P_API int ds9481p_enter_1wire_mode(ds9481p_device_handle handle) {
    if (!handle) return -1;
    uint8_t cmd[] = { CMD_RETURN_TO_1_WIRE_MODE[0], CMD_RETURN_TO_1_WIRE_MODE[1] };
    PRINT_CMD_FUN(cmd[0]);
    PRINT_CMD_FUN(cmd[1]);
    if (write(handle->fd, cmd, sizeof(cmd)) != sizeof(cmd)) {
        perror("ds9481p_enter_1wire_mode: write failed");
        return -1;
    }
    DBG("enter_1wire_mode: done");
    return 0;
}