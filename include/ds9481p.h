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

#ifndef DS9481P_H
#define DS9481P_H

#if defined(_WIN32)
    #ifdef DS9481P_EXPORTS
        #define DS9481P_API __declspec(dllexport)
    #else
        #define DS9481P_API __declspec(dllimport)
    #endif
#else
    #define DS9481P_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to a DS9481P-300 device.
 */
typedef struct ds9481p_device_t* ds9481p_device_handle;

/**
 * @brief Opens a connection to the DS9481P-300 device.
 *
 * @param port_name The name of the serial port (e.g., "/dev/tty.usbmodem14201").
 * @return A handle to the device, or NULL on failure.
 */
DS9481P_API ds9481p_device_handle ds9481p_open(const char* port_name);

/**
 * @brief Closes the connection to the device.
 *
 * @param handle A valid device handle.
 */
DS9481P_API void ds9481p_close(ds9481p_device_handle handle);

/**
 * @brief Switches the device to I2C mode.
 *
 * @param handle A valid device handle.
 * @return 0 on success, -1 on failure.
 */
DS9481P_API int ds9481p_enter_i2c_mode(ds9481p_device_handle handle);

/**
 * @brief Switches the device to 1-Wire mode.
 *
 * @param handle A valid device handle.
 * @return 0 on success, -1 on failure.
 */
DS9481P_API int ds9481p_enter_1wire_mode(ds9481p_device_handle handle);

/**
 * @brief Sends an I2C start condition.
 *
 * @param handle A valid device handle.
 * @return 0 on success, -1 on failure.
 */
DS9481P_API int ds9481p_i2c_start(ds9481p_device_handle handle);

/**
 * @brief Sends an I2C stop condition.
 *
 * @param handle A valid device handle.
 * @return 0 on success, -1 on failure.
 */
DS9481P_API int ds9481p_i2c_stop(ds9481p_device_handle handle);

/**
 * @brief Writes a byte to the I2C bus.
 *
 * @param handle A valid device handle.
 * @param byte The byte to write.
 * @return 0 on success, -1 on failure.
 */
DS9481P_API int ds9481p_i2c_write_byte(ds9481p_device_handle handle, unsigned char byte);

/**
 * @brief Reads a byte from the I2C bus and sends an ACK.
 *
 * @param handle A valid device handle.
 * @param byte Pointer to store the read byte.
 * @return 0 on success, -1 on failure.
 */
DS9481P_API int ds9481p_i2c_read_byte_ack(ds9481p_device_handle handle, unsigned char* byte);

/**
 * @brief Reads a byte from the I2C bus and sends a NACK.
 *
 * @param handle A valid device handle.
 * @param byte Pointer to store the read byte.
 * @return 0 on success, -1 on failure.
 */
DS9481P_API int ds9481p_i2c_read_byte_nack(ds9481p_device_handle handle, unsigned char* byte);

/**
 * @brief Reads the version of the adapter firmware.
 *
 * @param handle A valid device handle.
 * @param major Pointer to store the major version number.
 * @param minor Pointer to store the minor version number.
 * @return 0 on success, -1 on failure.
 */
DS9481P_API int ds9481p_get_version(ds9481p_device_handle handle, int* major, int* minor);


#ifdef __cplusplus
}
#endif

#endif // DS9481P_H