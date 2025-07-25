#MIT License
#
#Copyright (c) 2025 budgettsfrog@protonmail.com
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

import sys
import time
from pyds9481p import DS9481P

def main():
    """
    Scans the I2C bus for connected devices using the DS9481P-300.
    """
    if len(sys.argv) < 2:
        print("Error: Please provide the serial port name as an argument.")
        print(f"Usage: python {sys.argv[0]} /dev/tty.usbmodemXXXX")
        print("You can find the port name by running `ls /dev/tty.*` in your terminal.")
        sys.exit(1)

    port_name = sys.argv[1]

    try:
        with DS9481P(port_name) as device:
            print(f"Successfully connected to device on {port_name}")
            
            # Switch to I2C mode
            device.enter_i2c_mode()
            print("Switched to I2C mode.")

            # Get and display firmware version
            major, minor = device.get_version()
            print(f"Device Firmware Version: {major}.{minor}")
            time.sleep(0.1) # Short delay after mode switch

            # --- Scan for I2C devices ---
            print("Scanning for I2C devices (addresses 1-127)...")
            found_devices = []
            for address in range(1, 128):
                try:
                    device.i2c_start()
                    # Set slave address with write bit (0)
                    # The device will send an ACK if a slave responds.
                    device.i2c_write_byte((address << 1) | 0)
                    found_devices.append(address)
                except RuntimeError:
                    # No device responded at this address
                    pass
                finally:
                    # Always send a stop condition to release the bus
                    device.i2c_stop()

            if found_devices:
                print("\nFound I2C devices at the following addresses:")
                for addr in found_devices:
                    print(f" - 0x{addr:02X} ({addr})")
            else:
                print("\nNo I2C devices found.")

    except ConnectionError as e:
        print(f"\nError: {e}")
        print("Please ensure you have the correct port name and the device is connected.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")

if __name__ == "__main__":
    main()