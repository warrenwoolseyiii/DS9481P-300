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