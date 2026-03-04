# DS9481P-300 Driver

Open source driver and software interface for the DS9481P-300 USB to I2C / 1-Wire adapter.

This repository contains a C/C++ shared library (`libds9481p`) and a Python wrapper (`pyds9481p`) to control the DS9481P-300 on macOS.

---

## Quick Start

```bash
# Prerequisites: Xcode Command Line Tools, CMake 3.10+, Python 3.6+
brew install cmake                    # macOS

# 1. Build the shared library
make                                  # → build/libds9481p.dylib

# 2. Build & run the C example
make examples                         # → examples/c/build/c_i2c_scan
./examples/c/build/c_i2c_scan /dev/tty.usbmodemXXXX

# 3. Install the Python wrapper & run the Python example
pip install -e python
python examples/python/python_i2c_scan.py /dev/tty.usbmodemXXXX

# Clean everything
make clean                            # wipes build/ and examples/c/build/
```

*(Replace `/dev/tty.usbmodemXXXX` with your device's actual port — run `ls /dev/tty.usbmodem*` to find it.)*

---

## Make Targets

| Target | Description |
|--------|-------------|
| `make` / `make all` | Configure and build the library into `build/` |
| `make examples` | Build the library + the C example into `examples/c/build/` |
| `make clean` | Remove `build/` and `examples/c/build/` |

Extra CMake flags can be passed via `CMAKE_FLAGS`:

```bash
make CMAKE_FLAGS="-DDS9481P_ENABLE_DEBUG=ON"   # enable verbose serial tracing
```

---

## Prerequisites

- A C/C++ compiler (like Apple Clang, which comes with Xcode Command Line Tools)
- CMake (version 3.10 or higher)
- Python 3.6 or higher
- The DS9481P-300 device should be recognized as a serial port by macOS (e.g., `/dev/tty.usbmodemXXXX`).

---

## Installation

The project is composed of a C/C++ core library and a Python wrapper. You must build the C/C++ library first.

### 1. Build the C/C++ Library (`libds9481p`)

From the root of the project directory, run:

```bash
make                       # configure + build into build/
```

This will create the `libds9481p.dylib` (macOS) or `libds9481p.so` (Linux) file inside the `build/` directory.

To wipe the build directory and start fresh:

```bash
make clean                 # removes build/ and examples/c/build/
```

### 2. Install the Python Wrapper (`pyds9481p`)

Once the C/C++ library is built, you can install the Python package. It is recommended to install it in editable mode so that changes to the Python code are immediately reflected.

From the root of the project directory, run:

```bash
pip install -e python
```

---

## Usage

The `examples` directory contains scripts that demonstrate how to use the driver.

### Python I2C Scan Example

This script scans the I2C bus and prints the addresses of any devices that respond.

1.  **Build the C/C++ library** and **install the Python wrapper** as described in the [Installation](#installation) section.
2.  Run the script from the root of the project, providing the serial port name as an argument:

```bash
python examples/python/python_i2c_scan.py /dev/tty.usbmodemXXXX
```
*(Replace `/dev/tty.usbmodemXXXX` with your device's actual port name.)*

### C/C++ I2C Scan Example

This example does the same thing as the Python script but is written in C.

1.  **Build the library and example** from the project root:
    ```bash
    make examples             # builds libds9481p + the C example
    ```
2.  Run the compiled executable:
    ```bash
    ./examples/c/build/c_i2c_scan /dev/tty.usbmodemXXXX
    ```

---

## Developer Documentation

### Python Example

The easiest way to use the driver is through the Python wrapper. The following example shows how to connect to the device and scan the I2C bus for connected devices.

```python
from pyds9481p import DS9481P
import time

# Find your device's serial port name by running `ls /dev/tty.*`
# before and after plugging it in.
PORT_NAME = "/dev/tty.usbmodemXXXX" # <-- IMPORTANT: Change this!

try:
    with DS9481P(PORT_NAME) as device:
        print(f"Successfully connected to device on {PORT_NAME}")
        
        # Switch to I2C mode
        device.enter_i2c_mode()
        print("Switched to I2C mode.")
        time.sleep(0.1) # Short delay after mode switch

        # --- Scan for I2C devices ---
        print("Scanning for I2C devices...")
        found_devices = []
        for address in range(1, 128):
            try:
                device.i2c_start()
                # Set slave address with write bit (0)
                device.i2c_write_byte((address << 1) | 0)
                # If the write is successful, a device acknowledged
                found_devices.append(address)
            except RuntimeError:
                # No device responded at this address
                pass
            finally:
                # Always send a stop condition to release the bus
                device.i2c_stop()

        if found_devices:
            print("Found I2C devices at the following addresses:")
            for addr in found_devices:
                print(f" - 0x{addr:02X} ({addr})")
        else:
            print("No I2C devices found.")

except ConnectionError as e:
    print(f"Error: {e}")
except Exception as e:
    print(f"An unexpected error occurred: {e}")

```

### C/C++ Example

To use the library in a C/C++ project, you need to link against the `libds9481p.dylib` library and include the `ds9481p.h` header.

1.  **main.c:**
    ```c
    #include <stdio.h>
    #include "ds9481p.h"

    int main() {
        const char* port_name = "/dev/tty.usbmodemXXXX"; // <-- Change this!
        ds9481p_device_handle handle = ds9481p_open(port_name);

        if (!handle) {
            fprintf(stderr, "Failed to open device at %s\n", port_name);
            return 1;
        }

        printf("Device opened successfully.\n");

        // ... use the device ...
        // e.g., ds9481p_enter_i2c_mode(handle);

        ds9481p_close(handle);
        printf("Device closed.\n");

        return 0;
    }
    ```

2.  **CMakeLists.txt for your application:**
    ```cmake
    cmake_minimum_required(VERSION 3.10)
    project(my_app C)

    # Find the ds9481p library
    # Replace /path/to/ds9481p-driver/build with the actual path
    link_directories(/path/to/ds9481p-driver/build) 
    include_directories(/path/to/ds9481p-driver/include)

    add_executable(my_app main.c)

    # Link against the library
    target_link_libraries(my_app ds9481p)
    ```

---

## For Developers

The core logic is implemented in C for portability and performance. The Python wrapper uses `ctypes` to call the C functions, avoiding the need for a separate compilation step for the wrapper itself.

-   **C Library:** `src/ds9481p.c`, `include/ds9481p.h`
-   **Python Wrapper:** `python/pyds9481p/__init__.py`
-   **Build System:** `Makefile` (wraps `CMakeLists.txt` via CMake)

To add new functionality (e.g., 1-Wire commands):
1.  Add the function signature to `include/ds9481p.h`.
2.  Implement the function in `src/ds9481p.c`.
3.  Rebuild the C library (`make`).
4.  Add the function prototype and a corresponding Python method in `python/pyds9481p/__init__.py`.
