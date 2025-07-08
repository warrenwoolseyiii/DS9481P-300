import ctypes
import os
import platform

# --- Library Loading ---
def _get_lib_path():
    """Determines the path to the shared library."""
    lib_name = "libds9481p.dylib"
    if platform.system() == "Windows":
        lib_name = "ds9481p.dll"
    elif platform.system() == "Linux":
        lib_name = "libds9481p.so"

    # Look for the library in the build directory relative to this script
    # This is for development purposes
    script_dir = os.path.dirname(os.path.abspath(__file__))
    build_path = os.path.join(script_dir, '..', '..', 'build', lib_name)
    
    if os.path.exists(build_path):
        return build_path
        
    # Fallback to system search path
    return lib_name

_lib = ctypes.CDLL(_get_lib_path())

# --- C Function Prototypes ---
_lib.ds9481p_open.argtypes = [ctypes.c_char_p]
_lib.ds9481p_open.restype = ctypes.c_void_p

_lib.ds9481p_close.argtypes = [ctypes.c_void_p]
_lib.ds9481p_close.restype = None

_lib.ds9481p_enter_i2c_mode.argtypes = [ctypes.c_void_p]
_lib.ds9481p_enter_i2c_mode.restype = ctypes.c_int

_lib.ds9481p_enter_1wire_mode.argtypes = [ctypes.c_void_p]
_lib.ds9481p_enter_1wire_mode.restype = ctypes.c_int

_lib.ds9481p_i2c_start.argtypes = [ctypes.c_void_p]
_lib.ds9481p_i2c_start.restype = ctypes.c_int

_lib.ds9481p_i2c_stop.argtypes = [ctypes.c_void_p]
_lib.ds9481p_i2c_stop.restype = ctypes.c_int

_lib.ds9481p_i2c_write_byte.argtypes = [ctypes.c_void_p, ctypes.c_ubyte]
_lib.ds9481p_i2c_write_byte.restype = ctypes.c_int

_lib.ds9481p_i2c_read_byte_ack.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte)]
_lib.ds9481p_i2c_read_byte_ack.restype = ctypes.c_int

_lib.ds9481p_i2c_read_byte_nack.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte)]
_lib.ds9481p_i2c_read_byte_nack.restype = ctypes.c_int

_lib.ds9481p_get_version.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
_lib.ds9481p_get_version.restype = ctypes.c_int


# --- Python Wrapper Class ---
class DS9481P:
    """A Python wrapper for the DS9481P-300 driver."""

    def __init__(self, port_name):
        self._handle = _lib.ds9481p_open(port_name.encode('utf-8'))
        if not self._handle:
            raise ConnectionError(f"Failed to open device at {port_name}")

    def close(self):
        """Closes the connection to the device."""
        if self._handle:
            _lib.ds9481p_close(self._handle)
            self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def enter_i2c_mode(self):
        """Switches the device to I2C mode."""
        if _lib.ds9481p_enter_i2c_mode(self._handle) != 0:
            raise RuntimeError("Failed to enter I2C mode")

    def enter_1wire_mode(self):
        """Switches the device to 1-Wire mode."""
        if _lib.ds9481p_enter_1wire_mode(self._handle) != 0:
            raise RuntimeError("Failed to enter 1-Wire mode")

    def i2c_start(self):
        """Sends an I2C start condition."""
        if _lib.ds9481p_i2c_start(self._handle) != 0:
            raise RuntimeError("I2C start failed")

    def i2c_stop(self):
        """Sends an I2C stop condition."""
        if _lib.ds9481p_i2c_stop(self._handle) != 0:
            raise RuntimeError("I2C stop failed")

    def i2c_write_byte(self, byte):
        """Writes a single byte to the I2C bus."""
        if _lib.ds9481p_i2c_write_byte(self._handle, byte) != 0:
            raise RuntimeError("I2C write byte failed")

    def i2c_read_byte(self, ack=True):
        """Reads a single byte from the I2C bus."""
        byte = ctypes.c_ubyte()
        if ack:
            if _lib.ds9481p_i2c_read_byte_ack(self._handle, ctypes.byref(byte)) != 0:
                raise RuntimeError("I2C read byte with ACK failed")
        else:
            if _lib.ds9481p_i2c_read_byte_nack(self._handle, ctypes.byref(byte)) != 0:
                raise RuntimeError("I2C read byte with NACK failed")
        return byte.value

    def get_version(self):
        """Gets the firmware version of the adapter."""
        major = ctypes.c_int()
        minor = ctypes.c_int()
        if _lib.ds9481p_get_version(self._handle, ctypes.byref(major), ctypes.byref(minor)) != 0:
            raise RuntimeError("Failed to get device version")
        return (major.value, minor.value)