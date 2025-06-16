"""
Defines a serial communication handler
"""

import time
from serial import Serial, SerialException
from serial.tools.list_ports import comports
from serial.tools.list_ports_common import ListPortInfo
import threading
from threading import Lock
from typing import Literal, Callable, Any, Optional
from logging import Logger

def get_serial_ports() -> list[ListPortInfo]:
    """Get a list of available serial ports.
    
    Return: list[ListPortInfo]: A list of available serial ports."""
    return [port.device for port in comports()]

class SerialHandler:
    """Class to handle serial communication"""
    locks: dict[str, Lock] = {} # Locks for all serial handles

    
    def __init__(self, port: str, baudrate: int, timeout: float = 1.0):
        """Class to handle serial communication
        
        Args:
            port (str): The serial port to connect to. Example: 'COM3' on Windows or '/dev/ttyUSB0' on Linux.
            baudrate (int): The baud rate for the serial communication. Example: 9600.
            timeout (float, optional): The timeout for the serial communication. Defaults to 1.0."""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        SerialHandler.locks[port] = Lock()
        
        self.serial = Serial()
        self.serial.port, self.serial.baudrate, self.serial.timeout = port, baudrate, timeout
    
    def open(self) -> None:
        """Open the serial port"""
        self.serial.open()
    
    def is_open(self) -> bool:
        """Check if the serial port is open.
        
        Return: bool: True if the serial port is open, False otherwise."""
        return self.serial.is_open

    def close(self) -> None:
        """Close the serial port."""
        if self.is_open(): 
            self.serial.close()
        else:
            raise SerialException("Serial port is not open")

    def send(self, data: bytearray) -> None:
        """Send data to the serial port."""
        if self.is_open():
            with SerialHandler.locks[self.port]:
                self.serial.write(data)
        else:
            raise SerialException("Serial port is not open")

    def receive_line(self, encoding: str = 'ascii', remove_newline: bool = True) -> str:
        """Receive a line of data from the serial port.
        
        Args: 
           timeout (float, optional): The timeout in seconds for the serial communication. Defaults to 1.0.
           encoding (str, optional): The encoding for the received data. Defaults to 'ascii'."""
        if self.is_open():  # Check if the serial port is open
            with SerialHandler.locks[self.port]:
                line = self.serial.readline().decode(encoding)  # Read a line of data and decode it to a string
            if remove_newline:
                line = line.rstrip('\n\r')  # Remove newline characters at the end of the received data
            return line
        else:
            raise SerialException("Serial port is not open")

class ThreadedSerialHandler:
    """Wrapper of SerialHandler with auto-reconnecting behaviour and safety measures."""
    def __init__(self, baudrate: int=115200, timeout: float=1.0, wait_time: float = 0.1, logger: Optional[Logger] = None):
        self._lock = threading.Lock()
        self._running = False
        self._serial_handler: SerialHandler | None = None
        self.logger = logger

        self._baudrate, self._timeout, self._wait_time = baudrate, timeout, wait_time
        self._message_queue: list[str] = []

        self.start_threaded()
        
    def queue_message(self, msg: str) -> None:
        """Queue a message to be sent to the serial port."""
        if self._serial_handler and self._serial_handler.is_open():
            with self._lock:
                self._message_queue.append(msg)

    def _loop(self):
        self._running = True
        port = None
        while self._running:
            try: 
                port = get_serial_ports()[0]
            except Exception:
                self._log("No serial port found.")
                time.sleep(self._wait_time)
                continue
            try: # Try connection
                if self._serial_handler is None:
                    self._log("SerialHandler is None: attempting to create a new instance...")
                    self._serial_handler = SerialHandler(port, self._baudrate, self._timeout)
                if self._serial_handler.is_open() is False:
                    self._log("Serial connection is not connected: attempting to connect...")
                    self._serial_handler.open()
                    self._message_queue.clear() # Avoid stacked queued messages
                    self._log(f"Serial connection established to port {port}.")
            except SerialException as e:
                self._log(f"Serial connection failed: {e}")
            
            try: # Try sending message
                if self._serial_handler and self._serial_handler.is_open():
                    if len(self._message_queue) > 0:
                        try: # try sending message
                            with self._lock:
                                msg = self._message_queue.pop(0)
                            self._serial_handler.send(bytearray(msg.encode("ascii")))
                            self._log(f"Message sent to {port}: {msg}")
                        except SerialException as e:
                            self._log(f"Failed to send message: {e}")
                            self._serial_handler = None
                    else: # No message to send
                        pass
            except SerialException as e:
                self._log(f"Serial connection failed: {e}")
                self._serial_handler = None

            time.sleep(self._wait_time)

    def start_threaded(self) -> None:
        """Start threaded execution."""
        if self._running is False:
            with self._lock:
                self._running = True
            self._threader(self._loop)

    def _threader(self, target: Callable, **args: Any) -> None:  # create a thread
        """Launch thread."""
        thr = threading.Thread(target=self._safe_thread, kwargs={"target": target, **args})
        thr.daemon = True
        thr.start()

    def _safe_thread(self, target: Callable, **args: Any) -> None:  # thread wrapper
        """Thread wrapper to handle exceptions."""
        try:
            target(**args)
        except KeyboardInterrupt:
            try:
                with self._lock:
                    self._running = False
            except Exception:
                pass
            pass
        except Exception as e:
            with self._lock:
                self._running = False
            raise e from e
        finally:
            if self._serial_handler is not None:
                if self._serial_handler.is_open():
                    self._serial_handler.close()
                    
    def stop(self) -> None:
        """Stop threaded execution."""
        with self._lock:
            self._running = False

    def _log(self, msg:str, log_level: Literal[10, 20, 30, 40, 50] = 10): # 10 = DEBUG, 20 = INFO, 30 = WARNING, 40 = ERROR, 50 = CRITICAL
        """Log given message to self.logger if is not None"""
        if self.logger:
            with self._lock:
                self.logger.log(log_level, msg)
