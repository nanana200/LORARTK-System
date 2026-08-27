"""Full-duplex serial transport for the STM32 ST-LINK VCP."""

from __future__ import annotations

import threading
from collections.abc import Callable

import serial


class SerialBridge:
    def __init__(
        self,
        port: str,
        baudrate: int,
        line_callback: Callable[[str], None],
        error_callback: Callable[[str], None],
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self._line_callback = line_callback
        self._error_callback = error_callback
        self._serial: serial.Serial | None = None
        self._stop_event = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._write_lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def open(self) -> None:
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=5.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self._stop_event.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_worker,
            name="stm32-serial-rx",
            daemon=True,
        )
        self._rx_thread.start()

    def write(self, data: bytes) -> int:
        if not data:
            return 0
        serial_port = self._serial
        if serial_port is None or not serial_port.is_open:
            raise serial.SerialException(f"{self.port} is not open")

        total = 0
        view = memoryview(data)
        with self._write_lock:
            while total < len(data):
                written = serial_port.write(view[total:])
                if written <= 0:
                    raise serial.SerialTimeoutException("serial write made no progress")
                total += written
        return total

    def _rx_worker(self) -> None:
        text_buffer = ""
        try:
            while not self._stop_event.is_set():
                serial_port = self._serial
                if serial_port is None:
                    return
                waiting = serial_port.in_waiting
                data = serial_port.read(waiting if waiting else 1)
                if not data:
                    continue
                text_buffer += data.decode("ascii", errors="replace")
                while "\n" in text_buffer:
                    line, text_buffer = text_buffer.split("\n", 1)
                    self._line_callback(line.rstrip("\r"))
        except (OSError, serial.SerialException) as exc:
            if not self._stop_event.is_set():
                self._error_callback(str(exc))
        finally:
            if text_buffer:
                self._line_callback(text_buffer.rstrip("\r"))

    def close(self) -> None:
        self._stop_event.set()
        serial_port, self._serial = self._serial, None
        if serial_port is not None:
            try:
                serial_port.cancel_read()
            except (AttributeError, OSError, serial.SerialException):
                pass
            serial_port.close()
        thread, self._rx_thread = self._rx_thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

