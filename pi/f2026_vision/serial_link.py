from __future__ import annotations

import os
import select
import time

if os.name == "posix":
    import termios
    import tty
else:
    termios = None
    tty = None


class McuLink:
    """Adapter for the ASCII protocol already implemented by the STM32."""

    def __init__(
        self,
        port: str = "/dev/serial0",
        baudrate: int = 115200,
        timeout: float = 0.25,
    ) -> None:
        try:
            import serial
        except ImportError:
            self._serial = _PosixSerial(port, baudrate, timeout)
        else:
            self._serial = serial.Serial(port, baudrate, timeout=timeout)

    def close(self) -> None:
        self._serial.close()

    def command(self, text: str, expect_reply: bool = False) -> str | None:
        command = text.strip().upper()
        self._serial.write((command + "\r\n").encode("ascii"))
        self._serial.flush()
        if not expect_reply:
            return None
        return self._serial.readline().decode("ascii", errors="replace").strip()

    def set_idle(self) -> None:
        self.command("IDLE", expect_reply=True)

    def request_probe(self, ramp_us: int, frame_us: int | None = None) -> str | None:
        if frame_us is None:
            return self.command(f"PROBE {int(ramp_us)}", expect_reply=True)
        return self.command(f"PROBE {int(ramp_us)} {int(frame_us)}", expect_reply=True)

    def start_probe_sweep(self) -> str | None:
        """Arm the FPGA-resident eight-setting Q5 probe sweep once."""
        return self.command("SWEEP", expect_reply=True)

    def __enter__(self) -> "McuLink":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


class _PosixSerial:
    """Tiny pyserial-compatible fallback for Orange Pi board UARTs."""

    def __init__(self, port: str, baudrate: int, timeout: float) -> None:
        if os.name != "posix" or termios is None or tty is None:
            raise RuntimeError("pyserial is required for STM32 UART on this OS")
        baudrates = {
            9600: termios.B9600,
            19200: termios.B19200,
            38400: termios.B38400,
            57600: termios.B57600,
            115200: termios.B115200,
        }
        if baudrate not in baudrates:
            raise RuntimeError(f"unsupported fallback UART baudrate: {baudrate}")
        self.timeout = timeout
        self._fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        attrs = termios.tcgetattr(self._fd)
        attrs[4] = baudrates[baudrate]
        attrs[5] = baudrates[baudrate]
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[2] &= ~termios.CSTOPB
        attrs[2] &= ~termios.PARENB
        attrs[2] &= ~termios.CSIZE
        attrs[2] |= termios.CS8
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        termios.tcflush(self._fd, termios.TCIOFLUSH)

    def write(self, data: bytes) -> int:
        return os.write(self._fd, data)

    def flush(self) -> None:
        termios.tcdrain(self._fd)

    def readline(self) -> bytes:
        deadline = time.monotonic() + self.timeout
        chunks: list[bytes] = []
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            readable, _, _ = select.select([self._fd], [], [], remaining)
            if not readable:
                break
            chunk = os.read(self._fd, 1)
            if not chunk:
                continue
            chunks.append(chunk)
            if chunk == b"\n":
                break
        return b"".join(chunks)

    def close(self) -> None:
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1
