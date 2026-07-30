from __future__ import annotations

from dataclasses import dataclass, field


VALID_MODES = {"DIAG", "CIRCLE", "DOUBLE"}


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
        except ImportError as error:
            raise RuntimeError("pyserial is required for the STM32 UART") from error
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

    def status(self) -> str | None:
        return self.command("STATUS", expect_reply=True)

    def set_frequency_hz(self, frequency_hz: float) -> None:
        self.command(f"FREQ {int(round(frequency_hz))}")

    def set_phase_degrees(self, phase_degrees: float) -> None:
        self.command(f"PHASE {int(round(phase_degrees)) % 360}")

    def set_auto_mode(self, mode: str) -> None:
        normalized = mode.upper()
        if normalized not in VALID_MODES:
            raise ValueError(f"unsupported auto mode: {mode}")
        self.command(f"AUTO {normalized}")

    def request_probe(self, ramp_us: int) -> None:
        if ramp_us not in {100, 200, 500, 1000, 2000, 5000}:
            raise ValueError(f"unsupported probe ramp: {ramp_us} us")
        self.command(f"PROBE {ramp_us}")

    def __enter__(self) -> "McuLink":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


@dataclass
class RecordingMcuLink:
    """Test/dry-run implementation with the same public command methods."""

    commands: list[str] = field(default_factory=list)

    def command(self, text: str, expect_reply: bool = False) -> str | None:
        self.commands.append(text.strip().upper())
        return "OK" if expect_reply else None

    def set_frequency_hz(self, frequency_hz: float) -> None:
        self.command(f"FREQ {int(round(frequency_hz))}")

    def set_phase_degrees(self, phase_degrees: float) -> None:
        self.command(f"PHASE {int(round(phase_degrees)) % 360}")

    def set_auto_mode(self, mode: str) -> None:
        normalized = mode.upper()
        if normalized not in VALID_MODES:
            raise ValueError(f"unsupported auto mode: {mode}")
        self.command(f"AUTO {normalized}")

    def request_probe(self, ramp_us: int) -> None:
        if ramp_us not in {100, 200, 500, 1000, 2000, 5000}:
            raise ValueError(f"unsupported probe ramp: {ramp_us} us")
        self.command(f"PROBE {ramp_us}")
