"""STM32 için 5 baytlık imzalı hedef hata protokolü."""

from time import sleep
import struct

import serial


PACKET_TRACKING = 0xFF
PACKET_LOCKED = 0xFE


def int16_value(value):
    """Referans int() davranışıyla değeri signed int16 aralığına sınırla."""
    return max(-32768, min(32767, int(value)))


def pack_target_packet(header, error_x, error_y):
    return struct.pack(
        "<Bhh",
        int(header),
        int16_value(error_x),
        int16_value(error_y),
    )


class TargetUart:
    """Aktif hedef hatasını STM32'ye ileten opsiyonel seri bağlantı."""

    def __init__(self, enabled, port, baudrate, invert_x=True, serial_factory=None):
        self.enabled = bool(enabled)
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.invert_x = bool(invert_x)
        self.serial_factory = serial_factory or serial.Serial
        self.serial = None

    @property
    def connected(self):
        return self.serial is not None and self.serial.is_open

    def open(self):
        if not self.enabled or self.connected:
            return
        try:
            self.serial = self.serial_factory(
                self.port,
                self.baudrate,
                timeout=0,
                write_timeout=0.1,
            )
            sleep(0.3)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            print(f"Seri bağlantı başarılı: {self.port} @ {self.baudrate}")
        except (OSError, serial.SerialException) as error:
            self.serial = None
            raise RuntimeError(f"Seri port açılamadı ({self.port}): {error}") from error

    def wire_errors(self, error_x, error_y):
        x_value = int16_value(error_x)
        y_value = int16_value(error_y)
        return (-x_value if self.invert_x else x_value), y_value

    def send_target(self, error_x, error_y, locked=False):
        wire_x, wire_y = self.wire_errors(error_x, error_y)
        header = PACKET_LOCKED if locked else PACKET_TRACKING
        self._write(pack_target_packet(header, wire_x, wire_y))
        return wire_x, wire_y

    def send_no_target(self):
        self._write(pack_target_packet(PACKET_TRACKING, 0, 0))

    def _write(self, packet):
        if not self.connected:
            return
        try:
            self.serial.write(packet)
        except (OSError, serial.SerialException) as error:
            print(f"UART gönderme hatası: {error}")

    def read_message(self):
        if not self.connected:
            return ""
        try:
            waiting = self.serial.in_waiting
            if waiting <= 0:
                return ""
            message = self.serial.read(waiting).decode("utf-8", errors="ignore").strip()
            if message:
                print(f"STM32: {message}")
            return message
        except (OSError, serial.SerialException) as error:
            print(f"UART okuma hatası: {error}")
            return ""

    def close(self):
        if self.connected:
            self.serial.close()
            print("Seri port kapatıldı.")
        self.serial = None
