from typing import Optional
import pigpio, time

def make_crc_table(poly=0x31) -> list[int]:
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ poly
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return table

CRC_TABLE = make_crc_table()

def crc8_fast(data: bytes, init=0xFF) -> int:
    crc = init
    for byte in data:
        crc = CRC_TABLE[crc ^ byte]
    return crc

class SHT4x:
    SOFT_RESET = 0x94
    # No-heater measurement commands
    NOHEAT_HIGHPRECISION  = 0xFD  # what Adafruit calls NOHEAT_HIGHPRECISION
    NOHEAT_MEDPRECISION   = 0xF6
    NOHEAT_LOWPRECISION   = 0xE0

    # Heater-enabled single-shot cmds (optional)
    HIGHHEAT_1S           = 0x39
    HIGHHEAT_100MS        = 0x3A
    MEDHEAT_1S            = 0x32
    MEDHEAT_100MS         = 0x33
    LOWHEAT_1S            = 0x2F
    LOWHEAT_100MS         = 0x30

    def __init__(self, pi: Optional[pigpio.pi] = None, bus: int = 0, address: int = 0x44, mode=None):
        if not pi: pi = pigpio.pi()
        self.pi = pi
        self.handle = pi.i2c_open(bus, address)
        if self.handle < 0:
            raise RuntimeError(f"Cannot open I2C bus{bus}@0x{address:02X}")
        self.mode = mode or self.NOHEAT_HIGHPRECISION

    def reset(self):
        # single‐byte reset
        rc = self.pi.i2c_write_byte(self.handle, self.SOFT_RESET)
        if rc != 0:
            raise RuntimeError("SHT4x soft reset failed")

    def read(self):
        # trigger measurement
        rc = self.pi.i2c_write_byte(self.handle, self.mode)
        if rc != 0:
            raise RuntimeError("SHT4x measure cmd failed")

        time.sleep(0.01)
        count, data = self.pi.i2c_read_device(self.handle, 6)
        if count != 6:
            raise RuntimeError(f"Expected 6 bytes, got {count}")
        return data.encode("latin1") if isinstance(data, str) else data

    def parse_sht4x(self, buf: bytes):
        # Split raw words and CRCs
        t_bytes = buf[0:2]
        t_crc   = buf[2]
        h_bytes = buf[3:5]
        h_crc   = buf[5]

        # Validate CRCs
        if crc8_fast(t_bytes) != t_crc:
            raise RuntimeError(f"SHT4x temperature CRC fail: {hex(crc8_fast(t_bytes))} != {hex(t_crc)}")
        if crc8_fast(h_bytes) != h_crc:
            raise RuntimeError(f"SHT4x humidity    CRC fail: {hex(crc8_fast(h_bytes))} != {hex(h_crc)}")

        # Parse values
        t_raw = int.from_bytes(t_bytes, "big")
        h_raw = int.from_bytes(h_bytes, "big")

        temperature = -45 + 175 * (t_raw / 65535.0)
        humidity    = 100 * (h_raw  / 65535.0)
        return temperature, humidity
    
    @property
    def measurements(self) -> tuple[float, float]:
        """
        Returns a (temperature_C, humidity_%RH) tuple that is CRC checked.
        """
        return self.parse_sht4x(self.read())

    def close(self):
        self.pi.i2c_close(self.handle)
