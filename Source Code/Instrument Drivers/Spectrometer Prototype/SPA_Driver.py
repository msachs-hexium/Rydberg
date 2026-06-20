
import argparse
import json
import math
import os
import random
import struct
import sys
import time
from collections import deque

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


# =======================================================================
#  Constants
# =======================================================================

BAUD_RATE = 115200
TX_CHECKSUM_SEED = 0x5555
RX_PACKET_SIZE = 16
TX_PACKET_SIZE = 8       # per register
NUM_REGISTERS = 32       # $00 .. $1F
FULL_PACKET_SIZE = NUM_REGISTERS * TX_PACKET_SIZE   # 256 bytes
KEEPALIVE_INTERVAL_MS = 200
CONNECTION_TIMEOUT_MS = 2000
ADC_OVERLOAD_THRESHOLD = 8_000_000
ADC_FULL_SCALE = 8_388_608      # 2^23

# Register addresses
REG_CONTROL     = 0x01
REG_SAMPLE_RATE = 0x02
REG_CH1_RELAY   = 0x03
REG_CH1_PGA     = 0x04
REG_SAMPLE_DEPTH = 0x05
REG_CH1_SHORT   = 0x06
REG_CH1_BIAS    = 0x07
REG_CH2_RELAY   = 0x0B
REG_CH2_PGA     = 0x0C
REG_CH2_SHORT   = 0x0E
REG_CH2_BIAS    = 0x0F
REG_FLASH_KEY   = 0x1E

# Control register ($01) bit masks
BIT_LED_DISABLE      = 1 << 12   # b12 -- 0 = LED enabled, 1 = LED disabled
BIT_FLASH_READ_RESET = 1 << 13   # b13 -- 0 = reset read counter, 1 = normal
BIT_FLASH_ERASE      = 1 << 14   # b14
BIT_FLASH_WRITE      = 1 << 15   # b15
BIT_TRANSMIT_ENABLE  = 1 << 16   # b16 -- 1 = SPA sends measurement data
BIT_WATCHDOG_DISABLE = 1 << 17   # b17

# Range table: index -> (relay, PGA, full-scale label, JSON key)
#   Index 0 = most sensitive (200 pA), index 7 = least sensitive (2 mA)
RANGE_TABLE = [
    (3, 8, "200 pA", "Range8"),
    (3, 1, "2 nA",   "Range7"),
    (2, 8, "20 nA",  "Range6"),
    (2, 1, "200 nA", "Range5"),
    (1, 8, "2 uA",   "Range4"),
    (1, 1, "20 uA",  "Range3"),
    (0, 8, "200 uA", "Range2"),
    (0, 1, "2 mA",   "Range1"),
]

# Sample rate: Hz -> (timer value, sample depth)
SAMPLE_RATES = {
    2:   (50000, 18),
    10:  (10000, 16),
    100: (1000,  16),
}


# =======================================================================
#  Low-level packet building and parsing
# =======================================================================

def build_write_packet(address: int, value: int) -> bytes:
    """Build an 8-byte register-write packet.

    Packet layout:
        [addr_hi | addr_lo | d3 | d2 | d1 | d0 | cs_hi | cs_lo]

    The MSB of addr_hi is SET to indicate a write operation.
    Checksum = 0x5555 + addr_word + data_hi_word + data_lo_word (16-bit).
    """
    value = value & 0xFFFFFFFF
    addr_hi = 0x80 | ((address >> 8) & 0x7F)
    addr_lo = address & 0xFF
    d3 = (value >> 24) & 0xFF
    d2 = (value >> 16) & 0xFF
    d1 = (value >> 8)  & 0xFF
    d0 = value & 0xFF
    cs = TX_CHECKSUM_SEED + (addr_hi << 8 | addr_lo) + (d3 << 8 | d2) + (d1 << 8 | d0)
    return bytes([addr_hi, addr_lo, d3, d2, d1, d0, (cs >> 8) & 0xFF, cs & 0xFF])


def build_read_packet(address: int = 0) -> bytes:
    """Build an 8-byte read (keep-alive) packet.  MSB of addr is CLEAR."""
    addr_hi = (address >> 8) & 0x7F
    addr_lo = address & 0xFF
    cs = TX_CHECKSUM_SEED + (addr_hi << 8 | addr_lo)
    return bytes([addr_hi, addr_lo, 0, 0, 0, 0, (cs >> 8) & 0xFF, cs & 0xFF])


def build_config_block(registers: list) -> bytes:
    """Build the full 256-byte configuration block (32 registers, reverse order)."""
    buf = bytearray()
    for addr in range(NUM_REGISTERS - 1, -1, -1):
        buf.extend(build_write_packet(addr, registers[addr]))
    return bytes(buf)


def parse_response(data: bytes) -> dict | None:
    """Parse a 16-byte SPA response packet.

    Returns a dict with raw fields, or None if the checksum is bad.

    Packet layout:
        [status_hi, status_lo, usbcal_hi, usbcal_lo, adc0_hi, adc0_lo,
         ch1_hi, ch1_mid, ch1_lo, ch2_hi, ch2_mid, ch2_lo,
         adc3_hi, adc3_mid, adc3_lo, checksum]
    """
    if len(data) < RX_PACKET_SIZE:
        return None

    expected_cs = sum(data[:15]) & 0xFF
    if expected_cs != data[15]:
        return None

    status  = (data[0] << 8) | data[1]
    usbcal  = (data[2] << 8) | data[3]
    ch1_raw = (data[6] << 16) | (data[7] << 8) | data[8]
    ch2_raw = (data[9] << 16) | (data[10] << 8) | data[11]

    return {
        "status":  status,
        "usbcal":  usbcal,
        "ch1_raw": _sign_extend_24(ch1_raw),
        "ch2_raw": _sign_extend_24(ch2_raw),
        "ch1_overload": abs(_sign_extend_24(ch1_raw)) > ADC_OVERLOAD_THRESHOLD,
        "ch2_overload": abs(_sign_extend_24(ch2_raw)) > ADC_OVERLOAD_THRESHOLD,
        "cal_data_flag": bool(status & (1 << 12)),
        "cal_sync_flag": bool(status & (1 << 13)),
    }


def _sign_extend_24(val: int) -> int:
    """Convert an unsigned 24-bit integer to signed."""
    val &= 0xFFFFFF
    return val - 0x1000000 if val >= 0x800000 else val


# =======================================================================
#  Calibration
# =======================================================================

def load_calibration_json(filepath: str) -> dict | None:
    """Load calibration data from a JSON file written by EPIC.

    Returns a dict with keys 'Ch1' and 'Ch2', each containing per-range
    calibration ('+ADC', '-ADC', '+I', '-I') and source DAC cal.
    Returns None if the file does not exist or cannot be parsed.
    """
    if not os.path.isfile(filepath):
        return None
    try:
        with open(filepath, "r") as f:
            cal = json.load(f)
        # Sanity check: must have Ch1 with at least Range1
        if "Ch1" in cal and "Range1" in cal["Ch1"]:
            return cal
    except (json.JSONDecodeError, KeyError):
        pass
    return None


def save_calibration_json(cal: dict, filepath: str) -> None:
    """Save calibration dict to a JSON file (same format as EPIC)."""
    os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
    with open(filepath, "w") as f:
        json.dump(cal, f, indent=2)
    print(f"[cal] Saved calibration to {filepath}")


def download_calibration(ser: serial.Serial) -> dict | None:
    """Download calibration from the SPA's internal flash memory.

    The SPA streams 250 x 16-bit words per pass.  Two passes are performed
    and compared; on mismatch the download retries (up to 3 attempts).

    Word layout per channel (100 words each):
        words 0-1   : +DAC (unsigned 32-bit)
        words 2-3   : -DAC (unsigned 32-bit)
        words 4-99  : 8 ranges x 12 words
            Per range (R=1..8), offset = 4 + (R-1)*12 :
                +0,+1       : +ADC  (signed 32-bit long)
                +2,+3       : -ADC  (signed 32-bit long)
                +4..+7      : +I    (64-bit double, Amps)
                +8..+11     : -I    (64-bit double, Amps)
        Ch1 = words 0..99,  Ch2 = words 100..199
        Metadata (serial number etc.) = words 200..249
    """
    MAX_RETRIES = 3

    for attempt in range(MAX_RETRIES):
        print(f"[cal] Downloading calibration from instrument (attempt {attempt+1})...")

        passes = []
        for pass_num in range(2):
            words = _download_one_pass(ser)
            if words is None:
                break
            passes.append(words)

        if len(passes) < 2:
            print("[cal] Download failed -- no data received.")
            continue

        # Compare the two passes
        if passes[0] == passes[1]:
            print("[cal] Verification OK -- two passes match.")
            return _parse_cal_words(passes[0])
        else:
            mismatches = sum(1 for a, b in zip(passes[0], passes[1]) if a != b)
            print(f"[cal] Verification FAILED -- {mismatches} word(s) differ. Retrying...")

    print("[cal] Calibration download failed after all retries.")
    return None


def _download_one_pass(ser: serial.Serial, timeout_s: float = 10.0) -> list | None:
    """Download one pass of 250 calibration words from the SPA."""
    words = []
    synced = False
    deadline = time.time() + timeout_s

    # Send a config packet with flash read counter reset to trigger download
    regs = [0] * NUM_REGISTERS
    regs[REG_CONTROL] = BIT_TRANSMIT_ENABLE  # b16=1 (TX on), b13=0 (reset flash counter)
    regs[REG_SAMPLE_RATE] = 10000   # 10 Hz
    regs[REG_SAMPLE_DEPTH] = 16
    regs[REG_CH1_RELAY] = 3
    regs[REG_CH1_PGA] = 1
    regs[REG_CH2_RELAY] = 3
    regs[REG_CH2_PGA] = 1
    ser.write(build_config_block(regs))
    time.sleep(0.1)

    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf.extend(chunk)

        while len(buf) >= RX_PACKET_SIZE:
            pkt = parse_response(bytes(buf[:RX_PACKET_SIZE]))
            if pkt is None:
                # Bad checksum -- skip one byte to try to re-sync
                buf.pop(0)
                continue
            buf = buf[RX_PACKET_SIZE:]

            if not pkt["cal_data_flag"]:
                # Normal data packet (no cal data), send keepalive
                ser.write(build_read_packet())
                continue

            if pkt["cal_sync_flag"] and not synced:
                # Start of a new pass
                synced = True
                words = [pkt["usbcal"]]
            elif synced:
                words.append(pkt["usbcal"])

            if len(words) >= 250:
                return words

        # Send keepalive
        ser.write(build_read_packet())
        time.sleep(0.05)

    return None


def _parse_cal_words(words: list) -> dict:
    """Convert 250 raw 16-bit words into a calibration dict (JSON-compatible)."""
    cal = {}

    for ch_idx, ch_name in enumerate(["Ch1", "Ch2"]):
        offset = ch_idx * 100
        ch_cal = {}

        # Source DAC calibration (words 0-1 = +DAC, 2-3 = -DAC)
        pos_dac = _words_to_long_unsigned(words, offset + 0)
        neg_dac = _words_to_long_unsigned(words, offset + 2)
        ch_cal["Source"] = {"+DAC": pos_dac, "-DAC": neg_dac, "+V": 0, "-V": 0}

        # Per-range calibration
        for r in range(1, 9):
            base = offset + 4 + (r - 1) * 12
            pos_adc = _words_to_long_signed(words, base + 0)
            neg_adc = _words_to_long_signed(words, base + 2)
            pos_i   = _words_to_double(words, base + 4)
            neg_i   = _words_to_double(words, base + 8)
            ch_cal[f"Range{r}"] = {
                "+ADC": pos_adc, "-ADC": neg_adc,
                "+I": pos_i,     "-I": neg_i,
            }
        cal[ch_name] = ch_cal

    # Metadata (words 200-249 = ASCII text)
    meta_chars = []
    for i in range(200, min(250, len(words))):
        lo = words[i] & 0xFF
        hi = (words[i] >> 8) & 0xFF
        if lo == 0:
            break
        meta_chars.append(chr(lo))
        if hi == 0:
            break
        meta_chars.append(chr(hi))
    meta = "".join(meta_chars).strip("\x00")

    # Parse metadata fields (comma-separated: SerialID, DeviceID, ProtocolID, CalCertID)
    parts = meta.split(",") if meta else []
    cal["SerialID"]   = parts[0].strip() if len(parts) > 0 else ""
    cal["DeviceID"]   = parts[1].strip() if len(parts) > 1 else ""
    cal["ProtocolID"] = parts[2].strip() if len(parts) > 2 else ""
    cal["CalCertID"]  = parts[3].strip() if len(parts) > 3 else ""

    return cal


def _words_to_long_unsigned(words: list, idx: int) -> int:
    """Assemble 2 x 16-bit words into an unsigned 32-bit integer."""
    raw = struct.pack("<HH", words[idx] & 0xFFFF, words[idx + 1] & 0xFFFF)
    return struct.unpack("<I", raw)[0]


def _words_to_long_signed(words: list, idx: int) -> int:
    """Assemble 2 x 16-bit words into a signed 32-bit integer."""
    raw = struct.pack("<HH", words[idx] & 0xFFFF, words[idx + 1] & 0xFFFF)
    return struct.unpack("<l", raw)[0]


def _words_to_double(words: list, idx: int) -> float:
    """Assemble 4 x 16-bit words into a 64-bit double."""
    raw = struct.pack(
        "<HHHH",
        words[idx] & 0xFFFF, words[idx + 1] & 0xFFFF,
        words[idx + 2] & 0xFFFF, words[idx + 3] & 0xFFFF,
    )
    return struct.unpack("<d", raw)[0]


def adc_to_amps(signed_adc: int, range_index: int, channel_cal: dict) -> float:
    """Convert a signed 24-bit ADC reading to Amps using calibration data.

    range_index: 0 (200 pA) .. 7 (2 mA)
    channel_cal: the "Ch1" or "Ch2" dict from the calibration file
    """
    json_key = RANGE_TABLE[range_index][3]      # e.g. "Range8" for index 0
    r = channel_cal[json_key]

    adc_pos = r["+ADC"]
    adc_neg = r["-ADC"]
    i_pos   = r["+I"]
    i_neg   = r["-I"]

    # Guard against uncalibrated range (divide by zero)
    if adc_pos == adc_neg:
        return 0.0

    scale  = (i_pos - i_neg) / (adc_pos - adc_neg)
    offset = i_neg - (adc_neg * scale)
    return (signed_adc * scale) + offset

def SPA_Get_Reading(port: str, cal_file: str, parameters: tuple)->tuple:
    channel,range_index,bias,sample_rate,samples=parameters
    polarity = "Negative" if bias < 0 else "Positive"
    enable=(bias != 0)
    spa = SPA(port,cal_file)
    spa.connect()
    spa.set_range(channel,range_index)
    spa.set_sample_rate(sample_rate)
    spa.set_bias(channel,abs(bias),polarity,enable)
    total=0
    for _ in range(samples):
        reading=spa.read_sample()
        total += reading
    reading=total/samples
    spa.disconnect()
    return (reading,polarity,enable)

# =======================================================================
#  SPA class -- high-level interface
# =======================================================================

class SPA:
   
    def __init__(self, port: str, cal_file: str | None = None):
        self.port = port
        self.cal_file = cal_file
        self.cal = None
        self.ser = None
        self._last_tx_time = 0.0

        # Register state (32 x 32-bit values)
        self._regs = [0] * NUM_REGISTERS

        # Per-channel settings
        self._range = {1: 1, 2: 1}             # default: 2 nA (safe)
        self._bias_voltage = {1: 0.0, 2: 0.0}
        self._bias_polarity = {1: "Positive", 2: "Positive"}
        self._bias_enabled = {1: False, 2: False}

        # Sample rate
        self._sample_rate_hz = 10

        # Rolling average buffers
        self._avg_size = 1
        self._avg_buf_ch1 = deque(maxlen=1)
        self._avg_buf_ch2 = deque(maxlen=1)

    # -- Connection ---------------------------------------------

    def connect(self) -> None:
        """Open serial port, load calibration, enable instrument transmit."""
        print(f"[spa] Opening {self.port} at {BAUD_RATE} baud...")
        self.ser = serial.Serial(
            port=self.port,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
            timeout=0.1,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # Load calibration -- try JSON file first, then download from device
        if self.cal_file:
            self.cal = load_calibration_json(self.cal_file)
            if self.cal:
                device_id = self.cal.get("DeviceID", "unknown")
                serial_id = self.cal.get("SerialID", "unknown")
                print(f"[cal] Loaded calibration from {self.cal_file} "
                      f"(device={device_id}, serial={serial_id})")

        if self.cal is None:
            print("[cal] No calibration file found -- downloading from instrument...")
            self.cal = download_calibration(self.ser)
            if self.cal and self.cal_file:
                save_calibration_json(self.cal, self.cal_file)

        if self.cal is None:
            print("[cal] WARNING: No calibration available. "
                  "Current readings will be raw ADC counts, not calibrated Amps.")

        # Send initial configuration with transmit enable
        self._build_registers()
        self._send_config()
        time.sleep(0.1)

        # Flush any stale data
        self.ser.reset_input_buffer()

        print("[spa] Connected and streaming.")

    def disconnect(self) -> None:
        """Disable instrument transmit and close the serial port.

        The LED is kept enabled so it remains on after disconnect.
        The SPA's comms watchdog (bit 17 = 0) will eventually time out
        and put the device into a safe idle state.
        """
        if self.ser and self.ser.is_open:
            # Disable transmit, keep LED on (b12=0 = LED enabled)
            self._regs[REG_CONTROL] = BIT_FLASH_READ_RESET
            self._send_config()
            time.sleep(0.05)
            self.ser.close()
            print("[spa] Disconnected.")

    # -- Configuration ------------------------------------------

    def set_range(self, channel: int, range_index: int) -> None:
        """Set the input current range for a channel.

        Args:
            channel: 1 or 2
            range_index: 0 (200 pA) to 7 (2 mA)
        """
        if range_index < 0 or range_index > 7:
            raise ValueError(f"range_index must be 0..7, got {range_index}")
        self._range[channel] = range_index
        label = RANGE_TABLE[range_index][2]
        print(f"[spa] Ch{channel} range set to {label} (index {range_index})")
        self._build_registers()
        self._send_config()

    def set_sample_rate(self, hz: int) -> None:
        """Set the sample rate (2, 10, or 100 Hz)."""
        if hz not in SAMPLE_RATES:
            raise ValueError(f"Sample rate must be 2, 10, or 100 Hz, got {hz}")
        self._sample_rate_hz = hz
        print(f"[spa] Sample rate set to {hz} Hz")
        self._build_registers()
        self._send_config()

    def set_averaging(self, n: int) -> None:
        """Set the rolling average depth (1 = no averaging, up to 64).

        Averaging is performed in software, not in the instrument hardware.
        """
        n = max(1, min(64, n))
        self._avg_size = n
        self._avg_buf_ch1 = deque(maxlen=n)
        self._avg_buf_ch2 = deque(maxlen=n)
        print(f"[spa] Rolling average set to {n}x")

    def set_bias(self, channel: int, voltage: float,
                 polarity: str = "Positive", enable: bool = False) -> None:
        """Set the bias voltage source for a channel.

        Args:
            channel:  1 or 2
            voltage:  0.0 to 40.0 V
            polarity: "Positive" or "Negative"
            enable:   True to turn on the source output
        """
        voltage = max(0.0, min(40.0, voltage))
        self._bias_voltage[channel] = voltage
        self._bias_polarity[channel] = polarity
        self._bias_enabled[channel] = enable
        state = "ON" if enable else "OFF"
        sign = "+" if polarity == "Positive" else "-"
        print(f"[spa] Ch{channel} bias: {sign}{voltage:.1f} V, source {state}")
        self._build_registers()
        self._send_config()

    # -- Reading ------------------------------------------------

    def read_sample(self) -> dict:
        """Read one sample from the instrument.

        Returns a dict::

            {
                'ch1_raw':  int,    # signed 24-bit ADC
                'ch2_raw':  int,    # signed 24-bit ADC
                'ch1_amps': float,  # calibrated current (Amps)
                'ch2_amps': float,  # calibrated current (Amps)
                'ch1_avg':  float,  # rolling average (Amps)
                'ch2_avg':  float,  # rolling average (Amps)
                'ch1_overload': bool,
                'ch2_overload': bool,
            }
        """
        pkt = self._read_valid_packet()

        # Convert to Amps
        if self.cal:
            ch1_amps = adc_to_amps(pkt["ch1_raw"], self._range[1], self.cal["Ch1"])
            ch2_amps = adc_to_amps(pkt["ch2_raw"], self._range[2], self.cal["Ch2"])
        else:
            ch1_amps = float(pkt["ch1_raw"])
            ch2_amps = float(pkt["ch2_raw"])

        # Rolling average
        self._avg_buf_ch1.append(ch1_amps)
        self._avg_buf_ch2.append(ch2_amps)
        ch1_avg = sum(self._avg_buf_ch1) / len(self._avg_buf_ch1)
        ch2_avg = sum(self._avg_buf_ch2) / len(self._avg_buf_ch2)

        return {
            "ch1_raw": pkt["ch1_raw"],
            "ch2_raw": pkt["ch2_raw"],
            "ch1_amps": ch1_amps,
            "ch2_amps": ch2_amps,
            "ch1_avg": ch1_avg,
            "ch2_avg": ch2_avg,
            "ch1_overload": pkt["ch1_overload"],
            "ch2_overload": pkt["ch2_overload"],
        }

   
    # -- Internal helpers ---------------------------------------

    def _build_registers(self) -> None:
        """Populate the register array from the current settings."""
        # Control register  (LED is on by default when b12=0, so don't set it)
        ctrl = BIT_TRANSMIT_ENABLE | BIT_FLASH_READ_RESET
        self._regs[REG_CONTROL] = ctrl

        # Sample rate
        timer_val, depth = SAMPLE_RATES.get(self._sample_rate_hz, (10000, 16))
        self._regs[REG_SAMPLE_RATE] = timer_val
        self._regs[REG_SAMPLE_DEPTH] = depth

        # Ch1 range
        relay1, pga1 = RANGE_TABLE[self._range[1]][:2]
        self._regs[REG_CH1_RELAY] = relay1
        self._regs[REG_CH1_PGA] = pga1
        self._regs[REG_CH1_SHORT] = 0

        # Ch2 range
        relay2, pga2 = RANGE_TABLE[self._range[2]][:2]
        self._regs[REG_CH2_RELAY] = relay2
        self._regs[REG_CH2_PGA] = pga2
        self._regs[REG_CH2_SHORT] = 0

        # Bias supply
        self._regs[REG_CH1_BIAS] = self._calc_dac(1)
        self._regs[REG_CH2_BIAS] = self._calc_dac(2)

    def _calc_dac(self, channel: int) -> int:
        """Calculate the DAC register value for the bias supply."""
        if self.cal is None:
            # Use defaults (from EPIC source: pos=20000, neg=10000)
            dac_pos = 20000
            dac_neg = 10000
        else:
            ch_key = f"Ch{channel}"
            src = self.cal.get(ch_key, {}).get("Source", {})
            dac_pos = src.get("+DAC", 20000)
            dac_neg = src.get("-DAC", 10000)

        offset = (dac_pos + dac_neg) / 2.0
        gain = (dac_pos - dac_neg) / 80.0      # 80 = (+40) - (-40)

        voltage = self._bias_voltage[channel]
        if not self._bias_enabled[channel]:
            voltage = 0.0

        sign = 1.0 if self._bias_polarity[channel] == "Positive" else -1.0
        return int((voltage * sign * gain) + offset)

    def _send_config(self) -> None:
        """Send the full 256-byte register block to the instrument."""
        self.ser.write(build_config_block(self._regs))
        self._last_tx_time = time.time()

    def _send_keepalive(self) -> None:
        """Send a read/keepalive packet if needed."""
        self.ser.write(build_read_packet())
        self._last_tx_time = time.time()

    def _read_valid_packet(self) -> dict:
        """Read bytes until a valid 16-byte packet is received.

        Automatically sends keep-alive packets if idle too long.
        """
        buf = bytearray()
        bad_streak = 0
        deadline = time.time() + CONNECTION_TIMEOUT_MS / 1000.0

        while time.time() < deadline:
            # Keep-alive
            elapsed_ms = (time.time() - self._last_tx_time) * 1000
            if elapsed_ms > KEEPALIVE_INTERVAL_MS:
                self._send_keepalive()

            # Read available bytes
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                buf.extend(chunk)

            while len(buf) >= RX_PACKET_SIZE:
                pkt = parse_response(bytes(buf[:RX_PACKET_SIZE]))
                if pkt is not None:
                    buf = buf[RX_PACKET_SIZE:]
                    bad_streak = 0
                    # Skip cal-data packets during normal operation
                    if not pkt["cal_data_flag"]:
                        return pkt
                else:
                    # Bad checksum -- shift by one byte to attempt re-sync
                    bad_streak += 1
                    buf.pop(0)

        raise TimeoutError("No valid packet received within timeout")





