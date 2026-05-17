"""
SPA100 / SPA120 Source Picoammeter -- Python Communication Example
=================================================================

More info: www.electron.plus

Demonstrates how to communicate with an Electron Plus SPA100 or SPA120
Source Picoammeter over USB serial.  This script is a complete, working
example that can be used as-is or adapted to your own measurement code.

Supports:
  * SPA100 (single-channel picoammeter)
  * SPA120 (dual-channel picoammeter, independent range per channel)

Features:
  * Auto-detect the instrument's COM port (CH340 USB-serial bridge)
  * Load factory calibration from a JSON file, or download it directly
    from the instrument's flash memory and save it for future use
  * Set the input current range (200 pA to 2 mA, 8 ranges)
  * Set the sample rate (2, 10, or 100 Hz)
  * Software rolling average (1x to 64x)
  * Bias voltage source control (0-40 V, positive or negative)
  * Read calibrated current measurements with SI-prefix formatting
  * Run continuously (--samples 0) or for a fixed number of samples
  * Ctrl+C to stop at any time -- the instrument is left in a safe state
  * Dry-run self-test mode (--dry-run) for verifying protocol logic
    without any hardware connected

Requirements:
  * Python 3.7 or later
  * pyserial library:   pip install pyserial
  * No other dependencies

Quick start:
  1. Connect the SPA to your computer via USB
  2. Install pyserial:   pip install pyserial
  3. Run:  python SPA_python_example.py --range 5 --samples 20

  The script will auto-detect the COM port and calibration.  For more
  options, run:  python SPA_python_example.py --help

How communication works:
  The SPA uses a CH340 USB-to-serial bridge.  All communication is at
  115200 baud, 8 data bits, no parity, 1 stop bit, no flow control.

  To configure the instrument, the host sends a block of 32 register
  writes (256 bytes total).  Each register write is 8 bytes: a 2-byte
  address (with the MSB set to indicate write), 4 bytes of data, and a
  2-byte checksum.  Registers are sent in reverse order ($1F down to $00).

  Once configured, the instrument streams 16-byte measurement packets at
  the selected sample rate.  Each packet contains status flags, two
  24-bit signed ADC readings (Ch1 and Ch2), and a 1-byte checksum.

  A keep-alive read packet must be sent every 200 ms to prevent the
  instrument's comms watchdog from timing out and resetting.

Calibration:
  Each SPA is factory-calibrated with known reference currents.  The
  calibration data is stored in the instrument's flash memory and also
  saved to a JSON file on disk.

  The calibration consists of two reference points per range per channel:
  a positive and negative ADC reading at known positive and negative
  currents.  The script uses linear interpolation between these points
  to convert raw ADC counts into calibrated Amps.

  On first run, if no JSON calibration file is found, the script will
  automatically download the calibration from the instrument (a ~2-second
  process), verify it, and save it to disk for future sessions.

Protocol reference:
  EPIC_V26006.pb -- the SPA register map, packet format, and calibration
  routines were derived from the official EPIC application source code.

Copyright 2024-2026 Electron Plus Instruments Limited.
"""

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


# =======================================================================
#  SPA class -- high-level interface
# =======================================================================

class SPA:
    """High-level interface to an SPA100 or SPA120 Source Picoammeter.

    Usage::

        spa = SPA("COM3", cal_file="settings/SPA120_cal.JSON")
        spa.connect()
        spa.set_range(1, 5)         # Ch1, 20 uA range
        spa.set_sample_rate(10)     # 10 Hz
        spa.set_bias(1, 5.0, "Positive", True)   # 5 V bias on Ch1

        for _ in range(100):
            s = spa.read_sample()
            print(f"Ch1: {SPA.format_current(s['ch1_amps'])}")

        spa.disconnect()
    """

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

    # -- Formatting ---------------------------------------------

    @staticmethod
    def format_current(amps: float) -> str:
        """Format a current value with the appropriate SI prefix."""
        if amps == 0.0:
            return "0.000  A"
        absval = abs(amps)
        if absval >= 1e-3:
            return f"{amps * 1e3:+8.4f} mA"
        elif absval >= 1e-6:
            return f"{amps * 1e6:+8.4f} uA"
        elif absval >= 1e-9:
            return f"{amps * 1e9:+8.4f} nA"
        else:
            return f"{amps * 1e12:+8.4f} pA"

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


# =======================================================================
#  COM port auto-detection
# =======================================================================

def find_spa_port() -> str | None:
    """Try to find a CH340 serial port that likely has an SPA connected."""
    if not HAS_SERIAL:
        return None
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        mfg  = (p.manufacturer or "").lower()
        if "ch340" in desc or "ch340" in mfg or "wch" in mfg:
            return p.device
    return None


# =======================================================================
#  Dry-run / self-test  (no hardware required)
# =======================================================================

def _build_synthetic_rx_packet(ch1_raw: int, ch2_raw: int,
                               status: int = 0, usbcal: int = 0) -> bytes:
    """Build a valid 16-byte RX packet with known ADC values (for testing)."""
    # Ensure unsigned 24-bit representation
    ch1_u = ch1_raw & 0xFFFFFF
    ch2_u = ch2_raw & 0xFFFFFF
    pkt = bytes([
        (status >> 8) & 0xFF, status & 0xFF,         # status
        (usbcal >> 8) & 0xFF, usbcal & 0xFF,         # usbcal
        0x00, 0x00,                                    # adc0 (unused)
        (ch1_u >> 16) & 0xFF, (ch1_u >> 8) & 0xFF, ch1_u & 0xFF,   # ch1
        (ch2_u >> 16) & 0xFF, (ch2_u >> 8) & 0xFF, ch2_u & 0xFF,   # ch2
        0x00, 0x00, 0x00,                              # adc3 (unused)
        0x00,                                          # placeholder checksum
    ])
    pkt_arr = bytearray(pkt)
    pkt_arr[15] = sum(pkt_arr[:15]) & 0xFF
    return bytes(pkt_arr)


def run_dry_run(cal_file: str) -> None:
    """Run a comprehensive self-test of all protocol logic without hardware.

    Tests:  packet building, checksums, parsing, 24-bit sign extension,
            calibration loading, ADC->Amps conversion, DAC calculation,
            byte-sync recovery, rolling average, SI-prefix formatting.
    """
    passed = 0
    failed = 0
    total  = 0

    def check(name: str, condition: bool, detail: str = ""):
        nonlocal passed, failed, total
        total += 1
        if condition:
            passed += 1
            print(f"  [PASS] {name}")
        else:
            failed += 1
            print(f"  [FAIL] {name}  -- {detail}")

    print()
    print("=" * 64)
    print("  More info: www.electron.plus")
    print("=" * 64)
    print("  SPA Python Example -- Dry-Run Self-Test")
    print("  No hardware required.  Testing protocol logic only.")
    print("=" * 64)

    # -- 1. TX packet building --------------------------------------
    print("\n-- TX Packet Building --")

    pkt = build_write_packet(0x01, 0x00010000)
    check("Write packet length = 8", len(pkt) == 8, f"got {len(pkt)}")
    check("Write packet MSB set (write flag)", pkt[0] & 0x80 != 0)

    # Verify checksum manually: seed + addr_word + data_hi + data_lo
    addr_word = (pkt[0] << 8) | pkt[1]
    data_hi   = (pkt[2] << 8) | pkt[3]
    data_lo   = (pkt[4] << 8) | pkt[5]
    cs_calc   = (TX_CHECKSUM_SEED + addr_word + data_hi + data_lo) & 0xFFFF
    cs_actual = (pkt[6] << 8) | pkt[7]
    check("TX checksum correct", cs_calc == cs_actual,
          f"expected 0x{cs_calc:04X}, got 0x{cs_actual:04X}")

    # Read packet (keepalive)
    rpkt = build_read_packet(0)
    check("Read packet length = 8", len(rpkt) == 8)
    check("Read packet MSB clear (read flag)", rpkt[0] & 0x80 == 0)
    check("Read packet data bytes = 0",
          rpkt[2] == 0 and rpkt[3] == 0 and rpkt[4] == 0 and rpkt[5] == 0)

    # Full config block
    regs = [0] * NUM_REGISTERS
    regs[REG_CONTROL] = BIT_TRANSMIT_ENABLE | BIT_FLASH_READ_RESET
    regs[REG_SAMPLE_RATE] = 10000
    regs[REG_SAMPLE_DEPTH] = 16
    config = build_config_block(regs)
    check("Config block = 256 bytes", len(config) == FULL_PACKET_SIZE,
          f"got {len(config)}")
    # First 8 bytes should be register $1F (highest address, sent first)
    check("Config block starts with register $1F",
          config[0] == (0x80 | 0x00) and config[1] == 0x1F)
    # Last 8 bytes should be register $00
    check("Config block ends with register $00",
          config[248] == 0x80 and config[249] == 0x00)

    # -- 2. RX packet parsing --------------------------------------
    print("\n-- RX Packet Parsing --")

    # Known positive value
    pkt_pos = _build_synthetic_rx_packet(ch1_raw=100000, ch2_raw=-50000)
    result = parse_response(pkt_pos)
    check("Parse valid packet returns dict", result is not None)
    check("Ch1 raw = +100000", result["ch1_raw"] == 100000,
          f"got {result['ch1_raw']}")
    check("Ch2 raw = -50000", result["ch2_raw"] == -50000,
          f"got {result['ch2_raw']}")
    check("No overload at 100000", not result["ch1_overload"])

    # Overload detection
    pkt_ovr = _build_synthetic_rx_packet(ch1_raw=8_100_000, ch2_raw=0)
    result_ovr = parse_response(pkt_ovr)
    check("Overload detected at 8.1M", result_ovr["ch1_overload"])

    # Bad checksum
    bad_pkt = bytearray(pkt_pos)
    bad_pkt[15] ^= 0xFF
    check("Bad checksum returns None", parse_response(bytes(bad_pkt)) is None)

    # Too short
    check("Short packet returns None", parse_response(b"\x00" * 10) is None)

    # Cal data flags
    pkt_cal = _build_synthetic_rx_packet(0, 0, status=(1 << 12) | (1 << 13), usbcal=0x1234)
    result_cal = parse_response(pkt_cal)
    check("Cal data flag detected", result_cal["cal_data_flag"])
    check("Cal sync flag detected", result_cal["cal_sync_flag"])
    check("USBcal value = 0x1234", result_cal["usbcal"] == 0x1234,
          f"got 0x{result_cal['usbcal']:04X}")

    # -- 3. 24-bit sign extension ----------------------------------
    print("\n-- 24-bit Sign Extension --")
    check("0x000000 -> 0", _sign_extend_24(0x000000) == 0)
    check("0x7FFFFF -> +8388607", _sign_extend_24(0x7FFFFF) == 8388607)
    check("0x800000 -> -8388608", _sign_extend_24(0x800000) == -8388608)
    check("0xFFFFFF -> -1", _sign_extend_24(0xFFFFFF) == -1)
    check("0x800001 -> -8388607", _sign_extend_24(0x800001) == -8388607)

    # -- 4. Calibration loading ------------------------------------
    print("\n-- Calibration File Loading --")

    cal = load_calibration_json(cal_file)
    if cal is None:
        print(f"  [SKIP] Could not load {cal_file} -- skipping cal-dependent tests")
        print(f"         (Copy SPA120_cal.JSON to the expected path to enable)")
    else:
        check("Calibration loaded OK", cal is not None)
        check("Has Ch1", "Ch1" in cal)
        check("Has Ch2", "Ch2" in cal)
        check("Ch1 has all 8 ranges",
              all(f"Range{r}" in cal["Ch1"] for r in range(1, 9)))
        check("Ch2 has all 8 ranges",
              all(f"Range{r}" in cal["Ch2"] for r in range(1, 9)))
        check("Has SerialID", "SerialID" in cal,
              f"serial = {cal.get('SerialID', '?')}")
        check("Has DeviceID", "DeviceID" in cal,
              f"device = {cal.get('DeviceID', '?')}")
        print(f"         Device: {cal.get('DeviceID')}, "
              f"Serial: {cal.get('SerialID')}, "
              f"CalCert: {cal.get('CalCertID')}")

        # -- 5. ADC -> Amps conversion ------------------------------
        print("\n-- ADC -> Amps Conversion (all 8 ranges, Ch1) --")
        for idx in range(8):
            json_key = RANGE_TABLE[idx][3]
            label    = RANGE_TABLE[idx][2]
            r_cal = cal["Ch1"][json_key]

            # At zero ADC, current should be near zero
            amps_zero = adc_to_amps(0, idx, cal["Ch1"])
            # At +ADC calibration point, current should equal +I
            amps_pos = adc_to_amps(r_cal["+ADC"], idx, cal["Ch1"])
            amps_neg = adc_to_amps(r_cal["-ADC"], idx, cal["Ch1"])

            pos_ok = abs(amps_pos - r_cal["+I"]) < abs(r_cal["+I"]) * 1e-6
            neg_ok = abs(amps_neg - r_cal["-I"]) < abs(r_cal["-I"]) * 1e-6

            check(f"Range {idx} ({label:>7s}) +cal point",
                  pos_ok,
                  f"expected {r_cal['+I']:.6e}, got {amps_pos:.6e}")
            check(f"Range {idx} ({label:>7s}) -cal point",
                  neg_ok,
                  f"expected {r_cal['-I']:.6e}, got {amps_neg:.6e}")

            # Zero-crossing check: at ADC=0 the current should be small
            # relative to full-scale.  Asymmetric cal points (different
            # +ADC/-ADC magnitudes) naturally produce a small offset, so
            # a 3% tolerance is appropriate for two-point calibration.
            full_scale = max(abs(r_cal["+I"]), abs(r_cal["-I"]))
            zero_frac = abs(amps_zero) / full_scale if full_scale > 0 else 0
            check(f"Range {idx} ({label:>7s}) zero offset < 3%",
                  zero_frac < 0.03,
                  f"offset = {zero_frac*100:.3f}% of FS")

        # -- 6. DAC calculation -------------------------------------
        print("\n-- DAC Bias Calculation --")

        # Create a minimal SPA instance (no serial port) to test DAC maths
        spa_test = SPA.__new__(SPA)
        spa_test.cal = cal
        spa_test._bias_voltage = {1: 0.0, 2: 0.0}
        spa_test._bias_polarity = {1: "Positive", 2: "Positive"}
        spa_test._bias_enabled = {1: False, 2: False}

        # At 0 V (disabled), DAC should be near the midpoint (offset)
        dac_zero = spa_test._calc_dac(1)
        src = cal["Ch1"]["Source"]
        expected_offset = int((src["+DAC"] + src["-DAC"]) / 2.0)
        check("DAC at 0V ~= midpoint",
              abs(dac_zero - expected_offset) <= 1,
              f"expected ~{expected_offset}, got {dac_zero}")

        # At +40V, DAC should be near +DAC calibration
        spa_test._bias_voltage[1] = 40.0
        spa_test._bias_enabled[1] = True
        spa_test._bias_polarity[1] = "Positive"
        dac_max = spa_test._calc_dac(1)
        check("DAC at +40V ~= +DAC cal point",
              abs(dac_max - src["+DAC"]) <= 1,
              f"expected ~{src['+DAC']}, got {dac_max}")

        # At -40V, DAC should be near -DAC calibration
        spa_test._bias_polarity[1] = "Negative"
        dac_min = spa_test._calc_dac(1)
        check("DAC at -40V ~= -DAC cal point",
              abs(dac_min - src["-DAC"]) <= 1,
              f"expected ~{src['-DAC']}, got {dac_min}")

    # -- 7. SI-prefix formatting -----------------------------------
    print("\n-- SI-Prefix Formatting --")
    check("2 mA format",    "mA" in SPA.format_current(2.0e-3))
    check("1.5 uA format",  "uA" in SPA.format_current(1.5e-6))
    check("3.7 nA format",  "nA" in SPA.format_current(3.7e-9))
    check("45 pA format",   "pA" in SPA.format_current(45e-12))
    check("0 A format",     "A"  in SPA.format_current(0.0))
    check("Negative value",  "-" in SPA.format_current(-1.23e-6))

    # -- 8. Byte-sync recovery test --------------------------------
    print("\n-- Byte-Sync Recovery --")

    good_pkt = _build_synthetic_rx_packet(ch1_raw=12345, ch2_raw=-6789)
    # Prepend 3 garbage bytes -- parser should skip them and find the packet
    garbage = bytes([0xDE, 0xAD, 0xBE])
    stream = garbage + good_pkt

    # Simulate what _read_valid_packet does: scan through the buffer
    buf = bytearray(stream)
    found = None
    shifts = 0
    while len(buf) >= RX_PACKET_SIZE:
        result = parse_response(bytes(buf[:RX_PACKET_SIZE]))
        if result is not None:
            found = result
            break
        buf.pop(0)
        shifts += 1
    check("Found valid packet after garbage",
          found is not None and found["ch1_raw"] == 12345,
          f"shifts={shifts}, found={found}")
    check("Required exactly 3 byte shifts", shifts == 3, f"got {shifts}")

    # -- 9. Rolling average ----------------------------------------
    print("\n-- Rolling Average --")

    avg_buf = deque(maxlen=4)
    values = [1.0e-9, 2.0e-9, 3.0e-9, 4.0e-9, 5.0e-9]
    for v in values:
        avg_buf.append(v)
    avg = sum(avg_buf) / len(avg_buf)
    expected_avg = (2.0e-9 + 3.0e-9 + 4.0e-9 + 5.0e-9) / 4.0  # last 4 values
    check("Rolling avg (depth=4) correct",
          abs(avg - expected_avg) < 1e-15,
          f"expected {expected_avg:.3e}, got {avg:.3e}")

    # -- 10. Word assembly (cal download primitives) ---------------
    print("\n-- Word Assembly (cal download primitives) --")

    # Signed long: pack a known value, verify round-trip
    test_val_signed = -1234567
    packed = struct.pack("<l", test_val_signed)
    w0, w1 = struct.unpack("<HH", packed)
    reconstructed = _words_to_long_signed([w0, w1], 0)
    check("Signed long round-trip",
          reconstructed == test_val_signed,
          f"expected {test_val_signed}, got {reconstructed}")

    # Unsigned long
    test_val_unsigned = 3000000
    packed_u = struct.pack("<I", test_val_unsigned)
    w0u, w1u = struct.unpack("<HH", packed_u)
    reconstructed_u = _words_to_long_unsigned([w0u, w1u], 0)
    check("Unsigned long round-trip",
          reconstructed_u == test_val_unsigned,
          f"expected {test_val_unsigned}, got {reconstructed_u}")

    # Double
    test_val_double = -1.998833e-05
    packed_d = struct.pack("<d", test_val_double)
    w0d, w1d, w2d, w3d = struct.unpack("<HHHH", packed_d)
    reconstructed_d = _words_to_double([w0d, w1d, w2d, w3d], 0)
    check("Double round-trip",
          reconstructed_d == test_val_double,
          f"expected {test_val_double}, got {reconstructed_d}")

    # -- 11. Simulated reading session -----------------------------
    if cal:
        print("\n-- Simulated Reading Session (10 samples, range 5 = 20 uA) --")
        range_idx = 5
        label = RANGE_TABLE[range_idx][2]
        print(f"   Range: {label}")
        print(f"   {'#':>4}   {'Ch1 Current':>16}   {'Ch2 Current':>16}")
        print(f"   {'-'*4}   {'-'*16}   {'-'*16}")

        for i in range(10):
            # Generate a random ADC value within +/-half full-scale
            adc_ch1 = random.randint(-4_000_000, 4_000_000)
            adc_ch2 = random.randint(-4_000_000, 4_000_000)

            # Build and parse a synthetic packet (proves the round-trip)
            pkt = _build_synthetic_rx_packet(ch1_raw=adc_ch1, ch2_raw=adc_ch2)
            parsed = parse_response(pkt)
            assert parsed is not None, "Synthetic packet failed to parse"
            assert parsed["ch1_raw"] == adc_ch1, "Ch1 ADC mismatch in round-trip"
            assert parsed["ch2_raw"] == adc_ch2, "Ch2 ADC mismatch in round-trip"

            # Convert to Amps
            ch1_amps = adc_to_amps(parsed["ch1_raw"], range_idx, cal["Ch1"])
            ch2_amps = adc_to_amps(parsed["ch2_raw"], range_idx, cal["Ch2"])

            ch1_str = SPA.format_current(ch1_amps)
            ch2_str = SPA.format_current(ch2_amps)
            print(f"   {i+1:4d}   {ch1_str:>16}   {ch2_str:>16}")

    # -- Summary ---------------------------------------------------
    print()
    print("=" * 64)
    if failed == 0:
        print(f"  ALL {total} TESTS PASSED")
    else:
        print(f"  {passed}/{total} passed, {failed} FAILED")
    print("=" * 64)
    print()

    return failed == 0


# =======================================================================
#  Main demo
# =======================================================================

def main():
    parser = argparse.ArgumentParser(
        description="SPA100/SPA120 Source Picoammeter -- Python example",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  python SPA_python_example.py --dry-run                                        (self-test, no hardware)
  python SPA_python_example.py --port COM3 --range 5 --rate 10 --samples 20     (auto-detect SPA type)
  python SPA_python_example.py --port COM3 --device SPA100 --range 5            (SPA100, single channel)
  python SPA_python_example.py --port COM3 --range 5 --range2 3 --samples 50    (SPA120, different ranges)
  python SPA_python_example.py --port COM3 --range 7 --bias 5.0 --samples 100   (with bias source)
  python SPA_python_example.py --port COM3 --range 5 --samples 0               (run forever, Ctrl+C to stop)
  python SPA_python_example.py                                                  (auto-detect everything)
""",
    )
    parser.add_argument("--dry-run", action="store_true",
                        help="Run self-test (packet, checksum, cal, conversion) "
                             "without hardware")
    parser.add_argument("--device", choices=["SPA100", "SPA120"],
                        help="Device type (default: auto-detect from cal file)")
    parser.add_argument("--port", help="COM port (e.g. COM3). Auto-detected if omitted.")
    parser.add_argument("--cal", help="Calibration JSON file path",
                        default=None)
    parser.add_argument("--range", type=int, default=5,
                        help="Ch1 input range: 0=200pA, 1=2nA, 2=20nA, 3=200nA, "
                             "4=2uA, 5=20uA, 6=200uA, 7=2mA (default: 5)")
    parser.add_argument("--range2", type=int, default=None,
                        help="Ch2 input range (SPA120 only, default: same as --range)")
    parser.add_argument("--rate", type=int, default=10, choices=[2, 10, 100],
                        help="Sample rate in Hz (default: 10)")
    parser.add_argument("--avg", type=int, default=1,
                        help="Rolling average depth, 1-64 (default: 1)")
    parser.add_argument("--bias", type=float, default=0.0,
                        help="Bias voltage in Volts, 0-40 (default: 0 = off)")
    parser.add_argument("--polarity", choices=["Positive", "Negative"],
                        default="Positive", help="Bias polarity (default: Positive)")
    parser.add_argument("--samples", type=int, default=20,
                        help="Number of samples to read (default: 20). "
                             "Use 0 for continuous (Ctrl+C to stop)")
    args = parser.parse_args()

    # Determine device type and cal file defaults
    device = args.device            # may be None (auto-detect)
    cal_file = args.cal
    if cal_file is None:
        # Pick a default cal file based on device type (or SPA120 as fallback)
        if device == "SPA100":
            cal_file = "settings/SPA100_cal.JSON"
        else:
            cal_file = "settings/SPA120_cal.JSON"

    # Dry-run mode -- self-test, no hardware needed
    if args.dry_run:
        ok = run_dry_run(cal_file=cal_file)
        sys.exit(0 if ok else 1)

    # Hardware mode -- pyserial required
    if not HAS_SERIAL:
        print("ERROR: pyserial is not installed.  Run:  pip install pyserial")
        print("       (or use --dry-run to test without hardware)")
        sys.exit(1)

    # Find port
    port = args.port
    if not port:
        port = find_spa_port()
        if not port:
            print("ERROR: No SPA instrument found. Specify --port manually.")
            sys.exit(1)
        print(f"[spa] Auto-detected port: {port}")

    # Create SPA instance and connect
    spa = SPA(port, cal_file=cal_file)
    try:
        spa.connect()

        # Auto-detect device type from calibration if not specified
        if device is None and spa.cal:
            device = spa.cal.get("DeviceID", "SPA120")
            print(f"[spa] Auto-detected device: {device}")
        elif device is None:
            device = "SPA120"       # safe default (superset)
        dual_channel = (device == "SPA120")

        # Set ranges
        spa.set_range(1, args.range)
        if dual_channel:
            range2 = args.range2 if args.range2 is not None else args.range
            spa.set_range(2, range2)

        spa.set_sample_rate(args.rate)
        spa.set_averaging(args.avg)

        if args.bias > 0:
            spa.set_bias(1, args.bias, args.polarity, True)

        # Header
        range1_label = RANGE_TABLE[args.range][2]
        continuous = (args.samples == 0)
        sample_desc = "continuous (Ctrl+C to stop)" if continuous else f"{args.samples} samples"
        print(f"\n{'='*60}")
        print(f"  More info: www.electron.plus")
        print(f"{'='*60}")
        print(f"  {device}, {sample_desc} at {args.rate} Hz, avg {args.avg}x")
        if dual_channel and args.range2 is not None and args.range2 != args.range:
            range2_label = RANGE_TABLE[args.range2][2]
            print(f"  Ch1 range: {range1_label},  Ch2 range: {range2_label}")
        else:
            print(f"  Range: {range1_label}")
        if args.bias > 0:
            sign = "+" if args.polarity == "Positive" else "-"
            print(f"  Bias: {sign}{args.bias:.1f} V (source ON)")
        print(f"{'='*60}\n")

        if dual_channel:
            print(f"  {'#':>4}   {'Ch1 Current':>16}   {'Ch2 Current':>16}   "
                  f"{'Ch1 Avg':>16}   {'Ch2 Avg':>16}")
            print(f"  {'-'*4}   {'-'*16}   {'-'*16}   {'-'*16}   {'-'*16}")
        else:
            print(f"  {'#':>4}   {'Ch1 Current':>16}   {'Ch1 Avg':>16}")
            print(f"  {'-'*4}   {'-'*16}   {'-'*16}")

        i = 0
        count = args.samples       # 0 = continuous
        while count == 0 or i < count:
            sample = spa.read_sample()
            i += 1

            ch1_str = "** OVR **" if sample["ch1_overload"] else SPA.format_current(sample["ch1_amps"])
            avg1_str = SPA.format_current(sample["ch1_avg"])

            if dual_channel:
                ch2_str = "** OVR **" if sample["ch2_overload"] else SPA.format_current(sample["ch2_amps"])
                avg2_str = SPA.format_current(sample["ch2_avg"])
                print(f"  {i:4d}   {ch1_str:>16}   {ch2_str:>16}   "
                      f"{avg1_str:>16}   {avg2_str:>16}")
            else:
                print(f"  {i:4d}   {ch1_str:>16}   {avg1_str:>16}")

        print(f"\n  Done -- {i} samples captured.\n")

    except KeyboardInterrupt:
        n = locals().get("i", 0)
        print(f"\n  Stopped by user after {n} samples.")
    except TimeoutError as e:
        print(f"\n  ERROR: {e}")
    finally:
        spa.disconnect()


if __name__ == "__main__":
    main()
