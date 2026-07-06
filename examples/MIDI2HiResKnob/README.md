# MIDI2HiResKnob

AS5600 encoder as a high-resolution USB MIDI controller.

This folder supports two build paths:

- `MIDI2HiResKnob.ino` is the Arduino IDE compatible version. It sends a MIDI
  1.0 high-resolution 14-bit Control Change pair via the Arduino MIDIUSB
  library.
- `src/main.cpp` is the PlatformIO Pico 2 version. It sends true MIDI 2.0 UMP
  with 32-bit Control Change values via the tusb_ump TinyUSB class driver.

Slow turns produce steps far below what 7-bit MIDI 1.0 can express — smooth,
zipper-free parameter sweeps. Fast turns cover the whole range in about a
quarter turn (the MagEncoder velocity curve).

## What it sends

| Build | Host type | Data |
|---|---|---|
| Arduino IDE / MIDIUSB | USB MIDI 1.0 host | 14-bit CC pair: CC 1 (MSB) + CC 33 (LSB) |
| PlatformIO / tusb_ump | MIDI 2.0 / UMP host | MIDI 2.0 CC 1, channel 1, 32-bit value |
| PlatformIO / tusb_ump | Legacy MIDI 1.0 host | 14-bit CC pair: CC 1 (MSB) + CC 33 (LSB) |

The PlatformIO build checks `tud_alt_setting()` and downgrades itself for MIDI
1.0 hosts — the tusb_ump driver drops MIDI 2.0 Channel Voice packets in that
mode, so this is not optional. It also answers UMP Endpoint Discovery (MT=0xF
stream messages) via `ump_stream_handler.h`, which UMP hosts require before
they will create the endpoint.

In the PlatformIO build, the 0..1 knob value is quantized to 16 bits and
upscaled to 32 bits by bit replication, the min-center-max upscaling the MIDI
2.0 spec recommends.

## Wiring

| AS5600 | Pico 2 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GP4 (default Wire SDA) |
| SCL | GP5 (default Wire SCL) |

LED: solid = USB mounted, fast blink = AS5600 not found.

## Arduino IDE

1. Install this AS5600 library.
2. Install **MIDIUSB** from Library Manager.
3. Open **File > Examples > AS5600 > MIDI2HiResKnob**.
4. Select a native USB board supported by MIDIUSB and upload.

## PlatformIO build & flash

The MIDI 2.0 UMP version is a PlatformIO project: the tusb_ump driver must be
enabled with `-DCFG_TUD_UMP=1` at compile time, which the Arduino IDE cannot
pass to the arduino-pico core.

```
cd examples/MIDI2HiResKnob
pio run                 # build
pio run -t upload       # flash (or drag the UF2 from .pio/build/ onto RPI-RP2)
pio device monitor      # 115200 baud debug output
```

Dependencies (fetched automatically): the parent AS5600/MagEncoder library via
symlink, and tusb_ump from GitHub.

## Testing

- **Windows 11 + Windows MIDI Services**: the device enumerates as
  "AS5600 HiRes"; use the MIDI Console (`midi enum`, `midi monitor`) to watch
  32-bit CC values.
- **macOS 14+**: Audio MIDI Setup shows a MIDI 2.0 endpoint; monitor with
  [MIDI Monitor](https://www.snoize.com/MIDIMonitor/).
- **Any MIDI 1.0 host**: appears as a normal USB MIDI device sending 14-bit
  CC 1/33 — map it in any DAW.

## Tuning the feel

Pass a `MagEncoder::Config` to the encoder constructor in `main.cpp` — see the
library README for the velocity-curve parameters (`minScale`, `maxScale`,
`curveExponent`, …).
