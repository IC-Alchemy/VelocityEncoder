# AS5600

A knob is still a hand on a shaft. This is the Arduino driver we ship with our
**[ENCODER NAME]** magnetic encoder — built around the **AS5600** 12-bit
contactless sensor, and tuned for instruments you play rather than things you
program.

It does the basics you would expect from an AS5600 library, and one thing you
probably would not: it knows how fast you are turning. Slow motion gives fine
control; fast motion covers range. That is the whole reason this driver exists.

## Features

- **Raw and normalized angle** — the native 12-bit value (0–4095) and a 0.0–1.0
  reading.
- **Multi-turn cumulative position** with 12-bit wrap-around correction. Several
  revolutions in the same direction add up instead of jumping back at 4096. Use
  it for endless-encoder UIs.
- **Filtered angular speed** in degrees/second. An adaptive low-pass filter
  smooths less when you move fast and more when you move slow, with a noise gate
  that keeps the reading still when the knob is still.
- **Velocity-scaled parameter increment** (`getParameterIncrement()`): turn slow
  for fine adjustment, fast for a wide sweep. The response curve is fully tunable
  via `MagEncoder::Config`.
- **Connection detection**, so a sketch can degrade gracefully when no magnet is
  fitted.
- **No application dependencies** — only `Wire` and the Arduino core.

## Hardware

| AS5600 pin | Connect to |
|---|---|
| VCC | 3.3V or 5V (check your breakout's regulator) |
| GND | GND |
| SDA | board SDA |
| SCL | board SCL |

The AS5600's default I2C address is **0x36**. Pass a different value in the
`Config` if your breakout has been readdressed.

## Installation

In the Arduino IDE, use **Sketch → Include Library → Add .ZIP Library…** on a
zip of this folder, or drop the `AS5600/` folder into your `libraries/`
directory. The library is architecture-independent (`architectures=*`) and builds
on any core that provides a working `TwoWire`.

## Quick start

```cpp
#include <MagEncoder.h>

MagEncoder encoder;

void setup()
{
    Serial.begin(115200);
    if (!encoder.begin())
    {
        Serial.println("AS5600 not found.");
        while (true) { delay(1000); }
    }
}

void loop()
{
    encoder.update();
    Serial.println(encoder.getNormalizedAngle(), 3);
    delay(50);
}
```

## Velocity-sensitive parameter control

The headline feature: turn slowly, get fine control; turn fast, cover range.
Pass a `[min, max]` range and how many full turns should span it:

```cpp
float parameter = 0.0f;

void loop()
{
    encoder.update();

    float increment = encoder.getParameterIncrement(0.0f, 1.0f, 4);
    parameter = constrain(parameter + increment, 0.0f, 1.0f);

    Serial.println(parameter, 3);
    delay(5);
}
```

The curve that maps angular speed to a per-tick multiplier is tunable via
`MagEncoder::Config`:

```cpp
MagEncoder::Config cfg;
cfg.minVelDps         = 90.0f;   // below this, multiplier = minScale
cfg.maxVelDps         = 2400.0f; // above this, multiplier = maxScale
cfg.minScale          = 0.008f;  // slow-turn multiplier
cfg.maxScale          = 3.2f;    // fast-turn multiplier
cfg.curveExponent     = 1.8f;    // mid-range curve shape
cfg.velocitySmoothing = 0.08f;   // EMA factor (smaller = smoother, laggier)

MagEncoder encoder(cfg);
```

These defaults are the curve we use in our own instruments. They are a starting
point, not a verdict — every knob feels different, and every player has a
preference.

## API reference

### `MagEncoder::Config`

A struct of tunables with sensible defaults. See the snippet above.

### Setup / polling

| Method | Description |
|---|---|
| `bool begin()` | Initialize I2C and detect the sensor. Returns false if not present. |
| `void update()` | Read the sensor and refresh derived state. Throttled to `readIntervalMs`. Call every loop. |
| `bool isConnected()` | Returns whether `begin()` detected the sensor. |

### Read-only state

| Method | Description |
|---|---|
| `uint16_t getRawAngle()` | 12-bit raw angle (0–4095). |
| `float getNormalizedAngle()` | Raw angle normalized to 0.0–1.0. |
| `int32_t getCumulativePosition()` | Multi-turn position with wrap-around unwrapped. |
| `float getAngularSpeed()` | Filtered angular speed in °/s. |
| `float getPositionPercentage(maxRotations)` | Cumulative position as 0.0–1.0 across N turns. |
| `VelocityZone getVelocityZone()` | Coarse `Idle`/`Low`/`Mid`/`High` qualitative reading. |

### Parameter control

| Method | Description |
|---|---|
| `float getParameterIncrement(min, max, maxRotations)` | Velocity-scaled per-call increment for adjusting a parameter. |
| `float mapPositionToRange(min, max, maxRotations)` | Absolute mapping of cumulative position into a range (no velocity scaling). |

### State management

| Method | Description |
|---|---|
| `void resetCumulativePosition(position = 0)` | Reset the counter and reseed the internal baseline. |

## Examples

- **BasicReading** — print raw/normalized/cumulative/speed every 50 ms.
- **VelocitySensitiveKnob** — adjust a float parameter with velocity scaling,
  plus a button to reset the cumulative position.
- **MIDI2HiResKnob** — USB MIDI knob example. Open it in the Arduino IDE to
  send a 14-bit MIDI 1.0 CC pair via MIDIUSB, or build it with PlatformIO on
  Pico 2 to send true MIDI 2.0 UMP with 32-bit CC values.

## License

MIT, see [LICENSE](LICENSE).
