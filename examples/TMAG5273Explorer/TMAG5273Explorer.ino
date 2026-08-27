/*
 * TMAG5273Explorer
 * ----------------
 * Everything a TMAG5273 knows, on one 128x64 OLED, ten screens at a time.
 *
 * The TMAG5273 is a 3D Hall-effect sensor: three magnetic axes, a die
 * temperature sensor, a CORDIC angle engine, a resultant vector magnitude, and
 * a pile of status and configuration registers. That is far too much to put on
 * one 128x64 panel and still be readable, so this sketch does not try. Instead
 * it draws ten separate views of the same live data — some numeric, most
 * graphical — with buttons for display and sensor configuration.
 *
 *    1  OVERVIEW    Angle, field strength, temperature and all three axes
 *    2  COMPASS     Circular dial, needle, turn counter and speed
 *    3  AXES        Full-width centre-zero bar meters for X, Y and Z
 *    4  VECTOR      XY-plane vectorscope with a persistence trail
 *    5  SCOPE       Three scrolling waveform lanes, one per axis
 *    6  3D          Isometric axes with the field vector drawn in space
 *    7  THERMAL     Big temperature read, thermometer and history sparkline
 *    8  RADAR       Field magnitude as a proximity radar with peak hold
 *    9  DIAG        Conversion and device status flags, IDs, addresses
 *   10  REGISTERS   Hex dump of the whole register map plus a live bit texture
 *
 * Controls
 *   GP11  cycle averaging
 *   GP12  cycle screens (long press resets statistics and history)
 *   GP13  cycle angle pairs
 *   GP14  cycle magnetic channels
 *   GP15  cycle low/high ranges
 *
 * Wiring
 *   TMAG5273 VCC -> 3.3V   (1.7-3.6V part; do not feed it 5V)
 *   TMAG5273 GND -> GND
 *   TMAG5273 SDA -> board SDA        (shared with the OLED)
 *   TMAG5273 SCL -> board SCL        (shared with the OLED)
 *   TMAG5273 TEST -> GND
 *   SH1106G OLED  -> same SDA/SCL, address 0x3C
 *   Pushbuttons    -> between GP11..GP15 and GND (the pins use INPUT_PULLUP,
 *                    so no external resistors are needed)
 *
 * The address below is 0x22, the TMAG5273B parts fitted on the Velocity
 * Encoder board and the library default. For an A, C or D part change
 * SENSOR_ADDRESS below, or run the I2CBusCheck example to find out which
 * one you have.
 */

#include <MagEncoder.h>

// The Adafruit headers are named here, not just inside AlchemyOled.h, because
// the Arduino builder decides which libraries to put on the include path by
// reading the sketch's own #include lines.
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <AlchemyOled.h>

#if !ALCHEMY_OLED_AVAILABLE
#error "This example needs the Adafruit SH110X and Adafruit GFX libraries installed."
#endif

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

static const uint8_t  BUTTON_PINS[]  = { 11, 12, 13, 14, 15 };
static const uint8_t  SENSOR_ADDRESS = TMAG5273::ADDRESS_B;  // 0x22
static const uint8_t  OLED_ADDRESS   = AlchemyOled::DEFAULT_ADDR;

// A full 128x64 frame over I2C costs a few milliseconds, so the display is
// redrawn on its own slower cadence than the sensor is sampled.
static const unsigned long SAMPLE_INTERVAL_MS = 20;
static const unsigned long DRAW_INTERVAL_MS   = 50;

static const unsigned long DEBOUNCE_MS   = 25;
static const unsigned long LONG_PRESS_MS = 700;
static const unsigned long CHANGE_DISPLAY_MS = 800;

// ---------------------------------------------------------------------------
// Objects and state
// ---------------------------------------------------------------------------

MagEncoder  encoder;
AlchemyOled oled;

// One history sample per horizontal pixel, so the scope scrolls exactly one
// column per sample with no resampling.
static const uint16_t TRACE_LEN = 128;
static uint8_t  histX[TRACE_LEN];
static uint8_t  histY[TRACE_LEN];
static uint8_t  histZ[TRACE_LEN];
static uint16_t traceHead = 0;

// Temperature history is kept in hundredths of a degree rather than pre-scaled,
// because the trace is autoscaled to whatever range it actually covers. Pinned
// to the sensor's full -40..170C span, a couple of degrees of die drift would be
// a flat line.
static const uint16_t TEMP_LEN = 126;
static int16_t  histTemp[TEMP_LEN];
static uint16_t tempHead = 0;

// Short persistence trail for the vectorscope, stored normalized to +/-127.
static const uint8_t TRAIL_LEN = 33;
static int8_t  trailX[TRAIL_LEN];
static int8_t  trailY[TRAIL_LEN];
static uint8_t trailCount = 0;
static uint8_t trailHead  = 0;

static float peakField = 0.0f;
static float minTemp   = 0.0f;
static float maxTemp   = 0.0f;

// The device answers on I2C before its first conversion has landed, so a read
// taken too early returns a temperature code of 0 — which converts to about
// -277C and would otherwise sit in the minimum forever. Nothing is folded into
// the statistics until a reading inside the sensor's specified range arrives.
static bool statsSeeded = false;

static const float TEMP_VALID_MIN = -45.0f;
static const float TEMP_VALID_MAX = 180.0f;

static uint8_t regMap[TMAG5273::REGISTER_COUNT];

static uint8_t currentScreen = 0;
static bool    sensorPresent = false;

enum ButtonAction : uint8_t
{
    CYCLE_AVERAGING,
    CYCLE_SCREEN,
    CYCLE_ANGLE_PAIR,
    CYCLE_MAG_CHANNELS,
    CYCLE_RANGE
};

struct ButtonState
{
    bool          lastRaw;
    bool          stable;
    unsigned long lastChange;
    unsigned long pressedAt;
    bool          longFired;
};

static ButtonState buttonStates[] = {
    { HIGH, HIGH, 0, 0, false },
    { HIGH, HIGH, 0, 0, false },
    { HIGH, HIGH, 0, 0, false },
    { HIGH, HIGH, 0, 0, false },
    { HIGH, HIGH, 0, 0, false }
};

static const uint8_t BUTTON_COUNT = sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]);

static const char  *changeTitle   = nullptr;
static const char  *changeValue   = nullptr;
static unsigned long changeShownAt = 0;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

static TMAG5273 &mag()
{
    return encoder.tmag();
}

/**
 * Format a float without pulling in printf's floating point support, which is
 * absent or expensive on several Arduino cores. Results rotate through a small
 * pool of buffers so a few calls can be in flight in the same expression.
 */
static const char *fmt(float value, uint8_t decimals)
{
    static const uint8_t POOL = 4;
    static char buffers[POOL][16];
    static uint8_t next = 0;

    char *buf = buffers[next];
    next = static_cast<uint8_t>((next + 1) % POOL);

    const bool negative = value < 0.0f;
    if (negative)
        value = -value;

    long scale = 1;
    for (uint8_t i = 0; i < decimals; ++i)
        scale *= 10;

    const long total = static_cast<long>(value * scale + 0.5f);
    const long whole = total / scale;
    const long frac  = total - (whole * scale);

    char *p = buf;
    if (negative)
        *p++ = '-';

    p += sprintf(p, "%ld", whole);

    if (decimals > 0)
    {
        *p++ = '.';
        for (int8_t d = static_cast<int8_t>(decimals) - 1; d >= 0; --d)
        {
            long divisor = 1;
            for (int8_t k = 0; k < d; ++k)
                divisor *= 10;
            *p++ = static_cast<char>('0' + ((frac / divisor) % 10));
        }
    }

    *p = '\0';
    return buf;
}

/** Map a signed value in +/-fullScale onto 0..255 with 128 as zero. */
static uint8_t encodeSigned(float value, float fullScale)
{
    if (fullScale <= 0.0f)
        return 128;

    float norm = value / fullScale;
    if (norm > 1.0f)
        norm = 1.0f;
    if (norm < -1.0f)
        norm = -1.0f;

    return static_cast<uint8_t>(128 + static_cast<int>(norm * 127.0f));
}

/**
 * Label for the resultant field strength. The classic GFX font draws the ASCII
 * pipe (0x7C) as a *broken* bar, which makes "|B|" look like a typo, so this
 * uses CP437 0xB3 — the box-drawing light vertical, which is solid. The pieces
 * are separate string literals because "\xB3B" would be parsed as one
 * out-of-range hex escape.
 */
static const char FIELD_LABEL[] = "\xB3" "B" "\xB3";

static const char *averagingName(TMAG5273::ConvAvg averaging)
{
    switch (averaging)
    {
        case TMAG5273::ConvAvg::X1:  return "1x";
        case TMAG5273::ConvAvg::X2:  return "2x";
        case TMAG5273::ConvAvg::X4:  return "4x";
        case TMAG5273::ConvAvg::X8:  return "8x";
        case TMAG5273::ConvAvg::X16: return "16x";
        case TMAG5273::ConvAvg::X32: return "32x";
    }
    return "?";
}

static const char *anglePairName(TMAG5273::AnglePair pair)
{
    switch (pair)
    {
        case TMAG5273::AnglePair::Off: return "OFF";
        case TMAG5273::AnglePair::XY:  return "XY";
        case TMAG5273::AnglePair::YZ:  return "YZ";
        case TMAG5273::AnglePair::XZ:  return "XZ";
    }
    return "?";
}

static const char *magChannelsName(TMAG5273::MagChannels channels)
{
    switch (channels)
    {
        case TMAG5273::MagChannels::Off: return "OFF";
        case TMAG5273::MagChannels::X:   return "X";
        case TMAG5273::MagChannels::Y:   return "Y";
        case TMAG5273::MagChannels::XY:  return "XY";
        case TMAG5273::MagChannels::Z:   return "Z";
        case TMAG5273::MagChannels::ZX:  return "ZX";
        case TMAG5273::MagChannels::YZ:  return "YZ";
        case TMAG5273::MagChannels::XYZ: return "XYZ";
        case TMAG5273::MagChannels::XYX: return "XYX";
        case TMAG5273::MagChannels::YXY: return "YXY";
        case TMAG5273::MagChannels::YZY: return "YZY";
        case TMAG5273::MagChannels::XZX: return "XZX";
    }
    return "?";
}

static void showChange(const char *title, const char *value)
{
    changeTitle   = title;
    changeValue   = value;
    changeShownAt = millis();
}

/** Label plus a checkbox, filled when the flag is set. */
static void flagRow(int x, int y, const char *label, bool on)
{
    oled.atPixel(x, y + 1, 1).print(label);

    const int boxX = x + 52;
    oled.gfx().drawRect(boxX, y, 7, 7, AlchemyOled::WHITE);
    if (on)
        oled.gfx().fillRect(boxX + 2, y + 2, 3, 3, AlchemyOled::WHITE);
}

static void resetStatistics()
{
    peakField   = 0.0f;
    statsSeeded = false;

    const uint8_t zero = encodeSigned(0.0f, 1.0f);
    for (uint16_t i = 0; i < TRACE_LEN; ++i)
    {
        histX[i] = zero;
        histY[i] = zero;
        histZ[i] = zero;
    }

    // Flat-fill the temperature history with the present reading so the trace
    // starts as a horizon rather than a step.
    const float temperature = mag().getTemperature();
    const int16_t seed = static_cast<int16_t>(
        constrain(temperature, TEMP_VALID_MIN, TEMP_VALID_MAX) * 100.0f);
    for (uint16_t i = 0; i < TEMP_LEN; ++i)
        histTemp[i] = seed;

    trailCount = 0;
    trailHead  = 0;

    encoder.resetCumulativePosition(0);
}

// ---------------------------------------------------------------------------
// Screen 1 — OVERVIEW
// Everything worth knowing in one glance: the CORDIC angle set large, the
// resultant field and temperature beside it, and the three axes underneath as
// centre-zero meters so their signs read without parsing a minus sign.
// ---------------------------------------------------------------------------
static void screenOverview()
{
    Adafruit_SH1106G &g = oled.gfx();

    // Headline: the CORDIC angle set large, with a superscript degree sign
    // placed from the measured text width so it tracks 3- and 5-digit values.
    const char *angleText = fmt(mag().getAngle(), 1);
    oled.atPixel(0, 10, 2).print(angleText);
    oled.atPixel(AlchemyOled::textWidth(angleText, 2) + 2, 10, 1)
        .print(AlchemyOled::DEGREE);

    oled.atPixel(76, 10, 1).print(FIELD_LABEL);
    oled.textRight(127, 10, fmt(mag().getFieldMagnitude(), 2));

    oled.atPixel(76, 19, 1).print("T");
    oled.textRight(127, 19, fmt(mag().getTemperature(), 1));

    const float rangeXY = mag().getRangeXY();
    const float rangeZ  = mag().getRangeZ();

    const char *labels[3] = { "X", "Y", "Z" };
    const float values[3] = { mag().getX(), mag().getY(), mag().getZ() };
    const float ranges[3] = { rangeXY, rangeXY, rangeZ };

    for (uint8_t i = 0; i < 3; ++i)
    {
        const int y = 30 + i * 11;
        oled.atPixel(0, y + 1, 1).print(labels[i]);
        oled.bipolarBar(8, y, 46, 9, values[i], ranges[i]);
        oled.textRight(127, y + 1, fmt(values[i], 2));
    }

    // Separator so the headline block and the axis block read as two groups.
    g.drawFastHLine(0, 27, 128, AlchemyOled::WHITE);
}

// ---------------------------------------------------------------------------
// Screen 2 — COMPASS
// The angle as an instrument rather than a number: a ticked dial, a needle,
// and an outer arc that sweeps with the reading. Turn count and speed sit
// beside it so multi-turn motion is visible too.
// ---------------------------------------------------------------------------
static void screenCompass()
{
    Adafruit_SH1106G &g = oled.gfx();

    const int cx = 32;
    const int cy = 36;
    const float angle = mag().getAngle();

    oled.dial(cx, cy, 23, 12, 3);
    oled.arc(cx, cy, 26, 0.0f, angle);
    oled.needle(cx, cy, 19, angle);

    // Index mark at zero so the dial has an origin.
    g.fillTriangle(cx - 3, 6, cx + 3, 6, cx, 11, AlchemyOled::WHITE);

    oled.atPixel(64, 11, 1).print("ANGLE");
    oled.atPixel(64, 19, 2).print(fmt(angle, 0));
    g.print(AlchemyOled::DEGREE);

    const float turns = static_cast<float>(encoder.getCumulativePosition()) /
                        static_cast<float>(encoder.getCountsPerRevolution());
    oled.atPixel(64, 37, 1).print("TURN ");
    g.print(fmt(turns, 2));

    const float dps = encoder.getAngularSpeed();
    oled.atPixel(64, 46, 1).print("DPS  ");
    g.print(fmt(dps, 0));

    // Direction indicator: an arrow while turning, a dot when parked.
    const int ax = 64;
    const int ay = 59;
    if (dps > 8.0f)
        g.fillTriangle(ax, ay - 4, ax, ay + 4, ax + 8, ay, AlchemyOled::WHITE);
    else if (dps < -8.0f)
        g.fillTriangle(ax + 8, ay - 4, ax + 8, ay + 4, ax, ay, AlchemyOled::WHITE);
    else
        g.fillCircle(ax + 4, ay, 2, AlchemyOled::WHITE);

    oled.atPixel(78, ay - 3, 1).print(mag().getMagnitude());
    g.print(" MAG");
}

// ---------------------------------------------------------------------------
// Screen 3 — AXES
// Three full-width centre-zero meters. The point of this screen is comparison:
// which axis dominates, and by how much, without reading any digits.
// ---------------------------------------------------------------------------
static void screenAxes()
{
    const float rangeXY = mag().getRangeXY();
    const float rangeZ  = mag().getRangeZ();

    const char *labels[3] = { "X", "Y", "Z" };
    const float values[3] = { mag().getX(), mag().getY(), mag().getZ() };
    const float ranges[3] = { rangeXY, rangeXY, rangeZ };

    for (uint8_t i = 0; i < 3; ++i)
    {
        const int y = 10 + i * 16;
        oled.atPixel(0, y + 2, 1).print(labels[i]);
        oled.bipolarBar(10, y, 74, 11, values[i], ranges[i]);
        oled.textRight(127, y + 2, fmt(values[i], 2));
    }

    // 0xF1 is the plus-minus sign in CP437, which begin() enables.
    char footer[24];
    snprintf(footer, sizeof(footer), "%c%smT   AVG %s", 0xF1,
             fmt(rangeXY, 0), averagingName(mag().config().averaging));
    oled.atPixel(0, 56, 1).print(footer);
}

// ---------------------------------------------------------------------------
// Screen 4 — VECTOR
// The XY plane seen from above. The dot is where the field is pointing right
// now; the fading trail behind it is where it has just been, which turns a
// rotating magnet into a visible circle and a wobbling one into a scribble.
// ---------------------------------------------------------------------------
static void screenVector()
{
    Adafruit_SH1106G &g = oled.gfx();

    const int cx = 32;
    const int cy = 37;
    const int r  = 24;

    g.drawCircle(cx, cy, r, AlchemyOled::WHITE);
    g.drawCircle(cx, cy, r / 2, AlchemyOled::WHITE);
    oled.dottedHLine(cx - r, cy, 2 * r + 1, 3);
    oled.dottedVLine(cx, cy - r, 2 * r + 1, 3);

    // Trail, oldest first so the newest point is drawn on top.
    for (uint8_t i = 0; i < trailCount; ++i)
    {
        const uint8_t index =
            static_cast<uint8_t>((trailHead + TRAIL_LEN - trailCount + i) % TRAIL_LEN);
        const int px = cx + (trailX[index] * r) / 127;
        const int py = cy - (trailY[index] * r) / 127;
        g.drawPixel(px, py, AlchemyOled::WHITE);
    }

    const float rangeXY = mag().getRangeXY();
    const float bx = mag().getX();
    const float by = mag().getY();

    int px = cx + static_cast<int>((bx / rangeXY) * r);
    int py = cy - static_cast<int>((by / rangeXY) * r);
    px = constrain(px, cx - r, cx + r);
    py = constrain(py, cy - r, cy + r);

    g.drawLine(cx, cy, px, py, AlchemyOled::WHITE);
    g.fillCircle(px, py, 2, AlchemyOled::WHITE);

    oled.atPixel(64, 11, 1).print("X ");
    g.print(fmt(bx, 2));
    oled.atPixel(64, 20, 1).print("Y ");
    g.print(fmt(by, 2));

    oled.atPixel(64, 32, 1).print("AZ ");
    g.print(fmt(mag().getAzimuth(), 0));
    g.print(AlchemyOled::DEGREE);

    const float planar = sqrtf(bx * bx + by * by);
    oled.atPixel(64, 41, 1).print("R  ");
    g.print(fmt(planar, 2));

    oled.bar(64, 52, 60, 8, planar / rangeXY);
}

// ---------------------------------------------------------------------------
// Screen 5 — SCOPE
// Three scrolling lanes, one axis each, newest sample at the right edge. Phase
// relationships between the axes are obvious here and nowhere else.
// ---------------------------------------------------------------------------
static void screenScope()
{
    Adafruit_SH1106G &g = oled.gfx();

    const char    *labels[3] = { "X", "Y", "Z" };
    const uint8_t *traces[3] = { histX, histY, histZ };

    for (uint8_t lane = 0; lane < 3; ++lane)
    {
        const int y = 11 + lane * 17;

        oled.dottedHLine(0, y + 8, 128, 4);
        oled.sparkline(0, y, 128, 17, traces[lane], TRACE_LEN, traceHead);

        // Lane label last, knocked out of a filled block so it survives being
        // drawn over by a trace that happens to sit at the left edge.
        g.fillRect(0, y, 7, 9, AlchemyOled::WHITE);
        g.setTextColor(AlchemyOled::BLACK);
        oled.atPixel(1, y + 1, 1).print(labels[lane]);
        g.setTextColor(AlchemyOled::WHITE);
    }
}

// ---------------------------------------------------------------------------
// Screen 6 — 3D
// Isometric projection of the three axes with the field drawn as a vector in
// that space, plus its shadow on the XY plane. This is the screen that makes
// "3D Hall sensor" mean something.
// ---------------------------------------------------------------------------
static void screen3D()
{
    Adafruit_SH1106G &g = oled.gfx();

    const int   ox  = 38; // projection origin
    const int   oy  = 40;
    const float len = 22.0f;

    // Isometric basis: X goes right and down, Y left and down, Z straight up.
    const float ux =  0.866f, uy =  0.5f;
    const float vx = -0.866f, vy =  0.5f;
    const float wy = -1.0f;

    // Axis lines. The positive half of each axis is solid and the negative half
    // dotted, so the octant the field vector is sitting in stays readable
    // instead of dissolving into a six-pointed asterisk.
    const float axes[3][2] = { { ux, uy }, { vx, vy }, { 0.0f, wy } };
    for (uint8_t a = 0; a < 3; ++a)
    {
        const int px = ox + static_cast<int>(axes[a][0] * len);
        const int py = oy + static_cast<int>(axes[a][1] * len);
        g.drawLine(ox, oy, px, py, AlchemyOled::WHITE);

        for (int t = 2; t <= static_cast<int>(len); t += 3)
            g.drawPixel(ox - static_cast<int>(axes[a][0] * t),
                        oy - static_cast<int>(axes[a][1] * t), AlchemyOled::WHITE);
    }

    oled.atPixel(ox + static_cast<int>(ux * len) + 1,
                 oy + static_cast<int>(uy * len) - 3, 1).print("X");
    oled.atPixel(ox + static_cast<int>(vx * len) - 7,
                 oy + static_cast<int>(vy * len) - 3, 1).print("Y");
    oled.atPixel(ox + 2, oy - static_cast<int>(len) - 1, 1).print("Z");

    const float rangeXY = mag().getRangeXY();
    const float rangeZ  = mag().getRangeZ();

    float nx = mag().getX() / rangeXY;
    float ny = mag().getY() / rangeXY;
    float nz = mag().getZ() / rangeZ;
    nx = constrain(nx, -1.0f, 1.0f);
    ny = constrain(ny, -1.0f, 1.0f);
    nz = constrain(nz, -1.0f, 1.0f);

    // Project the field vector, and separately its XY-plane shadow.
    const int shadowX = ox + static_cast<int>((nx * ux + ny * vx) * len);
    const int shadowY = oy + static_cast<int>((nx * uy + ny * vy) * len);
    const int tipX    = shadowX;
    const int tipY    = shadowY + static_cast<int>(nz * wy * len);

    const int drop = abs(tipY - shadowY);
    for (int i = 0; i <= drop; i += 3)
        g.drawPixel(shadowX, shadowY + ((tipY > shadowY) ? i : -i), AlchemyOled::WHITE);
    g.drawCircle(shadowX, shadowY, 2, AlchemyOled::WHITE);

    // Drawn twice, one pixel apart, so the field vector reads as bolder than
    // the axes it crosses.
    oled.vector(ox, oy, tipX, tipY);
    oled.vector(ox + 1, oy, tipX + 1, tipY);

    oled.atPixel(76, 11, 1).print("AZ");
    oled.textRight(127, 11, fmt(mag().getAzimuth(), 0));
    oled.atPixel(76, 20, 1).print("EL");
    oled.textRight(127, 20, fmt(mag().getElevation(), 0));
    oled.atPixel(76, 32, 1).print(FIELD_LABEL);
    oled.textRight(127, 32, fmt(mag().getFieldMagnitude(), 2));
    oled.atPixel(76, 41, 1).print("Z");
    oled.textRight(127, 41, fmt(mag().getZ(), 2));
}

// ---------------------------------------------------------------------------
// Screen 7 — THERMAL
// The die temperature, which is both a diagnostic and a real measurement: it
// is what the magnet's temperature compensation is computed from.
// ---------------------------------------------------------------------------
static void screenThermal()
{
    Adafruit_SH1106G &g = oled.gfx();

    const float temperature = mag().getTemperature();

    // Position the degree ring and the unit from the measured text width so a
    // sub-zero reading pushes them right instead of drawing underneath them.
    const char *tempText = fmt(temperature, 1);
    oled.atPixel(0, 12, 3).print(tempText);

    const int unitX = AlchemyOled::textWidth(tempText, 3) + 4;
    g.drawCircle(unitX + 2, 15, 2, AlchemyOled::WHITE);
    oled.atPixel(unitX + 7, 12, 2).print("C");

    oled.atPixel(74, 33, 1).print("H ");
    g.print(statsSeeded ? fmt(maxTemp, 1) : "--");
    oled.atPixel(74, 42, 1).print("L ");
    g.print(statsSeeded ? fmt(minTemp, 1) : "--");

    // Thermometer: stem plus bulb, filled over the sensor's -40..170C span.
    const int stemX = 114;
    const int stemY = 10;
    const int stemH = 28;
    g.drawRect(stemX, stemY, 9, stemH, AlchemyOled::WHITE);
    g.fillCircle(stemX + 4, stemY + stemH + 6, 6, AlchemyOled::WHITE);

    float level = (temperature + 40.0f) / 210.0f;
    level = constrain(level, 0.0f, 1.0f);
    const int fill = static_cast<int>(level * (stemH - 2));
    if (fill > 0)
        g.fillRect(stemX + 2, stemY + stemH - 1 - fill, 5, fill, AlchemyOled::WHITE);

    // History across the full width, framed so the trace has a horizon to be
    // read against. The window is autoscaled to whatever the trace actually
    // covers, widened to a minimum of 0.5C so a perfectly steady die does not
    // turn sensor noise into a mountain range.
    int16_t lo = histTemp[0];
    int16_t hi = histTemp[0];
    for (uint16_t i = 1; i < TEMP_LEN; ++i)
    {
        if (histTemp[i] < lo) lo = histTemp[i];
        if (histTemp[i] > hi) hi = histTemp[i];
    }
    if (hi - lo < 50)
    {
        const int16_t mid = static_cast<int16_t>((hi + lo) / 2);
        lo = static_cast<int16_t>(mid - 25);
        hi = static_cast<int16_t>(mid + 25);
    }

    uint8_t scaled[TEMP_LEN];
    for (uint16_t i = 0; i < TEMP_LEN; ++i)
        scaled[i] = static_cast<uint8_t>(
            (static_cast<long>(histTemp[i] - lo) * 255L) / (hi - lo));

    g.drawRect(0, 52, 128, 12, AlchemyOled::WHITE);
    oled.sparkline(1, 53, TEMP_LEN, 10, scaled, TEMP_LEN, tempHead);
}

// ---------------------------------------------------------------------------
// Screen 8 — RADAR
// Field magnitude as distance: the blob grows as a magnet approaches, and the
// hollow ring remembers the closest it ever got. Useful for setting up magnet
// spacing without watching numbers.
// ---------------------------------------------------------------------------
static void screenRadar()
{
    Adafruit_SH1106G &g = oled.gfx();

    const int cx = 32;
    const int cy = 37;
    const int rMax = 24;

    // Range rings and spokes.
    for (int r = 8; r <= rMax; r += 8)
        g.drawCircle(cx, cy, r, AlchemyOled::WHITE);
    for (uint8_t i = 0; i < 8; ++i)
    {
        int ex, ey;
        AlchemyOled::polar(cx, cy, static_cast<float>(rMax), i * 45.0f, ex, ey);
        g.drawLine(cx, cy, ex, ey, AlchemyOled::WHITE);
    }

    const float field    = mag().getFieldMagnitude();
    const float fullScale = mag().getRangeXY();

    float level = field / fullScale;
    level = constrain(level, 0.0f, 1.0f);
    const int blob = static_cast<int>(level * rMax);
    if (blob > 0)
        g.fillCircle(cx, cy, blob, AlchemyOled::WHITE);

    float peakLevel = peakField / fullScale;
    peakLevel = constrain(peakLevel, 0.0f, 1.0f);
    const int peakRadius = static_cast<int>(peakLevel * rMax);
    if (peakRadius > 0)
    {
        g.drawCircle(cx, cy, peakRadius, AlchemyOled::BLACK);
        g.drawCircle(cx, cy, peakRadius + 1, AlchemyOled::WHITE);
    }

    oled.atPixel(64, 11, 1).print(FIELD_LABEL);
    oled.gfx().print(" mT");
    oled.atPixel(64, 20, 2).print(fmt(field, 1));

    oled.atPixel(64, 38, 1).print("PEAK ");
    g.print(fmt(peakField, 1));

    oled.atPixel(64, 47, 1).print("MAG  ");
    g.print(mag().getMagnitude());

    oled.segmentBar(64, 56, 60, 6, mag().getMagnitude() / 255.0f, 10);
}

// ---------------------------------------------------------------------------
// Screen 9 — DIAG
// The status registers, decoded. Every latched error bit the device can raise
// is here; a long press clears them along with the rest of the statistics.
// ---------------------------------------------------------------------------
static void screenDiagnostics()
{
    Adafruit_SH1106G &g = oled.gfx();

    const TMAG5273::ConversionStatus &conv   = mag().getConversionStatus();
    const TMAG5273::DeviceStatus     &device = mag().getDeviceStatus();

    flagRow(0, 11, "RESULT",  conv.dataReady);
    flagRow(0, 20, "DIAGFAIL", conv.diagFail);
    flagRow(0, 29, "POR",     conv.powerOnReset);

    oled.atPixel(0, 39, 1).print("SET  ");
    g.print(conv.setCount);
    oled.atPixel(0, 48, 1).print("VER  ");
    g.print(mag().getVersionName());
    oled.atPixel(0, 57, 1).print("ADR  0x");
    g.print(mag().getI2CAddress(), HEX);

    oled.dottedVLine(62, 10, 52, 2);

    flagRow(66, 11, "INT PIN", device.intPinHigh);
    flagRow(66, 20, "OSC ER",  device.oscError);
    flagRow(66, 29, "INT ER",  device.intError);
    flagRow(66, 38, "OTP CRC", device.otpCrcError);
    flagRow(66, 47, "VCC UV",  device.vccUnderVolt);

    oled.atPixel(66, 57, 1).print("MFR ");
    g.print(mag().getManufacturerId(), HEX);
}

// ---------------------------------------------------------------------------
// Screen 10 — REGISTERS
// The whole register map as hex, and underneath it the same 29 bytes drawn as
// a bit texture: one column per register, MSB at the top. Config bits hold
// still; result bits shimmer. It is the fastest way to see the device working.
// ---------------------------------------------------------------------------
static void screenRegisters()
{
    Adafruit_SH1106G &g = oled.gfx();

    for (uint8_t row = 0; row < 4; ++row)
    {
        const uint8_t base = static_cast<uint8_t>(row * 8);

        char line[24];
        char *p = line;
        p += sprintf(p, "%02X:", base);

        for (uint8_t i = 0; i < 8; ++i)
        {
            const uint8_t offset = static_cast<uint8_t>(base + i);
            if (offset >= TMAG5273::REGISTER_COUNT)
                break;
            p += sprintf(p, "%02X", regMap[offset]);
        }
        *p = '\0';

        oled.atPixel(0, 11 + row * 8, 1).print(line);
    }

    g.drawFastHLine(0, 44, 128, AlchemyOled::WHITE);

    // Bit texture: one column per register, MSB at the top, a 3x2 block per set
    // bit. Configuration bits hold still while result bits shimmer, which makes
    // a working device obvious at a glance.
    for (uint8_t reg = 0; reg < TMAG5273::REGISTER_COUNT; ++reg)
    {
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            if (regMap[reg] & (0x80 >> bit))
                g.fillRect(2 + reg * 4, 47 + bit * 2, 3, 2, AlchemyOled::WHITE);
        }
    }
}

// ---------------------------------------------------------------------------
// Screen table
// ---------------------------------------------------------------------------

struct Screen
{
    const char *name;
    void (*draw)();
};

static const Screen SCREENS[] = {
    { "OVERVIEW",  screenOverview    },
    { "COMPASS",   screenCompass     },
    { "AXES",      screenAxes        },
    { "VECTOR",    screenVector      },
    { "SCOPE",     screenScope       },
    { "3D FIELD",  screen3D          },
    { "THERMAL",   screenThermal     },
    { "RADAR",     screenRadar       },
    { "DIAG",      screenDiagnostics },
    { "REGISTERS", screenRegisters   }
};

static const uint8_t SCREEN_COUNT = sizeof(SCREENS) / sizeof(SCREENS[0]);

// ---------------------------------------------------------------------------
// Sampling, input and drawing
// ---------------------------------------------------------------------------

static void sampleSensor()
{
    // encoder.update() reads the TMAG5273 for us and keeps the cumulative
    // position and angular speed current, so there is no second I2C read here.
    encoder.update();

    const float rangeXY = mag().getRangeXY();
    const float rangeZ  = mag().getRangeZ();
    const float bx = mag().getX();
    const float by = mag().getY();

    histX[traceHead] = encodeSigned(bx, rangeXY);
    histY[traceHead] = encodeSigned(by, rangeXY);
    histZ[traceHead] = encodeSigned(mag().getZ(), rangeZ);
    traceHead = static_cast<uint16_t>((traceHead + 1) % TRACE_LEN);

    const float temperature = mag().getTemperature();
    const bool  temperatureValid =
        (temperature > TEMP_VALID_MIN) && (temperature < TEMP_VALID_MAX);

    if (temperatureValid)
    {
        histTemp[tempHead] = static_cast<int16_t>(temperature * 100.0f);
        tempHead = static_cast<uint16_t>((tempHead + 1) % TEMP_LEN);

        if (!statsSeeded)
        {
            minTemp     = temperature;
            maxTemp     = temperature;
            statsSeeded = true;
        }
        else if (temperature < minTemp)
        {
            minTemp = temperature;
        }
        else if (temperature > maxTemp)
        {
            maxTemp = temperature;
        }
    }

    const float field = mag().getFieldMagnitude();
    if (field > peakField)
        peakField = field;

    trailX[trailHead] = static_cast<int8_t>(constrain(bx / rangeXY, -1.0f, 1.0f) * 127.0f);
    trailY[trailHead] = static_cast<int8_t>(constrain(by / rangeXY, -1.0f, 1.0f) * 127.0f);
    trailHead = static_cast<uint8_t>((trailHead + 1) % TRAIL_LEN);
    if (trailCount < TRAIL_LEN)
        ++trailCount;

    // The register screen is the only one that needs the full map, and it is a
    // 29-byte read, so only pay for it when that screen is up.
    if (currentScreen == SCREEN_COUNT - 1)
        mag().readRegisterMap(regMap);
}

static void handleButtonPress(uint8_t action)
{
    if (action == CYCLE_AVERAGING)
    {
        const uint8_t value = static_cast<uint8_t>(mag().config().averaging);
        const TMAG5273::ConvAvg next =
            static_cast<TMAG5273::ConvAvg>((value + 1) % 6);
        mag().setAveraging(next);
        showChange("AVERAGING", averagingName(next));
    }
    else if (action == CYCLE_SCREEN)
    {
        changeTitle = nullptr;
        currentScreen = static_cast<uint8_t>((currentScreen + 1) % SCREEN_COUNT);

        if (currentScreen == SCREEN_COUNT - 1)
            mag().readRegisterMap(regMap);
    }
    else if (action == CYCLE_ANGLE_PAIR)
    {
        const uint8_t value = static_cast<uint8_t>(mag().config().anglePair);
        const TMAG5273::AnglePair next =
            static_cast<TMAG5273::AnglePair>((value + 1) % 4);
        mag().setAnglePair(next);
        showChange("ANGLE PAIR", anglePairName(next));
    }
    else if (action == CYCLE_MAG_CHANNELS)
    {
        const uint8_t value = static_cast<uint8_t>(mag().config().channels);
        const TMAG5273::MagChannels next =
            static_cast<TMAG5273::MagChannels>((value + 1) % 12);
        mag().setMagChannels(next);
        showChange("MAG CHANNELS", magChannelsName(next));
    }
    else
    {
        const TMAG5273::Range next = mag().config().rangeXY == TMAG5273::Range::Low
                                         ? TMAG5273::Range::High
                                         : TMAG5273::Range::Low;
        mag().setRanges(next, next);
        showChange("RANGE", next == TMAG5273::Range::Low ? "LOW" : "HIGH");
    }
}

static void pollButtons()
{
    const unsigned long now = millis();

    for (uint8_t i = 0; i < BUTTON_COUNT; ++i)
    {
        ButtonState &button = buttonStates[i];
        const bool raw = digitalRead(BUTTON_PINS[i]);

        if (raw != button.lastRaw)
        {
            button.lastRaw    = raw;
            button.lastChange = now;
        }

        if (now - button.lastChange < DEBOUNCE_MS)
            continue;

        if (raw == button.stable)
        {
            if (i == CYCLE_SCREEN && button.stable == LOW &&
                !button.longFired && (now - button.pressedAt) >= LONG_PRESS_MS)
            {
                button.longFired = true;
                resetStatistics();
            }
            continue;
        }

        button.stable = raw;

        if (button.stable == LOW)
        {
            button.pressedAt = now;
            button.longFired = false;
        }
        else if (!button.longFired)
        {
            handleButtonPress(i);
        }
    }
}

static void drawFrame()
{
    if (changeTitle != nullptr && millis() - changeShownAt < CHANGE_DISPLAY_MS)
    {
        oled.clear();
        oled.title(changeTitle);
        oled.textCentered(27, changeValue, 3);
        oled.show();
        return;
    }

    changeTitle = nullptr;

    char counter[8];
    snprintf(counter, sizeof(counter), "%u/%u",
             static_cast<unsigned>(currentScreen + 1),
             static_cast<unsigned>(SCREEN_COUNT));

    oled.clear();
    oled.title(SCREENS[currentScreen].name, counter);
    SCREENS[currentScreen].draw();
    oled.show();
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

/**
 * Explain a failed begin() instead of just reporting it.
 *
 * TMAG5273::begin() gives up at one of three places, and they need very
 * different fixes: the address does not acknowledge at all, the address
 * acknowledges but the register read fails, or the read works and the
 * manufacturer ID is wrong. This walks the four addresses the family ships
 * with and reports what each one does, which also catches the common case of
 * holding a B, C or D part while the sketch asks for an A.
 */
static void reportSensorFailure()
{
    static const uint8_t candidates[] = {
        TMAG5273::ADDRESS_A, TMAG5273::ADDRESS_B,
        TMAG5273::ADDRESS_C, TMAG5273::ADDRESS_D
    };

    Serial.println();
    Serial.println("Probing the TMAG5273 address family:");

    for (uint8_t i = 0; i < sizeof(candidates); ++i)
    {
        const uint8_t address = candidates[i];

        Serial.print("  0x");
        Serial.print(address, HEX);
        Serial.print("  ");

        Wire.beginTransmission(address);
        if (Wire.endTransmission() != 0)
        {
            Serial.println("no ACK");
            continue;
        }

        Serial.print("ACK, ");

        // DEVICE_ID (0x0D), then MANUFACTURER_ID LSB (0x0E) and MSB (0x0F).
        Wire.beginTransmission(address);
        Wire.write(static_cast<uint8_t>(TMAG5273::REG_DEVICE_ID));
        if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)address, 3) != 3)
        {
            Serial.println("but the register read failed (repeated START refused?)");
            continue;
        }

        const uint8_t deviceId = static_cast<uint8_t>(Wire.read());
        const uint8_t mfgLsb   = static_cast<uint8_t>(Wire.read());
        const uint8_t mfgMsb   = static_cast<uint8_t>(Wire.read());
        const uint16_t mfgId   = static_cast<uint16_t>(mfgMsb) << 8 | mfgLsb;

        Serial.print("DEVICE_ID 0x");
        Serial.print(deviceId, HEX);
        Serial.print(", MFG_ID 0x");
        Serial.print(mfgId, HEX);

        if (mfgId == TMAG5273::MANUFACTURER_ID)
        {
            Serial.println("  <-- a real TMAG5273 lives here");
            Serial.print("      Set SENSOR_ADDRESS to 0x");
            Serial.print(address, HEX);
            Serial.println(" and rebuild.");
        }
        else
        {
            Serial.println("  (expected 0x5449, so this is some other chip)");
        }
    }

    Serial.println();
}

void setup()
{
    Serial.begin(115200);

    // Boards with native USB re-enumerate after an upload, and the serial
    // monitor takes a moment to reattach. Without this wait every diagnostic
    // printed during setup() is sent into a void and the port looks dead.
    // The timeout keeps the sketch usable when nothing is listening at all.
    while (!Serial && millis() < 3000)
    {
    }

    for (uint8_t i = 0; i < BUTTON_COUNT; ++i)
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);

    const bool oledReady = oled.begin(OLED_ADDRESS);
    if (!oledReady)
        Serial.println("OLED not found at 0x3C. Check wiring.");

    MagEncoder::Config cfg;
    cfg.sensor     = MagEncoder::Sensor::TMAG5273;
    cfg.i2cAddress = SENSOR_ADDRESS;

    // Everything on: three axes, temperature, and the XY angle engine. This is
    // the whole point of the sketch, so nothing gets switched off to save
    // conversion time.
    cfg.tmag.channels      = TMAG5273::MagChannels::XYZ;
    cfg.tmag.enableTemp    = true;
    cfg.tmag.anglePair     = TMAG5273::AnglePair::XY;
    cfg.tmag.averaging     = TMAG5273::ConvAvg::X32;
    cfg.tmag.operatingMode = TMAG5273::OperatingMode::Continuous;
    cfg.tmag.lowNoiseMode  = true;
    cfg.tmag.rangeZ  = TMAG5273::Range::High;

    encoder = MagEncoder(cfg);
    sensorPresent = encoder.begin();

    if (!sensorPresent)
    {
        Serial.print("TMAG5273 not found at 0x");
        Serial.println(SENSOR_ADDRESS, HEX);
        reportSensorFailure();

        if (oledReady)
        {
            oled.clear();
            oled.title("SENSOR ERROR");
            oled.atPixel(0, 16, 1).print("TMAG5273 not found");
            oled.atPixel(0, 28, 1).print("addr 0x");
            oled.gfx().print(SENSOR_ADDRESS, HEX);
            oled.atPixel(0, 40, 1).print("check SDA/SCL, 3V3");
            oled.atPixel(0, 50, 1).print("and TEST pin to GND");
            oled.show();
        }

        // Repeat the probe rather than halting silently. A monitor opened late,
        // or reopened after a reset, still gets the full diagnostic, and
        // re-seating the sensor's wiring shows up on the next pass.
        while (true)
        {
            delay(5000);
            reportSensorFailure();
        }
    }

    Serial.print("TMAG5273");
    Serial.print(mag().getVersionName());
    Serial.print(" at 0x");
    Serial.print(mag().getI2CAddress(), HEX);
    Serial.print(", range +/-");
    Serial.print(mag().getRangeXY(), 0);
    Serial.println(" mT");
    Serial.println("GP11 averaging, GP12 screen, GP13 angle, GP14 channels, GP15 range.");

    mag().readRegisterMap(regMap);
    resetStatistics();

    if (oledReady)
        drawFrame();
}

void loop()
{
    static unsigned long lastSample = 0;
    static unsigned long lastDraw   = 0;

    pollButtons();

    const unsigned long now = millis();

    if (now - lastSample >= SAMPLE_INTERVAL_MS)
    {
        lastSample = now;
        sampleSensor();
    }

    if (now - lastDraw >= DRAW_INTERVAL_MS)
    {
        lastDraw = now;
        if (oled.isReady())
            drawFrame();
    }
}
