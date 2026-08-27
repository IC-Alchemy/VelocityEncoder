/*
 * VelocitySensitiveKnob
 * ---------------------
 * The velocity-sensitive parameter control: a single float parameter is
 * adjusted by the encoder. Slow turns nudge it by tiny amounts; fast turns
 * sweep across a wider range. This is the reason the driver exists.
 *
 * The parameter is constrained to [0.0, 1.0] and four full revolutions of the
 * magnet span the entire range. A pushbutton on pin 7 resets the cumulative
 * position counter, which is useful when a knob gets lifted and replaced.
 *
 * Two things are selectable at the top of this sketch:
 *
 *   SENSOR_CHOICE — which magnetic sensor is fitted, an AMS AS5600 or a TI
 *                   TMAG5273. The knob feel is identical either way; only the
 *                   part number and I2C address change.
 *
 *   ENABLE_OLED   — set to 1 to mirror the parameter onto a 128x64 SH1106G
 *                   OLED (ring gauge, bar, velocity zone and live speed).
 *                   Set to 0 and the sketch is Serial-only with no display
 *                   libraries required.
 *
 * Wiring: connect the sensor's SDA/SCL to your board's I2C pins and power it
 * from 3.3V (the TMAG5273 is a 1.7-3.6V part; do not feed it 5V). Wire a
 * pushbutton between pin 7 and ground to act as the "reset position" key
 * (optional). The OLED shares the same I2C bus.
 */

#include <MagEncoder.h>

// --- Pick your sensor ------------------------------------------------------
#define SENSOR_AS5600   0
#define SENSOR_TMAG5273 1

#define SENSOR_CHOICE SENSOR_TMAG5273

// --- Optional OLED readout -------------------------------------------------
#define ENABLE_OLED 1

#if ENABLE_OLED
// The Adafruit headers are named here, not just inside AlchemyOled.h, because
// the Arduino builder decides which libraries to put on the include path by
// reading the sketch's own #include lines.
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <AlchemyOled.h>
#if !ALCHEMY_OLED_AVAILABLE
#error "ENABLE_OLED needs the Adafruit SH110X and Adafruit GFX libraries installed."
#endif
AlchemyOled oled;
bool oledPresent = false;
#endif

static const int   RESET_BUTTON_PIN = 7;
static const float PARAM_MIN        = 0.0f;
static const float PARAM_MAX        = 1.0f;
static const uint8_t MAX_ROTATIONS  = 4;

MagEncoder encoder;
float      parameter = 0.0f;

static const char *velocityZoneName(MagEncoder::VelocityZone zone)
{
    switch (zone)
    {
        case MagEncoder::VelocityZone::Idle: return "idle";
        case MagEncoder::VelocityZone::Low:  return "low";
        case MagEncoder::VelocityZone::Mid:  return "mid";
        case MagEncoder::VelocityZone::High: return "high";
    }
    return "?";
}

#if ENABLE_OLED
void drawScreen()
{
    if (!oledPresent)
        return;

    oled.clear();
    oled.title(encoder.getSensorName(), velocityZoneName(encoder.getVelocityZone()));

    // Ring gauge on the left: the parameter as a fraction of a full turn.
    const int cx = 27;
    const int cy = 38;
    oled.ringGauge(cx, cy, 20, 4, parameter);

    // The value itself, centred in the ring.
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", static_cast<int>(parameter * 100.0f + 0.5f));
    const int textX = cx - (AlchemyOled::textWidth(buf, 2) / 2);
    oled.atPixel(textX, cy - 7, 2).print(buf);

    // Numeric readout and a linear bar on the right.
    oled.at(9, 2).print("param");
    oled.atPixel(54, 26, 1).print(parameter, 3);
    oled.bar(54, 36, 70, 8, parameter);

    // Live angular speed, so the velocity curve is visible while turning.
    oled.at(9, 6).print("dps ");
    oled.gfx().print(encoder.getAngularSpeed(), 0);

    oled.show();
}
#endif

void setup()
{
    Serial.begin(115200);

    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

    // Custom tuning: a wider dynamic range than the defaults.
    MagEncoder::Config cfg;

#if SENSOR_CHOICE == SENSOR_TMAG5273
    cfg.sensor = MagEncoder::Sensor::TMAG5273;
    // Leaving i2cAddress at 0 picks the sensor's own default (0x22, the
    // TMAG5273B parts fitted on the Velocity Encoder board). Spelled out
    // here so the address is visible; set ADDRESS_A / _C / _D for a
    // different part.
    cfg.i2cAddress = TMAG5273::ADDRESS_B;

    // The knob only needs the two axes the CORDIC angle engine uses, so turn
    // the third one off and spend the saved conversion time on averaging.
    cfg.tmag.channels  = TMAG5273::MagChannels::XY;
    cfg.tmag.anglePair = TMAG5273::AnglePair::XY;
    cfg.tmag.averaging = TMAG5273::ConvAvg::X4;
#else
    cfg.sensor = MagEncoder::Sensor::AS5600;
#endif

    cfg.minScale   = 0.005f;
    cfg.maxScale   = 4.0f;
    encoder = MagEncoder(cfg);

#if ENABLE_OLED
    oledPresent = oled.begin();
    if (!oledPresent)
        Serial.println("OLED not found; continuing without a display.");
#endif

    if (!encoder.begin())
    {
        Serial.print(encoder.getSensorName());
        Serial.println(" not found. Check wiring and I2C address.");

#if ENABLE_OLED
        if (oledPresent)
        {
            oled.clear();
            oled.title("SENSOR ERROR");
            oled.text(0, 2, encoder.getSensorName());
            oled.text(0, 3, "not responding");
            oled.at(0, 5).print("addr 0x");
            oled.gfx().print(encoder.getI2CAddress(), HEX);
            oled.show();
        }
#endif

        while (true)
        {
            delay(1000);
        }
    }

    Serial.print(encoder.getSensorName());
    Serial.print(" ready at 0x");
    Serial.print(encoder.getI2CAddress(), HEX);
    Serial.print(", ");
    Serial.print(encoder.getCountsPerRevolution());
    Serial.println(" counts/rev.");
    Serial.println("Turn the encoder to adjust the parameter.");
    Serial.println("Ground pin 7 to reset the cumulative position.");
}

void loop()
{
    encoder.update();

    // Apply a velocity-scaled increment to the parameter.
    const float increment =
        encoder.getParameterIncrement(PARAM_MIN, PARAM_MAX, MAX_ROTATIONS);
    parameter += increment;
    parameter = constrain(parameter, PARAM_MIN, PARAM_MAX);

    // Optional: reset cumulative position on button press.
    if (digitalRead(RESET_BUTTON_PIN) == LOW)
    {
        encoder.resetCumulativePosition(0);
        parameter = 0.0f;
        delay(250); // crude debounce
    }

    // Print only when the value actually changes, to avoid flooding.
    static float lastPrinted = -1.0f;
    if (fabsf(parameter - lastPrinted) > 0.0005f)
    {
        lastPrinted = parameter;
        Serial.print("parameter=");
        Serial.print(parameter, 3);
        Serial.print("  (");
        Serial.print(velocityZoneName(encoder.getVelocityZone()));
        Serial.println(")");
    }

#if ENABLE_OLED
    // Redraw on a fixed cadence rather than every loop: a full 128x64 frame
    // over I2C costs a few milliseconds and would otherwise starve the encoder.
    static unsigned long lastDraw = 0;
    const unsigned long now = millis();
    if (now - lastDraw >= 40)
    {
        lastDraw = now;
        drawScreen();
    }
#endif

    delay(5);
}
