/*
 * VelocitySensitiveKnob
 * ---------------------
 * The velocity-sensitive parameter control: a single float parameter is
 * adjusted by the encoder. Slow turns nudge it by tiny amounts; fast turns
 * sweep across a wider range. This is the reason the driver exists.
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
 * from 3.3V (the TMAG5273 is a 1.7-3.6V part; do not feed it 5V). 
 * (optional). The OLED shares the same I2C bus.
 */

#include <MagEncoder.h>

// --- Pick your sensor ------------------------------------------------------
#define SENSOR_AS5600   0
#define SENSOR_TMAG5273 1

#define SENSOR_CHOICE SENSOR_TMAG5273
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

static const float PARAM_MIN        = 0.0f;
static const float PARAM_MAX        = 2999.0f;
static const uint8_t MAX_ROTATIONS  = 4;

MagEncoder encoder;
float      parameter = 0.0f;
int32_t    lastAppliedPosition = 0;

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

    const int cx = 27;
    const int cy = 38;

    // The value itself, centred in the ring.
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", static_cast<int>(parameter * 100.0f + 0.5f));
    const int textX = cx - (AlchemyOled::textWidth(buf, 2) / 2);

    oled.atPixel(2, 20, 2 ).print(parameter, 3);



    oled.show();
}
#endif

void setup()
{
    Serial.begin(115200);


    // Custom tuning: a wider dynamic range than the defaults.
    MagEncoder::Config cfg;

    cfg.sensor = MagEncoder::Sensor::TMAG5273;
    // Leaving i2cAddress at 0 picks the sensor's own default (0x35 for the
    // TMAG5273A parts). Set it explicitly for a B/C/D part:
  cfg.i2cAddress = TMAG5273::ADDRESS_A;
cfg.minVelDps = 90;
cfg.maxVelDps =1500;
    // The knob only needs the two axes the CORDIC angle engine uses, so turn
    // the third one off and spend the saved conversion time on averaging.
    cfg.tmag.channels  = TMAG5273::MagChannels::XY;
    cfg.tmag.anglePair = TMAG5273::AnglePair::XY;
    cfg.tmag.averaging = TMAG5273::ConvAvg::X32;
cfg.minScale   = 0.0001f;
    cfg.maxScale   = 3.f;
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

        // Leave the error on screen without trapping the runtime in setup().
        return;
    }

    lastAppliedPosition = encoder.getCumulativePosition();

    Serial.print(encoder.getSensorName());
    Serial.print(" ready at 0x");
    Serial.print(encoder.getI2CAddress(), HEX);
    Serial.print(", ");
    Serial.print(encoder.getCountsPerRevolution());
    Serial.println(" counts/rev.");
    Serial.println("Turn the encoder to adjust the parameter.");
}

void loop()
{
    if (!encoder.isConnected())
        return;

    encoder.update();
    const unsigned long now = millis();

    // update() is internally rate-limited, but the parameter increment is
    // retained until the next sample. Apply it only once per new position so
    // a fast loop cannot reuse the same physical movement.
    const int32_t currentPosition = encoder.getCumulativePosition();
    if (currentPosition != lastAppliedPosition)
    {
        // Apply a velocity-scaled increment to the parameter.
        const float increment =
            encoder.getParameterIncrement(PARAM_MIN, PARAM_MAX, MAX_ROTATIONS);
        parameter += increment;
        parameter = constrain(parameter, PARAM_MIN, PARAM_MAX);
        lastAppliedPosition = currentPosition;
    }

  
    // Print only when the value actually changes, to avoid flooding.
    static float lastPrinted = -1.0f;
    if (fabsf(parameter - lastPrinted) > 0.0008f)
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
    if (now - lastDraw >= 50)
    {
        lastDraw = now;
        drawScreen();
    }
#endif
}
