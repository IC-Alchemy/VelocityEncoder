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
 * Wiring: connect the AS5600's SDA/SCL to your board's I2C pins and power
 * the sensor from 3.3V/5V as appropriate. Wire a pushbutton between pin 7
 * and ground to act as the "reset position" key (optional).
 */

#include <MagEncoder.h>

static const int   RESET_BUTTON_PIN = 7;
static const float PARAM_MIN        = 0.0f;
static const float PARAM_MAX        = 1.0f;
static const uint8_t MAX_ROTATIONS  = 4;

MagEncoder encoder;
float      parameter = 0.0f;

void setup()
{
    Serial.begin(115200);

    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

    // Custom tuning: a wider dynamic range than the defaults.
    MagEncoder::Config cfg;
    cfg.minScale   = 0.005f;
    cfg.maxScale   = 4.0f;
    encoder = MagEncoder(cfg);

    if (!encoder.begin())
    {
        Serial.println("AS5600 not found. Check wiring.");
        while (true)
        {
            delay(1000);
        }
    }

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

        switch (encoder.getVelocityZone())
        {
            case MagEncoder::VelocityZone::Idle: Serial.print("  (idle)");  break;
            case MagEncoder::VelocityZone::Low:  Serial.print("  (low)");   break;
            case MagEncoder::VelocityZone::Mid:  Serial.print("  (mid)");   break;
            case MagEncoder::VelocityZone::High: Serial.print("  (high)");  break;
        }
        Serial.println();
    }

    delay(5);
}
