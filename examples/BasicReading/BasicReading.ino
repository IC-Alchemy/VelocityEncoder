/*
 * BasicReading
 * ------------
 * Minimal usage of the MagEncoder (AS5600) library: initialize the sensor and
 * print the raw angle, normalized angle, cumulative position, and angular
 * speed to the Serial Monitor every 50 ms.
 *
 * Wiring: connect the AS5600's SDA/SCL to your board's I2C pins and power
 * the sensor from 3.3V/5V as appropriate.
 */

#include <MagEncoder.h>

MagEncoder encoder;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Wait for the USB serial port to connect (native USB boards only).
    }

    if (!encoder.begin())
    {
        Serial.println("AS5600 not found. Check wiring and I2C address.");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("AS5600 ready.");
}

void loop()
{
    encoder.update();

    Serial.print("raw=");
    Serial.print(encoder.getRawAngle());
    Serial.print("  norm=");
    Serial.print(encoder.getNormalizedAngle(), 3);
    Serial.print("  cum=");
    Serial.print(encoder.getCumulativePosition());
    Serial.print("  speed(dps)=");
    Serial.println(encoder.getAngularSpeed(), 1);

    delay(50);
}
