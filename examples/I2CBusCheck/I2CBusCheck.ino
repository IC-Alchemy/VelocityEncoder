/*
 * I2CBusCheck
 * -----------
 * Diagnostic-only sketch. No library, no display, no sensor driver — just the
 * bus. Run this before debugging anything else when a device on a shared I2C
 * chain misbehaves.
 *
 * It does three things:
 *   1  Reports the idle DC level of SDA and SCL with the pins as plain inputs.
 *      Both must read HIGH. A line stuck LOW means a device is holding the bus
 *      or a pull-up is missing, and no amount of software will fix it.
 *   2  Scans every address at 100 kHz, then again at 400 kHz. A device that
 *      answers at 100 but not 400 has a wiring/rise-time problem, not a
 *      configuration problem.
 *   3  Names the addresses it knows about.
 *
 * Edit SDA_PIN / SCL_PIN below to match your board if they are not the core's
 * defaults. On the RP2040/RP2350 Arduino core, Wire defaults to GP4 (SDA) and
 * GP5 (SCL); Wire1 defaults to GP26/GP27.
 */

#include <Wire.h>

// Set to -1 to leave the core's default pins alone.
static const int SDA_PIN = 4;
static const int SCL_PIN = 5;

#define I2C_PORT Wire // change to Wire1 if that is the port in use

static const char *knownDevice(uint8_t address)
{
    switch (address)
    {
        case 0x35: return "TMAG5273A1/A2";
        case 0x22: return "TMAG5273B1/B2";
        case 0x78: return "TMAG5273C1/C2";
        case 0x44: return "TMAG5273D1/D2";
        case 0x3C: return "SSD1306/SH1106 OLED";
        case 0x3D: return "SSD1306/SH1106 OLED (alt)";
        default:   return nullptr;
    }
}

static void reportIdleLevels()
{
    if (SDA_PIN < 0 || SCL_PIN < 0)
        return;

    // Released as inputs, the pull-ups on the bus should hold both lines high.
    pinMode(SDA_PIN, INPUT);
    pinMode(SCL_PIN, INPUT);
    delay(5);

    const int sda = digitalRead(SDA_PIN);
    const int scl = digitalRead(SCL_PIN);

    Serial.print("Idle SDA (GP");
    Serial.print(SDA_PIN);
    Serial.print("): ");
    Serial.println(sda ? "HIGH  ok" : "LOW   <-- stuck, bus is held down");

    Serial.print("Idle SCL (GP");
    Serial.print(SCL_PIN);
    Serial.print("): ");
    Serial.println(scl ? "HIGH  ok" : "LOW   <-- stuck, bus is held down");

    if (!sda || !scl)
    {
        Serial.println();
        Serial.println("A line stuck LOW with no bus traffic means one of:");
        Serial.println("  - a device is powered off or unpowered while wired to the bus");
        Serial.println("  - the TMAG5273 TEST pin is floating (it must go to GND)");
        Serial.println("  - a device is mid-transaction from a previous run: power cycle");
        Serial.println("  - no pull-up resistors on that line at all");
    }
    Serial.println();
}

static uint8_t scanAt(uint32_t frequency)
{
    I2C_PORT.setClock(frequency);
    delay(5);

    Serial.print("--- scan at ");
    Serial.print(frequency / 1000);
    Serial.println(" kHz ---");

    uint8_t found = 0;

    for (uint8_t address = 0x08; address <= 0x77; address++)
    {
        I2C_PORT.beginTransmission(address);
        if (I2C_PORT.endTransmission() != 0)
            continue;

        found++;
        Serial.print("  0x");
        if (address < 0x10)
            Serial.print('0');
        Serial.print(address, HEX);

        const char *name = knownDevice(address);
        if (name)
        {
            Serial.print("  ");
            Serial.print(name);
        }
        Serial.println();
    }

    if (found == 0)
        Serial.println("  nothing answered");

    Serial.println();
    return found;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("I2CBusCheck");
    Serial.println("===========");

    reportIdleLevels();

#if defined(ARDUINO_ARCH_RP2040)
    if (SDA_PIN >= 0)
        I2C_PORT.setSDA(SDA_PIN);
    if (SCL_PIN >= 0)
        I2C_PORT.setSCL(SCL_PIN);
#endif

    I2C_PORT.begin();

    const uint8_t at100 = scanAt(100000UL);
    const uint8_t at400 = scanAt(400000UL);

    Serial.println("--- verdict ---");
    if (at100 == 0)
    {
        Serial.println("Nothing on the bus at all. Check SDA/SCL pin numbers,");
        Serial.println("that both devices share a ground, and that each has power.");
    }
    else if (at400 < at100)
    {
        Serial.println("Devices drop out at 400 kHz. Rise time is too slow:");
        Serial.println("shorten the wires, or run the bus at 100 kHz.");
    }
    else
    {
        Serial.println("Bus is clean at both speeds.");
    }
    Serial.println("Unplug one device at a time and rerun to see who is who.");
}

void loop()
{
    delay(1000);
}
