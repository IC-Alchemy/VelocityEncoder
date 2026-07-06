/*
 * MIDI2HiResKnob
 * --------------
 * Arduino IDE compatible USB MIDI version of the AS5600 high-resolution knob.
 *
 * This sketch sends a MIDI 1.0 high-resolution 14-bit Control Change pair:
 * CC 1 carries the MSB and CC 33 carries the LSB. It is the portable Arduino
 * fallback for hosts and boards that support the Arduino MIDIUSB library.
 *
 * The PlatformIO build in src/main.cpp sends true MIDI 2.0 UMP with a 32-bit
 * value. Arduino IDE builds generally cannot enable that TinyUSB UMP class
 * driver at compile time, so this sketch intentionally uses USB MIDI 1.0.
 *
 * Requirements:
 *   - A native USB Arduino-compatible board supported by MIDIUSB.
 *   - Install the "MIDIUSB" library from Library Manager.
 *   - Connect the AS5600 to the board's SDA/SCL pins.
 */

#include <MagEncoder.h>
#include <MIDIUSB.h>

static const uint8_t MIDI_CHANNEL = 1;       // Human channel number: 1..16
static const uint8_t CC_MSB       = 1;       // Mod wheel MSB
static const uint8_t CC_LSB       = CC_MSB + 32;
static const uint8_t KNOB_TURNS   = 4;       // Turns for a full 0..1 sweep

static MagEncoder encoder;
static float      parameter  = 0.0f;
static uint16_t   lastSent14 = 0xFFFF;

static uint16_t to14Bit(float value)
{
    if (value <= 0.0f)
        return 0;
    if (value >= 1.0f)
        return 0x3FFF;

    return static_cast<uint16_t>((value * 16383.0f) + 0.5f);
}

static void sendControlChange(uint8_t channel, uint8_t controller, uint8_t value)
{
    const uint8_t status = 0xB0 | ((channel - 1) & 0x0F);
    midiEventPacket_t event = {
        0x0B,                         // USB MIDI CIN: Control Change
        status,
        static_cast<uint8_t>(controller & 0x7F),
        static_cast<uint8_t>(value & 0x7F)
    };
    MidiUSB.sendMIDI(event);
}

static void send14BitControlChange(uint16_t value14)
{
    value14 &= 0x3FFF;

    sendControlChange(MIDI_CHANNEL, CC_MSB, static_cast<uint8_t>((value14 >> 7) & 0x7F));
    sendControlChange(MIDI_CHANNEL, CC_LSB, static_cast<uint8_t>(value14 & 0x7F));
    MidiUSB.flush();
}

static void haltNoSensor()
{
    Serial.println("AS5600 not found. Check wiring and I2C address.");

    while (true)
    {
#if defined(LED_BUILTIN)
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif
        delay(100);
    }
}

void setup()
{
    Serial.begin(115200);

#if defined(LED_BUILTIN)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
#endif

    const unsigned long serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart < 2000))
    {
        ; // Native USB boards only.
    }

    if (!encoder.begin())
        haltNoSensor();

    Serial.println("AS5600 USB MIDI 14-bit high-resolution knob ready.");
    Serial.println("Sending CC 1/33 on MIDI channel 1.");
}

void loop()
{
    encoder.update();

    const float increment = encoder.getParameterIncrement(0.0f, 1.0f, KNOB_TURNS);
    parameter = constrain(parameter + increment, 0.0f, 1.0f);

    const uint16_t value14 = to14Bit(parameter);
    if (value14 != lastSent14)
    {
        send14BitControlChange(value14);
        lastSent14 = value14;

#if defined(LED_BUILTIN)
        digitalWrite(LED_BUILTIN, HIGH);
#endif
    }
    else
    {
#if defined(LED_BUILTIN)
        digitalWrite(LED_BUILTIN, LOW);
#endif
    }

    static unsigned long lastDebugMs = 0;
    const unsigned long now = millis();
    if (now - lastDebugMs >= 250)
    {
        lastDebugMs = now;
        Serial.print("value14=");
        Serial.print(value14);
        Serial.print(" parameter=");
        Serial.print(parameter, 4);
        Serial.print(" speed(dps)=");
        Serial.println(encoder.getAngularSpeed(), 1);
    }

    delay(1);
}
