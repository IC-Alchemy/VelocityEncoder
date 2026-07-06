// ============================================================
// MIDI2HiResKnob — AS5600 encoder as a high-resolution
// MIDI 2.0 controller on a Raspberry Pi Pico 2 (RP2350)
// ============================================================
//
// Turns the MagEncoder velocity-scaled knob into a 32-bit
// MIDI 2.0 Control Change (CC 1, channel 1, group 1) using the
// AmeNote tusb_ump class driver for TinyUSB.
//
//   Slow turn  → per-tick steps of a few parts in 65536: fine,
//                zipper-free sweeps that plain 7-bit MIDI 1.0
//                CCs simply cannot express.
//   Fast turn  → covers the full range in about a quarter turn.
//
// Host compatibility (handled automatically):
//   • UMP-capable host (Windows MIDI Services, macOS 14+,
//     Linux ALSA seq UMP) selects Alt Setting 1 → we send
//     MIDI 2.0 Channel Voice CC packets with a 32-bit value.
//   • Legacy MIDI 1.0 host stays on Alt Setting 0 → we send
//     14-bit CC pairs (CC1 MSB + CC33 LSB); the tusb_ump driver
//     converts the UMP MIDI 1.0 packets to USB MIDI 1.0 for us.
//     (The driver DROPS MIDI 2.0 CV packets in that mode, so the
//     downgrade must happen here in the sketch.)
//
// Wiring (Pico 2, default Wire pins):
//   AS5600 VCC → 3V3, GND → GND, SDA → GP4, SCL → GP5
//
// SPDX-License-Identifier: MIT
// ============================================================

#if defined(PLATFORMIO)

#include <Arduino.h>
#include <MagEncoder.h>
#include "ump_device.h"
#include "ump_stream_handler.h"

// Injects the MIDI 2.0 config descriptor into TinyUSBDevice.
// Must be the first thing called in setup(). See usb_descriptors.cpp.
extern void usb_descriptors_begin(void);

// ── What we send ─────────────────────────────────────────────
static const uint8_t UMP_GROUP   = 0;  // UMP group 1
static const uint8_t MIDI_CH     = 0;  // channel 1
static const uint8_t CC_NUMBER   = 1;  // mod wheel (CC 33 = LSB in MIDI 1.0)
static const int     KNOB_TURNS  = 4;  // full turns to sweep min → max at 1:1

// ── UMP Endpoint Discovery identity (MT=0xF stream messages) ─
static const UMPStreamConfig streamCfg = {
    .umpVersionMajor      = 1,
    .umpVersionMinor      = 1,
    .numFunctionBlocks    = 1,
    .staticFunctionBlocks = true,
    .protocolCaps         = UMP_PROTO_CAP_MIDI1 | UMP_PROTO_CAP_MIDI2,
    .manufacturerId       = { 0x00, 0x00, 0x7D },  // 0x7D = educational/dev
    .familyId             = 0x0001,
    .modelId              = 0x0010,
    .swRevision           = { '0', '1', '0', '0' },
    .endpointName         = "AS5600 HiRes",        // ≤14 chars → 1 packet
    .productInstanceId    = "AS5600MIDI2001",      // ≤14 chars → 1 packet
    .fbName               = "Knob",
    .fbDirection          = UMP_FB_DIR_BIDIRECTIONAL,
    .fbFirstGroup         = 0,
    .fbNumGroups          = 1,
    .fbUIHint             = 0x00,
    .fbMidi10             = 0x00,
    .fbMidiCIVer          = 0x00,
    .fbSysEx8             = 0x00,
};

// ── State ────────────────────────────────────────────────────
static MagEncoder encoder;          // default velocity curve; see README
static float      param       = 0.0f;   // normalized knob value [0..1]
static uint32_t   lastSent32  = 0xFFFFFFFF;
static uint16_t   lastSent14  = 0xFFFF;
static bool       usbWasUp    = false;

// ── UMP packet builders ──────────────────────────────────────
// tusb_ump stores UMP words in wire order: byte 0 (the uint32_t
// LSB) is the first byte on the USB wire and carries the MT
// nibble. packWord() comes from ump_stream_handler.h and matches
// the driver's own convention.

// MIDI 2.0 Channel Voice CC (MT=0x4, 64 bits): 32-bit value.
static void sendCC32(uint32_t value32)
{
    uint32_t w[2];
    w[0] = packWord(0x40 | UMP_GROUP,          // MT=4, group
                    0xB0 | MIDI_CH,            // CC status, channel
                    CC_NUMBER, 0x00);          // index, reserved
    w[1] = packWord((uint8_t)(value32 >> 24), (uint8_t)(value32 >> 16),
                    (uint8_t)(value32 >> 8),  (uint8_t)value32);
    if (tud_ump_n_writeable(0) >= 2) tud_ump_write(0, w, 2);
}

// MIDI 1.0 Channel Voice CC (MT=0x2, 32 bits): one 7-bit CC.
static void sendCC7(uint8_t cc, uint8_t value7)
{
    uint32_t w = packWord(0x20 | UMP_GROUP, 0xB0 | MIDI_CH, cc, value7);
    if (tud_ump_n_writeable(0) >= 1) tud_ump_write(0, &w, 1);
}

// ── RX: answer UMP Endpoint Discovery ────────────────────────
// A UMP host interrogates the device with MT=0xF stream messages
// right after selecting Alt 1. Without these replies, Windows
// MIDI Services / macOS will not finish creating the endpoint.

static inline uint8_t umpWordsForMT(uint8_t mt)
{
    switch (mt) {
    case 0x0: case 0x1: case 0x2: case 0x6: case 0x7: return 1;
    case 0x3: case 0x4: case 0x8: case 0x9: case 0xA: case 0xD: return 2;
    case 0xB: case 0xC: return 3;
    default: return 4;   // 0x5, 0xE, 0xF
    }
}

static void processRxUMP()
{
    for (int pass = 0; pass < 16; pass++) {
        if (tud_ump_n_available(0) == 0) break;

        uint32_t words[4] = { 0, 0, 0, 0 };
        if (tud_ump_read(0, &words[0], 1) == 0) break;

        uint8_t mt = (uint8_t)((words[0] & 0xF0) >> 4);
        uint8_t nw = umpWordsForMT(mt);

        if (nw > 1 && tud_ump_n_available(0) < (uint32_t)(nw - 1)) break;
        for (uint8_t i = 1; i < nw; i++) tud_ump_read(0, &words[i], 1);

        if (mt == 0xF)
            umpStreamHandleRx(0, words, streamCfg, nullptr);
        // Everything else is ignored — this device is a controller.
    }
}

// ── Value scaling ────────────────────────────────────────────
// float [0..1] → 16-bit, then upscale 16 → 32 by bit replication
// (the min-center-max upscaling recommended by the MIDI 2.0
// spec). Avoids float-precision artifacts at full scale.

static inline uint16_t to16(float v)
{
    if (v <= 0.0f) return 0;
    if (v >= 1.0f) return 0xFFFF;
    return (uint16_t)lroundf(v * 65535.0f);
}

static inline uint32_t upscale16to32(uint16_t v)
{
    return ((uint32_t)v << 16) | v;
}

// ── Arduino ──────────────────────────────────────────────────

void setup()
{
    usb_descriptors_begin();   // must be first — before Serial

    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    if (!encoder.begin()) {
        Serial.println("[knob] AS5600 not found — check wiring.");
        while (true) {                       // fast blink = no sensor
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(100);
        }
    }
    Serial.println("[knob] AS5600 MIDI 2.0 hi-res knob ready");
}

void loop()
{
    bool mounted = tud_ump_n_mounted(0);
    if (mounted != usbWasUp) {
        usbWasUp = mounted;
        digitalWrite(LED_BUILTIN, mounted ? HIGH : LOW);
    }

    if (mounted) processRxUMP();

    encoder.update();

    // Velocity-scaled increment: slow = fine, fast = wide.
    float inc = encoder.getParameterIncrement(0.0f, 1.0f, KNOB_TURNS);
    param = constrain(param + inc, 0.0f, 1.0f);

    if (mounted) {
        uint16_t v16 = to16(param);

        if (tud_alt_setting(0) == 1) {
            // UMP host → one 32-bit MIDI 2.0 CC
            uint32_t v32 = upscale16to32(v16);
            if (v32 != lastSent32) {
                sendCC32(v32);
                lastSent32 = v32;
                lastSent14 = 0xFFFF;         // force resync on downgrade
            }
        } else {
            // MIDI 1.0 host → 14-bit CC pair (MSB first, then LSB)
            uint16_t v14 = v16 >> 2;
            if (v14 != lastSent14) {
                sendCC7(CC_NUMBER,      (uint8_t)((v14 >> 7) & 0x7F));
                sendCC7(CC_NUMBER + 32, (uint8_t)(v14 & 0x7F));
                lastSent14 = v14;
                lastSent32 = 0xFFFFFFFF;     // force resync on upgrade
            }
        }
    }

    delay(1);   // encoder reads are throttled internally (5 ms default)
}

#endif // defined(PLATFORMIO)
