#ifndef MagEncoder_h
#define MagEncoder_h

#include <Arduino.h>
#include <Wire.h>

#include "TMAG5273.h"

/**
 * MagEncoder
 * ----------
 * Arduino driver for magnetic rotary position sensors, with velocity-sensitive
 * parameter control built in.
 *
 * Two sensors are supported and selected through Config::sensor:
 *   - Sensor::AS5600   — AMS AS5600, 12-bit on-axis magnetic encoder
 *                        (4096 counts per revolution, I2C address 0x36).
 *   - Sensor::TMAG5273 — TI TMAG5273, 3D Hall-effect sensor whose CORDIC
 *                        engine reports angle at 1/16 degree
 *                        (5760 counts per revolution, I2C address 0x22 for
 *                        the B parts fitted on the Velocity Encoder board;
 *                        see TMAG5273::ADDRESS_A..ADDRESS_D for the others).
 *
 * Everything above the raw angle read is shared, so the knob feel is identical
 * on both parts and a sketch can switch sensors by changing one field:
 *   - Raw and normalized angle (0..countsPerRevolution-1, 0.0..1.0).
 *   - Cumulative position tracking with wrap-around correction, so multi-turn
 *     motion across several revolutions is reported as a single monotonically-
 *     increasing (or decreasing) int32_t.
 *   - Filtered angular speed in degrees/second (adaptive low-pass filter).
 *   - A velocity-scaled "parameter increment" for knob-driven musical or UI
 *     applications: slow turns give fine control, fast turns cover more range,
 *     with a tunable response curve.
 *
 * Every number that shapes the knob feel is exposed either as a constructor
 * argument or as a configurable default, so the library carries no dependency
 * on any particular instrument or application.
 *
 * When the TMAG5273 is selected, tmag() gives direct access to the underlying
 * driver so a sketch can also read the three magnetic axes, die temperature,
 * vector magnitude and diagnostics that the AS5600 has no equivalent for.
 *
 * A TwoWire instance must be available on the target board; begin() calls
 * Wire.begin() (or the bus you pass in). Builds on any Arduino core that
 * provides a working TwoWire implementation.
 */
class MagEncoder
{
public:
    /** Which physical sensor the encoder is reading. */
    enum class Sensor : uint8_t
    {
        AS5600,
        TMAG5273
    };

    /**
     * Tunable response curve parameters.
     *
     * The defaults are the curve we use in our own instruments. A slow turn
     * gives very fine control — per-call increments in the 0.01 range — while a
     * fast twist can move the value a thousand to ten thousand times faster.
     * Override any field in the constructor to retune the knob feel without
     * rewriting the velocity logic.
     */
    struct Config
    {
        Sensor   sensor            = Sensor::AS5600; // Which part is fitted
        uint8_t  i2cAddress        = 0;      // 0 = the selected sensor's default address
        uint32_t readIntervalMs    = 5;      // Minimum ms between sensor reads
        float    minVelDps         = 90.0f;  // Below this speed, output is clamped to minScale
        float    maxVelDps         = 2400.0f; // Above this speed, output is clamped to maxScale
        float    minScale          = 0.008f; // Slow-turn scale (parameter increment multiplier)
        float    maxScale          = 3.2f;   // Fast-turn scale (parameter increment multiplier)
        float    curveExponent     = 1.8f;   // Mid-range curve shape
        float    velocitySmoothing = 0.08f;  // EMA factor for the velocity scale (smaller = smoother)

        /**
         * TMAG5273-only settings, ignored when sensor is AS5600. The i2cAddress
         * field above wins over tmag.i2cAddress so both sensors are addressed
         * the same way.
         */
        TMAG5273::Config tmag;
    };

    /** Default I2C address of each supported sensor. */
    static constexpr uint8_t AS5600_ADDRESS   = 0x36;
    // The Velocity Encoder board fits a TMAG5273B, which answers at 0x22, so
    // that is what the "0 = use the default" sentinel resolves to. Pass
    // ADDRESS_A / _C / _D in Config::i2cAddress for the other factory-
    // programmed variants.
    static constexpr uint8_t TMAG5273_ADDRESS = TMAG5273::ADDRESS_B;

    /**
     * Construct an encoder with default tuning, reading an AS5600.
     */
    MagEncoder();

    /**
     * Construct an encoder with default tuning for the given sensor. Equivalent
     * to filling in Config::sensor and leaving everything else alone.
     */
    explicit MagEncoder(Sensor sensor);

    /**
     * Construct an encoder with custom response tuning.
     */
    explicit MagEncoder(const Config &config);

    /**
     * Initialize I2C and verify the sensor is present at the configured
     * address. For the TMAG5273 this also verifies the manufacturer ID and
     * writes the sensor configuration.
     *
     * Returns true if the sensor acknowledged, false otherwise. On success,
     * also seeds the cumulative position and speed baseline from the current
     * reading so the first update() call doesn't produce a spurious delta.
     *
     * Pass a different TwoWire instance to run the sensor on a secondary I2C
     * bus (Wire1 on boards that have one).
     */
    bool begin(TwoWire &wire = Wire);

    /**
     * Read the current angle from the sensor and update derived state
     * (cumulative position, angular speed). Call this regularly from loop().
     * Reads are throttled to readIntervalMs; calls closer than that together
     * are no-ops, so it's safe to call every loop iteration.
     */
    void update();

    // ------------------------------------------------------------------
    // Read-only state accessors
    // ------------------------------------------------------------------

    /**
     * Raw angle in the sensor's native counts: 0..4095 from the AS5600 ANGLE
     * register, 0..5759 from the TMAG5273 CORDIC angle engine.
     */
    uint16_t getRawAngle() const;

    /** Raw angle normalized to [0.0, 1.0]. */
    float getNormalizedAngle() const;

    /** Current shaft angle in degrees, 0.0 .. 360.0. */
    float getAngleDegrees() const;

    /**
     * Cumulative position in encoder counts, with wrap-around unwrapped across
     * multiple revolutions. Resets to 0 (or a value you supply) via
     * resetCumulativePosition().
     */
    int32_t getCumulativePosition() const;

    /** Filtered angular speed in degrees/second. */
    float getAngularSpeed() const;

    /** Position normalized to [0.0, 1.0] across `maxRotations` full turns. */
    float getPositionPercentage(uint8_t maxRotations = 4) const;

    /** True if begin() detected the sensor on the bus. */
    bool isConnected() const;

    /** Which sensor this instance is configured for. */
    Sensor getSensor() const;

    /** Human-readable sensor name, "AS5600" or "TMAG5273". */
    const char *getSensorName() const;

    /** The 7-bit I2C address in use, with the "0 = default" sentinel resolved. */
    uint8_t getI2CAddress() const;

    /** Native counts per full revolution: 4096 (AS5600) or 5760 (TMAG5273). */
    uint16_t getCountsPerRevolution() const;

    /** Coarse qualitative description of the current turn speed. */
    enum class VelocityZone { Idle, Low, Mid, High };
    VelocityZone getVelocityZone() const;

    // ------------------------------------------------------------------
    // TMAG5273 extras
    // ------------------------------------------------------------------

    /**
     * The underlying TMAG5273 driver. Only meaningful when the encoder is
     * configured for Sensor::TMAG5273 — update() keeps it fed, so a sketch can
     * read the magnetic axes, temperature, magnitude and diagnostics straight
     * off it without a second I2C transaction.
     */
    TMAG5273 &tmag();
    const TMAG5273 &tmag() const;

    // ------------------------------------------------------------------
    // Velocity-sensitive parameter control
    // ------------------------------------------------------------------

    /**
     * Compute a velocity-scaled increment suitable for adjusting a parameter
     * across [minVal, maxVal].
     *
     * The returned value is the per-call change to apply to the parameter:
     * sign indicates direction, magnitude scales with how fast the knob is
     * being turned. `maxRotations` controls how many full turns span the full
     * [minVal, maxVal] range.
     *
     * Example:
     *   float inc = encoder.getParameterIncrement(0.0, 1.0, 4);
     *   myParam = constrain(myParam + inc, 0.0, 1.0);
     */
    float getParameterIncrement(float minVal, float maxVal, uint8_t maxRotations = 4) const;

    /**
     * Consuming counterpart to getParameterIncrement().
     *
     * getParameterIncrement() derives its delta from the last two *sensor
     * reads*, which are throttled to readIntervalMs. A control loop that spins
     * faster than that therefore sees the same delta on every iteration and
     * applies it several times — the parameter runs away.
     * takeParameterIncrement() instead drains an internal tick accumulator that
     * update() fills, so every encoder count is applied exactly once no matter
     * how often the caller polls.
     *
     * Prefer this in any loop that is not hand-paced to readIntervalMs.
     *
     * Example:
     *   encoder.update();
     *   param = constrain(param + encoder.takeParameterIncrement(0, 1, 4), 0, 1);
     */
    float takeParameterIncrement(float minVal, float maxVal, uint8_t maxRotations = 4);

    /** Unconsumed encoder counts since the last drain (sign = direction). */
    int32_t pendingTicks() const;

    /** Drop any unconsumed counts (e.g. after a mode switch). */
    void clearPendingTicks();

    /**
     * Current velocity multiplier, in [Config::minScale, Config::maxScale].
     * Exposed so a UI can display the same number the increment math uses.
     */
    float getVelocityScale() const;

    /**
     * Convenience: map the current cumulative position directly into
     * [minVal, maxVal] (no velocity scaling). Useful for absolute-position
     * knobs rather than incremental ones.
     */
    float mapPositionToRange(float minVal, float maxVal, uint8_t maxRotations = 4) const;

    // ------------------------------------------------------------------
    // State management
    // ------------------------------------------------------------------

    /**
     * Reset the cumulative position counter to `position` and reseed the
     * internal baseline so the next update() doesn't produce a large delta.
     * Call this whenever the meaning of "zero" changes (e.g. the user lifts
     * and replaces the knob).
     */
    void resetCumulativePosition(int32_t position = 0);

private:
    // AS5600 register addresses (high byte; low byte follows sequentially).
    static constexpr uint8_t REG_RAW_ANGLE = 0x0C;
    static constexpr uint8_t REG_ANGLE     = 0x0E;

    // AS5600 is a 12-bit sensor -> 4096 counts per revolution.
    static constexpr uint16_t AS5600_COUNTS_PER_REV = 4096;

    Config    _cfg;
    TwoWire  *_wire;
    TMAG5273  _tmag;

    // Per-sensor geometry, resolved in the constructor so the shared position
    // and velocity math works in whichever tick space the part reports.
    uint16_t _countsPerRev;
    int16_t  _wrapThreshold;
    float    _countsToDegrees;
    float    _countsToNormalized;

    bool          _connected;
    uint16_t      _rawAngle;
    uint16_t      _lastRawAngle;
    int32_t       _cumulativePosition;
    int32_t       _pendingTicks;   // drained by takeParameterIncrement()
    uint16_t      _lastPosition;
    uint16_t      _lastSpeedAngle;  // angle at the last speed sample
    float         _angularSpeed;
    unsigned long _lastReadTime;
    unsigned long _lastSpeedTime;
    mutable float _lastCurvedSpeed;

    // Fill in _countsPerRev and friends from _cfg.sensor, and resolve the
    // "0 means default" I2C address sentinel.
    void configureForSensor();

    // I2C helpers
    uint16_t readAS5600Register16(uint8_t reg) const;
    bool     checkConnection();

    // Read the current angle from whichever sensor is configured.
    uint16_t readAngle();

    // Unwrap the wrap-around between two raw angle samples so that multi-turn
    // motion is reported as a signed delta of at most half a revolution.
    int16_t unwrapAngleDelta(uint16_t current, uint16_t previous) const;

    // Internal update steps
    void updateCumulativePosition();
    void updateAngularSpeed(unsigned long currentTime);

    // Velocity-curve internals (return a multiplier in [_cfg.minScale, _cfg.maxScale])
    float calculateVelocityScale(float absSpeedDps) const;
    float normalizeSpeed(float absSpeedDps) const;
    float applyVelCurve(float normedSpeed) const;
    float smoothVel(float curvedSpeed) const;
};

#endif // MagEncoder_h
