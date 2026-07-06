#ifndef MagEncoder_h
#define MagEncoder_h

#include <Arduino.h>
#include <Wire.h>

/**
 * MagEncoder
 * ----------
 * Arduino driver for the AS5600 12-bit magnetic rotary encoder, with
 * velocity-sensitive parameter control built in.
 *
 * Wraps I2C communication with the AS5600 and provides:
 *   - Raw and normalized angle reads (0..4095, 0.0..1.0).
 *   - Cumulative position tracking with 12-bit wrap-around correction, so
 *     multi-turn motion across several revolutions is reported as a single
 *     monotonically-increasing (or decreasing) int32_t.
 *   - Filtered angular speed in degrees/second (adaptive low-pass filter).
 *   - A velocity-scaled "parameter increment" for knob-driven musical or UI
 *     applications: slow turns give fine control, fast turns cover more range,
 *     with a tunable response curve.
 *
 * Every number that shapes the knob feel is exposed either as a constructor
 * argument or as a configurable default, so the library carries no dependency
 * on any particular instrument or application.
 *
 * Wire must be available on the target board; the class calls Wire.begin().
 * Builds on any Arduino core that provides a working TwoWire implementation.
 */
class MagEncoder
{
public:
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
        uint8_t  i2cAddress        = 0x36;   // AS5600 default I2C address
        uint32_t readIntervalMs    = 5;      // Minimum ms between sensor reads
        float    minVelDps         = 90.0f;  // Below this speed, output is clamped to minScale
        float    maxVelDps         = 2400.0f; // Above this speed, output is clamped to maxScale
        float    minScale          = 0.008f; // Slow-turn scale (parameter increment multiplier)
        float    maxScale          = 3.2f;   // Fast-turn scale (parameter increment multiplier)
        float    curveExponent     = 1.8f;   // Mid-range curve shape
        float    velocitySmoothing = 0.08f;  // EMA factor for the velocity scale (smaller = smoother)
    };

    /**
     * Construct an encoder with default tuning.
     */
    MagEncoder();

    /**
     * Construct an encoder with custom response tuning.
     */
    explicit MagEncoder(const Config &config);

    /**
     * Initialize I2C and verify the sensor is present at the configured address.
     * Returns true if the sensor acknowledged, false otherwise. On success,
     * also seeds the cumulative position and speed baseline from the current
     * reading so the first update() call doesn't produce a spurious delta.
     */
    bool begin();

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

    /** Raw 12-bit angle from the AS5600 RAW_ANGLE register (0..4095). */
    uint16_t getRawAngle() const;

    /** Raw angle normalized to [0.0, 1.0]. */
    float getNormalizedAngle() const;

    /**
     * Cumulative position in encoder ticks, with wrap-around unwrapped across
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

    /** Coarse qualitative description of the current turn speed. */
    enum class VelocityZone { Idle, Low, Mid, High };
    VelocityZone getVelocityZone() const;

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

    // 12-bit sensor -> 4096 ticks per revolution.
    static constexpr uint16_t TICKS_PER_REV    = 4096;
    static constexpr int16_t  WRAP_THRESHOLD   = TICKS_PER_REV / 2; // 2048
    static constexpr float    RAW_TO_NORMALIZED = 1.0f / 4095.0f;
    static constexpr float    RAW_TO_DEGREES    = 360.0f / 4096.0f;

    Config _cfg;

    bool          _connected;
    uint16_t      _rawAngle;
    uint16_t      _lastRawAngle;
    int32_t       _cumulativePosition;
    uint16_t      _lastPosition;
    float         _angularSpeed;
    unsigned long _lastReadTime;
    unsigned long _lastSpeedTime;
    mutable float _lastCurvedSpeed;

    // I2C helpers
    uint16_t readRegister16(uint8_t reg) const;
    bool     checkConnection();

    // Unwrap the 12-bit wrap-around between two raw angle samples so that
    // multi-turn motion is reported as a signed delta in [-2048, 2047].
    static int16_t unwrapAngleDelta(uint16_t current, uint16_t previous);

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
