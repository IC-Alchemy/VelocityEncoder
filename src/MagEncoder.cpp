#include "MagEncoder.h"

#include <algorithm> // std::min / std::max, used by the velocity curve
#include <cmath>

namespace
{
    // --- Adaptive low-pass filter parameters (updateAngularSpeed) ---

    // Minimum time between speed samples; below this the dt is too small for a
    // stable derivative.
    constexpr unsigned long MIN_SPEED_DT_MS = 8;

    // Speed (deg/s) thresholds that select the EMA alpha.
    constexpr float SLOW_DPS   = 30.0f;
    constexpr float MEDIUM_DPS = 70.0f;

    // EMA factors: less smoothing for faster movements.
    constexpr float SMOOTH_ALPHA_SLOW   = 0.3f;
    constexpr float SMOOTH_ALPHA_MEDIUM = 0.4f;
    constexpr float SMOOTH_ALPHA_FAST   = 0.6f;

    // Below this filtered speed the output is attenuated to suppress jitter.
    constexpr float NOISE_GATE_DPS = 1.0f;

    MagEncoder::Config configForSensor(MagEncoder::Sensor sensor)
    {
        MagEncoder::Config cfg;
        cfg.sensor = sensor;
        return cfg;
    }
}

MagEncoder::MagEncoder()
    : MagEncoder(Config())
{
}

MagEncoder::MagEncoder(Sensor sensor)
    : MagEncoder(configForSensor(sensor))
{
}

MagEncoder::MagEncoder(const Config &config)
    : _cfg(config),
      _wire(&Wire),
      _countsPerRev(AS5600_COUNTS_PER_REV),
      _wrapThreshold(AS5600_COUNTS_PER_REV / 2),
      _countsToDegrees(360.0f / AS5600_COUNTS_PER_REV),
      _countsToNormalized(1.0f / (AS5600_COUNTS_PER_REV - 1)),
      _connected(false),
      _rawAngle(0),
      _lastRawAngle(0),
      _cumulativePosition(0),
      _pendingTicks(0),
      _lastPosition(0),
      _lastSpeedAngle(0),
      _angularSpeed(0.0f),
      _lastReadTime(0),
      _lastSpeedTime(0),
      _lastCurvedSpeed(0.0f)
{
    configureForSensor();
}

void MagEncoder::configureForSensor()
{
    // Resolve the "0 means use the sensor's own default" address sentinel.
    if (_cfg.i2cAddress == 0)
    {
        _cfg.i2cAddress = (_cfg.sensor == Sensor::TMAG5273) ? TMAG5273_ADDRESS
                                                            : AS5600_ADDRESS;
    }

    _countsPerRev = (_cfg.sensor == Sensor::TMAG5273)
                        ? TMAG5273::ANGLE_COUNTS_PER_REV
                        : AS5600_COUNTS_PER_REV;

    _wrapThreshold      = static_cast<int16_t>(_countsPerRev / 2);
    _countsToDegrees    = 360.0f / static_cast<float>(_countsPerRev);
    _countsToNormalized = 1.0f / static_cast<float>(_countsPerRev - 1);

    // Keep the embedded TMAG driver in step with the encoder's address choice
    // so a sketch only has to set one field.
    _cfg.tmag.i2cAddress = _cfg.i2cAddress;
    _tmag = TMAG5273(_cfg.tmag);
}

bool MagEncoder::begin(TwoWire &wire)
{
    _wire = &wire;

    if (_cfg.sensor == Sensor::TMAG5273)
    {
        _connected = _tmag.begin(wire);
    }
    else
    {
        _wire->begin();
        delay(50);
        _connected = checkConnection();
    }

    if (!_connected)
        return false;

    // Seed the baselines from the current reading so the first update() does
    // not report a jump from zero.
    _rawAngle       = readAngle();
    _lastRawAngle   = _rawAngle;
    _lastPosition   = _rawAngle;
    _lastSpeedAngle = _rawAngle;

    _lastReadTime  = millis();
    _lastSpeedTime = millis();
    _angularSpeed  = 0.0f;
    _pendingTicks  = 0;
    return true;
}

int16_t MagEncoder::unwrapAngleDelta(uint16_t current, uint16_t previous) const
{
    int16_t delta = static_cast<int16_t>(current) - static_cast<int16_t>(previous);

    // Correct the wrap-around at the top of the count range so multi-turn
    // motion is reported as a signed delta of at most half a revolution.
    if (delta > _wrapThreshold)
        delta -= static_cast<int16_t>(_countsPerRev);
    else if (delta < -_wrapThreshold)
        delta += static_cast<int16_t>(_countsPerRev);

    return delta;
}

uint16_t MagEncoder::readAngle()
{
    if (_cfg.sensor == Sensor::TMAG5273)
    {
        _tmag.update();
        return _tmag.getRawAngle();
    }

    return readAS5600Register16(REG_ANGLE);
}

void MagEncoder::update()
{
    if (!_connected)
        return;

    const unsigned long currentTime = millis();
    if (currentTime - _lastReadTime < _cfg.readIntervalMs)
        return;

    _lastReadTime = currentTime;
    _lastRawAngle = _rawAngle;
    _rawAngle     = readAngle();

    updateCumulativePosition();
    updateAngularSpeed(currentTime);
}

void MagEncoder::updateCumulativePosition()
{
    const int16_t delta = unwrapAngleDelta(_rawAngle, _lastPosition);
    _cumulativePosition += delta;
    _pendingTicks += delta;
    _lastPosition = _rawAngle;
}

void MagEncoder::updateAngularSpeed(unsigned long currentTime)
{
    if (_lastSpeedTime == 0)
    {
        _lastSpeedTime = currentTime;
        _angularSpeed = 0.0f;
        return;
    }

    const unsigned long deltaTime = currentTime - _lastSpeedTime;
    if (deltaTime < MIN_SPEED_DT_MS)
        return; // Too small a dt for a stable derivative.

    // Measure the angle travelled since the last *speed sample*, not since the
    // last sensor read. Reads are throttled to readIntervalMs (5 ms by
    // default) while this runs no more often than MIN_SPEED_DT_MS (8 ms), so
    // pairing one read's worth of movement with the longer speed window used
    // to under-report speed by roughly half: a true 90 deg/s turn measured 45.
    // Everything downstream is calibrated in deg/s -- minVelDps, maxVelDps and
    // the velocity zones -- so that halving pushed the whole curve out by 2x
    // and left normal turning pinned at minScale.
    const int16_t angleDelta = unwrapAngleDelta(_rawAngle, _lastSpeedAngle);
    _lastSpeedAngle = _rawAngle;

    // Instantaneous speed in degrees/second.
    const float instantSpeed = (angleDelta * _countsToDegrees) / (deltaTime / 1000.0f);

    // Adaptive low-pass filter: less smoothing for faster movements so
    // high-speed turns stay responsive.
    const float speedMagnitude = fabsf(instantSpeed);
    float alpha;
    if (speedMagnitude < SLOW_DPS)
        alpha = SMOOTH_ALPHA_SLOW;
    else if (speedMagnitude < MEDIUM_DPS)
        alpha = SMOOTH_ALPHA_MEDIUM;
    else
        alpha = SMOOTH_ALPHA_FAST;

    _angularSpeed = (alpha * instantSpeed) + ((1.0f - alpha) * _angularSpeed);

    // Noise gate for very low speeds (suppresses sensor jitter when still).
    if (fabsf(_angularSpeed) < NOISE_GATE_DPS)
        _angularSpeed *= 0.5f;

    _lastSpeedTime = currentTime;
}

float MagEncoder::calculateVelocityScale(float absSpeedDps) const
{
    if (absSpeedDps <= _cfg.minVelDps)
        return _cfg.minScale;

    const float normedSpeed = normalizeSpeed(absSpeedDps);
    const float curvedSpeed = applyVelCurve(normedSpeed);
    const float smoothedSpeed = smoothVel(curvedSpeed);

    return _cfg.minScale + (smoothedSpeed * (_cfg.maxScale - _cfg.minScale));
}

float MagEncoder::normalizeSpeed(float absSpeedDps) const
{
    if (absSpeedDps <= _cfg.minVelDps)
        return 0.0f;
    const float normalized =
        (absSpeedDps - _cfg.minVelDps) / (_cfg.maxVelDps - _cfg.minVelDps);
    return std::min(normalized, 1.0f);
}

float MagEncoder::applyVelCurve(float normedSpeed) const
{
    float curvedSpeed;

    if (normedSpeed <= 0.35f)
    {
        // Low speed range: quadratic curve for finer control.
        const float lowSpeedNormed = normedSpeed / 0.35f;
        curvedSpeed = (lowSpeedNormed * lowSpeedNormed) * 0.525f;
    }
    else if (normedSpeed >= 0.75f)
    {
        // High speed range: enhanced responsiveness.
        const float highSpeedBoost = (normedSpeed - 0.65f) / 0.35f;
        curvedSpeed = 0.3f + (highSpeedBoost * 0.7f) + (highSpeedBoost * highSpeedBoost * 0.5f);
    }
    else
    {
        // Mid range: exponential curve shaped by curveExponent.
        const float midNormalized = (normedSpeed - 0.2f) / 0.45f;
        curvedSpeed = 0.3f + (powf(midNormalized, _cfg.curveExponent) * 0.4f);
    }

    return std::min(curvedSpeed, 1.0f);
}

float MagEncoder::smoothVel(float curvedSpeed) const
{
    const float inv = 1.0f - _cfg.velocitySmoothing;
    _lastCurvedSpeed = (_cfg.velocitySmoothing * curvedSpeed) + (inv * _lastCurvedSpeed);
    return _lastCurvedSpeed;
}

float MagEncoder::getParameterIncrement(float minVal, float maxVal, uint8_t maxRotations) const
{
    const float totalRange = maxVal - minVal;
    if (totalRange <= 0.0f)
        return 0.0f;

    // Base increment per encoder count across the requested number of turns.
    const float baseIncrement =
        totalRange / (static_cast<float>(_countsPerRev) * maxRotations);

    // Velocity-sensitive scaling: faster turns cover more range per count.
    const float velocityScale = calculateVelocityScale(fabsf(_angularSpeed));

    const int16_t angleDelta = unwrapAngleDelta(_rawAngle, _lastRawAngle);
    return angleDelta * baseIncrement * velocityScale;
}

float MagEncoder::takeParameterIncrement(float minVal, float maxVal, uint8_t maxRotations)
{
    const int32_t ticks = _pendingTicks;
    _pendingTicks = 0;

    const float totalRange = maxVal - minVal;
    if (ticks == 0 || totalRange <= 0.0f || maxRotations == 0)
        return 0.0f;

    const float baseIncrement =
        totalRange / (static_cast<float>(_countsPerRev) * maxRotations);
    const float velocityScale = calculateVelocityScale(fabsf(_angularSpeed));

    return static_cast<float>(ticks) * baseIncrement * velocityScale;
}

int32_t MagEncoder::pendingTicks() const
{
    return _pendingTicks;
}

void MagEncoder::clearPendingTicks()
{
    _pendingTicks = 0;
}

float MagEncoder::getVelocityScale() const
{
    return calculateVelocityScale(fabsf(_angularSpeed));
}

float MagEncoder::mapPositionToRange(float minVal, float maxVal, uint8_t maxRotations) const
{
    return minVal + getPositionPercentage(maxRotations) * (maxVal - minVal);
}

uint16_t MagEncoder::getRawAngle() const
{
    return _rawAngle;
}

float MagEncoder::getNormalizedAngle() const
{
    return static_cast<float>(_rawAngle) * _countsToNormalized;
}

float MagEncoder::getAngleDegrees() const
{
    return static_cast<float>(_rawAngle) * _countsToDegrees;
}

int32_t MagEncoder::getCumulativePosition() const
{
    return _cumulativePosition;
}

float MagEncoder::getAngularSpeed() const
{
    return _angularSpeed;
}

float MagEncoder::getPositionPercentage(uint8_t maxRotations) const
{
    const float percentage = static_cast<float>(_cumulativePosition) /
                             (static_cast<float>(_countsPerRev) * maxRotations);
    return std::max(0.0f, std::min(percentage, 1.0f));
}

bool MagEncoder::isConnected() const
{
    return _connected;
}

MagEncoder::Sensor MagEncoder::getSensor() const
{
    return _cfg.sensor;
}

const char *MagEncoder::getSensorName() const
{
    return (_cfg.sensor == Sensor::TMAG5273) ? "TMAG5273" : "AS5600";
}

uint8_t MagEncoder::getI2CAddress() const
{
    return _cfg.i2cAddress;
}

uint16_t MagEncoder::getCountsPerRevolution() const
{
    return _countsPerRev;
}

TMAG5273 &MagEncoder::tmag()
{
    return _tmag;
}

const TMAG5273 &MagEncoder::tmag() const
{
    return _tmag;
}

MagEncoder::VelocityZone MagEncoder::getVelocityZone() const
{
    const float scale = calculateVelocityScale(fabsf(_angularSpeed));
    // scale ranges [_cfg.minScale, _cfg.maxScale]; pick thresholds relative
    // to that range so the zone meaning tracks the configured tuning.
    const float span = _cfg.maxScale - _cfg.minScale;
    if (scale <= _cfg.minScale + span * 0.25f)
        return VelocityZone::Idle;
    if (scale <= _cfg.minScale + span * 0.5f)
        return VelocityZone::Low;
    if (scale <= _cfg.minScale + span * 0.75f)
        return VelocityZone::Mid;
    return VelocityZone::High;
}

void MagEncoder::resetCumulativePosition(int32_t position)
{
    _cumulativePosition = position;
    _pendingTicks = 0;
    _lastPosition = _rawAngle;
    _lastSpeedAngle = _rawAngle;
}

uint16_t MagEncoder::readAS5600Register16(uint8_t reg) const
{
    _wire->beginTransmission(_cfg.i2cAddress);
    _wire->write(reg);
    if (_wire->endTransmission() != 0)
        return 0;

    _wire->requestFrom(static_cast<int>(_cfg.i2cAddress), 2);
    if (_wire->available() < 2)
        return 0;

    uint16_t result = _wire->read() << 8;
    result |= _wire->read();
    return result & 0x0FFF; // 12-bit mask
}

bool MagEncoder::checkConnection()
{
    _wire->beginTransmission(_cfg.i2cAddress);
    return (_wire->endTransmission() == 0);
}
