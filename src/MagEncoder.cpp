#include "MagEncoder.h"

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
}

MagEncoder::MagEncoder()
    : MagEncoder(Config())
{
}

MagEncoder::MagEncoder(const Config &config)
    : _cfg(config),
      _connected(false),
      _rawAngle(0),
      _lastRawAngle(0),
      _cumulativePosition(0),
      _lastPosition(0),
      _angularSpeed(0.0f),
      _lastReadTime(0),
      _lastSpeedTime(0),
      _lastCurvedSpeed(0.0f)
{
}

bool MagEncoder::begin()
{
    Wire.begin();
    delay(50);

    _connected = checkConnection();
    if (!_connected)
        return false;

    update();
    _lastRawAngle = _rawAngle;
    _lastPosition = _rawAngle;
    _lastSpeedTime = millis();
    return true;
}

int16_t MagEncoder::unwrapAngleDelta(uint16_t current, uint16_t previous)
{
    int16_t delta = static_cast<int16_t>(current) - static_cast<int16_t>(previous);

    // Correct 12-bit wrap-around so multi-turn motion is reported as a signed
    // delta in [-2048, 2047].
    if (delta > WRAP_THRESHOLD)
        delta -= TICKS_PER_REV;
    else if (delta < -WRAP_THRESHOLD)
        delta += TICKS_PER_REV;

    return delta;
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
    _rawAngle = readRegister16(REG_ANGLE);

    updateCumulativePosition();
    updateAngularSpeed(currentTime);
}

void MagEncoder::updateCumulativePosition()
{
    const int16_t delta = unwrapAngleDelta(_rawAngle, _lastPosition);
    _cumulativePosition += delta;
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

    const int16_t angleDelta = unwrapAngleDelta(_rawAngle, _lastRawAngle);

    // Instantaneous speed in degrees/second.
    const float instantSpeed = (angleDelta * RAW_TO_DEGREES) / (deltaTime / 1000.0f);

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

    // Base increment per encoder tick across the requested number of turns.
    const float baseIncrement = totalRange / (static_cast<float>(TICKS_PER_REV) * maxRotations);

    // Velocity-sensitive scaling: faster turns cover more range per tick.
    const float velocityScale = calculateVelocityScale(fabsf(_angularSpeed));

    const int16_t angleDelta = unwrapAngleDelta(_rawAngle, _lastRawAngle);
    return angleDelta * baseIncrement * velocityScale;
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
    return static_cast<float>(_rawAngle) * RAW_TO_NORMALIZED;
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
    const float percentage =
        static_cast<float>(_cumulativePosition) / (static_cast<float>(TICKS_PER_REV) * maxRotations);
    return std::max(0.0f, std::min(percentage, 1.0f));
}

bool MagEncoder::isConnected() const
{
    return _connected;
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
    _lastPosition = static_cast<int16_t>(_rawAngle);
}

uint16_t MagEncoder::readRegister16(uint8_t reg) const
{
    Wire.beginTransmission(_cfg.i2cAddress);
    Wire.write(reg);
    if (Wire.endTransmission() != 0)
        return 0;

    Wire.requestFrom(static_cast<int>(_cfg.i2cAddress), 2);
    if (Wire.available() < 2)
        return 0;

    uint16_t result = Wire.read() << 8;
    result |= Wire.read();
    return result & 0x0FFF; // 12-bit mask
}

bool MagEncoder::checkConnection()
{
    Wire.beginTransmission(_cfg.i2cAddress);
    return (Wire.endTransmission() == 0);
}
