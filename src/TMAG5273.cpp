#include "TMAG5273.h"

#include <cmath>

namespace
{
    // Bit fields, expressed as (mask, shift) pairs applied by updateBits().

    // DEVICE_CONFIG_1
    constexpr uint8_t MASK_MAG_TEMPCO = 0x60; // bits 6-5
    constexpr uint8_t SHIFT_MAG_TEMPCO = 5;
    constexpr uint8_t MASK_CONV_AVG   = 0x1C; // bits 4-2
    constexpr uint8_t SHIFT_CONV_AVG  = 2;

    // DEVICE_CONFIG_2
    constexpr uint8_t MASK_LP_LN            = 0x10; // bit 4
    constexpr uint8_t MASK_I2C_GLITCH_FILT  = 0x08; // bit 3
    constexpr uint8_t MASK_TRIGGER_MODE     = 0x04; // bit 2
    constexpr uint8_t MASK_OPERATING_MODE   = 0x03; // bits 1-0

    // SENSOR_CONFIG_1
    constexpr uint8_t MASK_MAG_CH_EN  = 0xF0; // bits 7-4
    constexpr uint8_t SHIFT_MAG_CH_EN = 4;
    constexpr uint8_t MASK_SLEEPTIME  = 0x0F; // bits 3-0

    // SENSOR_CONFIG_2
    constexpr uint8_t MASK_ANGLE_EN   = 0x0C; // bits 3-2
    constexpr uint8_t SHIFT_ANGLE_EN  = 2;
    constexpr uint8_t MASK_X_Y_RANGE  = 0x02; // bit 1
    constexpr uint8_t MASK_Z_RANGE    = 0x01; // bit 0

    // T_CONFIG
    constexpr uint8_t MASK_T_CH_EN = 0x01; // bit 0

    // INT_CONFIG_1
    constexpr uint8_t MASK_RSLT_INT   = 0x80; // bit 7
    constexpr uint8_t MASK_THRSLD_INT = 0x40; // bit 6
    constexpr uint8_t MASK_INT_MODE   = 0x1C; // bits 4-2
    constexpr uint8_t SHIFT_INT_MODE  = 2;

    // CONV_STATUS
    constexpr uint8_t MASK_SET_COUNT     = 0xE0; // bits 7-5
    constexpr uint8_t SHIFT_SET_COUNT    = 5;
    constexpr uint8_t MASK_POR           = 0x10; // bit 4
    constexpr uint8_t MASK_DIAG_STATUS   = 0x02; // bit 1
    constexpr uint8_t MASK_RESULT_STATUS = 0x01; // bit 0

    // DEVICE_STATUS
    constexpr uint8_t MASK_INTB_RB     = 0x10; // bit 4
    constexpr uint8_t MASK_OSC_ER      = 0x08; // bit 3
    constexpr uint8_t MASK_INT_ER      = 0x04; // bit 2
    constexpr uint8_t MASK_OTP_CRC_ER  = 0x02; // bit 1
    constexpr uint8_t MASK_VCC_UV_ER   = 0x01; // bit 0

    // DEVICE_ID version bits: 1h = +/-40mT and +/-80mT, 2h = +/-133mT and +/-266mT.
    constexpr uint8_t MASK_VER = 0x03;

    // Base full-scale range in mT for the two orderable range families.
    constexpr float RANGE_BASE_LOW_MT  = 40.0f;  // TMAG5273x1
    constexpr float RANGE_BASE_HIGH_MT = 133.0f; // TMAG5273x2

    // Result block: T_MSB (0x10) through DEVICE_STATUS (0x1C).
    constexpr uint8_t RESULT_BLOCK_START = TMAG5273::REG_T_MSB_RESULT;
    constexpr uint8_t RESULT_BLOCK_LEN   = 13;

    // Largest burst this driver asks Wire for. Keeps the driver inside the
    // 32-byte buffer of the smallest Arduino cores.
    constexpr uint8_t MAX_BURST = 16;

    // Time to let the device settle after a configuration write before its
    // first conversion is trustworthy.
    constexpr unsigned long CONFIG_SETTLE_MS = 5;

    constexpr float RAD_TO_DEG_F = 57.29577951308232f;
}

TMAG5273::TMAG5273()
    : TMAG5273(Config())
{
}

TMAG5273::TMAG5273(const Config &config)
    : _cfg(config),
      _wire(&Wire),
      _connected(false),
      _version(0),
      _manufacturerId(0),
      _rawX(0),
      _rawY(0),
      _rawZ(0),
      _rawTemp(0),
      _rawAngle(0),
      _magnitude(0)
{
}

bool TMAG5273::begin(TwoWire &wire)
{
    _wire = &wire;
    _wire->begin();

    _wire->beginTransmission(_cfg.i2cAddress);
    if (_wire->endTransmission() != 0)
    {
        _connected = false;
        return false;
    }

    // A device that answers is not necessarily a TMAG5273; check the ID.
    uint8_t id[3] = {0, 0, 0};
    if (!readRegisters(REG_DEVICE_ID, id, sizeof(id)))
    {
        _connected = false;
        return false;
    }

    _version        = id[0] & MASK_VER;
    _manufacturerId = static_cast<uint16_t>(id[2]) << 8 | id[1];

    if (_manufacturerId != MANUFACTURER_ID)
    {
        _connected = false;
        return false;
    }

    _connected = true;

    if (!applyConfig())
    {
        _connected = false;
        return false;
    }

    delay(CONFIG_SETTLE_MS);
    update();
    return true;
}

bool TMAG5273::applyConfig()
{
    if (!_connected)
        return false;

    const uint8_t deviceConfig1 =
        ((static_cast<uint8_t>(_cfg.tempco) << SHIFT_MAG_TEMPCO) & MASK_MAG_TEMPCO) |
        ((static_cast<uint8_t>(_cfg.averaging) << SHIFT_CONV_AVG) & MASK_CONV_AVG);

    // TRIGGER_MODE stays 0 (conversion starts on I2C command) so that
    // triggerConversion() works without an INT pin wired up.
    const uint8_t deviceConfig2 =
        (_cfg.lowNoiseMode ? MASK_LP_LN : 0) |
        (_cfg.glitchFilter ? 0 : MASK_I2C_GLITCH_FILT) |
        (static_cast<uint8_t>(_cfg.operatingMode) & MASK_OPERATING_MODE);

    const uint8_t sensorConfig1 =
        ((static_cast<uint8_t>(_cfg.channels) << SHIFT_MAG_CH_EN) & MASK_MAG_CH_EN) |
        (static_cast<uint8_t>(_cfg.sleepTime) & MASK_SLEEPTIME);

    const uint8_t sensorConfig2 =
        ((static_cast<uint8_t>(_cfg.anglePair) << SHIFT_ANGLE_EN) & MASK_ANGLE_EN) |
        (_cfg.rangeXY == Range::High ? MASK_X_Y_RANGE : 0) |
        (_cfg.rangeZ  == Range::High ? MASK_Z_RANGE   : 0);

    bool ok = true;
    ok &= writeRegister(REG_DEVICE_CONFIG_1, deviceConfig1);
    ok &= writeRegister(REG_SENSOR_CONFIG_1, sensorConfig1);
    ok &= writeRegister(REG_SENSOR_CONFIG_2, sensorConfig2);
    ok &= updateBits(REG_T_CONFIG, MASK_T_CH_EN, _cfg.enableTemp ? MASK_T_CH_EN : 0);

    // DEVICE_CONFIG_2 carries OPERATING_MODE, so write it last: everything the
    // conversion depends on is already in place when the device starts running.
    ok &= writeRegister(REG_DEVICE_CONFIG_2, deviceConfig2);

    return ok;
}

bool TMAG5273::update()
{
    if (!_connected)
        return false;

    uint8_t block[RESULT_BLOCK_LEN];
    if (!readRegisters(RESULT_BLOCK_START, block, RESULT_BLOCK_LEN))
        return false;

    _rawTemp  = static_cast<uint16_t>(block[0]) << 8 | block[1];
    _rawX     = static_cast<int16_t>(static_cast<uint16_t>(block[2]) << 8 | block[3]);
    _rawY     = static_cast<int16_t>(static_cast<uint16_t>(block[4]) << 8 | block[5]);
    _rawZ     = static_cast<int16_t>(static_cast<uint16_t>(block[6]) << 8 | block[7]);

    const uint8_t conv = block[8];
    _convStatus.raw          = conv;
    _convStatus.setCount     = (conv & MASK_SET_COUNT) >> SHIFT_SET_COUNT;
    _convStatus.powerOnReset = (conv & MASK_POR) != 0;
    _convStatus.diagFail     = (conv & MASK_DIAG_STATUS) != 0;
    _convStatus.dataReady    = (conv & MASK_RESULT_STATUS) != 0;

    _rawAngle  = (static_cast<uint16_t>(block[9]) << 8 | block[10]) & ANGLE_MASK;
    _magnitude = block[11];

    const uint8_t status = block[12];
    _deviceStatus.raw          = status;
    _deviceStatus.intPinHigh   = (status & MASK_INTB_RB) != 0;
    _deviceStatus.oscError     = (status & MASK_OSC_ER) != 0;
    _deviceStatus.intError     = (status & MASK_INT_ER) != 0;
    _deviceStatus.otpCrcError  = (status & MASK_OTP_CRC_ER) != 0;
    _deviceStatus.vccUnderVolt = (status & MASK_VCC_UV_ER) != 0;

    return true;
}

// ----------------------------------------------------------------------
// Conversions
// ----------------------------------------------------------------------

float TMAG5273::rangeToMilliTesla(Range range) const
{
    // DEVICE_ID VER == 2 marks the x2 parts (+/-133mT and +/-266mT); anything
    // else is treated as the x1 family (+/-40mT and +/-80mT).
    const float base = (_version == 2) ? RANGE_BASE_HIGH_MT : RANGE_BASE_LOW_MT;
    return (range == Range::High) ? (base * 2.0f) : base;
}

float TMAG5273::getRangeXY() const
{
    return rangeToMilliTesla(_cfg.rangeXY);
}

float TMAG5273::getRangeZ() const
{
    return rangeToMilliTesla(_cfg.rangeZ);
}

const char *TMAG5273::getVersionName() const
{
    return (_version == 2) ? "x2" : "x1";
}

float TMAG5273::getX() const
{
    return (static_cast<float>(_rawX) / MAG_FULL_SCALE) * getRangeXY();
}

float TMAG5273::getY() const
{
    return (static_cast<float>(_rawY) / MAG_FULL_SCALE) * getRangeXY();
}

float TMAG5273::getZ() const
{
    return (static_cast<float>(_rawZ) / MAG_FULL_SCALE) * getRangeZ();
}

float TMAG5273::getFieldMagnitude() const
{
    const float x = getX();
    const float y = getY();
    const float z = getZ();
    return sqrtf(x * x + y * y + z * z);
}

float TMAG5273::getElevation() const
{
    const float x = getX();
    const float y = getY();
    const float z = getZ();
    const float planar = sqrtf(x * x + y * y);
    if (planar == 0.0f && z == 0.0f)
        return 0.0f;
    return atan2f(z, planar) * RAD_TO_DEG_F;
}

float TMAG5273::getAzimuth() const
{
    const float x = getX();
    const float y = getY();
    if (x == 0.0f && y == 0.0f)
        return 0.0f;
    return atan2f(y, x) * RAD_TO_DEG_F;
}

float TMAG5273::getTemperature() const
{
    return TEMP_T0_C + ((static_cast<float>(_rawTemp) - TEMP_ADC_T0) / TEMP_ADC_RES);
}

float TMAG5273::getAngle() const
{
    return static_cast<float>(_rawAngle) * ANGLE_LSB_DEG;
}

// ----------------------------------------------------------------------
// Runtime configuration
// ----------------------------------------------------------------------

bool TMAG5273::setMagChannels(MagChannels channels)
{
    _cfg.channels = channels;
    return updateBits(REG_SENSOR_CONFIG_1, MASK_MAG_CH_EN,
                      static_cast<uint8_t>(channels) << SHIFT_MAG_CH_EN);
}

bool TMAG5273::setTemperatureChannel(bool enabled)
{
    _cfg.enableTemp = enabled;
    return updateBits(REG_T_CONFIG, MASK_T_CH_EN, enabled ? MASK_T_CH_EN : 0);
}

bool TMAG5273::setAnglePair(AnglePair pair)
{
    _cfg.anglePair = pair;
    return updateBits(REG_SENSOR_CONFIG_2, MASK_ANGLE_EN,
                      static_cast<uint8_t>(pair) << SHIFT_ANGLE_EN);
}

bool TMAG5273::setAveraging(ConvAvg averaging)
{
    _cfg.averaging = averaging;
    return updateBits(REG_DEVICE_CONFIG_1, MASK_CONV_AVG,
                      static_cast<uint8_t>(averaging) << SHIFT_CONV_AVG);
}

bool TMAG5273::setTempCo(TempCo tempco)
{
    _cfg.tempco = tempco;
    return updateBits(REG_DEVICE_CONFIG_1, MASK_MAG_TEMPCO,
                      static_cast<uint8_t>(tempco) << SHIFT_MAG_TEMPCO);
}

bool TMAG5273::setOperatingMode(OperatingMode mode)
{
    _cfg.operatingMode = mode;
    return updateBits(REG_DEVICE_CONFIG_2, MASK_OPERATING_MODE,
                      static_cast<uint8_t>(mode));
}

bool TMAG5273::setSleepTime(SleepTime sleepTime)
{
    _cfg.sleepTime = sleepTime;
    return updateBits(REG_SENSOR_CONFIG_1, MASK_SLEEPTIME,
                      static_cast<uint8_t>(sleepTime));
}

bool TMAG5273::setRanges(Range rangeXY, Range rangeZ)
{
    _cfg.rangeXY = rangeXY;
    _cfg.rangeZ  = rangeZ;
    const uint8_t value = (rangeXY == Range::High ? MASK_X_Y_RANGE : 0) |
                          (rangeZ  == Range::High ? MASK_Z_RANGE   : 0);
    return updateBits(REG_SENSOR_CONFIG_2, MASK_X_Y_RANGE | MASK_Z_RANGE, value);
}

bool TMAG5273::setLowNoiseMode(bool lowNoise)
{
    _cfg.lowNoiseMode = lowNoise;
    return updateBits(REG_DEVICE_CONFIG_2, MASK_LP_LN, lowNoise ? MASK_LP_LN : 0);
}

bool TMAG5273::setMagneticGain(float gain)
{
    // The register holds a fraction of full scale as value/256.
    if (gain < 0.0f)
        gain = 0.0f;
    if (gain > 1.0f)
        gain = 1.0f;

    const uint8_t code = static_cast<uint8_t>(lroundf(gain * 256.0f) & 0xFF);
    return writeRegister(REG_MAG_GAIN_CONFIG, code);
}

bool TMAG5273::setMagneticOffsets(int8_t firstAxis, int8_t secondAxis)
{
    bool ok = writeRegister(REG_MAG_OFFSET_CONFIG_1, static_cast<uint8_t>(firstAxis));
    ok &= writeRegister(REG_MAG_OFFSET_CONFIG_2, static_cast<uint8_t>(secondAxis));
    return ok;
}

bool TMAG5273::setMagneticThresholds(int8_t x, int8_t y, int8_t z)
{
    bool ok = writeRegister(REG_X_THR_CONFIG, static_cast<uint8_t>(x));
    ok &= writeRegister(REG_Y_THR_CONFIG, static_cast<uint8_t>(y));
    ok &= writeRegister(REG_Z_THR_CONFIG, static_cast<uint8_t>(z));
    return ok;
}

bool TMAG5273::setInterrupt(IntMode mode, bool onConversionComplete, bool onThreshold)
{
    const uint8_t mask = MASK_RSLT_INT | MASK_THRSLD_INT | MASK_INT_MODE;
    const uint8_t value = (onConversionComplete ? MASK_RSLT_INT : 0) |
                          (onThreshold ? MASK_THRSLD_INT : 0) |
                          ((static_cast<uint8_t>(mode) << SHIFT_INT_MODE) & MASK_INT_MODE);
    return updateBits(REG_INT_CONFIG_1, mask, value);
}

bool TMAG5273::triggerConversion()
{
    // With TRIGGER_MODE = 0 the device starts a conversion on an I2C command,
    // so re-writing DEVICE_CONFIG_2 unchanged is the trigger.
    const uint8_t current = readRegister(REG_DEVICE_CONFIG_2);
    return writeRegister(REG_DEVICE_CONFIG_2, current);
}

bool TMAG5273::clearStatusFlags()
{
    // Both registers use write-1-to-clear semantics on their latched bits.
    bool ok = writeRegister(REG_CONV_STATUS, MASK_POR);
    ok &= writeRegister(REG_DEVICE_STATUS,
                        MASK_OSC_ER | MASK_INT_ER | MASK_OTP_CRC_ER | MASK_VCC_UV_ER);
    return ok;
}

// ----------------------------------------------------------------------
// Raw register access
// ----------------------------------------------------------------------

uint8_t TMAG5273::readRegister(uint8_t reg) const
{
    uint8_t value = 0;
    if (!readRegisters(reg, &value, 1))
        return 0;
    return value;
}

bool TMAG5273::writeRegister(uint8_t reg, uint8_t value)
{
    _wire->beginTransmission(_cfg.i2cAddress);
    _wire->write(reg);
    _wire->write(value);
    return _wire->endTransmission() == 0;
}

bool TMAG5273::readRegisters(uint8_t reg, uint8_t *dest, uint8_t count) const
{
    if (dest == nullptr || count == 0)
        return false;

    uint8_t remaining = count;
    uint8_t offset    = 0;

    // Split long reads so a core with a 32-byte Wire buffer still works.
    while (remaining > 0)
    {
        const uint8_t chunk = (remaining > MAX_BURST) ? MAX_BURST : remaining;

        _wire->beginTransmission(_cfg.i2cAddress);
        _wire->write(static_cast<uint8_t>(reg + offset));
        if (_wire->endTransmission(false) != 0)
            return false;

        if (_wire->requestFrom(static_cast<int>(_cfg.i2cAddress), static_cast<int>(chunk)) !=
            static_cast<int>(chunk))
            return false;

        for (uint8_t i = 0; i < chunk; ++i)
        {
            if (!_wire->available())
                return false;
            dest[offset + i] = static_cast<uint8_t>(_wire->read());
        }

        offset += chunk;
        remaining = static_cast<uint8_t>(remaining - chunk);
    }

    return true;
}

bool TMAG5273::readRegisterMap(uint8_t *dest) const
{
    return readRegisters(0x00, dest, REGISTER_COUNT);
}

bool TMAG5273::updateBits(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current = 0;
    if (!readRegisters(reg, &current, 1))
        return false;

    const uint8_t updated = static_cast<uint8_t>((current & ~mask) | (value & mask));
    if (updated == current)
        return true;

    return writeRegister(reg, updated);
}

const char *TMAG5273::registerName(uint8_t reg)
{
    switch (reg)
    {
        case REG_DEVICE_CONFIG_1:     return "DEVICE_CONFIG_1";
        case REG_DEVICE_CONFIG_2:     return "DEVICE_CONFIG_2";
        case REG_SENSOR_CONFIG_1:     return "SENSOR_CONFIG_1";
        case REG_SENSOR_CONFIG_2:     return "SENSOR_CONFIG_2";
        case REG_X_THR_CONFIG:        return "X_THR_CONFIG";
        case REG_Y_THR_CONFIG:        return "Y_THR_CONFIG";
        case REG_Z_THR_CONFIG:        return "Z_THR_CONFIG";
        case REG_T_CONFIG:            return "T_CONFIG";
        case REG_INT_CONFIG_1:        return "INT_CONFIG_1";
        case REG_MAG_GAIN_CONFIG:     return "MAG_GAIN_CONFIG";
        case REG_MAG_OFFSET_CONFIG_1: return "MAG_OFFSET_1";
        case REG_MAG_OFFSET_CONFIG_2: return "MAG_OFFSET_2";
        case REG_I2C_ADDRESS:         return "I2C_ADDRESS";
        case REG_DEVICE_ID:           return "DEVICE_ID";
        case REG_MANUFACTURER_ID_LSB: return "MFR_ID_LSB";
        case REG_MANUFACTURER_ID_MSB: return "MFR_ID_MSB";
        case REG_T_MSB_RESULT:        return "T_MSB";
        case REG_T_LSB_RESULT:        return "T_LSB";
        case REG_X_MSB_RESULT:        return "X_MSB";
        case REG_X_LSB_RESULT:        return "X_LSB";
        case REG_Y_MSB_RESULT:        return "Y_MSB";
        case REG_Y_LSB_RESULT:        return "Y_LSB";
        case REG_Z_MSB_RESULT:        return "Z_MSB";
        case REG_Z_LSB_RESULT:        return "Z_LSB";
        case REG_CONV_STATUS:         return "CONV_STATUS";
        case REG_ANGLE_RESULT_MSB:    return "ANGLE_MSB";
        case REG_ANGLE_RESULT_LSB:    return "ANGLE_LSB";
        case REG_MAGNITUDE_RESULT:    return "MAGNITUDE";
        case REG_DEVICE_STATUS:       return "DEVICE_STATUS";
        default:                      return "RESERVED";
    }
}
