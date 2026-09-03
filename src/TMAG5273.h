#ifndef TMAG5273_h
#define TMAG5273_h

#include <Arduino.h>
#include <Wire.h>

/**
 * TMAG5273
 * --------
 * Arduino driver for the Texas Instruments TMAG5273 low-power linear 3D
 * Hall-effect sensor with I2C interface (datasheet SLYS045).
 *
 * Where the AS5600 hands you a single angle, the TMAG5273 hands you the whole
 * field: three independent Hall channels (X, Y, Z), a die temperature sensor,
 * an on-chip CORDIC angle engine that works on any pair of axes, and a
 * resultant-vector magnitude. This class exposes all of it.
 *
 * What you get:
 *   - X/Y/Z magnetic flux density in millitesla, plus the signed 16-bit ADC
 *     codes behind them.
 *   - Die temperature in degrees Celsius.
 *   - CORDIC angle: 0..360 degrees with 1/16-degree resolution (5760 counts
 *     per revolution) computed from a user-selected axis pair.
 *   - Resultant vector magnitude (the sensor's own 8-bit value, and a float
 *     magnitude computed from the three axes).
 *   - Conversion and device diagnostic status flags, device version and
 *     manufacturer ID.
 *   - Raw access to the whole register map for register-level tools.
 *
 * Configuration lives in a single Config struct applied by begin(), and every
 * field can also be changed at runtime through the individual setters.
 *
 * The device ships in one of four factory-programmed I2C addresses depending on
 * the orderable part suffix (see ADDRESS_A..ADDRESS_D). The A/B/C/D suffix
 * selects the address; the trailing 1/2 selects the magnetic range family
 * (x1 = +/-40mT and +/-80mT, x2 = +/-133mT and +/-266mT), which begin() reads
 * back from DEVICE_ID so field conversions are automatically correct for the
 * part you actually fitted.
 */
class TMAG5273
{
public:
    // ------------------------------------------------------------------
    // Factory-programmed 7-bit I2C addresses (datasheet Table 6-3)
    // ------------------------------------------------------------------
    static constexpr uint8_t ADDRESS_A = 0x35; // TMAG5273A1 / A2
    static constexpr uint8_t ADDRESS_B = 0x22; // TMAG5273B1 / B2
    static constexpr uint8_t ADDRESS_C = 0x78; // TMAG5273C1 / C2
    static constexpr uint8_t ADDRESS_D = 0x44; // TMAG5273D1 / D2

    /** Expected MANUFACTURER_ID (MSB 0x54, LSB 0x49). */
    static constexpr uint16_t MANUFACTURER_ID = 0x5449;

    /** CORDIC angle counts per revolution: 360 degrees at 1/16-degree steps. */
    static constexpr uint16_t ANGLE_COUNTS_PER_REV = 5760;

    // ------------------------------------------------------------------
    // Register map (datasheet Table 8-1)
    // ------------------------------------------------------------------
    enum Register : uint8_t
    {
        REG_DEVICE_CONFIG_1     = 0x00,
        REG_DEVICE_CONFIG_2     = 0x01,
        REG_SENSOR_CONFIG_1     = 0x02,
        REG_SENSOR_CONFIG_2     = 0x03,
        REG_X_THR_CONFIG        = 0x04,
        REG_Y_THR_CONFIG        = 0x05,
        REG_Z_THR_CONFIG        = 0x06,
        REG_T_CONFIG            = 0x07,
        REG_INT_CONFIG_1        = 0x08,
        REG_MAG_GAIN_CONFIG     = 0x09,
        REG_MAG_OFFSET_CONFIG_1 = 0x0A,
        REG_MAG_OFFSET_CONFIG_2 = 0x0B,
        REG_I2C_ADDRESS         = 0x0C,
        REG_DEVICE_ID           = 0x0D,
        REG_MANUFACTURER_ID_LSB = 0x0E,
        REG_MANUFACTURER_ID_MSB = 0x0F,
        REG_T_MSB_RESULT        = 0x10,
        REG_T_LSB_RESULT        = 0x11,
        REG_X_MSB_RESULT        = 0x12,
        REG_X_LSB_RESULT        = 0x13,
        REG_Y_MSB_RESULT        = 0x14,
        REG_Y_LSB_RESULT        = 0x15,
        REG_Z_MSB_RESULT        = 0x16,
        REG_Z_LSB_RESULT        = 0x17,
        REG_CONV_STATUS         = 0x18,
        REG_ANGLE_RESULT_MSB    = 0x19,
        REG_ANGLE_RESULT_LSB    = 0x1A,
        REG_MAGNITUDE_RESULT    = 0x1B,
        REG_DEVICE_STATUS       = 0x1C
    };

    /** Number of readable registers, 0x00..0x1C inclusive. */
    static constexpr uint8_t REGISTER_COUNT = 0x1D;

    // ------------------------------------------------------------------
    // Configuration enums
    // ------------------------------------------------------------------

    /** Extra hardware averaging: more samples means less noise, lower rate. */
    enum class ConvAvg : uint8_t
    {
        X1  = 0, // 10.0 kSPS (3 axes) / 20 kSPS (1 axis)
        X2  = 1, //  5.7 kSPS / 13.3 kSPS
        X4  = 2, //  3.1 kSPS /  8.0 kSPS
        X8  = 3, //  1.6 kSPS /  4.4 kSPS
        X16 = 4, //  0.8 kSPS /  2.4 kSPS
        X32 = 5  //  0.4 kSPS /  1.2 kSPS
    };

    /** On-chip compensation for the magnet's own temperature coefficient. */
    enum class TempCo : uint8_t
    {
        None    = 0, // 0%/degC
        NdBFe   = 1, // 0.12%/degC
        Ceramic = 3  // 0.20%/degC
    };

    /** Power/measurement scheme. */
    enum class OperatingMode : uint8_t
    {
        Standby      = 0, // Convert on trigger only
        Sleep        = 1,
        Continuous   = 2, // Free-running conversions
        WakeAndSleep = 3  // Convert, then sleep for SleepTime
    };

    /** Which Hall channels are digitized. */
    enum class MagChannels : uint8_t
    {
        Off = 0x0,
        X   = 0x1,
        Y   = 0x2,
        XY  = 0x3,
        Z   = 0x4,
        ZX  = 0x5,
        YZ  = 0x6,
        XYZ = 0x7,
        XYX = 0x8,
        YXY = 0x9,
        YZY = 0xA,
        XZX = 0xB
    };

    /** Axis pair fed to the CORDIC angle engine. */
    enum class AnglePair : uint8_t
    {
        Off = 0, // No angle calculation, no gain/offset correction
        XY  = 1, // X first, Y second
        YZ  = 2, // Y first, Z second
        XZ  = 3  // X first, Z second
    };

    /** Idle time between conversions in wake-and-sleep mode. */
    enum class SleepTime : uint8_t
    {
        Ms1 = 0, Ms5, Ms10, Ms15, Ms20, Ms30, Ms50, Ms100,
        Ms500, Ms1000, Ms2000, Ms5000, Ms20000
    };

    /**
     * Per-axis full-scale selection. The millitesla value each option maps to
     * depends on the part's range family, which begin() reads from DEVICE_ID:
     *   Low  = +/-40mT  (x1 parts) or +/-133mT (x2 parts)
     *   High = +/-80mT  (x1 parts) or +/-266mT (x2 parts)
     */
    enum class Range : uint8_t
    {
        Low  = 0,
        High = 1
    };

    /** Interrupt source/behaviour on the INT pin. */
    enum class IntMode : uint8_t
    {
        None            = 0,
        Int             = 1,
        IntExceptBusy   = 2,
        Scl             = 3,
        SclExceptBusy   = 4
    };

    /**
     * Everything begin() writes into the device. The defaults give a
     * free-running three-axis measurement with the temperature channel and the
     * X/Y angle engine enabled, which is the configuration a rotary-knob or
     * "show me everything" sketch wants.
     */
    struct Config
    {
        uint8_t       i2cAddress    = ADDRESS_B;
        MagChannels   channels      = MagChannels::XYZ;
        bool          enableTemp    = true;
        AnglePair     anglePair     = AnglePair::XY;
        ConvAvg       averaging     = ConvAvg::X4;
        TempCo        tempco        = TempCo::None;
        OperatingMode operatingMode = OperatingMode::Continuous;
        SleepTime     sleepTime     = SleepTime::Ms10;
        Range         rangeXY       = Range::Low;
        Range         rangeZ        = Range::Low;
        bool          lowNoiseMode  = true;  // false = low active current mode
        bool          glitchFilter  = true;  // I2C glitch filter on
    };

    /** Decoded CONV_STATUS register (offset 0x18). */
    struct ConversionStatus
    {
        uint8_t setCount     = 0;     // Rolling count of conversion data sets
        bool    powerOnReset = false; // POR occurred since last clear
        bool    diagFail     = false; // Any internal diagnostic failed
        bool    dataReady    = false; // Conversion data buffer ready to read
        uint8_t raw          = 0;
    };

    /** Decoded DEVICE_STATUS register (offset 0x1C). */
    struct DeviceStatus
    {
        bool    intPinHigh   = false; // Level read back from the INT pin
        bool    oscError     = false;
        bool    intError     = false;
        bool    otpCrcError  = false;
        bool    vccUnderVolt = false;
        uint8_t raw          = 0;
    };

    // ------------------------------------------------------------------
    // Construction and setup
    // ------------------------------------------------------------------

    /** Construct with default configuration (address 0x22, XYZ + temp + angle). */
    TMAG5273();

    /** Construct with custom configuration. */
    explicit TMAG5273(const Config &config);

    /**
     * Start the bus, verify the manufacturer ID, learn the part's magnetic
     * range family from DEVICE_ID, and write the configuration.
     *
     * Returns true when a TMAG5273 answered at the configured address and
     * reported the expected manufacturer ID. On failure nothing is written and
     * isConnected() stays false.
     */
    bool begin(TwoWire &wire = Wire);

    /**
     * Re-apply the whole Config struct to the device. Called by begin(); call
     * it again yourself after editing config().
     */
    bool applyConfig();

    /** Mutable access to the configuration. Call applyConfig() after editing. */
    Config &config() { return _cfg; }
    const Config &config() const { return _cfg; }

    // ------------------------------------------------------------------
    // Measurement
    // ------------------------------------------------------------------

    /**
     * Read the result block (temperature, X, Y, Z, conversion status, angle,
     * magnitude and device status) in one burst and cache it. Every getter
     * below returns data from the most recent successful update().
     *
     * Returns true if the burst read completed.
     */
    bool update();

    // --- Magnetic field -----------------------------------------------

    /** Signed 16-bit ADC code for the X channel. */
    int16_t getRawX() const { return _rawX; }
    int16_t getRawY() const { return _rawY; }
    int16_t getRawZ() const { return _rawZ; }

    /** X magnetic flux density in millitesla. */
    float getX() const;
    float getY() const;
    float getZ() const;

    /** Resultant field strength sqrt(x^2 + y^2 + z^2) in millitesla. */
    float getFieldMagnitude() const;

    /** Angle of the field vector out of the XY plane, -90..+90 degrees. */
    float getElevation() const;

    /** Angle of the field vector within the XY plane, -180..+180 degrees. */
    float getAzimuth() const;

    // --- Temperature ---------------------------------------------------

    /** Raw 16-bit temperature ADC code. */
    uint16_t getRawTemperature() const { return _rawTemp; }

    /** Die temperature in degrees Celsius. */
    float getTemperature() const;

    // --- CORDIC angle and magnitude ------------------------------------

    /**
     * Raw CORDIC angle in 1/16-degree counts, 0..5759. This is the natural
     * tick space of the sensor and what MagEncoder uses for position tracking.
     */
    uint16_t getRawAngle() const { return _rawAngle; }

    /** CORDIC angle in degrees, 0.0 .. 359.9375. */
    float getAngle() const;

    /**
     * The sensor's own 8-bit resultant vector magnitude, in ADC-code units.
     * On an on-axis rotary setup this stays constant through a full turn, so
     * it doubles as a magnet-placement quality indicator.
     */
    uint8_t getMagnitude() const { return _magnitude; }

    // --- Status --------------------------------------------------------

    /** Decoded CONV_STATUS from the last update(). */
    const ConversionStatus &getConversionStatus() const { return _convStatus; }

    /** Decoded DEVICE_STATUS from the last update(). */
    const DeviceStatus &getDeviceStatus() const { return _deviceStatus; }

    /** True if begin() found the sensor and it reported the right ID. */
    bool isConnected() const { return _connected; }

    /** The 7-bit address the driver is talking to. */
    uint8_t getI2CAddress() const { return _cfg.i2cAddress; }

    /** DEVICE_ID version bits: 1 = +/-40mT and +/-80mT, 2 = +/-133mT and +/-266mT. */
    uint8_t getVersion() const { return _version; }

    /** MANUFACTURER_ID read at begin(); 0x5449 for a genuine part. */
    uint16_t getManufacturerId() const { return _manufacturerId; }

    /** Full-scale range of the X and Y channels in millitesla. */
    float getRangeXY() const;

    /** Full-scale range of the Z channel in millitesla. */
    float getRangeZ() const;

    /** Human-readable part suffix for the detected range family ("x1"/"x2"). */
    const char *getVersionName() const;

    // ------------------------------------------------------------------
    // Runtime configuration
    // ------------------------------------------------------------------

    bool setMagChannels(MagChannels channels);
    bool setTemperatureChannel(bool enabled);
    bool setAnglePair(AnglePair pair);
    bool setAveraging(ConvAvg averaging);
    bool setTempCo(TempCo tempco);
    bool setOperatingMode(OperatingMode mode);
    bool setSleepTime(SleepTime sleepTime);
    bool setRanges(Range rangeXY, Range rangeZ);
    bool setLowNoiseMode(bool lowNoise);

    /**
     * Magnetic gain trim applied to the axis selected by MAG_GAIN_CH, as a
     * fraction of full scale in [0.0, 1.0]. A value of 0 is interpreted by the
     * device as unity gain.
     */
    bool setMagneticGain(float gain);

    /** Per-axis offset trim codes for the angle pair, -128..127 each. */
    bool setMagneticOffsets(int8_t firstAxis, int8_t secondAxis);

    /** Magnetic threshold codes for the interrupt comparator, -128..127. */
    bool setMagneticThresholds(int8_t x, int8_t y, int8_t z);

    /** Configure the INT pin behaviour. */
    bool setInterrupt(IntMode mode, bool onConversionComplete, bool onThreshold);

    /** Trigger one conversion in standby mode. */
    bool triggerConversion();

    /**
     * Clear the latched POR flag in CONV_STATUS and every latched error bit in
     * DEVICE_STATUS by writing 1 back to them.
     */
    bool clearStatusFlags();

    // ------------------------------------------------------------------
    // Raw register access
    // ------------------------------------------------------------------

    /** Read a single register. Returns 0 on bus error. */
    uint8_t readRegister(uint8_t reg) const;

    /** Write a single register. Returns true on success. */
    bool writeRegister(uint8_t reg, uint8_t value);

    /** Read `count` consecutive registers starting at `reg` into `dest`. */
    bool readRegisters(uint8_t reg, uint8_t *dest, uint8_t count) const;

    /**
     * Snapshot the whole readable register map (0x00..0x1C) into `dest`, which
     * must have room for REGISTER_COUNT bytes. Useful for register-dump UIs.
     */
    bool readRegisterMap(uint8_t *dest) const;

    /** Short name for each register, indexed by offset. Never null. */
    static const char *registerName(uint8_t reg);

private:
    // Temperature transfer function constants (datasheet Section 5.6).
    // TADC_RES was revised from 60 to 58 LSB/degC in datasheet revision C.
    static constexpr float TEMP_T0_C     = 25.0f;    // TSENS_T0
    static constexpr float TEMP_ADC_T0   = 17508.0f; // TADC_T0
    static constexpr float TEMP_ADC_RES  = 58.0f;    // TADC_RES, LSB per degC

    // Signed 16-bit full scale: B = raw / 32768 * range_mT (Equation 10).
    static constexpr float MAG_FULL_SCALE = 32768.0f;

    // Angle result is 13 bits of degrees * 16 (Equation 14).
    static constexpr uint16_t ANGLE_MASK      = 0x1FFF;
    static constexpr float    ANGLE_LSB_DEG   = 1.0f / 16.0f;

    Config   _cfg;
    TwoWire *_wire;

    bool     _connected;
    uint8_t  _version;
    uint16_t _manufacturerId;

    int16_t  _rawX;
    int16_t  _rawY;
    int16_t  _rawZ;
    uint16_t _rawTemp;
    uint16_t _rawAngle;
    uint8_t  _magnitude;

    ConversionStatus _convStatus;
    DeviceStatus     _deviceStatus;

    // Read-modify-write a bit field within a register.
    bool updateBits(uint8_t reg, uint8_t mask, uint8_t value);

    // Full-scale range in mT for a Range selection on this part.
    float rangeToMilliTesla(Range range) const;
};

#endif // TMAG5273_h
