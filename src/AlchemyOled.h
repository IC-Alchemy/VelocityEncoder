#ifndef AlchemyOled_h
#define AlchemyOled_h

#include <Arduino.h>
#include <Wire.h>

/**
 * ALCHEMY_OLED_AVAILABLE
 * ----------------------
 * The OLED helper is optional. It compiles to nothing unless the Adafruit
 * SH110X driver is reachable, so the rest of EncoderAlchemy still builds on a
 * system with no display libraries installed at all.
 *
 * IMPORTANT: a sketch that wants the display must include the Adafruit headers
 * itself, before this one:
 *
 *   #include <Adafruit_GFX.h>
 *   #include <Adafruit_SH110X.h>
 *   #include <AlchemyOled.h>
 *
 * That is not decoration. The Arduino builder works out which libraries to put
 * on the include path by following the #include lines it can actually see in
 * the sketch. The include below is behind __has_include, so the builder never
 * sees it and never adds Adafruit SH110X — the sketch has to name it.
 *
 * With that done, a sketch can check the macro and fail with one clear line
 * instead of a wall of "no such file" errors:
 *
 *   #if !ALCHEMY_OLED_AVAILABLE
 *   #error "Install the Adafruit SH110X and Adafruit GFX libraries."
 *   #endif
 */
#if defined(__has_include)
#  if __has_include(<Adafruit_SH110X.h>)
#    define ALCHEMY_OLED_AVAILABLE 1
#  endif
#endif

#ifndef ALCHEMY_OLED_AVAILABLE
#  define ALCHEMY_OLED_AVAILABLE 0
#endif

#if ALCHEMY_OLED_AVAILABLE

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

/**
 * AlchemyOled
 * -----------
 * A thin, opinionated wrapper around Adafruit_SH1106G for the 128x64 monochrome
 * modules that pair naturally with a magnetic knob.
 *
 * Adafruit_GFX already draws lines, rectangles and circles. What it does not
 * give you is the vocabulary a sensor readout is actually written in: a title
 * bar, a bar meter, a bipolar meter that grows out from the centre, a dial with
 * a needle on it, an arc, a ring gauge, a sparkline, and text addressed by
 * character cell rather than by pixel. Those live here.
 *
 * The underlying Adafruit_SH1106G is always reachable through gfx(), so nothing
 * in this class blocks you from drawing whatever you like.
 *
 * Typical use:
 *
 *   AlchemyOled oled;
 *   oled.begin();
 *   ...
 *   oled.clear();
 *   oled.title("FIELD");
 *   oled.bipolarBar(2, 16, 100, 7, bx, 40.0f);
 *   oled.at(0, 4).print("Bx");
 *   oled.show();
 */
class AlchemyOled
{
public:
    // Display geometry. FONT_H is the row height of the default GFX font at
    // text size 1 (6x8 pixels); row indices multiply through FONT_H * size.
    static constexpr int WIDTH      = 128;
    static constexpr int HEIGHT     = 64;
    static constexpr int FONT_W     = 6;
    static constexpr int FONT_H     = 8;
    static constexpr int COLS_SIZE1 = 21;   // 128 / 6 chars at size 1
    static constexpr int ROWS_SIZE1 = 8;    // 64 / 8 rows at size 1

    // Default I2C address for the common 1.3" SH1106G modules.
    static constexpr uint8_t DEFAULT_ADDR = 0x3C;
    static constexpr uint32_t DEFAULT_I2C_FREQ = 400000UL; // 400 kHz

    // Height of the inverted header drawn by title().
    static constexpr int TITLE_H = 9;

    /** Colours, named so a sketch does not have to spell SH110X_ every time. */
    static constexpr uint16_t BLACK = SH110X_BLACK;
    static constexpr uint16_t WHITE = SH110X_WHITE;

    /**
     * Degree sign in the built-in font. begin() turns on CP437 mapping, which
     * is what makes this character come out as a degree sign rather than as
     * the classic font's off-by-one substitute.
     */
    static constexpr char DEGREE = '\xF8';

    /**
     * Construct against an I2C bus. Nothing touches the hardware until
     * begin() is called.
     */
    explicit AlchemyOled(TwoWire &wire = Wire);

    /**
     * Bring up the panel. Returns false if the module did not answer, which
     * lets a sketch carry on headless instead of hanging.
     */
    bool begin(uint8_t address = DEFAULT_ADDR, uint32_t i2cFrequency = DEFAULT_I2C_FREQ);

    /** True once begin() has succeeded. */
    bool isReady() const { return _ready; }

    /** The panel's I2C address, as passed to begin(). */
    uint8_t getAddress() const { return _address; }

    /** Direct access to the Adafruit_GFX surface. */
    Adafruit_SH1106G &gfx() { return _display; }

    // ------------------------------------------------------------------
    // Frame control
    // ------------------------------------------------------------------

    /** Clear the back buffer and reset text state to white, size 1. */
    void clear();

    /** Push the back buffer to the panel. */
    void show();

    // ------------------------------------------------------------------
    // Text
    // ------------------------------------------------------------------

    /**
     * Place the text cursor at a character cell and set the text size, then
     * hand back the GFX surface so prints can be chained:
     *
     *   oled.at(0, 3).print("angle");
     *
     * Cells are FONT_W x FONT_H pixels multiplied by `size`.
     */
    Adafruit_SH1106G &at(int col, int row, uint8_t size = 1);

    /** As at(), but positioned in pixels. */
    Adafruit_SH1106G &atPixel(int x, int y, uint8_t size = 1);

    /** Draw a string at a character cell. */
    void text(int col, int row, const char *str, uint8_t size = 1);

    /** Draw a string centred horizontally on a pixel row. */
    void textCentered(int y, const char *str, uint8_t size = 1);

    /** Draw a string ending at pixel x (right-aligned). */
    void textRight(int xRight, int y, const char *str, uint8_t size = 1);

    /** Pixel width a string occupies at the given text size. */
    static int textWidth(const char *str, uint8_t size = 1);

    /**
     * Inverted header bar across the top of the screen with `str` on the left
     * and optional `rightStr` on the right. Leaves the text state white-on-
     * black so the rest of the frame draws normally.
     */
    void title(const char *str, const char *rightStr = nullptr);

    // ------------------------------------------------------------------
    // Meters
    // ------------------------------------------------------------------

    /**
     * Horizontal bar meter. `fraction` is clamped to [0, 1] and fills the
     * frame from the left edge.
     */
    void bar(int x, int y, int w, int h, float fraction);

    /** Vertical bar meter, filling from the bottom edge. */
    void barVertical(int x, int y, int w, int h, float fraction);

    /**
     * Centre-zero bar meter: the fill grows right for positive values and left
     * for negative ones, with a tick marking zero. `fullScale` is the value
     * that fills half the width.
     */
    void bipolarBar(int x, int y, int w, int h, float value, float fullScale);

    /**
     * Segmented level meter — `segments` discrete blocks of which the first
     * `fraction * segments` are filled. Reads better than a solid bar when the
     * value is being watched rather than measured.
     */
    void segmentBar(int x, int y, int w, int h, float fraction, uint8_t segments = 10);

    // ------------------------------------------------------------------
    // Round things
    // ------------------------------------------------------------------

    /**
     * Circular dial face: an outline circle with `majorTicks` long marks and
     * `minorPerMajor - 1` short marks between each pair. Angles run clockwise
     * from 12 o'clock, the way a compass reads.
     */
    void dial(int cx, int cy, int r, uint8_t majorTicks = 12, uint8_t minorPerMajor = 3);

    /**
     * Needle from the hub to `length` pixels out, pointing at `angleDeg`
     * measured clockwise from 12 o'clock. Draws a filled hub so short needles
     * still read as a pointer.
     */
    void needle(int cx, int cy, int length, float angleDeg, uint16_t color = WHITE);

    /** Arc of radius r between two clockwise-from-12-o'clock angles. */
    void arc(int cx, int cy, int r, float startDeg, float endDeg, uint16_t color = WHITE);

    /**
     * Ring gauge: a `thickness`-pixel annulus filled clockwise from 12 o'clock
     * by `fraction` of a full turn, with the empty remainder outlined.
     */
    void ringGauge(int cx, int cy, int r, int thickness, float fraction);

    /** Point on a circle, angle measured clockwise from 12 o'clock. */
    static void polar(int cx, int cy, float radius, float angleDeg, int &x, int &y);

    /** Small filled arrowhead pointing along `angleDeg`. */
    void arrowHead(int x, int y, float angleDeg, int size = 4, uint16_t color = WHITE);

    /** Line with an arrowhead on the far end. */
    void vector(int x0, int y0, int x1, int y1, uint16_t color = WHITE);

    // ------------------------------------------------------------------
    // Plots
    // ------------------------------------------------------------------

    /**
     * Sparkline of a ring buffer of samples normalized to 0..255, oldest first.
     * `head` is the index the next sample would be written to, so the plot
     * scrolls without having to shuffle the buffer.
     */
    void sparkline(int x, int y, int w, int h, const uint8_t *samples,
                   uint16_t count, uint16_t head, bool connect = true);

    /** Dotted horizontal rule, useful as a zero line inside a plot lane. */
    void dottedHLine(int x, int y, int w, int step = 3);

    /** Dotted vertical rule. */
    void dottedVLine(int x, int y, int h, int step = 3);

    /** Rectangle with a crosshair through the middle — a plot frame. */
    void plotFrame(int x, int y, int w, int h);

private:
    TwoWire         *_wire;
    Adafruit_SH1106G _display;
    bool             _ready;
    uint8_t          _address;
};

#endif // ALCHEMY_OLED_AVAILABLE

#endif // AlchemyOled_h
