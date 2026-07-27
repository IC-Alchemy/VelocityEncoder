#include "AlchemyOled.h"

#if ALCHEMY_OLED_AVAILABLE

#include <cmath>
#include <string.h>

namespace
{
    constexpr float DEG_TO_RAD_F = 0.017453292519943295f;

    float clamp01(float v)
    {
        if (v < 0.0f)
            return 0.0f;
        if (v > 1.0f)
            return 1.0f;
        return v;
    }
}

AlchemyOled::AlchemyOled(TwoWire &wire)
    : _wire(&wire),
      _display(WIDTH, HEIGHT, &wire, -1),
      _ready(false),
      _address(DEFAULT_ADDR)
{
}

bool AlchemyOled::begin(uint8_t address, uint32_t i2cFrequency)
{
    _address = address;

    // Adafruit_SH110X::begin() brings up Wire itself; the second argument asks
    // it not to toggle a reset pin, which these I2C modules do not have.
    if (!_display.begin(address, false))
    {
        _ready = false;
        return false;
    }

    if (i2cFrequency > 0)
        _wire->setClock(i2cFrequency);

    _display.setTextWrap(false);
    _display.cp437(true); // so DEGREE and the other high glyphs render correctly
    _display.setTextColor(WHITE);
    _display.setTextSize(1);
    clear();
    show();

    _ready = true;
    return true;
}

void AlchemyOled::clear()
{
    _display.clearDisplay();
    _display.setTextColor(WHITE);
    _display.setTextSize(1);
    _display.setCursor(0, 0);
}

void AlchemyOled::show()
{
    _display.display();
}

// ----------------------------------------------------------------------
// Text
// ----------------------------------------------------------------------

Adafruit_SH1106G &AlchemyOled::at(int col, int row, uint8_t size)
{
    return atPixel(col * FONT_W * size, row * FONT_H * size, size);
}

Adafruit_SH1106G &AlchemyOled::atPixel(int x, int y, uint8_t size)
{
    _display.setTextSize(size);
    _display.setCursor(x, y);
    return _display;
}

void AlchemyOled::text(int col, int row, const char *str, uint8_t size)
{
    if (str == nullptr)
        return;
    at(col, row, size).print(str);
}

int AlchemyOled::textWidth(const char *str, uint8_t size)
{
    if (str == nullptr)
        return 0;
    // The built-in GFX font is a fixed 6x8 cell, so width is exact without
    // paying for getTextBounds().
    return static_cast<int>(strlen(str)) * FONT_W * size;
}

void AlchemyOled::textCentered(int y, const char *str, uint8_t size)
{
    if (str == nullptr)
        return;
    const int x = (WIDTH - textWidth(str, size)) / 2;
    atPixel(x < 0 ? 0 : x, y, size).print(str);
}

void AlchemyOled::textRight(int xRight, int y, const char *str, uint8_t size)
{
    if (str == nullptr)
        return;
    const int x = xRight - textWidth(str, size);
    atPixel(x < 0 ? 0 : x, y, size).print(str);
}

void AlchemyOled::title(const char *str, const char *rightStr)
{
    _display.fillRect(0, 0, WIDTH, TITLE_H, WHITE);
    _display.setTextColor(BLACK);
    _display.setTextSize(1);

    if (str != nullptr)
    {
        _display.setCursor(2, 1);
        _display.print(str);
    }

    if (rightStr != nullptr)
    {
        const int x = WIDTH - 2 - textWidth(rightStr, 1);
        _display.setCursor(x < 0 ? 0 : x, 1);
        _display.print(rightStr);
    }

    // Hand the frame back in the normal drawing state.
    _display.setTextColor(WHITE);
}

// ----------------------------------------------------------------------
// Meters
// ----------------------------------------------------------------------

void AlchemyOled::bar(int x, int y, int w, int h, float fraction)
{
    _display.drawRect(x, y, w, h, WHITE);

    const int inner = w - 2;
    if (inner <= 0 || h <= 2)
        return;

    const int fill = static_cast<int>(clamp01(fraction) * inner + 0.5f);
    if (fill > 0)
        _display.fillRect(x + 1, y + 1, fill, h - 2, WHITE);
}

void AlchemyOled::barVertical(int x, int y, int w, int h, float fraction)
{
    _display.drawRect(x, y, w, h, WHITE);

    const int inner = h - 2;
    if (inner <= 0 || w <= 2)
        return;

    const int fill = static_cast<int>(clamp01(fraction) * inner + 0.5f);
    if (fill > 0)
        _display.fillRect(x + 1, y + h - 1 - fill, w - 2, fill, WHITE);
}

void AlchemyOled::bipolarBar(int x, int y, int w, int h, float value, float fullScale)
{
    _display.drawRect(x, y, w, h, WHITE);

    const int inner = w - 2;
    if (inner <= 0 || h <= 2 || fullScale <= 0.0f)
        return;

    const int centre    = x + 1 + inner / 2;
    const int halfWidth = inner / 2;

    // Zero marker: a full-height tick that stays visible under the fill.
    _display.drawFastVLine(centre, y, h, WHITE);

    float norm = value / fullScale;
    if (norm > 1.0f)
        norm = 1.0f;
    if (norm < -1.0f)
        norm = -1.0f;

    const int extent = static_cast<int>(fabsf(norm) * halfWidth + 0.5f);
    if (extent <= 0)
        return;

    if (norm > 0.0f)
        _display.fillRect(centre, y + 1, extent, h - 2, WHITE);
    else
        _display.fillRect(centre - extent, y + 1, extent, h - 2, WHITE);
}

void AlchemyOled::segmentBar(int x, int y, int w, int h, float fraction, uint8_t segments)
{
    if (segments == 0)
        return;

    const float step = static_cast<float>(w) / segments;
    const int   lit  = static_cast<int>(clamp01(fraction) * segments + 0.5f);

    for (uint8_t i = 0; i < segments; ++i)
    {
        const int sx = x + static_cast<int>(i * step + 0.5f);
        const int sw = static_cast<int>((i + 1) * step + 0.5f) -
                       static_cast<int>(i * step + 0.5f) - 1;
        if (sw <= 0)
            continue;

        if (i < lit)
            _display.fillRect(sx, y, sw, h, WHITE);
        else
            _display.drawRect(sx, y, sw, h, WHITE);
    }
}

// ----------------------------------------------------------------------
// Round things
// ----------------------------------------------------------------------

void AlchemyOled::polar(int cx, int cy, float radius, float angleDeg, int &x, int &y)
{
    // 0 degrees points up, angles increase clockwise.
    const float rad = angleDeg * DEG_TO_RAD_F;
    x = cx + static_cast<int>(lroundf(radius * sinf(rad)));
    y = cy - static_cast<int>(lroundf(radius * cosf(rad)));
}

void AlchemyOled::dial(int cx, int cy, int r, uint8_t majorTicks, uint8_t minorPerMajor)
{
    _display.drawCircle(cx, cy, r, WHITE);

    if (majorTicks == 0)
        return;

    if (minorPerMajor == 0)
        minorPerMajor = 1;

    const uint16_t total = static_cast<uint16_t>(majorTicks) * minorPerMajor;
    for (uint16_t i = 0; i < total; ++i)
    {
        const float angle  = (360.0f * i) / total;
        const bool  major  = (i % minorPerMajor) == 0;
        const int   length = major ? 4 : 2;

        int x0, y0, x1, y1;
        polar(cx, cy, static_cast<float>(r), angle, x0, y0);
        polar(cx, cy, static_cast<float>(r - length), angle, x1, y1);
        _display.drawLine(x0, y0, x1, y1, WHITE);
    }
}

void AlchemyOled::needle(int cx, int cy, int length, float angleDeg, uint16_t color)
{
    int tipX, tipY;
    polar(cx, cy, static_cast<float>(length), angleDeg, tipX, tipY);

    // A short tail on the far side reads as a pointer rather than a spoke.
    int tailX, tailY;
    polar(cx, cy, -static_cast<float>(length) * 0.22f, angleDeg, tailX, tailY);

    _display.drawLine(tailX, tailY, tipX, tipY, color);
    _display.fillCircle(cx, cy, 2, color);
    arrowHead(tipX, tipY, angleDeg, 4, color);
}

void AlchemyOled::arc(int cx, int cy, int r, float startDeg, float endDeg, uint16_t color)
{
    if (r <= 0)
        return;

    float sweep = endDeg - startDeg;
    if (fabsf(sweep) < 0.5f)
        return;

    // One segment per ~4 pixels of arc keeps curves smooth without wasting
    // time on hair-thin steps.
    const float arcLength = fabsf(sweep) * DEG_TO_RAD_F * r;
    int steps = static_cast<int>(arcLength / 4.0f) + 2;
    if (steps > 96)
        steps = 96;

    int prevX, prevY;
    polar(cx, cy, static_cast<float>(r), startDeg, prevX, prevY);

    for (int i = 1; i <= steps; ++i)
    {
        const float angle = startDeg + (sweep * i) / steps;
        int x, y;
        polar(cx, cy, static_cast<float>(r), angle, x, y);
        _display.drawLine(prevX, prevY, x, y, color);
        prevX = x;
        prevY = y;
    }
}

void AlchemyOled::ringGauge(int cx, int cy, int r, int thickness, float fraction)
{
    if (thickness < 1)
        thickness = 1;

    const float sweep = clamp01(fraction) * 360.0f;

    // Fill the annulus one radius at a time; on a 128x64 panel the ring is
    // only a few pixels thick so this stays cheap.
    for (int i = 0; i < thickness; ++i)
    {
        const int radius = r - i;
        if (radius <= 0)
            break;
        if (sweep > 0.0f)
            arc(cx, cy, radius, 0.0f, sweep, WHITE);
    }

    // Outline the whole track so the empty part still reads as a gauge.
    _display.drawCircle(cx, cy, r + 1, WHITE);
    const int innerR = r - thickness;
    if (innerR > 0)
        _display.drawCircle(cx, cy, innerR, WHITE);
}

void AlchemyOled::arrowHead(int x, int y, float angleDeg, int size, uint16_t color)
{
    // Two barbs swept back from the tip.
    int leftX, leftY, rightX, rightY;
    polar(x, y, static_cast<float>(size), angleDeg + 150.0f, leftX, leftY);
    polar(x, y, static_cast<float>(size), angleDeg - 150.0f, rightX, rightY);
    _display.fillTriangle(x, y, leftX, leftY, rightX, rightY, color);
}

void AlchemyOled::vector(int x0, int y0, int x1, int y1, uint16_t color)
{
    _display.drawLine(x0, y0, x1, y1, color);

    const int dx = x1 - x0;
    const int dy = y1 - y0;
    if (dx == 0 && dy == 0)
        return;

    // Convert the pixel delta back into the clockwise-from-12 convention.
    const float angle = atan2f(static_cast<float>(dx), static_cast<float>(-dy)) / DEG_TO_RAD_F;
    arrowHead(x1, y1, angle, 4, color);
}

// ----------------------------------------------------------------------
// Plots
// ----------------------------------------------------------------------

void AlchemyOled::sparkline(int x, int y, int w, int h, const uint8_t *samples,
                            uint16_t count, uint16_t head, bool connect)
{
    if (samples == nullptr || count == 0 || w <= 0 || h <= 0)
        return;

    int prevX = 0;
    int prevY = 0;
    bool havePrev = false;

    for (int col = 0; col < w; ++col)
    {
        // Map the column onto the ring buffer, oldest sample at the left edge.
        const uint16_t age   = static_cast<uint16_t>((w - 1 - col) % count);
        const uint16_t index = static_cast<uint16_t>((head + count - 1 - age) % count);

        const int px = x + col;
        const int py = y + h - 1 - ((samples[index] * (h - 1)) / 255);

        if (connect && havePrev)
            _display.drawLine(prevX, prevY, px, py, WHITE);
        else
            _display.drawPixel(px, py, WHITE);

        prevX = px;
        prevY = py;
        havePrev = true;
    }
}

void AlchemyOled::dottedHLine(int x, int y, int w, int step)
{
    if (step < 1)
        step = 1;
    for (int i = 0; i < w; i += step)
        _display.drawPixel(x + i, y, WHITE);
}

void AlchemyOled::dottedVLine(int x, int y, int h, int step)
{
    if (step < 1)
        step = 1;
    for (int i = 0; i < h; i += step)
        _display.drawPixel(x, y + i, WHITE);
}

void AlchemyOled::plotFrame(int x, int y, int w, int h)
{
    _display.drawRect(x, y, w, h, WHITE);
    dottedHLine(x + 1, y + h / 2, w - 2, 3);
    dottedVLine(x + w / 2, y + 1, h - 2, 3);
}

#endif // ALCHEMY_OLED_AVAILABLE
