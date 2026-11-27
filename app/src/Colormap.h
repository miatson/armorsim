#pragma once
// -----------------------------------------------------------------------------
// Colormap.h - turn a number into a colour.
//
//  - turbo():    dark blue (lowest) -> cyan -> green -> yellow -> red (highest).
//                For magnitudes (speed, von Mises stress, density).
//                Polynomial fit of Google's "Turbo" map (Mikhailov, 2019).
//  - diverging(): blue (most negative) -> white (zero) -> red (most positive).
//                For signed fields (pressure, stress components) so that
//                tension and compression are instantly distinguishable.
//  - category(): distinct colours for materials.
// -----------------------------------------------------------------------------
#include <algorithm>

#include <raylib.h>

namespace colormap {

inline unsigned char toByte(double v) { return static_cast<unsigned char>(std::clamp(v, 0.0, 1.0) * 255.0 + 0.5); }

inline Color turbo(double t) {
    const double x = std::clamp(t, 0.0, 1.0);
    const double r = 0.13572138 + x * (4.61539260 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))));
    const double g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))));
    const double b = 0.10667330 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))));
    return {toByte(r), toByte(g), toByte(b), 255};
}

// t in [-1, 1]
inline Color diverging(double t) {
    const double x = std::clamp(t, -1.0, 1.0);
    const Color neg{59, 76, 192, 255}, mid{235, 235, 235, 255}, pos{180, 4, 38, 255};
    const Color& end = x < 0 ? neg : pos;
    const double a = x < 0 ? -x : x;
    auto mix = [a](unsigned char from, unsigned char to) {
        return static_cast<unsigned char>(from + (to - from) * a);
    };
    return {mix(mid.r, end.r), mix(mid.g, end.g), mix(mid.b, end.b), 255};
}

inline Color category(int i) {
    static const Color kColors[] = {
        {160, 170, 185, 255}, // steel grey
        {120, 190, 230, 255}, // light blue
        {222, 140, 80, 255},  // copper
        {110, 110, 125, 255}, // dark tungsten
        {170, 200, 140, 255}, // pale green
        {231, 138, 195, 255}, {255, 217, 47, 255}, {141, 160, 203, 255},
    };
    return kColors[static_cast<unsigned>(i) % (sizeof(kColors) / sizeof(kColors[0]))];
}

} // namespace colormap
