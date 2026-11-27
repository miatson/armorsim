#pragma once
// -----------------------------------------------------------------------------
// View.h - camera: converts metres (y up) to screen pixels (y down).
// Scenes are millimetres across, so the default zoom is thousands of pixels
// per metre.
// -----------------------------------------------------------------------------
#include <algorithm>

#include <raylib.h>

struct View {
    double pixelsPerMetre = 10000.0;
    Vector2 origin{800.0f, 450.0f}; // screen position of world (0, 0)

    Vector2 toScreen(double x, double y) const {
        return {static_cast<float>(origin.x + x * pixelsPerMetre), static_cast<float>(origin.y - y * pixelsPerMetre)};
    }
    double toWorldX(float sx) const { return (sx - origin.x) / pixelsPerMetre; }
    double toWorldY(float sy) const { return (origin.y - sy) / pixelsPerMetre; }

    // Zoom by `factor`, keeping the world point under the cursor fixed.
    void zoomAt(Vector2 cursor, double factor) {
        const double wx = toWorldX(cursor.x), wy = toWorldY(cursor.y);
        pixelsPerMetre = std::clamp(pixelsPerMetre * factor, 100.0, 5e6);
        origin.x = static_cast<float>(cursor.x - wx * pixelsPerMetre);
        origin.y = static_cast<float>(cursor.y + wy * pixelsPerMetre);
    }

    // Fit the world rectangle into the screen rectangle (with a margin).
    void fit(double minX, double minY, double maxX, double maxY, Rectangle area) {
        const double w = std::max(maxX - minX, 1e-6), h = std::max(maxY - minY, 1e-6);
        pixelsPerMetre = 0.85 * std::min(area.width / w, area.height / h);
        const double cx = 0.5 * (minX + maxX), cy = 0.5 * (minY + maxY);
        origin.x = static_cast<float>(area.x + area.width * 0.5 - cx * pixelsPerMetre);
        origin.y = static_cast<float>(area.y + area.height * 0.5 + cy * pixelsPerMetre);
    }
};
