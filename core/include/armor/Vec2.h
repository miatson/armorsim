#pragma once
// -----------------------------------------------------------------------------
// Vec2.h - 2D vector (double precision).
//
// Why double here when the rigid-body engine used float? Impact simulations
// take tens of thousands of tiny steps, and elastic displacements can be a
// millionth of a metre on parts half a metre wide. float has only ~7
// significant digits; double has ~16, so small changes aren't lost in rounding.
// -----------------------------------------------------------------------------
#include <cmath>

namespace armor {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

    constexpr Vec2 operator-() const { return {-x, -y}; }
    constexpr Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
    constexpr Vec2& operator*=(double s) { x *= s; y *= s; return *this; }
    constexpr bool operator==(const Vec2&) const = default;
};

constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
constexpr Vec2 operator*(Vec2 v, double s) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator*(double s, Vec2 v) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator/(Vec2 v, double s) { return {v.x / s, v.y / s}; }

constexpr double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
constexpr double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
constexpr double lengthSquared(Vec2 v) { return dot(v, v); }
inline double length(Vec2 v) { return std::sqrt(lengthSquared(v)); }

inline Vec2 rotate(Vec2 v, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}

} // namespace armor
