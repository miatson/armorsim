#include <doctest/doctest.h>

#include <armor/Mat2.h>
#include <armor/Material.h>
#include <armor/SymTensor.h>

using namespace armor;
using doctest::Approx;

TEST_CASE("Mat2 inverse and product") {
    const Mat2 m{4.0, 7.0, 2.0, 6.0};
    const Mat2 id = m * m.inverse();
    CHECK(id.m00 == Approx(1.0));
    CHECK(id.m01 == Approx(0.0));
    CHECK(id.m10 == Approx(0.0));
    CHECK(id.m11 == Approx(1.0));
    const Mat2 o = outer({1.0, 2.0}, {3.0, 4.0});
    CHECK(o.m00 == 3.0);
    CHECK(o.m01 == 4.0);
    CHECK(o.m10 == 6.0);
    CHECK(o.m11 == 8.0);
}

TEST_CASE("von Mises stress of a simple pull test equals the pull stress") {
    // Uniaxial stress σ along x: total stress diag(σ, 0, 0).
    const SymTensor sigma{100e6, 0.0, 0.0, 0.0};
    CHECK(vonMises(deviator(sigma)) == Approx(100e6));
    // Pure hydrostatic pressure has no deviatoric part.
    CHECK(vonMises(deviator(SymTensor{-5e9, -5e9, 0.0, -5e9})) == Approx(0.0).epsilon(1e-9));
    // Pure shear τ: von Mises = sqrt(3)·τ
    CHECK(vonMises(deviator(SymTensor{0.0, 0.0, 50e6, 0.0})) == Approx(std::sqrt(3.0) * 50e6));
}

TEST_CASE("principal stresses") {
    const Principal2D p = principal(SymTensor{0.0, 0.0, 10.0, 0.0}); // pure shear -> ±10 at 45°
    CHECK(std::max(p.s1, p.s2) == Approx(10.0));
    CHECK(std::min(p.s1, p.s2) == Approx(-10.0));
    const Principal2D q = principal(SymTensor{3.0, -2.0, 0.0, 0.0});
    CHECK(q.s1 == Approx(3.0));
    CHECK(q.s2 == Approx(-2.0));
}

TEST_CASE("elastic constants convert correctly") {
    const Material steel = defaultMaterials()[kSteelRHA];
    // K now comes from the shock data (K = ρ0·c0²), so E and ν match the
    // handbook values to within a fraction of a percent.
    CHECK(steel.youngsModulus() == Approx(207e9).epsilon(0.01));
    // .scale(0): doctest's Approx otherwise adds an absolute tolerance of
    // epsilon × 1.0, which swamps comparisons of numbers smaller than ~1.
    CHECK(steel.poissonRatio() == Approx(0.29).epsilon(0.01).scale(0));
    // Rod wave speed in steel ~5.1 km/s
    CHECK(steel.barWaveSpeed() == Approx(5135.0).epsilon(0.01));
    // 1% compression ≈ 1% of K. (Mie-Grüneisen is slightly stiffer than the
    // linear law even here: ~+1.7%.)
    CHECK(steel.pressure(steel.density0 * 1.01, 0.0) == Approx(0.01 * steel.bulkModulus).epsilon(0.03));
}
