#include <catch2/catch_all.hpp>

#include <cmath>
#include <vector>

#include "libslic3r/CircleCompensation.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/PrintConfig.hpp"

using namespace Slic3r;

namespace {

Polygon circle(double cx, double cy, double radius, bool counter_clockwise = true, size_t segments = 128)
{
    std::vector<Vec2d> points;
    for (size_t i = 0; i < segments; ++i) {
        const double angle = 2. * M_PI * double(i) / double(segments);
        points.emplace_back(cx + radius * std::cos(angle), cy + radius * std::sin(angle));
    }
    Polygon polygon = Polygon::new_scale(points);
    if (!counter_clockwise)
        polygon.reverse();
    return polygon;
}

Polygon square(double cx, double cy, double half)
{
    return Polygon::new_scale({ { cx - half, cy - half }, { cx + half, cy - half }, { cx + half, cy + half }, { cx - half, cy + half } });
}

double measured_diameter(const Polygon &polygon)
{
    Point  center;
    double diameter = 0.;
    REQUIRE(is_approx_circle(polygon, center, diameter));
    return unscale<double>(diameter);
}

CircleCompensationParams neutral_params()
{
    CircleCompensationParams params;
    params.speed          = 200.;
    params.contour_min    = -0.5;
    params.contour_max    = 0.5;
    params.hole_min       = -0.5;
    params.hole_max       = 0.5;
    params.diameter_limit = 50.;
    return params;
}

} // namespace

TEST_CASE("Circle compensation detects circles only", "[CircleCompensation]")
{
    Point  center;
    double diameter = 0.;
    REQUIRE(is_approx_circle(circle(10., 10., 5.), center, diameter));
    REQUIRE(unscale<double>(diameter) == Catch::Approx(10.).margin(0.01));
    REQUIRE(unscale<double>(center.x()) == Catch::Approx(10.).margin(0.001));
    REQUIRE_FALSE(is_approx_circle(square(0., 0., 10.), center, diameter));
}

TEST_CASE("Circle compensation model follows the Bambu formula", "[CircleCompensation]")
{
    CircleCompensationParams params = neutral_params();
    params.hole_speed_coef    = 0.0005;
    params.hole_diameter_coef = -0.01;
    params.hole_constant      = 0.2;
    REQUIRE(circle_hole_correction(params, 10.) == Catch::Approx(0.0005 * 200. - 0.01 * 10. + 0.2));

    params.hole_max = 0.1;
    REQUIRE(circle_hole_correction(params, 10.) == Catch::Approx(0.1));

    params.contour_constant = -0.9;
    REQUIRE(circle_contour_correction(params, 10.) == Catch::Approx(-0.5));

    params.contour_constant = 0.;
    params.manual_offset    = 0.2;
    REQUIRE(circle_contour_correction(params, 10.) == Catch::Approx(0.1));
    params.hole_max = 0.5;
    REQUIRE(circle_hole_correction(params, 10.) == Catch::Approx(0.1));
}

TEST_CASE("Circle compensation resizes holes and contours", "[CircleCompensation]")
{
    ExPolygon plate(square(0., 0., 20.));
    plate.holes.push_back(circle(0., 0., 5., false));
    ExPolygon pin(circle(40., 0., 4.));

    CircleCompensationParams params = neutral_params();

    SECTION("neutral coefficients keep the geometry")
    {
        REQUIRE(compensate_circles(plate, params) == plate);
        REQUIRE(compensate_circles(pin, params) == pin);
    }

    SECTION("positive hole correction enlarges the hole radially")
    {
        params.hole_constant      = 0.1;
        const ExPolygon result    = compensate_circles(plate, params);
        REQUIRE(result.contour == plate.contour);
        REQUIRE(result.holes.size() == 1);
        REQUIRE(result.holes.front().is_clockwise());
        REQUIRE(measured_diameter(result.holes.front()) == Catch::Approx(10.2).margin(0.005));
    }

    SECTION("negative hole correction shrinks the hole")
    {
        params.hole_constant   = -0.15;
        const ExPolygon result = compensate_circles(plate, params);
        REQUIRE(measured_diameter(result.holes.front()) == Catch::Approx(9.7).margin(0.005));
    }

    SECTION("contour correction scales with the diameter coefficient")
    {
        params.contour_diameter_coef = 0.01;
        const ExPolygon result       = compensate_circles(pin, params);
        REQUIRE(result.contour.is_counter_clockwise());
        REQUIRE(measured_diameter(result.contour) == Catch::Approx(8. + 2. * 0.08).margin(0.005));
    }

    SECTION("manual offset tightens the fit")
    {
        params.manual_offset = 0.2;
        REQUIRE(measured_diameter(compensate_circles(pin, params).contour) == Catch::Approx(8.2).margin(0.005));
        REQUIRE(measured_diameter(compensate_circles(plate, params).holes.front()) == Catch::Approx(9.8).margin(0.005));
    }

    SECTION("circles above the diameter limit are left alone")
    {
        params.hole_constant  = 0.1;
        params.diameter_limit = 9.;
        REQUIRE(compensate_circles(plate, params) == plate);
    }

    SECTION("a hole that would break through the contour is left alone")
    {
        ExPolygon thin(square(0., 0., 5.1));
        thin.holes.push_back(circle(0., 0., 5., false));
        params.hole_constant = 0.5;
        REQUIRE(compensate_circles(thin, params) == thin);
    }
}

TEST_CASE("Circle compensation leaves boundaries between regions alone", "[CircleCompensation]")
{
    ExPolygon plate(square(0., 0., 20.));
    plate.holes.push_back(circle(0., 0., 5., false));
    ExPolygon insert(circle(0., 0., 5.));

    const CircleBoundaries outline = detect_circle_boundaries(ExPolygons{ ExPolygon(square(0., 0., 20.)) });
    REQUIRE(outline.empty());

    CircleCompensationParams params = neutral_params();
    params.hole_constant    = 0.1;
    params.contour_constant = 0.1;
    REQUIRE(compensate_circles(plate, params, &outline) == plate);
    REQUIRE(compensate_circles(insert, params, &outline) == insert);

    const CircleBoundaries real_hole = detect_circle_boundaries(ExPolygons{ plate });
    REQUIRE(real_hole.size() == 1);
    REQUIRE(real_hole.front().hole);
    REQUIRE(measured_diameter(compensate_circles(plate, params, &real_hole).holes.front()) == Catch::Approx(10.2).margin(0.005));
}

TEST_CASE("Circle compensation defaults are neutral", "[CircleCompensation]")
{
    const PrintConfig              config = PrintConfig::defaults();
    const CircleCompensationParams params = CircleCompensationParams::from_config(config, 0, 0.);
    ExPolygon plate(square(0., 0., 20.));
    plate.holes.push_back(circle(0., 0., 5., false));
    REQUIRE(compensate_circles(plate, params) == plate);
    REQUIRE_FALSE(PrintObjectConfig::defaults().enable_circle_compensation.value);
}
