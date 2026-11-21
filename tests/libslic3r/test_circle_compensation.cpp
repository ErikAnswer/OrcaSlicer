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

TEST_CASE("Circle compensation records the circles it handles", "[CircleCompensation]")
{
    const ExPolygon tube(circle(0., 0., 5.), circle(0., 0., 2., false));

    SECTION("with a zero correction the circles are kept and recorded") {
        CircleBoundaries handled;
        const ExPolygon  result = compensate_circles(tube, neutral_params(), nullptr, &handled);
        REQUIRE(result.contour.points == tube.contour.points);
        REQUIRE(handled.size() == 2);
        REQUIRE(unscale<double>(handled[0].diameter) == Catch::Approx(10.).margin(0.01));
        REQUIRE_FALSE(handled[0].hole);
        REQUIRE(unscale<double>(handled[1].diameter) == Catch::Approx(4.).margin(0.01));
        REQUIRE(handled[1].hole);
    }

    SECTION("with a correction the recorded diameter is the new one") {
        CircleCompensationParams params = neutral_params();
        params.contour_constant         = 0.1;
        CircleBoundaries handled;
        compensate_circles(tube, params, nullptr, &handled);
        REQUIRE(handled.size() == 2);
        REQUIRE(unscale<double>(handled[0].diameter) == Catch::Approx(10.2).margin(0.01));
    }

    SECTION("circles above the diameter limit are not recorded") {
        CircleCompensationParams params = neutral_params();
        params.diameter_limit           = 6.;
        CircleBoundaries handled;
        compensate_circles(tube, params, nullptr, &handled);
        REQUIRE(handled.size() == 1);
        REQUIRE(handled[0].hole);
    }
}

TEST_CASE("Circle compensation finds the walls of compensated circles", "[CircleCompensation]")
{
    const ExPolygon  tube(circle(0., 0., 5.), circle(0., 0., 2., false));
    CircleBoundaries handled;
    compensate_circles(tube, neutral_params(), nullptr, &handled);

    const CircleBoundaries found = find_compensated_circles(tube, handled);
    REQUIRE(found.size() == 2);

    SECTION("inner walls around the same center match on their own side") {
        REQUIRE(is_compensated_loop(circle(0., 0., 4.6), false, found));
        REQUIRE(is_compensated_loop(circle(0., 0., 2.4, false), true, found));
    }

    SECTION("walls elsewhere do not match") {
        REQUIRE_FALSE(is_compensated_loop(circle(1., 0., 4.6), false, found));
        REQUIRE_FALSE(is_compensated_loop(square(10., 10., 1.), false, found));
    }

    SECTION("a surface whose circles were not compensated has none") {
        const ExPolygon other(circle(20., 0., 5.));
        REQUIRE(find_compensated_circles(other, handled).empty());
        REQUIRE(find_compensated_circles(tube, CircleBoundaries{}).empty());
    }

    SECTION("only the contour is found when only the contour was compensated") {
        CircleBoundaries contour_only{ handled.front() };
        const CircleBoundaries contour = find_compensated_circles(tube, contour_only);
        REQUIRE(contour.size() == 1);
        REQUIRE_FALSE(is_compensated_loop(circle(0., 0., 2.4, false), true, contour));
    }
}

TEST_CASE("Circle compensation applies the correction of each region to the layer outline", "[CircleCompensation]")
{
    const ExPolygon outline(circle(0., 0., 10.), circle(0., 0., 4., false));
    const double    elephant_foot = scale_(0.2);

    CircleBoundaries compensated;
    compensated.push_back({ Point::new_scale(0., 0.), scale_(19.6 + 0.2), false, scale_(0.1) });
    compensated.push_back({ Point::new_scale(0., 0.), scale_(8.4 - 0.1), true, scale_(-0.05) });

    SECTION("each circle takes the correction of the region it belongs to") {
        const ExPolygon result = apply_compensated_circles(outline, compensated, elephant_foot);
        REQUIRE(measured_diameter(result.contour) == Catch::Approx(20.2).margin(0.01));
        REQUIRE(result.holes.size() == 1);
        REQUIRE(measured_diameter(result.holes.front()) == Catch::Approx(7.9).margin(0.01));
    }

    SECTION("a circle no region changed is left alone") {
        CircleBoundaries only_contour{ compensated.front() };
        only_contour.push_back({ Point::new_scale(0., 0.), scale_(8.4), true, 0. });
        const ExPolygon result = apply_compensated_circles(outline, only_contour, elephant_foot);
        REQUIRE(measured_diameter(result.contour) == Catch::Approx(20.2).margin(0.01));
        REQUIRE(result.holes.front().points == outline.holes.front().points);
    }

    SECTION("circles of another island with the same center are not mixed up") {
        CircleBoundaries nested = compensated;
        nested.push_back({ Point::new_scale(0., 0.), scale_(5.6 - 0.6), false, scale_(-0.3) });
        const ExPolygon result = apply_compensated_circles(outline, nested, elephant_foot);
        REQUIRE(measured_diameter(result.contour) == Catch::Approx(20.2).margin(0.01));
    }

    SECTION("a circle that differs more than the offset is not matched") {
        const ExPolygon result = apply_compensated_circles(outline, compensated, 0.);
        REQUIRE(result.contour.points == outline.contour.points);
        REQUIRE(result.holes.front().points == outline.holes.front().points);
    }

    SECTION("without compensated circles nothing changes") {
        const ExPolygon result = apply_compensated_circles(outline, {}, elephant_foot);
        REQUIRE(result.contour.points == outline.contour.points);
    }
}

TEST_CASE("Circle compensation records the correction it applies", "[CircleCompensation]")
{
    const ExPolygon          tube(circle(0., 0., 5.), circle(0., 0., 2., false));
    CircleCompensationParams params = neutral_params();
    params.contour_constant         = 0.1;
    params.hole_constant            = -0.05;
    CircleBoundaries handled;
    compensate_circles(tube, params, nullptr, &handled);
    REQUIRE(handled.size() == 2);
    REQUIRE(unscale<double>(handled[0].correction) == Catch::Approx(0.1).margin(1e-6));
    REQUIRE(unscale<double>(handled[1].correction) == Catch::Approx(-0.05).margin(1e-6));

    CircleBoundaries untouched;
    compensate_circles(tube, neutral_params(), nullptr, &untouched);
    REQUIRE(untouched.size() == 2);
    REQUIRE(untouched[0].correction == 0.);
    REQUIRE(untouched[1].correction == 0.);
}
