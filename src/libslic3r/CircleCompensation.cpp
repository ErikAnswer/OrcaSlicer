#include "CircleCompensation.hpp"

#include "ClipperUtils.hpp"
#include "PrintConfig.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace Slic3r {

namespace {

const double circle_max_deviation = scale_(0.5);
const double circle_max_variance  = 5. * scale_(0.01) * scale_(0.01);
const double circle_min_radius    = scale_(0.05);
const double boundary_center_tolerance   = scale_(0.05);
const double boundary_diameter_tolerance = scale_(0.1);
const double compensated_diameter_tolerance = scale_(0.2);

double clamp_correction(double value, double min_value, double max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

bool keeps_topology(const ExPolygon &original, const ExPolygon &compensated)
{
    const ExPolygons merged = union_ex(ExPolygons{ compensated });
    if (merged.size() != 1 || merged.front().holes.size() != original.holes.size())
        return false;
    return std::abs(merged.front().area() - compensated.area()) <= std::abs(compensated.area()) * 1e-3;
}

bool matches_circle(const CircleBoundaries &circles, const Point &center, double diameter, bool hole, double diameter_tolerance)
{
    return std::any_of(circles.begin(), circles.end(), [&](const CircleBoundary &circle) {
        return circle.hole == hole && (circle.center - center).cast<double>().norm() <= boundary_center_tolerance &&
               std::abs(circle.diameter - diameter) <= diameter_tolerance;
    });
}

bool is_outline_circle(const CircleBoundaries *outline, const Point &center, double diameter, bool hole)
{
    return outline == nullptr || matches_circle(*outline, center, diameter, hole, boundary_diameter_tolerance);
}

} // namespace

CircleCompensationParams CircleCompensationParams::from_config(const PrintConfig &config, size_t filament_idx, double manual_offset)
{
    CircleCompensationParams params;
    params.speed                 = config.circle_compensation_speed.get_at(filament_idx);
    params.contour_speed_coef    = config.counter_coef_1.get_at(filament_idx);
    params.contour_diameter_coef = config.counter_coef_2.get_at(filament_idx);
    params.contour_constant      = config.counter_coef_3.get_at(filament_idx);
    params.hole_speed_coef       = config.hole_coef_1.get_at(filament_idx);
    params.hole_diameter_coef    = config.hole_coef_2.get_at(filament_idx);
    params.hole_constant         = config.hole_coef_3.get_at(filament_idx);
    params.contour_min           = config.counter_limit_min.get_at(filament_idx);
    params.contour_max           = config.counter_limit_max.get_at(filament_idx);
    params.hole_min              = config.hole_limit_min.get_at(filament_idx);
    params.hole_max              = config.hole_limit_max.get_at(filament_idx);
    params.diameter_limit        = config.diameter_limit.get_at(filament_idx);
    params.manual_offset         = manual_offset;
    return params;
}

double circle_contour_correction(const CircleCompensationParams &params, double diameter)
{
    const double model = params.contour_speed_coef * params.speed + params.contour_diameter_coef * diameter + params.contour_constant;
    return clamp_correction(model, params.contour_min, params.contour_max) + params.manual_offset / 2.;
}

double circle_hole_correction(const CircleCompensationParams &params, double diameter)
{
    const double model = params.hole_speed_coef * params.speed + params.hole_diameter_coef * diameter + params.hole_constant;
    return clamp_correction(model, params.hole_min, params.hole_max) - params.manual_offset / 2.;
}

bool is_approx_circle(const Polygon &polygon, Point &center, double &diameter)
{
    if (polygon.points.size() < 8)
        return false;

    center = polygon.centroid();
    std::vector<double> distances;
    distances.reserve(polygon.points.size());
    for (const Point &point : polygon.points)
        distances.push_back((point - center).cast<double>().norm());

    const auto [min_it, max_it] = std::minmax_element(distances.begin(), distances.end());
    if (*max_it - *min_it > circle_max_deviation)
        return false;

    const double average  = std::accumulate(distances.begin(), distances.end(), 0.) / double(distances.size());
    double       variance = 0.;
    for (double distance : distances)
        variance += (distance - average) * (distance - average);
    variance /= double(distances.size());
    if (variance > circle_max_variance)
        return false;

    diameter = 2. * average;
    return true;
}

bool change_circle_radius(Polygon &polygon, const Point &center, double radius, double delta)
{
    const double new_radius = radius + delta;
    if (radius <= 0. || new_radius < circle_min_radius)
        return false;
    const double  factor = new_radius / radius;
    const Vec2d   origin = center.cast<double>();
    for (Point &point : polygon.points) {
        const Vec2d moved = origin + (point.cast<double>() - origin) * factor;
        point = Point(coord_t(std::round(moved.x())), coord_t(std::round(moved.y())));
    }
    return true;
}

CircleBoundaries detect_circle_boundaries(const ExPolygons &outline)
{
    CircleBoundaries boundaries;
    Point            center;
    double           diameter = 0.;
    for (const ExPolygon &expolygon : outline) {
        if (is_approx_circle(expolygon.contour, center, diameter))
            boundaries.push_back({ center, diameter, false });
        for (const Polygon &hole : expolygon.holes)
            if (is_approx_circle(hole, center, diameter))
                boundaries.push_back({ center, diameter, true });
    }
    return boundaries;
}

ExPolygon compensate_circles(const ExPolygon &expolygon, const CircleCompensationParams &params, const CircleBoundaries *outline,
                             CircleBoundaries *compensated_circles)
{
    const double     diameter_limit = scale_(params.diameter_limit);
    ExPolygon        compensated    = expolygon;
    CircleBoundaries handled;
    CircleBoundaries original;
    bool             changed = false;
    Point            center;
    double           diameter = 0.;

    auto compensate = [&](Polygon &polygon, bool hole) {
        if (!is_approx_circle(polygon, center, diameter) || diameter > diameter_limit || !is_outline_circle(outline, center, diameter, hole))
            return;
        original.push_back({ center, diameter, hole });
        const double delta = hole ? circle_hole_correction(params, unscale<double>(diameter)) :
                                    circle_contour_correction(params, unscale<double>(diameter));
        if (delta != 0. && change_circle_radius(polygon, center, diameter / 2., scale_(delta))) {
            handled.push_back({ center, diameter + 2. * scale_(delta), hole });
            changed = true;
        } else {
            handled.push_back(original.back());
        }
    };
    compensate(compensated.contour, false);
    for (Polygon &hole : compensated.holes)
        compensate(hole, true);

    const bool keep = changed && keeps_topology(expolygon, compensated);
    if (compensated_circles != nullptr)
        append(*compensated_circles, keep || !changed ? std::move(handled) : std::move(original));
    return keep ? compensated : expolygon;
}

CircleBoundaries find_compensated_circles(const ExPolygon &expolygon, const CircleBoundaries &compensated)
{
    CircleBoundaries found;
    if (compensated.empty())
        return found;
    Point  center;
    double diameter = 0.;
    auto   find = [&](const Polygon &polygon, bool hole) {
        if (is_approx_circle(polygon, center, diameter) && matches_circle(compensated, center, diameter, hole, compensated_diameter_tolerance))
            found.push_back({ center, diameter, hole });
    };
    find(expolygon.contour, false);
    for (const Polygon &hole : expolygon.holes)
        find(hole, true);
    return found;
}

bool is_compensated_loop(const Polygon &loop, bool hole, const CircleBoundaries &circles)
{
    if (circles.empty() || loop.points.size() < 3 || std::abs(loop.area()) < 1.)
        return false;
    const Point center = loop.centroid();
    return std::any_of(circles.begin(), circles.end(), [&](const CircleBoundary &circle) {
        return circle.hole == hole && (circle.center - center).cast<double>().norm() <= boundary_center_tolerance;
    });
}

} // namespace Slic3r
