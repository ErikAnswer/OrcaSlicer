#ifndef slic3r_CircleCompensation_hpp_
#define slic3r_CircleCompensation_hpp_

#include "ExPolygon.hpp"

namespace Slic3r {

class PrintConfig;

// Per-filament diameter correction of circular holes and contours, ported from Bambu Studio 2.0
// (bambulab/BambuStudio, LayerRegion::auto_circle_compensation, AGPL-3.0).
// All lengths in mm. The correction is radial: a positive contour correction makes a cylinder
// bigger, a positive hole correction makes a hole bigger.
struct CircleCompensationParams
{
    double speed{ 0. };
    double contour_speed_coef{ 0. };
    double contour_diameter_coef{ 0. };
    double contour_constant{ 0. };
    double hole_speed_coef{ 0. };
    double hole_diameter_coef{ 0. };
    double hole_constant{ 0. };
    double contour_min{ 0. };
    double contour_max{ 0. };
    double hole_min{ 0. };
    double hole_max{ 0. };
    double diameter_limit{ 0. };
    // Positive values make the fit tighter: cylinders grow and holes shrink by half of it.
    double manual_offset{ 0. };

    static CircleCompensationParams from_config(const PrintConfig &config, size_t filament_idx, double manual_offset);
};

double circle_contour_correction(const CircleCompensationParams &params, double diameter);
double circle_hole_correction(const CircleCompensationParams &params, double diameter);

struct CircleBoundary
{
    Point  center;
    double diameter{ 0. };
    bool   hole{ false };
    // Radial change applied to the circle, scaled; zero when it was left as it was.
    double correction{ 0. };
};
using CircleBoundaries = std::vector<CircleBoundary>;

bool is_approx_circle(const Polygon &polygon, Point &center, double &diameter);
bool change_circle_radius(Polygon &polygon, const Point &center, double radius, double delta);

// Circular contours and holes of the layer outline. With several regions in a layer, only the
// circles found here are real boundaries; the others separate two regions and must not move.
CircleBoundaries detect_circle_boundaries(const ExPolygons &outline);

// Every circle the compensation applies to is appended to compensated, with its new diameter, or the
// original one when the correction is zero or cannot be applied: their walls print at the circle
// compensation speed.
ExPolygon compensate_circles(const ExPolygon &expolygon, const CircleCompensationParams &params, const CircleBoundaries *outline = nullptr,
                             CircleBoundaries *compensated = nullptr);

// Applies to the circles of an outline the correction each one received in its layer region. A circle is
// matched by center, side and original diameter, which may differ from the outline one by up to
// max_offset (scaled) on each side; circles no region changed are left alone.
ExPolygon apply_compensated_circles(const ExPolygon &expolygon, const CircleBoundaries &compensated, double max_offset);

// Circles of a surface that are among the compensated ones of its layer region.
CircleBoundaries find_compensated_circles(const ExPolygon &expolygon, const CircleBoundaries &compensated);
// Whether a wall loop runs around one of the given circles: the loop is itself a circle with the same center,
// on the same side (contour or hole) and inside the material, so no wider than a contour nor narrower than a hole.
bool is_compensated_loop(const Polygon &loop, bool hole, const CircleBoundaries &circles);

} // namespace Slic3r

#endif
