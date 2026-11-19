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
};
using CircleBoundaries = std::vector<CircleBoundary>;

bool is_approx_circle(const Polygon &polygon, Point &center, double &diameter);
bool change_circle_radius(Polygon &polygon, const Point &center, double radius, double delta);

// Circular contours and holes of the layer outline. With several regions in a layer, only the
// circles found here are real boundaries; the others separate two regions and must not move.
CircleBoundaries detect_circle_boundaries(const ExPolygons &outline);

ExPolygon compensate_circles(const ExPolygon &expolygon, const CircleCompensationParams &params, const CircleBoundaries *outline = nullptr);

} // namespace Slic3r

#endif
