#include <catch2/catch_all.hpp>

#include "libslic3r/CircleCompensation.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_helpers.hpp"

#include <string>

using namespace Slic3r;

namespace {

double pin_diameter(const std::string &enabled, const std::string &contour_constant)
{
    Print print;
    Model model;
    Test::init_print({ make_cylinder(4., 4.) }, print, model,
                     { { "enable_circle_compensation", enabled },
                       { "counter_coef_3", contour_constant },
                       { "elefant_foot_compensation", "0" },
                       { "layer_height", "0.2" },
                       { "initial_layer_print_height", "0.2" } });
    print.process();
    const Layer *layer = print.objects().front()->layers()[5];
    REQUIRE(layer->lslices.size() == 1);
    Point  center;
    double diameter = 0.;
    REQUIRE(is_approx_circle(layer->lslices.front().contour, center, diameter));
    return unscale<double>(diameter);
}

} // namespace

TEST_CASE("Circle compensation resizes a sliced cylinder only when enabled", "[CircleCompensation]")
{
    const double reference = pin_diameter("0", "0");
    REQUIRE(pin_diameter("0", "0.1") == Catch::Approx(reference).margin(1e-6));
    REQUIRE(pin_diameter("1", "0") == Catch::Approx(reference).margin(1e-6));
    REQUIRE(pin_diameter("1", "0.1") == Catch::Approx(reference + 0.2).margin(0.005));
    REQUIRE(pin_diameter("1", "-0.1") == Catch::Approx(reference - 0.2).margin(0.005));
}
