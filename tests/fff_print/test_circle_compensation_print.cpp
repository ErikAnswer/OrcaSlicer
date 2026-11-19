#include <catch2/catch_all.hpp>

#include "libslic3r/CircleCompensation.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
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

size_t compensated_loops(const Layer &layer)
{
    size_t count = 0;
    for (const LayerRegion *layerm : layer.regions())
        for (const ExtrusionEntity *entity : layerm->perimeters.flatten().entities)
            if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity); loop != nullptr && loop->circle_compensation)
                ++count;
    return count;
}

struct WallSpeedResult
{
    size_t      compensated_loops{ 0 };
    std::string gcode;
};

WallSpeedResult slice_pin(const std::string &wall_generator, const std::string &enabled)
{
    Print print;
    Model model;
    Test::init_print({ make_cylinder(4., 4.) }, print, model,
                     { { "enable_circle_compensation", enabled },
                       { "wall_generator", wall_generator },
                       { "circle_compensation_speed", "17" },
                       { "outer_wall_speed", "60" },
                       { "inner_wall_speed", "80" },
                       { "wall_loops", "2" },
                       { "slow_down_for_layer_cooling", "0" },
                       { "elefant_foot_compensation", "0" },
                       { "layer_height", "0.2" },
                       { "initial_layer_print_height", "0.2" } });
    WallSpeedResult result;
    result.gcode             = Test::gcode(print);
    result.compensated_loops = compensated_loops(*print.objects().front()->layers()[5]);
    return result;
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

TEST_CASE("Circle compensation prints the walls of compensated circles at their own speed", "[CircleCompensation]")
{
    const std::string wall_generator = GENERATE(std::string("classic"), std::string("arachne"));
    CAPTURE(wall_generator);

    const WallSpeedResult enabled = slice_pin(wall_generator, "1");
    REQUIRE(enabled.compensated_loops == 2);
    REQUIRE(enabled.gcode.find("F1020") != std::string::npos);

    const WallSpeedResult disabled = slice_pin(wall_generator, "0");
    REQUIRE(disabled.compensated_loops == 0);
    REQUIRE(disabled.gcode.find("F1020") == std::string::npos);
}
