#include <catch2/catch_all.hpp>

#include "libslic3r/CircleCompensation.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <string>
#include <vector>

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

struct FirstLayerDiameters
{
    double outline{ 0. };
    double region{ 0. };
};

FirstLayerDiameters first_layer_pin_diameters(const std::string &enabled, const std::string &contour_constant)
{
    Print print;
    Model model;
    Test::init_print({ make_cylinder(4., 4.) }, print, model,
                     { { "enable_circle_compensation", enabled },
                       { "counter_coef_3", contour_constant },
                       { "elefant_foot_compensation", "0.2" },
                       { "elefant_foot_compensation_layers", "1" },
                       { "layer_height", "0.2" },
                       { "initial_layer_print_height", "0.2" } });
    print.process();
    const Layer *layer = print.objects().front()->layers().front();
    REQUIRE(layer->lslices.size() == 1);
    REQUIRE(layer->regions().size() == 1);
    REQUIRE(layer->regions().front()->slices.surfaces.size() == 1);
    FirstLayerDiameters diameters;
    Point               center;
    double              diameter = 0.;
    REQUIRE(is_approx_circle(layer->lslices.front().contour, center, diameter));
    diameters.outline = unscale<double>(diameter);
    REQUIRE(is_approx_circle(layer->regions().front()->slices.surfaces.front().expolygon.contour, center, diameter));
    diameters.region = unscale<double>(diameter);
    return diameters;
}

std::vector<FirstLayerDiameters> first_layer_two_filament_pins(const std::string &contour_constants)
{
    DynamicPrintConfig config = Test::multifilament_config(2, { { "enable_circle_compensation", "1" },
                                                                { "counter_coef_3", contour_constants },
                                                                { "elefant_foot_compensation", "0.2" },
                                                                { "elefant_foot_compensation_layers", "1" },
                                                                { "layer_height", "0.2" },
                                                                { "initial_layer_print_height", "0.2" },
                                                                { "skirt_loops", 0 },
                                                                { "brim_type", "no_brim" } });
    TriangleMesh second = make_cylinder(4., 4.);
    second.translate(15.f, 0.f, 0.f);
    Print print;
    Model model;
    Test::init_print({ make_cylinder(4., 4.) }, print, model, config, nullptr, false);
    ModelObject &object = *model.objects.front();
    object.volumes.front()->config.set("extruder", 1);
    object.add_volume(std::move(second), ModelVolumeType::MODEL_PART, false)->config.set("extruder", 2);
    print.apply(model, config);
    print.process();

    const Layer *layer = print.objects().front()->layers().front();
    REQUIRE(layer->regions().size() == 2);
    REQUIRE(layer->lslices.size() == 2);
    std::vector<FirstLayerDiameters> pins;
    Point                            center;
    double                           diameter = 0.;
    for (unsigned int filament = 1; filament <= 2; ++filament) {
        const auto region = std::find_if(layer->regions().begin(), layer->regions().end(), [filament](const LayerRegion *layerm) {
            return layerm->region().extruder(frExternalPerimeter) == filament;
        });
        REQUIRE(region != layer->regions().end());
        REQUIRE((*region)->slices.surfaces.size() == 1);
        const ExPolygon &slice = (*region)->slices.surfaces.front().expolygon;
        REQUIRE(is_approx_circle(slice.contour, center, diameter));
        FirstLayerDiameters pin;
        pin.region = unscale<double>(diameter);
        const auto island = std::find_if(layer->lslices.begin(), layer->lslices.end(),
                                         [&slice](const ExPolygon &expolygon) { return expolygon.contains(slice.contour.centroid()); });
        REQUIRE(island != layer->lslices.end());
        REQUIRE(is_approx_circle(island->contour, center, diameter));
        pin.outline = unscale<double>(diameter);
        pins.push_back(pin);
    }
    return pins;
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

WallSpeedResult slice_pin(const std::string &wall_generator, const std::string &enabled, const std::string &speed = "17")
{
    Print print;
    Model model;
    TriangleMesh block = make_cube(20., 20., 4.);
    block.translate(Vec3f(15.f, 0.f, 0.f));
    Test::init_print({ make_cylinder(4., 4.), std::move(block) }, print, model,
                     { { "enable_circle_compensation", enabled },
                       { "wall_generator", wall_generator },
                       { "circle_compensation_speed", speed },
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

TEST_CASE("Circle compensation also resizes the first layer outline kept for the elephant foot", "[CircleCompensation]")
{
    const FirstLayerDiameters reference = first_layer_pin_diameters("0", "0");
    REQUIRE(reference.region < reference.outline);

    const FirstLayerDiameters neutral = first_layer_pin_diameters("1", "0");
    REQUIRE(neutral.outline == Catch::Approx(reference.outline).margin(1e-6));

    const FirstLayerDiameters bigger = first_layer_pin_diameters("1", "0.1");
    REQUIRE(bigger.outline == Catch::Approx(reference.outline + 0.2).margin(0.005));
    REQUIRE(bigger.region == Catch::Approx(reference.region + 0.2).margin(0.005));

    const FirstLayerDiameters smaller = first_layer_pin_diameters("1", "-0.1");
    REQUIRE(smaller.outline == Catch::Approx(reference.outline - 0.2).margin(0.005));
    REQUIRE(smaller.region == Catch::Approx(reference.region - 0.2).margin(0.005));
}

TEST_CASE("Circle compensation resizes each island of the first layer outline with its own filament", "[CircleCompensation]")
{
    const std::vector<FirstLayerDiameters> reference = first_layer_two_filament_pins("0,0");
    const std::vector<FirstLayerDiameters> mixed     = first_layer_two_filament_pins("0.1,-0.1");
    REQUIRE(reference.size() == 2);
    REQUIRE(mixed.size() == 2);

    REQUIRE(mixed[0].region == Catch::Approx(reference[0].region + 0.2).margin(0.005));
    REQUIRE(mixed[0].outline == Catch::Approx(reference[0].outline + 0.2).margin(0.005));
    REQUIRE(mixed[1].region == Catch::Approx(reference[1].region - 0.2).margin(0.005));
    REQUIRE(mixed[1].outline == Catch::Approx(reference[1].outline - 0.2).margin(0.005));
}

TEST_CASE("Circle compensation prints the walls of compensated circles at their own speed", "[CircleCompensation]")
{
    const std::string wall_generator = GENERATE(std::string("classic"), std::string("arachne"));
    CAPTURE(wall_generator);

    const WallSpeedResult enabled = slice_pin(wall_generator, "1");
    REQUIRE(enabled.compensated_loops == 2);
    REQUIRE(enabled.gcode.find("F1020") != std::string::npos);
    // The block has no circles: its walls keep the outer and inner wall speeds.
    REQUIRE(enabled.gcode.find("F3600") != std::string::npos);
    REQUIRE(enabled.gcode.find("F4800") != std::string::npos);

    const WallSpeedResult unset = slice_pin(wall_generator, "1", "0");
    REQUIRE(unset.gcode.find("F1020") == std::string::npos);

    const WallSpeedResult disabled = slice_pin(wall_generator, "0");
    REQUIRE(disabled.compensated_loops == 0);
    REQUIRE(disabled.gcode.find("F1020") == std::string::npos);
}
