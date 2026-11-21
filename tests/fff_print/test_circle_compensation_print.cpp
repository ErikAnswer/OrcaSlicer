#include <catch2/catch_all.hpp>

#include "libslic3r/CircleCompensation.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <utility>
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

enum class CirclePart { Pin, Hole };

const double part_center    = 10.;
const double circle_radius  = 4.;
const double middle_layer_z = 2.2;
const double first_layer_z  = 0.2;

struct PartSlice
{
    size_t      compensated_loops{ 0 };
    std::string gcode;
};

// An 8mm pin, or a 20x20 block with an 8mm bore through its middle, 4mm tall, with walls only: no sparse
// infill, skirt or brim, and every wall speed set so each feed rate in the G-code tells which one it is.
PartSlice slice_part(CirclePart part, const std::string &wall_generator, const std::vector<std::pair<std::string, std::string>> &overrides)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "enable_circle_compensation", "1" },
                                    { "wall_generator", wall_generator },
                                    { "circle_compensation_speed", "17" },
                                    { "outer_wall_speed", "60" },
                                    { "inner_wall_speed", "80" },
                                    { "initial_layer_speed", "30" },
                                    { "small_perimeter_threshold", "0" },
                                    { "wall_loops", "2" },
                                    { "sparse_infill_density", "0%" },
                                    { "slow_down_for_layer_cooling", "0" },
                                    { "slow_down_layers", "0" },
                                    { "elefant_foot_compensation", "0" },
                                    { "skirt_loops", "0" },
                                    { "brim_type", "no_brim" },
                                    { "layer_height", "0.2" },
                                    { "initial_layer_print_height", "0.2" } });
    for (const auto &[key, value] : overrides)
        config.set_deserialize_strict({ { key, value } });

    Print        print;
    Model        model;
    ModelObject *object = model.add_object();
    object->name        = part == CirclePart::Pin ? "pin.stl" : "bored_block.stl";
    if (part == CirclePart::Pin) {
        TriangleMesh pin = make_cylinder(circle_radius, 4.);
        pin.translate(float(part_center), float(part_center), 0.f);
        object->add_volume(std::move(pin), ModelVolumeType::MODEL_PART, false);
    } else {
        object->add_volume(make_cube(2. * part_center, 2. * part_center, 4.), ModelVolumeType::MODEL_PART, false);
        TriangleMesh bore = make_cylinder(circle_radius, 6.);
        bore.translate(float(part_center), float(part_center), -1.f);
        object->add_volume(std::move(bore), ModelVolumeType::NEGATIVE_VOLUME, false);
    }
    object->add_instance();
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();

    PartSlice result;
    result.gcode             = Test::gcode(print);
    result.compensated_loops = compensated_loops(*print.objects().front()->layers()[10]);
    return result;
}

bool is_wall_feature(const std::string &raw)
{
    if (raw.empty() || raw.front() != ';')
        return false;
    for (const std::string &tag : { std::string("TYPE:"), std::string("FEATURE: ") })
        if (const size_t at = raw.find(tag); at != std::string::npos)
            return raw.compare(at + tag.size(), 10, "Outer wall") == 0 || raw.compare(at + tag.size(), 10, "Inner wall") == 0;
    return false;
}

bool is_feature_line(const std::string &raw)
{
    return !raw.empty() && raw.front() == ';' && (raw.find("TYPE:") != std::string::npos || raw.find("FEATURE: ") != std::string::npos);
}

struct WallMove
{
    Vec2d end;
    int   feed{ 0 };
};

std::vector<WallMove> wall_moves(const std::string &gcode, double print_z)
{
    std::vector<WallMove> moves;
    bool                  in_wall = false;
    GCodeReader           reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string raw = line.raw();
        if (is_feature_line(raw))
            in_wall = is_wall_feature(raw);
        if (!in_wall || std::abs(self.z() - print_z) > 1e-3 || !line.extruding(self))
            return;
        moves.push_back({ Vec2d(line.has_x() ? line.x() : self.x(), line.has_y() ? line.y() : self.y()), int(std::lround(line.new_F(self))) });
    });
    return moves;
}

struct WallFeeds
{
    std::set<int> circle;
    std::set<int> other;
};

// Feed rates (mm/min) of the walls printed at print_z: the circle ones end within circle_reach of the middle of
// the part, the others (the outside of the block) further away.
WallFeeds wall_feeds(const std::string &gcode, double print_z, double circle_reach)
{
    const std::vector<WallMove> moves = wall_moves(gcode, print_z);
    REQUIRE_FALSE(moves.empty());
    Vec2d min_point = Vec2d::Constant(std::numeric_limits<double>::max());
    Vec2d max_point = Vec2d::Constant(std::numeric_limits<double>::lowest());
    for (const WallMove &move : moves) {
        min_point = min_point.cwiseMin(move.end);
        max_point = max_point.cwiseMax(move.end);
    }
    const Vec2d middle = (min_point + max_point) / 2.;
    WallFeeds   feeds;
    for (const WallMove &move : moves)
        ((move.end - middle).norm() <= circle_reach ? feeds.circle : feeds.other).insert(move.feed);
    return feeds;
}

const double circle_reach = circle_radius + 2.;

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

TEST_CASE("Circle compensation prints the walls of a pin and of a bore at their own speed", "[CircleCompensation]")
{
    const std::string wall_generator = GENERATE(std::string("classic"), std::string("arachne"));
    const CirclePart  part           = GENERATE(CirclePart::Pin, CirclePart::Hole);
    CAPTURE(wall_generator, part == CirclePart::Pin ? "pin" : "hole");

    const PartSlice enabled = slice_part(part, wall_generator, {});
    REQUIRE(enabled.compensated_loops == 2);
    const WallFeeds feeds = wall_feeds(enabled.gcode, middle_layer_z, circle_reach);
    REQUIRE(feeds.circle == std::set<int>{ 1020 });
    if (part == CirclePart::Hole)
        REQUIRE(feeds.other == std::set<int>{ 3600, 4800 });
    else
        REQUIRE(feeds.other.empty());
}

TEST_CASE("Circle compensation leaves the wall speeds alone when disabled or without its own speed", "[CircleCompensation]")
{
    const std::string wall_generator = GENERATE(std::string("classic"), std::string("arachne"));
    const CirclePart  part           = GENERATE(CirclePart::Pin, CirclePart::Hole);
    const std::string setting        = GENERATE(std::string("enable_circle_compensation"), std::string("circle_compensation_speed"));
    CAPTURE(wall_generator, part == CirclePart::Pin ? "pin" : "hole", setting);

    const PartSlice off   = slice_part(part, wall_generator, { { setting, "0" } });
    const WallFeeds feeds = wall_feeds(off.gcode, middle_layer_z, circle_reach);
    REQUIRE(feeds.circle == std::set<int>{ 3600, 4800 });
    if (part == CirclePart::Hole)
        REQUIRE(feeds.other == std::set<int>{ 3600, 4800 });
    if (setting == "enable_circle_compensation")
        REQUIRE(off.compensated_loops == 0);
}

TEST_CASE("Circle compensation keeps the small perimeter speed on small circles", "[CircleCompensation]")
{
    const std::string wall_generator = GENERATE(std::string("classic"), std::string("arachne"));
    const CirclePart  part           = GENERATE(CirclePart::Pin, CirclePart::Hole);
    CAPTURE(wall_generator, part == CirclePart::Pin ? "pin" : "hole");

    // Loops up to 2 * pi * 10mm long are small: the circles (about 25mm) are, the outside of the block (80mm) is not.
    const PartSlice small = slice_part(part, wall_generator, { { "small_perimeter_threshold", "10" }, { "small_perimeter_speed", "25" } });
    REQUIRE(small.compensated_loops == 2);
    const WallFeeds feeds = wall_feeds(small.gcode, middle_layer_z, circle_reach);
    REQUIRE(feeds.circle == std::set<int>{ 1500 });
    if (part == CirclePart::Hole)
        REQUIRE(feeds.other == std::set<int>{ 3600, 4800 });
}

TEST_CASE("Circle compensation keeps the first layer speed on the first layer", "[CircleCompensation]")
{
    const std::string wall_generator = GENERATE(std::string("classic"), std::string("arachne"));
    const CirclePart  part           = GENERATE(CirclePart::Pin, CirclePart::Hole);
    CAPTURE(wall_generator, part == CirclePart::Pin ? "pin" : "hole");

    const PartSlice enabled = slice_part(part, wall_generator, {});
    const WallFeeds feeds   = wall_feeds(enabled.gcode, first_layer_z, circle_reach);
    REQUIRE(feeds.circle == std::set<int>{ 1800 });
    if (part == CirclePart::Hole)
        REQUIRE(feeds.other == std::set<int>{ 1800 });
}
