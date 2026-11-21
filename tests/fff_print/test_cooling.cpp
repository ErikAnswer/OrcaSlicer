#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

// The fan is held off for the first close_fan_the_first_x_layers layers, so an explicit
// fan-off command is emitted.
TEST_CASE("Fan is held off for the initial layers", "[Cooling]")
{
    const std::string gcode = slice({ cube(20) }, {
        { "cooling",                      true },
        { "close_fan_the_first_x_layers", 5 },
    });
    CHECK(gcode.find("M106 S0") != std::string::npos);
}

// The cooling pass resolves and strips its internal speed placeholders; none leak into
// the final G-code.
TEST_CASE("Cooling consumes its internal speed markers", "[Cooling]")
{
    const std::string gcode = slice({ cube(20) }, { { "layer_height", 0.2 } });
    CHECK(gcode.find(";_EXTRUDE_SET_SPEED") == std::string::npos);
}

TEST_CASE("Arc fitting remains active with overhang fan control", "[Cooling][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_arc_fitting",             "enabled" },
        { "enable_overhang_bridge_fan",     true },
        { "enable_overhang_speed",          true },
        { "initial_layer_print_height",     0.2 },
        { "layer_height",                   0.2 },
        { "slow_down_for_layer_cooling",    false },
    });
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{100});
    config.set_key_value("overhang_fan_threshold", new ConfigOptionEnumsGeneric{Overhang_threshold_2_4});
    config.set_key_value("layer_change_gcode", new ConfigOptionString{";TEST_LAYER=[layer_num]"});

    const auto has_arc_after_first_layer = [](const std::string &gcode) {
        std::istringstream input(gcode);
        std::string line;
        size_t layers_seen = 0;
        while (std::getline(input, line)) {
            if (line.rfind(";TEST_LAYER=", 0) == 0)
                ++layers_seen;
            else if (layers_seen > 1 && (line.rfind("G2 ", 0) == 0 || line.rfind("G3 ", 0) == 0))
                return true;
        }
        return false;
    };
    const auto has_arc_with_overhang_fan = [](const std::string &gcode) {
        std::istringstream input(gcode);
        std::string line;
        bool overhang_fan_active = false;
        bool outer_wall = false;
        while (std::getline(input, line)) {
            if (line.rfind("M106 S255", 0) == 0)
                overhang_fan_active = true;
            else if (line.rfind("M106 S0", 0) == 0 || line.rfind("M107", 0) == 0)
                overhang_fan_active = false;
            else if (line.rfind(";TYPE:", 0) == 0)
                outer_wall = line == ";TYPE:Outer wall";
            else if (overhang_fan_active && outer_wall && (line.rfind("G2 ", 0) == 0 || line.rfind("G3 ", 0) == 0))
                return true;
        }
        return false;
    };

    const std::string with_overhang_speed = slice({ make_cylinder(25.0, 10.0, 2.0 * PI / 72.0) }, config);
    CHECK(has_arc_after_first_layer(with_overhang_speed));

    config.set_deserialize_strict({ { "enable_overhang_speed", false } });
    const std::string without_overhang_speed = slice({ make_cylinder(25.0, 10.0, 2.0 * PI / 72.0) }, config);
    CHECK(has_arc_after_first_layer(without_overhang_speed));

    const std::string uniform_overhang_gcode = slice({ make_sphere(50.0, 2.0 * PI / 72.0) }, config);
    CHECK(has_arc_with_overhang_fan(uniform_overhang_gcode));
}

TEST_CASE("Overhang fan transitions do not depend on overhang speed", "[Cooling][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "bridge_speed",                   2.0 },
        { "enable_arc_fitting",             "disabled" },
        { "enable_overhang_bridge_fan",     true },
        { "enable_overhang_speed",          false },
        { "initial_layer_print_height",     0.3 },
        { "inner_wall_speed",              30.0 },
        { "layer_height",                   0.3 },
        { "outer_wall_speed",              30.0 },
        { "overhang_1_4_speed",            "30" },
        { "overhang_2_4_speed",            "29" },
        { "overhang_3_4_speed",             "6" },
        { "overhang_4_4_speed",             "3" },
        { "slow_down_for_layer_cooling",    false },
        { "slowdown_for_curled_perimeters", false },
    });
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{100});
    config.set_key_value("overhang_fan_threshold", new ConfigOptionEnumsGeneric{Overhang_threshold_2_4});
    config.set_key_value("layer_change_gcode", new ConfigOptionString{";TEST_LAYER_Z=[layer_z]"});

    const auto fan_commands = [](const std::string &gcode) {
        std::vector<std::pair<std::string, std::string>> commands;
        std::istringstream input(gcode);
        std::string layer;
        std::string line;
        while (std::getline(input, line)) {
            if (line.rfind(";TEST_LAYER_Z=", 0) == 0)
                layer = line;
            else if (!layer.empty() && (line.rfind("M106", 0) == 0 || line.rfind("M107", 0) == 0))
                commands.emplace_back(layer, line);
        }
        return commands;
    };
    const auto feedrates = [](const std::string &gcode) {
        std::vector<std::string> values;
        std::istringstream input(gcode);
        std::string word;
        while (input >> word)
            if (!word.empty() && word.front() == 'F')
                values.push_back(word);
        return values;
    };

    constexpr double sphere_radius = 50.0; // 100 mm diameter.
    const std::string without_speed_gcode = slice({make_sphere(sphere_radius, PI / 24.0)}, config);
    config.set_deserialize_strict({{"enable_overhang_speed", true}});
    const std::string with_speed_gcode = slice({make_sphere(sphere_radius, PI / 24.0)}, config);

    const auto without_speed_fan = fan_commands(without_speed_gcode);
    const auto with_speed_fan = fan_commands(with_speed_gcode);
    const auto without_speed_feedrates = feedrates(without_speed_gcode);
    const auto with_speed_feedrates = feedrates(with_speed_gcode);

    REQUIRE_FALSE(without_speed_fan.empty());
    REQUIRE(std::any_of(without_speed_fan.begin(), without_speed_fan.end(),
                        [](const auto &command) { return command.second.find("S255") != std::string::npos; }));
    REQUIRE(with_speed_feedrates != without_speed_feedrates);
    CHECK(with_speed_fan == without_speed_fan);
}

namespace {

DynamicPrintConfig layer_time_bound_config(CoolingSlowdownLogicType logic)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_arc_fitting",          "disabled" },
        { "enable_overhang_speed",       false },
        { "initial_layer_print_height",  0.2 },
        { "layer_height",                0.2 },
        { "outer_wall_speed",            60.0 },
        { "inner_wall_speed",            120.0 },
        { "sparse_infill_speed",         200.0 },
        { "internal_solid_infill_speed", 200.0 },
        { "top_surface_speed",           200.0 },
        { "small_perimeter_threshold",   0.0 },
    });
    config.set_key_value("slow_down_for_layer_cooling", new ConfigOptionBools{true});
    config.set_key_value("dont_slow_down_outer_wall", new ConfigOptionBools{false});
    config.set_key_value("slow_down_layer_time", new ConfigOptionFloats{12.0});
    config.set_key_value("slow_down_min_speed", new ConfigOptionFloats{5.0});
    config.set_key_value("cooling_slowdown_logic", new ConfigOptionEnumsGeneric{(int)logic});
    config.set_key_value("layer_change_gcode", new ConfigOptionString{";TEST_LAYER=[layer_num]"});
    return config;
}

struct FeatureFeedrates
{
    std::vector<double> outer_wall;
    std::vector<double> other;
};

FeatureFeedrates extrusion_feedrates_in_layers(const std::string &gcode, size_t first_layer, size_t last_layer)
{
    FeatureFeedrates out;
    std::istringstream input(gcode);
    std::string line;
    size_t layers_seen = 0;
    bool outer_wall = false;
    double feedrate = 0.;
    while (std::getline(input, line)) {
        if (line.rfind(";TEST_LAYER=", 0) == 0) {
            ++layers_seen;
            continue;
        }
        if (line.rfind(";TYPE:", 0) == 0) {
            outer_wall = line == ";TYPE:Outer wall";
            continue;
        }
        if (line.rfind("G1 ", 0) != 0)
            continue;
        std::istringstream words(line.substr(0, line.find(';')));
        std::string word;
        bool extrudes = false;
        bool moves_xy = false;
        while (words >> word) {
            if (word.front() == 'F')
                feedrate = std::stod(word.substr(1));
            else if (word.front() == 'E' && std::stod(word.substr(1)) > 0.)
                extrudes = true;
            else if (word.front() == 'X' || word.front() == 'Y')
                moves_xy = true;
        }
        if (extrudes && moves_xy && layers_seen >= first_layer && layers_seen <= last_layer)
            (outer_wall ? out.outer_wall : out.other).push_back(feedrate);
    }
    return out;
}

FeatureFeedrates extrusion_feedrates_after_first_layers(const std::string &gcode)
{
    return extrusion_feedrates_in_layers(gcode, 3, std::numeric_limits<size_t>::max());
}

struct ExtrusionMove
{
    size_t      layer;
    std::string type;
    double      feedrate;
};

std::vector<ExtrusionMove> extrusion_moves(const std::string &gcode)
{
    std::vector<ExtrusionMove> out;
    std::istringstream input(gcode);
    std::string line;
    size_t layers_seen = 0;
    std::string type;
    double feedrate = 0.;
    while (std::getline(input, line)) {
        if (line.rfind(";TEST_LAYER=", 0) == 0) {
            ++layers_seen;
            continue;
        }
        if (line.rfind(";TYPE:", 0) == 0) {
            type = line.substr(6);
            continue;
        }
        if (line.rfind("G1 ", 0) != 0)
            continue;
        std::istringstream words(line.substr(0, line.find(';')));
        std::string word;
        bool extrudes = false;
        bool moves_xy = false;
        while (words >> word) {
            if (word.front() == 'F')
                feedrate = std::stod(word.substr(1));
            else if (word.front() == 'E' && std::stod(word.substr(1)) > 0.)
                extrudes = true;
            else if (word.front() == 'X' || word.front() == 'Y')
                moves_xy = true;
        }
        if (extrudes && moves_xy)
            out.push_back({ layers_seen, type, feedrate });
    }
    return out;
}

} // namespace

TEST_CASE("Consistent surface slowdown keeps the outer wall speed", "[Cooling]")
{
    const double outer_wall_feedrate = 60.0 * 60.0;
    const auto max_of = [](const std::vector<double> &values) { return *std::max_element(values.begin(), values.end()); };

    const FeatureFeedrates uniform = extrusion_feedrates_after_first_layers(
        slice({ cube(10) }, layer_time_bound_config(cslUniformCooling)));
    const FeatureFeedrates consistent = extrusion_feedrates_after_first_layers(
        slice({ cube(10) }, layer_time_bound_config(cslConsistentSurface)));

    REQUIRE_FALSE(uniform.outer_wall.empty());
    REQUIRE_FALSE(consistent.outer_wall.empty());
    REQUIRE_FALSE(consistent.other.empty());

    CHECK(std::any_of(uniform.outer_wall.begin(), uniform.outer_wall.end(),
                      [&](double f) { return f < outer_wall_feedrate - 1.; }));
    CHECK(std::all_of(consistent.outer_wall.begin(), consistent.outer_wall.end(),
                      [&](double f) { return std::abs(f - outer_wall_feedrate) < 1.; }));
    CHECK(max_of(consistent.other) < max_of(uniform.other) + 1.);
    CHECK(max_of(consistent.other) < 120.0 * 60.0 - 1.);
}

TEST_CASE("Consistent surface slows the outer wall once nothing else is left", "[Cooling]")
{
    // Hollow 10 mm cube with two walls: a middle layer is about 40 mm of outer wall and 38 mm of inner wall.
    // At the 10 mm/s minimum, the inner wall alone gives about 4.5 s with the outer wall at speed and the
    // whole layer about 7.8 s, so a 6 s threshold needs the second pass: the inner wall goes to its minimum
    // and the outer wall is slowed down, but not to its minimum.
    DynamicPrintConfig config = layer_time_bound_config(cslConsistentSurface);
    config.set_deserialize_strict({
        { "wall_loops",            2 },
        { "sparse_infill_density", "0%" },
    });
    config.set_key_value("slow_down_layer_time", new ConfigOptionFloats{6.0});
    config.set_key_value("slow_down_min_speed", new ConfigOptionFloats{10.0});
    const double min_feedrate        = 10.0 * 60.0;
    const double outer_wall_feedrate = 60.0 * 60.0;

    const FeatureFeedrates middle = extrusion_feedrates_in_layers(slice({ cube(10) }, config), 15, 35);

    REQUIRE_FALSE(middle.outer_wall.empty());
    REQUIRE_FALSE(middle.other.empty());
    CHECK(std::all_of(middle.other.begin(), middle.other.end(),
                      [&](double f) { return std::abs(f - min_feedrate) < 1.; }));
    CHECK(std::all_of(middle.outer_wall.begin(), middle.outer_wall.end(),
                      [&](double f) { return f > min_feedrate + 1. && f < outer_wall_feedrate - 1.; }));
}

TEST_CASE("Consistent surface keeps the overhangs of the outer wall at their speed", "[Cooling]")
{
    // Inverted cone, 6 mm radius and 4 mm high, with a single wall: each layer reaches about 0.3 mm past the one
    // below, so the outer wall is split into outer wall and overhang wall paths. In every layer where the consistent
    // surface logic keeps the outer wall at its speed, its overhang paths must keep theirs too, also when the
    // pressure equalizer rewrites their feedrates.
    const double extrusion_rate_slope = GENERATE(0., 10.);
    CAPTURE(extrusion_rate_slope);
    TriangleMesh cone = make_cone(6.0, 4.0);
    cone.mirror_z();
    DynamicPrintConfig config = layer_time_bound_config(cslConsistentSurface);
    config.set_deserialize_strict({
        { "wall_loops",                          1 },
        { "max_volumetric_extrusion_rate_slope", extrusion_rate_slope },
    });

    DynamicPrintConfig reference_config = config;
    reference_config.set_key_value("slow_down_for_layer_cooling", new ConfigOptionBools{false});

    const std::string consistent_gcode = slice({ cone }, config);
    const std::vector<ExtrusionMove> consistent = extrusion_moves(consistent_gcode);
    const std::vector<ExtrusionMove> reference  = extrusion_moves(slice({ cone }, reference_config));
    REQUIRE(consistent.size() == reference.size());
    CHECK(consistent_gcode.find(";_OUTER_WALL_OVERHANG") == std::string::npos);

    const auto same_speed = [](double lhs, double rhs) { return std::abs(lhs - rhs) < 1.; };
    size_t layers_checked = 0;
    for (size_t begin = 0; begin < consistent.size();) {
        size_t end = begin;
        while (end < consistent.size() && consistent[end].layer == consistent[begin].layer)
            ++end;
        bool outer_wall_held = true;
        bool slowed_down     = false;
        bool has_overhang    = false;
        for (size_t i = begin; i < end; ++i) {
            const bool kept = same_speed(consistent[i].feedrate, reference[i].feedrate);
            slowed_down |= !kept;
            if (consistent[i].type == "Outer wall")
                outer_wall_held &= kept;
            else if (consistent[i].type == "Overhang wall")
                has_overhang = true;
        }
        if (outer_wall_held && slowed_down && has_overhang) {
            ++layers_checked;
            for (size_t i = begin; i < end; ++i)
                if (consistent[i].type == "Overhang wall")
                    CHECK(same_speed(consistent[i].feedrate, reference[i].feedrate));
        }
        begin = end;
    }
    CHECK(layers_checked > 0);
}

namespace {

// Filament 1 prints a 10 mm cube at X 40..50 and filament 2 another one at X 80..90, both at Y 60..70.
constexpr double second_cube_min_x = 65.;

DynamicPrintConfig two_filament_layer_time_config(double slow_down_layer_time, double second_min_speed)
{
    DynamicPrintConfig config = multifilament_config(2, {
        { "enable_prime_tower",          "0" },
        { "use_relative_e_distances",    true },
        { "enable_arc_fitting",          "disabled" },
        { "enable_overhang_speed",       false },
        { "initial_layer_print_height",  0.2 },
        { "layer_height",                0.2 },
        { "outer_wall_speed",            60.0 },
        { "inner_wall_speed",            120.0 },
        { "sparse_infill_speed",         200.0 },
        { "internal_solid_infill_speed", 200.0 },
        { "top_surface_speed",           200.0 },
        { "small_perimeter_threshold",   0.0 },
    });
    config.set_key_value("slow_down_for_layer_cooling", new ConfigOptionBools{true, true});
    config.set_key_value("dont_slow_down_outer_wall", new ConfigOptionBools{false, false});
    config.set_key_value("slow_down_layer_time", new ConfigOptionFloats{slow_down_layer_time, slow_down_layer_time});
    config.set_key_value("slow_down_min_speed", new ConfigOptionFloats{10.0, second_min_speed});
    config.set_key_value("cooling_slowdown_logic",
                         new ConfigOptionEnumsGeneric{(int)cslUniformCooling, (int)cslConsistentSurface});
    config.set_key_value("layer_change_gcode", new ConfigOptionString{";TEST_LAYER=[layer_num]"});
    return config;
}

std::string slice_two_filament_cubes(const DynamicPrintConfig &config,
                                     const std::vector<std::vector<ConfigBase::SetDeserializeItem>> &overrides)
{
    std::vector<TriangleMesh> meshes;
    for (const float x : { 40.f, 80.f }) {
        TriangleMesh mesh = cube(10);
        mesh.translate(Vec3f(x, 60.f, 0.f));
        meshes.push_back(std::move(mesh));
    }
    Print print;
    Model model;
    init_print(std::move(meshes), print, model, config, &overrides, false);
    return gcode(print);
}

struct FilamentFeedrates
{
    std::vector<double> outer_wall;
    std::vector<double> other;
};

struct TwoFilamentLayers
{
    FilamentFeedrates   first;
    FilamentFeedrates   second;
    std::vector<double> layer_times;
};

// Extrusion feedrates of layers first_layer..last_layer, split by cube, and the time of each of those layers as
// length over feedrate of every move, the model the cooling buffer stretches without accelerations.
TwoFilamentLayers two_filament_layers(const std::string &gcode, size_t first_layer, size_t last_layer)
{
    TwoFilamentLayers out;
    std::istringstream input(gcode);
    std::string line;
    size_t layers_seen = 0;
    bool outer_wall = false;
    double feedrate = 0.;
    double x = 0.;
    double y = 0.;
    double z = 0.;
    const auto in_range = [&]() { return layers_seen >= first_layer && layers_seen <= last_layer; };
    while (std::getline(input, line)) {
        if (line.rfind(";TEST_LAYER=", 0) == 0) {
            ++layers_seen;
            if (in_range())
                out.layer_times.push_back(0.);
            continue;
        }
        if (line.rfind(";TYPE:", 0) == 0) {
            outer_wall = line == ";TYPE:Outer wall";
            continue;
        }
        if (line.rfind("G1 ", 0) != 0 && line.rfind("G0 ", 0) != 0)
            continue;
        std::istringstream words(line.substr(0, line.find(';')));
        std::string word;
        words >> word;
        double next_x = x;
        double next_y = y;
        double next_z = z;
        double e = 0.;
        while (words >> word) {
            const double value = std::stod(word.substr(1));
            switch (word.front()) {
            case 'F': feedrate = value; break;
            case 'X': next_x = value; break;
            case 'Y': next_y = value; break;
            case 'Z': next_z = value; break;
            case 'E': e = value; break;
            default: break;
            }
        }
        const double xy_length = std::hypot(next_x - x, next_y - y);
        const double length    = std::hypot(xy_length, next_z - z);
        if (in_range() && feedrate > 0.) {
            out.layer_times.back() += (length > 0. ? length : std::abs(e)) / (feedrate / 60.);
            if (e > 0. && xy_length > 0.) {
                FilamentFeedrates &filament = next_x < second_cube_min_x ? out.first : out.second;
                (outer_wall ? filament.outer_wall : filament.other).push_back(feedrate);
            }
        }
        x = next_x;
        y = next_y;
        z = next_z;
    }
    return out;
}

double max_of(const std::vector<double> &values) { return *std::max_element(values.begin(), values.end()); }
double min_of(const std::vector<double> &values) { return *std::min_element(values.begin(), values.end()); }

} // namespace

TEST_CASE("Consistent surface with no minimum speed and only held outer walls lets the other filaments slow down", "[Cooling]")
{
    // Filament 1 (uniform, 10 mm/s minimum) prints a cube with two walls and sparse infill, so its lines run at
    // 200, 120 and 60 mm/s; filament 2 (consistent surface, no minimum speed) prints a cube with a single wall and
    // no infill, so a middle layer only has its held outer wall. Both at speed make a layer of about 2 s, well
    // under the 6 s threshold, which filament 1 can reach alone at about 20 mm/s. The slow down of filament 1 goes
    // step by step through its feedrates and meets filament 2 with nothing to stretch on every step: stopping
    // there leaves filament 1 half slowed down, and the next pass, with filament 2 alone, fills the layer by
    // slowing down its outer wall.
    const double slow_down_layer_time = 6.0;
    const DynamicPrintConfig config = two_filament_layer_time_config(slow_down_layer_time, 0.0);
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{
        { { "extruder", "1" }, { "wall_loops", 2 }, { "sparse_infill_density", "20%" } },
        { { "extruder", "2" }, { "wall_loops", 1 }, { "sparse_infill_density", "0%" } },
    };
    const double min_feedrate        = 10.0 * 60.0;
    const double outer_wall_feedrate = 60.0 * 60.0;

    const TwoFilamentLayers middle = two_filament_layers(slice_two_filament_cubes(config, overrides), 15, 35);

    REQUIRE_FALSE(middle.first.outer_wall.empty());
    REQUIRE_FALSE(middle.first.other.empty());
    REQUIRE_FALSE(middle.second.outer_wall.empty());
    CHECK(middle.second.other.empty());
    for (const double feedrate : middle.second.outer_wall)
        CHECK_THAT(feedrate, Catch::Matchers::WithinRel(outer_wall_feedrate, 0.01));
    CHECK(max_of(middle.first.other) < outer_wall_feedrate - 1.);
    CHECK(max_of(middle.first.outer_wall) < outer_wall_feedrate - 1.);
    CHECK(min_of(middle.first.other) > min_feedrate + 1.);
    for (const double layer_time : middle.layer_times)
        CHECK(layer_time > 0.9 * slow_down_layer_time);
}

TEST_CASE("Mixed uniform and consistent surface filaments slow the held outer wall only once the rest is at its minimum", "[Cooling]")
{
    // Two hollow 10 mm cubes with two walls, filament 1 uniform and filament 2 consistent surface, both with a
    // 10 mm/s minimum. A middle layer is about 40 mm of outer wall and 35 mm of inner wall per cube: about 2 s at
    // speed, about 11.5 s with everything but the outer wall of filament 2 at the minimum and about 15 s with
    // everything at the minimum.
    const DynamicPrintConfig base = two_filament_layer_time_config(0.0, 10.0);
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{
        { { "extruder", "1" }, { "wall_loops", 2 }, { "sparse_infill_density", "0%" } },
        { { "extruder", "2" }, { "wall_loops", 2 }, { "sparse_infill_density", "0%" } },
    };
    const double min_feedrate        = 10.0 * 60.0;
    const double outer_wall_feedrate = 60.0 * 60.0;
    const auto slice_middle_layers = [&](double slow_down_layer_time) {
        DynamicPrintConfig config = base;
        config.set_key_value("slow_down_layer_time", new ConfigOptionFloats{slow_down_layer_time, slow_down_layer_time});
        return two_filament_layers(slice_two_filament_cubes(config, overrides), 15, 35);
    };
    const auto require_moves = [](const TwoFilamentLayers &layers) {
        REQUIRE_FALSE(layers.first.outer_wall.empty());
        REQUIRE_FALSE(layers.first.other.empty());
        REQUIRE_FALSE(layers.second.outer_wall.empty());
        REQUIRE_FALSE(layers.second.other.empty());
    };

    SECTION("the other features absorb the whole slow down")
    {
        const double slow_down_layer_time = 8.0;
        const TwoFilamentLayers middle = slice_middle_layers(slow_down_layer_time);
        require_moves(middle);
        for (const double feedrate : middle.second.outer_wall)
            CHECK_THAT(feedrate, Catch::Matchers::WithinRel(outer_wall_feedrate, 0.01));
        CHECK(max_of(middle.first.outer_wall) < outer_wall_feedrate - 1.);
        CHECK(min_of(middle.first.outer_wall) > min_feedrate + 1.);
        CHECK(min_of(middle.first.other) > min_feedrate + 1.);
        CHECK(min_of(middle.second.other) > min_feedrate + 1.);
        for (const double layer_time : middle.layer_times)
            CHECK(layer_time > 0.9 * slow_down_layer_time);
    }

    SECTION("the held outer wall takes what the other features cannot")
    {
        const double slow_down_layer_time = 13.5;
        const TwoFilamentLayers middle = slice_middle_layers(slow_down_layer_time);
        require_moves(middle);
        for (const std::vector<double> *feedrates : { &middle.first.outer_wall, &middle.first.other, &middle.second.other })
            for (const double feedrate : *feedrates)
                CHECK_THAT(feedrate, Catch::Matchers::WithinRel(min_feedrate, 0.01));
        CHECK(min_of(middle.second.outer_wall) > min_feedrate + 1.);
        CHECK(max_of(middle.second.outer_wall) < outer_wall_feedrate - 1.);
        for (const double layer_time : middle.layer_times)
            CHECK(layer_time > 0.9 * slow_down_layer_time);
    }
}

TEST_CASE("Short travel acceleration next to outer walls", "[Cooling][GCode]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "gcode_flavor",         "klipper" },
        { "layer_height",         0.2 },
        { "wall_sequence",        "inner wall/outer wall" },
    });
    config.set_key_value("default_acceleration", new ConfigOptionFloatsNullable{5000.0});
    config.set_key_value("outer_wall_acceleration", new ConfigOptionFloatsNullable{3000.0});
    config.set_key_value("travel_acceleration", new ConfigOptionFloatsNullable{6000.0});

    config.set_key_value("travel_short_distance_acceleration", new ConfigOptionFloatsNullable{0.0});
    const std::string disabled = slice({ cube(20) }, config);
    config.set_key_value("travel_short_distance_acceleration", new ConfigOptionFloatsNullable{1234.0});
    const std::string enabled = slice({ cube(20) }, config);

    CHECK(disabled.find("ACCEL=1234") == std::string::npos);
    CHECK(enabled.find("ACCEL=1234") != std::string::npos);
}

struct TravelMoves
{
    std::vector<double> feedrates;
    std::vector<double> accelerations;
};

TravelMoves travels_in_layers(const std::string &gcode, size_t first_layer, size_t last_layer)
{
    TravelMoves out;
    std::istringstream input(gcode);
    std::string line;
    size_t layers_seen = 0;
    double feedrate = 0.;
    double acceleration = 0.;
    const std::string acceleration_command = "SET_VELOCITY_LIMIT ACCEL=";
    while (std::getline(input, line)) {
        if (line.rfind(";TEST_LAYER=", 0) == 0) {
            ++layers_seen;
            continue;
        }
        if (line.rfind(acceleration_command, 0) == 0) {
            acceleration = std::stod(line.substr(acceleration_command.size()));
            continue;
        }
        if (line.rfind("G1 ", 0) != 0)
            continue;
        std::istringstream words(line.substr(0, line.find(';')));
        std::string word;
        bool has_e = false;
        bool moves_xy = false;
        while (words >> word) {
            if (word.front() == 'F')
                feedrate = std::stod(word.substr(1));
            else if (word.front() == 'E')
                has_e = true;
            else if (word.front() == 'X' || word.front() == 'Y')
                moves_xy = true;
        }
        if (!has_e && moves_xy && layers_seen >= first_layer && layers_seen <= last_layer) {
            out.feedrates.push_back(feedrate);
            out.accelerations.push_back(acceleration);
        }
    }
    return out;
}

TEST_CASE("Slow down by height limits speed and acceleration above the starting height", "[Speed][GCode]")
{
    const double layer_height          = 0.2;
    const double slowdown_start_height = 5.0;
    const double slowdown_end_height   = 10.0;
    const double slowdown_start_speed  = 100.0;
    const double slowdown_end_speed    = 20.0;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "gcode_flavor",                "klipper" },
        { "initial_layer_print_height",  layer_height },
        { "layer_height",                layer_height },
        { "slow_down_for_layer_cooling", false },
    });
    config.set_key_value("layer_change_gcode", new ConfigOptionString{";TEST_LAYER=[layer_num]"});
    config.set_key_value("default_acceleration", new ConfigOptionFloatsNullable{5000.0});
    config.set_key_value("outer_wall_acceleration", new ConfigOptionFloatsNullable{3000.0});
    config.set_key_value("travel_acceleration", new ConfigOptionFloatsNullable{6000.0});
    config.set_key_value("travel_speed", new ConfigOptionFloatsNullable{200.0});
    config.set_key_value("wipe", new ConfigOptionBools{false});
    config.set_key_value("slowdown_start_height", new ConfigOptionFloatsNullable{slowdown_start_height});
    config.set_key_value("slowdown_end_height", new ConfigOptionFloatsNullable{slowdown_end_height});
    config.set_key_value("slowdown_start_speed", new ConfigOptionFloatsNullable{slowdown_start_speed});
    config.set_key_value("slowdown_end_speed", new ConfigOptionFloatsNullable{slowdown_end_speed});
    config.set_key_value("slowdown_start_acc", new ConfigOptionFloatsNullable{100000.0});
    config.set_key_value("slowdown_end_acc", new ConfigOptionFloatsNullable{1234.0});
    const double end_feedrate = slowdown_end_speed * 60.0;
    const auto   all_feedrates = [](const FeatureFeedrates &feedrates) {
        std::vector<double> all = feedrates.outer_wall;
        all.insert(all.end(), feedrates.other.begin(), feedrates.other.end());
        return all;
    };

    config.set_key_value("enable_height_slowdown", new ConfigOptionBoolsNullable{false});
    const std::string disabled = slice({ cube(20) }, config);
    config.set_key_value("enable_height_slowdown", new ConfigOptionBoolsNullable{true});
    const std::string enabled = slice({ cube(20) }, config);

    const std::vector<double> disabled_top = all_feedrates(extrusion_feedrates_in_layers(disabled, 60, 90));
    const std::vector<double> enabled_top  = all_feedrates(extrusion_feedrates_in_layers(enabled, 60, 90));
    const std::vector<double> enabled_low  = all_feedrates(extrusion_feedrates_in_layers(enabled, 5, 20));

    REQUIRE_FALSE(enabled_top.empty());
    CHECK(std::any_of(disabled_top.begin(), disabled_top.end(), [&](double f) { return f > end_feedrate + 1.; }));
    CHECK(std::all_of(enabled_top.begin(), enabled_top.end(), [&](double f) { return f < end_feedrate + 1.; }));
    CHECK(std::any_of(enabled_low.begin(), enabled_low.end(), [&](double f) { return f > end_feedrate + 1.; }));
    CHECK(all_feedrates(extrusion_feedrates_in_layers(disabled, 5, 20)) == enabled_low);
    CHECK(disabled.find("ACCEL=1234") == std::string::npos);
    CHECK(enabled.find("ACCEL=1234") != std::string::npos);

    const TravelMoves disabled_top_travels = travels_in_layers(disabled, 60, 90);
    const TravelMoves enabled_top_travels  = travels_in_layers(enabled, 60, 90);
    REQUIRE_FALSE(enabled_top_travels.feedrates.empty());
    CHECK(std::any_of(disabled_top_travels.feedrates.begin(), disabled_top_travels.feedrates.end(),
                      [&](double f) { return f > end_feedrate + 1.; }));
    CHECK(std::all_of(enabled_top_travels.feedrates.begin(), enabled_top_travels.feedrates.end(),
                      [&](double f) { return f < end_feedrate + 1.; }));
    CHECK(std::any_of(disabled_top_travels.accelerations.begin(), disabled_top_travels.accelerations.end(),
                      [](double a) { return a > 1234.5; }));
    CHECK(std::all_of(enabled_top_travels.accelerations.begin(), enabled_top_travels.accelerations.end(),
                      [](double a) { return a < 1234.5; }));
    CHECK(travels_in_layers(disabled, 5, 20).feedrates == travels_in_layers(enabled, 5, 20).feedrates);

    const size_t first_ramp_layer   = 26;
    const size_t last_ramp_layer    = 49;
    const double feedrate_tolerance = 0.5 * 60.0;
    const auto   ramp_feedrate      = [&](size_t layer) {
        const double ratio = (double(layer) * layer_height - slowdown_start_height) / (slowdown_end_height - slowdown_start_height);
        return (slowdown_start_speed + (slowdown_end_speed - slowdown_start_speed) * ratio) * 60.0;
    };
    for (size_t layer = first_ramp_layer; layer <= last_ramp_layer; ++layer) {
        CAPTURE(layer);
        const std::vector<double> ramp_extrusions = all_feedrates(extrusion_feedrates_in_layers(enabled, layer, layer));
        const std::vector<double> ramp_travels    = travels_in_layers(enabled, layer, layer).feedrates;
        REQUIRE_FALSE(ramp_extrusions.empty());
        REQUIRE_FALSE(ramp_travels.empty());
        CHECK_THAT(*std::max_element(ramp_extrusions.begin(), ramp_extrusions.end()),
                   Catch::Matchers::WithinAbs(ramp_feedrate(layer), feedrate_tolerance));
        CHECK_THAT(*std::max_element(ramp_travels.begin(), ramp_travels.end()),
                   Catch::Matchers::WithinAbs(ramp_feedrate(layer), feedrate_tolerance));
    }
}
