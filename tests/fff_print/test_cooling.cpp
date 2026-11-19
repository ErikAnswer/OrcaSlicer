#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
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
        while (words >> word) {
            if (word.front() == 'F')
                feedrate = std::stod(word.substr(1));
            else if (word.front() == 'E' && std::stod(word.substr(1)) > 0.)
                extrudes = true;
        }
        if (extrudes && layers_seen >= first_layer && layers_seen <= last_layer)
            (outer_wall ? out.outer_wall : out.other).push_back(feedrate);
    }
    return out;
}

FeatureFeedrates extrusion_feedrates_after_first_layers(const std::string &gcode)
{
    return extrusion_feedrates_in_layers(gcode, 3, std::numeric_limits<size_t>::max());
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
