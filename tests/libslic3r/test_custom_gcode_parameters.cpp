#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <string>
#include <vector>

#include "libslic3r/Config.hpp"
#include "libslic3r/CustomGCodeParameters.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/PlaceholderParser.hpp"
#include "libslic3r/PrintConfig.hpp"

using namespace Slic3r;
using Catch::Matchers::ContainsSubstring;

namespace {

PlaceholderParser parser_with(const std::string &print, const std::string &printer, const std::vector<std::string> &filaments)
{
    PlaceholderParser parser;
    CustomGCodeParameters::apply(parser.config_writable(), print, printer, filaments, filaments.size());
    return parser;
}

const std::vector<std::string> two_filaments = { R"({"chamber": 45, "soak": true, "fan": 30})", R"({"chamber": 60.5, "soak": false})" };

} // namespace

TEST_CASE("Custom parameters of the process and the printer are scalar placeholders", "[CustomGCodeParameters]")
{
    const PlaceholderParser parser = parser_with(R"({"purge_lines": 2, "ratio": 0.25, "slow_start": true})", R"({"park_x": 150})", { "" });

    CHECK(parser.process("{custom_parameter_print_purge_lines}") == "2");
    CHECK(parser.process("{custom_parameter_print_purge_lines + 1}") == "3");
    CHECK(parser.process("{custom_parameter_print_ratio}") == "0.25");
    CHECK(parser.process("{custom_parameter_print_ratio * 4}") == "1");
    CHECK(parser.process("{if custom_parameter_print_slow_start}yes{else}no{endif}") == "yes");
    CHECK(parser.process("G1 X{custom_parameter_printer_park_x}") == "G1 X150");
    CHECK(parser.process("G1 X[custom_parameter_printer_park_x]") == "G1 X150");
}

TEST_CASE("A custom filament parameter without an index reads the filament in use", "[CustomGCodeParameters]")
{
    const PlaceholderParser parser = parser_with("", "", two_filaments);

    CHECK(parser.process("M191 S{custom_parameter_filament_chamber}", 0) == "M191 S45");
    CHECK(parser.process("M191 S{custom_parameter_filament_chamber}", 1) == "M191 S60.5");
    CHECK(parser.process("M191 S[custom_parameter_filament_chamber]", 1) == "M191 S60.5");
    CHECK(parser.process("{custom_parameter_filament_chamber[0]}", 1) == "45");
    CHECK(parser.process("{custom_parameter_filament_chamber[1] - custom_parameter_filament_chamber[0]}", 0) == "15.5");
    CHECK(parser.process("{if custom_parameter_filament_soak}soak{else}skip{endif}", 0) == "soak");
    CHECK(parser.process("{if custom_parameter_filament_soak}soak{else}skip{endif}", 1) == "skip");
}

TEST_CASE("A custom parameter that is not defined stops the expansion with its own message", "[CustomGCodeParameters]")
{
    const PlaceholderParser parser = parser_with("", "", two_filaments);

    CHECK(parser.process("{custom_parameter_filament_fan}", 0) == "30");
    CHECK_THROWS_WITH(parser.process("{custom_parameter_filament_fan}", 1), ContainsSubstring("not defined in the filament"));
    CHECK_THROWS_WITH(parser.process("[custom_parameter_filament_fan]", 1), ContainsSubstring("not defined in the filament"));
    CHECK_THROWS_WITH(parser.process("{custom_parameter_filament_fan[1]}", 0), ContainsSubstring("not defined in the filament"));
    CHECK_THROWS_WITH(parser.process("{custom_parameter_filament_chamber[2]}", 0), ContainsSubstring("not defined in the filament"));
    CHECK_THROWS_WITH(parser.process("{custom_parameter_filament_chamber}", 2), ContainsSubstring("not defined in the filament"));
    CHECK_THROWS_WITH(parser.process("[custom_parameter_filament_chamber]", 2), ContainsSubstring("not defined in the filament"));
    CHECK_THROWS_WITH(parser.process("[custom_parameter_filament_chamber_2]", 0), ContainsSubstring("not defined in the filament"));
    CHECK_THROWS_WITH(parser.process("{custom_parameter_filament_chamber[-1]}", 0), ContainsSubstring("not defined in the filament"));
}

TEST_CASE("The legacy indexed syntax reads custom filament parameters with the same errors", "[CustomGCodeParameters]")
{
    PlaceholderParser parser = parser_with("", "", two_filaments);
    parser.set("first", 0);
    parser.set("second", 1);
    parser.set("third", 2);

    CHECK(parser.process("[custom_parameter_filament_chamber[second]]", 0) == "60.5");
    CHECK(parser.process("[custom_parameter_filament_fan[first]]", 1) == "30");
    CHECK_THROWS_WITH(parser.process("[custom_parameter_filament_fan[second]]", 0), ContainsSubstring("not defined in the filament"));
    CHECK_THROWS_WITH(parser.process("[custom_parameter_filament_chamber[third]]", 0), ContainsSubstring("not defined in the filament"));
    CHECK_THROWS_WITH(parser.process("[custom_parameter_filament_missing[first]]", 0), ContainsSubstring("custom parameter is not defined"));
}

TEST_CASE("is_nil tells whether a filament defines a custom parameter", "[CustomGCodeParameters]")
{
    const PlaceholderParser parser = parser_with("", "", two_filaments);

    CHECK(parser.process("{if is_nil(custom_parameter_filament_fan)}none{else}{custom_parameter_filament_fan}{endif}", 0) == "30");
    CHECK(parser.process("{if is_nil(custom_parameter_filament_fan)}none{else}{custom_parameter_filament_fan}{endif}", 1) == "none");
    CHECK(parser.process("{if is_nil(custom_parameter_filament_fan[1])}none{else}set{endif}", 0) == "none");
    CHECK(parser.process("{if is_nil(custom_parameter_filament_chamber[1])}none{else}set{endif}", 0) == "set");
    CHECK(parser.process("{if is_nil(custom_parameter_filament_chamber[2])}none{else}set{endif}", 0) == "none");
    CHECK_THROWS_WITH(parser.process("{custom_parameter_filament_missing}", 0), ContainsSubstring("custom parameter is not defined"));
    CHECK_THROWS_WITH(parser.process("[custom_parameter_print_missing]", 0), ContainsSubstring("custom parameter is not defined"));
    CHECK_THROWS_WITH(parser.process("{custom_parameter_printer_missing + 1}", 0), ContainsSubstring("custom parameter is not defined"));
}

TEST_CASE("Custom parameters that are not a flat object of numbers and booleans are refused", "[CustomGCodeParameters]")
{
    const std::vector<std::string> no_filaments;

    CHECK(CustomGCodeParameters::validate("", "  \n", { "", "" }).empty());
    CHECK(CustomGCodeParameters::validate(R"({"a": 1})", R"({"b": -2.5})", { R"({"c": true})" }).empty());

    CHECK_THAT(CustomGCodeParameters::validate("{\"a\": 1", "", no_filaments), ContainsSubstring("the process"));
    CHECK_THAT(CustomGCodeParameters::validate("", "[1, 2]", no_filaments), ContainsSubstring("the printer"));
    CHECK_THAT(CustomGCodeParameters::validate("", "", { "", "a = 1" }), ContainsSubstring("filament 2"));
    CHECK_THAT(CustomGCodeParameters::validate(R"({"gcode": "M104 S300"})", "", no_filaments), ContainsSubstring("\"gcode\""));
    CHECK_THAT(CustomGCodeParameters::validate(R"({"nested": {"a": 1}})", "", no_filaments), ContainsSubstring("\"nested\""));
    CHECK_THAT(CustomGCodeParameters::validate(R"({"list": [1]})", "", no_filaments), ContainsSubstring("\"list\""));
    CHECK_THAT(CustomGCodeParameters::validate(R"({"nothing": null})", "", no_filaments), ContainsSubstring("\"nothing\""));
    CHECK_THAT(CustomGCodeParameters::validate(R"({"2fast": 1})", "", no_filaments), ContainsSubstring("\"2fast\""));
    CHECK_THAT(CustomGCodeParameters::validate(R"({"with space": 1})", "", no_filaments), ContainsSubstring("\"with space\""));
    CHECK_THAT(CustomGCodeParameters::validate(R"({"": 1})", "", no_filaments), ContainsSubstring("\"\""));
    CHECK_THAT(CustomGCodeParameters::validate("", "", { R"({"soak": true})", R"({"soak": 10})" }), ContainsSubstring("\"soak\""));

    PlaceholderParser parser;
    CHECK_THROWS_AS(CustomGCodeParameters::apply(parser.config_writable(), R"({"gcode": "M104 S300"})", "", no_filaments, 1),
                    Slic3r::PlaceholderParserError);
    CHECK(parser.config().empty());
}

TEST_CASE("Integer and real values of one filament parameter share a real vector", "[CustomGCodeParameters]")
{
    const PlaceholderParser parser = parser_with("", "", { R"({"wait": 10})", R"({"wait": 12.5})", "" });

    CHECK(parser.process("{custom_parameter_filament_wait}", 0) == "10");
    CHECK(parser.process("{custom_parameter_filament_wait}", 1) == "12.5");
    CHECK_THROWS_WITH(parser.process("{custom_parameter_filament_wait}", 2), ContainsSubstring("not defined in the filament"));
}

TEST_CASE("A filament beyond the entries of the setting has no custom parameters", "[CustomGCodeParameters]")
{
    PlaceholderParser parser;
    CustomGCodeParameters::apply(parser.config_writable(), "", "", { R"({"chamber": 45})" }, 2);

    CHECK(parser.process("{custom_parameter_filament_chamber}", 0) == "45");
    CHECK_THROWS_WITH(parser.process("{custom_parameter_filament_chamber}", 1), ContainsSubstring("not defined in the filament"));
}

TEST_CASE("Custom parameter settings survive a serialization round trip", "[CustomGCodeParameters][Config]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("custom_parameters_print", new ConfigOptionString("{\n  \"purge_lines\": 2\n}"));
    config.set_key_value("custom_parameters_printer", new ConfigOptionString(R"({"park_x": 150})"));
    config.set_key_value("custom_parameters_filament", new ConfigOptionStrings(two_filaments));

    DynamicPrintConfig restored = DynamicPrintConfig::full_print_config();
    for (const std::string key : { "custom_parameters_print", "custom_parameters_printer", "custom_parameters_filament" }) {
        CAPTURE(key);
        restored.set_deserialize_strict(key, config.opt_serialize(key));
        CHECK(*restored.option(key) == *config.option(key));
    }
    CHECK(restored.option<ConfigOptionStrings>("custom_parameters_filament")->values == two_filaments);
}
