#include <catch2/catch_all.hpp>

#include <cmath>
#include <numeric>
#include <sstream>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "test_helpers.hpp" // get access to init_print, etc

#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <string>

#include "libslic3r/Config.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;

/// Test the expected behavior for auto-width,
/// spacing, etc
SCENARIO("Flow math for non-bridges", "[Flow]") {
    GIVEN("Nozzle Diameter of 0.4, a desired width of 1mm and layer height of 0.5") {
        ConfigOptionFloatOrPercent	width(1.0, false);
        float nozzle_diameter	= 0.4f;
        float layer_height		= 0.4f;

        // Spacing for non-bridges is has some overlap
        THEN("External perimeter flow has spacing fixed to 1.125 * nozzle_diameter") {
            auto flow = Flow::new_from_config_width(frExternalPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Catch::Approx(1.125 * nozzle_diameter - layer_height * (1.0 - PI / 4.0)));
        }

        THEN("Internal perimeter flow has spacing fixed to 1.125 * nozzle_diameter") {
            auto flow = Flow::new_from_config_width(frPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Catch::Approx(1.125 *nozzle_diameter - layer_height * (1.0 - PI / 4.0)));
        }
        THEN("Spacing for supplied width is 0.8927f") {
            auto flow = Flow::new_from_config_width(frExternalPerimeter, width, nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Catch::Approx(width.value - layer_height * (1.0 - PI / 4.0)));
            flow = Flow::new_from_config_width(frPerimeter, width, nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Catch::Approx(width.value - layer_height * (1.0 - PI / 4.0)));
        }
    }
    /// Check the min/max
    GIVEN("Nozzle Diameter of 0.25") {
        float nozzle_diameter	= 0.25f;
        float layer_height		= 0.5f;
        WHEN("layer height is set to 0.2") {
            layer_height = 0.15f;
            THEN("Max width is set.") {
                auto flow = Flow::new_from_config_width(frPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
                REQUIRE(flow.width() == Catch::Approx(1.125 * nozzle_diameter));
            }
        }
        WHEN("Layer height is set to 0.25") {
            layer_height = 0.25f;
            THEN("Min width is set.") {
                auto flow = Flow::new_from_config_width(frPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
                REQUIRE(flow.width() == Catch::Approx(1.125 * nozzle_diameter));
            }
        }
    }

#if 0
    /// Check for an edge case in the maths where the spacing could be 0; original
    /// math is 0.99. Slic3r issue #4654
    GIVEN("Input spacing of 0.414159 and a total width of 2") {
        double in_spacing = 0.414159;
        double total_width = 2.0;
        auto flow = Flow::new_from_spacing(1.0, 0.4, 0.3);
        WHEN("solid_spacing() is called") {
            double result = flow.solid_spacing(total_width, in_spacing);
            THEN("Yielded spacing is greater than 0") {
                REQUIRE(result > 0);
            }
        }
    }
#endif    

}

/// Spacing, width calculation for bridge extrusions
SCENARIO("Flow math for bridges", "[Flow]") {
    GIVEN("Nozzle Diameter of 0.4, a desired width of 1mm and layer height of 0.5") {
		float nozzle_diameter	= 0.4f;
		float bridge_flow		= 1.0f;
        WHEN("Flow role is frExternalPerimeter") {
            auto flow = Flow::bridging_flow(nozzle_diameter * sqrt(bridge_flow), nozzle_diameter);
            THEN("Bridge width is same as nozzle diameter") {
                REQUIRE(flow.width() == Catch::Approx(nozzle_diameter));
            }
            THEN("Bridge spacing is same as nozzle diameter + BRIDGE_EXTRA_SPACING") {
                REQUIRE(flow.spacing() == Catch::Approx(nozzle_diameter + BRIDGE_EXTRA_SPACING));
            }
        }
    }
}

namespace {

struct FirstLayerWidths
{
    double first_layer_wall;
    double first_layer_outer_wall;
    double first_layer_sparse_infill;
    double first_layer_solid_infill;
    double first_layer_top_surface;
    double second_layer_solid_infill;
    double first_layer_fill_mm3_per_mm;
    std::string gcode;
};

FirstLayerWidths first_layer_widths(const char *first_layer_infill_width)
{
    Print print;
    Slic3r::Test::init_and_process_print({ Slic3r::Test::cube(20) }, print, {
        { "nozzle_diameter",                  "0.4" },
        { "layer_height",                     0.2 },
        { "initial_layer_print_height",       0.2 },
        { "line_width",                       "0.45" },
        { "outer_wall_line_width",            "0" },
        { "inner_wall_line_width",            "0" },
        { "sparse_infill_line_width",         "0" },
        { "internal_solid_infill_line_width", "0" },
        { "top_surface_line_width",           "0" },
        { "initial_layer_line_width",         "0.4" },
        { "initial_layer_infill_line_width",  first_layer_infill_width },
    });

    const Layer       &first  = *print.objects().front()->get_layer(0);
    const LayerRegion &region = *first.regions().front();
    double volume = 0.;
    double length = 0.;
    for (const ExtrusionEntity *entity : region.fills.flatten().entities) {
        volume += entity->total_volume();
        length += unscale<double>(entity->length());
    }
    REQUIRE(length > 0.);

    return {
        region.flow(frPerimeter).width(),
        region.flow(frExternalPerimeter).width(),
        region.flow(frInfill).width(),
        region.flow(frSolidInfill).width(),
        region.flow(frTopSolidInfill).width(),
        print.objects().front()->get_layer(1)->regions().front()->flow(frSolidInfill).width(),
        volume / length,
        Slic3r::Test::gcode(print),
    };
}

} // namespace

TEST_CASE("First layer infill line width widens only the infill of the first layer", "[Flow]")
{
    const FirstLayerWidths widths = first_layer_widths("0.8");

    CHECK_THAT(widths.first_layer_sparse_infill, Catch::Matchers::WithinAbs(0.8, 1e-4));
    CHECK_THAT(widths.first_layer_solid_infill,  Catch::Matchers::WithinAbs(0.8, 1e-4));
    CHECK_THAT(widths.first_layer_top_surface,   Catch::Matchers::WithinAbs(0.8, 1e-4));
    CHECK_THAT(widths.first_layer_wall,          Catch::Matchers::WithinAbs(0.4, 1e-4));
    CHECK_THAT(widths.first_layer_outer_wall,    Catch::Matchers::WithinAbs(0.4, 1e-4));
    CHECK_THAT(widths.second_layer_solid_infill, Catch::Matchers::WithinAbs(0.45, 1e-4));
    CHECK(widths.gcode.find("; first layer infill extrusion width = 0.80mm") != std::string::npos);

    const FirstLayerWidths unset = first_layer_widths("0");
    CHECK(widths.first_layer_fill_mm3_per_mm > 1.5 * unset.first_layer_fill_mm3_per_mm);
}

TEST_CASE("First layer infill line width of zero follows the first layer line width", "[Flow]")
{
    const FirstLayerWidths widths = first_layer_widths("0");

    CHECK_THAT(widths.first_layer_sparse_infill, Catch::Matchers::WithinAbs(0.4, 1e-4));
    CHECK_THAT(widths.first_layer_solid_infill,  Catch::Matchers::WithinAbs(0.4, 1e-4));
    CHECK_THAT(widths.first_layer_top_surface,   Catch::Matchers::WithinAbs(0.4, 1e-4));
    CHECK_THAT(widths.first_layer_wall,          Catch::Matchers::WithinAbs(0.4, 1e-4));
    CHECK(widths.gcode.find("; first layer infill extrusion width") == std::string::npos);
}

TEST_CASE("First layer infill line width accepts a percentage of the nozzle diameter", "[Flow]")
{
    const FirstLayerWidths widths = first_layer_widths("150%");

    CHECK_THAT(widths.first_layer_solid_infill, Catch::Matchers::WithinAbs(0.6, 1e-4));
    CHECK_THAT(widths.first_layer_wall,         Catch::Matchers::WithinAbs(0.4, 1e-4));
}

TEST_CASE("First layer infill line width of zero resolves to the first layer line width as a G-code variable", "[Flow]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "nozzle_diameter",                 "0.4" },
        { "line_width",                      "0.45" },
        { "initial_layer_line_width",        "0.6" },
        { "initial_layer_infill_line_width", "0" },
    });
    CHECK_THAT(Flow::extrusion_width("initial_layer_infill_line_width", config), Catch::Matchers::WithinAbs(0.6, 1e-4));

    config.set_deserialize_strict({ { "initial_layer_infill_line_width", "0.8" } });
    CHECK_THAT(Flow::extrusion_width("initial_layer_infill_line_width", config), Catch::Matchers::WithinAbs(0.8, 1e-4));
}
