#include <algorithm>
#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"

#include <cmath>
#include <cstddef>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/PrintConfig.hpp"
#include <map>
#include <math.h>
#include <mutex>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "test_helpers.hpp" // get access to init_print, etc

// Not self-contained: its inline constructor uses PrintObject, PrintRegion, SlicingParameters and
// Geometry, so it must follow the headers (pulled in via test_helpers.hpp) that define them.
#include "libslic3r/Support/SupportParameters.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/GCode/AvoidCrossingPerimeters.hpp"

using namespace Slic3r::Test;
using namespace Slic3r;

// Distinct layer Z heights carrying support interface extrusion.
static size_t support_interface_layer_count(const std::string &gcode)
{
    return layers_with_role(gcode, "support material interface").size();
}

// Distinct layer Z heights carrying support base extrusion. The base G-code label "support material"
// is a substring of "support material interface", so a base line is a support line that is not an
// interface line.
static size_t support_base_layer_count(const std::string &gcode)
{
    std::set<double> layers;
    GCodeReader parser;
    parser.parse_buffer(gcode, [&layers](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (! line.extruding(self)) return;
        const std::string_view comment = line.comment();
        if (comment.find("support material") != std::string_view::npos &&
            comment.find("interface") == std::string_view::npos)
            layers.insert(self.z());
    });
    return layers.size();
}

// Dominant support-interface fill direction per interface layer, in radians [0, pi). Uses the
// length-weighted axial mean (each segment angle doubled so a line and its reverse agree, then
// halved): the parallel infill lines reinforce while the surrounding perimeter cancels.
static std::map<double, double> interface_fill_angle_by_layer(const std::string &gcode)
{
    std::map<double, std::pair<double, double>> acc; // z -> summed length*(cos2a, sin2a)
    GCodeReader parser;
    parser.parse_buffer(gcode, [&acc](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (! line.extruding(self)) return;
        if (line.comment().find("support material interface") == std::string_view::npos) return;
        const double dx = line.dist_X(self), dy = line.dist_Y(self);
        const double len = std::hypot(dx, dy);
        if (len < 1e-6) return;
        const double a2 = 2.0 * std::atan2(dy, dx);
        auto &p = acc[self.z()];
        p.first  += len * std::cos(a2);
        p.second += len * std::sin(a2);
    });
    std::map<double, double> out;
    for (const auto &kv : acc) {
        double a = 0.5 * std::atan2(kv.second.second, kv.second.first);
        if (a < 0) a += M_PI;
        out[kv.first] = a;
    }
    return out;
}

// Acute angle (degrees) between two axial fill directions in [0, pi).
static double axial_angle_diff_deg(double a, double b)
{
    const double d = std::fmod(std::fabs(a - b), M_PI);
    return std::min(d, M_PI - d) * 180.0 / M_PI;
}

// Denser interface spacing yields more extruded length.
static double support_interface_extrusion_length(const std::string &gcode)
{
    double len = 0;
    GCodeReader parser;
    parser.parse_buffer(gcode, [&len](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (! line.extruding(self)) return;
        if (line.comment().find("support material interface") == std::string_view::npos) return;
        len += std::hypot(line.dist_X(self), line.dist_Y(self));
    });
    return len;
}

// A cap slab overhanging a base, joined by a central stem: the cap can only be supported by resting on the
// base, forcing a genuine bottom contact. A horizontal tunnel does not work here -- tree/organic can arch a
// branch in from the opening and avoid the floor entirely.
static TriangleMesh support_capital()
{
    TriangleMesh model = make_cube(40, 40, 2);                              // base  [0,40]x[0,40]x[0,2]
    TriangleMesh stem  = make_cube(8, 8, 12);   stem.translate(16, 16, 1);  // stem  centered, z 1..13
    TriangleMesh cap   = make_cube(40, 40, 2);  cap.translate(0, 0, 12);    // cap   z 12..14
    model.merge(stem);
    model.merge(cap);
    return model;
}

TEST_CASE("Three raft layers are created", "[SupportMaterial]")
{
	Slic3r::Print print;
	Slic3r::Test::init_and_process_print({ cube(20) }, print, {
        { "enable_support", 1 },
        { "raft_layers",    3 }
		});
    REQUIRE(print.objects().front()->support_layers().size() == 3);
}

TEST_CASE("Enforced support layers are generated", "[SupportMaterial]")
{
    // enforce_support_layers forces support on the first N layers even with support off.
    Slic3r::Print baseline;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, baseline, {
        { "enable_support",         0 },
        { "enforce_support_layers", 0 }
    });
    REQUIRE(baseline.objects().front()->support_layers().empty());

    Slic3r::Print enforced;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, enforced, {
        { "enable_support",         0 },
        { "enforce_support_layers", 100 }
    });
    REQUIRE(enforced.objects().front()->support_layers().size() > 0);
}

// Support-needed statuses raised while slicing support_capital() with support off. The CLI lists these
// in result.json and fails on them under --strict. Collected under a lock: generate_support_material()
// runs on TBB workers.
static std::vector<PrintBase::SlicingStatus> support_needed_statuses(bool no_check)
{
    Slic3r::Print print;
    Slic3r::Model model;
    Slic3r::Test::init_print({ support_capital() }, print, model, {
        { "enable_support",         0 },
        { "enforce_support_layers", 0 }
    });
    print.set_no_check_flag(no_check);

    std::mutex mutex;
    std::vector<PrintBase::SlicingStatus> statuses;
    print.set_status_callback([&mutex, &statuses](const PrintBase::SlicingStatus &status) {
        if (status.message_type != PrintStateBase::SlicingNeedSupportOn)
            return;
        std::lock_guard<std::mutex> lock(mutex);
        statuses.push_back(status);
    });
    print.process();
    return statuses;
}

TEST_CASE("An overhang sliced with support off reports that support is needed", "[SupportMaterial]")
{
    // The 40mm cap reaches ~22mm past its 8mm stem, beyond the 6mm cantilever limit of
    // PrintObject::is_support_necessary().
    const std::vector<PrintBase::SlicingStatus> statuses = support_needed_statuses(false);
    REQUIRE(! statuses.empty());
    for (const PrintBase::SlicingStatus &status : statuses) {
        // The CLI only considers step warnings (warning_step != -1), and --strict only NON_CRITICAL ones.
        CHECK(status.warning_level == PrintStateBase::WarningLevel::NON_CRITICAL);
        CHECK(status.warning_step != -1);
    }
}

TEST_CASE("The no-check flag skips the support-needed check", "[SupportMaterial]")
{
    CHECK(support_needed_statuses(true).empty());
}

SCENARIO("Support layer Z honors contact distance", "[SupportMaterial]")
{
    // Box h = 20mm, hole bottom at 5mm, hole height 10mm (top edge at 15mm).
    TriangleMesh mesh = Slic3r::Test::mesh(Slic3r::Test::TestMesh::cube_with_hole);
    mesh.rotate_x(float(M_PI / 2));

	auto check = [](Slic3r::Print &print, bool &first_support_layer_height_ok, bool &layer_height_minimum_ok, bool &layer_height_maximum_ok)
	{
        ConstSupportLayerPtrsAdaptor support_layers = print.objects().front()->support_layers();

		first_support_layer_height_ok = support_layers.front()->print_z == print.config().initial_layer_print_height.value;

		layer_height_minimum_ok = true;
		layer_height_maximum_ok = true;
		double min_layer_height = print.config().min_layer_height.values.front();
		double max_layer_height = print.config().nozzle_diameter.values.front();
		if (print.config().max_layer_height.values.front() > EPSILON)
			max_layer_height = std::min(max_layer_height, print.config().max_layer_height.values.front());
		for (size_t i = 1; i < support_layers.size(); ++ i) {
			if (support_layers[i]->print_z - support_layers[i - 1]->print_z < min_layer_height - EPSILON)
				layer_height_minimum_ok = false;
			if (support_layers[i]->print_z - support_layers[i - 1]->print_z > max_layer_height + EPSILON)
				layer_height_maximum_ok = false;
		}
	};

    GIVEN("A print object having one modelObject") {
        WHEN("Layer height = 0.2 and first layer height = 0.4") {
			Slic3r::Print print;
			Slic3r::Test::init_and_process_print({ mesh }, print, {
                { "enable_support",             1 },
                { "layer_height",               0.2 },
                { "initial_layer_print_height", 0.4 },
                { "dont_support_bridges",       false },
			});
			bool first_layer_ok, layer_min_ok, layer_max_ok;
            check(print, first_layer_ok, layer_min_ok, layer_max_ok);
            THEN("First layer height is honored")			{ REQUIRE(first_layer_ok == true); }
            THEN("No null or negative support layers")		{ REQUIRE(layer_min_ok == true); }
            THEN("No layers thicker than nozzle diameter")	{ REQUIRE(layer_max_ok == true); }
        }
        WHEN("Layer height = 0.2 and first layer height = 0.3") {
			Slic3r::Print print;
			Slic3r::Test::init_and_process_print({ mesh }, print, {
                { "enable_support",             1 },
                { "layer_height",               0.2 },
                { "initial_layer_print_height", 0.3 },
                { "dont_support_bridges",       false },
            });
            bool first_layer_ok, layer_min_ok, layer_max_ok;
            check(print, first_layer_ok, layer_min_ok, layer_max_ok);
            THEN("First layer height is honored")			{ REQUIRE(first_layer_ok == true); }
            THEN("No null or negative support layers")		{ REQUIRE(layer_min_ok == true); }
            THEN("No layers thicker than nozzle diameter")	{ REQUIRE(layer_max_ok == true); }
        }
    }
}

// extrude_support once held a `static` lambda capturing `this`, so a second export in the
// same process dereferenced a returned stack frame (ASan: stack-use-after-return).
TEST_CASE("Support G-code emission survives a second slice in the same process", "[SupportMaterial][Regression]")
{
    const std::string first = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(! layers_with_role(first, "support").empty());

    const std::string second = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(! layers_with_role(second, "support").empty());
}

// The contact layer counts toward the configured interface layer count, so N configured top
// interface layers produce exactly N interface layers, not N+1.
TEST_CASE("Support top interface layer count matches the configured value", "[SupportMaterial]")
{
    const int top = GENERATE(1, 2, 3, 4, 6);
    const std::string g = slice({ TestMesh::overhang }, {
        { "enable_support",                  1 },
        { "layer_height",                    0.2 },
        { "support_on_build_plate_only",     1 },
        { "support_interface_top_layers",    top },
        { "support_interface_bottom_layers", 0 },
    });
    CAPTURE(top);
    REQUIRE(support_base_layer_count(g)      > 0);          // support actually formed
    REQUIRE(support_interface_layer_count(g) == size_t(top));
}

// A rotated cube-with-hole is a horizontal tunnel whose ceiling and floor both receive support, so top
// and bottom interfaces can be exercised independently (the floor is the bottom contact).
static TriangleMesh support_tunnel()
{
    TriangleMesh tunnel = Slic3r::Test::mesh(TestMesh::cube_with_hole);
    tunnel.rotate_x(float(M_PI / 2));
    return tunnel;
}

static size_t tunnel_interface_layers(const TriangleMesh &tunnel, int top, int bottom)
{
    const std::string g = slice({ tunnel }, {
        { "enable_support",                  1 },
        { "layer_height",                    0.2 },
        { "support_on_build_plate_only",     0 },
        { "support_interface_top_layers",    top },
        { "support_interface_bottom_layers", bottom },
    });
    REQUIRE(support_base_layer_count(g) > 0); // support actually formed
    return support_interface_layer_count(g);
}

TEST_CASE("No support interface is generated when neither top nor bottom is configured", "[SupportMaterial]")
{
    REQUIRE(tunnel_interface_layers(support_tunnel(), 0, 0) == 0);
}

TEST_CASE("Bottom interface layer count matches its setting with top interface off", "[SupportMaterial]")
{
    const int bottom = GENERATE(1, 3, 6);
    CAPTURE(bottom);
    REQUIRE(tunnel_interface_layers(support_tunnel(), 0, bottom) == size_t(bottom));
}

// support_interface_bottom_layers = -1 means "same as top".
TEST_CASE("Support interface bottom layers default to the top layer count", "[SupportMaterial]")
{
    const TriangleMesh tunnel = support_tunnel();
    REQUIRE(tunnel_interface_layers(tunnel, 0, -1) == tunnel_interface_layers(tunnel, 0, 0));
    REQUIRE(tunnel_interface_layers(tunnel, 3, -1) == tunnel_interface_layers(tunnel, 3, 3));
}

TEST_CASE("Default support still emits base and interface material", "[SupportMaterial][Regression]")
{
    const std::string g = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(support_base_layer_count(g)      > 0);
    REQUIRE(support_interface_layer_count(g) > 0);
}

// Organic runs TreeSupport3D + TreeModelVolumes, the others the classic TreeSupport.cpp path.
TEST_CASE("Every tree support style produces base and interface material", "[SupportMaterial]")
{
    const char *style = GENERATE("organic", "tree_slim", "tree_strong", "tree_hybrid");
    INFO("style=" << style);
    const std::string g = slice({ TestMesh::overhang }, {
        { "enable_support",               1 },
        { "layer_height",                 0.2 },
        { "support_type",                 "tree(auto)" },
        { "support_style",                style },
        { "support_interface_top_layers", 3 },
    });
    CHECK(support_base_layer_count(g)      > 0);
    CHECK(support_interface_layer_count(g) > 0);
}

TEST_CASE("Raft interface angle alternates by 45 degrees per interface id", "[SupportMaterial]")
{
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, print, { { "enable_support", 1 } });
    SupportParameters sp(*print.objects().front());
    sp.raft_angle_interface = 0.5f;
    REQUIRE_THAT(sp.raft_interface_angle(0), Catch::Matchers::WithinAbs(0.5 + M_PI / 4., 1e-6));
    REQUIRE_THAT(sp.raft_interface_angle(1), Catch::Matchers::WithinAbs(0.5 - M_PI / 4., 1e-6));
}

// The angle inputs are overwritten directly, so the pattern-to-angle mapping is checked
// independently of the sliced object's configuration.
TEST_CASE("Support interface fill angle follows the configured interface pattern", "[SupportMaterial]")
{
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, print, { { "enable_support", 1 } });
    SupportParameters sp(*print.objects().front());
    sp.interface_angle = 0.3f;
    sp.base_angle      = 1.1f;
    const double tol   = 1e-6;

    SECTION("Rectilinear shifts the interface angle by -45deg for snug support") {
        sp.support_interface_pattern = smipRectilinear;
        sp.support_style             = smsSnug;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.interface_angle - M_PI_4, tol));
        REQUIRE_THAT(sp.support_interface_angle(3), Catch::Matchers::WithinAbs(sp.interface_angle - M_PI_4, tol));
    }
    SECTION("Rectilinear leaves the interface angle alone for the other styles") {
        sp.support_interface_pattern = smipRectilinear;
        sp.support_style             = smsGrid;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.interface_angle, tol));
    }
    SECTION("Rectilinear interlaced alternates -/+45deg by interface id parity") {
        sp.support_interface_pattern = smipRectilinearInterlaced;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.interface_angle - M_PI_4, tol));
        REQUIRE_THAT(sp.support_interface_angle(1), Catch::Matchers::WithinAbs(sp.interface_angle + M_PI_4, tol));
    }
    SECTION("Grid uses the base angle") {
        sp.support_interface_pattern = smipGrid;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.base_angle, tol));
    }
    SECTION("Auto and concentric use the interface angle unchanged") {
        sp.support_interface_pattern = smipAuto;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.interface_angle, tol));
        sp.support_interface_pattern = smipConcentric;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.interface_angle, tol));
    }
}

// End-to-end that the pattern reaches the emitted fill, not just support_interface_angle().
TEST_CASE("Interlaced support interface alternates fill angle while rectilinear does not", "[SupportMaterial]")
{
    auto interface_angles = [](const char *pattern) {
        std::vector<double> a;
        for (const auto &kv : interface_fill_angle_by_layer(slice({ TestMesh::overhang }, {
                 { "enable_support",               1 },
                 { "layer_height",                 0.2 },
                 { "support_on_build_plate_only",  1 },
                 { "support_interface_top_layers", 6 },
                 { "support_interface_pattern",    pattern } })))
            a.push_back(kv.second);
        return a;
    };

    const std::vector<double> rectilinear = interface_angles("rectilinear");
    const std::vector<double> interlaced  = interface_angles("rectilinear_interlaced");
    REQUIRE(rectilinear.size() >= 3);
    REQUIRE(interlaced.size()  >= 3);

    for (size_t i = 1; i < rectilinear.size(); ++i)
        REQUIRE(axial_angle_diff_deg(rectilinear[i], rectilinear[0]) < 15.0);

    for (size_t i = 1; i < interlaced.size(); ++i)
        REQUIRE(axial_angle_diff_deg(interlaced[i], interlaced[i - 1]) > 60.0);
}

// Normal and non-organic tree support share the same interface angle logic: with a rectilinear interface
// pattern both emit their interface fill at the same angle (both go through support_interface_angle()).
TEST_CASE("Normal and tree support use the same interface fill angle", "[SupportMaterial]")
{
    auto mean_interface_angle = [](const char *type, const char *style) {
        const auto angles = interface_fill_angle_by_layer(slice({ TestMesh::overhang }, {
            { "enable_support", 1 }, { "layer_height", 0.2 }, { "support_on_build_plate_only", 1 },
            { "support_type", type }, { "support_style", style },
            { "support_interface_top_layers", 6 }, { "support_interface_pattern", "rectilinear" } }));
        REQUIRE(angles.size() >= 3);
        // Axial mean, as in interface_fill_angle_by_layer: a plain mean would split angles either
        // side of the [0, pi) wrap.
        double x = 0, y = 0;
        for (const auto &kv : angles) {
            x += std::cos(2.0 * kv.second);
            y += std::sin(2.0 * kv.second);
        }
        double mean = 0.5 * std::atan2(y, x);
        if (mean < 0) mean += M_PI;
        return mean;
    };
    REQUIRE(axial_angle_diff_deg(mean_interface_angle("normal(auto)", "default"),
                                 mean_interface_angle("tree(auto)", "tree_slim")) < 10.0);
}

// Every style, because the non-organic tree styles once emitted one more top interface layer than the rest.
TEST_CASE("Top interface layer count equals the configured value for every support style", "[SupportMaterial]")
{
    auto [type, style] = GENERATE(table<const char *, const char *>({
        { "normal(auto)", "grid" },        { "normal(auto)", "snug" },
        { "tree(auto)",   "organic" },     { "tree(auto)",   "tree_slim" },
        { "tree(auto)",   "tree_strong" }, { "tree(auto)",   "tree_hybrid" },
    }));
    CAPTURE(style);
    const std::string g = slice({ TestMesh::overhang }, {
        { "enable_support",               1 },
        { "layer_height",                 0.2 },
        { "support_type",                 type },
        { "support_style",                style },
        { "support_interface_top_layers", 4 },
    });
    REQUIRE(support_interface_layer_count(g) == 4u);
}

// The bottom interface was dropped in earlier versions when support started on the model rather
// than the plate.
TEST_CASE("Non-organic tree support generates a bottom interface on internal geometry", "[SupportMaterial]")
{
    const std::string g = slice({ support_tunnel() }, {
        { "enable_support",                  1 },
        { "layer_height",                    0.2 },
        { "support_on_build_plate_only",     0 },
        { "support_type",                    "tree(auto)" },
        { "support_style",                   "tree_slim" },
        { "support_interface_top_layers",    0 },
        { "support_interface_bottom_layers", 6 },
    });
    REQUIRE(support_base_layer_count(g)      > 0);
    REQUIRE(support_interface_layer_count(g) > 0);
}

// The capital forces the model contact; on a horizontal tunnel organic can arch a branch in and make none.
TEST_CASE("A bottom interface is produced for every support style on a forced model contact", "[SupportMaterial]")
{
    auto [type, style] = GENERATE(table<const char *, const char *>({
        { "normal(auto)", "default" },     { "tree(auto)", "tree_slim" },
        { "tree(auto)",   "tree_strong" }, { "tree(auto)", "tree_hybrid" },
        { "tree(auto)",   "organic" },
    }));
    CAPTURE(style);
    REQUIRE(support_interface_layer_count(slice({ support_capital() }, {
        { "enable_support", 1 }, { "layer_height", 0.2 }, { "support_on_build_plate_only", 0 },
        { "support_type", type }, { "support_style", style },
        { "support_interface_top_layers", 0 }, { "support_interface_bottom_layers", 6 } })) > 0);
}

TEST_CASE("Bottom interface spacing controls bottom interface density for every support style", "[SupportMaterial]")
{
    auto [type, style] = GENERATE(table<const char *, const char *>({
        { "normal(auto)", "default" },     { "tree(auto)", "tree_slim" },
        { "tree(auto)",   "tree_strong" }, { "tree(auto)", "tree_hybrid" },
        { "tree(auto)",   "organic" },
    }));
    CAPTURE(style);
    const TriangleMesh model = support_capital();
    auto len = [&model](const char *support_type, const char *support_style, double spacing) {
        return support_interface_extrusion_length(slice({ model }, {
            { "enable_support", 1 }, { "layer_height", 0.2 }, { "support_on_build_plate_only", 0 },
            { "support_type", support_type }, { "support_style", support_style }, { "support_interface_top_layers", 0 },
            { "support_interface_bottom_layers", 6 }, { "support_bottom_interface_spacing", spacing } }));
    };
    REQUIRE(len(type, style, 0.0) > len(type, style, 4.0) * 1.5);
}

// Interface and base flows are identical in width and rate unless a separate support-interface
// filament is used, so density is the observable here, not flow.
TEST_CASE("Bottom-only support interface keeps the dense interface density", "[SupportMaterial]")
{
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, print, {
        { "enable_support",                   1 },
        { "support_interface_top_layers",     0 },
        { "support_interface_bottom_layers",  6 },
        { "support_bottom_interface_spacing", 0.0 },  // solid: density resolves to 1.0
        { "support_base_pattern_spacing",     2.5 },  // sparse: density stays below 1.0
    });
    SupportParameters sp(*print.objects().front());
    REQUIRE(sp.bottom_interface_density > sp.support_density);
}


struct MoveLengths {
    double travel    = 0;
    double extrusion = 0;
};

static MoveLengths move_lengths(const std::string &gcode)
{
    MoveLengths lengths;
    GCodeReader parser;
    parser.parse_buffer(gcode, [&lengths](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const double len = std::hypot(line.dist_X(self), line.dist_Y(self));
        if (line.extruding(self))
            lengths.extrusion += len;
        else
            lengths.travel += len;
    });
    return lengths;
}

static MoveLengths supported_capital_move_lengths(bool avoid_crossing_walls, bool include_supports)
{
    return move_lengths(slice({ support_capital() }, {
        { "enable_support",                       1 },
        { "layer_height",                         0.2 },
        { "support_on_build_plate_only",          0 },
        { "reduce_crossing_wall",                 avoid_crossing_walls ? 1 : 0 },
        { "avoid_crossing_wall_includes_support", include_supports ? 1 : 0 },
    }));
}

TEST_CASE("Including supports when avoiding crossing walls is off by default", "[SupportMaterial][AvoidCrossingPerimeters]")
{
    const DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    REQUIRE_FALSE(config.opt_bool("avoid_crossing_wall_includes_support"));
}

TEST_CASE("Including supports does nothing while avoiding crossing walls is disabled", "[SupportMaterial][AvoidCrossingPerimeters]")
{
    const MoveLengths without_supports = supported_capital_move_lengths(false, false);
    const MoveLengths with_supports    = supported_capital_move_lengths(false, true);
    REQUIRE(without_supports.extrusion > 0);
    REQUIRE_THAT(with_supports.travel, Catch::Matchers::WithinAbs(without_supports.travel, 1e-6));
    REQUIRE_THAT(with_supports.extrusion, Catch::Matchers::WithinAbs(without_supports.extrusion, 1e-6));
}

struct TravelsOverMaterial {
    MoveLengths lengths;
    double      travel_over_air      = 0;
    double      extrusion_on_layers  = 0;
    double      extrusion_over_area  = 0;
    size_t      support_layers_with_fills   = 0;
    size_t      support_layers_with_islands = 0;
};

// Footprint of what is printed at print_z: the object slices plus the area its support extrusions cover,
// in G-code coordinates and grown by half a millimetre so that a travel hugging an edge counts as over material.
static Polygons printed_area(const PrintObject &object, double print_z)
{
    Polygons area;
    if (const Layer *layer = object.get_layer_at_printz(print_z, 0.01))
        append(area, to_polygons(layer->lslices));
    if (const SupportLayer *support_layer = object.get_support_layer_at_printz(print_z, 0.01))
        append(area, support_layer->support_fills.polygons_covered_by_spacing());
    for (Polygon &polygon : area)
        polygon.translate(object.instances().front().shift);
    return offset(union_(area), scaled<float>(0.5));
}

static TravelsOverMaterial travels_over_material(const PrintObject &object, const std::string &gcode)
{
    std::map<double, std::pair<Polylines, Polylines>> moves_by_z;
    GCodeReader parser;
    parser.parse_buffer(gcode, [&moves_by_z](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (!line.cmd_is("G0") && !line.cmd_is("G1"))
            return;
        if (std::hypot(line.dist_X(self), line.dist_Y(self)) < 1e-3)
            return;
        Polyline segment(Point::new_scale(self.x(), self.y()), Point::new_scale(line.new_X(self), line.new_Y(self)));
        auto &moves = moves_by_z[line.new_Z(self)];
        (line.extruding(self) ? moves.second : moves.first).push_back(std::move(segment));
    });

    TravelsOverMaterial result;
    result.lengths = move_lengths(gcode);
    for (const auto &[z, moves] : moves_by_z) {
        const Polygons area = printed_area(object, z);
        if (area.empty())
            continue;
        result.travel_over_air     += unscale<double>(total_length(diff_pl(moves.first, area)));
        result.extrusion_on_layers += unscale<double>(total_length(moves.second));
        result.extrusion_over_area += unscale<double>(total_length(intersection_pl(moves.second, area)));
    }
    return result;
}

static TravelsOverMaterial supported_capital_travels(std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> support_items, bool include_supports)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support",                       "1" },
        { "layer_height",                         "0.2" },
        { "support_on_build_plate_only",          "0" },
        { "z_hop",                                "0" },
        { "reduce_crossing_wall",                 "1" },
        { "avoid_crossing_wall_includes_support", include_supports ? "1" : "0" },
    });
    config.set_deserialize_strict(support_items);
    Print print;
    Model model;
    init_print({ support_capital() }, print, model, config);
    const std::string   g        = gcode(print);
    const PrintObject  &object   = *print.objects().front();
    TravelsOverMaterial travels  = travels_over_material(object, g);
    for (const SupportLayer *support_layer : object.support_layers())
        if (!support_layer->support_fills.empty()) {
            ++travels.support_layers_with_fills;
            if (!support_layer->support_islands.empty())
                ++travels.support_layers_with_islands;
        }
    return travels;
}

static void require_same_extrusions_and_mapped_coordinates(const TravelsOverMaterial &without_supports, const TravelsOverMaterial &with_supports)
{
    REQUIRE(without_supports.lengths.extrusion > 0);
    REQUIRE_THAT(with_supports.lengths.extrusion, Catch::Matchers::WithinAbs(without_supports.lengths.extrusion, 1e-3));
    REQUIRE(without_supports.extrusion_on_layers > 0.9 * without_supports.lengths.extrusion);
    REQUIRE(without_supports.extrusion_over_area > 0.99 * without_supports.extrusion_on_layers);
}

static void report(const char *label, const TravelsOverMaterial &without_supports, const TravelsOverMaterial &with_supports)
{
    WARN(label << ": travel " << without_supports.lengths.travel << " -> " << with_supports.lengths.travel
               << " mm, travel over air " << without_supports.travel_over_air << " -> " << with_supports.travel_over_air
               << " mm, extrusion " << without_supports.lengths.extrusion << " mm, on mapped layers "
               << without_supports.extrusion_on_layers << " mm, over the printed area " << without_supports.extrusion_over_area
               << " mm, support layers " << with_supports.support_layers_with_fills << " (with islands " << with_supports.support_layers_with_islands << ")");
}

TEST_CASE("Including supports reroutes travels without touching extrusions", "[SupportMaterial][AvoidCrossingPerimeters]")
{
    const TravelsOverMaterial without_supports = supported_capital_travels({}, false);
    const TravelsOverMaterial with_supports    = supported_capital_travels({}, true);
    report("normal support", without_supports, with_supports);
    require_same_extrusions_and_mapped_coordinates(without_supports, with_supports);
    REQUIRE(with_supports.support_layers_with_islands > 0);
    REQUIRE(with_supports.travel_over_air < without_supports.travel_over_air - 1.0);
}

TEST_CASE("Including supports also reroutes travels around tree supports, which carry no support islands", "[SupportMaterial][AvoidCrossingPerimeters]")
{
    const TravelsOverMaterial without_supports = supported_capital_travels({ { "support_type", "tree(auto)" }, { "support_style", "tree_hybrid" } }, false);
    const TravelsOverMaterial with_supports    = supported_capital_travels({ { "support_type", "tree(auto)" }, { "support_style", "tree_hybrid" } }, true);
    report("tree support", without_supports, with_supports);
    REQUIRE(with_supports.support_layers_with_fills > 0);
    REQUIRE(with_supports.support_layers_with_islands == 0);
    require_same_extrusions_and_mapped_coordinates(without_supports, with_supports);
    REQUIRE(with_supports.travel_over_air < without_supports.travel_over_air - 1.0);
}

static void place_beside_a_plain_cube(TriangleMesh first, bool include_supports, Print &print, Model &model)
{
    first.translate(80, 80, 0);
    TriangleMesh second = make_cube(20, 20, 14);
    second.translate(140, 90, 0);
    std::vector<TriangleMesh> meshes;
    meshes.push_back(std::move(first));
    meshes.push_back(std::move(second));

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support",                       "1" },
        { "layer_height",                         "0.2" },
        { "support_on_build_plate_only",          "0" },
        { "reduce_crossing_wall",                 "1" },
        { "avoid_crossing_wall_includes_support", include_supports ? "1" : "0" },
    });
    init_print(std::move(meshes), print, model, config, nullptr, false);
}

static MoveLengths two_objects_move_lengths(TriangleMesh first, bool include_supports)
{
    Print print;
    Model model;
    place_beside_a_plain_cube(std::move(first), include_supports, print, model);
    return move_lengths(gcode(print));
}

static bool stays_within(const Polygons &boundary, const Point &center, coord_t reach)
{
    const BoundingBox extents = get_extents(boundary);
    return extents.min.x() > center.x() - reach && extents.max.x() < center.x() + reach &&
           extents.min.y() > center.y() - reach && extents.max.y() < center.y() + reach;
}

TEST_CASE("The boundary between objects takes the supports of each object at its own position", "[SupportMaterial][AvoidCrossingPerimeters]")
{
    Print print;
    Model model;
    place_beside_a_plain_cube(support_capital(), true, print, model);
    print.process();

    const PrintObject  &supported_object = *print.objects().front();
    const PrintObject  &plain_object     = *print.objects().back();
    const SupportLayer *support_layer    = nullptr;
    for (const SupportLayer *candidate : supported_object.support_layers())
        if (!candidate->support_fills.empty()) {
            support_layer = candidate;
            if (plain_object.get_layer_at_printz(candidate->print_z, EPSILON) != nullptr)
                break;
        }
    REQUIRE(support_layer != nullptr);
    REQUIRE(plain_object.support_layers().empty());

    const Point   supported_center = supported_object.instances().front().shift;
    const coord_t reach            = scaled<coord_t>(30.);

    REQUIRE(AvoidCrossingPerimeters::boundary_between_objects(*support_layer, false).empty());

    const Polygons from_support_layer = AvoidCrossingPerimeters::boundary_between_objects(*support_layer, true);
    REQUIRE(!from_support_layer.empty());
    REQUIRE(stays_within(from_support_layer, supported_center, reach));

    if (const Layer *plain_layer = plain_object.get_layer_at_printz(support_layer->print_z, EPSILON)) {
        REQUIRE(AvoidCrossingPerimeters::boundary_between_objects(*plain_layer, false).empty());
        const Polygons from_plain_layer = AvoidCrossingPerimeters::boundary_between_objects(*plain_layer, true);
        REQUIRE(!from_plain_layer.empty());
        REQUIRE(stays_within(from_plain_layer, supported_center, reach));
    }
}

TEST_CASE("Including supports leaves the travels between two unsupported objects alone", "[SupportMaterial][AvoidCrossingPerimeters]")
{
    const MoveLengths without_supports = two_objects_move_lengths(make_cube(20, 20, 14), false);
    const MoveLengths with_supports    = two_objects_move_lengths(make_cube(20, 20, 14), true);
    REQUIRE(without_supports.extrusion > 0);
    REQUIRE_THAT(with_supports.travel, Catch::Matchers::WithinAbs(without_supports.travel, 1e-6));
    REQUIRE_THAT(with_supports.extrusion, Catch::Matchers::WithinAbs(without_supports.extrusion, 1e-6));
}
