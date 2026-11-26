#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Point.hpp"
#include <cstddef>
#include "libslic3r/libslic3r.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/Layer.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_helpers.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Print.hpp"

using namespace Slic3r::Test;
using namespace Slic3r;

namespace {

// The upper plate overhangs both the lower plate and open air, so branches land on the model and on
// the bed in the same slice.
TriangleMesh two_tier_mesh()
{
    TriangleMesh lower  = make_cube(30, 30, 3);
    TriangleMesh column = make_cube(8, 8, 15);
    TriangleMesh upper  = make_cube(50, 50, 3);
    // Each part overlaps the one below rather than resting on it; a coplanar join slices ambiguously.
    column.translate(11.f, 11.f, 2.f);
    upper.translate(-10.f, -10.f, 16.f);
    TriangleMesh mesh = lower;
    mesh.merge(column);
    mesh.merge(upper);
    return mesh;
}

TriangleMesh scaled(TestMesh id, float scale)
{
    TriangleMesh mesh = Slic3r::Test::mesh(id);
    mesh.scale(scale);
    return mesh;
}

// `extra` is applied last, so a caller can add or override any key.
void slice_with_tree_support(const TriangleMesh &mesh, Slic3r::Print &print, const char *style,
                             int threshold_angle = 30, int build_plate_only = 0, int raft_layers = 0,
                             std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra = {})
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support",              1 },
        { "support_type",                "tree(auto)" },
        { "support_style",               style },
        { "support_on_build_plate_only", build_plate_only },
        { "support_threshold_angle",     threshold_angle },
        { "raft_layers",                 raft_layers },
        { "layer_height",                0.2 },
    });
    config.set_deserialize_strict(extra);
    Slic3r::Test::init_and_process_print({ mesh }, print, config);
}

Points support_points(const Slic3r::Print &print)
{
    Points points;
    for (const SupportLayer *layer : print.objects().front()->support_layers())
        layer->support_fills.collect_points(points);
    return points;
}

size_t support_point_count(const TriangleMesh &mesh, const char *style, int threshold_angle = 30,
                           int build_plate_only = 0)
{
    Slic3r::Print print;
    slice_with_tree_support(mesh, print, style, threshold_angle, build_plate_only);
    return support_points(print).size();
}

// Index of the first differing point, or the common length when they match. An index keeps a
// failure readable; comparing the vectors themselves dumps thousands of points.
size_t first_difference(const Points &a, const Points &b)
{
    const size_t common = std::min(a.size(), b.size());
    for (size_t i = 0; i < common; ++i)
        if (a[i] != b[i])
            return i;
    return common;
}

// Slice `mesh` twice and require an identical support point sequence. Point counts and total
// length are order insensitive, so the sequence is what a reordering shows up in.
void sliced_twice_matches(const TriangleMesh &mesh, int build_plate_only, const char *style = "tree_slim",
                          std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra = {})
{
    Slic3r::Print first_print, second_print;
    slice_with_tree_support(mesh, first_print, style, 30, build_plate_only, 0, extra);
    slice_with_tree_support(mesh, second_print, style, 30, build_plate_only, 0, extra);
    const Points first  = support_points(first_print);
    const Points second = support_points(second_print);
    REQUIRE(first.size() > 1000); // without support the comparison below passes vacuously
    REQUIRE(second.size() == first.size());
    REQUIRE(first_difference(first, second) == first.size());
}

// Smallest distance from a support toolpath to the column of two_tier_mesh(), over the layers where
// the column is all there is of the object.
double support_to_column_gap(const char *type, const char *style, bool top_z_overrides_xy)
{
    Slic3r::Print print;
    slice_with_tree_support(two_tier_mesh(), print, style, 30, 0, 0, {
        { "support_type",                type },
        { "support_object_xy_distance",  1.0 },
        { "support_top_z_distance",      0.2 },
        { "top_z_overrides_xy_distance", top_z_overrides_xy ? 1 : 0 },
    });
    const PrintObject *object = print.objects().front();
    double gap = std::numeric_limits<double>::max();
    for (const SupportLayer *support_layer : object->support_layers()) {
        if (support_layer->print_z < 5. || support_layer->print_z > 13.)
            continue;
        const Layer *layer = *std::min_element(object->layers().begin(), object->layers().end(),
            [support_layer](const Layer *a, const Layer *b) {
                return std::abs(a->print_z - support_layer->print_z) < std::abs(b->print_z - support_layer->print_z);
            });
        Points points;
        support_layer->support_fills.collect_points(points);
        for (const ExPolygon &island : layer->lslices)
            for (const Line &edge : to_lines(island))
                for (const Point &point : points)
                    gap = std::min(gap, edge.distance_to(point));
    }
    return unscale<double>(gap);
}

void collect_points_with_role(const ExtrusionEntityCollection &collection, ExtrusionRole role, Points &points)
{
    for (const ExtrusionEntity *entity : collection.entities) {
        if (entity->is_collection())
            collect_points_with_role(*static_cast<const ExtrusionEntityCollection *>(entity), role, points);
        else if (entity->role() == role)
            entity->collect_points(points);
    }
}

struct ClassicTreeGaps
{
    double interface;
    double branches;
};

// Smallest distance from the interface and from the branches of a classic tree to the column of
// two_tier_mesh(). The column is a prism, so its outline at mid height stands for every layer up to
// the upper plate, including the interface layers that sit just under it.
ClassicTreeGaps classic_tree_to_column_gaps(const char *style, double xy_distance, double top_z_distance,
                                            bool top_z_overrides_xy)
{
    Slic3r::Print print;
    slice_with_tree_support(two_tier_mesh(), print, style, 30, 0, 0, {
        { "support_object_xy_distance",      xy_distance },
        { "support_top_z_distance",          top_z_distance },
        { "support_interface_top_layers",    3 },
        { "support_interface_bottom_layers", 0 },
        { "top_z_overrides_xy_distance",     top_z_overrides_xy ? 1 : 0 },
    });
    const PrintObject *object = print.objects().front();
    const Layer *mid_column = *std::min_element(object->layers().begin(), object->layers().end(),
        [](const Layer *a, const Layer *b) { return std::abs(a->print_z - 10.) < std::abs(b->print_z - 10.); });
    const Lines column = to_lines(mid_column->lslices);

    auto gap_to_column = [&column](const Points &points) {
        double gap = std::numeric_limits<double>::max();
        for (const Line &edge : column)
            for (const Point &point : points)
                gap = std::min(gap, edge.distance_to(point));
        return unscale<double>(gap);
    };

    Points interface_points, branch_points;
    for (const SupportLayer *support_layer : object->support_layers()) {
        if (support_layer->print_z < 5. || support_layer->print_z >= 16.)
            continue;
        collect_points_with_role(support_layer->support_fills, erSupportMaterialInterface, interface_points);
        if (support_layer->print_z <= 13.)
            collect_points_with_role(support_layer->support_fills, erSupportMaterial, branch_points);
    }
    REQUIRE_FALSE(interface_points.empty());
    REQUIRE_FALSE(branch_points.empty());
    return { gap_to_column(interface_points), gap_to_column(branch_points) };
}

} // namespace

TEST_CASE("Tree support is generated for an overhang and not for a plain cube", "[TreeSupport]")
{
    REQUIRE(support_point_count(scaled(TestMesh::overhang, 2.f), "tree_slim") > 1000);
    REQUIRE(support_point_count(Slic3r::Test::cube(20), "tree_slim") == 0);
}

TEST_CASE("Restricting tree support to the build plate changes what is generated", "[TreeSupport]")
{
    const TriangleMesh mesh = two_tier_mesh();
    const size_t anywhere    = support_point_count(mesh, "tree_slim", 30, 0);
    const size_t plate_only  = support_point_count(mesh, "tree_slim", 30, 1);
    REQUIRE(anywhere > 1000);
    REQUIRE(plate_only > 1000);
    // The upper plate overhangs the lower one, so some branches would land on the model.
    REQUIRE(plate_only != anywhere);
}

TEST_CASE("Tree support layers rise monotonically within the layer height limits", "[TreeSupport]")
{
    Slic3r::Print print;
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), print, "tree_slim");
    const double nozzle = print.config().nozzle_diameter.values.front();

    size_t checked = 0;
    double previous = 0;
    bool   previous_was_adjacent = false;
    for (const SupportLayer *layer : print.objects().front()->support_layers()) {
        if (layer->print_z <= 0 || layer->height <= 0) {
            // Layers with no nodes are left at zero. Skipping one leaves a hole, so the next pair
            // spans more than one layer and its gap says nothing about the layer height limit.
            previous_was_adjacent = false;
            continue;
        }
        if (previous > 0) {
            CAPTURE(previous, layer->print_z);
            REQUIRE(layer->print_z > previous);
            if (previous_was_adjacent)
                REQUIRE(layer->print_z - previous <= nozzle + EPSILON);
        }
        previous = layer->print_z;
        previous_was_adjacent = true;
        ++checked;
    }
    REQUIRE(checked > 10);
}

TEST_CASE("A raft is still generated under tree support", "[TreeSupport]")
{
    // The mesh supports itself, so a layer count alone passes with no raft at all.
    Slic3r::Print rafted, unrafted;
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), rafted, "tree_slim", 30, 0, 3);
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), unrafted, "tree_slim", 30, 0, 0);
    const PrintObject *rafted_object   = rafted.objects().front();
    const PrintObject *unrafted_object = unrafted.objects().front();
    REQUIRE(rafted_object->support_layers().size() > unrafted_object->support_layers().size());
    // The raft goes under the object.
    REQUIRE(rafted_object->layers().front()->print_z > unrafted_object->layers().front()->print_z);
}

// drop_nodes() decides the node merges and spawns the next layer's nodes in parallel. Every one of
// those decisions has to be applied in a fixed order, or the same model gives different branches on
// each slice.
TEST_CASE("Tree support toolpaths do not depend on thread scheduling", "[TreeSupport][Regression]")
{
    // Scaled up so that a layer holds enough nodes for the parallel range to be split. At stock
    // size it stays in one chunk and the order never varies.
    SECTION("overhang")            { sliced_twice_matches(scaled(TestMesh::overhang, 2.f), 0); }
    SECTION("bridge with hole")    { sliced_twice_matches(scaled(TestMesh::bridge_with_hole, 3.f), 0); }
    // Dropping every branch that cannot reach the bed leaves the survivors dense enough that the
    // neighbour merge fires in bulk.
    SECTION("on the build plate")  { sliced_twice_matches(scaled(TestMesh::overhang, 4.f), 1); }
    // Branches resting on the model are what put nodes in a part group other than 0, which is the
    // only way to reach the prune in the second pass. tree_hybrid additionally builds polygon
    // nodes, so it is the only style that exercises the overhang merge.
    SECTION("resting on the model") { sliced_twice_matches(two_tier_mesh(), 0); }
    SECTION("hybrid on the model")  { sliced_twice_matches(two_tier_mesh(), 0, "tree_hybrid"); }
}

// Prim breaks equal-distance ties by heap address. A 1 mm branch diameter puts neighbours close
// enough to tie, and an explicit line width pins max_move_dist, so the moved tie winner reaches
// the support toolpaths.
TEST_CASE("Tree support toolpaths do not depend on the MST tie order", "[TreeSupport][Regression]")
{
    sliced_twice_matches(two_tier_mesh(), 0, "tree_hybrid", {
        { "tree_support_branch_diameter", 1.0 },
        { "tree_support_branch_distance", 5.0 },
        { "tree_support_branch_angle",    40 },
        { "support_line_width",           0.4 },
    });
}

TEST_CASE("Top Z distance overriding the XY distance is off by default", "[TreeSupport]")
{
    REQUIRE_FALSE(DynamicPrintConfig::full_print_config().opt_bool("top_z_overrides_xy_distance"));
}

// With a 1 mm XY distance and a 0.2 mm top Z distance, the override brings the supports 0.8 mm closer
// to the walls. Hybrid trees support the wide plate with the same polygons as normal supports.
TEST_CASE("Top Z distance overriding the XY distance lets supports closer to the walls", "[TreeSupport]")
{
    const char *type  = "normal(auto)";
    const char *style = "default";
    SECTION("normal support") {}
    SECTION("hybrid tree support") { type = "tree(auto)"; style = "tree_hybrid"; }

    const double kept       = support_to_column_gap(type, style, false);
    const double overridden = support_to_column_gap(type, style, true);
    CAPTURE(kept, overridden);
    REQUIRE(kept < 100.); // without support near the column the comparison below is meaningless
    REQUIRE(overridden < kept - 0.4);
}

// Classic trees build their interface from the tree nodes instead of the overhang polygons, so the
// override reaches it through its own collision. Only the interface may get closer: the branches
// below keep the full XY distance.
TEST_CASE("Top Z distance overriding the XY distance lets the classic tree interface closer to the walls", "[TreeSupport]")
{
    const char *style = "tree_slim";
    SECTION("slim tree support") {}
    SECTION("strong tree support") { style = "tree_strong"; }

    const double xy_distance    = 1.0;
    const double top_z_distance = 0.2;
    const ClassicTreeGaps kept       = classic_tree_to_column_gaps(style, xy_distance, top_z_distance, false);
    const ClassicTreeGaps overridden = classic_tree_to_column_gaps(style, xy_distance, top_z_distance, true);
    CAPTURE(style, kept.interface, kept.branches, overridden.interface, overridden.branches);

    REQUIRE(kept.interface < 100.); // without an interface near the column the comparison below is meaningless
    REQUIRE(overridden.interface < kept.interface - 0.4);

    // Toolpaths are centre lines inside the support area, which already stays xy_distance away. The
    // margin only absorbs the chords that approximate the rounded corners of the offset outline.
    const double chord_margin = 0.05;
    REQUIRE(kept.branches >= xy_distance - chord_margin);
    REQUIRE(overridden.branches >= xy_distance - chord_margin);
}
