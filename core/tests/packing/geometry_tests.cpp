#include "packing/geometry.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace packing {
namespace {

    TEST(geometry_tests, rejects_non_finite_or_non_positive_extents) {
        EXPECT_TRUE(is_positive_finite({ 10.0, 5.0 }));
        EXPECT_FALSE(is_positive_finite({ 0.0, 5.0 }));
        EXPECT_FALSE(is_positive_finite({ -1.0, 5.0 }));
        EXPECT_FALSE(is_positive_finite(
            { std::numeric_limits<double>::quiet_NaN(), 5.0 }
        ));
        EXPECT_FALSE(is_positive_finite(
            { 10.0, std::numeric_limits<double>::infinity() }
        ));
    }

    TEST(geometry_tests, touching_edges_do_not_overlap) {
        const rectangle left { 0.0, 0.0, 10.0, 10.0 };
        const rectangle touching { 10.0, 0.0, 4.0, 10.0 };
        const rectangle overlapping { 9.0, 0.0, 4.0, 10.0 };

        EXPECT_FALSE(intersects(left, touching));
        EXPECT_TRUE(intersects(left, overlapping));
    }

    TEST(geometry_tests, validates_bounds_overlap_and_non_finite_coordinates) {
        const std::vector valid {
            rectangle { 0.0, 0.0, 5.0, 10.0, 0, false },
            rectangle { 5.0, 0.0, 5.0, 10.0, 1, false },
        };
        EXPECT_TRUE(validate({ 10.0, 10.0 }, valid).valid);

        auto invalid = valid;
        invalid[1].x = 4.0;
        const validation_report overlap = validate({ 10.0, 10.0 }, invalid);
        EXPECT_FALSE(overlap.valid);
        EXPECT_EQ(overlap.overlap_count, 1U);

        invalid[1].x = std::numeric_limits<double>::quiet_NaN();
        const validation_report non_finite = validate({ 10.0, 10.0 }, invalid);
        EXPECT_FALSE(non_finite.valid);
        EXPECT_EQ(non_finite.out_of_bounds_count, 1U);
    }

    TEST(geometry_tests, computes_bounding_box) {
        const std::vector values {
            rectangle { 4.0, 3.0, 2.0, 5.0 },
            rectangle { 1.0, 7.0, 4.0, 2.0 },
        };
        const bounds box = bounding_box(values);
        EXPECT_DOUBLE_EQ(box.left, 1.0);
        EXPECT_DOUBLE_EQ(box.top, 3.0);
        EXPECT_DOUBLE_EQ(box.right, 6.0);
        EXPECT_DOUBLE_EQ(box.bottom, 9.0);
        EXPECT_DOUBLE_EQ(box.area(), 30.0);
    }

    TEST(geometry_tests, computes_point_to_rectangle_distance) {
        const rectangle item { 2.0, 3.0, 4.0, 5.0 };

        EXPECT_DOUBLE_EQ(squared_distance({ 4.0, 5.0 }, item), 0.0);
        EXPECT_DOUBLE_EQ(squared_distance({ 0.0, 0.0 }, item), 13.0);
        EXPECT_DOUBLE_EQ(squared_distance({ 8.0, 6.0 }, item), 4.0);
    }

    TEST(geometry_tests, translation_is_atomic_when_input_is_invalid) {
        std::vector values {
            rectangle { 1.0, 2.0, 3.0, 4.0 },
            rectangle { 5.0, 6.0, 7.0, 8.0 },
        };

        EXPECT_TRUE(translate(values, { 3.0, -1.0 }));
        EXPECT_DOUBLE_EQ(values[0].x, 4.0);
        EXPECT_DOUBLE_EQ(values[0].y, 1.0);
        EXPECT_DOUBLE_EQ(values[1].x, 8.0);
        EXPECT_DOUBLE_EQ(values[1].y, 5.0);

        const std::vector translated = values;
        EXPECT_FALSE(
            translate(values, { std::numeric_limits<double>::infinity(), 0.0 })
        );
        EXPECT_EQ(values[0].x, translated[0].x);
        EXPECT_EQ(values[1].x, translated[1].x);
    }

} // namespace
} // namespace packing
