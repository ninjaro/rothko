#include "packing/geometry.hpp"
#include "packing/layout/reciprocal_layout.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

namespace packing {
namespace {

    TEST(
        reciprocal_layout_tests,
        places_the_supported_prefix_without_backtracking
    ) {
        const reciprocal_layout_result result = layout_reciprocal_rectangles(
            { .count = reciprocal_layout_count_limit }
        );

        EXPECT_EQ(result.status, reciprocal_layout_status::complete);
        EXPECT_EQ(result.rectangles.size(), reciprocal_layout_count_limit);
        EXPECT_TRUE(validate({ 1.0, 1.0 }, result.rectangles).valid);
        for (std::size_t index = 0; index < result.rectangles.size(); ++index) {
            EXPECT_EQ(result.rectangles[index].source_index, index + 1);
        }
    }

    TEST(reciprocal_layout_tests, is_deterministic) {
        const reciprocal_layout_request request { .count = 24 };
        const auto first = layout_reciprocal_rectangles(request);
        const auto second = layout_reciprocal_rectangles(request);
        ASSERT_EQ(first.rectangles.size(), second.rectangles.size());
        for (std::size_t index = 0; index < first.rectangles.size(); ++index) {
            EXPECT_DOUBLE_EQ(
                first.rectangles[index].x, second.rectangles[index].x
            );
            EXPECT_DOUBLE_EQ(
                first.rectangles[index].y, second.rectangles[index].y
            );
            EXPECT_EQ(
                first.rectangles[index].rotated,
                second.rectangles[index].rotated
            );
        }
    }

    TEST(reciprocal_layout_tests, reports_empty_invalid_and_bounded_requests) {
        EXPECT_EQ(
            layout_reciprocal_rectangles({ .count = 0 }).status,
            reciprocal_layout_status::complete
        );
        EXPECT_EQ(
            layout_reciprocal_rectangles({ .container = {}, .count = 1 })
                .status,
            reciprocal_layout_status::invalid_request
        );
        EXPECT_EQ(
            layout_reciprocal_rectangles({ .first_index = 0, .count = 1 })
                .status,
            reciprocal_layout_status::invalid_request
        );
        EXPECT_EQ(
            layout_reciprocal_rectangles(
                {
                    .count = 1,
                    .algorithm = static_cast<reciprocal_layout_algorithm>(99),
                }
            )
                .status,
            reciprocal_layout_status::invalid_request
        );
        EXPECT_EQ(
            layout_reciprocal_rectangles(
                { .count = reciprocal_layout_count_limit + 1 }
            )
                .status,
            reciprocal_layout_status::limit_exceeded
        );
        EXPECT_EQ(
            layout_reciprocal_rectangles(
                {
                    .first_index = std::numeric_limits<std::size_t>::max(),
                    .count = 1,
                }
            )
                .status,
            reciprocal_layout_status::invalid_request
        );
    }

    TEST(reciprocal_layout_tests, supports_a_finite_shifted_prefix) {
        const reciprocal_layout_request request {
            .container = { 2.0, 3.0 },
            .first_index = 8,
            .count = 12,
            .allow_rotation = false,
        };
        const auto result = layout_reciprocal_rectangles(request);
        EXPECT_EQ(result.status, reciprocal_layout_status::complete);
        EXPECT_TRUE(validate(request.container, result.rectangles).valid);
        ASSERT_FALSE(result.rectangles.empty());
        EXPECT_EQ(result.rectangles.front().source_index, 8U);
        EXPECT_FALSE(result.rectangles.front().rotated);
    }

} // namespace
} // namespace packing
