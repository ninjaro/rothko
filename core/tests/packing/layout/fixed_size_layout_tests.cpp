#include "packing/layout/fixed_size_layout.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <numeric>
#include <span>
#include <vector>

namespace packing {
namespace {

    constexpr double tolerance = 1e-9;

    void expect_complete_geometry(
        const fixed_size_layout_result& result,
        const fixed_size_layout_request& request
    ) {
        ASSERT_TRUE(result.complete(request.items.size()));
        EXPECT_TRUE(validate(result.content, result.rectangles).valid);
        std::vector<std::size_t> sorted_order = result.placement_order;
        std::ranges::sort(sorted_order);
        for (std::size_t index = 0; index < sorted_order.size(); ++index) {
            EXPECT_EQ(sorted_order[index], index);
            const rectangle& placed = result.rectangles[index];
            EXPECT_EQ(placed.source_index, index);
            EXPECT_DOUBLE_EQ(placed.width, request.items[index].size.width);
            EXPECT_DOUBLE_EQ(placed.height, request.items[index].size.height);
            if (request.policy == scroll_policy::horizontal) {
                EXPECT_LE(
                    placed.y + placed.height,
                    request.viewport.height + tolerance
                );
            } else if (request.policy == scroll_policy::vertical) {
                EXPECT_LE(
                    placed.x + placed.width, request.viewport.width + tolerance
                );
            }
        }
        ASSERT_FALSE(result.groups.empty());
        EXPECT_EQ(result.groups.front().begin, 0U);
        EXPECT_EQ(result.groups.back().end, result.placement_order.size());
    }

    TEST(FixedSizeLayoutDispatcher, UsesDocumentedRealtimeBoundary) {
        std::vector<fixed_size_item> at_limit(
            fixed_best_fit_dispatch_limit, fixed_size_item { { 100.0, 60.0 } }
        );
        std::vector<fixed_size_item> above_limit(
            fixed_best_fit_dispatch_limit + 1,
            fixed_size_item { { 100.0, 60.0 } }
        );
        fixed_size_layout_request request {
            .viewport = { 1280.0, 720.0 },
            .items = at_limit,
        };
        EXPECT_EQ(
            select_fixed_size_layout_algorithm(request),
            fixed_size_layout_algorithm::best_fit_decreasing_shelf
        );
        request.items = above_limit;
        EXPECT_EQ(
            select_fixed_size_layout_algorithm(request),
            fixed_size_layout_algorithm::preserve_order_shelf
        );
        request.algorithm
            = fixed_size_layout_algorithm::best_fit_decreasing_shelf;
        EXPECT_EQ(
            select_fixed_size_layout_algorithm(request),
            fixed_size_layout_algorithm::best_fit_decreasing_shelf
        );
    }

    TEST(FixedSizeLayoutDispatcher, NamesEveryPublicImplementation) {
        EXPECT_EQ(
            algorithm_name(fixed_size_layout_algorithm::automatic), "automatic"
        );
        EXPECT_EQ(
            algorithm_name(fixed_size_layout_algorithm::preserve_order_shelf),
            "preserve_order_shelf"
        );
        EXPECT_EQ(
            algorithm_name(
                fixed_size_layout_algorithm::best_fit_decreasing_shelf
            ),
            "best_fit_decreasing_shelf"
        );
        EXPECT_EQ(
            algorithm_name(static_cast<fixed_size_layout_algorithm>(999)),
            "unknown"
        );
    }

    TEST(FixedSizeLayoutValidation, RejectsInvalidInputWithoutPartialOutput) {
        const std::array items { fixed_size_item { { 100.0, 60.0 } } };
        fixed_size_layout_request request {
            .viewport = { 640.0, 480.0 },
            .items = items,
            .spacing = 6.0,
        };
        const auto expect_rejected = [](fixed_size_layout_request invalid) {
            const fixed_size_layout_result result
                = layout_fixed_size_rectangles(invalid);
            EXPECT_TRUE(result.rectangles.empty());
            EXPECT_TRUE(result.placement_order.empty());
            EXPECT_TRUE(result.groups.empty());
            EXPECT_EQ(result.content.width, 0.0);
            EXPECT_EQ(result.content.height, 0.0);
        };

        request.viewport.height = 0.0;
        expect_rejected(request);
        request.viewport = { 640.0, 480.0 };
        request.spacing = -1.0;
        expect_rejected(request);
        request.spacing = 6.0;
        request.policy = static_cast<scroll_policy>(999);
        expect_rejected(request);
        request.policy = scroll_policy::horizontal;
        request.algorithm = static_cast<fixed_size_layout_algorithm>(999);
        expect_rejected(request);

        const std::array invalid_items { fixed_size_item {
            { std::numeric_limits<double>::infinity(), 60.0 } } };
        request.algorithm = fixed_size_layout_algorithm::automatic;
        request.items = invalid_items;
        expect_rejected(request);
    }

    TEST(FixedSizeLayoutValidation, EmptyInputIsAValidVacuousLayout) {
        const std::span<const fixed_size_item> no_items;
        const fixed_size_layout_request request {
            .viewport = { 640.0, 480.0 },
            .items = no_items,
            .spacing = 6.0,
        };
        const fixed_size_layout_result result
            = layout_fixed_size_rectangles(request);
        EXPECT_TRUE(result.complete(0));
        EXPECT_DOUBLE_EQ(result.content.width, request.viewport.width);
        EXPECT_DOUBLE_EQ(result.content.height, request.viewport.height);
    }

    TEST(FixedSizeLayoutGeometry, PreservesExactSizesOnBothScrollAxes) {
        const std::array items {
            fixed_size_item { { 160.0, 90.0 } },
            fixed_size_item { { 90.0, 160.0 } },
            fixed_size_item { { 120.0, 90.0 } },
            fixed_size_item { { 80.0, 80.0 } },
        };
        for (const fixed_size_layout_algorithm algorithm : {
                 fixed_size_layout_algorithm::preserve_order_shelf,
                 fixed_size_layout_algorithm::best_fit_decreasing_shelf,
             }) {
            for (const scroll_policy policy : {
                     scroll_policy::horizontal,
                     scroll_policy::vertical,
                 }) {
                const fixed_size_layout_request request {
                    .viewport = { 360.0, 260.0 },
                    .items = items,
                    .policy = policy,
                    .spacing = 5.0,
                    .algorithm = algorithm,
                };
                const fixed_size_layout_result result
                    = layout_fixed_size_rectangles(request);
                EXPECT_EQ(result.algorithm, algorithm);
                expect_complete_geometry(result, request);
            }
        }
    }

    TEST(FixedSizeLayoutGeometry, RejectsAnItemOversizedOnTheBoundedAxis) {
        const std::array too_tall { fixed_size_item { { 40.0, 201.0 } } };
        const fixed_size_layout_request horizontal {
            .viewport = { 300.0, 200.0 },
            .items = too_tall,
            .policy = scroll_policy::horizontal,
        };
        EXPECT_FALSE(
            layout_fixed_size_rectangles(horizontal).complete(too_tall.size())
        );

        const std::array too_wide { fixed_size_item { { 301.0, 40.0 } } };
        fixed_size_layout_request vertical = horizontal;
        vertical.items = too_wide;
        vertical.policy = scroll_policy::vertical;
        EXPECT_FALSE(
            layout_fixed_size_rectangles(vertical).complete(too_wide.size())
        );
    }

    TEST(FixedSizeLayoutGeometry, DisabledPolicyRejectsALayoutThatCannotFit) {
        const std::array items {
            fixed_size_item { { 220.0, 220.0 } },
            fixed_size_item { { 220.0, 220.0 } },
        };
        const fixed_size_layout_request request {
            .viewport = { 300.0, 300.0 },
            .items = items,
            .policy = scroll_policy::disabled,
            .spacing = 5.0,
        };
        const fixed_size_layout_result result
            = layout_fixed_size_rectangles(request);
        EXPECT_FALSE(result.complete(items.size()));
        EXPECT_TRUE(result.rectangles.empty());
        EXPECT_FALSE(result.used_scroll);
    }

    TEST(FixedSizeLayoutGeometry, DisabledPolicyReturnsViewportBoundedOutput) {
        const std::array items {
            fixed_size_item { { 120.0, 80.0 } },
            fixed_size_item { { 80.0, 120.0 } },
            fixed_size_item { { 100.0, 70.0 } },
        };
        const fixed_size_layout_request request {
            .viewport = { 400.0, 300.0 },
            .items = items,
            .policy = scroll_policy::disabled,
            .spacing = 5.0,
        };
        const fixed_size_layout_result result
            = layout_fixed_size_rectangles(request);
        expect_complete_geometry(result, request);
        EXPECT_FALSE(result.used_scroll);
        EXPECT_DOUBLE_EQ(result.content.width, request.viewport.width);
        EXPECT_DOUBLE_EQ(result.content.height, request.viewport.height);
    }

    TEST(FixedSizeLayoutQuality, BestFitCanAvoidANextFitShelf) {
        const std::array items {
            fixed_size_item { { 100.0, 60.0 } },
            fixed_size_item { { 100.0, 50.0 } },
            fixed_size_item { { 100.0, 40.0 } },
            fixed_size_item { { 100.0, 50.0 } },
        };
        fixed_size_layout_request request {
            .viewport = { 180.0, 100.0 },
            .items = items,
            .policy = scroll_policy::horizontal,
            .algorithm = fixed_size_layout_algorithm::preserve_order_shelf,
        };
        const fixed_size_layout_result preserve
            = layout_fixed_size_rectangles(request);
        request.algorithm
            = fixed_size_layout_algorithm::best_fit_decreasing_shelf;
        const fixed_size_layout_result best_fit
            = layout_fixed_size_rectangles(request);
        expect_complete_geometry(preserve, request);
        expect_complete_geometry(best_fit, request);
        EXPECT_LT(best_fit.primary_extent, preserve.primary_extent);
        EXPECT_NE(best_fit.placement_order, preserve.placement_order);
    }

    TEST(FixedSizeLayoutDeterminism, StableTiesProduceIdenticalOutput) {
        const std::array items {
            fixed_size_item { { 100.0, 60.0 } },
            fixed_size_item { { 100.0, 60.0 } },
            fixed_size_item { { 100.0, 40.0 } },
            fixed_size_item { { 100.0, 40.0 } },
        };
        const fixed_size_layout_request request {
            .viewport = { 200.0, 100.0 },
            .items = items,
            .algorithm = fixed_size_layout_algorithm::best_fit_decreasing_shelf,
        };
        const fixed_size_layout_result first
            = layout_fixed_size_rectangles(request);
        const fixed_size_layout_result second
            = layout_fixed_size_rectangles(request);
        EXPECT_EQ(first.placement_order, second.placement_order);
        ASSERT_EQ(first.rectangles.size(), second.rectangles.size());
        for (std::size_t index = 0; index < first.rectangles.size(); ++index) {
            EXPECT_DOUBLE_EQ(
                first.rectangles[index].x, second.rectangles[index].x
            );
            EXPECT_DOUBLE_EQ(
                first.rectangles[index].y, second.rectangles[index].y
            );
        }
    }

} // namespace
} // namespace packing
