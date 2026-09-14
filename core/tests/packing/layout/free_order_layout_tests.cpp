#include "packing/layout/free_order_layout.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <span>
#include <vector>

namespace packing {
namespace {

    constexpr double tolerance = 1e-7;

    void expect_complete_geometry(
        const free_order_layout_result& result,
        const free_order_layout_request& request
    ) {
        ASSERT_TRUE(result.complete(request.items.size()));
        ASSERT_EQ(result.rectangles.size(), request.items.size());
        EXPECT_TRUE(validate(result.content, result.rectangles).valid);

        std::vector<std::size_t> sorted_order = result.placement_order;
        std::ranges::sort(sorted_order);
        for (std::size_t index = 0; index < sorted_order.size(); ++index) {
            EXPECT_EQ(sorted_order[index], index);
        }

        for (std::size_t index = 0; index < result.rectangles.size(); ++index) {
            const rectangle& placed = result.rectangles[index];
            const ordered_slot& item = request.items[index];
            EXPECT_EQ(placed.source_index, index);
            EXPECT_EQ(placed.rotated, item.rotated);
            const double aspect
                = item.rotated ? 1.0 / item.aspect : item.aspect;
            EXPECT_NEAR(placed.width / placed.height, aspect, tolerance);
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

        if (request.policy == scroll_policy::disabled) {
            EXPECT_FALSE(result.used_scroll);
            EXPECT_DOUBLE_EQ(result.content.width, request.viewport.width);
            EXPECT_DOUBLE_EQ(result.content.height, request.viewport.height);
        }

        ASSERT_FALSE(result.groups.empty());
        EXPECT_EQ(result.groups.front().begin, 0U);
        EXPECT_EQ(result.groups.back().end, result.placement_order.size());
        for (std::size_t index = 0; index < result.groups.size(); ++index) {
            EXPECT_LT(result.groups[index].begin, result.groups[index].end);
            if (index != 0) {
                EXPECT_EQ(
                    result.groups[index - 1].end, result.groups[index].begin
                );
            }
        }
    }

    TEST(FreeOrderLayoutDispatcher, UsesDocumentedRealtimeBoundary) {
        std::vector<ordered_slot> at_limit(
            free_order_multistart_dispatch_limit
        );
        std::vector<ordered_slot> above_limit(
            free_order_multistart_dispatch_limit + 1
        );
        free_order_layout_request request {
            .viewport = { 1280.0, 720.0 },
            .items = at_limit,
        };
        EXPECT_EQ(
            select_free_order_layout_algorithm(request),
            free_order_layout_algorithm::multistart_shelf
        );
        request.items = above_limit;
        EXPECT_EQ(
            select_free_order_layout_algorithm(request),
            free_order_layout_algorithm::sorted_shelf
        );
        request.algorithm = free_order_layout_algorithm::sorted_shelf;
        EXPECT_EQ(
            select_free_order_layout_algorithm(request),
            free_order_layout_algorithm::sorted_shelf
        );
    }

    TEST(FreeOrderLayoutDispatcher, NamesEveryPublicImplementation) {
        EXPECT_EQ(
            algorithm_name(free_order_layout_algorithm::automatic), "automatic"
        );
        EXPECT_EQ(
            algorithm_name(free_order_layout_algorithm::sorted_shelf),
            "sorted_shelf"
        );
        EXPECT_EQ(
            algorithm_name(free_order_layout_algorithm::multistart_shelf),
            "multistart_shelf"
        );
        EXPECT_EQ(
            algorithm_name(static_cast<free_order_layout_algorithm>(999)),
            "unknown"
        );
    }

    TEST(FreeOrderLayoutValidation, RejectsInvalidInputWithoutPartialOutput) {
        const std::array slots { ordered_slot { 16.0 / 9.0, false } };
        free_order_layout_request request {
            .viewport = { 640.0, 480.0 },
            .items = slots,
            .minimum_short_side = 100.0,
            .spacing = 6.0,
        };
        const auto expect_rejected = [](free_order_layout_request invalid) {
            const free_order_layout_result result
                = layout_free_order_slots(invalid);
            EXPECT_TRUE(result.rectangles.empty());
            EXPECT_TRUE(result.placement_order.empty());
            EXPECT_TRUE(result.groups.empty());
            EXPECT_EQ(result.content.width, 0.0);
            EXPECT_EQ(result.content.height, 0.0);
        };

        request.viewport.width = 0.0;
        expect_rejected(request);
        request.viewport = { 640.0, 480.0 };
        request.minimum_short_side = -1.0;
        expect_rejected(request);
        request.minimum_short_side = 100.0;
        request.spacing = std::numeric_limits<double>::infinity();
        expect_rejected(request);
        request.spacing = 6.0;
        request.policy = static_cast<scroll_policy>(999);
        expect_rejected(request);
        request.policy = scroll_policy::horizontal;
        request.algorithm = static_cast<free_order_layout_algorithm>(999);
        expect_rejected(request);

        const std::array invalid_slots { ordered_slot {
            std::numeric_limits<double>::denorm_min(), true } };
        request.algorithm = free_order_layout_algorithm::automatic;
        request.items = invalid_slots;
        expect_rejected(request);
    }

    TEST(FreeOrderLayoutValidation, EmptyInputIsAValidVacuousLayout) {
        const std::span<const ordered_slot> no_items;
        const free_order_layout_request request {
            .viewport = { 640.0, 480.0 },
            .items = no_items,
            .minimum_short_side = 100.0,
            .spacing = 6.0,
        };
        const free_order_layout_result result
            = layout_free_order_slots(request);
        EXPECT_TRUE(result.complete(0));
        EXPECT_DOUBLE_EQ(result.content.width, request.viewport.width);
        EXPECT_DOUBLE_EQ(result.content.height, request.viewport.height);
        EXPECT_TRUE(result.threshold_met);
    }

    TEST(FreeOrderLayoutGeometry, MapsPermutationBackToOriginalSourceIndices) {
        const std::array slots {
            ordered_slot { 16.0 / 9.0, false },
            ordered_slot { 9.0 / 16.0, false },
            ordered_slot { 4.0 / 3.0, false },
            ordered_slot { 1.0, false },
            ordered_slot { 21.0 / 9.0, true },
            ordered_slot { 3.0 / 4.0, true },
        };
        for (const free_order_layout_algorithm algorithm : {
                 free_order_layout_algorithm::sorted_shelf,
                 free_order_layout_algorithm::multistart_shelf,
             }) {
            for (const scroll_policy policy : {
                     scroll_policy::horizontal,
                     scroll_policy::vertical,
                     scroll_policy::disabled,
                 }) {
                const free_order_layout_request request {
                    .viewport = { 720.0, 480.0 },
                    .items = slots,
                    .policy = policy,
                    .minimum_short_side = 110.0,
                    .spacing = 5.0,
                    .algorithm = algorithm,
                };
                const free_order_layout_result result
                    = layout_free_order_slots(request);
                EXPECT_EQ(result.algorithm, algorithm);
                expect_complete_geometry(result, request);
            }
        }
    }

    TEST(FreeOrderLayoutQuality, MultistartIsNeverWorseThanItsSortedSeed) {
        const std::array slots {
            ordered_slot { 16.0 / 9.0, false },
            ordered_slot { 9.0 / 16.0, false },
            ordered_slot { 2.4, false },
            ordered_slot { 1.0, false },
            ordered_slot { 3.0 / 4.0, false },
            ordered_slot { 4.0 / 3.0, true },
            ordered_slot { 2.0, false },
            ordered_slot { 0.5, false },
        };
        free_order_layout_request request {
            .viewport = { 900.0, 520.0 },
            .items = slots,
            .policy = scroll_policy::horizontal,
            .minimum_short_side = 150.0,
            .spacing = 6.0,
            .algorithm = free_order_layout_algorithm::sorted_shelf,
        };
        const free_order_layout_result sorted
            = layout_free_order_slots(request);
        request.algorithm = free_order_layout_algorithm::multistart_shelf;
        const free_order_layout_result multistart
            = layout_free_order_slots(request);
        expect_complete_geometry(sorted, request);
        expect_complete_geometry(multistart, request);
        EXPECT_GE(
            multistart.common_short_side + tolerance, sorted.common_short_side
        );
        if (std::abs(multistart.common_short_side - sorted.common_short_side)
            <= tolerance) {
            EXPECT_LE(
                multistart.primary_extent, sorted.primary_extent + tolerance
            );
        }
    }

    TEST(FreeOrderLayoutDeterminism, StableTiesProduceIdenticalOutput) {
        const std::array slots {
            ordered_slot { 16.0 / 9.0, false },
            ordered_slot { 16.0 / 9.0, false },
            ordered_slot { 9.0 / 16.0, false },
            ordered_slot { 9.0 / 16.0, false },
        };
        const free_order_layout_request request {
            .viewport = { 640.0, 360.0 },
            .items = slots,
            .minimum_short_side = 120.0,
            .spacing = 4.0,
            .algorithm = free_order_layout_algorithm::multistart_shelf,
        };
        const free_order_layout_result first = layout_free_order_slots(request);
        const free_order_layout_result second
            = layout_free_order_slots(request);
        EXPECT_EQ(first.placement_order, second.placement_order);
        ASSERT_EQ(first.rectangles.size(), second.rectangles.size());
        for (std::size_t index = 0; index < first.rectangles.size(); ++index) {
            EXPECT_DOUBLE_EQ(
                first.rectangles[index].x, second.rectangles[index].x
            );
            EXPECT_DOUBLE_EQ(
                first.rectangles[index].y, second.rectangles[index].y
            );
            EXPECT_DOUBLE_EQ(
                first.rectangles[index].width, second.rectangles[index].width
            );
            EXPECT_DOUBLE_EQ(
                first.rectangles[index].height, second.rectangles[index].height
            );
        }
    }

} // namespace
} // namespace packing
