#include "packing/layout/ordered_layout.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace packing {
namespace {

    constexpr double tolerance = 1e-7;

    struct oracle_value {
        double extent { std::numeric_limits<double>::infinity() };
        double raggedness { std::numeric_limits<double>::infinity() };
        std::size_t groups { std::numeric_limits<std::size_t>::max() };
        bool feasible { false };
    };

    [[nodiscard]] std::pair<double, double>
    unit_size(const ordered_slot& slot) {
        const double aspect = slot.rotated ? 1.0 / slot.aspect : slot.aspect;
        return aspect >= 1.0 ? std::pair { aspect, 1.0 }
                             : std::pair { 1.0, 1.0 / aspect };
    }

    [[nodiscard]] bool
    overlap(const rectangle& lhs, const rectangle& rhs) noexcept {
        const double overlap_x = std::min(lhs.x + lhs.width, rhs.x + rhs.width)
            - std::max(lhs.x, rhs.x);
        const double overlap_y
            = std::min(lhs.y + lhs.height, rhs.y + rhs.height)
            - std::max(lhs.y, rhs.y);
        return overlap_x > tolerance && overlap_y > tolerance;
    }

    void expect_valid_layout(
        const ordered_layout_result& result,
        const ordered_layout_request& request
    ) {
        ASSERT_EQ(result.rectangles.size(), request.items.size());
        EXPECT_GT(result.common_short_side, 0.0);
        EXPECT_GE(result.content.width + tolerance, request.viewport.width);
        EXPECT_GE(result.content.height + tolerance, request.viewport.height);

        for (std::size_t index = 0; index < result.rectangles.size(); ++index) {
            const rectangle& placed = result.rectangles[index];
            const ordered_slot& slot = request.items[index];
            EXPECT_EQ(placed.source_index, index);
            EXPECT_EQ(placed.rotated, slot.rotated);
            EXPECT_TRUE(std::isfinite(placed.x));
            EXPECT_TRUE(std::isfinite(placed.y));
            EXPECT_TRUE(std::isfinite(placed.width));
            EXPECT_TRUE(std::isfinite(placed.height));
            EXPECT_GE(placed.x, -tolerance);
            EXPECT_GE(placed.y, -tolerance);
            EXPECT_GT(placed.width, 0.0);
            EXPECT_GT(placed.height, 0.0);
            EXPECT_LE(
                placed.x + placed.width, result.content.width + tolerance
            );
            EXPECT_LE(
                placed.y + placed.height, result.content.height + tolerance
            );

            const double effective_aspect
                = slot.rotated ? 1.0 / slot.aspect : slot.aspect;
            EXPECT_NEAR(
                placed.width / placed.height, effective_aspect, tolerance
            );

            if (request.policy == scroll_policy::horizontal) {
                EXPECT_LE(
                    placed.y + placed.height,
                    request.viewport.height + tolerance
                );
            } else if (request.policy == scroll_policy::vertical) {
                EXPECT_LE(
                    placed.x + placed.width, request.viewport.width + tolerance
                );
            } else {
                EXPECT_LE(
                    placed.x + placed.width, request.viewport.width + tolerance
                );
                EXPECT_LE(
                    placed.y + placed.height,
                    request.viewport.height + tolerance
                );
            }
        }

        for (std::size_t lhs = 0; lhs < result.rectangles.size(); ++lhs) {
            for (std::size_t rhs = lhs + 1; rhs < result.rectangles.size();
                 ++rhs) {
                EXPECT_FALSE(
                    overlap(result.rectangles[lhs], result.rectangles[rhs])
                );
            }
        }

        if (result.algorithm != ordered_layout_algorithm::equal_grid) {
            ASSERT_FALSE(result.groups.empty());
            EXPECT_EQ(result.groups.front().begin, 0U);
            EXPECT_EQ(result.groups.back().end, request.items.size());
            for (std::size_t index = 0; index < result.groups.size(); ++index) {
                EXPECT_LT(result.groups[index].begin, result.groups[index].end);
                if (index != 0) {
                    EXPECT_EQ(
                        result.groups[index - 1].end, result.groups[index].begin
                    );
                }
            }
        }
    }

    [[nodiscard]] oracle_value brute_partition(
        std::span<const ordered_slot> slots, extent viewport, shelf_axis axis,
        double short_side, double spacing, bool balanced
    ) {
        const std::size_t cut_count = slots.empty() ? 0 : slots.size() - 1;
        const std::size_t partition_count = std::size_t { 1 } << cut_count;
        const double bounded_cross
            = axis == shelf_axis::columns ? viewport.height : viewport.width;
        oracle_value best;

        for (std::size_t cuts = 0; cuts < partition_count; ++cuts) {
            std::size_t begin = 0;
            double extent_value = 0.0;
            double raggedness = 0.0;
            std::size_t group_count = 0;
            bool feasible = true;

            for (std::size_t end = 1; end <= slots.size(); ++end) {
                const bool cut = end == slots.size()
                    || ((cuts >> (end - 1)) & std::size_t { 1 }) != 0;
                if (!cut) {
                    continue;
                }

                double group_main = 0.0;
                double group_cross = 0.0;
                for (std::size_t index = begin; index < end; ++index) {
                    const auto [unit_width, unit_height]
                        = unit_size(slots[index]);
                    const double width = unit_width * short_side;
                    const double height = unit_height * short_side;
                    group_main = std::max(
                        group_main, axis == shelf_axis::columns ? width : height
                    );
                    group_cross += axis == shelf_axis::columns ? height : width;
                }
                group_cross += spacing * static_cast<double>(end - begin - 1);
                if (group_cross > bounded_cross + tolerance) {
                    feasible = false;
                    break;
                }

                if (group_count != 0) {
                    extent_value += spacing;
                }
                extent_value += group_main;
                const double slack = bounded_cross - group_cross;
                raggedness += slack * slack;
                ++group_count;
                begin = end;
            }

            if (!feasible) {
                continue;
            }

            const bool better_extent
                = !best.feasible || extent_value < best.extent - tolerance;
            const bool equal_extent = best.feasible
                && std::abs(extent_value - best.extent) <= tolerance;
            const bool better_raggedness = balanced && equal_extent
                && raggedness < best.raggedness - tolerance;
            const bool equal_raggedness = balanced && equal_extent
                && std::abs(raggedness - best.raggedness) <= tolerance;
            if (better_extent || better_raggedness
                || (equal_raggedness && group_count < best.groups)) {
                best = { extent_value, raggedness, group_count, true };
            }
        }
        return best;
    }

    TEST(OrderedLayoutDispatcher, UsesDocumentedConstantTimeBoundary) {
        std::vector<ordered_slot> at_limit(balanced_shelf_dispatch_limit);
        std::vector<ordered_slot> above_limit(
            balanced_shelf_dispatch_limit + 1
        );

        ordered_layout_request request;
        request.viewport = { 1280.0, 720.0 };
        request.items = at_limit;
        EXPECT_EQ(
            select_ordered_layout_algorithm(request),
            ordered_layout_algorithm::balanced_shelf
        );

        request.items = above_limit;
        EXPECT_EQ(
            select_ordered_layout_algorithm(request),
            ordered_layout_algorithm::greedy_shelf
        );

        request.algorithm = ordered_layout_algorithm::exact_shelf;
        EXPECT_EQ(
            select_ordered_layout_algorithm(request),
            ordered_layout_algorithm::exact_shelf
        );
    }

    TEST(OrderedLayoutDispatcher, NamesEveryPublicImplementation) {
        EXPECT_EQ(
            algorithm_name(ordered_layout_algorithm::automatic), "automatic"
        );
        EXPECT_EQ(
            algorithm_name(ordered_layout_algorithm::equal_grid), "equal_grid"
        );
        EXPECT_EQ(
            algorithm_name(ordered_layout_algorithm::greedy_shelf),
            "greedy_shelf"
        );
        EXPECT_EQ(
            algorithm_name(ordered_layout_algorithm::exact_shelf), "exact_shelf"
        );
        EXPECT_EQ(
            algorithm_name(ordered_layout_algorithm::balanced_shelf),
            "balanced_shelf"
        );
        EXPECT_EQ(
            algorithm_name(static_cast<ordered_layout_algorithm>(999)),
            "unknown"
        );
    }

    TEST(OrderedLayoutValidation, RejectsInvalidDataWithoutPartialOutput) {
        const std::array valid_slots { ordered_slot { 16.0 / 9.0, false } };
        ordered_layout_request request {
            { 640.0, 480.0 },
            valid_slots,
            scroll_policy::horizontal,
            100.0,
            6.0,
            ordered_layout_algorithm::balanced_shelf
        };

        const auto expect_rejected = [](const ordered_layout_request& invalid) {
            const ordered_layout_result result = layout_ordered_slots(invalid);
            EXPECT_TRUE(result.rectangles.empty());
            EXPECT_TRUE(result.groups.empty());
            EXPECT_EQ(result.content.width, 0.0);
            EXPECT_EQ(result.content.height, 0.0);
        };

        request.viewport.width = 0.0;
        expect_rejected(request);
        request.viewport = { 640.0, std::numeric_limits<double>::infinity() };
        expect_rejected(request);
        request.viewport = { 640.0, 480.0 };
        request.spacing = -1.0;
        expect_rejected(request);
        request.spacing = 6.0;
        request.minimum_short_side = std::numeric_limits<double>::quiet_NaN();
        expect_rejected(request);

        const std::array zero_aspect { ordered_slot { 0.0, true } };
        request.minimum_short_side = 100.0;
        request.items = zero_aspect;
        expect_rejected(request);

        const std::array nan_aspect { ordered_slot {
            std::numeric_limits<double>::quiet_NaN(), false } };
        request.items = nan_aspect;
        expect_rejected(request);

        const std::array underflowing_aspect { ordered_slot {
            std::numeric_limits<double>::denorm_min(), false } };
        request.items = underflowing_aspect;
        expect_rejected(request);
    }

    TEST(OrderedLayoutValidation, EmptyLayoutIsAValidVacuousResult) {
        const std::span<const ordered_slot> no_slots;
        const ordered_layout_request request {
            { 640.0, 480.0 },
            no_slots,
            scroll_policy::horizontal,
            100.0,
            6.0,
            ordered_layout_algorithm::automatic
        };
        const ordered_layout_result result = layout_ordered_slots(request);
        EXPECT_TRUE(result.rectangles.empty());
        EXPECT_EQ(result.content.width, request.viewport.width);
        EXPECT_EQ(result.content.height, request.viewport.height);
        EXPECT_TRUE(result.threshold_met);
        EXPECT_EQ(result.algorithm, ordered_layout_algorithm::balanced_shelf);
    }

    TEST(OrderedLayoutGeometry, PreservesOrderAndBoundsForEveryShelfMode) {
        const std::array slots {
            ordered_slot { 16.0 / 9.0, false },
            ordered_slot { 16.0 / 9.0, true },
            ordered_slot { 4.0 / 3.0, false },
            ordered_slot { 1.0, false },
            ordered_slot { 3.0 / 4.0, true },
            ordered_slot { 9.0 / 16.0, false },
        };

        for (const ordered_layout_algorithm algorithm : {
                 ordered_layout_algorithm::greedy_shelf,
                 ordered_layout_algorithm::exact_shelf,
                 ordered_layout_algorithm::balanced_shelf,
             }) {
            for (const scroll_policy policy : {
                     scroll_policy::horizontal,
                     scroll_policy::vertical,
                     scroll_policy::disabled,
                 }) {
                const ordered_layout_request request {
                    { 720.0, 480.0 }, slots, policy, 110.0, 5.0, algorithm
                };
                const ordered_layout_result result
                    = layout_ordered_slots(request);
                EXPECT_EQ(result.algorithm, algorithm);
                EXPECT_EQ(
                    result.axis,
                    policy == scroll_policy::vertical ? shelf_axis::rows
                        : policy == scroll_policy::horizontal
                        ? shelf_axis::columns
                        : result.axis
                );
                if (policy == scroll_policy::disabled) {
                    EXPECT_FALSE(result.used_scroll);
                    EXPECT_DOUBLE_EQ(
                        result.content.width, request.viewport.width
                    );
                    EXPECT_DOUBLE_EQ(
                        result.content.height, request.viewport.height
                    );
                }
                expect_valid_layout(result, request);
            }
        }
    }

    TEST(OrderedLayoutGeometry, RotationIsAnInputStateAndRecomputesTheLayout) {
        const std::array unrotated { ordered_slot { 2.0, false } };
        const std::array rotated { ordered_slot { 2.0, true } };
        const ordered_layout_request before_request {
            { 400.0, 200.0 },
            unrotated,
            scroll_policy::horizontal,
            0.0,
            0.0,
            ordered_layout_algorithm::exact_shelf
        };
        const ordered_layout_request after_request {
            { 400.0, 200.0 },
            rotated,
            scroll_policy::horizontal,
            0.0,
            0.0,
            ordered_layout_algorithm::exact_shelf
        };

        const ordered_layout_result before
            = layout_ordered_slots(before_request);
        const ordered_layout_result after = layout_ordered_slots(after_request);
        ASSERT_EQ(before.rectangles.size(), 1U);
        ASSERT_EQ(after.rectangles.size(), 1U);
        EXPECT_NEAR(before.common_short_side, 200.0, tolerance);
        EXPECT_NEAR(before.rectangles[0].width, 400.0, tolerance);
        EXPECT_NEAR(before.rectangles[0].height, 200.0, tolerance);
        EXPECT_NEAR(after.common_short_side, 100.0, tolerance);
        EXPECT_NEAR(after.rectangles[0].width, 100.0, tolerance);
        EXPECT_NEAR(after.rectangles[0].height, 200.0, tolerance);
        EXPECT_FALSE(before.rectangles[0].rotated);
        EXPECT_TRUE(after.rectangles[0].rotated);
    }

    TEST(OrderedLayoutThreshold, ScrollsOnlyAfterNoScrollScaleMissesThreshold) {
        const std::array slots { ordered_slot { 1.0, false },
                                 ordered_slot { 1.0, false } };
        const ordered_layout_request request {
            { 100.0, 100.0 },
            slots,
            scroll_policy::horizontal,
            80.0,
            5.0,
            ordered_layout_algorithm::balanced_shelf
        };

        const ordered_layout_result result = layout_ordered_slots(request);
        expect_valid_layout(result, request);
        EXPECT_NEAR(result.common_short_side, 80.0, tolerance);
        EXPECT_TRUE(result.threshold_met);
        EXPECT_TRUE(result.used_scroll);
        EXPECT_NEAR(result.primary_extent, 165.0, tolerance);
    }

    TEST(
        OrderedLayoutThreshold, ReportsWhenBoundedCrossAxisCannotMeetThreshold
    ) {
        const std::array slots { ordered_slot { 1.0, false },
                                 ordered_slot { 1.0, false } };
        const ordered_layout_request request {
            { 100.0, 100.0 },
            slots,
            scroll_policy::horizontal,
            150.0,
            5.0,
            ordered_layout_algorithm::greedy_shelf
        };

        const ordered_layout_result result = layout_ordered_slots(request);
        expect_valid_layout(result, request);
        EXPECT_NEAR(result.common_short_side, 100.0, tolerance);
        EXPECT_FALSE(result.threshold_met);
        EXPECT_TRUE(result.used_scroll);
        EXPECT_NEAR(result.primary_extent, 205.0, tolerance);
    }

    TEST(OrderedLayoutEqualGrid, KeepsScrollCrossAxisBounded) {
        const std::array slots {
            ordered_slot { 16.0 / 9.0, false },
            ordered_slot { 9.0 / 16.0, false },
            ordered_slot { 1.0, false },
        };

        for (const scroll_policy policy :
             { scroll_policy::horizontal, scroll_policy::vertical }) {
            const ordered_layout_request request {
                { 100.0, 60.0 }, slots, policy,
                100.0,           4.0,   ordered_layout_algorithm::equal_grid
            };
            const ordered_layout_result result = layout_ordered_slots(request);
            EXPECT_EQ(result.algorithm, ordered_layout_algorithm::equal_grid);
            EXPECT_FALSE(result.threshold_met);
            expect_valid_layout(result, request);
        }
    }

    TEST(OrderedLayoutExactness, ExactAndBalancedMatchIndependentSmallOracle) {
        constexpr std::array aspects {
            16.0 / 9.0, 9.0 / 16.0, 4.0 / 3.0, 3.0 / 4.0, 1.0,
        };
        constexpr double short_side = 31.0;
        constexpr double spacing = 3.0;

        for (std::size_t count = 1; count <= 8; ++count) {
            std::vector<ordered_slot> slots;
            slots.reserve(count);
            for (std::size_t index = 0; index < count; ++index) {
                slots.push_back(
                    { aspects[index % aspects.size()], index % 3 == 1 }
                );
            }

            for (const shelf_axis axis :
                 { shelf_axis::columns, shelf_axis::rows }) {
                const extent viewport = axis == shelf_axis::columns
                    ? extent { 1.0, 110.0 }
                    : extent { 110.0, 1.0 };
                const scroll_policy policy = axis == shelf_axis::columns
                    ? scroll_policy::horizontal
                    : scroll_policy::vertical;

                for (const ordered_layout_algorithm algorithm : {
                         ordered_layout_algorithm::exact_shelf,
                         ordered_layout_algorithm::balanced_shelf,
                     }) {
                    const bool balanced
                        = algorithm == ordered_layout_algorithm::balanced_shelf;
                    const oracle_value oracle = brute_partition(
                        slots, viewport, axis, short_side, spacing, balanced
                    );
                    ASSERT_TRUE(oracle.feasible);

                    const ordered_layout_request request {
                        viewport, slots, policy, short_side, spacing, algorithm
                    };
                    const ordered_layout_result result
                        = layout_ordered_slots(request);
                    expect_valid_layout(result, request);
                    EXPECT_NEAR(
                        result.common_short_side, short_side, tolerance
                    );
                    EXPECT_NEAR(
                        result.primary_extent, oracle.extent, tolerance
                    );
                    if (balanced) {
                        EXPECT_NEAR(
                            result.raggedness, oracle.raggedness, tolerance
                        );
                        EXPECT_EQ(result.groups.size(), oracle.groups);
                    }
                }
            }
        }
    }

    TEST(OrderedLayoutDeterminism, RepeatedCallsReturnIdenticalGeometry) {
        const std::array slots {
            ordered_slot { 16.0 / 9.0, false },
            ordered_slot { 9.0 / 16.0, false },
            ordered_slot { 4.0 / 3.0, true },
            ordered_slot { 1.0, false },
        };
        const ordered_layout_request request {
            { 640.0, 360.0 },
            slots,
            scroll_policy::horizontal,
            140.0,
            6.0,
            ordered_layout_algorithm::balanced_shelf
        };

        const ordered_layout_result first = layout_ordered_slots(request);
        const ordered_layout_result second = layout_ordered_slots(request);
        ASSERT_EQ(first.rectangles.size(), second.rectangles.size());
        ASSERT_EQ(first.groups.size(), second.groups.size());
        EXPECT_DOUBLE_EQ(first.common_short_side, second.common_short_side);
        EXPECT_DOUBLE_EQ(first.primary_extent, second.primary_extent);
        EXPECT_DOUBLE_EQ(first.raggedness, second.raggedness);
        EXPECT_EQ(first.axis, second.axis);
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
            EXPECT_EQ(
                first.rectangles[index].source_index,
                second.rectangles[index].source_index
            );
        }
        for (std::size_t index = 0; index < first.groups.size(); ++index) {
            EXPECT_EQ(first.groups[index].begin, second.groups[index].begin);
            EXPECT_EQ(first.groups[index].end, second.groups[index].end);
        }
    }

} // namespace
} // namespace packing
