#include "packing/geometry.hpp"
#include "packing/selection/spread_layout.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace packing {
namespace {

    std::vector<rectangle> grid_candidates() {
        std::vector<rectangle> result;
        constexpr double positions[] {
            0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0,
        };
        for (const double y : positions) {
            for (const double x : positions) {
                result.push_back(
                    {
                        .x = x,
                        .y = y,
                        .width = 10.0,
                        .height = 10.0,
                        .source_index = result.size(),
                    }
                );
            }
        }
        return result;
    }

    double point_distance_squared(
        const double x, const double y, const rectangle& item
    ) {
        const double dx = x < item.x
            ? item.x - x
            : (x > item.x + item.width ? x - item.x - item.width : 0.0);
        const double dy = y < item.y
            ? item.y - y
            : (y > item.y + item.height ? y - item.y - item.height : 0.0);
        return dx * dx + dy * dy;
    }

    double coverage_cost(const std::vector<rectangle>& rectangles) {
        double total = 0.0;
        for (int row = 0; row < 11; ++row) {
            for (int column = 0; column < 11; ++column) {
                const double x = static_cast<double>(column) * 10.0;
                const double y = static_cast<double>(row) * 10.0;
                double nearest = std::numeric_limits<double>::infinity();
                for (const rectangle& item : rectangles) {
                    nearest
                        = std::min(nearest, point_distance_squared(x, y, item));
                }
                total += nearest;
            }
        }
        return total;
    }

    TEST(spread_layout_tests, dispatcher_uses_the_measured_swap_bound) {
        const auto candidates = grid_candidates();
        EXPECT_EQ(
            select_spread_layout_algorithm(
                {
                    .container = { 100.0, 100.0 },
                    .candidates = candidates,
                    .count = spread_swap_dispatch_limit,
                }
            ),
            spread_layout_algorithm::coverage_with_swaps
        );
        EXPECT_EQ(
            select_spread_layout_algorithm(
                {
                    .container = { 100.0, 100.0 },
                    .candidates = candidates,
                    .count = spread_swap_dispatch_limit + 1,
                }
            ),
            spread_layout_algorithm::greedy_coverage
        );
    }

    TEST(spread_layout_tests, greedy_improves_whole_field_coverage) {
        const auto candidates = grid_candidates();
        const spread_layout_request request {
            .container = { 100.0, 100.0 },
            .candidates = candidates,
            .count = 4,
            .algorithm = spread_layout_algorithm::greedy_coverage,
            .maximum_samples = 512,
        };
        const auto result = select_spread_layout(request);
        ASSERT_EQ(result.rectangles.size(), 4U);
        EXPECT_TRUE(validate(request.container, result.rectangles).valid);

        const std::vector clustered(candidates.begin(), candidates.begin() + 4);
        EXPECT_LT(coverage_cost(result.rectangles), coverage_cost(clustered));
    }

    TEST(spread_layout_tests, translation_is_rigid_and_stays_in_bounds) {
        const std::vector candidates {
            rectangle { 0.0, 0.0, 10.0, 10.0, 0, false },
            rectangle { 12.0, 0.0, 10.0, 10.0, 1, false },
            rectangle { 0.0, 12.0, 10.0, 10.0, 2, false },
        };
        const spread_layout_request request {
            .container = { 100.0, 100.0 },
            .candidates = candidates,
            .count = candidates.size(),
            .maximum_samples = 256,
        };
        const auto result = select_spread_layout(request);
        ASSERT_EQ(result.rectangles.size(), candidates.size());
        EXPECT_TRUE(validate(request.container, result.rectangles).valid);

        const double dx = result.rectangles.front().x - candidates.front().x;
        const double dy = result.rectangles.front().y - candidates.front().y;
        EXPECT_NE(dx, 0.0);
        EXPECT_NE(dy, 0.0);
        for (std::size_t index = 0; index < candidates.size(); ++index) {
            EXPECT_DOUBLE_EQ(
                result.rectangles[index].x, candidates[index].x + dx
            );
            EXPECT_DOUBLE_EQ(
                result.rectangles[index].y, candidates[index].y + dy
            );
            EXPECT_EQ(
                result.rectangles[index].source_index,
                candidates[index].source_index
            );
        }
    }

    TEST(spread_layout_tests, automatic_and_swap_results_are_deterministic) {
        const auto candidates = grid_candidates();
        const spread_layout_request request {
            .container = { 100.0, 100.0 },
            .candidates = candidates,
            .count = 8,
            .swap_passes = 2,
            .maximum_samples = 512,
        };
        const auto first = select_spread_layout(request);
        const auto second = select_spread_layout(request);
        EXPECT_EQ(
            first.algorithm, spread_layout_algorithm::coverage_with_swaps
        );
        ASSERT_EQ(first.rectangles.size(), second.rectangles.size());
        for (std::size_t index = 0; index < first.rectangles.size(); ++index) {
            EXPECT_EQ(
                first.rectangles[index].source_index,
                second.rectangles[index].source_index
            );
            EXPECT_DOUBLE_EQ(
                first.rectangles[index].x, second.rectangles[index].x
            );
            EXPECT_DOUBLE_EQ(
                first.rectangles[index].y, second.rectangles[index].y
            );
        }
    }

    TEST(
        spread_layout_tests, rejects_invalid_requests_and_overlapping_candidates
    ) {
        auto candidates = grid_candidates();
        candidates[1].x = candidates[0].x;
        candidates[1].y = candidates[0].y;
        EXPECT_TRUE(select_spread_layout(
                        {
                            .container = { 100.0, 100.0 },
                            .candidates = candidates,
                            .count = 4,
                        }
        )
                        .rectangles.empty());
        EXPECT_TRUE(select_spread_layout(
                        {
                            .container = { 0.0, 100.0 },
                            .candidates = candidates,
                            .count = 4,
                        }
        )
                        .rectangles.empty());
        EXPECT_TRUE(select_spread_layout(
                        {
                            .container = { 100.0, 100.0 },
                            .candidates = candidates,
                            .count = 0,
                        }
        )
                        .rectangles.empty());
    }

} // namespace
} // namespace packing
