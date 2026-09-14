#include "packing/selection/compact_layout.hpp"

#include "packing/geometry.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <set>
#include <span>
#include <vector>

namespace {

using packing::bounds;
using packing::compact_layout_algorithm;
using packing::compact_layout_request;
using packing::extent;
using packing::rectangle;

constexpr double epsilon = 1e-9;

[[nodiscard]] bool
same_rectangle(const rectangle& lhs, const rectangle& rhs) noexcept {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.width == rhs.width
        && lhs.height == rhs.height && lhs.source_index == rhs.source_index
        && lhs.rotated == rhs.rotated;
}

void expect_same_rectangles(
    const std::span<const rectangle> lhs, const std::span<const rectangle> rhs
) {
    ASSERT_EQ(lhs.size(), rhs.size());
    for (std::size_t index = 0; index < lhs.size(); ++index) {
        EXPECT_TRUE(same_rectangle(lhs[index], rhs[index]));
    }
}

[[nodiscard]] std::vector<rectangle> line_candidates(const std::size_t count) {
    std::vector<rectangle> candidates;
    candidates.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        candidates.push_back(
            rectangle {
                .x = static_cast<double>(index) * 1.5,
                .y = 0.0,
                .width = 1.0,
                .height = 1.0,
                .source_index = 100 + index,
                .rotated = index % 2 != 0,
            }
        );
    }
    return candidates;
}

struct brute_force_key {
    double area { std::numeric_limits<double>::infinity() };
    double perimeter { std::numeric_limits<double>::infinity() };
};

void enumerate_brute_force(
    const std::span<const rectangle> candidates, const std::size_t need,
    const std::size_t next, std::vector<rectangle>& selected,
    brute_force_key& best
) {
    if (selected.size() == need) {
        const bounds box = packing::bounding_box(selected);
        const brute_force_key candidate {
            .area = box.area(),
            .perimeter = 2.0 * (box.width() + box.height()),
        };
        if (candidate.area < best.area
            || (candidate.area == best.area
                && candidate.perimeter < best.perimeter)) {
            best = candidate;
        }
        return;
    }

    const std::size_t remaining = need - selected.size();
    for (std::size_t index = next; index + remaining <= candidates.size();
         ++index) {
        selected.push_back(candidates[index]);
        enumerate_brute_force(candidates, need, index + 1, selected, best);
        selected.pop_back();
    }
}

[[nodiscard]] brute_force_key brute_force_minimum_box(
    const std::span<const rectangle> candidates, const std::size_t need
) {
    brute_force_key best;
    std::vector<rectangle> selected;
    selected.reserve(need);
    enumerate_brute_force(candidates, need, 0, selected, best);
    return best;
}

[[nodiscard]] double point_distance_squared(
    const double x, const double y, const rectangle& item
) noexcept {
    const double right = item.x + item.width;
    const double bottom = item.y + item.height;
    const double dx = x < item.x ? item.x - x : (x > right ? x - right : 0.0);
    const double dy = y < item.y ? item.y - y : (y > bottom ? y - bottom : 0.0);
    return dx * dx + dy * dy;
}

[[nodiscard]] double
dense_internal_void_score(const std::span<const rectangle> rectangles) {
    const bounds box = packing::bounding_box(rectangles);
    constexpr std::size_t axis_samples = 80;
    double total = 0.0;
    std::size_t empty_count = 0;
    for (std::size_t row = 0; row < axis_samples; ++row) {
        const double y = box.top
            + (static_cast<double>(row) + 0.5) * box.height()
                / static_cast<double>(axis_samples);
        for (std::size_t column = 0; column < axis_samples; ++column) {
            const double x = box.left
                + (static_cast<double>(column) + 0.5) * box.width()
                    / static_cast<double>(axis_samples);
            double nearest = std::numeric_limits<double>::infinity();
            for (const rectangle& item : rectangles) {
                nearest = std::min(nearest, point_distance_squared(x, y, item));
            }
            if (nearest > 0.0) {
                total += nearest;
                ++empty_count;
            }
        }
    }
    return empty_count == 0 ? 0.0 : total / static_cast<double>(empty_count);
}

[[nodiscard]] std::set<std::size_t>
source_indices(const std::span<const rectangle> rectangles) {
    std::set<std::size_t> result;
    for (const rectangle& item : rectangles) {
        result.insert(item.source_index);
    }
    return result;
}

TEST(CompactLayout, AutomaticDispatchHonorsBenchmarkBoundary) {
    const std::vector<rectangle> candidates = line_candidates(40);

    compact_layout_request request {
        .container = { 50.0, 2.0 },
        .candidates = candidates,
        .count = packing::compact_refinement_count_limit,
    };
    EXPECT_EQ(
        packing::select_compact_layout_algorithm(request),
        compact_layout_algorithm::void_refined
    );

    request.count = packing::compact_refinement_count_limit - 1;
    EXPECT_EQ(
        packing::select_compact_layout_algorithm(request),
        compact_layout_algorithm::exact_bounding_box
    );

    request.count = packing::compact_refinement_count_limit + 1;
    EXPECT_EQ(
        packing::select_compact_layout_algorithm(request),
        compact_layout_algorithm::exact_bounding_box
    );

    request.count = candidates.size();
    EXPECT_EQ(
        packing::select_compact_layout_algorithm(request),
        compact_layout_algorithm::automatic
    );

    request.count = 3;
    request.algorithm = compact_layout_algorithm::exact_bounding_box;
    EXPECT_EQ(
        packing::select_compact_layout_algorithm(request),
        compact_layout_algorithm::exact_bounding_box
    );
}

TEST(CompactLayout, DirectReturnPreservesEveryCandidateField) {
    const std::vector<rectangle> candidates {
        { 0.0, 0.0, 2.0, 1.0, 41, false },
        { 4.0, 3.0, 1.0, 2.0, 87, true },
    };
    const packing::compact_layout_result result
        = packing::select_compact_layout(
            compact_layout_request {
                .container = { 8.0, 8.0 },
                .candidates = candidates,
                .count = 3,
            }
        );

    EXPECT_EQ(result.algorithm, compact_layout_algorithm::automatic);
    expect_same_rectangles(result.rectangles, candidates);
}

TEST(CompactLayout, ExactBoundingBoxMatchesBruteForceOracle) {
    // A reproducible seed is intentional: failures in this oracle comparison
    // must be exactly replayable.
    std::mt19937 generator(0xC0FFEEu); // NOLINT(cert-msc51-cpp)
    std::vector<int> cells(36);
    for (int index = 0; index < static_cast<int>(cells.size()); ++index) {
        cells[static_cast<std::size_t>(index)] = index;
    }

    for (int case_index = 0; case_index < 40; ++case_index) {
        std::shuffle(cells.begin(), cells.end(), generator);
        std::vector<rectangle> candidates;
        for (std::size_t index = 0; index < 8; ++index) {
            const int cell = cells[index];
            candidates.push_back(
                rectangle {
                    .x = static_cast<double>(cell % 6) * 1.5,
                    .y = std::floor(static_cast<double>(cell) / 6.0) * 1.5,
                    .width = index % 3 == 0 ? 0.9 : 0.8,
                    .height = index % 4 == 0 ? 0.7 : 0.8,
                    .source_index = index + 300,
                    .rotated = index % 2 != 0,
                }
            );
        }

        constexpr std::size_t need = 4;
        const brute_force_key oracle
            = brute_force_minimum_box(candidates, need);
        const packing::compact_layout_result result
            = packing::select_compact_layout(
                compact_layout_request {
                    .container = { 10.0, 10.0 },
                    .candidates = candidates,
                    .count = need,
                    .algorithm = compact_layout_algorithm::exact_bounding_box,
                }
            );

        ASSERT_EQ(result.rectangles.size(), need);
        const bounds actual = packing::bounding_box(result.rectangles);
        EXPECT_NEAR(actual.area(), oracle.area, epsilon)
            << "case " << case_index;
        EXPECT_NEAR(
            2.0 * (actual.width() + actual.height()), oracle.perimeter, epsilon
        ) << "case "
          << case_index;
        EXPECT_TRUE(packing::validate({ 10.0, 10.0 }, result.rectangles).valid);
    }
}

TEST(CompactLayout, VoidRefinementPreservesMinimumBoxAndImprovesHoles) {
    const std::vector<rectangle> candidates {
        { 2.4, 4.8, 1.0, 1.0, 0, false }, { 3.6, 2.4, 1.0, 1.0, 1, true },
        { 4.8, 3.6, 1.0, 1.0, 2, false }, { 0.0, 3.6, 1.0, 1.0, 3, true },
        { 1.2, 0.0, 1.0, 1.0, 4, false }, { 6.0, 3.6, 1.0, 1.0, 5, true },
        { 3.6, 4.8, 1.0, 1.0, 6, false }, { 1.2, 3.6, 1.0, 1.0, 7, true },
        { 7.2, 0.0, 1.0, 1.0, 8, false }, { 4.8, 4.8, 1.0, 1.0, 9, true },
    };
    compact_layout_request request {
        .container = { 10.0, 8.0 },
        .candidates = candidates,
        .count = 5,
        .algorithm = compact_layout_algorithm::exact_bounding_box,
        .refinement_passes = 4,
        .maximum_samples = 2048,
    };
    const packing::compact_layout_result exact
        = packing::select_compact_layout(request);
    request.algorithm = compact_layout_algorithm::void_refined;
    const packing::compact_layout_result refined
        = packing::select_compact_layout(request);

    ASSERT_EQ(exact.rectangles.size(), request.count);
    ASSERT_EQ(refined.rectangles.size(), request.count);
    EXPECT_NEAR(
        packing::bounding_box(refined.rectangles).area(),
        packing::bounding_box(exact.rectangles).area(), epsilon
    );
    EXPECT_LT(
        dense_internal_void_score(refined.rectangles),
        dense_internal_void_score(exact.rectangles)
    );
    EXPECT_NE(
        source_indices(refined.rectangles), source_indices(exact.rectangles)
    );
    EXPECT_TRUE(packing::validate(request.container, refined.rectangles).valid);
}

TEST(CompactLayout, SelectionIsDeterministicAndPreservesMetadata) {
    const std::vector<rectangle> candidates {
        { 0.0, 0.0, 1.0, 1.0, 101, false }, { 2.0, 0.0, 1.0, 1.0, 102, true },
        { 4.0, 0.0, 1.0, 1.0, 103, false }, { 0.0, 2.0, 1.0, 1.0, 104, true },
        { 2.0, 2.0, 1.0, 1.0, 105, false }, { 4.0, 2.0, 1.0, 1.0, 106, true },
    };
    const compact_layout_request request {
        .container = { 8.0, 6.0 },
        .candidates = candidates,
        .count = 3,
        .algorithm = compact_layout_algorithm::void_refined,
        .refinement_passes = 3,
        .maximum_samples = 128,
    };

    const auto first = packing::select_compact_layout(request);
    const auto second = packing::select_compact_layout(request);
    expect_same_rectangles(first.rectangles, second.rectangles);
    for (const rectangle& selected : first.rectangles) {
        const auto source
            = std::ranges::find_if(candidates, [&](const rectangle& candidate) {
                  return candidate.source_index == selected.source_index;
              });
        ASSERT_NE(source, candidates.end());
        EXPECT_EQ(selected.width, source->width);
        EXPECT_EQ(selected.height, source->height);
        EXPECT_EQ(selected.rotated, source->rotated);
    }
}

TEST(CompactLayout, InvalidRequestsReturnNoRectangles) {
    const std::vector<rectangle> valid {
        { 0.0, 0.0, 1.0, 1.0, 0, false },
        { 2.0, 0.0, 1.0, 1.0, 1, false },
    };
    compact_layout_request request {
        .container = { 4.0, 4.0 },
        .candidates = valid,
        .count = 1,
        .algorithm = compact_layout_algorithm::exact_bounding_box,
    };

    request.container.width = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(packing::select_compact_layout(request).rectangles.empty());

    request.container = { 4.0, 4.0 };
    request.count = 0;
    EXPECT_TRUE(packing::select_compact_layout(request).rectangles.empty());

    request.count = 1;
    request.algorithm = static_cast<compact_layout_algorithm>(999);
    EXPECT_TRUE(packing::select_compact_layout(request).rectangles.empty());

    const std::vector<rectangle> overlapping {
        { 0.0, 0.0, 2.0, 2.0, 0, false },
        { 1.0, 1.0, 2.0, 2.0, 1, false },
    };
    request.candidates = overlapping;
    request.algorithm = compact_layout_algorithm::exact_bounding_box;
    EXPECT_TRUE(packing::select_compact_layout(request).rectangles.empty());

    request.candidates = valid;
    request.algorithm = compact_layout_algorithm::void_refined;
    request.maximum_samples = 0;
    EXPECT_TRUE(packing::select_compact_layout(request).rectangles.empty());
}

} // namespace
