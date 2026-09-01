#include "packing/compact_layout.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>
#include <span>
#include <utility>
#include <vector>

namespace packing {
namespace {

    constexpr double sample_step_fraction = 0.35;
    constexpr std::size_t absolute_sample_limit = 8192;
    constexpr std::size_t absolute_refinement_pass_limit = 16;

    struct selection_key {
        double area { std::numeric_limits<double>::infinity() };
        double perimeter { std::numeric_limits<double>::infinity() };
        std::vector<std::size_t> indices;
        bool valid { false };
    };

    struct void_key {
        double mean_squared_distance {
            std::numeric_limits<double>::infinity()
        };
        double maximum_squared_distance {
            std::numeric_limits<double>::infinity()
        };
        double area { std::numeric_limits<double>::infinity() };
        bool valid { false };
    };

    [[nodiscard]] bool
    is_known(const compact_layout_algorithm algorithm) noexcept {
        switch (algorithm) {
        case compact_layout_algorithm::automatic:
        case compact_layout_algorithm::nearest_center:
        case compact_layout_algorithm::exact_bounding_box:
        case compact_layout_algorithm::void_refined:
            return true;
        }
        return false;
    }

    [[nodiscard]] bool
    request_geometry_is_valid(const compact_layout_request& request) noexcept {
        return is_positive_finite(request.container)
            && validate(request.container, request.candidates).valid;
    }

    [[nodiscard]] std::vector<rectangle>
    copy_candidates(const std::span<const rectangle> candidates) {
        return { candidates.begin(), candidates.end() };
    }

    [[nodiscard]] bounds bounds_for_indices(
        const std::span<const rectangle> candidates,
        const std::span<const std::size_t> indices
    ) noexcept {
        if (indices.empty()) {
            return {};
        }

        bounds result {
            .left = std::numeric_limits<double>::infinity(),
            .top = std::numeric_limits<double>::infinity(),
            .right = -std::numeric_limits<double>::infinity(),
            .bottom = -std::numeric_limits<double>::infinity(),
        };
        for (const std::size_t index : indices) {
            const rectangle& item = candidates[index];
            result.left = std::min(result.left, item.x);
            result.top = std::min(result.top, item.y);
            result.right = std::max(result.right, item.x + item.width);
            result.bottom = std::max(result.bottom, item.y + item.height);
        }
        return result;
    }

    [[nodiscard]] bool better_selection_key(
        const selection_key& candidate, const selection_key& current
    ) noexcept {
        if (!candidate.valid) {
            return false;
        }
        if (!current.valid) {
            return true;
        }
        if (candidate.area != current.area) {
            return candidate.area < current.area;
        }
        if (candidate.perimeter != current.perimeter) {
            return candidate.perimeter < current.perimeter;
        }
        return candidate.indices < current.indices;
    }

    [[nodiscard]] std::vector<std::size_t>
    select_nearest_center_indices(const compact_layout_request& request) {
        std::vector<std::size_t> indices(request.candidates.size());
        std::iota(indices.begin(), indices.end(), std::size_t { 0 });

        const double center_x = request.container.width / 2.0;
        const double center_y = request.container.height / 2.0;
        std::ranges::stable_sort(
            indices, [&](const std::size_t lhs, const std::size_t rhs) {
                const rectangle& lhs_item = request.candidates[lhs];
                const rectangle& rhs_item = request.candidates[rhs];
                const double lhs_dx
                    = lhs_item.x + lhs_item.width / 2.0 - center_x;
                const double lhs_dy
                    = lhs_item.y + lhs_item.height / 2.0 - center_y;
                const double rhs_dx
                    = rhs_item.x + rhs_item.width / 2.0 - center_x;
                const double rhs_dy
                    = rhs_item.y + rhs_item.height / 2.0 - center_y;
                const double lhs_distance = lhs_dx * lhs_dx + lhs_dy * lhs_dy;
                const double rhs_distance = rhs_dx * rhs_dx + rhs_dy * rhs_dy;
                if (lhs_distance != rhs_distance) {
                    return lhs_distance < rhs_distance;
                }
                return lhs < rhs;
            }
        );
        indices.resize(request.count);
        std::ranges::sort(indices);
        return indices;
    }

    [[nodiscard]] std::vector<std::size_t> select_exact_bounding_box_indices(
        const std::span<const rectangle> candidates, const std::size_t count
    ) {
        if (count == 0) {
            return {};
        }
        if (candidates.size() <= count) {
            std::vector<std::size_t> all(candidates.size());
            std::iota(all.begin(), all.end(), std::size_t { 0 });
            return all;
        }

        std::vector<double> left_edges;
        std::vector<double> right_edges;
        left_edges.reserve(candidates.size());
        right_edges.reserve(candidates.size());
        for (const rectangle& item : candidates) {
            left_edges.push_back(item.x);
            right_edges.push_back(item.x + item.width);
        }
        std::ranges::sort(left_edges);
        std::ranges::sort(right_edges);
        left_edges.erase(
            std::ranges::unique(left_edges).begin(), left_edges.end()
        );
        right_edges.erase(
            std::ranges::unique(right_edges).begin(), right_edges.end()
        );

        selection_key best;
        std::vector<std::size_t> eligible;
        eligible.reserve(candidates.size());

        for (const double left : left_edges) {
            std::size_t left_eligible_count = 0;
            for (const rectangle& item : candidates) {
                if (item.x >= left) {
                    ++left_eligible_count;
                }
            }
            if (left_eligible_count < count) {
                continue;
            }

            for (const double right : right_edges) {
                if (right <= left) {
                    continue;
                }

                eligible.clear();
                for (std::size_t index = 0; index < candidates.size();
                     ++index) {
                    const rectangle& item = candidates[index];
                    if (item.x >= left && item.x + item.width <= right) {
                        eligible.push_back(index);
                    }
                }
                if (eligible.size() < count) {
                    continue;
                }

                std::ranges::stable_sort(
                    eligible,
                    [&](const std::size_t lhs, const std::size_t rhs) {
                        if (candidates[lhs].y != candidates[rhs].y) {
                            return candidates[lhs].y > candidates[rhs].y;
                        }
                        return lhs < rhs;
                    }
                );

                using bottom_entry = std::pair<double, std::size_t>;
                std::priority_queue<bottom_entry> smallest_bottoms;
                for (const std::size_t index : eligible) {
                    const rectangle& item = candidates[index];
                    smallest_bottoms.emplace(item.y + item.height, index);
                    if (smallest_bottoms.size() > count) {
                        smallest_bottoms.pop();
                    }
                    if (smallest_bottoms.size() != count) {
                        continue;
                    }

                    const double height = smallest_bottoms.top().first - item.y;
                    if (height < 0.0) {
                        continue;
                    }

                    std::vector<std::size_t> selected;
                    selected.reserve(count);
                    auto heap_copy = smallest_bottoms;
                    while (!heap_copy.empty()) {
                        selected.push_back(heap_copy.top().second);
                        heap_copy.pop();
                    }
                    std::ranges::sort(selected);

                    const bounds actual
                        = bounds_for_indices(candidates, selected);
                    selection_key candidate_key {
                        .area = actual.area(),
                        .perimeter = 2.0 * (actual.width() + actual.height()),
                        .indices = std::move(selected),
                        .valid = true,
                    };
                    if (better_selection_key(candidate_key, best)) {
                        best = std::move(candidate_key);
                    }
                }
            }
        }

        if (best.valid) {
            return best.indices;
        }

        std::vector<std::size_t> fallback(count);
        std::iota(fallback.begin(), fallback.end(), std::size_t { 0 });
        return fallback;
    }

    [[nodiscard]] std::size_t bounded_axis_count(
        const double span, const double step, const std::size_t limit
    ) noexcept {
        const double raw = std::ceil(span / step);
        if (!std::isfinite(raw) || raw >= static_cast<double>(limit)) {
            return limit;
        }
        return std::max<std::size_t>(1, static_cast<std::size_t>(raw));
    }

    [[nodiscard]] std::pair<std::size_t, std::size_t> sample_grid_dimensions(
        const bounds domain, const double step,
        const std::size_t requested_limit
    ) noexcept {
        const std::size_t limit
            = std::min(requested_limit, absolute_sample_limit);
        if (limit == 0 || domain.width() <= 0.0 || domain.height() <= 0.0
            || !std::isfinite(step) || step <= 0.0) {
            return { 0, 0 };
        }

        std::size_t columns = bounded_axis_count(domain.width(), step, limit);
        std::size_t rows = bounded_axis_count(domain.height(), step, limit);
        if (columns > limit / rows) {
            const double reduction = std::sqrt(
                static_cast<double>(columns) * static_cast<double>(rows)
                / static_cast<double>(limit)
            );
            columns = std::max<std::size_t>(
                1,
                static_cast<std::size_t>(
                    std::ceil(static_cast<double>(columns) / reduction)
                )
            );
            rows = std::max<std::size_t>(
                1,
                static_cast<std::size_t>(
                    std::ceil(static_cast<double>(rows) / reduction)
                )
            );
        }
        while (columns > limit / rows && (columns > 1 || rows > 1)) {
            if (columns >= rows && columns > 1) {
                --columns;
            } else if (rows > 1) {
                --rows;
            }
        }
        return { columns, rows };
    }

    [[nodiscard]] double point_distance_squared(
        const double x, const double y, const rectangle& item
    ) noexcept {
        const double right = item.x + item.width;
        const double bottom = item.y + item.height;
        const double dx
            = x < item.x ? item.x - x : (x > right ? x - right : 0.0);
        const double dy
            = y < item.y ? item.y - y : (y > bottom ? y - bottom : 0.0);
        return dx * dx + dy * dy;
    }

    [[nodiscard]] void_key score_internal_void(
        const std::span<const rectangle> candidates,
        const std::span<const std::size_t> indices,
        const std::size_t maximum_samples
    ) noexcept {
        if (indices.empty() || maximum_samples == 0) {
            return {};
        }

        const bounds domain = bounds_for_indices(candidates, indices);
        double shortest_side = std::numeric_limits<double>::infinity();
        for (const std::size_t index : indices) {
            shortest_side = std::min(
                shortest_side,
                std::min(candidates[index].width, candidates[index].height)
            );
        }
        const double step
            = std::max(shortest_side * sample_step_fraction, 1e-9);
        const auto [columns, rows]
            = sample_grid_dimensions(domain, step, maximum_samples);
        if (columns == 0 || rows == 0) {
            return {};
        }

        const double cell_width = domain.width() / static_cast<double>(columns);
        const double cell_height = domain.height() / static_cast<double>(rows);
        double total = 0.0;
        double maximum = 0.0;
        std::size_t empty_samples = 0;

        for (std::size_t row = 0; row < rows; ++row) {
            const double y
                = domain.top + (static_cast<double>(row) + 0.5) * cell_height;
            for (std::size_t column = 0; column < columns; ++column) {
                const double x = domain.left
                    + (static_cast<double>(column) + 0.5) * cell_width;
                double nearest = std::numeric_limits<double>::infinity();
                for (const std::size_t index : indices) {
                    nearest = std::min(
                        nearest, point_distance_squared(x, y, candidates[index])
                    );
                    if (nearest <= 0.0) {
                        break;
                    }
                }
                if (nearest <= 0.0 || !std::isfinite(nearest)) {
                    continue;
                }
                total += nearest;
                maximum = std::max(maximum, nearest);
                ++empty_samples;
            }
        }

        return void_key {
            .mean_squared_distance = empty_samples == 0
                ? 0.0
                : total / static_cast<double>(empty_samples),
            .maximum_squared_distance = maximum,
            .area = domain.area(),
            .valid = true,
        };
    }

    [[nodiscard]] bool better_void_key(
        const void_key& candidate, const void_key& current
    ) noexcept {
        if (!candidate.valid) {
            return false;
        }
        if (!current.valid) {
            return true;
        }
        if (candidate.mean_squared_distance != current.mean_squared_distance) {
            return candidate.mean_squared_distance
                < current.mean_squared_distance;
        }
        if (candidate.maximum_squared_distance
            != current.maximum_squared_distance) {
            return candidate.maximum_squared_distance
                < current.maximum_squared_distance;
        }
        return candidate.area < current.area;
    }

    [[nodiscard]] std::vector<std::size_t> refine_internal_void(
        const compact_layout_request& request, std::vector<std::size_t> selected
    ) {
        if (selected.empty() || request.refinement_passes == 0
            || request.maximum_samples == 0) {
            return selected;
        }

        const double minimum_area
            = bounds_for_indices(request.candidates, selected).area();
        const double area_epsilon = std::max(1e-9, minimum_area * 1e-12);
        const std::size_t pass_count = std::min(
            request.refinement_passes, absolute_refinement_pass_limit
        );

        for (std::size_t pass = 0; pass < pass_count; ++pass) {
            const void_key current = score_internal_void(
                request.candidates, selected, request.maximum_samples
            );
            if (!current.valid) {
                break;
            }

            std::vector<std::uint8_t> used(request.candidates.size(), 0);
            for (const std::size_t index : selected) {
                used[index] = 1;
            }

            void_key best = current;
            std::vector<std::size_t> best_indices = selected;
            bool improved = false;
            for (std::size_t position = 0; position < selected.size();
                 ++position) {
                for (std::size_t replacement = 0;
                     replacement < request.candidates.size(); ++replacement) {
                    if (used[replacement] != 0) {
                        continue;
                    }

                    std::vector<std::size_t> trial = selected;
                    trial[position] = replacement;
                    const bounds trial_bounds
                        = bounds_for_indices(request.candidates, trial);
                    if (trial_bounds.area() > minimum_area + area_epsilon) {
                        continue;
                    }

                    const void_key trial_key = score_internal_void(
                        request.candidates, trial, request.maximum_samples
                    );
                    if (better_void_key(trial_key, best)) {
                        best = trial_key;
                        best_indices = std::move(trial);
                        improved = true;
                    }
                }
            }
            if (!improved) {
                break;
            }
            selected = std::move(best_indices);
            std::ranges::sort(selected);
        }
        return selected;
    }

    [[nodiscard]] std::vector<rectangle> materialize(
        const std::span<const rectangle> candidates,
        const std::span<const std::size_t> indices
    ) {
        std::vector<rectangle> selected;
        selected.reserve(indices.size());
        for (const std::size_t index : indices) {
            selected.push_back(candidates[index]);
        }
        return selected;
    }

    [[nodiscard]] bool recenter(
        const extent container, std::vector<rectangle>& rectangles
    ) noexcept {
        if (rectangles.empty()) {
            return true;
        }

        const bounds current = bounding_box(rectangles);
        const double desired_x
            = container.width / 2.0 - (current.left + current.right) / 2.0;
        const double desired_y
            = container.height / 2.0 - (current.top + current.bottom) / 2.0;
        const double minimum_x = -current.left;
        const double maximum_x = container.width - current.right;
        const double minimum_y = -current.top;
        const double maximum_y = container.height - current.bottom;
        if (minimum_x > maximum_x || minimum_y > maximum_y) {
            return false;
        }

        const double offset_x = std::clamp(desired_x, minimum_x, maximum_x);
        const double offset_y = std::clamp(desired_y, minimum_y, maximum_y);
        for (rectangle& item : rectangles) {
            item.x += offset_x;
            item.y += offset_y;
        }
        return true;
    }

} // namespace

compact_layout_algorithm select_compact_layout_algorithm(
    const compact_layout_request& request
) noexcept {
    if (request.algorithm != compact_layout_algorithm::automatic) {
        return is_known(request.algorithm)
            ? request.algorithm
            : compact_layout_algorithm::automatic;
    }
    if (request.candidates.size() <= request.count) {
        return compact_layout_algorithm::automatic;
    }
    return request.count <= compact_refinement_dispatch_limit
        ? compact_layout_algorithm::void_refined
        : compact_layout_algorithm::exact_bounding_box;
}

compact_layout_result
select_compact_layout(const compact_layout_request& request) {
    const compact_layout_algorithm algorithm
        = select_compact_layout_algorithm(request);
    compact_layout_result result {
        .rectangles = {},
        .algorithm = algorithm,
    };
    if (!is_known(request.algorithm) || request.count == 0
        || request.candidates.empty() || !request_geometry_is_valid(request)) {
        return result;
    }

    if (request.candidates.size() <= request.count) {
        result.rectangles = copy_candidates(request.candidates);
        return result;
    }

    std::vector<std::size_t> selected_indices;
    switch (algorithm) {
    case compact_layout_algorithm::nearest_center:
        selected_indices = select_nearest_center_indices(request);
        break;
    case compact_layout_algorithm::exact_bounding_box:
        selected_indices = select_exact_bounding_box_indices(
            request.candidates, request.count
        );
        break;
    case compact_layout_algorithm::void_refined:
        if (request.maximum_samples == 0) {
            return result;
        }
        selected_indices = refine_internal_void(
            request,
            select_exact_bounding_box_indices(request.candidates, request.count)
        );
        break;
    case compact_layout_algorithm::automatic:
        return result;
    }

    result.rectangles = materialize(request.candidates, selected_indices);
    if (!recenter(request.container, result.rectangles)
        || !validate(request.container, result.rectangles).valid) {
        result.rectangles.clear();
    }
    return result;
}

std::string_view
algorithm_name(const compact_layout_algorithm algorithm) noexcept {
    switch (algorithm) {
    case compact_layout_algorithm::automatic:
        return "automatic";
    case compact_layout_algorithm::nearest_center:
        return "nearest_center";
    case compact_layout_algorithm::exact_bounding_box:
        return "exact_bounding_box";
    case compact_layout_algorithm::void_refined:
        return "void_refined";
    }
    return "unknown";
}

} // namespace packing
