#include "packing/spread_layout.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <utility>
#include <vector>

namespace packing {
namespace {

    constexpr double sample_step_fraction = 0.40;
    constexpr std::size_t absolute_sample_limit = 8192;
    constexpr std::size_t absolute_swap_pass_limit = 16;
    constexpr std::size_t translation_move_limit = 128;

    struct point {
        double x { 0.0 };
        double y { 0.0 };
    };

    struct coverage_key {
        double mean_squared_distance {
            std::numeric_limits<double>::infinity()
        };
        double maximum_squared_distance {
            std::numeric_limits<double>::infinity()
        };
        bool valid { false };
    };

    [[nodiscard]] bool
    is_known(const spread_layout_algorithm algorithm) noexcept {
        switch (algorithm) {
        case spread_layout_algorithm::automatic:
        case spread_layout_algorithm::greedy_coverage:
        case spread_layout_algorithm::coverage_with_swaps:
            return true;
        }
        return false;
    }

    [[nodiscard]] bool
    request_geometry_is_valid(const spread_layout_request& request) noexcept {
        return is_positive_finite(request.container)
            && validate(request.container, request.candidates).valid;
    }

    [[nodiscard]] std::vector<rectangle>
    copy_candidates(const std::span<const rectangle> candidates) {
        return { candidates.begin(), candidates.end() };
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
        const extent container, const double step,
        const std::size_t requested_limit
    ) noexcept {
        const std::size_t limit
            = std::min(requested_limit, absolute_sample_limit);
        if (limit == 0 || !is_positive_finite(container) || !std::isfinite(step)
            || step <= 0.0) {
            return { 0, 0 };
        }

        std::size_t columns = bounded_axis_count(container.width, step, limit);
        std::size_t rows = bounded_axis_count(container.height, step, limit);
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

    [[nodiscard]] std::vector<point>
    make_sample_points(const spread_layout_request& request) {
        double shortest_side = std::numeric_limits<double>::infinity();
        for (const rectangle& item : request.candidates) {
            shortest_side
                = std::min(shortest_side, std::min(item.width, item.height));
        }
        const double step
            = std::max(shortest_side * sample_step_fraction, 1e-9);
        const auto [columns, rows] = sample_grid_dimensions(
            request.container, step, request.maximum_samples
        );
        if (columns == 0 || rows == 0) {
            return {};
        }

        std::vector<point> points;
        points.reserve(columns * rows);
        const double cell_width
            = request.container.width / static_cast<double>(columns);
        const double cell_height
            = request.container.height / static_cast<double>(rows);
        for (std::size_t row = 0; row < rows; ++row) {
            const double y = (static_cast<double>(row) + 0.5) * cell_height;
            for (std::size_t column = 0; column < columns; ++column) {
                const double x
                    = (static_cast<double>(column) + 0.5) * cell_width;
                points.push_back({ x, y });
            }
        }
        return points;
    }

    [[nodiscard]] double
    point_distance_squared(const point sample, const rectangle& item) noexcept {
        const double right = item.x + item.width;
        const double bottom = item.y + item.height;
        const double dx = sample.x < item.x
            ? item.x - sample.x
            : (sample.x > right ? sample.x - right : 0.0);
        const double dy = sample.y < item.y
            ? item.y - sample.y
            : (sample.y > bottom ? sample.y - bottom : 0.0);
        return dx * dx + dy * dy;
    }

    [[nodiscard]] bool better_coverage_key(
        const coverage_key& candidate, const coverage_key& current
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
        return candidate.maximum_squared_distance
            < current.maximum_squared_distance;
    }

    class coverage_matrix {
    public:
        coverage_matrix(
            const std::span<const rectangle> candidates,
            const std::span<const point> points
        )
            : point_count_(points.size()) {
            if (point_count_ == 0
                || candidates.size() > values_.max_size()
                        / std::max<std::size_t>(point_count_, 1)) {
                return;
            }

            values_.reserve(candidates.size() * point_count_);
            for (const rectangle& item : candidates) {
                for (const point sample : points) {
                    values_.push_back(point_distance_squared(sample, item));
                }
            }
        }

        [[nodiscard]] bool
        valid(const std::size_t candidate_count) const noexcept {
            return point_count_ > 0
                && values_.size() == candidate_count * point_count_;
        }

        [[nodiscard]] std::size_t point_count() const noexcept {
            return point_count_;
        }

        [[nodiscard]] double
        at(const std::size_t candidate,
           const std::size_t sample) const noexcept {
            return values_[candidate * point_count_ + sample];
        }

    private:
        std::size_t point_count_ { 0 };
        std::vector<double> values_;
    };

    [[nodiscard]] coverage_key
    key_for_distances(const std::span<const double> distances) noexcept {
        if (distances.empty()) {
            return {};
        }
        double total = 0.0;
        double maximum = 0.0;
        for (const double distance : distances) {
            if (!std::isfinite(distance)) {
                return {};
            }
            total += distance;
            maximum = std::max(maximum, distance);
        }
        return coverage_key {
            .mean_squared_distance
            = total / static_cast<double>(distances.size()),
            .maximum_squared_distance = maximum,
            .valid = true,
        };
    }

    [[nodiscard]] std::vector<std::size_t> greedy_selection(
        const coverage_matrix& matrix, const std::size_t candidate_count,
        const std::size_t count
    ) {
        std::vector<std::size_t> chosen;
        chosen.reserve(count);
        std::vector<std::uint8_t> used(candidate_count, 0);
        std::vector<double> current(
            matrix.point_count(), std::numeric_limits<double>::infinity()
        );
        std::vector<double> trial(matrix.point_count());

        for (std::size_t selected_count = 0; selected_count < count;
             ++selected_count) {
            coverage_key best;
            std::size_t best_index = candidate_count;

            // An indexed used vector deliberately replaces the Python set.
            // Besides avoiding hashing, this fixes tie traversal to candidate
            // source order.
            for (std::size_t candidate = 0; candidate < candidate_count;
                 ++candidate) {
                if (used[candidate] != 0) {
                    continue;
                }
                for (std::size_t sample = 0; sample < matrix.point_count();
                     ++sample) {
                    trial[sample] = std::min(
                        current[sample], matrix.at(candidate, sample)
                    );
                }
                const coverage_key candidate_key = key_for_distances(trial);
                if (better_coverage_key(candidate_key, best)) {
                    best = candidate_key;
                    best_index = candidate;
                }
            }

            if (best_index == candidate_count) {
                return {};
            }
            chosen.push_back(best_index);
            used[best_index] = 1;
            for (std::size_t sample = 0; sample < matrix.point_count();
                 ++sample) {
                current[sample]
                    = std::min(current[sample], matrix.at(best_index, sample));
            }
        }
        return chosen;
    }

    void refine_with_swaps(
        const coverage_matrix& matrix, const std::size_t candidate_count,
        const std::size_t requested_passes, std::vector<std::size_t>& chosen
    ) {
        const std::size_t pass_count
            = std::min(requested_passes, absolute_swap_pass_limit);
        const std::size_t point_count = matrix.point_count();

        for (std::size_t pass = 0; pass < pass_count; ++pass) {
            std::vector<double> best_distance(
                point_count, std::numeric_limits<double>::infinity()
            );
            std::vector<double> second_distance(
                point_count, std::numeric_limits<double>::infinity()
            );
            std::vector<std::size_t> best_owner(point_count, chosen.size());

            for (std::size_t position = 0; position < chosen.size();
                 ++position) {
                const std::size_t candidate = chosen[position];
                for (std::size_t sample = 0; sample < point_count; ++sample) {
                    const double distance = matrix.at(candidate, sample);
                    if (distance < best_distance[sample]) {
                        second_distance[sample] = best_distance[sample];
                        best_distance[sample] = distance;
                        best_owner[sample] = position;
                    } else if (distance < second_distance[sample]) {
                        second_distance[sample] = distance;
                    }
                }
            }

            coverage_key best = key_for_distances(best_distance);
            std::size_t best_position = chosen.size();
            std::size_t best_replacement = candidate_count;
            std::vector<std::uint8_t> used(candidate_count, 0);
            for (const std::size_t candidate : chosen) {
                used[candidate] = 1;
            }

            std::vector<double> trial(point_count);
            for (std::size_t position = 0; position < chosen.size();
                 ++position) {
                for (std::size_t replacement = 0; replacement < candidate_count;
                     ++replacement) {
                    if (used[replacement] != 0) {
                        continue;
                    }
                    for (std::size_t sample = 0; sample < point_count;
                         ++sample) {
                        const double base = best_owner[sample] == position
                            ? second_distance[sample]
                            : best_distance[sample];
                        trial[sample]
                            = std::min(base, matrix.at(replacement, sample));
                    }
                    const coverage_key candidate_key = key_for_distances(trial);
                    if (better_coverage_key(candidate_key, best)) {
                        best = candidate_key;
                        best_position = position;
                        best_replacement = replacement;
                    }
                }
            }

            if (best_position == chosen.size()) {
                break;
            }
            chosen[best_position] = best_replacement;
        }
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

    [[nodiscard]] coverage_key coverage_score(
        const std::span<const point> points,
        const std::span<const rectangle> rectangles
    ) noexcept {
        if (points.empty() || rectangles.empty()) {
            return {};
        }
        double total = 0.0;
        double maximum = 0.0;
        for (const point sample : points) {
            double nearest = std::numeric_limits<double>::infinity();
            for (const rectangle& item : rectangles) {
                nearest
                    = std::min(nearest, point_distance_squared(sample, item));
                if (nearest <= 0.0) {
                    break;
                }
            }
            if (!std::isfinite(nearest)) {
                return {};
            }
            total += nearest;
            maximum = std::max(maximum, nearest);
        }
        return {
            .mean_squared_distance = total / static_cast<double>(points.size()),
            .maximum_squared_distance = maximum,
            .valid = true,
        };
    }

    void translate(
        std::vector<rectangle>& rectangles, const double dx, const double dy
    ) noexcept {
        for (rectangle& item : rectangles) {
            item.x += dx;
            item.y += dy;
        }
    }

    [[nodiscard]] bool center_in_container(
        const extent container, std::vector<rectangle>& rectangles
    ) noexcept {
        const bounds current = bounding_box(rectangles);
        const double minimum_dx = -current.left;
        const double maximum_dx = container.width - current.right;
        const double minimum_dy = -current.top;
        const double maximum_dy = container.height - current.bottom;
        if (minimum_dx > maximum_dx || minimum_dy > maximum_dy) {
            return false;
        }
        const double desired_dx
            = container.width / 2.0 - (current.left + current.right) / 2.0;
        const double desired_dy
            = container.height / 2.0 - (current.top + current.bottom) / 2.0;
        translate(
            rectangles, std::clamp(desired_dx, minimum_dx, maximum_dx),
            std::clamp(desired_dy, minimum_dy, maximum_dy)
        );
        return true;
    }

    [[nodiscard]] bool optimize_translation(
        const extent container, const std::span<const point> points,
        std::vector<rectangle>& rectangles
    ) {
        if (rectangles.empty() || points.empty()
            || !center_in_container(container, rectangles)) {
            return false;
        }

        double shortest_side = std::numeric_limits<double>::infinity();
        for (const rectangle& item : rectangles) {
            shortest_side
                = std::min(shortest_side, std::min(item.width, item.height));
        }
        const std::array steps {
            shortest_side,
            shortest_side / 2.0,
            shortest_side / 4.0,
            shortest_side / 8.0,
            1.0,
        };
        coverage_key current = coverage_score(points, rectangles);
        if (!current.valid) {
            return false;
        }

        std::size_t moves = 0;
        for (const double step : steps) {
            if (!std::isfinite(step) || step <= 1e-9) {
                continue;
            }
            bool improved = true;
            while (improved && moves < translation_move_limit) {
                improved = false;
                const bounds current_bounds = bounding_box(rectangles);
                const double minimum_dx = -current_bounds.left;
                const double maximum_dx
                    = container.width - current_bounds.right;
                const double minimum_dy = -current_bounds.top;
                const double maximum_dy
                    = container.height - current_bounds.bottom;

                std::vector<rectangle> best_rectangles;
                coverage_key best = current;
                constexpr std::array directions {
                    std::pair { -1.0, -1.0 }, std::pair { 0.0, -1.0 },
                    std::pair { 1.0, -1.0 },  std::pair { -1.0, 0.0 },
                    std::pair { 1.0, 0.0 },   std::pair { -1.0, 1.0 },
                    std::pair { 0.0, 1.0 },   std::pair { 1.0, 1.0 },
                };
                for (const auto [x_direction, y_direction] : directions) {
                    const double dx = std::clamp(
                        x_direction * step, minimum_dx, maximum_dx
                    );
                    const double dy = std::clamp(
                        y_direction * step, minimum_dy, maximum_dy
                    );
                    if (std::abs(dx) <= 1e-12 && std::abs(dy) <= 1e-12) {
                        continue;
                    }
                    std::vector<rectangle> trial = rectangles;
                    translate(trial, dx, dy);
                    const coverage_key candidate
                        = coverage_score(points, trial);
                    if (better_coverage_key(candidate, best)) {
                        best = candidate;
                        best_rectangles = std::move(trial);
                    }
                }
                if (!best_rectangles.empty()) {
                    rectangles = std::move(best_rectangles);
                    current = best;
                    improved = true;
                    ++moves;
                }
            }
        }
        return validate(container, rectangles).valid;
    }

} // namespace

spread_layout_algorithm
select_spread_layout_algorithm(const spread_layout_request& request) noexcept {
    if (request.algorithm != spread_layout_algorithm::automatic) {
        return is_known(request.algorithm) ? request.algorithm
                                           : spread_layout_algorithm::automatic;
    }
    if (request.candidates.size() <= request.count) {
        return spread_layout_algorithm::automatic;
    }
    return request.count <= spread_swap_dispatch_limit
        ? spread_layout_algorithm::coverage_with_swaps
        : spread_layout_algorithm::greedy_coverage;
}

spread_layout_result
select_spread_layout(const spread_layout_request& request) {
    const spread_layout_algorithm algorithm
        = select_spread_layout_algorithm(request);
    spread_layout_result result {
        .rectangles = {},
        .algorithm = algorithm,
    };
    if (!is_known(request.algorithm) || request.count == 0
        || request.candidates.empty() || !request_geometry_is_valid(request)) {
        return result;
    }

    if (request.maximum_samples == 0) {
        return result;
    }

    const std::vector<point> points = make_sample_points(request);
    if (request.candidates.size() <= request.count) {
        result.rectangles = copy_candidates(request.candidates);
        if (!optimize_translation(
                request.container, points, result.rectangles
            )) {
            result.rectangles.clear();
        }
        return result;
    }
    const coverage_matrix matrix(request.candidates, points);
    if (!matrix.valid(request.candidates.size())) {
        return result;
    }

    std::vector<std::size_t> chosen
        = greedy_selection(matrix, request.candidates.size(), request.count);
    if (chosen.size() != request.count) {
        return result;
    }

    switch (algorithm) {
    case spread_layout_algorithm::coverage_with_swaps:
        refine_with_swaps(
            matrix, request.candidates.size(), request.swap_passes, chosen
        );
        break;
    case spread_layout_algorithm::greedy_coverage:
        break;
    case spread_layout_algorithm::automatic:
        return result;
    }

    result.rectangles = materialize(request.candidates, chosen);
    if (!optimize_translation(request.container, points, result.rectangles)) {
        result.rectangles.clear();
    }
    return result;
}

std::string_view
algorithm_name(const spread_layout_algorithm algorithm) noexcept {
    switch (algorithm) {
    case spread_layout_algorithm::automatic:
        return "automatic";
    case spread_layout_algorithm::greedy_coverage:
        return "greedy_coverage";
    case spread_layout_algorithm::coverage_with_swaps:
        return "coverage_with_swaps";
    }
    return "unknown";
}

} // namespace packing
