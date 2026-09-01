#include "packing/layout/free_order_layout.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>
#include <vector>

namespace packing {
namespace {

    constexpr double comparison_epsilon = 1e-9;

    struct unit_size {
        double width { 0.0 };
        double height { 0.0 };
    };

    [[nodiscard]] bool
    known_algorithm(free_order_layout_algorithm algorithm) noexcept {
        switch (algorithm) {
        case free_order_layout_algorithm::automatic:
        case free_order_layout_algorithm::sorted_shelf:
        case free_order_layout_algorithm::multistart_shelf:
            return true;
        }
        return false;
    }

    [[nodiscard]] bool known_policy(scroll_policy policy) noexcept {
        switch (policy) {
        case scroll_policy::horizontal:
        case scroll_policy::vertical:
        case scroll_policy::disabled:
            return true;
        }
        return false;
    }

    [[nodiscard]] shelf_axis axis_for_policy(scroll_policy policy) noexcept {
        return policy == scroll_policy::vertical ? shelf_axis::rows
                                                 : shelf_axis::columns;
    }

    [[nodiscard]] bool make_unit_sizes(
        std::span<const ordered_slot> items, std::vector<unit_size>& output
    ) {
        output.clear();
        output.reserve(items.size());
        for (const ordered_slot& item : items) {
            if (!(item.aspect > 0.0) || !std::isfinite(item.aspect)) {
                return false;
            }
            const double aspect
                = item.rotated ? 1.0 / item.aspect : item.aspect;
            if (!(aspect > 0.0) || !std::isfinite(aspect)) {
                return false;
            }
            const unit_size size = aspect >= 1.0
                ? unit_size { aspect, 1.0 }
                : unit_size { 1.0, 1.0 / aspect };
            if (!(size.width > 0.0) || !(size.height > 0.0)
                || !std::isfinite(size.width) || !std::isfinite(size.height)) {
                return false;
            }
            output.push_back(size);
        }
        return true;
    }

    [[nodiscard]] bool valid_request(
        const free_order_layout_request& request, std::vector<unit_size>& sizes
    ) {
        return is_positive_finite(request.viewport)
            && request.minimum_short_side >= 0.0
            && std::isfinite(request.minimum_short_side)
            && request.spacing >= 0.0 && std::isfinite(request.spacing)
            && known_policy(request.policy)
            && known_algorithm(request.algorithm)
            && make_unit_sizes(request.items, sizes);
    }

    [[nodiscard]] std::vector<std::size_t> identity_order(std::size_t count) {
        std::vector<std::size_t> result(count);
        std::iota(result.begin(), result.end(), std::size_t { 0 });
        return result;
    }

    template <typename Primary, typename Secondary>
    [[nodiscard]] std::vector<std::size_t>
    decreasing_order(std::size_t count, Primary primary, Secondary secondary) {
        std::vector<std::size_t> result = identity_order(count);
        std::ranges::stable_sort(result, [&](std::size_t lhs, std::size_t rhs) {
            const double lhs_primary = primary(lhs);
            const double rhs_primary = primary(rhs);
            if (lhs_primary != rhs_primary) {
                return lhs_primary > rhs_primary;
            }
            const double lhs_secondary = secondary(lhs);
            const double rhs_secondary = secondary(rhs);
            if (lhs_secondary != rhs_secondary) {
                return lhs_secondary > rhs_secondary;
            }
            return lhs < rhs;
        });
        return result;
    }

    [[nodiscard]] std::vector<std::size_t> increasing_cross_order(
        const std::vector<unit_size>& sizes, shelf_axis axis
    ) {
        std::vector<std::size_t> result = identity_order(sizes.size());
        const auto cross = [&](std::size_t index) {
            return axis == shelf_axis::columns ? sizes[index].height
                                               : sizes[index].width;
        };
        const auto main = [&](std::size_t index) {
            return axis == shelf_axis::columns ? sizes[index].width
                                               : sizes[index].height;
        };
        std::ranges::stable_sort(result, [&](std::size_t lhs, std::size_t rhs) {
            if (cross(lhs) != cross(rhs)) {
                return cross(lhs) < cross(rhs);
            }
            if (main(lhs) != main(rhs)) {
                return main(lhs) > main(rhs);
            }
            return lhs < rhs;
        });
        return result;
    }

    void append_unique_order(
        std::vector<std::vector<std::size_t>>& orders,
        std::vector<std::size_t> candidate
    ) {
        if (std::ranges::find(orders, candidate) == orders.end()) {
            orders.push_back(std::move(candidate));
        }
    }

    [[nodiscard]] std::vector<std::vector<std::size_t>> make_orders(
        const std::vector<unit_size>& sizes, shelf_axis axis,
        free_order_layout_algorithm algorithm
    ) {
        const auto cross = [&](std::size_t index) {
            return axis == shelf_axis::columns ? sizes[index].height
                                               : sizes[index].width;
        };
        const auto main = [&](std::size_t index) {
            return axis == shelf_axis::columns ? sizes[index].width
                                               : sizes[index].height;
        };

        std::vector<std::vector<std::size_t>> result;
        append_unique_order(
            result, decreasing_order(sizes.size(), cross, main)
        );
        if (algorithm == free_order_layout_algorithm::sorted_shelf) {
            return result;
        }

        append_unique_order(result, identity_order(sizes.size()));
        append_unique_order(
            result, decreasing_order(sizes.size(), main, cross)
        );
        append_unique_order(result, increasing_cross_order(sizes, axis));
        return result;
    }

    [[nodiscard]] double comparison_tolerance(double lhs, double rhs) noexcept {
        return comparison_epsilon
            * std::max({ 1.0, std::abs(lhs), std::abs(rhs) });
    }

    [[nodiscard]] bool meaningfully_greater(double lhs, double rhs) noexcept {
        return lhs > rhs + comparison_tolerance(lhs, rhs);
    }

    [[nodiscard]] bool meaningfully_less(double lhs, double rhs) noexcept {
        return lhs < rhs - comparison_tolerance(lhs, rhs);
    }

    [[nodiscard]] double normalized_primary(
        const free_order_layout_result& result, extent viewport
    ) noexcept {
        const double limit = result.axis == shelf_axis::columns
            ? viewport.width
            : viewport.height;
        return result.primary_extent / limit;
    }

    [[nodiscard]] bool better_result(
        const free_order_layout_result& candidate,
        const free_order_layout_result& incumbent, extent viewport,
        std::size_t expected
    ) noexcept {
        if (!candidate.complete(expected)) {
            return false;
        }
        if (!incumbent.complete(expected)) {
            return true;
        }
        if (meaningfully_greater(
                candidate.common_short_side, incumbent.common_short_side
            )) {
            return true;
        }
        if (meaningfully_less(
                candidate.common_short_side, incumbent.common_short_side
            )) {
            return false;
        }
        if (candidate.used_scroll != incumbent.used_scroll) {
            return !candidate.used_scroll;
        }
        const double candidate_primary
            = normalized_primary(candidate, viewport);
        const double incumbent_primary
            = normalized_primary(incumbent, viewport);
        if (meaningfully_less(candidate_primary, incumbent_primary)) {
            return true;
        }
        if (meaningfully_greater(candidate_primary, incumbent_primary)) {
            return false;
        }
        if (meaningfully_less(candidate.raggedness, incumbent.raggedness)) {
            return true;
        }
        if (meaningfully_greater(candidate.raggedness, incumbent.raggedness)) {
            return false;
        }
        return candidate.placement_order < incumbent.placement_order;
    }

    [[nodiscard]] free_order_layout_result solve_order(
        const free_order_layout_request& request,
        const std::vector<std::size_t>& order,
        free_order_layout_algorithm selected_algorithm
    ) {
        std::vector<ordered_slot> reordered;
        reordered.reserve(order.size());
        for (const std::size_t source_index : order) {
            reordered.push_back(request.items[source_index]);
        }

        const ordered_layout_algorithm shelf_algorithm
            = selected_algorithm == free_order_layout_algorithm::sorted_shelf
            ? ordered_layout_algorithm::greedy_shelf
            : ordered_layout_algorithm::balanced_shelf;
        const ordered_layout_result ordered = layout_ordered_slots(
            {
                .viewport = request.viewport,
                .items = reordered,
                .policy = request.policy,
                .minimum_short_side = request.minimum_short_side,
                .spacing = request.spacing,
                .algorithm = shelf_algorithm,
            }
        );

        free_order_layout_result result;
        result.algorithm = selected_algorithm;
        result.axis = ordered.axis;
        if (ordered.rectangles.size() != order.size()) {
            return result;
        }

        result.rectangles.resize(order.size());
        result.placement_order = order;
        result.groups = ordered.groups;
        result.content = ordered.content;
        result.common_short_side = ordered.common_short_side;
        result.primary_extent = ordered.primary_extent;
        result.raggedness = ordered.raggedness;
        result.used_scroll = ordered.used_scroll;
        result.threshold_met = ordered.threshold_met;

        for (std::size_t position = 0; position < order.size(); ++position) {
            rectangle placed = ordered.rectangles[position];
            placed.source_index = order[position];
            result.rectangles[order[position]] = placed;
        }
        return result;
    }

} // namespace

free_order_layout_algorithm select_free_order_layout_algorithm(
    const free_order_layout_request& request
) noexcept {
    if (request.algorithm != free_order_layout_algorithm::automatic
        && known_algorithm(request.algorithm)) {
        return request.algorithm;
    }
    return request.items.size() <= free_order_multistart_dispatch_limit
        ? free_order_layout_algorithm::multistart_shelf
        : free_order_layout_algorithm::sorted_shelf;
}

free_order_layout_result
layout_free_order_slots(const free_order_layout_request& request) {
    const free_order_layout_algorithm algorithm
        = select_free_order_layout_algorithm(request);
    free_order_layout_result invalid;
    invalid.algorithm = algorithm;
    invalid.axis = axis_for_policy(request.policy);

    std::vector<unit_size> sizes;
    if (!valid_request(request, sizes)) {
        return invalid;
    }
    if (request.items.empty()) {
        invalid.content = request.viewport;
        invalid.threshold_met = true;
        return invalid;
    }

    free_order_layout_result best;
    best.algorithm = algorithm;
    best.axis = axis_for_policy(request.policy);
    for (const std::vector<std::size_t>& order :
         make_orders(sizes, axis_for_policy(request.policy), algorithm)) {
        free_order_layout_result candidate
            = solve_order(request, order, algorithm);
        if (better_result(
                candidate, best, request.viewport, request.items.size()
            )) {
            best = std::move(candidate);
        }
    }
    return best;
}

std::string_view
algorithm_name(free_order_layout_algorithm algorithm) noexcept {
    switch (algorithm) {
    case free_order_layout_algorithm::automatic:
        return "automatic";
    case free_order_layout_algorithm::sorted_shelf:
        return "sorted_shelf";
    case free_order_layout_algorithm::multistart_shelf:
        return "multistart_shelf";
    }
    return "unknown";
}

} // namespace packing
