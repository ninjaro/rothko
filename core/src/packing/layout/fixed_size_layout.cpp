#include "packing/layout/fixed_size_layout.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace packing {
namespace {

    constexpr double geometry_epsilon = 1e-9;

    struct shelf_group {
        std::vector<std::size_t> items;
        double cross_extent { 0.0 };
        double main_extent { 0.0 };
    };

    [[nodiscard]] bool
    known_algorithm(fixed_size_layout_algorithm algorithm) noexcept {
        switch (algorithm) {
        case fixed_size_layout_algorithm::automatic:
        case fixed_size_layout_algorithm::preserve_order_shelf:
        case fixed_size_layout_algorithm::best_fit_decreasing_shelf:
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

    [[nodiscard]] double
    cross_limit(extent viewport, shelf_axis axis) noexcept {
        return axis == shelf_axis::columns ? viewport.height : viewport.width;
    }

    [[nodiscard]] double main_limit(extent viewport, shelf_axis axis) noexcept {
        return axis == shelf_axis::columns ? viewport.width : viewport.height;
    }

    [[nodiscard]] double
    item_cross(const fixed_size_item& item, shelf_axis axis) noexcept {
        return axis == shelf_axis::columns ? item.size.height : item.size.width;
    }

    [[nodiscard]] double
    item_main(const fixed_size_item& item, shelf_axis axis) noexcept {
        return axis == shelf_axis::columns ? item.size.width : item.size.height;
    }

    [[nodiscard]] bool
    valid_request(const fixed_size_layout_request& request) noexcept {
        if (!is_positive_finite(request.viewport) || request.spacing < 0.0
            || !std::isfinite(request.spacing) || !known_policy(request.policy)
            || !known_algorithm(request.algorithm)) {
            return false;
        }
        return std::ranges::all_of(
            request.items, [](const fixed_size_item& item) {
                return is_positive_finite(item.size);
            }
        );
    }

    [[nodiscard]] bool fits_cross(
        double current, double addition, double spacing, double limit,
        bool group_empty
    ) noexcept {
        const double candidate
            = current + (group_empty ? 0.0 : spacing) + addition;
        return std::isfinite(candidate)
            && candidate <= limit + geometry_epsilon;
    }

    [[nodiscard]] std::vector<shelf_group> preserve_order_groups(
        const fixed_size_layout_request& request, shelf_axis axis
    ) {
        std::vector<shelf_group> groups;
        const double limit = cross_limit(request.viewport, axis);

        for (std::size_t index = 0; index < request.items.size(); ++index) {
            const double cross = item_cross(request.items[index], axis);
            const double main = item_main(request.items[index], axis);
            if (cross > limit + geometry_epsilon) {
                return {};
            }
            if (groups.empty()
                || !fits_cross(
                    groups.back().cross_extent, cross, request.spacing, limit,
                    groups.back().items.empty()
                )) {
                groups.emplace_back();
            }

            shelf_group& group = groups.back();
            if (!group.items.empty()) {
                group.cross_extent += request.spacing;
            }
            group.cross_extent += cross;
            group.main_extent = std::max(group.main_extent, main);
            group.items.push_back(index);
        }
        return groups;
    }

    [[nodiscard]] std::vector<shelf_group>
    best_fit_groups(const fixed_size_layout_request& request, shelf_axis axis) {
        std::vector<std::size_t> order(request.items.size());
        std::iota(order.begin(), order.end(), std::size_t { 0 });
        std::ranges::stable_sort(order, [&](std::size_t lhs, std::size_t rhs) {
            const double lhs_cross = item_cross(request.items[lhs], axis);
            const double rhs_cross = item_cross(request.items[rhs], axis);
            if (lhs_cross != rhs_cross) {
                return lhs_cross > rhs_cross;
            }
            const double lhs_main = item_main(request.items[lhs], axis);
            const double rhs_main = item_main(request.items[rhs], axis);
            if (lhs_main != rhs_main) {
                return lhs_main > rhs_main;
            }
            return lhs < rhs;
        });

        std::vector<shelf_group> groups;
        const double limit = cross_limit(request.viewport, axis);
        for (const std::size_t index : order) {
            const double cross = item_cross(request.items[index], axis);
            const double main = item_main(request.items[index], axis);
            if (cross > limit + geometry_epsilon) {
                return {};
            }

            std::size_t best_group = groups.size();
            double best_residual = std::numeric_limits<double>::infinity();
            double best_growth = std::numeric_limits<double>::infinity();
            for (std::size_t group_index = 0; group_index < groups.size();
                 ++group_index) {
                const shelf_group& group = groups[group_index];
                if (!fits_cross(
                        group.cross_extent, cross, request.spacing, limit, false
                    )) {
                    continue;
                }
                const double candidate_cross
                    = group.cross_extent + request.spacing + cross;
                const double residual = limit - candidate_cross;
                const double growth
                    = std::max(group.main_extent, main) - group.main_extent;
                if (residual < best_residual
                    || (residual == best_residual && growth < best_growth)) {
                    best_group = group_index;
                    best_residual = residual;
                    best_growth = growth;
                }
            }

            if (best_group == groups.size()) {
                groups.emplace_back();
                best_group = groups.size() - 1;
            }
            shelf_group& group = groups[best_group];
            if (!group.items.empty()) {
                group.cross_extent += request.spacing;
            }
            group.cross_extent += cross;
            group.main_extent = std::max(group.main_extent, main);
            group.items.push_back(index);
        }
        return groups;
    }

    [[nodiscard]] fixed_size_layout_result place_groups(
        const fixed_size_layout_request& request, shelf_axis axis,
        fixed_size_layout_algorithm algorithm,
        const std::vector<shelf_group>& groups
    ) {
        fixed_size_layout_result result;
        result.algorithm = algorithm;
        result.axis = axis;
        if (groups.empty() && !request.items.empty()) {
            return result;
        }

        result.rectangles.resize(request.items.size());
        result.placement_order.reserve(request.items.size());
        result.groups.reserve(groups.size());
        double main_position = 0.0;

        for (std::size_t group_index = 0; group_index < groups.size();
             ++group_index) {
            const shelf_group& group = groups[group_index];
            const std::size_t begin = result.placement_order.size();
            double cross_position = 0.0;
            for (std::size_t item_position = 0;
                 item_position < group.items.size(); ++item_position) {
                const std::size_t source_index = group.items[item_position];
                const extent size = request.items[source_index].size;
                rectangle placed {
                    .x = axis == shelf_axis::columns ? main_position
                                                     : cross_position,
                    .y = axis == shelf_axis::columns ? cross_position
                                                     : main_position,
                    .width = size.width,
                    .height = size.height,
                    .source_index = source_index,
                };
                if (!is_finite(placed)) {
                    return {};
                }
                result.rectangles[source_index] = placed;
                result.placement_order.push_back(source_index);
                cross_position += item_cross(request.items[source_index], axis);
                if (item_position + 1 < group.items.size()) {
                    cross_position += request.spacing;
                }
            }
            result.groups.push_back(
                { .begin = begin, .end = result.placement_order.size() }
            );
            main_position += group.main_extent;
            if (group_index + 1 < groups.size()) {
                main_position += request.spacing;
            }
        }

        if (!std::isfinite(main_position)) {
            return {};
        }
        result.primary_extent = main_position;
        result.used_scroll = main_position
            > main_limit(request.viewport, axis) + geometry_epsilon;
        result.content = request.viewport;
        if (axis == shelf_axis::columns) {
            result.content.width
                = std::max(result.content.width, main_position);
        } else {
            result.content.height
                = std::max(result.content.height, main_position);
        }
        return result;
    }

    [[nodiscard]] fixed_size_layout_result solve_axis(
        const fixed_size_layout_request& request, shelf_axis axis,
        fixed_size_layout_algorithm algorithm
    ) {
        const std::vector<shelf_group> groups
            = algorithm == fixed_size_layout_algorithm::preserve_order_shelf
            ? preserve_order_groups(request, axis)
            : best_fit_groups(request, axis);
        return place_groups(request, axis, algorithm, groups);
    }

    [[nodiscard]] double normalized_primary(
        const fixed_size_layout_result& result, extent viewport
    ) noexcept {
        const double limit = main_limit(viewport, result.axis);
        return result.primary_extent / limit;
    }

} // namespace

fixed_size_layout_algorithm select_fixed_size_layout_algorithm(
    const fixed_size_layout_request& request
) noexcept {
    if (request.algorithm != fixed_size_layout_algorithm::automatic
        && known_algorithm(request.algorithm)) {
        return request.algorithm;
    }
    return request.items.size() <= fixed_best_fit_dispatch_limit
        ? fixed_size_layout_algorithm::best_fit_decreasing_shelf
        : fixed_size_layout_algorithm::preserve_order_shelf;
}

fixed_size_layout_result
layout_fixed_size_rectangles(const fixed_size_layout_request& request) {
    const fixed_size_layout_algorithm algorithm
        = select_fixed_size_layout_algorithm(request);
    fixed_size_layout_result invalid;
    invalid.algorithm = algorithm;
    invalid.axis = axis_for_policy(request.policy);

    if (!valid_request(request)) {
        return invalid;
    }
    if (request.items.empty()) {
        invalid.content = request.viewport;
        return invalid;
    }

    if (request.policy != scroll_policy::disabled) {
        return solve_axis(request, axis_for_policy(request.policy), algorithm);
    }

    fixed_size_layout_result columns
        = solve_axis(request, shelf_axis::columns, algorithm);
    fixed_size_layout_result rows
        = solve_axis(request, shelf_axis::rows, algorithm);
    const bool columns_fit
        = columns.complete(request.items.size()) && !columns.used_scroll;
    const bool rows_fit
        = rows.complete(request.items.size()) && !rows.used_scroll;
    if (!columns_fit && !rows_fit) {
        return invalid;
    }

    fixed_size_layout_result result;
    if (!rows_fit
        || (columns_fit
            && normalized_primary(columns, request.viewport)
                <= normalized_primary(rows, request.viewport))) {
        result = std::move(columns);
    } else {
        result = std::move(rows);
    }
    result.content = request.viewport;
    result.used_scroll = false;
    return result;
}

std::string_view
algorithm_name(fixed_size_layout_algorithm algorithm) noexcept {
    switch (algorithm) {
    case fixed_size_layout_algorithm::automatic:
        return "automatic";
    case fixed_size_layout_algorithm::preserve_order_shelf:
        return "preserve_order_shelf";
    case fixed_size_layout_algorithm::best_fit_decreasing_shelf:
        return "best_fit_decreasing_shelf";
    }
    return "unknown";
}

} // namespace packing
