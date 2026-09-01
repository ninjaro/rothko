#include "packing/layout/ordered_layout.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace packing {
namespace {

    constexpr double geometry_epsilon = 1e-9;
    constexpr std::size_t scale_search_iterations = 48;
    constexpr std::size_t no_index = std::numeric_limits<std::size_t>::max();

    struct unit_slot {
        double width;
        double height;
        bool rotated;
    };

    struct axis_item {
        double main;
        double cross;
    };

    struct group_geometry {
        std::size_t begin;
        std::size_t end;
        double main_extent;
        double cross_extent;
        double slack;
    };

    struct partition_result {
        std::vector<group_geometry> groups;
        double extent { 0.0 };
        double raggedness { 0.0 };
        bool feasible { false };
    };

    struct balanced_state {
        double extent { 0.0 };
        double raggedness { 0.0 };
        std::size_t group_count { 0 };
        std::size_t previous { no_index };
        bool reachable { false };
    };

    [[nodiscard]] bool nonnegative_finite(double value) noexcept {
        return value >= 0.0 && std::isfinite(value);
    }

    [[nodiscard]] bool positive_finite(double value) noexcept {
        return value > 0.0 && std::isfinite(value);
    }

    [[nodiscard]] double comparison_tolerance(double lhs, double rhs) noexcept {
        return 1e-10 * std::max({ 1.0, std::abs(lhs), std::abs(rhs) });
    }

    [[nodiscard]] bool meaningfully_less(double lhs, double rhs) noexcept {
        return lhs < rhs - comparison_tolerance(lhs, rhs);
    }

    [[nodiscard]] bool nearly_equal(double lhs, double rhs) noexcept {
        return std::abs(lhs - rhs) <= comparison_tolerance(lhs, rhs);
    }

    [[nodiscard]] bool exceeds(double value, double limit) noexcept {
        return value > limit + geometry_epsilon;
    }

    [[nodiscard]] double square_and_add(double total, double value) noexcept {
        const auto maximum = std::numeric_limits<double>::max();
        if (value > std::sqrt(maximum)) {
            return maximum;
        }
        const double square = value * value;
        if (total > maximum - square) {
            return maximum;
        }
        return total + square;
    }

    [[nodiscard]] shelf_axis axis_for_policy(scroll_policy policy) noexcept {
        return policy == scroll_policy::vertical ? shelf_axis::rows
                                                 : shelf_axis::columns;
    }

    [[nodiscard]] bool valid_policy(scroll_policy policy) noexcept {
        switch (policy) {
        case scroll_policy::horizontal:
        case scroll_policy::vertical:
        case scroll_policy::disabled:
            return true;
        }
        return false;
    }

    [[nodiscard]] bool
    valid_algorithm(ordered_layout_algorithm algorithm) noexcept {
        switch (algorithm) {
        case ordered_layout_algorithm::automatic:
        case ordered_layout_algorithm::equal_grid:
        case ordered_layout_algorithm::greedy_shelf:
        case ordered_layout_algorithm::exact_shelf:
        case ordered_layout_algorithm::balanced_shelf:
            return true;
        }
        return false;
    }

    [[nodiscard]] bool make_unit_slots(
        std::span<const ordered_slot> slots, std::vector<unit_slot>& output
    ) {
        output.clear();
        output.reserve(slots.size());

        for (const ordered_slot& slot : slots) {
            if (!positive_finite(slot.aspect)) {
                return false;
            }

            const double aspect
                = slot.rotated ? 1.0 / slot.aspect : slot.aspect;
            if (!positive_finite(aspect)) {
                return false;
            }

            const double width = aspect >= 1.0 ? aspect : 1.0;
            const double height = aspect >= 1.0 ? 1.0 : 1.0 / aspect;
            if (!positive_finite(width) || !positive_finite(height)) {
                return false;
            }
            output.push_back({ width, height, slot.rotated });
        }
        return true;
    }

    [[nodiscard]] bool valid_request(
        const ordered_layout_request& request, std::vector<unit_slot>& units
    ) {
        if (!positive_finite(request.viewport.width)
            || !positive_finite(request.viewport.height)
            || !nonnegative_finite(request.minimum_short_side)
            || !nonnegative_finite(request.spacing)
            || !valid_policy(request.policy)
            || !valid_algorithm(request.algorithm)) {
            return false;
        }
        return make_unit_slots(request.items, units);
    }

    [[nodiscard]] double
    cross_limit(extent viewport, shelf_axis axis) noexcept {
        return axis == shelf_axis::columns ? viewport.height : viewport.width;
    }

    [[nodiscard]] double main_limit(extent viewport, shelf_axis axis) noexcept {
        return axis == shelf_axis::columns ? viewport.width : viewport.height;
    }

    [[nodiscard]] bool make_axis_items(
        const std::vector<unit_slot>& units, double short_side, shelf_axis axis,
        std::vector<axis_item>& items
    ) {
        items.clear();
        items.reserve(units.size());
        for (const unit_slot& unit : units) {
            const double width = unit.width * short_side;
            const double height = unit.height * short_side;
            if (!nonnegative_finite(width) || !nonnegative_finite(height)) {
                return false;
            }
            if (axis == shelf_axis::columns) {
                items.push_back({ width, height });
            } else {
                items.push_back({ height, width });
            }
        }
        return true;
    }

    [[nodiscard]] group_geometry make_group_geometry(
        const std::vector<axis_item>& items, std::size_t begin, std::size_t end,
        double spacing, double bounded_cross
    ) noexcept {
        double main = 0.0;
        double cross = 0.0;
        for (std::size_t index = begin; index < end; ++index) {
            main = std::max(main, items[index].main);
            cross += items[index].cross;
        }
        if (end > begin + 1) {
            cross += spacing * static_cast<double>(end - begin - 1);
        }
        return {
            begin, end, main, cross, std::max(0.0, bounded_cross - cross),
        };
    }

    [[nodiscard]] partition_result greedy_partition(
        const std::vector<unit_slot>& units, double short_side, double spacing,
        extent viewport, shelf_axis axis
    ) {
        partition_result result;
        std::vector<axis_item> items;
        if (!make_axis_items(units, short_side, axis, items)) {
            return result;
        }
        if (items.empty()) {
            result.feasible = true;
            return result;
        }

        const double bounded_cross = cross_limit(viewport, axis);
        std::size_t begin = 0;
        while (begin < items.size()) {
            std::size_t end = begin;
            double group_cross = 0.0;
            double group_main = 0.0;

            while (end < items.size()) {
                const double candidate_cross = group_cross
                    + (end == begin ? 0.0 : spacing) + items[end].cross;
                if (!std::isfinite(candidate_cross)
                    || exceeds(candidate_cross, bounded_cross)) {
                    break;
                }
                group_cross = candidate_cross;
                group_main = std::max(group_main, items[end].main);
                ++end;
            }

            if (end == begin || !std::isfinite(group_main)) {
                return {};
            }

            const double slack = std::max(0.0, bounded_cross - group_cross);
            result.groups.push_back(
                { begin, end, group_main, group_cross, slack }
            );
            result.raggedness = square_and_add(result.raggedness, slack);
            begin = end;
        }

        for (std::size_t index = 0; index < result.groups.size(); ++index) {
            if (index != 0) {
                result.extent += spacing;
            }
            result.extent += result.groups[index].main_extent;
            if (!std::isfinite(result.extent)) {
                return {};
            }
        }
        result.feasible = true;
        return result;
    }

    [[nodiscard]] partition_result exact_partition(
        const std::vector<unit_slot>& units, double short_side, double spacing,
        extent viewport, shelf_axis axis
    ) {
        partition_result result;
        std::vector<axis_item> items;
        if (!make_axis_items(units, short_side, axis, items)) {
            return result;
        }
        const std::size_t count = items.size();
        if (count == 0) {
            result.feasible = true;
            return result;
        }

        const double infinity = std::numeric_limits<double>::infinity();
        const double bounded_cross = cross_limit(viewport, axis);
        std::vector<double> costs(count + 1, infinity);
        std::vector<std::size_t> previous(count + 1, no_index);
        costs[0] = 0.0;

        for (std::size_t end = 1; end <= count; ++end) {
            double group_cross = 0.0;
            double group_main = 0.0;
            for (std::size_t start = end; start-- > 0;) {
                group_cross += items[start].cross;
                if (start + 1 < end) {
                    group_cross += spacing;
                }
                group_main = std::max(group_main, items[start].main);
                if (!std::isfinite(group_cross) || !std::isfinite(group_main)
                    || exceeds(group_cross, bounded_cross)) {
                    break;
                }
                if (!std::isfinite(costs[start])) {
                    continue;
                }

                const double candidate
                    = costs[start] + group_main + (start == 0 ? 0.0 : spacing);
                if (!std::isfinite(candidate)) {
                    continue;
                }
                if (previous[end] == no_index
                    || meaningfully_less(candidate, costs[end])) {
                    costs[end] = candidate;
                    previous[end] = start;
                }
            }
        }

        if (previous[count] == no_index) {
            return result;
        }

        std::size_t end = count;
        while (end > 0) {
            const std::size_t begin = previous[end];
            result.groups.push_back(
                make_group_geometry(items, begin, end, spacing, bounded_cross)
            );
            end = begin;
        }
        std::reverse(result.groups.begin(), result.groups.end());
        for (const group_geometry& group : result.groups) {
            result.raggedness = square_and_add(result.raggedness, group.slack);
        }
        result.extent = costs[count];
        result.feasible = true;
        return result;
    }

    [[nodiscard]] bool better_balanced_candidate(
        double extent_value, double raggedness, std::size_t group_count,
        const balanced_state& incumbent
    ) noexcept {
        if (!incumbent.reachable) {
            return true;
        }
        if (meaningfully_less(extent_value, incumbent.extent)) {
            return true;
        }
        if (!nearly_equal(extent_value, incumbent.extent)) {
            return false;
        }
        if (meaningfully_less(raggedness, incumbent.raggedness)) {
            return true;
        }
        return nearly_equal(raggedness, incumbent.raggedness)
            && group_count < incumbent.group_count;
    }

    [[nodiscard]] partition_result balanced_partition(
        const std::vector<unit_slot>& units, double short_side, double spacing,
        extent viewport, shelf_axis axis
    ) {
        partition_result result;
        std::vector<axis_item> items;
        if (!make_axis_items(units, short_side, axis, items)) {
            return result;
        }
        const std::size_t count = items.size();
        if (count == 0) {
            result.feasible = true;
            return result;
        }

        const double bounded_cross = cross_limit(viewport, axis);
        std::vector<balanced_state> states(count + 1);
        states[0].reachable = true;

        for (std::size_t end = 1; end <= count; ++end) {
            double group_cross = 0.0;
            double group_main = 0.0;
            for (std::size_t start = end; start-- > 0;) {
                group_cross += items[start].cross;
                if (start + 1 < end) {
                    group_cross += spacing;
                }
                group_main = std::max(group_main, items[start].main);
                if (!std::isfinite(group_cross) || !std::isfinite(group_main)
                    || exceeds(group_cross, bounded_cross)) {
                    break;
                }
                if (!states[start].reachable) {
                    continue;
                }

                const double candidate_extent = states[start].extent
                    + group_main + (start == 0 ? 0.0 : spacing);
                if (!std::isfinite(candidate_extent)) {
                    continue;
                }
                const double slack = std::max(0.0, bounded_cross - group_cross);
                const double candidate_raggedness
                    = square_and_add(states[start].raggedness, slack);
                const std::size_t candidate_groups
                    = states[start].group_count + 1;

                if (better_balanced_candidate(
                        candidate_extent, candidate_raggedness,
                        candidate_groups, states[end]
                    )) {
                    states[end] = {
                        candidate_extent,
                        candidate_raggedness,
                        candidate_groups,
                        start,
                        true,
                    };
                }
            }
        }

        if (!states[count].reachable) {
            return result;
        }

        std::size_t end = count;
        while (end > 0) {
            const std::size_t begin = states[end].previous;
            result.groups.push_back(
                make_group_geometry(items, begin, end, spacing, bounded_cross)
            );
            end = begin;
        }
        std::reverse(result.groups.begin(), result.groups.end());
        result.extent = states[count].extent;
        result.raggedness = states[count].raggedness;
        result.feasible = true;
        return result;
    }

    [[nodiscard]] partition_result partition_slots(
        const std::vector<unit_slot>& units, double short_side, double spacing,
        extent viewport, shelf_axis axis, ordered_layout_algorithm algorithm
    ) {
        switch (algorithm) {
        case ordered_layout_algorithm::greedy_shelf:
            return greedy_partition(units, short_side, spacing, viewport, axis);
        case ordered_layout_algorithm::exact_shelf:
            return exact_partition(units, short_side, spacing, viewport, axis);
        case ordered_layout_algorithm::balanced_shelf:
            return balanced_partition(
                units, short_side, spacing, viewport, axis
            );
        case ordered_layout_algorithm::automatic:
        case ordered_layout_algorithm::equal_grid:
            break;
        }
        return {};
    }

    [[nodiscard]] double individual_cross_short_limit(
        const std::vector<unit_slot>& units, extent viewport, shelf_axis axis
    ) noexcept {
        double limit = cross_limit(viewport, axis);
        for (const unit_slot& unit : units) {
            const double unit_cross
                = axis == shelf_axis::columns ? unit.height : unit.width;
            limit = std::min(limit, cross_limit(viewport, axis) / unit_cross);
        }
        return limit;
    }

    [[nodiscard]] double find_no_scroll_scale(
        const std::vector<unit_slot>& units, double spacing, extent viewport,
        shelf_axis axis, ordered_layout_algorithm algorithm
    ) {
        const double maximum
            = individual_cross_short_limit(units, viewport, axis);
        const double bounded_main = main_limit(viewport, axis);
        const auto fits = [&](double short_side) {
            const partition_result partition = partition_slots(
                units, short_side, spacing, viewport, axis, algorithm
            );
            return partition.feasible
                && !exceeds(partition.extent, bounded_main);
        };

        if (fits(maximum)) {
            return maximum;
        }

        double lower = 0.0;
        double upper = maximum;
        for (std::size_t iteration = 0; iteration < scale_search_iterations;
             ++iteration) {
            const double middle = lower + (upper - lower) / 2.0;
            if (fits(middle)) {
                lower = middle;
            } else {
                upper = middle;
            }
        }
        return lower;
    }

    [[nodiscard]] ordered_layout_result place_partition(
        const std::vector<unit_slot>& units, const partition_result& partition,
        double short_side, double spacing, extent viewport, shelf_axis axis,
        double requested_minimum, ordered_layout_algorithm algorithm
    ) {
        ordered_layout_result result;
        result.algorithm = algorithm;
        result.axis = axis;
        if (!partition.feasible) {
            return result;
        }

        result.rectangles.reserve(units.size());
        result.groups.reserve(partition.groups.size());
        double main_position = 0.0;

        for (std::size_t group_index = 0; group_index < partition.groups.size();
             ++group_index) {
            const group_geometry& group = partition.groups[group_index];
            result.groups.push_back({ group.begin, group.end });
            double cross_position = 0.0;

            for (std::size_t index = group.begin; index < group.end; ++index) {
                const double width = units[index].width * short_side;
                const double height = units[index].height * short_side;
                rectangle placed;
                placed.x = axis == shelf_axis::columns ? main_position
                                                       : cross_position;
                placed.y = axis == shelf_axis::columns ? cross_position
                                                       : main_position;
                placed.width = width;
                placed.height = height;
                placed.source_index = index;
                placed.rotated = units[index].rotated;
                if (!is_finite(placed)) {
                    return {};
                }
                result.rectangles.push_back(placed);
                cross_position
                    += (axis == shelf_axis::columns ? height : width) + spacing;
            }

            main_position += group.main_extent;
            if (group_index + 1 < partition.groups.size()) {
                main_position += spacing;
            }
        }

        result.common_short_side = short_side;
        result.primary_extent = partition.extent;
        result.raggedness = partition.raggedness;
        result.threshold_met
            = short_side + comparison_tolerance(short_side, requested_minimum)
            >= requested_minimum;
        result.used_scroll
            = exceeds(partition.extent, main_limit(viewport, axis));
        result.content = viewport;
        if (axis == shelf_axis::columns) {
            result.content.width = std::max(viewport.width, partition.extent);
        } else {
            result.content.height = std::max(viewport.height, partition.extent);
        }
        return result;
    }

    [[nodiscard]] ordered_layout_result solve_shelf_axis(
        const std::vector<unit_slot>& units, extent viewport, shelf_axis axis,
        double minimum_short_side, double spacing,
        ordered_layout_algorithm algorithm
    ) {
        const double fit
            = find_no_scroll_scale(units, spacing, viewport, axis, algorithm);
        double short_side = fit;
        if (meaningfully_less(fit, minimum_short_side)) {
            short_side = std::min(
                minimum_short_side,
                individual_cross_short_limit(units, viewport, axis)
            );
        }

        const partition_result partition = partition_slots(
            units, short_side, spacing, viewport, axis, algorithm
        );
        return place_partition(
            units, partition, short_side, spacing, viewport, axis,
            minimum_short_side, algorithm
        );
    }

    struct grid_choice {
        double short_side { 0.0 };
        std::size_t rows { 0 };
        std::size_t columns { 0 };
        double cell_width { 0.0 };
        double cell_height { 0.0 };
        bool feasible { false };
    };

    [[nodiscard]] std::size_t
    ceiling_divide(std::size_t numerator, std::size_t denominator) noexcept {
        return numerator / denominator + (numerator % denominator != 0 ? 1 : 0);
    }

    [[nodiscard]] std::pair<double, double>
    maximum_unit_size(const std::vector<unit_slot>& units) noexcept {
        double width = 0.0;
        double height = 0.0;
        for (const unit_slot& unit : units) {
            width = std::max(width, unit.width);
            height = std::max(height, unit.height);
        }
        return { width, height };
    }

    [[nodiscard]] grid_choice best_no_scroll_grid(
        const std::vector<unit_slot>& units, extent viewport, double spacing
    ) noexcept {
        grid_choice best;
        const auto [maximum_width, maximum_height] = maximum_unit_size(units);

        for (std::size_t rows = 1; rows <= units.size(); ++rows) {
            const std::size_t columns = ceiling_divide(units.size(), rows);
            const double horizontal_gaps
                = spacing * static_cast<double>(columns - 1);
            const double vertical_gaps
                = spacing * static_cast<double>(rows - 1);
            if (!std::isfinite(horizontal_gaps) || !std::isfinite(vertical_gaps)
                || horizontal_gaps >= viewport.width
                || vertical_gaps >= viewport.height) {
                continue;
            }
            const double cell_width = (viewport.width - horizontal_gaps)
                / static_cast<double>(columns);
            const double cell_height
                = (viewport.height - vertical_gaps) / static_cast<double>(rows);
            const double short_side = std::min(
                cell_width / maximum_width, cell_height / maximum_height
            );
            if (!positive_finite(short_side)) {
                continue;
            }
            if (!best.feasible
                || meaningfully_less(best.short_side, short_side)) {
                best = {
                    short_side, rows, columns, cell_width, cell_height, true,
                };
            }
        }
        return best;
    }

    [[nodiscard]] ordered_layout_result place_grid(
        const std::vector<unit_slot>& units, double short_side,
        std::size_t columns, double cell_width, double cell_height,
        double spacing, extent viewport, shelf_axis axis,
        double requested_minimum, ordered_layout_algorithm algorithm,
        double primary_extent
    ) {
        ordered_layout_result result;
        result.algorithm = algorithm;
        result.axis = axis;
        result.rectangles.reserve(units.size());

        for (std::size_t index = 0; index < units.size(); ++index) {
            const std::size_t row = index / columns;
            const std::size_t column = index % columns;
            const double width = units[index].width * short_side;
            const double height = units[index].height * short_side;
            rectangle placed {
                static_cast<double>(column) * (cell_width + spacing)
                    + (cell_width - width) / 2.0,
                static_cast<double>(row) * (cell_height + spacing)
                    + (cell_height - height) / 2.0,
                width,
                height,
                index,
                units[index].rotated,
            };
            if (!is_finite(placed)) {
                return {};
            }
            result.rectangles.push_back(placed);
        }

        result.content = viewport;
        if (axis == shelf_axis::columns) {
            result.content.width = std::max(viewport.width, primary_extent);
        } else {
            result.content.height = std::max(viewport.height, primary_extent);
        }
        result.common_short_side = short_side;
        result.primary_extent = primary_extent;
        result.used_scroll
            = exceeds(primary_extent, main_limit(viewport, axis));
        result.threshold_met
            = short_side + comparison_tolerance(short_side, requested_minimum)
            >= requested_minimum;
        return result;
    }

    [[nodiscard]] ordered_layout_result solve_equal_grid(
        const std::vector<unit_slot>& units,
        const ordered_layout_request& request
    ) {
        const grid_choice best
            = best_no_scroll_grid(units, request.viewport, request.spacing);
        if (!best.feasible) {
            ordered_layout_result result;
            result.algorithm = ordered_layout_algorithm::equal_grid;
            result.axis = axis_for_policy(request.policy);
            return result;
        }

        const shelf_axis requested_axis = axis_for_policy(request.policy);
        if (request.policy == scroll_policy::disabled
            || !meaningfully_less(
                best.short_side, request.minimum_short_side
            )) {
            return place_grid(
                units, best.short_side, best.columns, best.cell_width,
                best.cell_height, request.spacing, request.viewport,
                requested_axis, request.minimum_short_side,
                ordered_layout_algorithm::equal_grid,
                main_limit(request.viewport, requested_axis)
            );
        }

        const auto [maximum_width, maximum_height] = maximum_unit_size(units);
        if (request.policy == scroll_policy::horizontal) {
            const double short_side = std::min(
                request.minimum_short_side,
                request.viewport.height / maximum_height
            );
            const double cell_width = maximum_width * short_side;
            const double cell_height = maximum_height * short_side;
            const double rows_value = std::floor(
                (request.viewport.height + request.spacing)
                / (cell_height + request.spacing)
            );
            const std::size_t rows = std::min(
                units.size(),
                std::max<std::size_t>(1, static_cast<std::size_t>(rows_value))
            );
            const std::size_t columns = ceiling_divide(units.size(), rows);
            const double primary = static_cast<double>(columns) * cell_width
                + static_cast<double>(columns - 1) * request.spacing;
            return place_grid(
                units, short_side, columns, cell_width, cell_height,
                request.spacing, request.viewport, shelf_axis::columns,
                request.minimum_short_side,
                ordered_layout_algorithm::equal_grid, primary
            );
        }

        const double short_side = std::min(
            request.minimum_short_side, request.viewport.width / maximum_width
        );
        const double cell_width = maximum_width * short_side;
        const double cell_height = maximum_height * short_side;
        const double columns_value = std::floor(
            (request.viewport.width + request.spacing)
            / (cell_width + request.spacing)
        );
        const std::size_t columns = std::min(
            units.size(),
            std::max<std::size_t>(1, static_cast<std::size_t>(columns_value))
        );
        const std::size_t rows = ceiling_divide(units.size(), columns);
        const double primary = static_cast<double>(rows) * cell_height
            + static_cast<double>(rows - 1) * request.spacing;
        return place_grid(
            units, short_side, columns, cell_width, cell_height,
            request.spacing, request.viewport, shelf_axis::rows,
            request.minimum_short_side, ordered_layout_algorithm::equal_grid,
            primary
        );
    }

    [[nodiscard]] bool complete_solution(
        const ordered_layout_result& result, std::size_t expected
    ) noexcept {
        return result.rectangles.size() == expected;
    }

} // namespace

ordered_layout_algorithm select_ordered_layout_algorithm(
    const ordered_layout_request& request
) noexcept {
    if (request.algorithm != ordered_layout_algorithm::automatic
        && valid_algorithm(request.algorithm)) {
        return request.algorithm;
    }
    return request.items.size() <= balanced_shelf_dispatch_limit
        ? ordered_layout_algorithm::balanced_shelf
        : ordered_layout_algorithm::greedy_shelf;
}

ordered_layout_result
layout_ordered_slots(const ordered_layout_request& request) {
    ordered_layout_result invalid;
    invalid.algorithm = select_ordered_layout_algorithm(request);
    invalid.axis = axis_for_policy(request.policy);

    std::vector<unit_slot> units;
    if (!valid_request(request, units)) {
        return invalid;
    }

    if (units.empty()) {
        invalid.content = request.viewport;
        invalid.threshold_met = true;
        return invalid;
    }

    const ordered_layout_algorithm algorithm
        = select_ordered_layout_algorithm(request);
    if (algorithm == ordered_layout_algorithm::equal_grid) {
        return solve_equal_grid(units, request);
    }

    if (request.policy == scroll_policy::horizontal
        || request.policy == scroll_policy::vertical) {
        return solve_shelf_axis(
            units, request.viewport, axis_for_policy(request.policy),
            request.minimum_short_side, request.spacing, algorithm
        );
    }

    ordered_layout_result columns = solve_shelf_axis(
        units, request.viewport, shelf_axis::columns, 0.0, request.spacing,
        algorithm
    );
    ordered_layout_result rows = solve_shelf_axis(
        units, request.viewport, shelf_axis::rows, 0.0, request.spacing,
        algorithm
    );

    const bool columns_valid
        = complete_solution(columns, units.size()) && !columns.used_scroll;
    const bool rows_valid
        = complete_solution(rows, units.size()) && !rows.used_scroll;
    if (!columns_valid && !rows_valid) {
        return invalid;
    }

    ordered_layout_result result;
    if (!rows_valid
        || (columns_valid
            && !meaningfully_less(
                columns.common_short_side, rows.common_short_side
            ))) {
        result = std::move(columns);
    } else {
        result = std::move(rows);
    }
    result.content = request.viewport;
    result.used_scroll = false;
    result.threshold_met
        = result.common_short_side
            + comparison_tolerance(
                result.common_short_side, request.minimum_short_side
            )
        >= request.minimum_short_side;
    return result;
}

std::string_view algorithm_name(ordered_layout_algorithm algorithm) noexcept {
    switch (algorithm) {
    case ordered_layout_algorithm::automatic:
        return "automatic";
    case ordered_layout_algorithm::equal_grid:
        return "equal_grid";
    case ordered_layout_algorithm::greedy_shelf:
        return "greedy_shelf";
    case ordered_layout_algorithm::exact_shelf:
        return "exact_shelf";
    case ordered_layout_algorithm::balanced_shelf:
        return "balanced_shelf";
    }
    return "unknown";
}

} // namespace packing
