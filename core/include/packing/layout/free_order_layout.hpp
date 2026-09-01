#ifndef PACKING_LAYOUT_FREE_ORDER_LAYOUT_HPP
#define PACKING_LAYOUT_FREE_ORDER_LAYOUT_HPP

#include "packing/layout/ordered_layout.hpp"

#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace packing {

enum class free_order_layout_algorithm {
    automatic,
    sorted_shelf,
    multistart_shelf,
};

struct free_order_layout_request {
    extent viewport;
    std::span<const ordered_slot> items;
    scroll_policy policy { scroll_policy::horizontal };
    double minimum_short_side { 0.0 };
    double spacing { 0.0 };
    free_order_layout_algorithm algorithm {
        free_order_layout_algorithm::automatic
    };
};

struct free_order_layout_result {
    // Rectangles remain indexed like request.items. placement_order records
    // the permutation used by the solver, and groups are ranges into it.
    std::vector<rectangle> rectangles;
    std::vector<std::size_t> placement_order;
    std::vector<ordered_group> groups;
    extent content;
    double common_short_side { 0.0 };
    double primary_extent { 0.0 };
    double raggedness { 0.0 };
    bool used_scroll { false };
    bool threshold_met { false };
    shelf_axis axis { shelf_axis::columns };
    free_order_layout_algorithm algorithm {
        free_order_layout_algorithm::automatic
    };

    [[nodiscard]] bool complete(std::size_t expected_count) const noexcept {
        return rectangles.size() == expected_count
            && placement_order.size() == expected_count;
    }
};

// The deterministic multistart policy is intended to stay within this
// interactive-layout budget on the benchmark matrix. Above the measured
// count boundary, automatic falls back to the O(N log N) sorted shelf.
inline constexpr double free_order_realtime_budget_milliseconds = 8.0;
inline constexpr std::size_t free_order_multistart_dispatch_limit = 256;

[[nodiscard]] free_order_layout_algorithm select_free_order_layout_algorithm(
    const free_order_layout_request& request
) noexcept;
[[nodiscard]] free_order_layout_result
layout_free_order_slots(const free_order_layout_request& request);
[[nodiscard]] std::string_view
algorithm_name(free_order_layout_algorithm algorithm) noexcept;

} // namespace packing

#endif // PACKING_LAYOUT_FREE_ORDER_LAYOUT_HPP
