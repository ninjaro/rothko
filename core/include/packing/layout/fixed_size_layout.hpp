#ifndef PACKING_LAYOUT_FIXED_SIZE_LAYOUT_HPP
#define PACKING_LAYOUT_FIXED_SIZE_LAYOUT_HPP

#include "packing/layout/ordered_layout.hpp"

#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace packing {

struct fixed_size_item {
    // This is the final requested size. The fixed-size task never rescales it.
    extent size;
};

enum class fixed_size_layout_algorithm {
    automatic,
    preserve_order_shelf,
    best_fit_decreasing_shelf,
};

struct fixed_size_layout_request {
    extent viewport;
    std::span<const fixed_size_item> items;
    scroll_policy policy { scroll_policy::horizontal };
    double spacing { 0.0 };
    fixed_size_layout_algorithm algorithm {
        fixed_size_layout_algorithm::automatic
    };
};

struct fixed_size_layout_result {
    // Rectangles remain indexed like request.items. placement_order records
    // the possibly reordered shelf traversal, and groups are ranges into it.
    std::vector<rectangle> rectangles;
    std::vector<std::size_t> placement_order;
    std::vector<ordered_group> groups;
    extent content;
    double primary_extent { 0.0 };
    bool used_scroll { false };
    shelf_axis axis { shelf_axis::columns };
    fixed_size_layout_algorithm algorithm {
        fixed_size_layout_algorithm::automatic
    };

    [[nodiscard]] bool complete(std::size_t expected_count) const noexcept {
        return rectangles.size() == expected_count
            && placement_order.size() == expected_count;
    }
};

inline constexpr double fixed_size_realtime_budget_milliseconds = 4.0;
inline constexpr std::size_t fixed_best_fit_dispatch_limit = 3072;

[[nodiscard]] fixed_size_layout_algorithm select_fixed_size_layout_algorithm(
    const fixed_size_layout_request& request
) noexcept;
[[nodiscard]] fixed_size_layout_result
layout_fixed_size_rectangles(const fixed_size_layout_request& request);
[[nodiscard]] std::string_view
algorithm_name(fixed_size_layout_algorithm algorithm) noexcept;

} // namespace packing

#endif // PACKING_LAYOUT_FIXED_SIZE_LAYOUT_HPP
