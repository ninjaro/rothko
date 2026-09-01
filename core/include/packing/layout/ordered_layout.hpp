#ifndef PACKING_LAYOUT_ORDERED_LAYOUT_HPP
#define PACKING_LAYOUT_ORDERED_LAYOUT_HPP

#include "packing/geometry.hpp"

#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace packing {

enum class scroll_policy {
    horizontal,
    vertical,
    disabled,
};

enum class shelf_axis {
    columns,
    rows,
};

enum class ordered_layout_algorithm {
    automatic,
    equal_grid,
    greedy_shelf,
    exact_shelf,
    balanced_shelf,
};

struct ordered_slot {
    double aspect { 1.0 };
    bool rotated { false };
};

struct ordered_group {
    std::size_t begin { 0 };
    std::size_t end { 0 };
};

struct ordered_layout_request {
    extent viewport;
    std::span<const ordered_slot> items;
    scroll_policy policy { scroll_policy::horizontal };
    double minimum_short_side { 0.0 };
    double spacing { 0.0 };
    ordered_layout_algorithm algorithm { ordered_layout_algorithm::automatic };
};

struct ordered_layout_result {
    std::vector<rectangle> rectangles;
    std::vector<ordered_group> groups;
    extent content;
    double common_short_side { 0.0 };
    double primary_extent { 0.0 };
    double raggedness { 0.0 };
    bool used_scroll { false };
    bool threshold_met { false };
    shelf_axis axis { shelf_axis::columns };
    ordered_layout_algorithm algorithm { ordered_layout_algorithm::automatic };
};

inline constexpr std::size_t balanced_shelf_dispatch_limit = 128;

[[nodiscard]] ordered_layout_algorithm
select_ordered_layout_algorithm(const ordered_layout_request& request) noexcept;
[[nodiscard]] ordered_layout_result
layout_ordered_slots(const ordered_layout_request& request);
[[nodiscard]] std::string_view
algorithm_name(ordered_layout_algorithm algorithm) noexcept;

} // namespace packing

#endif // PACKING_LAYOUT_ORDERED_LAYOUT_HPP
