#ifndef PACKING_SELECTION_SPREAD_LAYOUT_HPP
#define PACKING_SELECTION_SPREAD_LAYOUT_HPP

#include "packing/geometry.hpp"

#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace packing {

enum class spread_layout_algorithm {
    automatic,
    greedy_coverage,
    coverage_with_swaps,
};

struct spread_layout_request {
    extent container;
    std::span<const rectangle> candidates;
    std::size_t count { 0 };
    spread_layout_algorithm algorithm { spread_layout_algorithm::automatic };
    std::size_t swap_passes { 2 };
    std::size_t maximum_samples { 2048 };
};

struct spread_layout_result {
    std::vector<rectangle> rectangles;
    spread_layout_algorithm algorithm { spread_layout_algorithm::automatic };
};

inline constexpr std::size_t spread_swap_dispatch_limit = 32;

[[nodiscard]] spread_layout_algorithm
select_spread_layout_algorithm(const spread_layout_request& request) noexcept;
[[nodiscard]] spread_layout_result
select_spread_layout(const spread_layout_request& request);
[[nodiscard]] std::string_view
algorithm_name(spread_layout_algorithm algorithm) noexcept;

} // namespace packing

#endif // PACKING_SELECTION_SPREAD_LAYOUT_HPP
