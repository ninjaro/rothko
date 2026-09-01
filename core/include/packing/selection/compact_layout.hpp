#ifndef PACKING_SELECTION_COMPACT_LAYOUT_HPP
#define PACKING_SELECTION_COMPACT_LAYOUT_HPP

#include "packing/geometry.hpp"

#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace packing {

enum class compact_layout_algorithm {
    automatic,
    exact_bounding_box,
    void_refined,
};

struct compact_layout_request {
    extent container;
    std::span<const rectangle> candidates;
    std::size_t count { 0 };
    compact_layout_algorithm algorithm { compact_layout_algorithm::automatic };
    std::size_t refinement_passes { 2 };
    std::size_t maximum_samples { 2048 };
};

struct compact_layout_result {
    std::vector<rectangle> rectangles;
    compact_layout_algorithm algorithm { compact_layout_algorithm::automatic };
};

inline constexpr std::size_t compact_refinement_count_limit = 32;
inline constexpr std::size_t compact_refinement_surplus_limit = 8;

[[nodiscard]] compact_layout_algorithm
select_compact_layout_algorithm(const compact_layout_request& request) noexcept;
[[nodiscard]] compact_layout_result
select_compact_layout(const compact_layout_request& request);
[[nodiscard]] std::string_view
algorithm_name(compact_layout_algorithm algorithm) noexcept;

} // namespace packing

#endif // PACKING_SELECTION_COMPACT_LAYOUT_HPP
