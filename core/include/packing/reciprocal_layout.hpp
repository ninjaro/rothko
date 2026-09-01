#ifndef PACKING_RECIPROCAL_LAYOUT_HPP
#define PACKING_RECIPROCAL_LAYOUT_HPP

#include "packing/geometry.hpp"

#include <cstddef>
#include <string_view>
#include <vector>

namespace packing {

enum class reciprocal_layout_algorithm {
    greedy_edge_contacts,
};

enum class reciprocal_layout_status {
    complete,
    invalid_request,
    limit_exceeded,
    placement_failed,
};

struct reciprocal_layout_request {
    extent container { 1.0, 1.0 };
    std::size_t first_index { 1 };
    std::size_t count { 0 };
    bool allow_rotation { true };
    reciprocal_layout_algorithm algorithm {
        reciprocal_layout_algorithm::greedy_edge_contacts
    };
};

struct reciprocal_layout_result {
    std::vector<rectangle> rectangles;
    reciprocal_layout_status status {
        reciprocal_layout_status::invalid_request
    };
    reciprocal_layout_algorithm algorithm {
        reciprocal_layout_algorithm::greedy_edge_contacts
    };
};

// Candidate corners and collision checks are intentionally bounded while this
// remains a non-backtracking research placeholder, not a general proof solver.
inline constexpr std::size_t reciprocal_layout_count_limit = 64;

[[nodiscard]] reciprocal_layout_result
layout_reciprocal_rectangles(const reciprocal_layout_request& request);
[[nodiscard]] std::string_view
algorithm_name(reciprocal_layout_algorithm algorithm) noexcept;

} // namespace packing

#endif // PACKING_RECIPROCAL_LAYOUT_HPP
