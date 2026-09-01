#ifndef PACKING_LAYOUT_EQUAL_RECTANGLES_HPP
#define PACKING_LAYOUT_EQUAL_RECTANGLES_HPP

#include "packing/geometry.hpp"

#include <cstddef>
#include <string_view>
#include <vector>

namespace packing {

enum class orientation_constraint {
    allow_rotation,
    horizontal_only,
    vertical_only,
};

enum class equal_packing_algorithm {
    automatic,
    certified_frame,
    count_binary_frame,
    horizontal_grid,
    vertical_grid,
};

struct equal_packing_request {
    extent container;
    extent item;
    std::size_t count { 0 };
    orientation_constraint orientation {
        orientation_constraint::allow_rotation
    };
    equal_packing_algorithm algorithm { equal_packing_algorithm::automatic };
    double long_side_step { 0.5 };
};

struct equal_packing_result {
    double scale { 0.0 };
    std::vector<rectangle> rectangles;
    equal_packing_algorithm algorithm { equal_packing_algorithm::automatic };

    [[nodiscard]] bool complete(std::size_t requested_count) const noexcept {
        return scale > 0.0 && rectangles.size() == requested_count;
    }
};

// The frame algorithms can produce more valid positions than the request
// ultimately needs. Keeping this intermediate result explicit lets compact and
// coverage selectors reuse the same scale search without duplicating frame
// generation or depending on the packer's default post-processing policy.
struct equal_candidate_result {
    double scale { 0.0 };
    std::vector<rectangle> rectangles;
    equal_packing_algorithm algorithm { equal_packing_algorithm::automatic };

    [[nodiscard]] bool sufficient(std::size_t requested_count) const noexcept {
        return scale > 0.0 && rectangles.size() >= requested_count;
    }
};

[[nodiscard]] equal_packing_algorithm
select_equal_packing_algorithm(const equal_packing_request& request) noexcept;
[[nodiscard]] equal_candidate_result
generate_equal_rectangle_candidates(const equal_packing_request& request);
[[nodiscard]] equal_packing_result
pack_equal_rectangles(const equal_packing_request& request);
[[nodiscard]] std::string_view
algorithm_name(equal_packing_algorithm algorithm) noexcept;

} // namespace packing

#endif // PACKING_LAYOUT_EQUAL_RECTANGLES_HPP
