#include "packing/layout/reciprocal_layout.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

namespace packing {
namespace {

    constexpr double epsilon = 1e-10;

    void add_coordinate(std::vector<double>& values, const double value) {
        if (!std::isfinite(value)) {
            return;
        }
        for (const double existing : values) {
            if (std::abs(existing - value) <= epsilon) {
                return;
            }
        }
        values.push_back(value);
    }

    double positive_overlap(
        const double lhs_begin, const double lhs_end, const double rhs_begin,
        const double rhs_end
    ) noexcept {
        return std::max(
            0.0, std::min(lhs_end, rhs_end) - std::max(lhs_begin, rhs_begin)
        );
    }

    struct candidate_score {
        int contact_count { 0 };
        double contact_length { 0.0 };
        double bounding_area { 0.0 };
        double bottom { 0.0 };
        double right { 0.0 };
        bool rotated { false };
        double y { 0.0 };
        double x { 0.0 };

        [[nodiscard]] auto key() const noexcept {
            return std::tuple { -contact_count,
                                -contact_length,
                                bounding_area,
                                bottom,
                                right,
                                rotated,
                                y,
                                x };
        }
    };

    candidate_score score_candidate(
        const extent container, const rectangle& candidate,
        const std::vector<rectangle>& placed
    ) {
        candidate_score score {
            .bottom = candidate.y + candidate.height,
            .right = candidate.x + candidate.width,
            .rotated = candidate.rotated,
            .y = candidate.y,
            .x = candidate.x,
        };

        const auto add_contact = [&](const double length) {
            ++score.contact_count;
            score.contact_length += length;
        };

        if (std::abs(candidate.x) <= epsilon) {
            add_contact(candidate.height);
        }
        if (std::abs(candidate.y) <= epsilon) {
            add_contact(candidate.width);
        }
        if (std::abs(candidate.x + candidate.width - container.width)
            <= epsilon) {
            add_contact(candidate.height);
        }
        if (std::abs(candidate.y + candidate.height - container.height)
            <= epsilon) {
            add_contact(candidate.width);
        }

        double right = candidate.x + candidate.width;
        double bottom = candidate.y + candidate.height;
        for (const rectangle& item : placed) {
            const double vertical = positive_overlap(
                candidate.y, bottom, item.y, item.y + item.height
            );
            const double horizontal = positive_overlap(
                candidate.x, right, item.x, item.x + item.width
            );
            if (vertical > epsilon
                && (std::abs(candidate.x - item.x - item.width) <= epsilon
                    || std::abs(right - item.x) <= epsilon)) {
                add_contact(vertical);
            }
            if (horizontal > epsilon
                && (std::abs(candidate.y - item.y - item.height) <= epsilon
                    || std::abs(bottom - item.y) <= epsilon)) {
                add_contact(horizontal);
            }
            right = std::max(right, item.x + item.width);
            bottom = std::max(bottom, item.y + item.height);
        }
        score.bounding_area = right * bottom;
        return score;
    }

    bool fits(
        const extent container, const rectangle& candidate,
        const std::vector<rectangle>& placed
    ) noexcept {
        if (candidate.x < -epsilon || candidate.y < -epsilon
            || candidate.x + candidate.width > container.width + epsilon
            || candidate.y + candidate.height > container.height + epsilon) {
            return false;
        }
        return std::ranges::none_of(placed, [&](const rectangle& item) {
            return intersects(candidate, item, epsilon);
        });
    }

} // namespace

reciprocal_layout_result
layout_reciprocal_rectangles(const reciprocal_layout_request& request) {
    reciprocal_layout_result result;
    result.algorithm = request.algorithm;
    if (!is_positive_finite(request.container) || request.first_index == 0
        || request.algorithm
            != reciprocal_layout_algorithm::greedy_edge_contacts) {
        result.status = reciprocal_layout_status::invalid_request;
        return result;
    }
    if (request.count == 0) {
        result.status = reciprocal_layout_status::complete;
        return result;
    }
    if (request.count > reciprocal_layout_count_limit) {
        result.status = reciprocal_layout_status::limit_exceeded;
        return result;
    }
    // Every rectangle needs both k and k + 1.  Keep the successor of the last
    // requested index representable as well as the index itself.
    if (request.first_index
        > std::numeric_limits<std::size_t>::max() - request.count) {
        result.status = reciprocal_layout_status::invalid_request;
        return result;
    }

    result.rectangles.reserve(request.count);
    for (std::size_t offset = 0; offset < request.count; ++offset) {
        const std::size_t index = request.first_index + offset;
        const auto divisor = static_cast<double>(index);
        const auto next_divisor = static_cast<double>(index + 1);
        const extent normal {
            request.container.width / divisor,
            request.container.height / next_divisor,
        };

        std::vector<rectangle> orientations;
        orientations.push_back(
            rectangle {
                .width = normal.width,
                .height = normal.height,
                .source_index = index,
                .rotated = false,
            }
        );
        if (request.allow_rotation
            && std::abs(normal.width - normal.height) > epsilon) {
            orientations.push_back(
                rectangle {
                    .width = normal.height,
                    .height = normal.width,
                    .source_index = index,
                    .rotated = true,
                }
            );
        }

        bool found = false;
        rectangle best;
        candidate_score best_score;
        for (const rectangle& shape : orientations) {
            std::vector<double> xs;
            std::vector<double> ys;
            add_coordinate(xs, 0.0);
            add_coordinate(xs, request.container.width - shape.width);
            add_coordinate(ys, 0.0);
            add_coordinate(ys, request.container.height - shape.height);
            for (const rectangle& item : result.rectangles) {
                add_coordinate(xs, item.x + item.width);
                add_coordinate(xs, item.x - shape.width);
                add_coordinate(ys, item.y + item.height);
                add_coordinate(ys, item.y - shape.height);
            }
            std::ranges::sort(xs);
            std::ranges::sort(ys);

            for (const double y : ys) {
                for (const double x : xs) {
                    rectangle candidate = shape;
                    candidate.x = x;
                    candidate.y = y;
                    if (!fits(
                            request.container, candidate, result.rectangles
                        )) {
                        continue;
                    }
                    const candidate_score score = score_candidate(
                        request.container, candidate, result.rectangles
                    );
                    if (!found || score.key() < best_score.key()) {
                        found = true;
                        best = candidate;
                        best_score = score;
                    }
                }
            }
        }

        if (!found) {
            result.status = reciprocal_layout_status::placement_failed;
            return result;
        }
        result.rectangles.push_back(best);
    }

    result.status = reciprocal_layout_status::complete;
    return result;
}

std::string_view
algorithm_name(const reciprocal_layout_algorithm algorithm) noexcept {
    switch (algorithm) {
    case reciprocal_layout_algorithm::greedy_edge_contacts:
        return "greedy_edge_contacts";
    }
    return "unknown";
}

} // namespace packing
