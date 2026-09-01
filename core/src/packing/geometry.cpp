#include "packing/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace packing {

bool is_positive_finite(const extent value) noexcept {
    return std::isfinite(value.width) && std::isfinite(value.height)
        && value.width > 0.0 && value.height > 0.0;
}

bool is_finite(const rectangle value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.width) && std::isfinite(value.height)
        && value.width > 0.0 && value.height > 0.0;
}

bool intersects(
    const rectangle& lhs, const rectangle& rhs, const double epsilon
) noexcept {
    if (!is_finite(lhs) || !is_finite(rhs) || !std::isfinite(epsilon)) {
        return false;
    }

    return lhs.x + lhs.width > rhs.x + epsilon
        && rhs.x + rhs.width > lhs.x + epsilon
        && lhs.y + lhs.height > rhs.y + epsilon
        && rhs.y + rhs.height > lhs.y + epsilon;
}

double squared_distance(const point sample, const rectangle& item) noexcept {
    if (!std::isfinite(sample.x) || !std::isfinite(sample.y)
        || !is_finite(item)) {
        return std::numeric_limits<double>::infinity();
    }

    const double right = item.x + item.width;
    const double bottom = item.y + item.height;
    const double dx = sample.x < item.x
        ? item.x - sample.x
        : (sample.x > right ? sample.x - right : 0.0);
    const double dy = sample.y < item.y
        ? item.y - sample.y
        : (sample.y > bottom ? sample.y - bottom : 0.0);
    return dx * dx + dy * dy;
}

bool translate(
    const std::span<rectangle> rectangles, const point offset
) noexcept {
    if (!std::isfinite(offset.x) || !std::isfinite(offset.y)) {
        return false;
    }
    for (const rectangle& item : rectangles) {
        if (!is_finite(item) || !std::isfinite(item.x + offset.x)
            || !std::isfinite(item.y + offset.y)) {
            return false;
        }
    }
    for (rectangle& item : rectangles) {
        item.x += offset.x;
        item.y += offset.y;
    }
    return true;
}

bounds bounding_box(const std::span<const rectangle> rectangles) noexcept {
    if (rectangles.empty()) {
        return {};
    }

    bounds result {
        .left = std::numeric_limits<double>::infinity(),
        .top = std::numeric_limits<double>::infinity(),
        .right = -std::numeric_limits<double>::infinity(),
        .bottom = -std::numeric_limits<double>::infinity(),
    };

    for (const rectangle& item : rectangles) {
        if (!is_finite(item)) {
            return {};
        }
        result.left = std::min(result.left, item.x);
        result.top = std::min(result.top, item.y);
        result.right = std::max(result.right, item.x + item.width);
        result.bottom = std::max(result.bottom, item.y + item.height);
    }
    return result;
}

validation_report validate(
    const extent container, const std::span<const rectangle> rectangles,
    const double epsilon
) noexcept {
    validation_report result { .valid = false };
    if (!is_positive_finite(container) || !std::isfinite(epsilon)
        || epsilon < 0.0) {
        return result;
    }

    for (const rectangle& item : rectangles) {
        if (!is_finite(item) || item.x < -epsilon || item.y < -epsilon
            || item.x + item.width > container.width + epsilon
            || item.y + item.height > container.height + epsilon) {
            ++result.out_of_bounds_count;
        }
    }

    for (std::size_t lhs = 0; lhs < rectangles.size(); ++lhs) {
        for (std::size_t rhs = lhs + 1; rhs < rectangles.size(); ++rhs) {
            if (intersects(rectangles[lhs], rectangles[rhs], epsilon)) {
                ++result.overlap_count;
            }
        }
    }

    result.valid = result.out_of_bounds_count == 0 && result.overlap_count == 0;
    return result;
}

} // namespace packing
