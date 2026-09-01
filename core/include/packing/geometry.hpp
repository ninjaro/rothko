#ifndef PACKING_GEOMETRY_HPP
#define PACKING_GEOMETRY_HPP

#include <cstddef>
#include <span>

namespace packing {

struct extent {
    double width { 0.0 };
    double height { 0.0 };
};

struct rectangle {
    double x { 0.0 };
    double y { 0.0 };
    double width { 0.0 };
    double height { 0.0 };
    std::size_t source_index { 0 };
    bool rotated { false };
};

struct bounds {
    double left { 0.0 };
    double top { 0.0 };
    double right { 0.0 };
    double bottom { 0.0 };

    [[nodiscard]] double width() const noexcept { return right - left; }

    [[nodiscard]] double height() const noexcept { return bottom - top; }

    [[nodiscard]] double area() const noexcept { return width() * height(); }
};

struct validation_report {
    bool valid { false };
    std::size_t out_of_bounds_count { 0 };
    std::size_t overlap_count { 0 };
};

[[nodiscard]] bool is_positive_finite(extent value) noexcept;
[[nodiscard]] bool is_finite(rectangle value) noexcept;
[[nodiscard]] bool intersects(
    const rectangle& lhs, const rectangle& rhs, double epsilon = 1e-9
) noexcept;
[[nodiscard]] bounds
bounding_box(std::span<const rectangle> rectangles) noexcept;
[[nodiscard]] validation_report validate(
    extent container, std::span<const rectangle> rectangles,
    double epsilon = 1e-9
) noexcept;

} // namespace packing

#endif // PACKING_GEOMETRY_HPP
