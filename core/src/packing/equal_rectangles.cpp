#include "packing/equal_rectangles.hpp"

#include "packing/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <tuple>
#include <utility>
#include <vector>

namespace packing {
namespace {

    constexpr std::size_t maximum_frame_recursion_depth = 4096;
    constexpr std::uint64_t maximum_certified_scan_steps = 1'000'000;

    struct canonical_item {
        double long_side { 0.0 };
        double short_side { 0.0 };
        bool horizontal_rotated { false };
        bool vertical_rotated { true };
    };

    [[nodiscard]] canonical_item canonicalize(extent item) noexcept {
        if (item.width >= item.height) {
            return { item.width, item.height, false, true };
        }
        return { item.height, item.width, true, false };
    }

    [[nodiscard]] bool
    valid_orientation(orientation_constraint orientation) noexcept {
        switch (orientation) {
        case orientation_constraint::allow_rotation:
        case orientation_constraint::horizontal_only:
        case orientation_constraint::vertical_only:
            return true;
        }
        return false;
    }

    [[nodiscard]] bool
    valid_algorithm(equal_packing_algorithm algorithm) noexcept {
        switch (algorithm) {
        case equal_packing_algorithm::automatic:
        case equal_packing_algorithm::certified_frame:
        case equal_packing_algorithm::count_binary_frame:
        case equal_packing_algorithm::horizontal_grid:
        case equal_packing_algorithm::vertical_grid:
            return true;
        }
        return false;
    }

    [[nodiscard]] bool
    valid_request(const equal_packing_request& request) noexcept {
        return request.count > 0 && is_positive_finite(request.container)
            && is_positive_finite(request.item)
            && std::isfinite(request.long_side_step)
            && request.long_side_step > 0.0
            && valid_orientation(request.orientation)
            && valid_algorithm(request.algorithm);
    }

    [[nodiscard]] bool compatible(
        orientation_constraint orientation, equal_packing_algorithm algorithm
    ) noexcept {
        switch (algorithm) {
        case equal_packing_algorithm::certified_frame:
        case equal_packing_algorithm::count_binary_frame:
            return orientation == orientation_constraint::allow_rotation;
        case equal_packing_algorithm::horizontal_grid:
            return orientation != orientation_constraint::vertical_only;
        case equal_packing_algorithm::vertical_grid:
            return orientation != orientation_constraint::horizontal_only;
        case equal_packing_algorithm::automatic:
            return false;
        }
        return false;
    }

    [[nodiscard]] double
    area_scale_upper_bound(const equal_packing_request& request) noexcept {
        const long double width_ratio
            = static_cast<long double>(request.container.width)
            / static_cast<long double>(request.item.width);
        const long double height_ratio
            = static_cast<long double>(request.container.height)
            / static_cast<long double>(request.item.height);
        const long double area_per_item = width_ratio * height_ratio
            / static_cast<long double>(request.count);
        if (!std::isfinite(area_per_item) || area_per_item <= 0.0L) {
            return 0.0;
        }

        const long double scale = std::sqrt(area_per_item);
        if (!std::isfinite(scale) || scale <= 0.0L
            || scale > static_cast<long double>(
                   std::numeric_limits<double>::max()
               )) {
            return 0.0;
        }
        return static_cast<double>(scale);
    }

    class frame_generator {
    public:
        frame_generator(
            extent container, canonical_item item, double scale,
            std::size_t expected_count
        )
            : container_(container)
            , item_(item)
            , long_side_(item.long_side * scale)
            , short_side_(item.short_side * scale) {
            rectangles_.reserve(expected_count);
        }

        [[nodiscard]] std::vector<rectangle> run() {
            if (!std::isfinite(long_side_) || !std::isfinite(short_side_)
                || long_side_ <= 0.0 || short_side_ <= 0.0) {
                return {};
            }

            place_frame(container_.width, container_.height, 0.0, 0.0, 0);
            if (!valid_) {
                return {};
            }
            return std::move(rectangles_);
        }

    private:
        [[nodiscard]] bool append(rectangle value) {
            if (!is_finite(value) || value.width <= 0.0 || value.height <= 0.0
                || rectangles_.size() == rectangles_.max_size()) {
                valid_ = false;
                return false;
            }
            value.source_index = rectangles_.size();
            rectangles_.push_back(value);
            return true;
        }

        [[nodiscard]] double
        fill_horizontal(double offset, double width, double x, double y) {
            double x_offset = offset;
            while (valid_) {
                const double next = x_offset + long_side_;
                if (!std::isfinite(next) || next > width) {
                    break;
                }
                if (!append(
                        {
                            x + x_offset,
                            y,
                            long_side_,
                            short_side_,
                            0,
                            item_.horizontal_rotated,
                        }
                    )) {
                    break;
                }
                if (next <= x_offset) {
                    valid_ = false;
                    break;
                }
                x_offset = next;
            }
            return x_offset;
        }

        [[nodiscard]] double
        fill_vertical(double offset, double height, double x, double y) {
            double y_offset = offset;
            while (valid_) {
                const double next = y_offset + long_side_;
                if (!std::isfinite(next) || next > height) {
                    break;
                }
                if (!append(
                        {
                            x,
                            y + y_offset,
                            short_side_,
                            long_side_,
                            0,
                            item_.vertical_rotated,
                        }
                    )) {
                    break;
                }
                if (next <= y_offset) {
                    valid_ = false;
                    break;
                }
                y_offset = next;
            }
            return y_offset;
        }

        void place_frame(
            double width, double height, double x, double y, std::size_t depth
        ) {
            if (!valid_) {
                return;
            }
            if (depth >= maximum_frame_recursion_depth || !std::isfinite(width)
                || !std::isfinite(height) || !std::isfinite(x)
                || !std::isfinite(y)) {
                valid_ = false;
                return;
            }

            const bool fits_horizontal
                = width >= long_side_ && height >= short_side_;
            const bool fits_vertical
                = width >= short_side_ && height >= long_side_;
            if (!fits_horizontal && !fits_vertical) {
                return;
            }

            const double rows = std::floor(height / long_side_);
            const double columns = std::floor(width / long_side_);
            const double bottom_remainder = height - rows * long_side_;
            const double right_remainder = width - columns * long_side_;

            if (bottom_remainder >= short_side_) {
                const std::size_t first_index = rectangles_.size();
                const double x_offset = fill_horizontal(0.0, width, x, y);
                const double y_offset
                    = fill_vertical(short_side_, height, x, y);
                if (!valid_) {
                    return;
                }

                // The opposite strips form a distinct four-sided frame only
                // when both first strips contain at least one rectangle.
                if (x_offset > 0.0 && y_offset > short_side_
                    && x_offset + short_side_ <= width) {
                    static_cast<void>(fill_horizontal(
                        short_side_, width, x, y + y_offset - short_side_
                    ));
                    static_cast<void>(
                        fill_vertical(0.0, height, x + x_offset, y)
                    );
                    if (!valid_) {
                        return;
                    }

                    // Recurse into the actual frame interior rather than the
                    // nominal outer rectangle. This is the alg8 geometry
                    // repair.
                    place_frame(
                        x_offset - short_side_, y_offset - 2.0 * short_side_,
                        x + short_side_, y + short_side_, depth + 1
                    );
                    if (!valid_) {
                        return;
                    }

                    const double x_shift
                        = (width - x_offset - short_side_) / 2.0;
                    const double y_shift = (height - y_offset) / 2.0;
                    for (std::size_t index = first_index;
                         index < rectangles_.size(); ++index) {
                        rectangles_[index].x += x_shift;
                        rectangles_[index].y += y_shift;
                        if (!is_finite(rectangles_[index])) {
                            valid_ = false;
                            return;
                        }
                    }
                } else {
                    place_frame(
                        width - short_side_, height - short_side_,
                        x + short_side_, y + short_side_, depth + 1
                    );
                }
            } else if (right_remainder >= short_side_) {
                static_cast<void>(fill_vertical(0.0, height, x, y));
                static_cast<void>(fill_horizontal(short_side_, width, x, y));
                if (!valid_) {
                    return;
                }
                place_frame(
                    width - short_side_, height - short_side_, x + short_side_,
                    y + short_side_, depth + 1
                );
            } else if (right_remainder * height < bottom_remainder * width) {
                static_cast<void>(
                    fill_horizontal(right_remainder / 2.0, width, x, y)
                );
                if (!valid_) {
                    return;
                }
                place_frame(
                    width, height - short_side_, x, y + short_side_, depth + 1
                );
            } else {
                static_cast<void>(
                    fill_vertical(bottom_remainder / 2.0, height, x, y)
                );
                if (!valid_) {
                    return;
                }
                place_frame(
                    width - short_side_, height, x + short_side_, y, depth + 1
                );
            }
        }

        extent container_;
        canonical_item item_;
        double long_side_ { 0.0 };
        double short_side_ { 0.0 };
        std::vector<rectangle> rectangles_;
        bool valid_ { true };
    };

    class frame_counter {
    public:
        frame_counter(
            extent container, canonical_item item, double scale, std::size_t cap
        )
            : container_(container)
            , long_side_(item.long_side * scale)
            , short_side_(item.short_side * scale)
            , cap_(cap) { }

        [[nodiscard]] std::size_t run() {
            if (!std::isfinite(long_side_) || !std::isfinite(short_side_)
                || long_side_ <= 0.0 || short_side_ <= 0.0 || cap_ == 0) {
                return 0;
            }
            count_frame(container_.width, container_.height, 0);
            return valid_ ? count_ : 0;
        }

    private:
        [[nodiscard]] bool complete() const noexcept { return count_ >= cap_; }

        [[nodiscard]] double fill_count(double offset, double span) {
            double value = offset;
            while (!complete() && valid_) {
                const double next = value + long_side_;
                if (!std::isfinite(next) || next > span) {
                    break;
                }
                ++count_;
                if (next <= value) {
                    valid_ = false;
                    break;
                }
                value = next;
            }
            return value;
        }

        void count_frame(double width, double height, std::size_t depth) {
            if (complete() || !valid_) {
                return;
            }
            if (depth >= maximum_frame_recursion_depth || !std::isfinite(width)
                || !std::isfinite(height)) {
                valid_ = false;
                return;
            }

            const bool fits_horizontal
                = width >= long_side_ && height >= short_side_;
            const bool fits_vertical
                = width >= short_side_ && height >= long_side_;
            if (!fits_horizontal && !fits_vertical) {
                return;
            }

            const double rows = std::floor(height / long_side_);
            const double columns = std::floor(width / long_side_);
            const double bottom_remainder = height - rows * long_side_;
            const double right_remainder = width - columns * long_side_;

            if (bottom_remainder >= short_side_) {
                const double x_offset = fill_count(0.0, width);
                if (complete() || !valid_) {
                    return;
                }
                const double y_offset = fill_count(short_side_, height);
                if (complete() || !valid_) {
                    return;
                }

                if (x_offset > 0.0 && y_offset > short_side_
                    && x_offset + short_side_ <= width) {
                    static_cast<void>(fill_count(short_side_, width));
                    if (complete() || !valid_) {
                        return;
                    }
                    static_cast<void>(fill_count(0.0, height));
                    if (complete() || !valid_) {
                        return;
                    }
                    count_frame(
                        x_offset - short_side_, y_offset - 2.0 * short_side_,
                        depth + 1
                    );
                } else {
                    count_frame(
                        width - short_side_, height - short_side_, depth + 1
                    );
                }
            } else if (right_remainder >= short_side_) {
                static_cast<void>(fill_count(0.0, height));
                if (complete() || !valid_) {
                    return;
                }
                static_cast<void>(fill_count(short_side_, width));
                if (complete() || !valid_) {
                    return;
                }
                count_frame(
                    width - short_side_, height - short_side_, depth + 1
                );
            } else if (right_remainder * height < bottom_remainder * width) {
                static_cast<void>(fill_count(right_remainder / 2.0, width));
                if (complete() || !valid_) {
                    return;
                }
                count_frame(width, height - short_side_, depth + 1);
            } else {
                static_cast<void>(fill_count(bottom_remainder / 2.0, height));
                if (complete() || !valid_) {
                    return;
                }
                count_frame(width - short_side_, height, depth + 1);
            }
        }

        extent container_;
        double long_side_ { 0.0 };
        double short_side_ { 0.0 };
        std::size_t cap_ { 0 };
        std::size_t count_ { 0 };
        bool valid_ { true };
    };

    [[nodiscard]] std::size_t count_frame_candidates(
        const equal_packing_request& request, canonical_item item, double scale
    ) {
        return frame_counter(request.container, item, scale, request.count)
            .run();
    }

    [[nodiscard]] std::vector<rectangle> generate_frame_candidates(
        const equal_packing_request& request, canonical_item item, double scale
    ) {
        return frame_generator(request.container, item, scale, request.count)
            .run();
    }

    [[nodiscard]] bool finite_positive(double value) noexcept {
        return std::isfinite(value) && value > 0.0;
    }

    [[nodiscard]] std::vector<rectangle> select_and_center(
        extent container, std::vector<rectangle> candidates, std::size_t count
    ) {
        if (candidates.size() < count || count == 0) {
            return {};
        }

        std::vector<std::size_t> indices(candidates.size());
        std::iota(indices.begin(), indices.end(), std::size_t { 0 });
        const long double center_x
            = static_cast<long double>(container.width) / 2.0L;
        const long double center_y
            = static_cast<long double>(container.height) / 2.0L;

        const auto key = [&](std::size_t index) {
            const rectangle& value = candidates[index];
            const long double rectangle_center_x
                = static_cast<long double>(value.x)
                + static_cast<long double>(value.width) / 2.0L;
            const long double rectangle_center_y
                = static_cast<long double>(value.y)
                + static_cast<long double>(value.height) / 2.0L;
            const long double dx = rectangle_center_x - center_x;
            const long double dy = rectangle_center_y - center_y;
            return std::tuple {
                dx * dx + dy * dy, value.y, value.x, value.rotated, index,
            };
        };

        std::partial_sort(
            indices.begin(),
            indices.begin() + static_cast<std::ptrdiff_t>(count), indices.end(),
            [&](std::size_t lhs, std::size_t rhs) {
                return key(lhs) < key(rhs);
            }
        );

        std::vector<rectangle> selected;
        selected.reserve(count);
        for (std::size_t output_index = 0; output_index < count;
             ++output_index) {
            rectangle value = candidates[indices[output_index]];
            value.source_index = output_index;
            selected.push_back(value);
        }

        double left = selected.front().x;
        double top = selected.front().y;
        double right = selected.front().x + selected.front().width;
        double bottom = selected.front().y + selected.front().height;
        for (const rectangle& value : selected) {
            left = std::min(left, value.x);
            top = std::min(top, value.y);
            right = std::max(right, value.x + value.width);
            bottom = std::max(bottom, value.y + value.height);
        }

        const double minimum_dx = -left;
        const double maximum_dx = container.width - right;
        const double minimum_dy = -top;
        const double maximum_dy = container.height - bottom;
        if (minimum_dx > maximum_dx || minimum_dy > maximum_dy) {
            return {};
        }

        const double desired_dx = container.width / 2.0 - (left + right) / 2.0;
        const double desired_dy = container.height / 2.0 - (top + bottom) / 2.0;
        const double dx = std::clamp(desired_dx, minimum_dx, maximum_dx);
        const double dy = std::clamp(desired_dy, minimum_dy, maximum_dy);
        for (rectangle& value : selected) {
            value.x += dx;
            value.y += dy;
            if (!is_finite(value)) {
                return {};
            }
        }
        return selected;
    }

    [[nodiscard]] equal_packing_result make_frame_result(
        const equal_packing_request& request, canonical_item item, double scale,
        equal_packing_algorithm algorithm
    ) {
        if (!finite_positive(scale)) {
            return { 0.0, {}, algorithm };
        }

        std::vector<rectangle> candidates
            = generate_frame_candidates(request, item, scale);
        if (candidates.size() < request.count
            || !validate(request.container, candidates).valid) {
            return { 0.0, {}, algorithm };
        }

        std::vector<rectangle> selected = select_and_center(
            request.container, std::move(candidates), request.count
        );
        if (selected.size() != request.count
            || !validate(request.container, selected).valid) {
            return { 0.0, {}, algorithm };
        }
        return { scale, std::move(selected), algorithm };
    }

    [[nodiscard]] equal_packing_result pack_certified_frame(
        const equal_packing_request& request, canonical_item item
    ) {
        constexpr equal_packing_algorithm algorithm
            = equal_packing_algorithm::certified_frame;
        const double upper = area_scale_upper_bound(request);
        if (!finite_positive(upper)) {
            return { 0.0, {}, algorithm };
        }

        const long double scale_step = std::max(
            static_cast<long double>(request.long_side_step)
                / static_cast<long double>(item.long_side),
            1.0e-9L
        );
        const long double step_count
            = std::floor(static_cast<long double>(upper) / scale_step);
        if (!std::isfinite(step_count) || step_count < 1.0L
            || step_count
                > static_cast<long double>(maximum_certified_scan_steps)) {
            return { 0.0, {}, algorithm };
        }

        for (auto step = static_cast<std::uint64_t>(step_count); step > 0;
             --step) {
            const long double candidate_scale
                = static_cast<long double>(step) * scale_step;
            if (!std::isfinite(candidate_scale)
                || candidate_scale > static_cast<long double>(
                       std::numeric_limits<double>::max()
                   )) {
                continue;
            }
            const auto scale = static_cast<double>(candidate_scale);
            if (count_frame_candidates(request, item, scale) < request.count) {
                continue;
            }

            equal_packing_result result
                = make_frame_result(request, item, scale, algorithm);
            if (result.complete(request.count)) {
                return result;
            }
        }
        return { 0.0, {}, algorithm };
    }

    [[nodiscard]] equal_packing_result pack_count_binary_frame(
        const equal_packing_request& request, canonical_item item
    ) {
        constexpr equal_packing_algorithm algorithm
            = equal_packing_algorithm::count_binary_frame;
        const double upper = area_scale_upper_bound(request);
        if (!finite_positive(upper)) {
            return { 0.0, {}, algorithm };
        }

        const double tolerance
            = std::max(request.long_side_step / item.long_side, 1.0e-9);
        double left = 0.0;
        double right = upper;
        for (std::size_t iteration = 0;
             iteration < 256 && right - left > tolerance; ++iteration) {
            const double scale = left + (right - left) / 2.0;
            if (count_frame_candidates(request, item, scale) >= request.count) {
                left = scale;
            } else {
                right = scale;
            }
        }
        return make_frame_result(request, item, left, algorithm);
    }

    [[nodiscard]] equal_packing_result pack_grid(
        const equal_packing_request& request, canonical_item item,
        bool vertical, equal_packing_algorithm algorithm
    ) {
        const long double item_width = vertical
            ? static_cast<long double>(item.short_side)
            : static_cast<long double>(item.long_side);
        const long double item_height = vertical
            ? static_cast<long double>(item.long_side)
            : static_cast<long double>(item.short_side);

        long double best_scale = 0.0L;
        std::size_t best_columns = 0;
        std::size_t best_rows = 0;
        std::size_t best_unused = std::numeric_limits<std::size_t>::max();

        for (std::size_t columns = 1;; ++columns) {
            const std::size_t rows = request.count / columns
                + static_cast<std::size_t>(request.count % columns != 0);
            const long double horizontal_scale
                = static_cast<long double>(request.container.width)
                / (static_cast<long double>(columns) * item_width);
            const long double vertical_scale
                = static_cast<long double>(request.container.height)
                / (static_cast<long double>(rows) * item_height);
            const long double scale
                = std::min(horizontal_scale, vertical_scale);
            const std::size_t unused
                = columns <= std::numeric_limits<std::size_t>::max() / rows
                ? columns * rows - request.count
                : std::numeric_limits<std::size_t>::max();
            if (std::isfinite(scale) && scale > 0.0L
                && (scale > best_scale
                    || (scale == best_scale && unused < best_unused))) {
                best_scale = scale;
                best_columns = columns;
                best_rows = rows;
                best_unused = unused;
            }
            if (columns == request.count) {
                break;
            }
        }

        if (best_columns == 0 || best_rows == 0 || !std::isfinite(best_scale)
            || best_scale <= 0.0L
            || best_scale > static_cast<long double>(
                   std::numeric_limits<double>::max()
               )) {
            return { 0.0, {}, algorithm };
        }

        const auto scale = static_cast<double>(best_scale);
        const auto rectangle_width
            = static_cast<double>(item_width * best_scale);
        const auto rectangle_height
            = static_cast<double>(item_height * best_scale);
        if (!finite_positive(scale) || !finite_positive(rectangle_width)
            || !finite_positive(rectangle_height)) {
            return { 0.0, {}, algorithm };
        }

        std::vector<rectangle> rectangles;
        rectangles.reserve(request.count);
        const double block_height
            = static_cast<double>(best_rows) * rectangle_height;
        const double y_origin = (request.container.height - block_height) / 2.0;
        for (std::size_t index = 0; index < request.count; ++index) {
            const std::size_t row = index / best_columns;
            const std::size_t column = index % best_columns;
            const std::size_t first_in_row = row * best_columns;
            const std::size_t row_count
                = std::min(best_columns, request.count - first_in_row);
            const double row_width
                = static_cast<double>(row_count) * rectangle_width;
            const double x_origin = (request.container.width - row_width) / 2.0;
            rectangles.push_back(
                {
                    x_origin + static_cast<double>(column) * rectangle_width,
                    y_origin + static_cast<double>(row) * rectangle_height,
                    rectangle_width,
                    rectangle_height,
                    index,
                    vertical ? item.vertical_rotated : item.horizontal_rotated,
                }
            );
        }

        if (rectangles.size() != request.count
            || !validate(request.container, rectangles).valid) {
            return { 0.0, {}, algorithm };
        }
        return { scale, std::move(rectangles), algorithm };
    }

} // namespace

equal_packing_algorithm
select_equal_packing_algorithm(const equal_packing_request& request) noexcept {
    if (request.algorithm != equal_packing_algorithm::automatic) {
        return request.algorithm;
    }
    switch (request.orientation) {
    case orientation_constraint::horizontal_only:
        return equal_packing_algorithm::horizontal_grid;
    case orientation_constraint::vertical_only:
        return equal_packing_algorithm::vertical_grid;
    case orientation_constraint::allow_rotation:
        return equal_packing_algorithm::certified_frame;
    }
    return equal_packing_algorithm::certified_frame;
}

equal_packing_result
pack_equal_rectangles(const equal_packing_request& request) {
    const equal_packing_algorithm algorithm
        = select_equal_packing_algorithm(request);
    if (!valid_request(request)
        || !compatible(request.orientation, algorithm)) {
        return { 0.0, {}, algorithm };
    }

    const canonical_item item = canonicalize(request.item);
    switch (algorithm) {
    case equal_packing_algorithm::certified_frame:
        return pack_certified_frame(request, item);
    case equal_packing_algorithm::count_binary_frame:
        return pack_count_binary_frame(request, item);
    case equal_packing_algorithm::horizontal_grid:
        return pack_grid(request, item, false, algorithm);
    case equal_packing_algorithm::vertical_grid:
        return pack_grid(request, item, true, algorithm);
    case equal_packing_algorithm::automatic:
        break;
    }
    return { 0.0, {}, algorithm };
}

std::string_view algorithm_name(equal_packing_algorithm algorithm) noexcept {
    switch (algorithm) {
    case equal_packing_algorithm::automatic:
        return "automatic";
    case equal_packing_algorithm::certified_frame:
        return "certified_frame";
    case equal_packing_algorithm::count_binary_frame:
        return "count_binary_frame";
    case equal_packing_algorithm::horizontal_grid:
        return "horizontal_grid";
    case equal_packing_algorithm::vertical_grid:
        return "vertical_grid";
    }
    return "unknown";
}

} // namespace packing
