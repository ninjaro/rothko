#include "packing/layout/equal_rectangles.hpp"
#include "packing/layout/fixed_size_layout.hpp"
#include "packing/layout/free_order_layout.hpp"
#include "packing/layout/ordered_layout.hpp"
#include "packing/layout/reciprocal_layout.hpp"
#include "packing/selection/compact_layout.hpp"
#include "packing/selection/spread_layout.hpp"

#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace packing {
namespace {

    struct equal_case {
        extent container;
        extent item;
        std::size_t count;
    };

    // Ported from card_packer_lab_v3/cases.py and the strongest finite stress
    // findings. These deliberately mix historical correctness regressions with
    // M >> N layouts where selection quality is visible.
    constexpr std::array equal_lab_cases {
        equal_case { { 800.0, 600.0 }, { 88.0, 63.0 }, 24 },
        equal_case { { 394.0, 465.0 }, { 76.0, 35.0 }, 66 },
        equal_case { { 271.0, 1145.0 }, { 95.0, 39.0 }, 15 },
        equal_case { { 840.0, 849.0 }, { 83.0, 68.0 }, 51 },
        equal_case { { 1039.0, 640.0 }, { 73.0, 39.0 }, 19 },
        equal_case { { 1337.0, 252.0 }, { 101.0, 72.0 }, 8 },
        equal_case { { 1334.0, 284.0 }, { 88.0, 60.0 }, 7 },
        equal_case { { 905.0, 330.0 }, { 75.0, 71.0 }, 6 },
        equal_case { { 1377.0, 320.0 }, { 108.0, 48.0 }, 23 },
        equal_case { { 1395.0, 400.0 }, { 109.0, 70.0 }, 45 },
        equal_case { { 264.0, 970.0 }, { 109.0, 101.0 }, 8 },
        equal_case { { 876.0, 252.0 }, { 87.0, 83.0 }, 7 },
        equal_case { { 1218.0, 317.0 }, { 96.0, 39.0 }, 32 },
        equal_case { { 382.0, 280.0 }, { 84.0, 77.0 }, 36 },
        equal_case { { 632.0, 848.0 }, { 66.0, 62.0 }, 36 },
    };

    struct void_quality {
        double rms { 0.0 };
        double maximum { 0.0 };
    };

    struct measured_quality {
        double bbox_fill { 0.0 };
        double bbox_container { 0.0 };
        void_quality compact;
        void_quality field;
    };

    [[nodiscard]] double
    rectangle_area(const std::span<const rectangle> rectangles) noexcept {
        double result = 0.0;
        for (const rectangle& item : rectangles) {
            result += item.width * item.height;
        }
        return result;
    }

    [[nodiscard]] std::vector<double> axis_lines(
        const double lower, const double upper, const double step,
        const std::span<const rectangle> rectangles, const bool horizontal
    ) {
        const double span = upper - lower;
        const std::size_t regular_count = std::max<std::size_t>(
            1, static_cast<std::size_t>(std::ceil(span / step))
        );
        std::vector<double> result;
        result.reserve(regular_count + rectangles.size() * 2 + 1);
        for (std::size_t index = 0; index <= regular_count; ++index) {
            result.push_back(
                lower
                + span * static_cast<double>(index)
                    / static_cast<double>(regular_count)
            );
        }
        for (const rectangle& item : rectangles) {
            const double first = horizontal ? item.x : item.y;
            const double second
                = horizontal ? item.x + item.width : item.y + item.height;
            if (first > lower && first < upper) {
                result.push_back(first);
            }
            if (second > lower && second < upper) {
                result.push_back(second);
            }
        }
        std::ranges::sort(result);
        const double epsilon = std::max(1.0e-12, span * 1.0e-12);
        result.erase(
            std::unique(
                result.begin(), result.end(),
                [epsilon](const double lhs, const double rhs) {
                    return std::abs(lhs - rhs) <= epsilon;
                }
            ),
            result.end()
        );
        return result;
    }

    [[nodiscard]] void_quality measure_void(
        const bounds domain, const std::span<const rectangle> rectangles,
        const double normalization
    ) {
        if (rectangles.empty() || domain.width() <= 0.0
            || domain.height() <= 0.0 || normalization <= 0.0) {
            return {};
        }
        const double step = std::max(normalization * 0.25, 1.0e-9);
        const std::vector<double> xs
            = axis_lines(domain.left, domain.right, step, rectangles, true);
        const std::vector<double> ys
            = axis_lines(domain.top, domain.bottom, step, rectangles, false);

        long double weighted_total = 0.0L;
        long double total_weight = 0.0L;
        double maximum = 0.0;
        for (std::size_t row = 1; row < ys.size(); ++row) {
            for (std::size_t column = 1; column < xs.size(); ++column) {
                const double width = xs[column] - xs[column - 1];
                const double height = ys[row] - ys[row - 1];
                if (width <= 0.0 || height <= 0.0) {
                    continue;
                }
                const point sample {
                    (xs[column] + xs[column - 1]) / 2.0,
                    (ys[row] + ys[row - 1]) / 2.0,
                };
                double nearest = std::numeric_limits<double>::infinity();
                for (const rectangle& item : rectangles) {
                    nearest = std::min(nearest, squared_distance(sample, item));
                    if (nearest <= 0.0) {
                        break;
                    }
                }
                if (nearest <= 0.0 || !std::isfinite(nearest)) {
                    continue;
                }
                const long double weight = static_cast<long double>(width)
                    * static_cast<long double>(height);
                weighted_total += static_cast<long double>(nearest) * weight;
                total_weight += weight;
                maximum = std::max(maximum, nearest);
            }
        }
        if (total_weight <= 0.0L) {
            return {};
        }
        return {
            std::sqrt(static_cast<double>(weighted_total / total_weight))
                / normalization,
            std::sqrt(maximum) / normalization,
        };
    }

    [[nodiscard]] measured_quality measure_quality(
        const extent container, const std::span<const rectangle> rectangles
    ) {
        if (rectangles.empty()) {
            return {};
        }
        const bounds box = bounding_box(rectangles);
        const double box_area = box.area();
        const double item_area = rectangle_area(rectangles);
        double shortest_side = std::numeric_limits<double>::infinity();
        for (const rectangle& item : rectangles) {
            shortest_side
                = std::min(shortest_side, std::min(item.width, item.height));
        }
        return {
            .bbox_fill = box_area > 0.0 ? item_area / box_area : 0.0,
            .bbox_container = container.width * container.height > 0.0
                ? box_area / (container.width * container.height)
                : 0.0,
            .compact = measure_void(box, rectangles, shortest_side),
            .field = measure_void(
                { 0.0, 0.0, container.width, container.height }, rectangles,
                shortest_side
            ),
        };
    }

    void publish_quality(
        benchmark::State& state, const extent container,
        const std::span<const rectangle> rectangles
    ) {
        const measured_quality quality = measure_quality(container, rectangles);
        state.counters["bbox_fill"] = quality.bbox_fill;
        state.counters["bbox_field"] = quality.bbox_container;
        state.counters["compact_rms"] = quality.compact.rms;
        state.counters["compact_max"] = quality.compact.maximum;
        state.counters["field_rms"] = quality.field.rms;
        state.counters["field_max"] = quality.field.maximum;
    }

    std::vector<ordered_slot> make_slots(const std::size_t count) {
        constexpr std::array aspects {
            16.0 / 9.0, 4.0 / 3.0, 1.0, 9.0 / 16.0, 3.0 / 4.0,
        };
        std::vector<ordered_slot> result;
        result.reserve(count);
        for (std::size_t index = 0; index < count; ++index) {
            result.push_back(
                {
                    .aspect = aspects[index % aspects.size()],
                    .rotated = index % 7 == 0,
                }
            );
        }
        return result;
    }

    std::vector<fixed_size_item> make_fixed_items(const std::size_t count) {
        constexpr std::array sizes {
            extent { 160.0, 90.0 },  extent { 120.0, 160.0 },
            extent { 144.0, 108.0 }, extent { 96.0, 96.0 },
            extent { 180.0, 76.0 },  extent { 88.0, 132.0 },
        };
        std::vector<fixed_size_item> result;
        result.reserve(count);
        for (std::size_t index = 0; index < count; ++index) {
            result.push_back({ .size = sizes[index % sizes.size()] });
        }
        return result;
    }

    std::vector<rectangle> make_candidates() {
        std::vector<rectangle> result;
        result.reserve(100);
        for (std::size_t row = 0; row < 10; ++row) {
            for (std::size_t column = 0; column < 10; ++column) {
                result.push_back(
                    {
                        .x = static_cast<double>(column) * 96.0,
                        .y = static_cast<double>(row) * 76.0,
                        .width = 88.0,
                        .height = 63.0,
                        .source_index = result.size(),
                    }
                );
            }
        }
        return result;
    }

    struct ordered_case {
        extent viewport;
        std::vector<ordered_slot> items;
        double minimum_short_side { 160.0 };
        double spacing { 6.0 };
    };

    [[nodiscard]] ordered_case make_ordered_lab_case(const std::size_t index) {
        constexpr ordered_slot landscape { 16.0 / 9.0, false };
        constexpr ordered_slot portrait { 9.0 / 16.0, false };
        constexpr ordered_slot four_three { 4.0 / 3.0, false };
        constexpr ordered_slot square { 1.0, false };
        switch (index) {
        case 0:
            return { { 1280.0, 720.0 },
                     std::vector<ordered_slot>(6, landscape) };
        case 1:
            return {
                { 1280.0, 720.0 },
                { landscape, landscape, portrait, landscape, portrait,
                  four_three, landscape, square },
            };
        case 2:
            return {
                { 1024.0, 700.0 },
                { portrait, portrait, landscape, portrait, portrait, four_three,
                  landscape },
                150.0,
            };
        case 3:
            return {
                { 1366.0, 768.0 },
                { landscape, portrait, landscape, four_three, landscape, square,
                  portrait, landscape, portrait, four_three, landscape,
                  landscape },
                150.0,
            };
        case 4:
            return {
                { 900.0, 650.0 },
                { landscape, landscape, portrait, landscape, portrait,
                  landscape, four_three, landscape },
                165.0,
            };
        case 5:
            return {
                { 1200.0, 700.0 },
                { landscape,
                  landscape,
                  { 16.0 / 9.0, true },
                  landscape,
                  four_three,
                  portrait },
            };
        default:
            return {};
        }
    }

    template <typename Result>
    void publish_selection_result(
        benchmark::State& state, const extent container,
        const std::size_t count, const Result& result
    ) {
        state.counters["valid"] = result.rectangles.size() == count
                && validate(container, result.rectangles).valid
            ? 1.0
            : 0.0;
        publish_quality(state, container, result.rectangles);
    }

    void benchmark_equal(benchmark::State& state) {
        const equal_packing_algorithm algorithm
            = static_cast<equal_packing_algorithm>(state.range(1));
        const equal_packing_request request {
            .container = { 1920.0, 1080.0 },
            .item = { 88.0, 63.0 },
            .count = static_cast<std::size_t>(state.range(0)),
            .algorithm = algorithm,
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(pack_equal_rectangles(request));
        }
        const equal_packing_result result = pack_equal_rectangles(request);
        const equal_candidate_result candidates
            = generate_equal_rectangle_candidates(request);
        state.counters["scale"] = result.scale;
        state.counters["candidates"]
            = static_cast<double>(candidates.rectangles.size());
        state.counters["valid"] = result.complete(request.count)
                && validate(request.container, result.rectangles).valid
            ? 1.0
            : 0.0;
    }

    void benchmark_equal_lab_case(benchmark::State& state) {
        const equal_case& value
            = equal_lab_cases[static_cast<std::size_t>(state.range(0))];
        const equal_packing_request request {
            .container = value.container,
            .item = value.item,
            .count = value.count,
            .algorithm = static_cast<equal_packing_algorithm>(state.range(1)),
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(pack_equal_rectangles(request));
        }
        const equal_packing_result result = pack_equal_rectangles(request);
        const equal_candidate_result candidates
            = generate_equal_rectangle_candidates(request);
        state.counters["N"] = static_cast<double>(value.count);
        state.counters["scale"] = result.scale;
        state.counters["candidates"]
            = static_cast<double>(candidates.rectangles.size());
        state.counters["valid"] = result.complete(value.count)
                && validate(value.container, result.rectangles).valid
            ? 1.0
            : 0.0;
    }

    void benchmark_ordered(benchmark::State& state) {
        const std::vector slots
            = make_slots(static_cast<std::size_t>(state.range(0)));
        const ordered_layout_algorithm algorithm
            = static_cast<ordered_layout_algorithm>(state.range(1));
        const ordered_layout_request request {
            .viewport = { 1280.0, 720.0 },
            .items = slots,
            .policy = scroll_policy::horizontal,
            .minimum_short_side = 160.0,
            .spacing = 6.0,
            .algorithm = algorithm,
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(layout_ordered_slots(request));
        }
        const ordered_layout_result result = layout_ordered_slots(request);
        state.counters["short_side"] = result.common_short_side;
        state.counters["primary"] = result.primary_extent;
        state.counters["raggedness"] = result.raggedness;
        state.counters["scroll"] = result.used_scroll ? 1.0 : 0.0;
        const double content_area
            = result.content.width * result.content.height;
        state.counters["area_eff"] = content_area > 0.0
            ? rectangle_area(result.rectangles) / content_area
            : 0.0;
    }

    void benchmark_ordered_lab_case(benchmark::State& state) {
        const ordered_case value
            = make_ordered_lab_case(static_cast<std::size_t>(state.range(0)));
        const ordered_layout_request request {
            .viewport = value.viewport,
            .items = value.items,
            .policy = scroll_policy::horizontal,
            .minimum_short_side = value.minimum_short_side,
            .spacing = value.spacing,
            .algorithm = static_cast<ordered_layout_algorithm>(state.range(1)),
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(layout_ordered_slots(request));
        }
        const ordered_layout_result result = layout_ordered_slots(request);
        state.counters["N"] = static_cast<double>(value.items.size());
        state.counters["short_side"] = result.common_short_side;
        state.counters["primary"] = result.primary_extent;
        state.counters["raggedness"] = result.raggedness;
        state.counters["scroll"] = result.used_scroll ? 1.0 : 0.0;
        const double content_area
            = result.content.width * result.content.height;
        state.counters["area_eff"] = content_area > 0.0
            ? rectangle_area(result.rectangles) / content_area
            : 0.0;
    }

    void publish_free_order_result(
        benchmark::State& state, const free_order_layout_request& request,
        const free_order_layout_result& result
    ) {
        state.counters["valid"] = result.complete(request.items.size())
                && validate(result.content, result.rectangles).valid
            ? 1.0
            : 0.0;
        state.counters["short_side"] = result.common_short_side;
        state.counters["primary"] = result.primary_extent;
        state.counters["raggedness"] = result.raggedness;
        state.counters["scroll"] = result.used_scroll ? 1.0 : 0.0;
        const double content_area
            = result.content.width * result.content.height;
        state.counters["area_eff"] = content_area > 0.0
            ? rectangle_area(result.rectangles) / content_area
            : 0.0;
    }

    void benchmark_free_order(benchmark::State& state) {
        const std::vector slots
            = make_slots(static_cast<std::size_t>(state.range(0)));
        const free_order_layout_request request {
            .viewport = { 1280.0, 720.0 },
            .items = slots,
            .policy = scroll_policy::horizontal,
            .minimum_short_side = 160.0,
            .spacing = 6.0,
            .algorithm
            = static_cast<free_order_layout_algorithm>(state.range(1)),
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(layout_free_order_slots(request));
        }
        publish_free_order_result(
            state, request, layout_free_order_slots(request)
        );
    }

    void benchmark_free_order_lab_case(benchmark::State& state) {
        const ordered_case value
            = make_ordered_lab_case(static_cast<std::size_t>(state.range(0)));
        const free_order_layout_request request {
            .viewport = value.viewport,
            .items = value.items,
            .policy = scroll_policy::horizontal,
            .minimum_short_side = value.minimum_short_side,
            .spacing = value.spacing,
            .algorithm
            = static_cast<free_order_layout_algorithm>(state.range(1)),
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(layout_free_order_slots(request));
        }
        state.counters["N"] = static_cast<double>(value.items.size());
        publish_free_order_result(
            state, request, layout_free_order_slots(request)
        );
    }

    void benchmark_fixed_size(benchmark::State& state) {
        const std::vector items
            = make_fixed_items(static_cast<std::size_t>(state.range(0)));
        const fixed_size_layout_request request {
            .viewport = { 1280.0, 720.0 },
            .items = items,
            .policy = scroll_policy::horizontal,
            .spacing = 6.0,
            .algorithm
            = static_cast<fixed_size_layout_algorithm>(state.range(1)),
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(layout_fixed_size_rectangles(request));
        }
        const fixed_size_layout_result result
            = layout_fixed_size_rectangles(request);
        state.counters["valid"] = result.complete(items.size())
                && validate(result.content, result.rectangles).valid
            ? 1.0
            : 0.0;
        state.counters["primary"] = result.primary_extent;
        state.counters["groups"] = static_cast<double>(result.groups.size());
        state.counters["scroll"] = result.used_scroll ? 1.0 : 0.0;
        const double content_area
            = result.content.width * result.content.height;
        state.counters["area_eff"] = content_area > 0.0
            ? rectangle_area(result.rectangles) / content_area
            : 0.0;
    }

    void benchmark_compact(benchmark::State& state) {
        const std::vector candidates = make_candidates();
        const compact_layout_algorithm algorithm
            = static_cast<compact_layout_algorithm>(state.range(1));
        const compact_layout_request request {
            .container = { 960.0, 760.0 },
            .candidates = candidates,
            .count = static_cast<std::size_t>(state.range(0)),
            .algorithm = algorithm,
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(select_compact_layout(request));
        }
        const compact_layout_result result = select_compact_layout(request);
        state.counters["valid"] = result.rectangles.size() == request.count
                && validate(request.container, result.rectangles).valid
            ? 1.0
            : 0.0;
        publish_quality(state, request.container, result.rectangles);
    }

    void benchmark_compact_surplus(benchmark::State& state) {
        const std::vector all_candidates = make_candidates();
        const std::size_t count = static_cast<std::size_t>(state.range(0));
        const std::size_t candidate_count
            = count + static_cast<std::size_t>(state.range(1));
        const std::span candidates(all_candidates.data(), candidate_count);
        const compact_layout_request request {
            .container = { 960.0, 760.0 },
            .candidates = candidates,
            .count = count,
            .algorithm = static_cast<compact_layout_algorithm>(state.range(2)),
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(select_compact_layout(request));
        }
        const compact_layout_result result = select_compact_layout(request);
        publish_selection_result(state, request.container, count, result);
    }

    void benchmark_compact_lab_case(benchmark::State& state) {
        const equal_case& value
            = equal_lab_cases[static_cast<std::size_t>(state.range(0))];
        const equal_candidate_result candidates
            = generate_equal_rectangle_candidates(
                {
                    .container = value.container,
                    .item = value.item,
                    .count = value.count,
                    .algorithm = equal_packing_algorithm::certified_frame,
                }
            );
        const compact_layout_request request {
            .container = value.container,
            .candidates = candidates.rectangles,
            .count = value.count,
            .algorithm = static_cast<compact_layout_algorithm>(state.range(1)),
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(select_compact_layout(request));
        }
        const compact_layout_result result = select_compact_layout(request);
        state.counters["N"] = static_cast<double>(value.count);
        state.counters["M"] = static_cast<double>(candidates.rectangles.size());
        publish_selection_result(state, value.container, value.count, result);
    }

    void benchmark_spread(benchmark::State& state) {
        const std::vector candidates = make_candidates();
        const spread_layout_algorithm algorithm
            = static_cast<spread_layout_algorithm>(state.range(1));
        const spread_layout_request request {
            .container = { 960.0, 760.0 },
            .candidates = candidates,
            .count = static_cast<std::size_t>(state.range(0)),
            .algorithm = algorithm,
            .maximum_samples = 512,
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(select_spread_layout(request));
        }
        const spread_layout_result result = select_spread_layout(request);
        state.counters["valid"] = result.rectangles.size() == request.count
                && validate(request.container, result.rectangles).valid
            ? 1.0
            : 0.0;
        publish_quality(state, request.container, result.rectangles);
    }

    void benchmark_spread_lab_case(benchmark::State& state) {
        const equal_case& value
            = equal_lab_cases[static_cast<std::size_t>(state.range(0))];
        const equal_candidate_result candidates
            = generate_equal_rectangle_candidates(
                {
                    .container = value.container,
                    .item = value.item,
                    .count = value.count,
                    .algorithm = equal_packing_algorithm::certified_frame,
                }
            );
        const spread_layout_request request {
            .container = value.container,
            .candidates = candidates.rectangles,
            .count = value.count,
            .algorithm = static_cast<spread_layout_algorithm>(state.range(1)),
            .maximum_samples = 512,
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(select_spread_layout(request));
        }
        const spread_layout_result result = select_spread_layout(request);
        state.counters["N"] = static_cast<double>(value.count);
        state.counters["M"] = static_cast<double>(candidates.rectangles.size());
        publish_selection_result(state, value.container, value.count, result);
    }

    void benchmark_reciprocal(benchmark::State& state) {
        const reciprocal_layout_request request {
            .count = static_cast<std::size_t>(state.range(0)),
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(layout_reciprocal_rectangles(request));
        }
        const reciprocal_layout_result result
            = layout_reciprocal_rectangles(request);
        state.counters["placed"]
            = static_cast<double>(result.rectangles.size());
        state.counters["complete"]
            = result.status == reciprocal_layout_status::complete ? 1.0 : 0.0;
    }

    BENCHMARK(benchmark_equal)
        ->ArgsProduct(
            { { 1, 2, 4, 8, 12, 16, 24, 32, 52, 64, 96, 128, 192, 256 },
              { static_cast<int>(equal_packing_algorithm::automatic),
                static_cast<int>(equal_packing_algorithm::certified_frame),
                static_cast<int>(equal_packing_algorithm::count_binary_frame),
                static_cast<int>(equal_packing_algorithm::horizontal_grid),
                static_cast<int>(equal_packing_algorithm::vertical_grid) } }
        );
    BENCHMARK(benchmark_equal_lab_case)
        ->ArgsProduct(
            { { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 },
              { static_cast<int>(equal_packing_algorithm::certified_frame),
                static_cast<int>(
                    equal_packing_algorithm::count_binary_frame
                ) } }
        );
    BENCHMARK(benchmark_ordered)
        ->ArgsProduct(
            { { 1, 2, 4, 8, 12, 16, 24, 32, 48, 64, 96, 127, 128, 129, 192, 256,
                512 },
              { static_cast<int>(ordered_layout_algorithm::automatic),
                static_cast<int>(ordered_layout_algorithm::equal_grid),
                static_cast<int>(ordered_layout_algorithm::greedy_shelf),
                static_cast<int>(ordered_layout_algorithm::exact_shelf),
                static_cast<int>(ordered_layout_algorithm::balanced_shelf) } }
        );
    BENCHMARK(benchmark_ordered_lab_case)
        ->ArgsProduct(
            { { 0, 1, 2, 3, 4, 5 },
              { static_cast<int>(ordered_layout_algorithm::equal_grid),
                static_cast<int>(ordered_layout_algorithm::greedy_shelf),
                static_cast<int>(ordered_layout_algorithm::exact_shelf),
                static_cast<int>(ordered_layout_algorithm::balanced_shelf) } }
        );
    BENCHMARK(benchmark_free_order)
        ->ArgsProduct(
            { { 1, 2, 4, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 224, 255, 256,
                257, 288, 320 },
              { static_cast<int>(free_order_layout_algorithm::automatic),
                static_cast<int>(free_order_layout_algorithm::sorted_shelf),
                static_cast<int>(
                    free_order_layout_algorithm::multistart_shelf
                ) } }
        );
    BENCHMARK(benchmark_free_order_lab_case)
        ->ArgsProduct(
            { { 0, 1, 2, 3, 4, 5 },
              { static_cast<int>(free_order_layout_algorithm::sorted_shelf),
                static_cast<int>(
                    free_order_layout_algorithm::multistart_shelf
                ) } }
        );
    BENCHMARK(benchmark_fixed_size)
        ->ArgsProduct(
            { { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 2560, 3071,
                3072, 3073, 3584, 4096 },
              { static_cast<int>(fixed_size_layout_algorithm::automatic),
                static_cast<int>(
                    fixed_size_layout_algorithm::preserve_order_shelf
                ),
                static_cast<int>(
                    fixed_size_layout_algorithm::best_fit_decreasing_shelf
                ) } }
        );
    BENCHMARK(benchmark_compact)
        ->ArgsProduct(
            { { 2, 4, 6, 8, 12, 16, 24, 32, 48, 64, 80, 96 },
              { static_cast<int>(compact_layout_algorithm::automatic),
                static_cast<int>(compact_layout_algorithm::exact_bounding_box),
                static_cast<int>(compact_layout_algorithm::void_refined) } }
        );
    BENCHMARK(benchmark_compact_surplus)
        ->ArgsProduct(
            { { 24, 31, 32, 33, 40 },
              { 4, 8, 9, 16 },
              { static_cast<int>(compact_layout_algorithm::automatic),
                static_cast<int>(compact_layout_algorithm::exact_bounding_box),
                static_cast<int>(compact_layout_algorithm::void_refined) } }
        );
    BENCHMARK(benchmark_compact_lab_case)
        ->ArgsProduct(
            { { 0, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 },
              { static_cast<int>(compact_layout_algorithm::automatic),
                static_cast<int>(compact_layout_algorithm::exact_bounding_box),
                static_cast<int>(compact_layout_algorithm::void_refined) } }
        );
    BENCHMARK(benchmark_spread)
        ->ArgsProduct(
            { { 2, 4, 6, 8, 12, 16, 24, 31, 32, 33, 40, 48, 64, 80, 96 },
              { static_cast<int>(spread_layout_algorithm::automatic),
                static_cast<int>(spread_layout_algorithm::greedy_coverage),
                static_cast<int>(
                    spread_layout_algorithm::coverage_with_swaps
                ) } }
        );
    BENCHMARK(benchmark_spread_lab_case)
        ->ArgsProduct(
            { { 0, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 },
              { static_cast<int>(spread_layout_algorithm::automatic),
                static_cast<int>(spread_layout_algorithm::greedy_coverage),
                static_cast<int>(
                    spread_layout_algorithm::coverage_with_swaps
                ) } }
        );
    BENCHMARK(benchmark_reciprocal)
        ->Arg(1)
        ->Arg(2)
        ->Arg(4)
        ->Arg(8)
        ->Arg(12)
        ->Arg(16)
        ->Arg(24)
        ->Arg(32)
        ->Arg(48)
        ->Arg(64);

} // namespace
} // namespace packing

BENCHMARK_MAIN();
