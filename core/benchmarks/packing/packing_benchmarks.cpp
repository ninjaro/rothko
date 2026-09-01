#include "packing/compact_layout.hpp"
#include "packing/equal_rectangles.hpp"
#include "packing/ordered_layout.hpp"
#include "packing/reciprocal_layout.hpp"
#include "packing/spread_layout.hpp"

#include <benchmark/benchmark.h>

#include <array>
#include <cstddef>
#include <vector>

namespace packing {
namespace {

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
    }

    void benchmark_reciprocal(benchmark::State& state) {
        const reciprocal_layout_request request {
            .count = static_cast<std::size_t>(state.range(0)),
        };
        for (auto _ : state) {
            benchmark::DoNotOptimize(layout_reciprocal_rectangles(request));
        }
    }

    BENCHMARK(benchmark_equal)
        ->Args({ 4, static_cast<int>(equal_packing_algorithm::automatic) })
        ->Args({ 16, static_cast<int>(equal_packing_algorithm::automatic) })
        ->Args({ 52, static_cast<int>(equal_packing_algorithm::automatic) })
        ->Args({ 128, static_cast<int>(equal_packing_algorithm::automatic) })
        ->Args(
            { 16, static_cast<int>(equal_packing_algorithm::certified_frame) }
        )
        ->Args(
            { 16,
              static_cast<int>(equal_packing_algorithm::count_binary_frame) }
        )
        ->Args(
            { 52, static_cast<int>(equal_packing_algorithm::certified_frame) }
        )
        ->Args(
            { 52,
              static_cast<int>(equal_packing_algorithm::count_binary_frame) }
        );
    BENCHMARK(benchmark_ordered)
        ->Args({ 8, static_cast<int>(ordered_layout_algorithm::automatic) })
        ->Args({ 16, static_cast<int>(ordered_layout_algorithm::automatic) })
        ->Args({ 17, static_cast<int>(ordered_layout_algorithm::automatic) })
        ->Args({ 32, static_cast<int>(ordered_layout_algorithm::automatic) })
        ->Args(
            { 16, static_cast<int>(ordered_layout_algorithm::balanced_shelf) }
        )
        ->Args(
            { 17, static_cast<int>(ordered_layout_algorithm::balanced_shelf) }
        )
        ->Args({ 16, static_cast<int>(ordered_layout_algorithm::greedy_shelf) })
        ->Args(
            { 17, static_cast<int>(ordered_layout_algorithm::greedy_shelf) }
        );
    BENCHMARK(benchmark_compact)
        ->Args({ 8, static_cast<int>(compact_layout_algorithm::automatic) })
        ->Args({ 16, static_cast<int>(compact_layout_algorithm::automatic) })
        ->Args({ 17, static_cast<int>(compact_layout_algorithm::automatic) })
        ->Args({ 32, static_cast<int>(compact_layout_algorithm::automatic) })
        ->Args({ 16, static_cast<int>(compact_layout_algorithm::void_refined) })
        ->Args(
            { 16,
              static_cast<int>(compact_layout_algorithm::exact_bounding_box) }
        )
        ->Args({ 17, static_cast<int>(compact_layout_algorithm::void_refined) })
        ->Args(
            { 17,
              static_cast<int>(compact_layout_algorithm::exact_bounding_box) }
        );
    BENCHMARK(benchmark_spread)
        ->Args({ 8, static_cast<int>(spread_layout_algorithm::automatic) })
        ->Args({ 9, static_cast<int>(spread_layout_algorithm::automatic) })
        ->Args({ 16, static_cast<int>(spread_layout_algorithm::automatic) })
        ->Args({ 32, static_cast<int>(spread_layout_algorithm::automatic) })
        ->Args(
            { 8,
              static_cast<int>(spread_layout_algorithm::coverage_with_swaps) }
        )
        ->Args(
            { 8, static_cast<int>(spread_layout_algorithm::greedy_coverage) }
        )
        ->Args(
            { 9,
              static_cast<int>(spread_layout_algorithm::coverage_with_swaps) }
        )
        ->Args(
            { 9, static_cast<int>(spread_layout_algorithm::greedy_coverage) }
        );
    BENCHMARK(benchmark_reciprocal)->Arg(8)->Arg(16)->Arg(32)->Arg(64);

} // namespace
} // namespace packing

BENCHMARK_MAIN();
