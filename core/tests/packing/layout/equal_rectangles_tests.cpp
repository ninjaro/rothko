#include "packing/layout/equal_rectangles.hpp"

#include "packing/geometry.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string_view>

namespace packing {
namespace {

    void expect_complete_and_valid(
        const equal_packing_request& request, const equal_packing_result& result
    ) {
        ASSERT_TRUE(result.complete(request.count));
        ASSERT_EQ(result.rectangles.size(), request.count);
        EXPECT_TRUE(validate(request.container, result.rectangles).valid);

        for (std::size_t index = 0; index < result.rectangles.size(); ++index) {
            const rectangle& value = result.rectangles[index];
            EXPECT_EQ(value.source_index, index);
            EXPECT_TRUE(is_finite(value));
            EXPECT_GT(value.width, 0.0);
            EXPECT_GT(value.height, 0.0);
        }
    }

    void expect_same_layout(
        const equal_packing_result& lhs, const equal_packing_result& rhs
    ) {
        ASSERT_DOUBLE_EQ(lhs.scale, rhs.scale);
        ASSERT_EQ(lhs.algorithm, rhs.algorithm);
        ASSERT_EQ(lhs.rectangles.size(), rhs.rectangles.size());
        for (std::size_t index = 0; index < lhs.rectangles.size(); ++index) {
            const rectangle& left = lhs.rectangles[index];
            const rectangle& right = rhs.rectangles[index];
            EXPECT_DOUBLE_EQ(left.x, right.x);
            EXPECT_DOUBLE_EQ(left.y, right.y);
            EXPECT_DOUBLE_EQ(left.width, right.width);
            EXPECT_DOUBLE_EQ(left.height, right.height);
            EXPECT_EQ(left.source_index, right.source_index);
            EXPECT_EQ(left.rotated, right.rotated);
        }
    }

    [[nodiscard]] equal_packing_request card_request() {
        return {
            .container = { 800.0, 600.0 },
            .item = { 88.0, 63.0 },
            .count = 24,
            .orientation = orientation_constraint::allow_rotation,
            .algorithm = equal_packing_algorithm::automatic,
            .long_side_step = 0.5,
        };
    }

    TEST(EqualRectangles, RejectsInvalidRequestsWithoutPartialOutput) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();

        equal_packing_request request = card_request();
        request.count = 0;
        EXPECT_EQ(pack_equal_rectangles(request).rectangles.size(), 0U);
        EXPECT_EQ(pack_equal_rectangles(request).scale, 0.0);

        request = card_request();
        request.container.width = 0.0;
        EXPECT_FALSE(pack_equal_rectangles(request).complete(request.count));

        request = card_request();
        request.container.height = infinity;
        EXPECT_FALSE(pack_equal_rectangles(request).complete(request.count));

        request = card_request();
        request.item.width = nan;
        EXPECT_FALSE(pack_equal_rectangles(request).complete(request.count));

        request = card_request();
        request.item.height = -1.0;
        EXPECT_FALSE(pack_equal_rectangles(request).complete(request.count));

        request = card_request();
        request.long_side_step = 0.0;
        EXPECT_FALSE(pack_equal_rectangles(request).complete(request.count));

        request = card_request();
        request.long_side_step = nan;
        EXPECT_FALSE(pack_equal_rectangles(request).complete(request.count));
    }

    TEST(EqualRectangles, CertifiedScanRejectsImpracticalFiniteStepCounts) {
        equal_packing_request request {
            .container = { 1.0e12, 1.0e12 },
            .item = { 1.0, 1.0 },
            .count = 1,
            .orientation = orientation_constraint::allow_rotation,
            .algorithm = equal_packing_algorithm::certified_frame,
            .long_side_step = 0.5,
        };

        const equal_packing_result result = pack_equal_rectangles(request);
        EXPECT_EQ(result.algorithm, equal_packing_algorithm::certified_frame);
        EXPECT_EQ(result.scale, 0.0);
        EXPECT_TRUE(result.rectangles.empty());
    }

    TEST(EqualRectangles, AutomaticDispatchHonorsOrientationConstraint) {
        equal_packing_request request = card_request();
        EXPECT_EQ(
            select_equal_packing_algorithm(request),
            equal_packing_algorithm::certified_frame
        );

        request.orientation = orientation_constraint::horizontal_only;
        EXPECT_EQ(
            select_equal_packing_algorithm(request),
            equal_packing_algorithm::horizontal_grid
        );

        request.orientation = orientation_constraint::vertical_only;
        EXPECT_EQ(
            select_equal_packing_algorithm(request),
            equal_packing_algorithm::vertical_grid
        );
    }

    TEST(EqualRectangles, ExplicitAlgorithmIsNotOverriddenByAutomaticPolicy) {
        equal_packing_request request = card_request();
        request.algorithm = equal_packing_algorithm::count_binary_frame;
        EXPECT_EQ(
            select_equal_packing_algorithm(request),
            equal_packing_algorithm::count_binary_frame
        );

        const equal_packing_result result = pack_equal_rectangles(request);
        EXPECT_EQ(
            result.algorithm, equal_packing_algorithm::count_binary_frame
        );
        expect_complete_and_valid(request, result);
    }

    TEST(EqualRectangles, IncompatibleExplicitOrientationIsRejected) {
        equal_packing_request request = card_request();
        request.orientation = orientation_constraint::horizontal_only;
        request.algorithm = equal_packing_algorithm::vertical_grid;

        const equal_packing_result result = pack_equal_rectangles(request);
        EXPECT_EQ(result.scale, 0.0);
        EXPECT_TRUE(result.rectangles.empty());
        EXPECT_EQ(result.algorithm, equal_packing_algorithm::vertical_grid);

        request.algorithm = equal_packing_algorithm::certified_frame;
        const equal_packing_result frame_result
            = pack_equal_rectangles(request);
        EXPECT_EQ(frame_result.scale, 0.0);
        EXPECT_TRUE(frame_result.rectangles.empty());
    }

    TEST(EqualRectangles, CertifiedFrameReturnsExactlyTheRequestedCount) {
        equal_packing_request request = card_request();
        const equal_packing_result result = pack_equal_rectangles(request);

        EXPECT_EQ(result.algorithm, equal_packing_algorithm::certified_frame);
        expect_complete_and_valid(request, result);
    }

    TEST(EqualRectangles, CandidateStagePortsSurplusLabLayouts) {
        struct lab_case {
            extent container {};
            extent item {};
            std::size_t count {};
        };

        // card_packer_lab_v3 MODE_EXAMPLE_CASES and high-signal spread cases.
        constexpr std::array cases {
            lab_case { { 800.0, 600.0 }, { 88.0, 63.0 }, 24 },
            lab_case { { 1337.0, 252.0 }, { 101.0, 72.0 }, 8 },
            lab_case { { 1334.0, 284.0 }, { 88.0, 60.0 }, 7 },
            lab_case { { 905.0, 330.0 }, { 75.0, 71.0 }, 6 },
            lab_case { { 1377.0, 320.0 }, { 108.0, 48.0 }, 23 },
        };

        for (const lab_case& value : cases) {
            const equal_packing_request request {
                .container = value.container,
                .item = value.item,
                .count = value.count,
                .algorithm = equal_packing_algorithm::certified_frame,
            };
            const equal_candidate_result candidates
                = generate_equal_rectangle_candidates(request);
            SCOPED_TRACE(value.count);
            ASSERT_TRUE(candidates.sufficient(value.count));
            EXPECT_GT(candidates.rectangles.size(), value.count);
            EXPECT_TRUE(validate(value.container, candidates.rectangles).valid);
            expect_complete_and_valid(request, pack_equal_rectangles(request));
        }
    }

    TEST(EqualRectangles, CertifiedFrameSupportsAlternativeAspectRatios) {
        equal_packing_request request {
            .container = { 1039.0, 640.0 },
            .item = { 73.0, 39.0 },
            .count = 19,
            .orientation = orientation_constraint::allow_rotation,
            .algorithm = equal_packing_algorithm::certified_frame,
            .long_side_step = 0.5,
        };

        const equal_packing_result result = pack_equal_rectangles(request);
        expect_complete_and_valid(request, result);
    }

    TEST(EqualRectangles, CertifiedFrameRepairsHistoricalOverlapCases) {
        struct regression_case {
            extent container {};
            extent item {};
            std::size_t count {};
        };

        constexpr std::array cases {
            regression_case { { 394.0, 465.0 }, { 76.0, 35.0 }, 66 },
            regression_case { { 271.0, 1145.0 }, { 95.0, 39.0 }, 15 },
            regression_case { { 840.0, 849.0 }, { 83.0, 68.0 }, 51 },
            regression_case { { 1039.0, 640.0 }, { 73.0, 39.0 }, 19 },
        };

        for (const regression_case& value : cases) {
            equal_packing_request request {
                .container = value.container,
                .item = value.item,
                .count = value.count,
                .orientation = orientation_constraint::allow_rotation,
                .algorithm = equal_packing_algorithm::certified_frame,
                .long_side_step = 0.5,
            };
            SCOPED_TRACE(value.count);
            expect_complete_and_valid(request, pack_equal_rectangles(request));
        }
    }

    TEST(EqualRectangles, FramePackingIsDeterministicIncludingSourceOrder) {
        equal_packing_request request = card_request();
        const equal_packing_result first = pack_equal_rectangles(request);
        const equal_packing_result second = pack_equal_rectangles(request);

        expect_complete_and_valid(request, first);
        expect_same_layout(first, second);
    }

    TEST(EqualRectangles, HorizontalGridUsesOnlyHorizontalRectangles) {
        equal_packing_request request {
            .container = { 100.0, 60.0 },
            .item = { 10.0, 5.0 },
            .count = 6,
            .orientation = orientation_constraint::horizontal_only,
            .algorithm = equal_packing_algorithm::automatic,
            .long_side_step = 0.5,
        };

        const equal_packing_result result = pack_equal_rectangles(request);
        EXPECT_EQ(result.algorithm, equal_packing_algorithm::horizontal_grid);
        expect_complete_and_valid(request, result);
        EXPECT_DOUBLE_EQ(result.scale, 4.0);
        for (const rectangle& value : result.rectangles) {
            EXPECT_FALSE(value.rotated);
            EXPECT_DOUBLE_EQ(value.width, 40.0);
            EXPECT_DOUBLE_EQ(value.height, 20.0);
        }
    }

    TEST(EqualRectangles, VerticalGridUsesOnlyVerticalRectangles) {
        equal_packing_request request {
            .container = { 100.0, 60.0 },
            .item = { 10.0, 5.0 },
            .count = 6,
            .orientation = orientation_constraint::vertical_only,
            .algorithm = equal_packing_algorithm::automatic,
            .long_side_step = 0.5,
        };

        const equal_packing_result result = pack_equal_rectangles(request);
        EXPECT_EQ(result.algorithm, equal_packing_algorithm::vertical_grid);
        expect_complete_and_valid(request, result);
        for (const rectangle& value : result.rectangles) {
            EXPECT_TRUE(value.rotated);
            EXPECT_LT(value.width, value.height);
        }
    }

    TEST(EqualRectangles, PortraitSourceRatioRetainsMeaningfulRotationFlags) {
        equal_packing_request request {
            .container = { 120.0, 80.0 },
            .item = { 2.0, 3.0 },
            .count = 7,
            .orientation = orientation_constraint::horizontal_only,
            .algorithm = equal_packing_algorithm::automatic,
            .long_side_step = 0.5,
        };

        const equal_packing_result horizontal = pack_equal_rectangles(request);
        expect_complete_and_valid(request, horizontal);
        for (const rectangle& value : horizontal.rectangles) {
            EXPECT_TRUE(value.rotated);
            EXPECT_GT(value.width, value.height);
        }

        request.orientation = orientation_constraint::vertical_only;
        const equal_packing_result vertical = pack_equal_rectangles(request);
        expect_complete_and_valid(request, vertical);
        for (const rectangle& value : vertical.rectangles) {
            EXPECT_FALSE(value.rotated);
            EXPECT_LT(value.width, value.height);
        }
    }

    TEST(EqualRectangles, HorizontalGridScaleDoesNotIncreaseWithCount) {
        double previous_scale = std::numeric_limits<double>::infinity();
        for (std::size_t count = 1; count <= 32; ++count) {
            equal_packing_request request {
                .container = { 640.0, 360.0 },
                .item = { 16.0, 9.0 },
                .count = count,
                .orientation = orientation_constraint::horizontal_only,
                .algorithm = equal_packing_algorithm::horizontal_grid,
                .long_side_step = 0.5,
            };
            const equal_packing_result result = pack_equal_rectangles(request);
            SCOPED_TRACE(count);
            expect_complete_and_valid(request, result);
            EXPECT_LE(result.scale, previous_scale + 1.0e-12);
            previous_scale = result.scale;
        }
    }

    TEST(EqualRectangles, CertifiedFrameScaleDoesNotIncreaseWithCount) {
        double previous_scale = std::numeric_limits<double>::infinity();
        for (std::size_t count = 1; count <= 32; ++count) {
            equal_packing_request request {
                .container = { 800.0, 600.0 },
                .item = { 88.0, 63.0 },
                .count = count,
                .orientation = orientation_constraint::allow_rotation,
                .algorithm = equal_packing_algorithm::certified_frame,
                .long_side_step = 0.5,
            };
            const equal_packing_result result = pack_equal_rectangles(request);
            SCOPED_TRACE(count);
            expect_complete_and_valid(request, result);
            EXPECT_LE(result.scale, previous_scale + 1.0e-12);
            previous_scale = result.scale;
        }
    }

    TEST(EqualRectangles, EveryNamedAlgorithmHasAStableDiagnosticName) {
        EXPECT_EQ(
            algorithm_name(equal_packing_algorithm::automatic), "automatic"
        );
        EXPECT_EQ(
            algorithm_name(equal_packing_algorithm::certified_frame),
            "certified_frame"
        );
        EXPECT_EQ(
            algorithm_name(equal_packing_algorithm::count_binary_frame),
            "count_binary_frame"
        );
        EXPECT_EQ(
            algorithm_name(equal_packing_algorithm::horizontal_grid),
            "horizontal_grid"
        );
        EXPECT_EQ(
            algorithm_name(equal_packing_algorithm::vertical_grid),
            "vertical_grid"
        );
        EXPECT_EQ(
            algorithm_name(static_cast<equal_packing_algorithm>(999)),
            std::string_view { "unknown" }
        );
    }

} // namespace
} // namespace packing
