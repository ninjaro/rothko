#include "packing/layout/equal_rectangles.hpp"

#include <iostream>

int main() {
    const packing::equal_packing_request request {
        .container = { 800.0, 600.0 },
        .item = { 88.0, 63.0 },
        .count = 16,
    };
    const packing::equal_packing_result result
        = packing::pack_equal_rectangles(request);

    std::cout << "algorithm=" << packing::algorithm_name(result.algorithm)
              << " scale=" << result.scale
              << " rectangles=" << result.rectangles.size() << '\n';
    return result.complete(request.count) ? 0 : 1;
}
