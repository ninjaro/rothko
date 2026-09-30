# packing core

This source root contains the public Qt-free C++20 library, its implementation,
tests, benchmarks, and a minimal facade:

- `include/packing/` is the stable value-type and algorithm-selection API;
- `packing/layout/` contains placement generators, while
  `packing/selection/` contains the separately linked candidate selectors;
- `src/packing/` mirrors those referents and contains their implementations;
- `tests/packing/` checks geometry, dispatch boundaries, independent small
  oracles, determinism, and invalid inputs;
- `benchmarks/packing/` compares every competing implementation on common
  inputs below, around, and above dispatch boundaries, and records quality
  counters alongside runtime.

The layout surface keeps three different heterogeneous problems explicit:
ordered layout preserves contiguous input order, free-order layout may return
a permutation, and fixed-size layout places final extents without optimizing a
common scale. They share only value types and proven placement mechanisms.

The `mvp` facade calls the same automatic dispatcher used by consumers and
prints the selected implementation, scale, and placement count for a small
card-packing request. Rendering stays in Yodau and Kcuckoounter so this library
does not impose a desktop or mobile UI dependency.
