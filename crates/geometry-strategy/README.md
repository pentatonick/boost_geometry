# geometry-strategy

Part of the [boost_geometry](https://crates.io/crates/boost_geometry) workspace — a Rust port of [Boost.Geometry](https://www.boost.org/doc/libs/release/libs/geometry/). Most users should depend on the facade crate, which re-exports this one; depend on this crate directly only for a slimmer build.

## Pluggable algorithm strategies, keyed by coordinate-system family.

Mirrors `boost/geometry/strategies/` — every algorithm has a
strategy trait here; concrete strategies live in submodules keyed
by coordinate-system family (`cartesian`, `spherical`,
`geographic`).

### Writing a new strategy

Take the worked example: a Cartesian point-to-point distance
strategy (`Pythagoras`). A strategy for any algorithm follows the
same three steps.

#### Step 1 — Pick the coordinate-system family to bind on

Strategies bind on the [`CoordinateSystem::Family`](geometry_cs::CoordinateSystem::Family)
— never on the concrete CS — so that one impl covers both
`Spherical<Degree>` and `Spherical<Radian>` (or
`Geographic<Degree>` / `Geographic<Radian>`). The bound is
expressed via the [`geometry_tag::SameAs`] trait, the Rust
analogue of C++'s `std::is_same`:

```rust
impl<P1, P2> DistanceStrategy<P1, P2> for Pythagoras
where
    P1: Point,
    P2: Point<Scalar = P1::Scalar>,
    <P1::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
    <P2::Cs as CoordinateSystem>::Family: SameAs<CartesianFamily>,
{ /* ... */ }
```

The two `SameAs<CartesianFamily>` bounds form the family fence:
they refuse to monomorphise for a `Spherical` or `Geographic` point.
The `#[diagnostic::on_unimplemented]` plate on
[`geometry_tag::SameAs`] then redirects the resulting
compile error to the correct mitigation (wrap in
`geometry_adapt::WithCs<_, Geographic<_>>`, or pick a
CS-appropriate strategy such as `Haversine` / `Andoyer` /
`Vincenty`).

#### Step 2 — Decide whether to provide a `Comparable` form

A "comparable" form is a sibling strategy that returns the *same
ordering* as the real strategy but skips work the ordering does
not need. For Pythagoras this means returning the *squared*
distance and skipping the final `sqrt`. The squared form sorts
identically and is roughly twice as fast on a hot inner loop:

```rust
impl<P1, P2> DistanceStrategy<P1, P2> for Pythagoras /* ... */ {
    type Out = P1::Scalar;
    type Comparable = ComparablePythagoras; // <- skip-sqrt sibling
    fn distance(&self, a: &P1, b: &P2) -> Self::Out {
        self.comparable().distance(a, b).sqrt()
    }
    fn comparable(&self) -> Self::Comparable { ComparablePythagoras }
}
```

If the math has no equivalent shortcut (Andoyer, Vincenty,
Haversine after the half-angle formula), set
`type Comparable = Self;` — the optimiser collapses the
indirection. The doc on [`distance::DistanceStrategy::Comparable`]
warns implementers not to over-engineer this.

#### Step 3 — Wire the default selection

Each coordinate-system family picks one default strategy per
algorithm via [`distance::DefaultDistance`]:

```rust
impl DefaultDistance<CartesianFamily>  for CartesianFamily  { type Strategy = Pythagoras; }
impl DefaultDistance<SphericalFamily>  for SphericalFamily  { type Strategy = Haversine;  }
impl DefaultDistance<GeographicFamily> for GeographicFamily { type Strategy = Andoyer;    }
```

That is what makes the no-strategy `distance(a, b)` overload
resolve to the right algorithm: the type-level walk
`A → A::Point → Cs → Family → DefaultDistance<…B's family…>::Strategy`
resolves to the correct concrete strategy at the call site.

### Reverse dispatch — argument symmetry

For algorithms whose arguments are symmetric, write one impl per
tag pair `(A, B)` and the `Reversed<S>` blanket impl in
[`distance::Reversed`] picks up `(B, A)` automatically. The
analogue of Boost's `core/reverse_dispatch.hpp` partial
specialisation, done once at the strategy-trait layer instead of
per-algorithm.

### Module layout

Each algorithm has its own strategy trait module — `distance`,
`area`, `length`, `envelope`, `within`, `intersects`, `disjoint`,
`equals`. Concrete strategies live under `cartesian`,
`spherical`, or `geographic` per coordinate-system family.

## License

BSL-1.0 — see [LICENSE](https://github.com/pentatonick/boost_geometry/blob/main/LICENSE).
