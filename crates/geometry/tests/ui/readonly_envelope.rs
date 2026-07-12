//! M-KC1 — handing a read-only borrowed adapter to a
//! `PointMut`-requiring algorithm (`envelope` materialises a
//! `model::Box<P>` whose corners are `P`-typed) is a compile error
//! at the facade layer.

use boost_geometry::adapt::Adapt;
use boost_geometry::algorithm::envelope;

fn main() {
    let storage = [[0.0_f64, 0.0], [3.0, 4.0]];
    // Borrowed adapter implements Point but not PointMut.
    let a = Adapt(&storage[0]);
    let _ = envelope(&a);
}
