//! OVL3 — traversal: walk the turn graph to assemble output rings.
//!
//! This is the densest single piece of overlay. Mirrors
//! `boost/geometry/algorithms/detail/overlay/traverse.hpp` and
//! `traversal.hpp`, but is a clean-room implementation of the classic
//! Weiler–Atherton ring traversal the turn graph encodes, rather than a
//! transliteration of Boost's template machinery.
//!
//! # The algorithm, for areal ∩ / ∪ / difference
//!
//! Both input polygons are treated as cyclic sequences of vertices. The
//! [`turn`](crate::turn) graph gives the points where the two
//! boundaries cross. Traversal threads these together:
//!
//! 1. **Enrich** ([`enrich`](mod@enrich)): splice every turn into both rings it lies
//!    on, so each ring becomes an alternating walk of original vertices
//!    and turn points, and every turn knows its position on *both*
//!    rings.
//! 2. **Walk** ([`state`]): start at an unvisited crossing turn whose
//!    operation matches the requested op, follow the current ring until
//!    the next turn, **switch** to the other ring there, and repeat
//!    until the walk returns to its start — emitting one output ring.
//!    Repeat until every crossing turn is visited.
//!
//! For **intersection** the walk keeps the arcs that lie *inside* the
//! other polygon; for **union**, the arcs *outside*; difference is union
//! against the reversed second polygon. Which arc that is, is decided at
//! each turn from the crossing's [`OperationType`](crate::turn::OperationType).
//!
//! # Scope (v1)
//!
//! The clean, non-degenerate areal case: simple polygons whose
//! boundaries cross transversally. Clustered turns (three-or-more
//! segments meeting at a point), self-intersections, and long collinear
//! overlaps are the two hardest Boost sub-problems and are deferred. Inputs that hit them return
//! [`TraversalError::Unsupported`] rather than a wrong ring.

pub mod enrich;
pub mod state;

pub use enrich::{EnrichedRings, enrich};
pub use state::{OverlayOp, TraversalError, traverse};
