//! OVL2.T1 — the turn data model.
//!
//! Mirrors `boost/geometry/algorithms/detail/overlay/turn_info.hpp`
//! (`turn_info` + `turn_operation`), the `operation_type` enum from
//! `overlay_type.hpp:31-39`, the `method_type` enum from
//! `turn_info.hpp:29-40`, and the `segment_identifier` from
//! `segment_identifier.hpp:35`.
//!
//! A turn belongs to exactly two geometries — the two inputs of an
//! overlay — so it carries **two** [`Operation`]s, one per source.
//! Boost stores them in a fixed `operations[2]` array; the port uses
//! `[Operation; 2]` for the same reason: the arity is a hard invariant,
//! not a runtime-variable list.

use geometry_trait::Point;

/// How a turn was formed — the case the segment-intersection
/// classification decided.
///
/// Mirrors `method_type` in
/// `boost/geometry/algorithms/detail/overlay/turn_info.hpp:29-40`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Method {
    /// No method assigned yet. Boost's `method_none`.
    #[default]
    None,
    /// The segments do not actually meet. Boost's `method_disjoint`.
    Disjoint,
    /// A proper crossing — the two segments cross transversally.
    /// Boost's `method_crosses`.
    Crosses,
    /// The segments touch at an endpoint of one of them, from outside.
    /// Boost's `method_touch`.
    Touch,
    /// One segment's endpoint touches the *interior* of the other.
    /// Boost's `method_touch_interior`.
    TouchInterior,
    /// The segments are collinear and overlap. Boost's
    /// `method_collinear`.
    Collinear,
    /// The segments are equal. Boost's `method_equal`.
    Equal,
    /// A start turn used to seed traversal. Boost's `method_start`.
    Start,
    /// An inconsistent turn the classifier could not resolve. Boost's
    /// `method_error`.
    Error,
}

/// What a single side of a turn does for the traversal — the role this
/// geometry's segment plays at the turn.
///
/// Mirrors `operation_type` in
/// `boost/geometry/algorithms/detail/overlay/overlay_type.hpp:31-39`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum OperationType {
    /// Unset. Boost's `operation_none`.
    #[default]
    None,
    /// Take this direction when computing a **union**. Boost's
    /// `operation_union`.
    Union,
    /// Take this direction when computing an **intersection**. Boost's
    /// `operation_intersection`.
    Intersection,
    /// This direction is blocked — traversal must not leave along it.
    /// Boost's `operation_blocked`.
    Blocked,
    /// Continue straight along the same ring (collinear continuation).
    /// Boost's `operation_continue`.
    Continue,
    /// The two geometries run in opposite directions here. Boost's
    /// `operation_opposite`.
    Opposite,
}

/// Which ring of a source geometry a segment came from.
///
/// Boost encodes this in `segment_identifier::ring_index`
/// (`-1` = exterior, `>= 0` = the n-th interior ring). The port makes
/// the exterior / interior distinction an explicit enum so call sites
/// never compare against a magic `-1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RingKind {
    /// The exterior (outer) ring. Boost's `ring_index == -1`.
    Exterior,
    /// The `n`-th interior ring (hole). Boost's `ring_index == n`.
    Interior(usize),
}

/// Names the segment a turn operation sits on, within one source
/// geometry.
///
/// Mirrors `segment_identifier` in
/// `boost/geometry/algorithms/detail/overlay/segment_identifier.hpp:35`,
/// reduced to the fields the v1 areal-areal overlay needs: which input,
/// which ring of it, and which segment index along that ring. Boost's
/// `multi_index` / `piece_index` are omitted until multi-geometry and
/// buffer support need them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SegmentId {
    /// Which input geometry (`0` or `1`). Boost's `source_index`.
    pub source_index: usize,
    /// Which ring of that geometry. Boost's `ring_index`.
    pub ring: RingKind,
    /// The index of the segment's start vertex along the ring. Boost's
    /// `segment_index`.
    pub segment_index: usize,
}

/// One side of a turn — the operation for one of the two source
/// geometries.
///
/// Mirrors `turn_operation` in
/// `boost/geometry/algorithms/detail/overlay/turn_info.hpp:52-58`. The
/// `fraction` field (Boost's `segment_ratio`) is the position of the
/// turn along its segment; the port stores it as a `[0, 1]` fraction
/// once enrichment (OVL2.5) needs it — the data-model task keeps only
/// the operation and the segment it belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Operation {
    /// The traversal role of this side. Boost's
    /// `turn_operation::operation`.
    pub operation: OperationType,
    /// The segment this side sits on. Boost's `turn_operation::seg_id`.
    pub seg_id: SegmentId,
}

impl Operation {
    /// Construct an operation with an as-yet-unclassified role.
    #[must_use]
    pub const fn new(seg_id: SegmentId) -> Self {
        Self {
            operation: OperationType::None,
            seg_id,
        }
    }
}

/// An intersection point between the two inputs, plus the metadata
/// traversal needs.
///
/// Mirrors `turn_info` in
/// `boost/geometry/algorithms/detail/overlay/turn_info.hpp:78-88`:
/// the intersection `point`, the `method` by which it was formed, and
/// the two `operations` (one per input geometry). `touch_only` mirrors
/// Boost's flag for a touch where the lines do not cross.
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_model::Point2D;
/// use geometry_overlay::turn::info::{
///     Method, Operation, RingKind, SegmentId, Turn,
/// };
///
/// type P = Point2D<f64, Cartesian>;
/// let seg = |src, seg| SegmentId { source_index: src, ring: RingKind::Exterior, segment_index: seg };
/// let t = Turn {
///     point: P::new(1.0, 1.0),
///     method: Method::Crosses,
///     operations: [Operation::new(seg(0, 3)), Operation::new(seg(1, 5))],
///     touch_only: false,
/// };
/// assert_eq!(t.method, Method::Crosses);
/// assert_eq!(t.operations[1].seg_id.segment_index, 5);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Turn<P: Point> {
    /// The intersection point. Boost's `turn_info::point`.
    pub point: P,
    /// How the turn was formed. Boost's `turn_info::method`.
    pub method: Method,
    /// One operation per input geometry. Boost's
    /// `turn_info::operations[2]`.
    pub operations: [Operation; 2],
    /// True for a `Touch` / `TouchInterior` turn where the two lines do
    /// not actually cross. Boost's `turn_info::touch_only`.
    pub touch_only: bool,
}

impl<P: Point> Turn<P> {
    /// True when both operations carry `op`. Mirrors
    /// `turn_info::both` (`turn_info.hpp:102-105`).
    #[must_use]
    pub fn both(&self, op: OperationType) -> bool {
        self.operations[0].operation == op && self.operations[1].operation == op
    }

    /// True when either operation carries `op`. Mirrors
    /// `turn_info::has` (`turn_info.hpp:107-111`).
    #[must_use]
    pub fn has(&self, op: OperationType) -> bool {
        self.operations[0].operation == op || self.operations[1].operation == op
    }

    /// True when the turn is blocked on both sides. Mirrors
    /// `turn_info::blocked` (`turn_info.hpp:118-121`).
    #[must_use]
    pub fn blocked(&self) -> bool {
        self.both(OperationType::Blocked)
    }
}

#[cfg(test)]
mod tests {
    //! Construct a turn and round-trip its fields — the OVL2.T1
    //! done-when. Mirrors the field-access checks in
    //! `test/algorithms/overlay/get_turn_info.cpp`.

    use super::{Method, Operation, OperationType, RingKind, SegmentId, Turn};
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;
    use geometry_trait::Point as _;

    type P = Point2D<f64, Cartesian>;

    fn seg(src: usize, ring: RingKind, idx: usize) -> SegmentId {
        SegmentId {
            source_index: src,
            ring,
            segment_index: idx,
        }
    }

    #[test]
    #[allow(clippy::float_cmp, reason = "Turn coordinates are exact literals.")]
    fn turn_round_trips_its_fields() {
        let t = Turn {
            point: P::new(2.0, 3.0),
            method: Method::Crosses,
            operations: [
                Operation::new(seg(0, RingKind::Exterior, 4)),
                Operation::new(seg(1, RingKind::Interior(2), 7)),
            ],
            touch_only: false,
        };
        assert_eq!(t.point.get::<0>(), 2.0);
        assert_eq!(t.point.get::<1>(), 3.0);
        assert_eq!(t.method, Method::Crosses);
        assert_eq!(t.operations[0].seg_id.source_index, 0);
        assert_eq!(t.operations[0].seg_id.ring, RingKind::Exterior);
        assert_eq!(t.operations[1].seg_id.ring, RingKind::Interior(2));
        assert_eq!(t.operations[1].seg_id.segment_index, 7);
        assert_eq!(t.operations[0].operation, OperationType::None);
    }

    #[test]
    fn both_has_blocked_predicates() {
        let mut t = Turn {
            point: P::new(0.0, 0.0),
            method: Method::Touch,
            operations: [
                Operation::new(seg(0, RingKind::Exterior, 0)),
                Operation::new(seg(1, RingKind::Exterior, 0)),
            ],
            touch_only: true,
        };
        assert!(!t.has(OperationType::Union));
        t.operations[0].operation = OperationType::Union;
        assert!(t.has(OperationType::Union));
        assert!(!t.both(OperationType::Union));

        t.operations[0].operation = OperationType::Blocked;
        t.operations[1].operation = OperationType::Blocked;
        assert!(t.blocked());
        assert!(t.both(OperationType::Blocked));
    }

    #[test]
    fn default_enums() {
        assert_eq!(Method::default(), Method::None);
        assert_eq!(OperationType::default(), OperationType::None);
    }
}
