//! The SVG mapper.
//!
//! Mirrors `boost/geometry/io/svg/svg_mapper.hpp` — the C++ side keeps a
//! running bounding box (`m_bounding_box`) as geometries are added, builds
//! a `strategy::transform::map_transformer` once the caller asks for
//! output, and streams each geometry through that world → pixel transform.
//! This port folds the same idea into [`SvgMapper`]: [`add`](SvgMapper::add)
//! extends the combined world-coordinate bounding box and stashes each
//! geometry as a resolution-independent [`Shape`] plus its SVG style
//! string, while [`to_svg`](SvgMapper::to_svg) derives the aspect-preserving,
//! y-flipped transform from the final box and renders every shape.
//!
//! Cartesian only, write-only. The SVG y-axis grows *downward*, so the
//! transform flips y (`py = height - margin - (y - min_y) * scale`), matching
//! the `map_transformer<…, SameScale = true>` Boost instantiates in
//! `svg_mapper.hpp:277-284`.

use core::fmt::Write as _;

use alloc::string::String;
use alloc::vec::Vec;

use geometry_cs::Cartesian;
use geometry_model::{Linestring, MultiLinestring, MultiPoint, MultiPolygon, Point, Polygon, Ring};
use geometry_trait::{
    Linestring as LinestringTrait, MultiLinestring as MultiLinestringTrait,
    MultiPoint as MultiPointTrait, MultiPolygon as MultiPolygonTrait, Point as PointTrait,
    Polygon as PolygonTrait, Ring as RingTrait,
};

/// Fraction of the canvas reserved as a border around the drawing, on
/// every side. Mirrors the visual breathing room Boost leaves via its
/// `map_transformer` margin; 10% keeps geometries clear of the edges.
const MARGIN_FRACTION: f64 = 0.1;

/// Pixel radius drawn for every mapped point. Boost's SVG point writer
/// emits a `<circle>` of a fixed size (`svg_map<point_tag>` passes the
/// element size straight through); this is that fixed size.
const POINT_RADIUS: f64 = 5.0;

/// A geometry captured in **world coordinates**, ready to be transformed
/// to pixels once the mapper's bounding box is final.
///
/// One variant per drawable kind, mirroring the tag-dispatched
/// `svg_map<Tag, …>` specialisations in `svg_mapper.hpp`: a point becomes
/// a `<circle>`, an open range a `<polyline>`, and a polygon (which may
/// carry holes) a `<path>` with the even-odd fill rule.
enum Shape {
    /// A single point → `<circle>`.
    Point(f64, f64),
    /// An open sequence of points (a linestring) → `<polyline>`.
    Polyline(Vec<(f64, f64)>),
    /// A ring set: exterior plus zero or more holes → `<path>` with
    /// even-odd fill so the holes punch through.
    Polygon {
        /// The exterior ring's world coordinates.
        outer: Vec<(f64, f64)>,
        /// Each interior ring's world coordinates.
        holes: Vec<Vec<(f64, f64)>>,
    },
}

/// Accumulates Cartesian geometries and emits a self-contained SVG
/// document mapping their combined world-coordinate bounding box onto a
/// fixed pixel canvas.
///
/// Mirrors `boost::geometry::svg_mapper` from
/// `boost/geometry/io/svg/svg_mapper.hpp`. Add geometries with
/// [`add`](Self::add) — each extends the running bounding box — then call
/// [`to_svg`](Self::to_svg) to render. The transform is aspect-preserving
/// and y-flipped (SVG y grows downward).
///
/// # Examples
///
/// ```
/// use geometry_cs::Cartesian;
/// use geometry_io_svg::SvgMapper;
/// use geometry_model::{Point2D, Polygon, Ring};
///
/// let outer = Ring::from_vec(vec![
///     Point2D::<f64, Cartesian>::new(0.0, 0.0),
///     Point2D::new(0.0, 10.0),
///     Point2D::new(10.0, 10.0),
///     Point2D::new(10.0, 0.0),
///     Point2D::new(0.0, 0.0),
/// ]);
/// let poly = Polygon::with_inners(outer, vec![]);
///
/// let mut mapper = SvgMapper::new(400, 400);
/// mapper.add(&poly, "fill:rgb(0,0,255);stroke:black");
/// let svg = mapper.to_svg();
/// assert!(svg.contains("<svg"));
/// assert!(svg.contains("</svg>"));
/// assert!(svg.contains("path"));
/// ```
pub struct SvgMapper {
    /// Canvas width in pixels.
    width: u32,
    /// Canvas height in pixels.
    height: u32,
    /// Captured shapes, each paired with its SVG style string, in
    /// insertion order.
    elements: Vec<(Shape, String)>,
    /// Combined world-coordinate bounding box `(min_x, min_y, max_x,
    /// max_y)`, or `None` until the first geometry is added.
    bbox: Option<(f64, f64, f64, f64)>,
}

impl SvgMapper {
    /// Create an empty mapper for a `width` × `height` pixel canvas.
    ///
    /// Mirrors the `svg_mapper(stream, width, height, …)` constructor in
    /// `svg_mapper.hpp` (which records the canvas size and defers the
    /// transform until output time).
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry_io_svg::SvgMapper;
    /// let mapper = SvgMapper::new(800, 600);
    /// assert!(mapper.to_svg().contains("<svg"));
    /// ```
    #[must_use]
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            elements: Vec::new(),
            bbox: None,
        }
    }

    /// Add a geometry with an SVG `style` string (e.g.
    /// `"fill:rgb(0,0,255);stroke:black"`).
    ///
    /// Extends the combined bounding box by every point of `g` and records
    /// the geometry for rendering. Mirrors `svg_mapper::add(geometry)`
    /// followed by `svg_mapper::map(geometry, style)` in
    /// `svg_mapper.hpp` — the box is grown eagerly, the pixels are
    /// computed lazily in [`to_svg`](Self::to_svg).
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry_cs::Cartesian;
    /// use geometry_io_svg::SvgMapper;
    /// use geometry_model::Point2D;
    ///
    /// let mut mapper = SvgMapper::new(200, 200);
    /// mapper.add(&Point2D::<f64, Cartesian>::new(3.0, 4.0), "fill:red");
    /// assert!(mapper.to_svg().contains("<circle"));
    /// ```
    pub fn add<G: MapGeometry>(&mut self, g: &G, style: &str) {
        g.collect_into(self, style);
    }

    /// Render the accumulated geometries as a self-contained SVG document.
    ///
    /// Derives the world → pixel transform from the final bounding box
    /// (10% margin on every side, aspect-preserving, y-flipped) and emits
    /// one element per shape. With nothing added — or a degenerate box —
    /// a valid empty `<svg>` of the canvas size is produced.
    ///
    /// Mirrors the output side of `svg_mapper`: the destructor writes the
    /// closing `</svg>` after every mapped element has been streamed
    /// through the transform (`svg_mapper.hpp`).
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry_io_svg::SvgMapper;
    /// let svg = SvgMapper::new(100, 100).to_svg();
    /// assert!(svg.contains("<svg"));
    /// assert!(svg.contains("</svg>"));
    /// ```
    #[must_use]
    pub fn to_svg(&self) -> String {
        let width = f64::from(self.width);
        let height = f64::from(self.height);

        let mut out = String::new();
        out.push_str("<?xml version=\"1.0\" standalone=\"no\"?>\n");
        // Writing into a `String` is infallible, so the `Result` is
        // discarded at every `write!` below.
        let _ = writeln!(
            out,
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}\" height=\"{}\">",
            self.width, self.height
        );

        let transform = Transform::fit(self.bbox, width, height);
        for (shape, style) in &self.elements {
            render_shape(&mut out, shape, &transform, style);
        }

        out.push_str("</svg>");
        out
    }

    /// Grow the running bounding box to include world point `(x, y)`.
    fn expand(&mut self, x: f64, y: f64) {
        self.bbox = Some(match self.bbox {
            None => (x, y, x, y),
            Some((min_x, min_y, max_x, max_y)) => {
                (min_x.min(x), min_y.min(y), max_x.max(x), max_y.max(y))
            }
        });
    }

    /// Record a shape/style pair for rendering.
    fn push(&mut self, shape: Shape, style: &str) {
        self.elements.push((shape, String::from(style)));
    }
}

/// The world → pixel mapping derived from a bounding box and canvas.
///
/// Holds the aspect-preserving scale and the two offsets that carry a
/// world point to a pixel, with the y-axis flipped. Mirrors the affine
/// matrix `strategy::transform::map_transformer` builds in
/// `svg_mapper.hpp` once the box is final.
struct Transform {
    min_x: f64,
    min_y: f64,
    scale: f64,
    margin: f64,
    height: f64,
}

impl Transform {
    /// Build the fit transform for `bbox` onto a `width` × `height`
    /// canvas. A degenerate or absent box (a single point, or a zero
    /// extent on either axis) falls back to unit scale so the drawing
    /// still lands on the canvas rather than dividing by zero.
    fn fit(bbox: Option<(f64, f64, f64, f64)>, width: f64, height: f64) -> Self {
        let margin = MARGIN_FRACTION * width.min(height);
        let (min_x, min_y, max_x, max_y) = bbox.unwrap_or((0.0, 0.0, 0.0, 0.0));

        let span_x = max_x - min_x;
        let span_y = max_y - min_y;

        // Guard against a zero extent on either axis (single point, a
        // horizontal or vertical line): fall back to unit scale.
        let scale = if span_x > 0.0 && span_y > 0.0 {
            let scale_x = (width - 2.0 * margin) / span_x;
            let scale_y = (height - 2.0 * margin) / span_y;
            scale_x.min(scale_y)
        } else {
            1.0
        };

        Self {
            min_x,
            min_y,
            scale,
            margin,
            height,
        }
    }

    /// Map world `(x, y)` to a pixel `(px, py)`, flipping y so the
    /// world's up is the canvas's up.
    fn apply(&self, x: f64, y: f64) -> (f64, f64) {
        let px = self.margin + (x - self.min_x) * self.scale;
        let py = self.height - self.margin - (y - self.min_y) * self.scale;
        (px, py)
    }
}

/// Render one `shape` (with its `style`) into `out` under `transform`.
fn render_shape(out: &mut String, shape: &Shape, transform: &Transform, style: &str) {
    match shape {
        Shape::Point(x, y) => {
            let (px, py) = transform.apply(*x, *y);
            let _ = writeln!(
                out,
                "<circle cx=\"{}\" cy=\"{}\" r=\"{POINT_RADIUS}\" style=\"{style}\" />",
                round(px),
                round(py),
            );
        }
        Shape::Polyline(points) => {
            out.push_str("<polyline points=\"");
            push_point_list(out, points, transform);
            let _ = writeln!(out, "\" style=\"{style}\" />");
        }
        Shape::Polygon { outer, holes } => {
            out.push_str("<path d=\"");
            push_ring_path(out, outer, transform);
            for hole in holes {
                out.push(' ');
                push_ring_path(out, hole, transform);
            }
            // Even-odd fill so interior rings punch holes through the
            // exterior, matching Boost's polygon SVG output.
            let _ = writeln!(out, "\" style=\"{style}\" fill-rule=\"evenodd\" />");
        }
    }
}

/// Append a space-separated `x,y` pixel list for a `<polyline>`.
fn push_point_list(out: &mut String, points: &[(f64, f64)], transform: &Transform) {
    for (i, &(x, y)) in points.iter().enumerate() {
        if i > 0 {
            out.push(' ');
        }
        let (px, py) = transform.apply(x, y);
        let _ = write!(out, "{},{}", round(px), round(py));
    }
}

/// Append one closed ring as an SVG path sub-path (`M … L … Z`).
fn push_ring_path(out: &mut String, ring: &[(f64, f64)], transform: &Transform) {
    for (i, &(x, y)) in ring.iter().enumerate() {
        let (px, py) = transform.apply(x, y);
        let cmd = if i == 0 { 'M' } else { 'L' };
        if i > 0 {
            out.push(' ');
        }
        let _ = write!(out, "{cmd} {} {}", round(px), round(py));
    }
    out.push_str(" Z");
}

/// Round a pixel coordinate to a compact, stable string. Two decimal
/// places is well below one device pixel and keeps the output small.
fn round(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

/// Turns a concrete geometry into [`Shape`]s and grows the mapper's
/// bounding box.
///
/// Hidden from the public docs: callers use [`SvgMapper::add`], which
/// bounds on this trait. It exists so each geometry kind supplies its own
/// point extraction, mirroring the tag-dispatched `svg_map<Tag, …>`
/// specialisations in `boost/geometry/io/svg/svg_mapper.hpp`.
#[doc(hidden)]
pub trait MapGeometry {
    /// Record `self` into `mapper` under `style`, extending the bounding
    /// box by every point.
    fn collect_into(&self, mapper: &mut SvgMapper, style: &str);
}

/// Collect a ring's points into a world-coordinate list, expanding the
/// mapper's bounding box as it goes.
fn ring_coords<R>(mapper: &mut SvgMapper, ring: &R) -> Vec<(f64, f64)>
where
    R: RingTrait,
    R::Point: PointTrait<Scalar = f64>,
{
    let mut coords = Vec::new();
    for p in ring.points() {
        let (x, y) = (p.get::<0>(), p.get::<1>());
        mapper.expand(x, y);
        coords.push((x, y));
    }
    coords
}

impl MapGeometry for Point<f64, 2, Cartesian> {
    fn collect_into(&self, mapper: &mut SvgMapper, style: &str) {
        let (x, y) = (self.get::<0>(), self.get::<1>());
        mapper.expand(x, y);
        mapper.push(Shape::Point(x, y), style);
    }
}

impl<P: PointTrait<Scalar = f64>> MapGeometry for Linestring<P> {
    fn collect_into(&self, mapper: &mut SvgMapper, style: &str) {
        let mut coords = Vec::new();
        for p in self.points() {
            let (x, y) = (p.get::<0>(), p.get::<1>());
            mapper.expand(x, y);
            coords.push((x, y));
        }
        mapper.push(Shape::Polyline(coords), style);
    }
}

impl<P: PointTrait<Scalar = f64>> MapGeometry for Ring<P, true, true> {
    fn collect_into(&self, mapper: &mut SvgMapper, style: &str) {
        // A bare ring renders as a single-ring polygon — same shape Boost
        // gives a `ring_tag` geometry.
        let outer = ring_coords(mapper, self);
        mapper.push(
            Shape::Polygon {
                outer,
                holes: Vec::new(),
            },
            style,
        );
    }
}

impl<P: PointTrait<Scalar = f64>> MapGeometry for Polygon<P, true, true> {
    fn collect_into(&self, mapper: &mut SvgMapper, style: &str) {
        let outer = ring_coords(mapper, self.exterior());
        let holes = self.interiors().map(|r| ring_coords(mapper, r)).collect();
        mapper.push(Shape::Polygon { outer, holes }, style);
    }
}

impl<P: PointTrait<Scalar = f64>> MapGeometry for MultiPoint<P> {
    fn collect_into(&self, mapper: &mut SvgMapper, style: &str) {
        for p in self.points() {
            let (x, y) = (p.get::<0>(), p.get::<1>());
            mapper.expand(x, y);
            mapper.push(Shape::Point(x, y), style);
        }
    }
}

impl<L> MapGeometry for MultiLinestring<L>
where
    L: LinestringTrait,
    L::Point: PointTrait<Scalar = f64>,
{
    fn collect_into(&self, mapper: &mut SvgMapper, style: &str) {
        for ls in self.linestrings() {
            let mut coords = Vec::new();
            for p in ls.points() {
                let (x, y) = (p.get::<0>(), p.get::<1>());
                mapper.expand(x, y);
                coords.push((x, y));
            }
            mapper.push(Shape::Polyline(coords), style);
        }
    }
}

impl<Pg> MapGeometry for MultiPolygon<Pg>
where
    Pg: PolygonTrait,
    Pg::Point: PointTrait<Scalar = f64>,
{
    fn collect_into(&self, mapper: &mut SvgMapper, style: &str) {
        for pg in self.polygons() {
            let outer = ring_coords(mapper, pg.exterior());
            let holes = pg.interiors().map(|r| ring_coords(mapper, r)).collect();
            mapper.push(Shape::Polygon { outer, holes }, style);
        }
    }
}

#[cfg(test)]
mod tests {
    //! Structural witnesses. Exact pixel values are transform-dependent
    //! and fiddly, so these assert document shape (well-formed `<svg>`,
    //! the right element per kind) and that mapped coordinates land inside
    //! the canvas — never exact strings.

    use super::*;
    use alloc::vec;
    use geometry_cs::Cartesian;
    use geometry_model::Point2D;

    type Pt = Point2D<f64, Cartesian>;

    /// Every `x`/`y` numeric attribute value in `svg` must fall within
    /// the canvas `[0, dim]` on its axis. Extracts values following each
    /// `attr="` occurrence.
    fn assert_coords_in_canvas(svg: &str, width: f64, height: f64) {
        for (attr, limit) in [("cx=\"", width), ("cy=\"", height)] {
            for chunk in svg.split(attr).skip(1) {
                let val: f64 = chunk
                    .split('"')
                    .next()
                    .unwrap()
                    .parse()
                    .expect("numeric attribute");
                assert!(
                    (0.0..=limit).contains(&val),
                    "coordinate {val} out of canvas 0..={limit}"
                );
            }
        }
    }

    fn square() -> Polygon<Pt> {
        let outer = Ring::from_vec(vec![
            Pt::new(0.0, 0.0),
            Pt::new(0.0, 10.0),
            Pt::new(10.0, 10.0),
            Pt::new(10.0, 0.0),
            Pt::new(0.0, 0.0),
        ]);
        Polygon::with_inners(outer, vec![])
    }

    #[test]
    fn polygon_renders_as_path() {
        let mut mapper = SvgMapper::new(400, 400);
        mapper.add(&square(), "fill:rgb(0,0,255);stroke:black");
        let svg = mapper.to_svg();

        assert!(svg.contains("<svg"));
        assert!(svg.contains("</svg>"));
        assert!(svg.contains("path"));
        assert!(svg.contains("fill-rule=\"evenodd\""));
    }

    #[test]
    fn polygon_coordinates_land_inside_canvas() {
        let mut mapper = SvgMapper::new(400, 400);
        mapper.add(&square(), "fill:blue");
        let svg = mapper.to_svg();

        // Every `M`/`L` pixel in the path must be within 0..=400.
        let d = svg.split("d=\"").nth(1).unwrap().split('"').next().unwrap();
        for token in d.split_whitespace() {
            if let Ok(v) = token.parse::<f64>() {
                assert!(
                    (0.0..=400.0).contains(&v),
                    "path coordinate {v} outside canvas"
                );
            }
        }
    }

    #[test]
    fn polygon_with_hole_emits_two_subpaths() {
        let outer = Ring::from_vec(vec![
            Pt::new(0.0, 0.0),
            Pt::new(0.0, 10.0),
            Pt::new(10.0, 10.0),
            Pt::new(10.0, 0.0),
            Pt::new(0.0, 0.0),
        ]);
        let hole = Ring::from_vec(vec![
            Pt::new(2.0, 2.0),
            Pt::new(2.0, 4.0),
            Pt::new(4.0, 4.0),
            Pt::new(4.0, 2.0),
            Pt::new(2.0, 2.0),
        ]);
        let poly = Polygon::with_inners(outer, vec![hole]);

        let mut mapper = SvgMapper::new(300, 300);
        mapper.add(&poly, "fill:green");
        let svg = mapper.to_svg();

        // Two `M` commands => exterior sub-path plus one hole sub-path.
        assert_eq!(svg.matches("M ").count(), 2);
        assert!(svg.contains("fill-rule=\"evenodd\""));
    }

    #[test]
    fn single_point_renders_as_circle() {
        let mut mapper = SvgMapper::new(200, 200);
        mapper.add(&Pt::new(3.0, 4.0), "fill:red");
        let svg = mapper.to_svg();

        assert!(svg.contains("<circle"));
        assert!(svg.contains("<svg"));
        assert!(svg.contains("</svg>"));
        // Degenerate box (one point): still lands on the canvas.
        assert_coords_in_canvas(&svg, 200.0, 200.0);
    }

    #[test]
    fn linestring_renders_as_polyline() {
        let ls = Linestring(vec![
            Pt::new(0.0, 0.0),
            Pt::new(5.0, 8.0),
            Pt::new(10.0, 2.0),
        ]);
        let mut mapper = SvgMapper::new(250, 250);
        mapper.add(&ls, "stroke:black;fill:none");
        let svg = mapper.to_svg();

        assert!(svg.contains("<polyline"));
        assert!(svg.contains("<svg"));
        assert!(svg.contains("</svg>"));
    }

    #[test]
    fn empty_mapper_emits_valid_empty_svg() {
        let svg = SvgMapper::new(120, 80).to_svg();

        assert!(svg.contains("<svg"));
        assert!(svg.contains("</svg>"));
        assert!(svg.contains("width=\"120\""));
        assert!(svg.contains("height=\"80\""));
        // Nothing was added, so no drawable element appears.
        assert!(!svg.contains("<circle"));
        assert!(!svg.contains("<path"));
        assert!(!svg.contains("<polyline"));
    }

    #[test]
    fn multipoint_renders_a_circle_per_point() {
        let mp = MultiPoint(vec![
            Pt::new(1.0, 1.0),
            Pt::new(9.0, 9.0),
            Pt::new(5.0, 2.0),
        ]);
        let mut mapper = SvgMapper::new(300, 300);
        mapper.add(&mp, "fill:orange");
        let svg = mapper.to_svg();

        assert_eq!(svg.matches("<circle").count(), 3);
        assert_coords_in_canvas(&svg, 300.0, 300.0);
    }

    /// A bare `Ring` renders as a single-ring polygon `<path>` with one
    /// sub-path (`M`) and the even-odd fill rule.
    #[test]
    fn bare_ring_renders_as_single_ring_polygon() {
        let ring: Ring<Pt> = Ring::from_vec(vec![
            Pt::new(0.0, 0.0),
            Pt::new(0.0, 10.0),
            Pt::new(10.0, 10.0),
            Pt::new(10.0, 0.0),
            Pt::new(0.0, 0.0),
        ]);
        let mut mapper = SvgMapper::new(300, 300);
        mapper.add(&ring, "fill:purple");
        let svg = mapper.to_svg();

        assert!(svg.contains("<path"));
        assert_eq!(svg.matches("M ").count(), 1); // one ring, one sub-path
        assert!(svg.contains("fill-rule=\"evenodd\""));
    }

    /// A `MultiLineString` renders one `<polyline>` per member.
    #[test]
    fn multilinestring_renders_a_polyline_per_member() {
        let mls: MultiLinestring<Linestring<Pt>> = MultiLinestring(vec![
            Linestring(vec![Pt::new(0.0, 0.0), Pt::new(5.0, 5.0)]),
            Linestring(vec![Pt::new(1.0, 9.0), Pt::new(9.0, 1.0)]),
        ]);
        let mut mapper = SvgMapper::new(300, 300);
        mapper.add(&mls, "stroke:black;fill:none");
        let svg = mapper.to_svg();

        assert_eq!(svg.matches("<polyline").count(), 2);
    }

    /// A `MultiPolygon` renders one polygon `<path>` per member, each
    /// with its own sub-path.
    #[test]
    fn multipolygon_renders_a_path_per_member() {
        let mut member = square();
        member.inners.push(Ring::from_vec(vec![
            Pt::new(2.0, 2.0),
            Pt::new(2.0, 4.0),
            Pt::new(4.0, 4.0),
            Pt::new(4.0, 2.0),
            Pt::new(2.0, 2.0),
        ]));
        let mpg: MultiPolygon<Polygon<Pt>> = MultiPolygon(vec![member.clone(), member]);
        let mut mapper = SvgMapper::new(400, 400);
        mapper.add(&mpg, "fill:teal");
        let svg = mapper.to_svg();

        assert_eq!(svg.matches("<path").count(), 2);
        assert_eq!(svg.matches("M ").count(), 4); // one exterior and one hole each
    }
}
