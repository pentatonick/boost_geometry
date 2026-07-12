// T31 silent-Cartesian mitigation — `Pythagoras` must refuse to
// compile when fed a `Geographic` point. The friendly diagnostic
// rides on `geometry_tag::SameAs<CartesianFamily>` (the bound the
// `impl DistanceStrategy<P1, P2> for Pythagoras` block places on
// each point's CS family).

use geometry_adapt::{Adapt, WithCs};
use geometry_cs::{Degree, Geographic};
use geometry_strategy::distance::DistanceStrategy;
use geometry_strategy::Pythagoras;

fn main() {
    let a = WithCs::<_, Geographic<Degree>>::new(Adapt([4.9_f64, 52.4]));
    let b = WithCs::<_, Geographic<Degree>>::new(Adapt([2.3_f64, 48.8]));
    let _ = Pythagoras.distance(&a, &b);
}
