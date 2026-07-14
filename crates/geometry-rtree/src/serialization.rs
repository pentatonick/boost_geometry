//! Serde persistence for R-tree values.
//!
//! Boost's `index/detail/serialization.hpp` persists an R-tree through
//! Boost.Serialization. The Rust port's opt-in `serde` feature writes
//! the public value sequence and reconstructs a valid STR-packed tree
//! on load. Internal node layout is deliberately not a wire-format
//! promise, so split-policy and invariant changes remain compatible.

use alloc::vec::Vec;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::{Indexable, Rtree, SplitParameters};

impl<T, Params> Serialize for Rtree<T, Params>
where
    T: Indexable + Serialize,
    Params: SplitParameters,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.collect_seq(self.iter())
    }
}

impl<'de, T, Params> Deserialize<'de> for Rtree<T, Params>
where
    T: Indexable + Deserialize<'de>,
    Params: SplitParameters,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let values = Vec::<T>::deserialize(deserializer)?;
        Ok(values.into_iter().collect())
    }
}
