//! Token-level implementation of `#[derive(Point)]`.
//!
//! Parses the annotated struct, reads the optional
//! `#[geometry(cs = "…", scalar = "…")]` attribute, and emits the
//! [`Geometry`](::geometry_trait::Geometry),
//! [`Point`](::geometry_trait::Point), and
//! [`PointMut`](::geometry_trait::PointMut) impl blocks —
//! `Point::get::<D>` reads, `PointMut::set::<D>` writes. The shape
//! mirrors `BOOST_GEOMETRY_REGISTER_POINT_2D`
//! (`boost/geometry/geometries/register/point.hpp:81-87`): one trait
//! specialisation per field, in declaration order.

use proc_macro_crate::{FoundCrate, crate_name};
use proc_macro2::{Span, TokenStream};
use quote::quote;
use syn::{Data, DeriveInput, Fields, parse2};

/// Absolute paths to the kernel crates the generated impls name, resolved
/// for the crate being compiled.
///
/// A crate that depends only on the `boost_geometry` facade has no
/// `geometry_trait` in its extern prelude (Cargo does not expose
/// transitive dependencies), so when the facade is a dependency there —
/// under whatever name Cargo gave it — the paths route through the
/// facade's hidden `__private` re-exports. Otherwise they name the kernel
/// crates directly, which is what a caller depending on `geometry-derive`
/// alongside `geometry-trait`, `geometry-tag`, and `geometry-cs` has in
/// scope.
struct KernelPaths {
    trait_: TokenStream,
    tag: TokenStream,
    cs: TokenStream,
}

fn kernel_paths() -> KernelPaths {
    paths_for(crate_name("boost_geometry").ok())
}

/// The paths implied by a facade lookup, split from [`kernel_paths`] so
/// the three outcomes can be decided without the ambient Cargo
/// environment that `crate_name` reads.
///
/// [`FoundCrate::Itself`] means the facade is the package being
/// compiled. That still names `::boost_geometry` rather than `crate`,
/// because the targets that reach it — the facade's own examples,
/// integration tests and benches — are separate crates that link the
/// facade as an extern crate; `crate` there would be the example, not
/// the facade.
fn paths_for(facade: Option<FoundCrate>) -> KernelPaths {
    let facade = match facade {
        Some(FoundCrate::Itself) => quote! { ::boost_geometry },
        Some(FoundCrate::Name(name)) => {
            let ident = syn::Ident::new(&name, Span::call_site());
            quote! { ::#ident }
        }
        None => {
            return KernelPaths {
                trait_: quote! { ::geometry_trait },
                tag: quote! { ::geometry_tag },
                cs: quote! { ::geometry_cs },
            };
        }
    };
    KernelPaths {
        trait_: quote! { #facade::__private::geometry_trait },
        tag: quote! { #facade::__private::geometry_tag },
        cs: quote! { #facade::__private::geometry_cs },
    }
}

/// Expand `#[derive(Point)]` on a single struct.
///
/// Returns the generated impls — or a `compile_error!` token stream if
/// the input is malformed (not a struct, no named fields, bad attribute
/// value, …). Errors are propagated by `to_compile_error` so the
/// downstream `rustc` diagnostic still points at the user's source.
pub(crate) fn expand(input: TokenStream) -> TokenStream {
    let ast: DeriveInput = match parse2(input) {
        Ok(a) => a,
        Err(e) => return e.to_compile_error(),
    };

    let name = &ast.ident;
    let (impl_generics, ty_generics, where_clause) = ast.generics.split_for_impl();
    let KernelPaths { trait_, tag, cs } = kernel_paths();

    // Parse #[geometry(cs = "…", scalar = "…")]. Both keys are optional.
    // Defaults: `Cartesian` and `f64`, matching the C++ register macro
    // family at `boost/geometry/geometries/register/point.hpp`. A `cs`
    // path is emitted as written; the generated block glob-imports the
    // CS crate so `Spherical<Degree>` resolves without a use-site import.
    let mut cs_path: TokenStream = quote! { #cs::Cartesian };
    let mut scalar: TokenStream = quote! { f64 };
    for attr in &ast.attrs {
        if !attr.path().is_ident("geometry") {
            continue;
        }
        let parse_result = attr.parse_nested_meta(|meta| {
            if meta.path.is_ident("cs") {
                let lit: syn::LitStr = meta.value()?.parse()?;
                let parsed: syn::Path = syn::parse_str(&lit.value())?;
                cs_path = quote! { #parsed };
                Ok(())
            } else if meta.path.is_ident("scalar") {
                let lit: syn::LitStr = meta.value()?.parse()?;
                let parsed: syn::Type = syn::parse_str(&lit.value())?;
                scalar = quote! { #parsed };
                Ok(())
            } else {
                Err(meta.error("unknown `#[geometry(...)]` key (expected `cs` or `scalar`)"))
            }
        });
        if let Err(e) = parse_result {
            return e.to_compile_error();
        }
    }

    // Field idents in declaration order become dimensions 0..N.
    let fields = match &ast.data {
        Data::Struct(s) => match &s.fields {
            Fields::Named(named) => named.named.iter().collect::<Vec<_>>(),
            _ => {
                return syn::Error::new_spanned(
                    name,
                    "#[derive(Point)] requires a struct with named fields",
                )
                .to_compile_error();
            }
        },
        _ => {
            return syn::Error::new_spanned(name, "#[derive(Point)] only supports structs")
                .to_compile_error();
        }
    };

    if fields.is_empty() {
        return syn::Error::new_spanned(
            name,
            "#[derive(Point)] requires at least one field (one dimension)",
        )
        .to_compile_error();
    }

    let dim = fields.len();
    let field_idents: Vec<_> = fields.iter().map(|f| f.ident.as_ref().unwrap()).collect();

    // Const-generic match arms: dimension index -> field access.
    let get_arms = field_idents
        .iter()
        .enumerate()
        .map(|(i, ident)| quote! { #i => self.#ident });
    let set_arms = field_idents
        .iter()
        .enumerate()
        .map(|(i, ident)| quote! { #i => self.#ident = value });

    // The impls live in an anonymous const so the CS glob import stays
    // scoped to the generated code.
    quote! {
        const _: () = {
            #[allow(unused_imports, clippy::wildcard_imports)]
            use #cs::*;
            impl #impl_generics #trait_::Geometry for #name #ty_generics #where_clause {
                type Kind  = #tag::PointTag;
                type Point = Self;
            }
            impl #impl_generics #trait_::Point for #name #ty_generics #where_clause {
                type Scalar = #scalar;
                type Cs     = #cs_path;
                const DIM: usize = #dim;

                fn get<const D: usize>(&self) -> Self::Scalar {
                    match D {
                        #( #get_arms , )*
                        _ => panic!("Point::get: dimension out of range"),
                    }
                }
            }
            impl #impl_generics #trait_::PointMut for #name #ty_generics #where_clause {
                fn set<const D: usize>(&mut self, value: Self::Scalar) {
                    match D {
                        #( #set_arms , )*
                        _ => panic!("PointMut::set: dimension out of range"),
                    }
                }
            }
        };
    }
}

#[cfg(test)]
mod tests {
    //! Drive `expand` directly over well-formed and malformed token
    //! streams. The generated code is checked by its stringified form:
    //! a success emits the three impls with the expected `Scalar`/`Cs`;
    //! each malformed input emits a `compile_error!` with a specific
    //! message.

    use super::{FoundCrate, KernelPaths, expand, paths_for};
    use quote::quote;

    /// The three emitted paths, as source text.
    fn rendered(paths: &KernelPaths) -> (String, String, String) {
        (
            paths.trait_.to_string(),
            paths.tag.to_string(),
            paths.cs.to_string(),
        )
    }

    /// The facade's own package: its examples, integration tests and
    /// benches are separate crates that link the facade under its real
    /// name, so this arm spells that name absolutely. `crate` would name
    /// the example instead, and `crate::__private` would not resolve —
    /// which is what `examples/parcel_buffer.rs` compiles to prove.
    #[test]
    fn the_facade_package_is_named_absolutely() {
        let (trait_, tag, cs) = rendered(&paths_for(Some(FoundCrate::Itself)));
        assert_eq!(trait_, ":: boost_geometry :: __private :: geometry_trait");
        assert_eq!(tag, ":: boost_geometry :: __private :: geometry_tag");
        assert_eq!(cs, ":: boost_geometry :: __private :: geometry_cs");
    }

    /// A downstream crate that renamed the facade in its `Cargo.toml` gets
    /// the name Cargo actually gave it, not the hard-coded package name —
    /// the whole reason the lookup exists.
    #[test]
    fn a_renamed_facade_dependency_is_named_as_renamed() {
        let found = FoundCrate::Name("bg".to_string());
        let (trait_, tag, cs) = rendered(&paths_for(Some(found)));
        assert_eq!(trait_, ":: bg :: __private :: geometry_trait");
        assert_eq!(tag, ":: bg :: __private :: geometry_tag");
        assert_eq!(cs, ":: bg :: __private :: geometry_cs");
    }

    /// No facade in the dependency graph: the caller pinned the kernel
    /// crates directly, so the generated impls must name them directly —
    /// routing through `__private` would not resolve.
    #[test]
    fn without_the_facade_the_kernel_crates_are_named_directly() {
        let (trait_, tag, cs) = rendered(&paths_for(None));
        assert_eq!(trait_, ":: geometry_trait");
        assert_eq!(tag, ":: geometry_tag");
        assert_eq!(cs, ":: geometry_cs");
    }

    /// A well-formed struct with no `#[geometry]` attribute defaults to
    /// `Cartesian` / `f64` and emits all three impls plus the field's
    /// dimension index.
    #[test]
    fn default_attributes_emit_cartesian_f64_impls() {
        let out = expand(quote! {
            struct P { x: f64, y: f64 }
        })
        .to_string();
        assert!(out.contains("Geometry"));
        assert!(out.contains("PointMut"));
        assert!(out.contains("Cartesian"));
        assert!(out.contains("f64"));
        assert!(out.contains("const DIM : usize = 2"));
        assert!(!out.contains("compile_error"));
    }

    /// `#[geometry(cs = "…", scalar = "…")]` overrides both defaults; the
    /// chosen coordinate system and scalar appear in the output.
    #[test]
    fn attribute_overrides_cs_and_scalar() {
        let out = expand(quote! {
            #[geometry(cs = "Spherical<Degree>", scalar = "f32")]
            struct P { lon: f32, lat: f32 }
        })
        .to_string();
        assert!(out.contains("Spherical"));
        assert!(out.contains("f32"));
        assert!(!out.contains("compile_error"));
    }

    /// An unrecognised `#[geometry(...)]` key yields a `compile_error!`
    /// naming the accepted keys.
    #[test]
    fn unknown_attribute_key_is_a_compile_error() {
        let out = expand(quote! {
            #[geometry(bogus = "x")]
            struct P { x: f64 }
        })
        .to_string();
        assert!(out.contains("compile_error"));
        assert!(out.contains("unknown"));
    }

    /// A tuple struct (unnamed fields) is rejected with the named-fields
    /// message.
    #[test]
    fn tuple_struct_is_rejected() {
        let out = expand(quote! {
            struct P(f64, f64);
        })
        .to_string();
        assert!(out.contains("compile_error"));
        assert!(out.contains("named fields"));
    }

    /// A non-struct item (here an enum) is rejected with the
    /// only-supports-structs message.
    #[test]
    fn enum_is_rejected() {
        let out = expand(quote! {
            enum E { A, B }
        })
        .to_string();
        assert!(out.contains("compile_error"));
        assert!(out.contains("only supports structs"));
    }

    /// A struct with no fields (zero dimensions) is rejected.
    #[test]
    fn empty_struct_is_rejected() {
        let out = expand(quote! {
            struct P {}
        })
        .to_string();
        assert!(out.contains("compile_error"));
        assert!(out.contains("at least one field"));
    }

    /// Input that does not even parse as a `DeriveInput` returns the
    /// parser's own `compile_error!` rather than panicking.
    #[test]
    fn unparseable_input_returns_compile_error() {
        let out = expand(quote! { this is not valid rust }).to_string();
        assert!(out.contains("compile_error"));
    }

    /// A non-string `cs` value is a `compile_error!`, not a silent fall
    /// back to the `Cartesian` default.
    #[test]
    fn non_string_cs_value_is_a_compile_error() {
        let out = expand(quote! {
            #[geometry(cs = 5)]
            struct P { x: f64 }
        })
        .to_string();
        assert!(out.contains("compile_error"), "got: {out}");
        assert!(out.contains("expected string literal"), "got: {out}");
        assert!(
            !out.contains("Cartesian"),
            "fell back to the default: {out}"
        );
    }

    /// A `cs` string that does not parse as a path is a `compile_error!`.
    #[test]
    fn unparsable_cs_path_is_a_compile_error() {
        let out = expand(quote! {
            #[geometry(cs = "not a path!")]
            struct P { x: f64 }
        })
        .to_string();
        assert!(out.contains("compile_error"), "got: {out}");
        assert!(
            !out.contains("Cartesian"),
            "fell back to the default: {out}"
        );
    }

    /// A non-string or unparsable `scalar` value is a `compile_error!`,
    /// not a silent fall back to `f64`.
    #[test]
    fn bad_scalar_values_are_compile_errors() {
        let out = expand(quote! {
            #[geometry(scalar = 32)]
            struct P { x: f64 }
        })
        .to_string();
        assert!(out.contains("compile_error"), "got: {out}");
        assert!(!out.contains("PointMut"), "fell back to the default: {out}");

        let out = expand(quote! {
            #[geometry(scalar = "f64 f64")]
            struct P { x: f64 }
        })
        .to_string();
        assert!(out.contains("compile_error"), "got: {out}");
        assert!(!out.contains("PointMut"), "fell back to the default: {out}");
    }
}
