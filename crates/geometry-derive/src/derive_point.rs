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

use proc_macro2::TokenStream;
use quote::quote;
use syn::{Data, DeriveInput, Fields, parse2};

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

    // Parse #[geometry(cs = "…", scalar = "…")]. Both keys are optional.
    // Defaults: `Cartesian` and `f64`, matching the C++ register macro
    // family at `boost/geometry/geometries/register/point.hpp`.
    let mut cs_path: TokenStream = quote! { ::geometry_cs::Cartesian };
    let mut scalar: TokenStream = quote! { f64 };
    for attr in &ast.attrs {
        if !attr.path().is_ident("geometry") {
            continue;
        }
        let parse_result = attr.parse_nested_meta(|meta| {
            if meta.path.is_ident("cs") {
                let lit: syn::LitStr = meta.value()?.parse()?;
                let parsed: syn::Path = syn::parse_str(&lit.value())?;
                cs_path = quote! { ::geometry_cs::#parsed };
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

    quote! {
        impl #impl_generics ::geometry_trait::Geometry for #name #ty_generics #where_clause {
            type Kind  = ::geometry_tag::PointTag;
            type Point = Self;
        }
        impl #impl_generics ::geometry_trait::Point for #name #ty_generics #where_clause {
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
        impl #impl_generics ::geometry_trait::PointMut for #name #ty_generics #where_clause {
            fn set<const D: usize>(&mut self, value: Self::Scalar) {
                match D {
                    #( #set_arms , )*
                    _ => panic!("PointMut::set: dimension out of range"),
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    //! Drive `expand` directly over well-formed and malformed token
    //! streams. The generated code is checked by its stringified form:
    //! a success emits the three impls with the expected `Scalar`/`Cs`;
    //! each malformed input emits a `compile_error!` with a specific
    //! message.

    use super::expand;
    use quote::quote;

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
}
