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
