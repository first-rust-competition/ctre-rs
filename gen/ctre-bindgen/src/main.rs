// Copyright 2018 First Rust Competition Developers.
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

extern crate bindgen;
use std::path::PathBuf;
use std::{env, io};

#[derive(Debug)]
struct BindgenCallbacks;

impl bindgen::callbacks::ParseCallbacks for BindgenCallbacks {
    fn enum_variant_name(
        &self,
        enum_name: Option<&str>,
        original_variant_name: &str,
        _variant_value: bindgen::callbacks::EnumVariantValue,
    ) -> Option<String> {
        let variant_name = if original_variant_name.ends_with('_') {
            &original_variant_name[..original_variant_name.len() - 1]
        } else {
            original_variant_name
        };
        match enum_name {
            // remove leading e from ParamEnums
            Some("ParamEnum") => {
                assert!(
                    variant_name.starts_with('e'),
                    "ParamEnum::{} does not start with e",
                    variant_name
                );
                Some(variant_name[1..].to_owned())
            }
            Some(enum_name) if variant_name.starts_with(enum_name) => {
                Some(variant_name[enum_name.len() + 1..].to_owned())
            }
            _ => Some(variant_name.to_owned()),
        }
    }
}

fn generate_bindings(
    header: &str,
    clang_args: Vec<String>,
    to_write: Box<dyn io::Write>,
) -> io::Result<()> {
    const SYMBOL_REGEX: &str = "ctre::phoenix::.+";
    let bindings = bindgen::Builder::default()
        .derive_default(true)
        // .derive_ord(true)
        .header(header)
        .blacklist_type("std::string")
        .blacklist_type("ctre::phoenix::.*Routines")
        .opaque_type("std::.*")
        .whitelist_type(SYMBOL_REGEX)
        .whitelist_function("c_.+")
        .rustified_enum(SYMBOL_REGEX)
        .disable_name_namespacing()
        .ignore_methods()
        .ctypes_prefix("raw")
        .parse_callbacks(Box::new(BindgenCallbacks))
        /*
        .rustfmt_configuration_file(Some(
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("rustfmt.toml"),
        ))
        */
        .clang_arg("-xc++")
        .clang_arg("-std=c++14")
        .clang_args(clang_args);
    eprintln!("builder_args: {:?}", bindings.command_line_flags());
    let out = bindings.generate().expect("Unable to generate bindings");
    out.write(to_write)
}

fn main() -> io::Result<()> {
    let mut args = env::args();
    let _ = args.next();
    let header = args.next().unwrap();
    let clang_args = args.collect();
    generate_bindings(&header, clang_args, Box::new(io::stdout()))
}
