# CTRE Phoenix bindings for Rust
This crate contains bindings for the CTRE Phoenix CCI libraries, and a port of the high-level Phoenix library.

This currently only supports motor controllers.
A couple of functions are still missing from the high-level abstraction.

This project is not affiliated with nor endorsed by Cross the Road Electronics.

## What is this `gen` directory?
The `gen` directory holds a couple of files that helped generate a starting point for everything.

The `gen_mot.j2` and `hooks.py` were used with [header2whatever](https://github.com/virtuald/header2whatever)
to quickly generate an initial motor control module.

The following command was used to bindgen the initial CCI bindings:
```bash
bindgen --disable-name-namespacing --rustified-enum 'ctre::phoenix::.*' --whitelist-type 'ctre::phoenix::.*' --whitelist-function 'c_.*' wrapper.hpp -- -std=c++11 -I $PHOENIX_ROOT/libraries/driver/include
```

## How do I actually use this?
Beats me.

## Does this actually work?
¯\\\_(ツ)_/¯

## License
CTRE-derived parts are under the [Phoenix Software License Agreement](LICENSE).
Rust-specific parts are under the [MIT license](LICENSE-MIT).
