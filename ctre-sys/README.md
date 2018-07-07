# CTRE Phoenix CCI bindings for Rust
This crate contains FFI bindings for the CTRE Phoenix CCI libraries.

This project is not affiliated with nor endorsed by Cross the Road Electronics.

## Was this auto-generated?
The following command was used to bindgen the initial CCI bindings:
```bash
bindgen --disable-name-namespacing --rustified-enum '.*' --whitelist-type 'ctre::phoenix::.*' --whitelist-function 'c_.*' wrapper.hpp -- -std=c++11 -I $PHOENIX_ROOT/libraries/driver/include
```

## License
[Phoenix Software License Agreement](LICENSE).
