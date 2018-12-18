# CTRE Phoenix bindings for Rust (enums and structs)
This crate contains enums and structs used in the CTRE Phoenix libraries.
Most of this crate features optional serde support.

This project is not affiliated with nor endorsed by Cross the Road Electronics.

## Was this auto-generated?
The following command was used to bindgen the initial CCI enums and structs:
```bash
bindgen --disable-name-namespacing --rustified-enum '.*' --whitelist-type 'ctre::phoenix::.*' wrapper.hpp -- -std=c++11 -I $PHOENIX_ROOT/libraries/driver/include
```

## License
[Phoenix Software License Agreement](LICENSE).
