# CTRE Phoenix library for Rust
This crate contains a port of the high-level CTRE Phoenix library.

This project is not affiliated with nor endorsed by Cross the Road Electronics.

## How do I actually use this?
Add [`ctre`](https://crates.io/crates/ctre) as a dependency in your Cargo.toml.

## Does this actually work?
It builds and links when linking with the WPILib HAL, so it should work.
¯\\\_(ツ)\_/¯

(Add [`wpilib`](https://crates.io/crates/wpilib) or wpilib-sys as a dependency
to your project. I can't guarantee this will work otherwise.)

## License
CTRE-derived parts are under the [Phoenix Software License Agreement](LICENSE).
Rust-specific parts are under the [MIT license](LICENSE-MIT).
