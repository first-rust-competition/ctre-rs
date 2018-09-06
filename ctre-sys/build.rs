// use std::env;

const LIB_LIST: &[&str] = &[
    "FRC_NetworkCommunication",
    "NiFpga",
    "NiFpgaLv",
    "niriodevenum",
    "niriosession",
    "NiRioSrv",
    "RoboRIO_FRC_ChipObject",
    "visa",
];

fn main() {
    println!("cargo:rustc-link-lib=static=CTRE_PhoenixCCI");
    for lib in LIB_LIST {
        println!("cargo:rustc-link-lib=dylib={}", lib);
    }
    println!("cargo:rustc-link-lib=stdc++");
}
