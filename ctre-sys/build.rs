use std::env;

const NI_LIB_LIST: &[&str] = &[
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
    println!("cargo:rustc-link-lib=stdc++");

    let path = env::current_dir().unwrap();
    let target = env::var("TARGET").unwrap();

    if target == "arm-unknown-linux-gnueabi" {
        // assume athena (FRC roboRIO)
        println!("cargo:rustc-link-search={}/lib/athena", path.display());
        for lib in NI_LIB_LIST {
            println!("cargo:rustc-link-lib=dylib={}", lib);
        }
    // } else if target.starts_with("x86_64") {
    //     println!("cargo:rustc-link-search={}/lib/x86-64", path.display());
    } else {
        panic!("Unsupported target!");
    }
}
