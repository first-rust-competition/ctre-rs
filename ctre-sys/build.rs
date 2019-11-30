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
const SIM_LIB_LIST: &[&str] = &["CTRE_PhoenixCanutils", "CTRE_PhoenixPlatform"];

fn main() {
    println!("cargo:rustc-link-lib=static=CTRE_PhoenixCCI");
    println!("cargo:rustc-link-lib=stdc++");

    let path = env::current_dir().unwrap();
    let target = env::var("TARGET").unwrap();

    if target == "arm-unknown-linux-gnueabi" || target == "armv7-unknown-linux-gnueabi" {
        // assume athena (FRC roboRIO)
        println!("cargo:rustc-link-search={}/lib/athena", path.display());
        for lib in NI_LIB_LIST {
            println!("cargo:rustc-link-lib=dylib={}", lib);
        }
    } else {
        // assume simulation
        println!(
            "cargo:rustc-link-search={}/lib/{}",
            path.display(),
            match target.as_ref() {
                "arm-unknown-linux-gnueabihf" | "armv7-unknown-linux-gnueabihf" => "armhf",
                "aarch64-unknown-linux-gnu" => "aarch64",
                _ => {
                    if target.starts_with("x86_64") {
                        "x86_64"
                    } else {
                        panic!("Unsupported target for ctre-sys!")
                    }
                }
            }
        );
        for lib in SIM_LIB_LIST {
            println!("cargo:rustc-link-lib=static={}", lib);
        }
    }
}
