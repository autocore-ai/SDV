use std::path::PathBuf;

fn main() {
    let path_api_inc = PathBuf::from("api/include");
    let path_api_src = PathBuf::from("api/src");
    let path_ros = PathBuf::from("/opt/ros/foxy/include");
    let path_autoware = PathBuf::from("/opt/AutowareAuto/include");
    let mut b = autocxx_build::build(
        "src/lib.rs",
        &[&path_ros, &path_api_inc, &path_api_src, &path_autoware],
        &[],
    )
    .unwrap();
    b.flag_if_supported("-std=c++14")
        .file("api/src/api.cpp")
        .compile("api");
    eprintln!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rustc-env=LD_LIBRARY_PATH=/opt/AutowareAuto/lib");
    println!("cargo:rustc-link-search=native=/opt/AutowareAuto/lib");
    println!("cargo:rustc-link-lib=dylib=simple_planning_simulator");
}
