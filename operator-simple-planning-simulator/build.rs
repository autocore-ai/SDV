fn main() {
    let path_api_inc = std::path::PathBuf::from("api/include");
    let path_api_src = std::path::PathBuf::from("api/src");
    let mut b = autocxx_build::build("src/lib.rs", &[&path_api_inc, &path_api_src], &[]).unwrap();
    b.flag_if_supported("-std=c++14")
        .file("api/src/api.cpp")
        .compile("api");
    eprintln!("cargo:rerun-if-changed=src/lib.rs");
}
