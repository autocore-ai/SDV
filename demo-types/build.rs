fn main() {
    let path = std::path::PathBuf::from("steam/src");
    //let path2 = std::path::PathBuf::from("src");
    let mut b = autocxx_build::build("src/lib.rs", &[&path], &[]).unwrap();
    b.flag_if_supported("-std=c++14")
        .file("steam/src/steam.cc")
        .compile("autocxx-steam-example");
    println!("cargo:rerun-if-changed=src/lib.rs");
}
