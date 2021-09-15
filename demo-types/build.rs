fn main() {
    let path_ros = std::path::PathBuf::from("/opt/ros/foxy/include/");
    let path_cov_inc = std::path::PathBuf::from("ros_msg_convertor/include");
    let path_cov_src = std::path::PathBuf::from("ros_msg_convertor/src");
    let mut b = autocxx_build::build(
        "src/lib.rs",
        &[&path_ros, &path_cov_inc, &path_cov_src],
        &[],
    )
    .unwrap();
    b.flag_if_supported("-std=c++14")
        .file("ros_msg_convertor/src/c_to_cpp.cpp")
        .file("ros_msg_convertor/src/cpp_to_c.cpp")
        .compile("ros_msg_convertor");
    eprintln!("cargo:rerun-if-changed=src/lib.rs");
}
