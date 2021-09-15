pub mod zf;
use autocxx::include_cpp;
use r2r::std_msgs::msg::Header;

include_cpp! {
    #include "c_to_cpp.hpp"
    #include "cpp_to_c.hpp"
    safety!(unsafe)
    generate!("CToCpp")
    generate!("CppToC")
    generate!("GetCovCToCpp")
    generate!("GetCovCppToC")
}

pub fn run() {
    let cov_c_to_cpp = ffi::GetCovCToCpp() as *mut ffi::CToCpp;
    let cov_cpp_to_c = ffi::GetCovCppToC() as *mut ffi::CppToC;
    let mut cov_c_to_cpp = unsafe { std::pin::Pin::new_unchecked(&mut *cov_c_to_cpp) };
    let mut cov_cpp_to_c = unsafe { std::pin::Pin::new_unchecked(&mut *cov_cpp_to_c) };
    let mut i = 1;
    while i < 10 {
        let ptr_c_header = cov_cpp_to_c.as_mut().CreateHeader(autocxx::c_int(i));
        std::thread::sleep(std::time::Duration::from_micros(1000));
        let mut native = r2r::NativeMsg::<Header>::new();
        unsafe {
            cov_c_to_cpp.as_mut().Time(ptr_c_header, native.void_ptr());
            // println!("{}", *tmp.);
            cov_c_to_cpp.as_mut().CovHeader(ptr_c_header);
        };
        i += 1;
    }
}
