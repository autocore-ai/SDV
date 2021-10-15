// macro_rules! println {
//     ($($rest:tt)*) => {
//         #[cfg(debug_assertions)]
//         std::println!($($rest)*)
//     }
// }

#[proc_macro_attribute]
pub fn bridge(_attr: TokenStream, tokens: TokenStream) -> TokenStream {
    quote! {
        #[cfg(target_os = "wasm32")]
        fn cxx::bridge() {}
        #[cfg(not(target_os = "wasm32"))]
        compile_error!("This attribute can be used only on wasm target");
    }
}

#[cxx::bridge(namespace = "zenoh::flow")]
pub mod ffi {
    pub struct GeometryMsgsVector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    pub struct GeometryMsgsTwist {
        pub linear: GeometryMsgsVector3,
        pub angular: GeometryMsgsVector3,
    }
}
