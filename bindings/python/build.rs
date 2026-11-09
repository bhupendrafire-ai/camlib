fn main() {
    pyo3_build_config::add_extension_module_link_args();
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos") {
        // Keep the Mach-O self identifier portable when the wheel relocates it.
        println!("cargo:rustc-cdylib-link-arg=-Wl,-install_name,@rpath/_native.abi3.so");
    }
}
