fn main() {
    built::write_built_file().expect("Failed to acquire build-time information");

    env_config::generate_env_config_constants();

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
