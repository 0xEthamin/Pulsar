//! Places the linker script where the linker finds it.

use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn Error>>
{
    let out_dir = PathBuf::from(env::var("OUT_DIR")?);
    fs::write(out_dir.join("memory.x"), include_bytes!("memory.x"))?;
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    Ok(())
}
