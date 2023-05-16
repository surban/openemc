use std::{
    env,
    ffi::OsStr,
    fs,
    fs::File,
    io::Write,
    path::{Path, PathBuf},
};

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    println!("cargo:rerun-if-env-changed=OPENEMC_BOARD");
    println!("cargo:rerun-if-changed=src/boards");

    let board = env::var("OPENEMC_BOARD").unwrap_or_else(|_| "generic".to_string());

    let src = format!("src/boards/{}.x", board.to_lowercase());
    if !Path::new(&src).is_file() {
        panic!("board {board} is unknown");
    }
    println!("cargo:rerun-if-changed={src}");
    let memory = fs::read(&src).unwrap_or_else(|_| panic!("cannot read board memory from {src}"));
    fs::write(out.join("memory.x"), memory).unwrap();

    let board_version_file = format!("src/boards/{}.version", board.to_lowercase());
    println!("cargo:rerun-if-changed={board_version_file}");
    let board_version = fs::read_to_string(&board_version_file)
        .unwrap_or_else(|_| panic!("cannot read board version from {board_version_file}"))
        .trim()
        .to_string();
    let pkg_version = env::var("CARGO_PKG_VERSION").unwrap();
    fs::write(out.join("bootloader_version.txt"), format!("{pkg_version}/{board_version}")).unwrap();

    let mut board_mods = File::create(out.join("board_mods.rs")).unwrap();
    for entry in fs::read_dir("src/boards").unwrap() {
        let entry = entry.unwrap();
        if entry.metadata().unwrap().is_file()
            && entry.path().extension() == Some(OsStr::new("rs"))
            && entry.file_name() != OsStr::new("mod.rs")
        {
            writeln!(&mut board_mods, "#[path=\"{}\"]", entry.path().canonicalize().unwrap().to_str().unwrap())
                .unwrap();
            writeln!(&mut board_mods, "mod {};", entry.path().file_stem().unwrap().to_str().unwrap()).unwrap();
        }
    }
    writeln!(&mut board_mods, "pub use {board}::BoardImpl as Chosen;").unwrap();

    println!("cargo:rerun-if-env-changed=OPENEMC_BOOTLOADER");
    println!("cargo:rerun-if-changed=emc-bootloader-big.x");
    println!("cargo:rerun-if-changed=emc-bootloader-normal.x");

    let ld_script = fs::read(if env::var("OPENEMC_BOOTLOADER").unwrap_or_default() == "big" {
        "emc-bootloader-big.x"
    } else {
        "emc-bootloader-normal.x"
    })
    .unwrap();
    fs::write(out.join("emc-bootloader.x"), ld_script).unwrap();

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-link-search={}", out.display());
}
