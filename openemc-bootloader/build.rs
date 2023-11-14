use std::{
    env,
    ffi::OsStr,
    fs,
    fs::File,
    io::Write,
    path::{Path, PathBuf},
};

use openemc_build::read_settings;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    println!("cargo:rerun-if-env-changed=OPENEMC_BOARD");
    println!("cargo:rerun-if-changed=src/boards");

    let board = env::var("OPENEMC_BOARD").unwrap_or_else(|_| "generic".to_string());

    let board_file = Path::new(&format!("src/boards/{}.rs", board.to_lowercase())).to_path_buf();
    if !board_file.is_file() {
        panic!("board {board} is unknown");
    }
    println!("cargo:rerun-if-changed={}", board_file.display());
    let board_settings = read_settings(&board_file).expect("cannot read board file");

    let board_version = board_settings.get("BOARD-VERSION").map(|v| v.as_str()).unwrap_or_else(|| "0");
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
    println!("cargo:rerun-if-changed=emc-bootloader.x");
    println!("cargo:rerun-if-changed=emc-bootloader-big.x");
    println!("cargo:rerun-if-changed=emc-bootloader-normal.x");

    fs::copy("emc-bootloader.x", out.join("emc-bootloader.x")).unwrap();
    let ld_script = fs::read(if env::var("OPENEMC_BOOTLOADER").unwrap_or_default() == "big" {
        "emc-bootloader-big.x"
    } else {
        "emc-bootloader-normal.x"
    })
    .unwrap();
    fs::write(out.join("memory.x"), ld_script).unwrap();

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-link-search={}", out.display());
}
