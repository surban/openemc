use std::{
    env,
    ffi::OsStr,
    fs,
    fs::File,
    io::Write,
    path::{Path, PathBuf},
};

use openemc_build::read_settings;
use openemc_shared::{flash, CFG_FLASH_PAGES};

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    println!("cargo:rerun-if-env-changed=OPENEMC_BOARD");
    println!("cargo:rerun-if-changed=src/boards");

    let board = env::var("OPENEMC_BOARD").unwrap_or_else(|_| "generic".to_string());
    let board_file = Path::new(&format!("src/boards/{}.rs", board.to_lowercase())).to_path_buf();
    if !board_file.is_file() {
        panic!("board {board} is unknown");
    }
    println!("cargo:rustc-cfg=board=\"{board}\"");
    println!("cargo:rerun-if-changed={}", board_file.display());
    let board_settings = read_settings(&board_file).expect("cannot read board file");

    let board_version = board_settings.get("BOARD-VERSION").map(|v| v.as_str()).unwrap_or_else(|| "0");
    let pkg_version = env::var("CARGO_PKG_VERSION").unwrap();
    fs::write(out.join("version.txt"), format!("{pkg_version}/{board_version}")).unwrap();

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
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=memory-standalone.x");
    println!("cargo:rerun-if-changed=../openemc-bootloader/emc-bootloader-normal.x");
    println!("cargo:rerun-if-changed=../openemc-bootloader/emc-bootloader-big.x");

    fs::copy("memory.x", out.join("memory.x")).unwrap();
    let ld_script = fs::read(match env::var("OPENEMC_BOOTLOADER").unwrap_or_default().as_str() {
        "big" => "../openemc-bootloader/emc-bootloader-big.x",
        "normal" => "../openemc-bootloader/emc-bootloader-normal.x",
        "standalone" | "" => "memory-standalone.x",
        other => panic!("unknown bootloader variant: {other}"),
    })
    .unwrap();
    fs::write(out.join("bootloader.x"), ld_script).unwrap();

    let flash_size: usize = board_settings
        .get("FLASH-SIZE")
        .and_then(|s| s.parse().ok())
        .expect("cannot read OPENEMC-FLASH-SIZE from board");
    let ram_size: usize = board_settings
        .get("RAM-SIZE")
        .and_then(|s| s.parse().ok())
        .expect("cannot read OPENEMC-RAM-SIZE from board");

    let cfg_size = CFG_FLASH_PAGES * flash::page_size_for(flash_size);

    let mut board = File::create(out.join("board.x")).unwrap();
    write!(board, "__flash_size = {flash_size};").unwrap();
    write!(board, "__cfg_size = {cfg_size};").unwrap();
    write!(board, "__ram_size = {ram_size};").unwrap();
    board.flush().unwrap();

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-link-search={}", out.display());
}
