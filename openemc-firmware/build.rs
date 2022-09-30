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
    let src = format!("src/boards/{}.rs", board.to_lowercase());

    if !Path::new(&src).is_file() {
        panic!("board {board} is unknown");
    }

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
    println!("cargo:rerun-if-changed=memory-big.x");
    println!("cargo:rerun-if-changed=memory-normal.x");
    println!("cargo:rerun-if-changed=memory-standalone.x");

    let ld_script = fs::read(match env::var("OPENEMC_BOOTLOADER").unwrap_or_default().as_str() {
        "big" => "memory-big.x",
        "normal" => "memory-normal.x",
        "standalone" | "" => "memory-standalone.x",
        other => panic!("unknown bootloader variant: {other}"),
    })
    .unwrap();
    fs::write(out.join("memory.x"), ld_script).unwrap();

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-link-search={}", out.display());
}
