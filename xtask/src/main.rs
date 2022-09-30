//! Build script.
//!
//! Use `cargo xtask` to execute.

use clap::Parser;
use devx_cmd::cmd;
use std::{
    env, fs,
    path::{Path, PathBuf},
};

/// Build OpenEMC images.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Build big bootloader for bootloader debugging.
    #[arg(short, long)]
    big_bootloader: bool,

    /// Board to build.
    #[arg(default_value = "generic")]
    board: String,
}

fn project_root() -> PathBuf {
    Path::new(&env!("CARGO_MANIFEST_DIR")).ancestors().nth(1).unwrap().to_path_buf()
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let board = &args.board;
    let (bootloader_variant, emc_model) = match args.big_bootloader {
        false => ("normal", 0x01),
        true => ("big", 0xd1),
    };

    let board_id = if board == "generic" { String::new() } else { format!("_{board}") };
    let bootloader = format!("openemc_bootloader_{emc_model:02x}{board_id}");
    let firmware = format!("openemc_{emc_model:02x}{board_id}");

    fs::create_dir_all(project_root().join("image"))?;

    let cargo = env::var("CARGO").unwrap_or_else(|_| "cargo".to_string());

    cmd!(&cargo, "build", "--release")
        .current_dir(project_root().join("openemc-bootloader"))
        .env("OPENEMC_BOARD", &board)
        .env("OPENEMC_BOOTLOADER", &bootloader_variant)
        .run()?;
    cmd!(&cargo, "objcopy", "--release", "--", "-O", "binary", format!("../image/{bootloader}.bin"))
        .current_dir(project_root().join("openemc-bootloader"))
        .env("OPENEMC_BOARD", &board)
        .env("OPENEMC_BOOTLOADER", &bootloader_variant)
        .run()?;

    cmd!(&cargo, "build", "--release")
        .current_dir(project_root().join("openemc-firmware"))
        .env("OPENEMC_BOARD", &board)
        .env("OPENEMC_BOOTLOADER", &bootloader_variant)
        .run()?;
    cmd!(&cargo, "objcopy", "--release", "--", "-O", "binary", format!("../image/{firmware}.bin"))
        .current_dir(project_root().join("openemc-firmware"))
        .env("OPENEMC_BOARD", &board)
        .env("OPENEMC_BOOTLOADER", &bootloader_variant)
        .run()?;

    cmd!(
        &cargo,
        "run",
        "--manifest-path",
        "openemc-pack/Cargo.toml",
        "--quiet",
        "--release",
        "--",
        format!("image/{firmware}.bin")
    )
    .current_dir(project_root())
    .run()?;

    fs::remove_file(project_root().join("image").join(format!("{firmware}.bin")))?;

    println!();
    println!(
        "Built {}bootloader image/{bootloader}.bin and OpenEMC firmware image/{firmware}.emc for board {board}",
        if emc_model == 0xd1 { "debug " } else { "" }
    );

    Ok(())
}
