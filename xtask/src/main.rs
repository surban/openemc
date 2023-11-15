//! Build script.
//!
//! Use `cargo xtask` to execute.

use clap::Parser;
use devx_cmd::cmd;
use std::{
    env, fs,
    path::{Path, PathBuf},
};

const ARCH: &str = "thumbv7m-none-eabi";

/// Build OpenEMC images.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Build big bootloader for bootloader debugging.
    #[arg(long)]
    big_bootloader: bool,

    /// Do not build the bootloader.
    #[arg(long)]
    no_bootloader: bool,

    /// Bootloader log level.
    #[arg(short, long, default_value = "info")]
    bootloader_log: String,

    /// Firmware log level.
    #[arg(short, long, default_value = "info")]
    log: String,

    /// Features to activate in firmware.
    #[clap(long, short = 'F')]
    features: Vec<String>,

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

    if !args.no_bootloader {
        cmd!(&cargo, "build", "--release", "--no-default-features", "--features", "defmt-ringbuf")
            .current_dir(project_root().join("openemc-bootloader"))
            .env("OPENEMC_BOARD", board)
            .env("OPENEMC_BOOTLOADER", bootloader_variant)
            .env("DEFMT_LOG", &args.bootloader_log)
            .run()?;
        fs::copy(
            project_root()
                .join("openemc-bootloader")
                .join("target")
                .join(ARCH)
                .join("release")
                .join("openemc-bootloader"),
            project_root().join("image").join(format!("{bootloader}.elf")),
        )?;
        cmd!(
            &cargo,
            "objcopy",
            "--release",
            "--no-default-features",
            "--features",
            "defmt-ringbuf",
            "--",
            "-O",
            "binary",
            format!("../image/{bootloader}.bin")
        )
        .current_dir(project_root().join("openemc-bootloader"))
        .env("OPENEMC_BOARD", board)
        .env("OPENEMC_BOOTLOADER", bootloader_variant)
        .env("DEFMT_LOG", &args.bootloader_log)
        .run()?;
    }

    cmd!(&cargo, "build", "--release", "--no-default-features", "--features", "defmt-ringbuf")
        .args(args.features.iter().flat_map(|feature| ["--features", feature]))
        .current_dir(project_root().join("openemc-firmware"))
        .env("OPENEMC_BOARD", board)
        .env("OPENEMC_BOOTLOADER", bootloader_variant)
        .env("DEFMT_LOG", &args.log)
        .run()?;
    fs::copy(
        project_root()
            .join("openemc-firmware")
            .join("target")
            .join(ARCH)
            .join("release")
            .join("openemc-firmware"),
        project_root().join("image").join(format!("{firmware}.elf")),
    )?;
    cmd!(&cargo, "objcopy", "--release", "--no-default-features", "--features", "defmt-ringbuf",)
        .args(args.features.iter().flat_map(|feature| ["--features", feature]))
        .args(["--", "-O", "binary", &format!("../image/{firmware}.bin")])
        .current_dir(project_root().join("openemc-firmware"))
        .env("OPENEMC_BOARD", board)
        .env("OPENEMC_BOOTLOADER", bootloader_variant)
        .env("DEFMT_LOG", &args.log)
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

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut files = vec![
            project_root().join("image").join(format!("{firmware}.emc")),
            project_root().join("image").join(format!("{firmware}.elf")),
        ];
        if !args.no_bootloader {
            files.extend([
                project_root().join("image").join(format!("{bootloader}.bin")),
                project_root().join("image").join(format!("{bootloader}.elf")),
            ]);
        }
        for file in files {
            let mut perms = fs::metadata(&file)?.permissions();
            perms.set_mode(perms.mode() & !0o111);
            fs::set_permissions(&file, perms)?;
        }
    }

    println!();
    println!("Board {board}:");
    if !args.no_bootloader {
        println!(
            "Built {}bootloader image/{bootloader}.{{bin,elf}} with log level {}",
            if emc_model == 0xd1 { "debug " } else { "" },
            &args.bootloader_log
        );
    }
    println!("Built OpenEMC firmware image/{firmware}.{{emc,elf}} with log level {}", &args.log);
    println!();

    Ok(())
}
