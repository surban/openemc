//! Pack firmware in format bootable by OpenEMC bootloader.

use byteorder::{WriteBytesExt, LE};
use clap::Parser;
use regex::Regex;
use std::{env, fs, fs::File, io, io::Write, mem::size_of, num::ParseIntError, path::PathBuf};

use openemc_shared::PROGRAM_SIGNATURE;

/// Pack firmware in format bootable by EMC bootloader.
#[derive(Parser, Debug)]
struct Args {
    /// Override start address of user flash.
    #[clap(long, value_parser=parse_maybe_hex)]
    start: Option<u32>,
    /// Override flash page size.
    #[clap(long, value_parser=parse_maybe_hex)]
    page_size: Option<u32>,
    /// Program id.
    #[clap(long, value_parser=parse_maybe_hex)]
    id: Option<u32>,
    /// Source path.
    src: PathBuf,
    /// Destination path of packed firmware.
    dst: Option<PathBuf>,
}

fn parse_maybe_hex(s: &str) -> Result<u32, ParseIntError> {
    match s.to_ascii_lowercase().strip_prefix("0x") {
        Some(hex) => u32::from_str_radix(hex, 16),
        None => s.parse::<u32>(),
    }
}

static MEMORY_BIG_BOOTLOADER: &str = include_str!("../../openemc-firmware/memory-big.x");
static MEMORY_NORMAL: &str = include_str!("../../openemc-firmware/memory-normal.x");

fn get_flash_params(linker_script: &str) -> (u32, u32) {
    let re = Regex::new(r"FLASH : ORIGIN = 0x(\w+)").unwrap();
    let origin = re.captures(linker_script).unwrap().get(1).unwrap().as_str();

    let re = Regex::new(r"__flash_page_size = 0x(\w+)").unwrap();
    let fps = re.captures(linker_script).unwrap().get(1).unwrap().as_str();

    (u32::from_str_radix(origin, 16).unwrap(), u32::from_str_radix(fps, 16).unwrap())
}

const SIG_LENGTH: usize = (PROGRAM_SIGNATURE.len() + 4) * size_of::<u32>();

fn main() -> io::Result<()> {
    let args = Args::parse();

    let bootloader = env::var("OPENEMC_BOOTLOADER").unwrap_or_else(|_| "normal".to_string());

    let (mut start, mut page_size) =
        get_flash_params(if bootloader == "big" { MEMORY_BIG_BOOTLOADER } else { MEMORY_NORMAL });
    if let Some(s) = args.start {
        start = s
    }
    if let Some(ps) = args.page_size {
        page_size = ps
    }
    assert!(start % page_size == 0, "flash start is not page aligned");

    let dst = args.dst.unwrap_or_else(|| args.src.with_extension("emc"));
    let id = args.id.unwrap_or_else(rand::random);

    // Copy firmware binary and append signature.
    let mut data = fs::read(&args.src)?;
    let src_len = data.len();
    for s in PROGRAM_SIGNATURE {
        data.write_u32::<LE>(s)?;
    }
    data.write_u32::<LE>(start)?;
    data.write_u32::<LE>(data.len() as u32)?;
    data.write_u32::<LE>(crc32fast::hash(&data))?;
    data.write_u32::<LE>(id)?;
    assert_eq!(data.len(), src_len + SIG_LENGTH);
    assert_eq!(data.len() % size_of::<u32>(), 0);

    // Pad to fill whole flash page.
    while data.len() % page_size as usize != 0 {
        data.write_u32::<LE>(id)?;
    }

    let file_crc32 = crc32fast::hash(&data);

    let mut dst_file = File::create(&dst)?;
    dst_file.write_all(&data)?;
    dst_file.flush()?;

    eprintln!(
        "{} -> {} @ 0x{start:x} % 0x{page_size:x} (id: {id:08x}, CRC32: {file_crc32:08x})",
        args.src.to_string_lossy(),
        dst.to_string_lossy()
    );

    Ok(())
}
