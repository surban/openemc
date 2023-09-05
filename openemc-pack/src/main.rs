//! Pack firmware in format bootable by OpenEMC bootloader.

use byteorder::{WriteBytesExt, LE};
use clap::Parser;
use regex::Regex;
use std::{env, fs, fs::File, io, io::Write, mem::size_of, num::ParseIntError, path::PathBuf};

use openemc_shared::{CFG_FLASH_PAGES, PROGRAM_SIGNATURE};

/// Pack firmware in format bootable by EMC bootloader.
#[derive(Parser, Debug)]
struct Args {
    /// Override start address of user flash.
    #[clap(long, value_parser=parse_maybe_hex)]
    origin: Option<u32>,
    /// Override flash length.
    #[clap(long, value_parser=parse_maybe_hex)]
    length: Option<u32>,
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

struct FlashParams {
    pub origin: u32,
    pub length: u32,
    pub page_size: u32,
}

impl FlashParams {
    pub fn from_linker_script(linker_script: &str) -> Self {
        let re = Regex::new(r"FLASH : ORIGIN = 0x(\w+), LENGTH = 0x(\w+)").unwrap();
        let cap = re.captures(linker_script).expect("cannot parse linker script");
        let origin = cap.get(1).unwrap().as_str();
        let length = cap.get(2).unwrap().as_str();

        let re = Regex::new(r"__flash_page_size = 0x(\w+)").unwrap();
        let fps = re.captures(linker_script).expect("cannot parse linker script").get(1).unwrap().as_str();

        Self {
            origin: u32::from_str_radix(origin, 16).unwrap(),
            length: u32::from_str_radix(length, 16).unwrap(),
            page_size: u32::from_str_radix(fps, 16).unwrap(),
        }
    }
}

const SIG_LENGTH: usize = (PROGRAM_SIGNATURE.len() + 4) * size_of::<u32>();

fn main() -> io::Result<()> {
    let args = Args::parse();

    let bootloader = env::var("OPENEMC_BOOTLOADER").unwrap_or_else(|_| "normal".to_string());

    let mut flash =
        FlashParams::from_linker_script(if bootloader == "big" { MEMORY_BIG_BOOTLOADER } else { MEMORY_NORMAL });
    if let Some(o) = args.origin {
        flash.origin = o;
    }
    if let Some(l) = args.length {
        flash.length = l;
    }
    if let Some(ps) = args.page_size {
        flash.page_size = ps;
    }
    assert!(flash.origin % flash.page_size == 0, "flash start is not page aligned");
    assert!(flash.length % flash.page_size == 0, "flash length is not page aligned");

    let dst = args.dst.unwrap_or_else(|| args.src.with_extension("emc"));
    let id = args.id.unwrap_or_else(rand::random);

    // Copy firmware binary and append signature.
    let mut data = fs::read(&args.src)?;
    let src_len = data.len();
    for s in PROGRAM_SIGNATURE {
        data.write_u32::<LE>(s)?;
    }
    data.write_u32::<LE>(flash.origin)?;
    data.write_u32::<LE>(data.len() as u32)?;
    data.write_u32::<LE>(crc32fast::hash(&data))?;
    data.write_u32::<LE>(id)?;
    assert_eq!(data.len(), src_len + SIG_LENGTH);
    assert_eq!(data.len() % size_of::<u32>(), 0);

    // Pad to fill whole flash page.
    let mut padding_size = 0;
    while data.len() % flash.page_size as usize != 0 {
        data.write_u32::<LE>(id)?;
        padding_size += 1;
    }

    // Verify length.
    let Some(avail) = (flash.length as usize).checked_sub(data.len()) else {
        panic!("program too big for flash ({} / {} kB)", data.len() / 1024, flash.length / 1024)
    };
    let cfg_size = 2 * CFG_FLASH_PAGES * flash.page_size as usize;
    if avail < cfg_size {
        panic!(
            "no flash space available for configuration (program size: {} / {} kB)",
            data.len() / 1024,
            flash.length / 1024
        );
    }

    let file_crc32 = crc32fast::hash(&data);

    let mut dst_file = File::create(&dst)?;
    dst_file.write_all(&data)?;
    dst_file.flush()?;

    eprintln!(
        "{} -> {} @ 0x{:x} % 0x{:x} (id: {id:08x}, CRC32: {file_crc32:08x}, size: {} / {} bytes)",
        args.src.to_string_lossy(),
        dst.to_string_lossy(),
        flash.origin,
        flash.page_size,
        data.len() - padding_size + cfg_size,
        flash.length,
    );

    Ok(())
}
