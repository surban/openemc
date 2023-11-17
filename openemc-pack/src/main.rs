//! Pack firmware in format bootable by OpenEMC bootloader.

use byteorder::{WriteBytesExt, LE};
use clap::Parser;
use regex::Regex;
use std::{env, fs, fs::File, io, io::Write, mem::size_of, num::ParseIntError, path::PathBuf};

use openemc_shared::{boot::PROGRAM_SIGNATURE, flash};

/// Pack firmware in format bootable by EMC bootloader.
#[derive(Parser, Debug)]
struct Args {
    /// Override start address of user flash.
    #[clap(long, value_parser=parse_maybe_hex)]
    origin: Option<u32>,
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

static MEMORY_BIG_BOOTLOADER: &str = include_str!("../../openemc-bootloader/emc-bootloader-big.x");
static MEMORY_NORMAL_BOOTLOADER: &str = include_str!("../../openemc-bootloader/emc-bootloader-normal.x");

struct FlashParams {
    pub bootloader_max_size: u32,
}

impl FlashParams {
    pub fn from_linker_script(linker_script: &str) -> Self {
        let re = Regex::new(r"__bootloader_max_size = (\w+)K").unwrap();
        let cap = re.captures(linker_script).expect("cannot parse linker script");
        let bootloader_max_size_kb = cap.get(1).unwrap().as_str();

        Self { bootloader_max_size: bootloader_max_size_kb.parse::<u32>().unwrap() * 1024 }
    }

    pub fn user_program_start(&self) -> u32 {
        (flash::START as u32) + self.bootloader_max_size
    }
}

const SIG_LENGTH: usize = (PROGRAM_SIGNATURE.len() + 4) * size_of::<u32>();

fn main() -> io::Result<()> {
    let args = Args::parse();

    let user_program_start = match args.origin {
        Some(origin) => origin,
        None => {
            let bootloader = env::var("OPENEMC_BOOTLOADER").unwrap_or_else(|_| "normal".to_string());
            let flash = FlashParams::from_linker_script(if bootloader == "big" {
                MEMORY_BIG_BOOTLOADER
            } else {
                MEMORY_NORMAL_BOOTLOADER
            });
            flash.user_program_start()
        }
    };

    let dst = args.dst.unwrap_or_else(|| args.src.with_extension("emc"));
    let id = args.id.unwrap_or_else(rand::random);

    // Copy firmware binary and append signature.
    let mut data = fs::read(&args.src)?;
    let src_len = data.len();
    for s in PROGRAM_SIGNATURE {
        data.write_u32::<LE>(s)?;
    }
    data.write_u32::<LE>(user_program_start)?;
    data.write_u32::<LE>(data.len() as u32)?;
    data.write_u32::<LE>(crc32fast::hash(&data))?;
    data.write_u32::<LE>(id)?;
    assert_eq!(data.len(), src_len + SIG_LENGTH);
    assert_eq!(data.len() % size_of::<u32>(), 0);

    let file_crc32 = crc32fast::hash(&data);

    let mut dst_file = File::create(&dst)?;
    dst_file.write_all(&data)?;
    dst_file.flush()?;

    eprintln!(
        "{} -> {} @ 0x{:x} (id: {id:08x}, CRC32: {file_crc32:08x}, size: {} bytes)",
        args.src.to_string_lossy(),
        dst.to_string_lossy(),
        user_program_start,
        data.len()
    );

    Ok(())
}
