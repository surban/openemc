//
// OpenEMC firmware for embedded controllers
// Copyright (C) 2022-2024 Sebastian Urban <surban@surban.net>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

//! OpenEMC logger.

use anyhow::{bail, Context, Result};
use clap::Parser;
use defmt_decoder::{DecodeError, Frame, Location, Table};
use defmt_parser::Level as DefmtLevel;
use env_logger::Target;
use flate2::read::GzDecoder;
use log::{Level, LevelFilter};
use nix::{
    errno::Errno,
    poll::{poll, PollFd, PollFlags},
};
use std::{
    ffi::{c_int, OsString},
    fmt::Write,
    fs::{self, File},
    io::{ErrorKind, Read, Seek},
    os::{fd::AsFd, unix::prelude::OsStringExt},
    path::{Path, PathBuf},
    sync::atomic::{AtomicBool, Ordering},
    thread::sleep,
    time::Duration,
};

mod syslog;

const READ_BUFFER_SIZE: usize = 128;
const UNPROCESSED_SIZE: usize = 16_777_216;
const POLL_TIMEOUT: c_int = 60_000;
const FIRMWARE_DIR: &str = "/lib/firmware";

static STOP: AtomicBool = AtomicBool::new(false);

/// OpenEMC logger.
#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Opts {
    /// Read bootloader log.
    #[arg(short, long)]
    bootloader: bool,
    /// Only process buffered log messages and then exit.
    #[arg(short, long)]
    oneshot: bool,
    /// Log level.
    #[arg(short, long, default_value_t = LevelFilter::Trace)]
    level: LevelFilter,
    /// Syslog identifier.
    #[arg(short, long)]
    ident: Option<String>,
    /// Include file path and line numbers in log messages.
    #[arg(short = 'n', long)]
    lines: bool,
    /// Log to standard output instead of syslog.
    #[arg(short, long)]
    foreground: bool,
    /// Show malformed log messages.
    #[arg(long)]
    malformed: bool,
    /// Gzipped ELF image of OpenEMC firmware.
    firmware_elf: Option<PathBuf>,
    /// OpenEMC log file or device.
    openemc_log: Option<PathBuf>,

    /// Read log from memory of device connected via debug probe.
    #[cfg(feature = "probe")]
    #[arg(short, long)]
    probe: bool,
}

/// Log message for Rust logger.
#[derive(Debug, Clone, PartialEq, Eq)]
struct LogMsg {
    /// Log level.
    pub level: Level,
    /// Message.
    pub msg: String,
}

impl LogMsg {
    /// Format defmt log message for Rust logger.
    pub fn from_defmt(frame: &Frame, loc: Option<&Location>, lines: bool) -> Self {
        let mut msg = String::with_capacity(256);

        if let Some(loc) = loc {
            write!(msg, "[{}", &loc.module).unwrap();
            if lines {
                write!(msg, " @ {}:{}", loc.file.display(), loc.line).unwrap();
            }
            write!(msg, "] ").unwrap();
        }

        write!(msg, "{}", frame.display_message()).unwrap();

        let level = match frame.level() {
            Some(DefmtLevel::Error) => Level::Error,
            Some(DefmtLevel::Warn) => Level::Warn,
            Some(DefmtLevel::Info) => Level::Info,
            Some(DefmtLevel::Debug) => Level::Debug,
            Some(DefmtLevel::Trace) => Level::Trace,
            None => Level::Info,
        };

        Self { level, msg }
    }

    /// Log the message to the Rust logger.
    pub fn log(&self) {
        log::log!(self.level, "{}", &self.msg);
    }
}

/// Gets the log file or log device path.
fn log_path(opts: &Opts) -> Result<PathBuf> {
    let openemc_log = match &opts.openemc_log {
        Some(openemc_log) => openemc_log.clone(),
        None => {
            let filename = if opts.bootloader { "openemc_bootloader_log" } else { "openemc_log" };
            let driver = fs::read_dir("/sys/module/openemc/drivers")
                .context("OpenEMC module not loaded")?
                .next()
                .context("OpenEMC driver not available")??;
            fs::read_dir(driver.path())?
                .filter_map(|e| e.ok())
                .find_map(|e| {
                    let p = e.path().join(filename);
                    p.is_file().then_some(p)
                })
                .context("OpenEMC log not available")?
        }
    };
    Ok(openemc_log.canonicalize()?)
}

/// Gets the path to the firmware ELF file.
fn firmware_elf_path(opts: &Opts, openemc_log: Option<&Path>) -> Result<PathBuf> {
    let firmware_elf = match (&opts.firmware_elf, openemc_log) {
        (Some(firmware_elf), _) => firmware_elf.clone(),

        (None, Some(openemc_log)) if opts.bootloader => {
            let firmware_bin = PathBuf::from(OsString::from_vec(
                fs::read(openemc_log.with_file_name("openemc_firmware"))
                    .context("OpenEMC firmware location not available")?,
            ));
            let firmware_dir = match firmware_bin.parent() {
                Some(dir) if firmware_bin.has_root() => dir,
                _ => Path::new(FIRMWARE_DIR),
            };

            let bootloader_crc32 = fs::read_to_string(openemc_log.with_file_name("openemc_bootloader_crc32"))
                .context("OpenEMC bootloader CRC32 not available")?;

            let bootloader_elf = firmware_dir.join(format!("openemc_bootloader_{bootloader_crc32}.elf.gz"));
            if !bootloader_elf.is_file() {
                bail!("OpenEMC bootloader ELF not found");
            }
            bootloader_elf
        }

        (None, Some(openemc_log)) => {
            let mut firmware_bin = PathBuf::from(OsString::from_vec(
                fs::read(openemc_log.with_file_name("openemc_firmware"))
                    .context("OpenEMC firmware location not available")?,
            ));
            if !firmware_bin.has_root() {
                firmware_bin = Path::new(FIRMWARE_DIR).join(firmware_bin);
            }

            let firmware_dir = firmware_bin.parent().context("no firmware directory")?;
            let mut firmware = firmware_bin.file_stem().context("unrecognized firmware name")?.to_os_string();

            loop {
                if firmware.is_empty() {
                    bail!("OpenEMC firmware ELF not found");
                }

                let firmware_elf = firmware_dir.join(&firmware).with_extension("elf.gz");
                if firmware_elf.is_file() {
                    break firmware_elf;
                }

                let firmware_str = firmware.to_string_lossy();
                let mut chars = firmware_str.chars();
                chars.next_back();
                firmware = chars.as_str().into();
            }
        }

        (None, None) => bail!("firmware ELF path must be specified"),
    };

    Ok(firmware_elf.canonicalize()?)
}

/// Read from device memory via debug probe.
#[cfg(feature = "probe")]
fn read_device_memory(addr: u64, data: &mut [u8]) -> Result<()> {
    use probe_rs::{Lister, MemoryInterface, Permissions};

    let lister = Lister::new();
    let probes = lister.list_all();
    let probe =
        probes.first().context("no debug probe found")?.open(&lister).context("cannot open debug probe")?;
    let mut session =
        probe.attach_under_reset("STM32F103RB", Permissions::default()).context("cannot connect to target")?;
    let mut core = session.core(0).context("cannot connect to core")?;
    core.read(addr, data)?;

    Ok(())
}

fn perform(opts: &Opts) -> Result<bool> {
    #[cfg(feature = "probe")]
    let probe = opts.probe;
    #[cfg(not(feature = "probe"))]
    let probe = false;

    let openemc_log;
    let firmware_elf;
    if probe {
        #[cfg(feature = "probe")]
        {
            openemc_log = None;
            firmware_elf = firmware_elf_path(opts, None).context("cannot determine firmware ELF path")?;
        }

        #[cfg(not(feature = "probe"))]
        {
            unreachable!()
        }
    } else {
        openemc_log = Some(log_path(opts).context("cannot determine log path")?);
        firmware_elf =
            firmware_elf_path(opts, openemc_log.as_deref()).context("cannot determine firmware ELF path")?;
    }

    match &openemc_log {
        Some(openemc_log) => {
            log::info!(
                "======== logging from {} using firmware image {} ========",
                openemc_log.display(),
                firmware_elf.display()
            );
        }
        None => {
            log::info!("======== logging from probe using firmware image {} ========", firmware_elf.display());
        }
    }

    let table;
    let locs;
    #[cfg(feature = "probe")]
    let log_sect_addr;
    #[cfg(feature = "probe")]
    let log_sect_size;
    {
        let mut bytes = Vec::new();
        let mut file = GzDecoder::new(File::open(&firmware_elf).context("cannot open firmware")?);
        file.read_to_end(&mut bytes).context("cannot read firmware")?;

        table = Table::parse(&bytes)?.context(".defmt data not found")?;
        locs = table.get_locations(&bytes)?;

        #[cfg(feature = "probe")]
        {
            use object::{Object, ObjectSection};
            let elf = object::File::parse(&*bytes).context("cannot parse firmware")?;
            let section_name = if opts.bootloader { ".defmt_boot_log" } else { ".defmt_log" };
            let section = elf.section_by_name(section_name).context("log section not found")?;
            log_sect_addr = section.address();
            log_sect_size = section.size();
            let which = if opts.bootloader { "bootloader " } else { "" };
            log::debug!("======== {which}log is at 0x{log_sect_addr:x} and has size {log_sect_size} ========");
        }
    }

    let locs = if table.indices().all(|idx| locs.contains_key(&(idx as u64))) {
        Some(locs)
    } else {
        log::warn!("location info is incomplete; it will be omitted from the output");
        None
    };

    let mut log;
    let is_dev;
    match &openemc_log {
        Some(openemc_log) => {
            log = File::open(openemc_log).context("cannot open OpenEMC log")?;
            is_dev = fs::canonicalize(openemc_log)?.starts_with("/sys");
        }
        None => {
            #[cfg(feature = "probe")]
            {
                log = tempfile::tempfile()?;
                is_dev = false;

                let mut buf = vec![0; log_sect_size.try_into()?];
                read_device_memory(log_sect_addr, &mut buf).context("cannot read log from device memory")?;

                let (data, overwritten) = defmt_ringbuf_offline::read::<defmt_ringbuf_offline::LE>(&buf)
                    .context("cannot parse log ring buffer")?;
                std::io::Write::write_all(&mut log, &data)?;
                log.rewind()?;

                if overwritten {
                    log::warn!("======== log data was lost ========");
                }
            }

            #[cfg(not(feature = "probe"))]
            {
                unreachable!()
            }
        }
    }

    let mut buf = [0; READ_BUFFER_SIZE];
    let mut stream_decoder = table.new_stream_decoder();
    let mut unprocessed = 0;

    while !STOP.load(Ordering::SeqCst) {
        // Read from start of device file.
        if is_dev {
            log.rewind()?;
        }

        // Read data.
        match log.read(&mut buf) {
            Ok(0) if unprocessed > 0 => (),
            Ok(0) if is_dev && !opts.oneshot && !opts.bootloader => {
                // Wait for data to arrive.
                let log_fd = log.as_fd();
                let poll_fd = PollFd::new(&log_fd, PollFlags::POLLPRI | PollFlags::POLLERR);
                match poll(&mut [poll_fd], POLL_TIMEOUT) {
                    Ok(_) => (),
                    Err(Errno::EINTR) => sleep(Duration::from_millis(100)),
                    Err(err) => return Err(err.into()),
                }
            }
            Ok(0) => return Ok(true),
            Ok(n) => {
                stream_decoder.received(&buf[..n]);
                unprocessed += n;
                if unprocessed < UNPROCESSED_SIZE {
                    continue;
                }
            }
            Err(err) if is_dev && err.kind() == ErrorKind::BrokenPipe => {
                log::warn!("======== log data was lost ========");
                stream_decoder = table.new_stream_decoder();
            }
            Err(err) if err.kind() == ErrorKind::Interrupted => continue,
            Err(err) => return Err(err.into()),
        }

        // Decode the received data.
        let mut log_msgs = Vec::new();
        loop {
            match stream_decoder.decode() {
                Ok(frame) => {
                    let loc = locs.as_ref().and_then(|locs| locs.get(&frame.index()));
                    let log_msg = LogMsg::from_defmt(&frame, loc, opts.lines);
                    log_msgs.push(log_msg);
                }
                Err(DecodeError::UnexpectedEof) => break,
                Err(DecodeError::Malformed) => {
                    log::warn!("======== malformed log data ========");
                    if !table.encoding().can_recover() {
                        return Err(DecodeError::Malformed.into());
                    }
                    if !opts.malformed {
                        log_msgs.clear();
                    }
                }
            }
        }
        unprocessed = 0;

        // Log to system log.
        for log_msg in log_msgs {
            log_msg.log();
        }
    }

    Ok(false)
}

fn main() -> Result<()> {
    ctrlc::set_handler(|| STOP.store(true, Ordering::SeqCst))?;

    #[allow(unused_mut)]
    let mut opts = Opts::parse();

    #[cfg(feature = "probe")]
    if opts.probe {
        opts.foreground = true;
    }

    // Initialize logger.
    if opts.foreground {
        env_logger::builder()
            .target(Target::Stdout)
            .filter_level(opts.level)
            .filter_module("probe_rs", LevelFilter::Warn)
            .filter_module("jaylink", LevelFilter::Warn)
            .filter_module("tracing", LevelFilter::Warn)
            .init();
    } else {
        let ident =
            opts.ident.as_deref().unwrap_or(if opts.bootloader { "OpenEMC-bootloader" } else { "OpenEMC" });
        log::set_boxed_logger(Box::new(syslog::Syslog::new(ident)))?;
        log::set_max_level(LevelFilter::Trace);
    }

    let res = perform(&opts);

    match &res {
        Ok(finished) => log::info!("======== log {} ========", if *finished { "finished" } else { "stopped" }),
        Err(err) => log::error!("======== logging failed: {err} ========"),
    }
    res?;

    Ok(())
}
