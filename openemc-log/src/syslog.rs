//! Syslog.

use log::Level;
use std::ffi::CString;

/// Syslog forwarder.
pub struct Syslog {
    _ident: CString,
}

impl Syslog {
    /// Creates a new syslog forwarder with the specified program identifier.
    pub fn new(ident: &str) -> Self {
        let ident = CString::new(ident).unwrap();
        unsafe { libc::openlog(ident.as_ptr(), libc::LOG_CONS, libc::LOG_DAEMON) };

        Self { _ident: ident }
    }
}

impl log::Log for Syslog {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        let priority = match record.level() {
            Level::Error => libc::LOG_ERR,
            Level::Warn => libc::LOG_WARNING,
            Level::Info => libc::LOG_INFO,
            Level::Debug => libc::LOG_DEBUG,
            Level::Trace => libc::LOG_DEBUG,
        };
        let msg = record.args().to_string();

        let cmsg = CString::new(msg).unwrap_or_else(|_| CString::new("<null>").unwrap());
        unsafe { libc::syslog(priority | libc::LOG_DAEMON, b"%s\0".as_ptr() as *const _, cmsg.as_ptr()) }
    }

    fn flush(&self) {}
}

impl Drop for Syslog {
    fn drop(&mut self) {
        unsafe { libc::closelog() };
    }
}
