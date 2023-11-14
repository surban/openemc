//! OpenEMC build tools.

use std::{
    collections::HashMap,
    fs::File,
    io::{BufRead, BufReader, Result},
    path::Path,
};

/// Reads OpenEMC settings from comments in a source file.
///
/// A setting line has the form: `// OPENEMC-SETTING: value`
pub fn read_settings(path: &Path) -> Result<HashMap<String, String>> {
    let file = File::open(path)?;
    let lines = BufReader::new(file);

    let mut settings = HashMap::new();
    for line in lines.lines() {
        let line = line?;
        let line = line.trim();
        if let Some(line) = line.strip_prefix("// OPENEMC-") {
            if let Some((name, value)) = line.split_once(':') {
                settings.insert(name.trim().to_ascii_uppercase().to_string(), value.trim().to_string());
            }
        }
    }

    Ok(settings)
}
