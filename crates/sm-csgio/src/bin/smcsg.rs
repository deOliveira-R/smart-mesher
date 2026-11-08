//! `smcsg`: check and inspect CSG decks. See `sm_csgio::cli` for the commands and exit codes.

use sm_csgio::cli::{LoadRequest, run};
use sm_csgio::{CsgIr, ImportError, openmc};
use std::path::Path;

/// Inputs above this are refused before reading.
const MAX_BYTES: u64 = 256 << 20;

fn read(p: &Path) -> Result<String, ImportError> {
    let len = std::fs::metadata(p).map_err(|e| ImportError::Invalid(format!("{}: {e}", p.display())))?.len();
    if len > MAX_BYTES {
        return Err(ImportError::Invalid(format!("{}: {len} bytes is over the {MAX_BYTES} byte input limit", p.display())));
    }
    std::fs::read_to_string(p).map_err(|e| ImportError::Invalid(format!("{}: {e}", p.display())))
}

fn load(req: &LoadRequest<'_>) -> Result<CsgIr, ImportError> {
    match req.format {
        "openmc" => {
            let geometry = read(req.deck)?;
            // materials.xml next to the deck is used when `--materials` is not given.
            let sibling = req.deck.with_file_name("materials.xml");
            let mpath = req.materials.map(Path::to_path_buf).or_else(|| sibling.exists().then_some(sibling));
            let materials = mpath.as_deref().map(read).transpose()?;
            openmc::import(&geometry, materials.as_deref())
        }
        other => Err(ImportError::Unsupported { line: None, what: format!("format {other:?}") }),
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let code = run(&args, &load, &mut std::io::stdout(), &mut std::io::stderr());
    std::process::exit(code);
}
