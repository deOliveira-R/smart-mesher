//! `include/smrt.h` and `fortran/smrt.f90` against the crate: exported symbols (acceptance 5),
//! struct sizes and offsets through `static_assert`s compiled by `cc`, Fortran compile and run
//! when `gfortran` exists, and the "no unsafe outside src/ffi" grep (acceptance 9).

mod cutil;
use cutil::*;
use sm_rt_capi::*;
use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::mem::{offset_of, size_of};
use std::path::Path;
use std::process::Command;

fn read(p: impl AsRef<Path>) -> String {
    std::fs::read_to_string(p.as_ref()).unwrap_or_else(|e| panic!("{:?}: {e}", p.as_ref()))
}

fn rust_files(dir: &Path, out: &mut Vec<std::path::PathBuf>) {
    let mut entries: Vec<_> = std::fs::read_dir(dir).expect("dir").map(|e| e.expect("entry").path()).collect();
    entries.sort();
    for p in entries {
        if p.is_dir() {
            rust_files(&p, out);
        } else if p.extension().is_some_and(|e| e == "rs") {
            out.push(p);
        }
    }
}

/// `(name, in_test_hooks)` of every `#[unsafe(no_mangle)] pub ... extern "C" fn` under `src/`.
fn exported_symbols() -> (BTreeSet<String>, BTreeSet<String>) {
    let mut files = Vec::new();
    rust_files(&crate_dir().join("src"), &mut files);
    let (mut public, mut hooks) = (BTreeSet::new(), BTreeSet::new());
    for f in files {
        let text = read(&f);
        let lines: Vec<&str> = text.lines().collect();
        for (i, l) in lines.iter().enumerate() {
            if !l.trim().starts_with("#[unsafe(no_mangle)]") {
                continue;
            }
            let decl = lines[i + 1..].iter().find(|x| x.contains("extern \"C\" fn")).expect("extern fn after no_mangle");
            let name = decl.split("fn ").nth(1).expect("fn name").split('(').next().expect("name").trim().to_string();
            assert!(!text.contains("#[no_mangle]"), "edition-2024 spelling only");
            if name.starts_with("smrt_test_") {
                assert!(f.ends_with("test_hooks.rs"), "{name} must live in test_hooks.rs");
                hooks.insert(name);
            } else {
                public.insert(name);
            }
        }
    }
    (public, hooks)
}

fn strip_c_comments(s: &str) -> String {
    let mut out = String::new();
    let mut rest = s;
    while let Some(i) = rest.find("/*") {
        out.push_str(&rest[..i]);
        rest = &rest[i + 2..];
        rest = rest.split_once("*/").map_or("", |(_, r)| r);
    }
    out.push_str(rest);
    out
}

#[test]
fn header_and_fortran_list_exactly_the_exported_symbols() {
    let (public, hooks) = exported_symbols();
    assert!(public.len() >= 26, "found only {public:?}");
    assert!(hooks.iter().all(|h| h.starts_with("smrt_test_")));

    let header = strip_c_comments(&read(crate_dir().join("include/smrt.h")));
    let mut in_header = BTreeSet::new();
    for l in header.lines() {
        let l = l.trim();
        if l.starts_with("int32_t smrt_") {
            let name = l["int32_t ".len()..].split('(').next().expect("name").to_string();
            assert!(in_header.insert(name.clone()), "{name} declared twice");
        }
    }
    assert_eq!(in_header, public, "include/smrt.h vs exported symbols");

    let f90 = read(crate_dir().join("fortran/smrt.f90")).to_lowercase();
    let mut in_f90 = BTreeSet::new();
    for part in f90.split("bind(c, name=\"").skip(1) {
        in_f90.insert(part.split('"').next().expect("name").to_string());
    }
    assert_eq!(in_f90, public.iter().map(|s| s.to_lowercase()).collect::<BTreeSet<_>>(), "fortran/smrt.f90 vs exported symbols");

    // The test header declares exactly the hooks.
    let th = strip_c_comments(&read(crate_dir().join("tests/c/smrt_test.h")));
    let in_th: BTreeSet<String> = th.lines().filter(|l| l.starts_with("int32_t smrt_test_")).map(|l| l["int32_t ".len()..].split('(').next().expect("n").to_string()).collect();
    assert_eq!(in_th, hooks);

    // Status constants agree across Rust, C and Fortran.
    let consts: [(&str, i32); 17] = [
        ("SMRT_OK", SMRT_OK),
        ("SMRT_NO_HIT", SMRT_NO_HIT),
        ("SMRT_ERR_BUILD", SMRT_ERR_BUILD),
        ("SMRT_ERR_BAD_DIRECTION", SMRT_ERR_BAD_DIRECTION),
        ("SMRT_ERR_NON_FINITE", SMRT_ERR_NON_FINITE),
        ("SMRT_ERR_UNKNOWN_ID", SMRT_ERR_UNKNOWN_ID),
        ("SMRT_ERR_UNSUPPORTED", SMRT_ERR_UNSUPPORTED),
        ("SMRT_ERR_CACHE_STALE", SMRT_ERR_CACHE_STALE),
        ("SMRT_ERR_CACHE_CORRUPT", SMRT_ERR_CACHE_CORRUPT),
        ("SMRT_ERR_GEOM", SMRT_ERR_GEOM),
        ("SMRT_ERR_BAD_ALBEDO", SMRT_ERR_BAD_ALBEDO),
        ("SMRT_ERR_BIN_COUNT", SMRT_ERR_BIN_COUNT),
        ("SMRT_ERR_INVALID_INPUT", SMRT_ERR_INVALID_INPUT),
        ("SMRT_ERR_NULL", SMRT_ERR_NULL),
        ("SMRT_ERR_PANIC", SMRT_ERR_PANIC),
        ("SMRT_ERR_BAD_ARG", SMRT_ERR_BAD_ARG),
        ("SMRT_ERR_LOST", SMRT_ERR_LOST),
    ];
    let raw_header = read(crate_dir().join("include/smrt.h"));
    let raw_f90 = read(crate_dir().join("fortran/smrt.f90"));
    for (name, v) in consts {
        let c = format!("#define {name} {}", if v < 0 { format!("({v})") } else { v.to_string() });
        assert!(raw_header.contains(&c), "{c}");
        let f = format!("{name} = {v}");
        assert!(raw_f90.contains(&f), "{f}");
    }
    let id_consts: [(&str, i64); 3] = [("SMRT_NONE", SMRT_NONE), ("SMRT_COMPLEMENT", SMRT_COMPLEMENT), ("SMRT_MAX_LEVELS", SMRT_MAX_LEVELS as i64)];
    for (name, v) in id_consts {
        assert!(raw_header.contains(&format!("#define {name} ")), "{name}");
        assert!(raw_header.contains(&format!("({v})")) || raw_header.contains(&format!(" {v}")), "{name}");
    }
}

fn layout_c() -> String {
    let mut s = String::from("#include <stddef.h>\n#include \"smrt.h\"\n");
    macro_rules! lay {
        ($t:ident { $($f:ident),* $(,)? }) => {{
            writeln!(s, "_Static_assert(sizeof({0}) == {1}, \"size of {0}\");", stringify!($t), size_of::<$t>()).unwrap();
            $( writeln!(s, "_Static_assert(offsetof({0}, {1}) == {2}, \"offset of {0}.{1}\");", stringify!($t), stringify!($f), offset_of!($t, $f)).unwrap(); )*
        }};
    }
    lay!(SmrtLevel { component, instance, lattice, pad });
    lay!(SmrtState { pos, dir, cell, region, entered_via, levels_total, levels_len, levels });
    lay!(SmrtStep { dist, cell, region, end, action, face, surface, to_region, to_cell, albedo, lost_cause, pad, lost_cell, lost_pos, lost_dir });
    lay!(SmrtLocated { cell, region, on_face, levels_total, levels_len, levels });
    lay!(SmrtClosest { dist, surface, point });
    lay!(SmrtInfo { cells, faces, regions, surfaces, prototypes, copies, memory_bytes, exact_faces, faceted_faces, caps, complement_region, max_events, structure_hash_lo, structure_hash_hi, bounds_lo, bounds_hi, tol_linear, tol_angular });
    lay!(SmrtRegionInfo { original_id, color_id, measure, implicit_complement });
    lay!(SmrtTraceOpts { max_dist, max_steps, stop_at_deck });
    lay!(SmrtRays { x, y, z, u, v, w, start_cell });
    s
}

#[test]
fn struct_sizes_and_offsets_match_the_header() {
    let dir = scratch("layout");
    let src = dir.join("layout.c");
    std::fs::write(&src, layout_c()).expect("write");
    let mut cc = c_compiler();
    cc.args(["-std=c11", "-fsyntax-only", "-Wall", "-Werror"]).arg(format!("-I{}", crate_dir().join("include").display())).arg(&src);
    check_ok("layout static_asserts", &run(&mut cc));
    // The header must also be valid C++ (extern "C" guards).
    let cpp = dir.join("hdr.cpp");
    std::fs::write(&cpp, "#include \"smrt.h\"\nint main() { return smrt_version() == SMRT_ABI_VERSION ? 0 : 1; }\n").expect("write");
    let mut cxx = c_compiler();
    // Reuse the C driver in C++ mode.
    cxx.args(["-x", "c++", "-std=c++17", "-fsyntax-only", "-Wall", "-Werror"]).arg(format!("-I{}", crate_dir().join("include").display())).arg(&cpp);
    let out = run(&mut cxx);
    if String::from_utf8_lossy(&out.stderr).contains("not found") || String::from_utf8_lossy(&out.stderr).contains("cannot execute") {
        eprintln!("SKIPPED C++ header check: no C++ front end ({})", String::from_utf8_lossy(&out.stderr));
    } else {
        check_ok("header as C++", &out);
    }
}

#[test]
fn fortran_module_compiles_and_runs_when_gfortran_exists() {
    let have = Command::new("gfortran").arg("--version").output().is_ok_and(|o| o.status.success());
    if !have {
        eprintln!("SKIPPED fortran sub-test: gfortran is not on PATH");
        return;
    }
    let dir = scratch("fortran");
    let deps = deps_dir();
    let lib = deps.join(cdylib_name());
    let compile = |args: &[&str]| {
        let mut c = Command::new("gfortran");
        c.current_dir(&dir).args(["-Wall", "-std=f2008"]).args(args);
        check_ok(&format!("gfortran {args:?}"), &run(&mut c));
    };
    let module = crate_dir().join("fortran/smrt.f90");
    compile(&["-c", module.to_str().expect("utf8"), "-o", "smrt.o"]);
    let link = |prog: &str, exe: &str, extra: &[&str]| {
        let mut args: Vec<String> = vec![crate_dir().join(prog).display().to_string(), "smrt.o".into(), "-o".into(), exe.into(), lib.display().to_string(), format!("-Wl,-rpath,{}", deps.display())];
        args.extend(extra.iter().map(|s| s.to_string()));
        let refs: Vec<&str> = args.iter().map(String::as_str).collect();
        compile(&refs);
        let out = run(&mut Command::new(dir.join(exe)));
        check_ok(exe, &out);
        String::from_utf8_lossy(&out.stdout).into_owned()
    };
    assert!(link("tests/fortran/version.f90", "version", &[]).contains("smrt ok"));
    if cfg!(feature = "stubs") {
        let defs = [
            format!("-DSIZE_STEP={}", size_of::<SmrtStep>()),
            format!("-DSIZE_STATE={}", size_of::<SmrtState>()),
            format!("-DSIZE_INFO={}", size_of::<SmrtInfo>()),
        ];
        let refs: Vec<&str> = defs.iter().map(String::as_str).collect();
        let mut extra = vec!["-cpp"];
        extra.extend(refs);
        assert!(link("tests/fortran/chord.f90", "chord", &extra).contains("fortran chord ok"));
    }
}

/// Crate source files outside `src/ffi/` must not contain the keyword at all (comments
/// included), so a reviewer can trust a one-line grep. Tests may use it (they call the C ABI).
#[test]
fn no_unsafe_outside_the_ffi_module() {
    let mut files = Vec::new();
    rust_files(&crate_dir().join("src"), &mut files);
    let ffi = crate_dir().join("src/ffi");
    let mut checked = 0;
    for f in files.iter().filter(|f| !f.starts_with(&ffi)) {
        checked += 1;
        for (n, l) in read(f).lines().enumerate() {
            let words: Vec<&str> = l.split(|c: char| !(c.is_alphanumeric() || c == '_')).collect();
            // `deny(unsafe_code)` / `allow(unsafe_code)` are lint names, not the keyword.
            assert!(!words.contains(&"unsafe"), "{}:{}: `unsafe` outside src/ffi", f.display(), n + 1);
        }
    }
    assert!(checked >= 5, "only {checked} files checked");
    // Inside src/ffi every `unsafe {` block is preceded by a SAFETY comment and every
    // `unsafe fn` entry has a `# Safety` doc section.
    let mut ffi_files = Vec::new();
    rust_files(&ffi, &mut ffi_files);
    for f in ffi_files {
        let text = read(&f);
        let lines: Vec<&str> = text.lines().collect();
        for (i, l) in lines.iter().enumerate() {
            let t = l.trim();
            let is_block = t.contains("unsafe {") && !t.starts_with("//") && !t.starts_with("///");
            if is_block {
                let window = lines[i.saturating_sub(4)..=i].join("\n");
                assert!(window.contains("SAFETY"), "{}:{}: unsafe block without SAFETY comment", f.display(), i + 1);
            }
            if t.contains("unsafe extern \"C\" fn") || t.starts_with("pub(crate) unsafe fn") || t.starts_with("unsafe fn") {
                let docs: String = lines[..i].iter().rev().take_while(|x| x.trim().starts_with("///") || x.trim().starts_with("#[")).copied().collect::<Vec<_>>().join("\n");
                assert!(docs.contains("# Safety"), "{}:{}: unsafe fn without `# Safety` docs", f.display(), i + 1);
            }
        }
    }
}
