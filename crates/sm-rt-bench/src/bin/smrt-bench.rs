//! `smrt-bench`: the benchmark runner (see the crate docs and `DESIGN.md`).

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match sm_rt_bench::cli::main_with(&args) {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("smrt-bench: {e}");
            std::process::exit(2);
        }
    }
}
