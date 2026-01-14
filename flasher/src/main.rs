mod convert;
mod flash;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "flasher")]
#[command(about = "STM32 flasher utility", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Flash firmware using probe-rs
    Flash(flash::Cmd),
    /// Convert ST-Link to eprobe using ST's bootloader
    Convert(convert::Cmd),
}

fn main() {
    pretty_env_logger::init();

    let cli = Cli::parse();

    match cli.command {
        Commands::Flash(ref cmd) => flash::run(cmd),
        Commands::Convert(ref cmd) => convert::run(cmd),
    }
}
