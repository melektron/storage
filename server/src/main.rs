/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
28.06.26, 15:02
*/

//! ELEKTRON inventory system (storage) server

mod app;
mod args;
mod repl;
mod webserver;
mod api;

use std::process::ExitCode;

use anyhow::Context;
use clap::Parser;
use el_std::terminal::{TerminalOpts, setup_terminal};

use crate::{app::App, args::Args};

#[tokio::main]
async fn main() -> ExitCode {
    // parse commandline arguments
    let args = Args::parse();

    let repl_io = setup_terminal(TerminalOpts {
        banner: Some("== Inventory Storage Server starting ==".to_owned()),
        ..Default::default()
    });

    // app may fail during initialization (maybe this should be restructured in the future)
    let app = match App::new(args, repl_io)
        .await
        .context("Application initialization failed")
    {
        Ok(app) => app,
        Err(e) => {
            eprintln!("\n{e:?}"); // logging via error! may not work anymore here due to repl shutdown
            return ExitCode::FAILURE;
        }
    };

    if let Err(e) = app
        .run()
        .await
        .context("Application core terminated with error")
    {
        // dropping the app here will clean up any state
        // of the repl, therefore resetting tty mode,
        // causing normal stdout printing to work correctly again.
        // Without this, \n doesn't imply \r.
        drop(app);
        // print global application error manually rather
        // than default std::process::Termination output gives more detail
        eprintln!("\n{e:?}"); // logging via error! may not work anymore here due to repl shutdown
        return ExitCode::FAILURE;
    }

    drop(app);
    println!("\nExiting..."); // logging via info! may not work anymore here due to repl shutdown
    return ExitCode::SUCCESS;
}
