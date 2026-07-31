/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
28.06.26, 15:02
*/

//! ELEKTRON inventory system (storage) server

mod globals;
pub use globals::AppState;

mod args;
pub use args::Args;

mod repl;
pub use repl::Repl;

pub mod webserver;
pub use webserver::WebServer;
