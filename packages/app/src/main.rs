/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
28.06.26, 15:02
*/

//! ELEKTRON inventory system (storage)

#[cfg(feature = "server")]
mod app_server;

#[cfg(feature = "server")]
#[tokio::main]
async fn main() -> std::process::ExitCode {
    crate::app_server::entry().await
}


// for now, all client targets just use the same UI
// and entrypoint, but this may be split further in the future

#[cfg(not(feature = "server"))]
mod app_client;

#[cfg(not(feature = "server"))]
fn main() {
    crate::app_client::entry()
}
