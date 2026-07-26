/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
28.06.26, 15:04
*/

//! Command line argument definitions



use std::{net::IpAddr, path::PathBuf};

use axum_client_ip::ClientIpSource;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
pub struct Args {
    /// Data folder where persistent files are stored.
    #[arg(short, long, default_value = "./data")]
    pub data: PathBuf,

    ///// Config file location
    //#[arg(short, long)]
    //config: PathBuf
    
    /// Force enabling the repl, even in non-interactive environments.
    /// (may lead to unexpected errors)
    #[arg(long="force-repl")]
    pub force_repl: Option<bool>,

    /// Source of client IP address. By default the socket IP address,
    /// but usage of proxy headers can be selected instead.
    #[arg(long, default_value = "ConnectInfo")]
    pub ip_source: ClientIpSource,

    /// Address to listen on for HTTP connections
    #[arg(long, default_value = "0.0.0.0")]
    pub listen_addr: IpAddr,
    /// Port to listen on for HTTP connections
    #[arg(long, default_value = "1234")]
    pub listen_port: u16,

    #[cfg(feature = "dev-proxy")]
    /// URL of dev server to route UI requests to 
    /// during development (this only exists on dev builds).
    /// HTTPS is not supported.
    #[arg(long, default_value = "http://localhost:5173")]
    pub dev_server_url: String,
}