//! This crate contains all shared fullstack server functions.
use dioxus::prelude::*;

/// server imports
#[cfg(feature = "server")]
mod srv {
    pub use axum_client_ip::ClientIp;
    pub use dioxus::fullstack::extract::State;
    pub use storage_core::webserver::WebServerContext;
}

#[get("/api/health", state: srv::State<srv::WebServerContext>, srv::ClientIp(ip): srv::ClientIp)]
pub async fn health_check() -> Result<String, ServerFnError> {
    let db_status = match state.db.ping().await {
        Ok(()) => "healthy".to_string(),
        Err(e) => format!("{e}"),
    };

    Ok(format!(
        "
        Request source: {ip}
        Webserver: healthy
        Database: {db_status}
        "
    ))
}

/// Echo the user input on the server.
#[post("/api/echo")]
pub async fn echo(input: String) -> Result<String, ServerFnError> {
    println!("Message from server");
    Ok(input)
}
