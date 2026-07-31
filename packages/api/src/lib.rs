//! This crate contains the API used by the frontend
//! to access the backend. The frontend can only
//! access the backend by calling a function from this crate,
//! as it does not have any other dependency on the backend.
//!
//! On the server, the API functions have access to the `ApiContext`
//! provided by `storage_core::webserver`. Any server-internal
//! state required by API handlers is to be added to this context.
use dioxus::prelude::*;

/// server imports
#[cfg(feature = "server")]
mod srv {
    pub use axum_client_ip::ClientIp;
    pub use dioxus::fullstack::extract::State;
    pub use storage_core::webserver::ApiContext;
}

/// Echo the user input on the server.
#[post("/api/echo")]
pub async fn echo(input: String) -> Result<String, ServerFnError> {
    println!("Message from server");
    Ok(input)
}
