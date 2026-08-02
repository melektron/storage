/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
08.03.26, 17:10

Subsystem for handling web/websocket requests
*/

use std::{net::SocketAddr, ops::Deref, sync::Arc};

use anyhow::{Context, Result};
use axum::{Extension, Router, routing::get};
use dioxus::{
    fullstack::FullstackContext,
    server::{DioxusRouterExt, ServeConfig},
};
use log::info;
use sea_orm::DatabaseConnection;
use tokio::net::TcpListener;

use crate::globals::AppState;

pub struct WebServer {
    // These fields are public so the API crate can access them
    pub app_state: Arc<AppState>,
    pub db: DatabaseConnection,
}

/// Context type to access webserver instance as
/// State from axum handlers. This behaves
/// equivalent to Arc<WebServer>, but we can't implement
/// FromRef on Arc<>, so a wrapper type is necessary.
#[derive(Clone)]
pub struct ApiContext {
    webserver: Arc<WebServer>,
}
impl Deref for ApiContext {
    type Target = Arc<WebServer>;

    fn deref(&self) -> &Self::Target {
        &self.webserver
    }
}
impl From<Arc<WebServer>> for ApiContext {
    fn from(value: Arc<WebServer>) -> Self {
        ApiContext {
            webserver: value.clone(),
        }
    }
}

// https://docs.rs/axum/0.8.9/axum/extract/struct.State.html#substates
impl axum::extract::FromRef<FullstackContext> for ApiContext {
    fn from_ref(state: &FullstackContext) -> Self {
        state
            .extension::<ApiContext>()
            .expect("Arc<WebServer> extension missing, cannot derive SubState.")
    }
}

#[cfg(feature = "dev-proxy")]
mod dev_server_proxy {
    //! For development, it's possible to use the server
    //! as a reverse proxy to the UI devserver. This way
    //! a reverse proxy is not required for local development.
    //! This is not needed for dioxus, as the dx dev environment
    //! acts as the proxy, forwarding any requests to server.

    use axum::Router;
    use axum_reverse_proxy::ReverseProxy;

    use crate::globals::AppState;

    /// adds dev server proxy routes
    pub fn apply(app_state: &AppState, router: Router) -> Router {
        let proxy = ReverseProxy::new("/", &app_state.args.dev_server_url.as_str());

        router.merge(proxy)
    }
}

mod routes {
    use axum::Extension;
    use axum_client_ip::ClientIp;

    /// Returns a human readable server health status page.
    pub async fn health_check(
        state: Extension<super::ApiContext>,
        ClientIp(ip): ClientIp
    ) -> String {
        let db_status = match state.db.ping().await {
            Ok(()) => "healthy".to_string(),
            Err(e) => format!("{e}"),
        };

        format!(
            "
            Request source: {ip}
            Webserver: healthy
            Database: {db_status}
            "
        )
    }
}

impl WebServer {
    pub fn new(app_state: &Arc<AppState>, db: &DatabaseConnection) -> Self {
        Self {
            app_state: app_state.clone(),
            db: db.clone(),
        }
    }

    pub async fn run(
        self: Arc<Self>,
        main_component: fn() -> dioxus::core::Element, /*impl ComponentFunction<()> + Send + Sync*/
    ) -> Result<()> {
        // If Dioxus CLI provides an addr/port combination (IP and PORT env vars),
        // we use those to listen, otherwise we use the ports from the server arguments.
        let ip = dioxus::cli_config::server_ip().unwrap_or(self.app_state.args.listen_addr);
        let port = dioxus::cli_config::server_port().unwrap_or(self.app_state.args.listen_port);
        let address = SocketAddr::new(ip, port);
        info!("HTTP server listening on: {address}");

        let listener = TcpListener::bind(address)
            .await
            .context("Binding TCP listener for HTTP")?;

        let router = Router::new() // or dioxus::server::router(main_component)
            // https://docs.rs/crate/dioxus-server/0.7.9
            .serve_dioxus_application(ServeConfig::new(), main_component)
            // add handlers for special subsystems manually
            .route("/health", get(routes::health_check))
            // Dioxus internally uses it's own FullstackState as the axum state, so we can't attach
            // our own state for our handlers. Instead we have to use an extension, which is not type-safe.
            // For server functions, we have implemented `axum::extract::FromRef` on the ApiContext type,
            // so they can extract it as a "sub-state" using the type-safe state extractor.
            // This doesn't work for non-server-function routes though unfortunately, so they need
            // to use the extension directly.
            // help-chat: https://discord.com/channels/899851952891002890/943190605067079712/1441581237880754400
            // internals: https://discord.com/channels/899851952891002890/928812591126569000/1432511520658555114
            // example: https://github.com/DioxusLabs/dioxus/blob/d0a5f89ae9df2dc74d7fbcbdb9324490a2c7048e/examples/07-fullstack/server_state.rs#L96
            .layer(Extension(ApiContext::from(self.clone())))
            // configure client IP source to use
            .layer(self.app_state.args.ip_source.clone().into_extension());

        // add dev server proxy if enabled
        #[cfg(feature = "dev-proxy")]
        let router = dev_server_proxy::apply(&self.app_state, router);

        axum::serve(
            listener,
            router.into_make_service_with_connect_info::<SocketAddr>(),
        )
        .with_graceful_shutdown({
            let this = self.clone();
            async move {
                // wait for shutdown signal
                this.app_state.terminate.cancelled().await;
                // TODO: more cleanup if needed later (e.g. disconnect WS clients)
            }
        })
        .await?;

        Ok(())
    }
}
