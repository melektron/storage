/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
08.03.26, 17:10

Subsystem for handling web/websocket requests
*/

use std::{collections::HashSet, net::SocketAddr, ops::Deref, path::PathBuf, sync::Arc};

use anyhow::{Context, Result};
use axum::{Extension, Router, routing::get};
use log::info;
use sea_orm::DatabaseConnection;
use tokio::net::TcpListener;
use dioxus::{fullstack::FullstackContext, server::{DioxusRouterExt, FullstackState, ServeConfig}};

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
pub struct WebServerContext {
    webserver: Arc<WebServer>
}
impl Deref for WebServerContext {
    type Target = Arc<WebServer>;

    fn deref(&self) -> &Self::Target {
        &self.webserver
    }
}
impl From<Arc<WebServer>> for WebServerContext {
    fn from(value: Arc<WebServer>) -> Self {
        WebServerContext { webserver: value.clone() }
    }
}

impl axum::extract::FromRef<FullstackContext> for WebServerContext {
    fn from_ref(state: &FullstackContext) -> Self {
        state.extension::<WebServerContext>().expect("Arc<WebServer> extension missing, cannot derive SubState.")
    }
}

#[cfg(feature = "dev-proxy")]
mod dev_server_proxy {
    //! For development, it's possible to use the server
    //! as a reverse proxy to the UI devserver. This way
    //! a reverse proxy is not required for local development.
    //! This is not needed for dioxus, as the dx dev environment
    //! acts as the proxy, forwarding any requests to server.

    use anyhow::{Context, Result};
    use axum::Router;
    use axum_reverse_proxy::ReverseProxy;

    use crate::globals::AppState;

    /// adds dev server proxy routes
    pub fn apply(app_state: &AppState, router: Router) -> Router {
        let proxy = ReverseProxy::new("/", &app_state.args.dev_server_url.as_str());

        router.merge(proxy)
    }
}

impl WebServer {
    pub fn new(app_state: &Arc<AppState>, db: &DatabaseConnection) -> Self {
        Self {
            app_state: app_state.clone(),
            db: db.clone(),
        }
    }

    pub async fn run(self: Arc<Self>, main_component: fn() -> dioxus::core::Element /*impl ComponentFunction<()> + Send + Sync*/,) -> Result<()> {

        // Get the address the server should run on. If the CLI is running, the CLI proxies fullstack into the main address
        // and we use the generated address the CLI gives us
        // TODO: integrate this with our own custom config
        let address = dioxus::cli_config::fullstack_address_or_localhost();

        let listener = TcpListener::bind(address)
        //let listener = TcpListener::bind(format!(
        //    "{}:{}",
        //    self.app_state.args.listen_addr, self.app_state.args.listen_port
        //))
        .await
        .context("Binding TCP listener for HTTP")?;
        
        let router = /*dioxus::server::router(main_component)*/Router::new()
            //.serve_dioxus_application(ServeConfig::new(), main_component)
            .register_server_functions()
            .serve_static_assets()
            .fallback(get(FullstackState::render_handler))
            .with_state(FullstackState::new(ServeConfig::new(), main_component))
            .layer(Extension(WebServerContext::from(self.clone())))
            // configure client IP source to use
            .layer(self.app_state.args.ip_source.clone().into_extension());
            //.with_state(self.clone());
            //.register_server_functions()
            //.serve_static_assets()
            //.fallback(get(FullstackState::render_handler))
            //.with_state(FullstackState::new(cfg, app))
            //.fallback_service(ServeFile::new(&public_path.join("index.html")).precompressed_br())
            //.with_state(FullstackState::headless());    // dummy state because we don't do SSR
            //.route("/collab", get(routes::collab_ws))

        // help-chat: https://discord.com/channels/899851952891002890/943190605067079712/1441581237880754400
        // internals: https://discord.com/channels/899851952891002890/928812591126569000/1432511520658555114
        // example: https://github.com/DioxusLabs/dioxus/blob/d0a5f89ae9df2dc74d7fbcbdb9324490a2c7048e/examples/07-fullstack/server_state.rs#L96


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
