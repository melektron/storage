/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
08.03.26, 17:10

Subsystem for handling web/websocket requests
*/

use std::{net::SocketAddr, sync::Arc};

use anyhow::{Context, Result};
use axum::{Router, routing::get};
use sea_orm::DatabaseConnection;
use tokio::net::TcpListener;

use crate::app::AppState;

pub struct WebServer {
    app_state: Arc<AppState>,
    db: DatabaseConnection,
}

mod routes {
    use axum_client_ip::ClientIp;
    use std::sync::Arc;

    use axum::extract::State;

    use crate::webserver::WebServer;

    pub async fn health_check(
        State(_state): State<Arc<WebServer>>,
        ClientIp(ip): ClientIp,
    ) -> String {
        let db_status = match _state.db.ping().await {
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

#[cfg(feature = "dev-proxy")]
mod dev_server_proxy {
    //! For development, it's possible to use the server
    //! as a reverse proxy to the UI devserver. This way
    //! a reverse proxy is not required for local development.

    use anyhow::{Context, Result};
    use axum::Router;
    use axum_reverse_proxy::ReverseProxy;

    use crate::app::AppState;

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

    pub async fn run(self: Arc<Self>) -> Result<()> {
        let listener = TcpListener::bind(format!(
            "{}:{}",
            self.app_state.args.listen_addr, self.app_state.args.listen_port
        ))
        .await
        .context("Binding TCP listener for HTTP")?;

        let router = Router::new()
            .route("/health", get(routes::health_check))
            //.route("/collab", get(routes::collab_ws))
            // configure client IP source to use
            .layer(self.app_state.args.ip_source.clone().into_extension())
            .with_state(self.clone());

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
