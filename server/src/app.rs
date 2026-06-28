/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
08.03.26, 15:24


*/

use std::{
    fs,
    path::{PathBuf},
    sync::Arc,
};

use anyhow::{Context, Result};
use el_std::terminal::{ReplIo, ReplLineHandler};
use log::info;
use sea_orm::{Database, DatabaseConnection};
use tokio_util::sync::CancellationToken;

use crate::{args::Args, repl::Repl, webserver::WebServer};

pub struct AppState {
    pub terminate: CancellationToken,
    pub args: Args,
    pub data_path: PathBuf,
    pub db_path: PathBuf,
}

pub struct App {
    app_state: Arc<AppState>,
    db: DatabaseConnection,
    web_server: Arc<WebServer>,
    repl: Arc<Repl>,
}

impl App {
    pub async fn new(
        args: Args, 
        repl_io: Option<ReplIo>
    ) -> Result<Self> {
        // make sure the data directory exists
        fs::create_dir_all(args.data.as_path()).context("failed to create data directory")?;
        let data_path = fs::canonicalize(args.data.as_path())?;
        // derive all other file paths from that core location
        let db_path = data_path.join("db.sqlite");

        let app_state = Arc::new(AppState {
            terminate: CancellationToken::new(),
            args,
            data_path,
            db_path,
        });

        // try to connect to database
        let db_url = format!("sqlite://{}?mode=rwc", app_state.db_path.display());
        let db = Database::connect(db_url).await?;

        let web_server = Arc::new(WebServer::new(&app_state, &db));
        let repl = Arc::new(Repl::new(&app_state, repl_io));

        Ok(Self {
            app_state,
            db,
            web_server,
            repl,
        })
    }

    pub async fn run(&self) -> Result<()> {
        info!(
            "Storing user data in {}",
            self.app_state.args.data.display()
        );

        // run all components until all complete or one fails.
        tokio::try_join!(
            self.repl.clone().run_anyhow(),
            self.web_server.clone().run(),
        )?;

        Ok(())
    }
}
