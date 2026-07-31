/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
08.03.26, 15:24
*/

//! storage system server entrypoint

use std::{fs, process::ExitCode, sync::Arc, time::Duration};

use anyhow::{Context, Result};
use clap::Parser;
use el_std::terminal::{ReplIo, ReplLineHandler, TerminalOpts, setup_terminal};
use log::info;
use migration::MigratorTrait;
use sea_orm::{ConnectOptions, Database, DatabaseConnection};
use tokio_util::sync::CancellationToken;

use storage_core::{AppState, Args, Repl, WebServer};
use web::MainComponent;

pub struct App {
    app_state: Arc<AppState>,
    db: DatabaseConnection,
    web_server: Arc<WebServer>,
    repl: Arc<Repl>,
}

impl App {
    pub async fn new(args: Args, repl_io: Option<ReplIo>) -> Result<Self> {
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
        let db = Database::connect(
            ConnectOptions::new(db_url)
                .connect_timeout(Duration::from_secs(10))
                .acquire_timeout(Duration::from_secs(10))
                //.sqlx_logging(false)
                // log SQL statements with trace level so they are available for debugging
                .sqlx_logging_level(log::LevelFilter::Trace)
                .to_owned(),
        )
        .await?;

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

        // migrate database to current version
        info!("Checking for database migrations...");
        let nr_pending_migrations = migration::Migrator::get_pending_migrations(&self.db)
            .await
            .context("failed to check pending migrations")?
            .len();
        if nr_pending_migrations > 0 {
            info!("{nr_pending_migrations} migration(s) pending");
            migration::Migrator::up(&self.db, None)
                .await
                .context("failed to apply migrations")?;
            info!("DB migration successful");
        } else {
            info!("DB up to date, no migrations required");
        }

        // run all components until all complete or one fails.
        tokio::try_join!(
            self.repl.clone().run_anyhow(),
            self.web_server.clone().run(MainComponent),
        )?;

        Ok(())
    }
}

pub async fn entry() -> ExitCode {
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
