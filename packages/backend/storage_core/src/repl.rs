/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
28.06.26, 15:02
*/

//! REPL for storage server

use std::sync::Arc;

use clap::Subcommand;
use el_std::terminal::{GetReplIo, ReplCommandHandler, ReplIo, shellout, shelloutln};
use tokio_util::sync::CancellationToken;

use crate::globals::AppState;


pub struct Repl {
    app_state: Arc<AppState>,

    repl_io: Option<ReplIo>,
}

impl Repl {
    pub fn new(app_state: &Arc<AppState>, repl_io: Option<ReplIo>) -> Self {
        Self {
            app_state: app_state.clone(),
            repl_io,
        }
    }
}

impl GetReplIo for Repl {
    fn get_repl_io(&self) -> Option<&ReplIo> {
        return self.repl_io.as_ref();
    }
}

#[derive(Subcommand, Debug)]
pub enum ReplCommands {

    #[command(
        visible_alias = "q",
        visible_alias = "quit",
        about = "Shuts down cleanly"
    )]
    Exit,

}

impl ReplCommandHandler for Repl {
    type ClapCommandsEnum = ReplCommands;

    fn get_cancellation_token(&self) -> Option<CancellationToken> {
        return Some(self.app_state.terminate.clone());
    }

    async fn handle_command(&self, cmd: Self::ClapCommandsEnum) -> anyhow::Result<()> {
        match cmd {
            ReplCommands::Exit => {
                self.app_state.terminate.cancel();
            }
        }

        Ok(())
    }
}