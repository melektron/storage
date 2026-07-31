/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
27.07.26, 23:04

Global app stat object holding simple
values shared with most components
*/

use std::path::PathBuf;

use tokio_util::sync::CancellationToken;

use crate::Args;

pub struct AppState {
    pub terminate: CancellationToken,
    pub args: Args,
    pub data_path: PathBuf,
    pub db_path: PathBuf,
}
