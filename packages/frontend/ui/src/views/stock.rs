use crate::{Route, components::camera::Camera};
use dioxus::prelude::*;

#[component]
pub fn Stock() -> Element {
    let video_id = "qr-scanner-preview";

    let a = use_resource(move || async move {
        document::eval("return window.isSecureContext;").await
    });

    rsx! {
        "Items",
        match &*a.read_unchecked() {
            Some(Ok(x)) => format!("{x}"),
            Some(Err(e)) => format!("js error: {e}"),
            None => "Loading...".to_string()
        }
        Camera { }
    }
}
