/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
02.08.26, 17:33
*/

//! Home view, giving quick access to often needed functions
//! (e.g. start scanning, ...). This is mostly relevant
//! for mobile use, as on desktop one would probably use the
//! command pallet / keyboard shortcuts instead.

use dioxus::prelude::*;

use dioxus_free_icons::{Icon, icons::ld_icons as ld};

use crate::components::sidebar_layout::SidebarLayout;

const HOME_CSS: Asset = asset!("./home.css");

#[component]
pub fn Home() -> Element {
    rsx! {
        document::Link { rel: "stylesheet", href: HOME_CSS }

        // TODO: nextup try to get recolorable logo working and then properly include fonts and get main CSS working
        //div { dangerous_inner_html: crate::assets::LOGO_COLORLESS_SVG }
        SidebarLayout {
            title: "Home",
            sidebar: rsx! {
                "Home Sidebar"
                for _ in 0..8 {
                    Echo {}
                }
            },
            main_view: rsx! {
                "Home main view"
                Echo {}
                Echo {}
                Echo {}
                Echo {}
                Echo {}
                Echo {}
            }
        }

    }
}

/// Echo component that demonstrates fullstack server functions.
#[component]
pub fn Echo() -> Element {
    let mut response = use_signal(|| String::new());

    rsx! {
        div { id: "echo",
            "test"
            h4 {
                Icon { icon: ld::LdEar }
                "ServerFn Echo"
            }
            input {
                placeholder: "Type here to echo...",
                oninput: move |event| async move {
                    let data = api::echo(event.value()).await.unwrap();
                    response.set(data);
                },
            }

            if !response().is_empty() {
                p {
                    "Server echoed: "
                    i { "{response}" }
                }
            }
        }
    }
}
