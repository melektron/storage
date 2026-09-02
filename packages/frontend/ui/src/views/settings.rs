/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
29.08.26, 12:45
*/

use dioxus::prelude::*;
use crate::components::sidebar_layout::SidebarLayout;

#[component]
pub fn Settings() -> Element {
    rsx! {
        SidebarLayout { 
            title: "Settings",
            sidebar: rsx! {
                div {
                    width: "100%",
                    height: "100%",
                    background: "white",
                    "Settings Sidebar"
                }
            },
            main_view: rsx! {
                div {
                    width: "100%",
                    height: "100%",
                    background: "white",
                    "Settings main view"
                }
            }
        }
    }
}
