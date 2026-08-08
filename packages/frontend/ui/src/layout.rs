/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
04.08.26, 14:58
*/

//! Router layout used for the main page.
//! Styled in main.css

use dioxus::prelude::*;
use crate::components::icon::{IconM, IconNavbar, ld};

use crate::{Route};


#[component]
pub fn MainLayout() -> Element {
    rsx! {
        div {
            id: "navbar",
            nav {
                id: "navbar-inner",
                Link { to: Route::Home {},                      IconNavbar { icon: ld::LdHome } }
                Link { to: Route::Blog { id: 1 },               IconNavbar { icon: ld::LdPen } }
                Link { to: Route::Home {}, class: "nav-end",    IconNavbar { icon: ld::LdCircleUserRound } }
            }
        }
        
        div {
            id: "root-view",
            main {
                id: "root-view-inner",
                Outlet::<Route> {}
            }
        }
    }
}