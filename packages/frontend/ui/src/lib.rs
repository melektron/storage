/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
02.08.26, 17:29
*/

//! Shared UI components for storage.
//! Since we don't directly target Desktop/Mobile atm,
//! This is more or less the complete UI for all of them.
//! How the UI is displayed depends only on the screen size and layout.

use dioxus::prelude::*;

pub mod assets;

mod navbar;
use navbar::Navbar;

mod views;
use views::{Blog, Home};

#[derive(Debug, Clone, Routable, PartialEq)]
#[rustfmt::skip]
enum Route {
    #[layout(WebNavbar)]
    #[route("/")]
    Home {},
    #[route("/blog/:id")]
    Blog { id: i32 },
}

/// A web-specific Router around the shared `Navbar` component
/// which allows us to use the web-specific `Route` enum.
#[component]
fn WebNavbar() -> Element {
    rsx! {
        Navbar {
            Link { to: Route::Home {}, "Home" }
            Link { to: Route::Blog { id: 1 }, "Blog" }
        }

        Outlet::<Route> {}
    }
}

#[component]
pub fn MainComponent() -> Element {
    rsx! {
        // Global app resources
        document::Link { rel: "icon", href: assets::FAVICON }
        document::Link { rel: "stylesheet", href: assets::MAIN_CSS }

        Router::<Route> {}
    }
}
