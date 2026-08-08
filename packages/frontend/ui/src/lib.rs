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
pub mod components;

mod layout;
use layout::MainLayout;

mod views;
use views::{Blog, Home};

#[derive(Debug, Clone, Routable, PartialEq)]
#[rustfmt::skip]
enum Route {
    #[layout(MainLayout)]
    #[route("/")]
    Home {},
    #[route("/blog/:id")]
    Blog { id: i32 },
}

#[component]
pub fn MainComponent() -> Element {
    rsx! {
        // Icons
        document::Link { rel: "icon", href: assets::FAVICON_ICO }
        document::Link { rel: "icon", type: "image/svg+xml", href: assets::FAVICON_SVG }
        document::Link { rel: "icon", type: "image/png", sizes: "32x32", href: assets::FAVICON_32 }
        document::Link { rel: "icon", type: "image/png", sizes: "192x192", href: assets::FAVICON_192 }
        document::Link { rel: "icon", type: "image/png", sizes: "512x512", href: assets::FAVICON_512 }
        document::Link { rel: "apple-touch-icon", id: "apple-touch-icon", href: assets::APPLE_TOUCH_ICON_LIGHT }
        document::Script { "
        // adapt apple touch icon depending on user color preference
        const iconLink = document.getElementById('apple-touch-icon');
            const darkModeMediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
            
            function updateAppleIcon(e) {{
                if (e.matches) {{
                    // default icon is dark
                    iconLink.href = '{assets::APPLE_TOUCH_ICON_DARK}';
                }} else {{
                    iconLink.href = '{assets::APPLE_TOUCH_ICON_LIGHT}';
                }}
            }}
            
            // initial check
            updateAppleIcon(darkModeMediaQuery);
            // listen for changes
            darkModeMediaQuery.addEventListener('change', updateAppleIcon);
        " }

        // PWA metadata
        // https://gist.github.com/fozzedout/5e77925381991a9570151550992baf14
        // Note: Updates to some of these don't apply in installed PWA, app must be reinstalled.
        document::Meta { name: "viewport", content: "width=device-width, initial-scale=1, maximum-scale=1.0, user-scalable=no, viewport-fit=cover, interactive-widget=resizes-content" }
        document::Meta { name: "apple-mobile-web-app-capable", content: "yes" }
        document::Meta { name: "apple-mobile-web-app-status-bar-style", content: "black-translucent" }
        document::Link { rel: "manifest", href: assets::PWA_WEBMANIFEST }

        // global styles
        document::Link { rel: "stylesheet", href: assets::BASE_CSS }
        document::Link { rel: "stylesheet", href: assets::MAIN_CSS }

        Router::<Route> {}
        //span {
        //    //style: "font-style: italic;",
        //    "Hello, World"
        //}
    }
}
