/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
01.08.26, 10:55
*/

//! Central export point for shared assets used across the application.
//! Assets that are only required locally (e.g. local CSS) may still be 
//! defined wherever they are needed.

use dioxus::prelude::*;

//pub const FONT_OPEN_SANS: Asset = asset!("/assets/fonts/Open_Sans/OpenSans-VariableFont_wdth,wght.ttf");
//pub const FONT_OPEN_SANS_ITALIC: Asset = asset!("/assets/fonts/Open_Sans/OpenSans-Italic-VariableFont_wdth,wght.ttf");
//pub const FONT_ROBOTO_MONO: Asset = asset!("/assets/fonts/Roboto_Mono/RobotoMono-VariableFont_wght.ttf");
//pub const FONT_ROBOTO_MONO_ITALIC: Asset = asset!("/assets/fonts/Roboto_Mono/RobotoMono-Italic-VariableFont_wght.ttf");


pub const LOGO_COLORLESS_SVG: &str = include_str!("../assets/logo/storage_logo_colorless_base.inkscape.svg");

pub const FAVICON_ICO: Asset = asset!("/assets/logo/icon/favicon.ico");
pub const FAVICON_SVG: Asset = asset!("/assets/logo/icon/favicon.svg");
pub const FAVICON_32: Asset = asset!("/assets/logo/icon/favicon-32.png");
pub const FAVICON_192: Asset = asset!("/assets/logo/icon/favicon-192.png");
pub const FAVICON_512: Asset = asset!("/assets/logo/icon/favicon-512.png");
pub const APPLE_TOUCH_ICON_DARK: Asset = asset!("/assets/logo/icon/apple-touch-icon-dark.png");
pub const APPLE_TOUCH_ICON_LIGHT: Asset = asset!("/assets/logo/icon/apple-touch-icon-light.png");

pub const PWA_WEBMANIFEST: Asset = asset!("/assets/site.webmanifest");

pub const MAIN_CSS: Asset = asset!("/assets/styling/main.css");