/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
04.08.26, 14:58
*/

//! Router layout used for the main page.
//! Styled in main.css

use dioxus::prelude::*;
use dioxus_free_icons::IconShape;
use crate::components::icon::{IconM, IconNavbar, ld};

use crate::{Route};

#[derive(Clone, PartialEq)]
enum Active {
    Never,
    Auto,
    Exact(&'static str),
    StartsWith(&'static str)
}

#[component]
fn NavLink<T: IconShape + Clone + PartialEq + 'static>(
    #[props(into)]
    to: NavigationTarget, 
    #[props(default = Active::Auto)]
    active: Active,
    icon: T,
    class: Option<String>,
    children: Element
) -> Element {

    let current_url = dioxus::router::router().full_route_string();
    let is_active = match active {
        Active::Never => false,
        Active::Auto => current_url == match &to {
            NavigationTarget::Internal(url) => url.clone(),
            NavigationTarget::External(route) => route.clone(),
        },
        Active::Exact(target) => current_url == target,
        // maybe this could be improved using child routs?
        Active::StartsWith(prefix) => current_url.starts_with(prefix)
    };

    let mut classes = String::from(if is_active { "nav-active" } else { "" });
    if let Some(outer) = class {
        classes.push_str(" ");
        classes.push_str(&outer);
    }
    let class = if classes.is_empty() { None } else { Some(classes) };

    rsx! {
        Link { 
            to: to,
            class: class,
            IconNavbar { icon: icon }
        }
    }
}

#[component]
pub fn MainLayout() -> Element {
    rsx! {
        div {
            id: "navbar",
            nav {
                id: "navbar-inner",
                NavLink { to: Route::Home {}, icon: ld::LdHome }
                NavLink { to: Route::Parts { id: 1 }, active: Active::StartsWith("/part"), icon: ld::LdComponent }
                NavLink { to: Route::Stock {}, active: Active::StartsWith("/stock"), icon: ld::LdBoxes }
                NavLink { to: Route::Home {}, icon: ld::LdCircleUserRound, class: "nav-end" }
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