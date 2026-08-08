/*
ELEKTRON © 2026 - now
Written by melektron
www.elektron.work
04.08.26, 18:33
*/

//! Components for standard icon sizes

use dioxus::prelude::*;
use dioxus_free_icons::{Icon, IconShape};

pub use dioxus_free_icons::icons::ld_icons as ld;

/// Component Props for customized icon components
#[derive(PartialEq, Props, Clone)]
pub struct IconProps<T: IconShape + Clone + PartialEq + 'static> {
    /// The icon shape to use.
    pub icon: T,
    /// The color to use for filling the icon. Defaults to "currentColor".
    #[props(default = "currentColor".to_string())]
    pub fill: String,
    /// An class for the `<svg>` element.
    #[props(default = "".to_string())]
    pub class: String,
    /// An accessible, short-text description for the icon.
    pub title: Option<String>,
    /// The style of the `<svg>` element.
    pub style: Option<String>,
}

#[component]
pub fn IconS<T: IconShape + Clone + PartialEq + 'static>(props: IconProps<T>) -> Element {
    rsx! {
        Icon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            class: "icon icon-s {props.class}",
            title: props.title,
            style: props.style
        }
    }
}

#[component]
pub fn IconM<T: IconShape + Clone + PartialEq + 'static>(props: IconProps<T>) -> Element {
    rsx! {
        Icon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            class: "icon icon-m {props.class}",
            title: props.title,
            style: props.style
        }
    }
}

#[component]
pub fn IconXL<T: IconShape + Clone + PartialEq + 'static>(props: IconProps<T>) -> Element {
    rsx! {
        Icon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            class: "icon icon-xl {props.class}",
            title: props.title,
            style: props.style
        }
    }
}

#[component]
pub fn IconNavbar<T: IconShape + Clone + PartialEq + 'static>(props: IconProps<T>) -> Element {
    rsx! {
        Icon {
            icon: props.icon,
            width: None,
            height: None,
            fill: props.fill,
            // navbar icons are styled differently depending on
            // orientation and platform. They are selected by 
            // being descendants of the navbar, so they don't need
            // a class.
            class: "icon {props.class}",
            title: props.title,
            style: props.style
        }
    }
}